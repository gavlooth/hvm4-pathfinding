module swr_hub;

import swr;
import dijkstra;
import swr_dijkstra;
import pathfind;

// ============================================================
// Hub Labeling (2-Hop Covers) via Pruned Landmark Labeling (PLL)
//
// Each node has forward/backward label sets: (hub_id, distance).
// Query: merge-join d(s,t) = min_h (fwd(s,h) + bwd(t,h))
// ============================================================

struct HubLabel {
    uint hub;
    uint dist;
}

struct HubData {
    HubLabel[] fwd_labels;
    uint[] fwd_offset;
    HubLabel[] bwd_labels;
    uint[] bwd_offset;
    uint v;
}

fn void HubData.destroy(&self) {
    if (self.fwd_labels.ptr != null) free(self.fwd_labels.ptr);
    if (self.fwd_offset.ptr != null) free(self.fwd_offset.ptr);
    if (self.bwd_labels.ptr != null) free(self.bwd_labels.ptr);
    if (self.bwd_offset.ptr != null) free(self.bwd_offset.ptr);
}

// Dynamic label builder (resizable per-node, packed into CSR after)
struct LabelBuilder {
    HubLabel[][] labels;
    uint[] counts;
    uint[] caps;
    uint n;
}

fn LabelBuilder label_builder_new(uint n) {
    HubLabel[][] labels = mem::new_array(HubLabel[], n);
    uint[] counts = mem::new_array(uint, n);
    uint[] caps = mem::new_array(uint, n);
    for (uint i = 0; i < n; i++) {
        caps[i] = 4;
        labels[i] = mem::new_array(HubLabel, 4);
        counts[i] = 0;
    }
    return (LabelBuilder){ .labels = labels, .counts = counts, .caps = caps, .n = n };
}

fn void LabelBuilder.add(&self, uint node, uint hub, uint dist) {
    if (self.counts[node] >= self.caps[node]) {
        uint new_cap = self.caps[node] * 2;
        HubLabel[] new_arr = mem::new_array(HubLabel, new_cap);
        for (uint i = 0; i < self.counts[node]; i++) {
            new_arr[i] = self.labels[node][i];
        }
        free(self.labels[node].ptr);
        self.labels[node] = new_arr;
        self.caps[node] = new_cap;
    }
    self.labels[node][self.counts[node]] = (HubLabel){ .hub = hub, .dist = dist };
    self.counts[node]++;
}

fn void LabelBuilder.destroy(&self) {
    for (uint i = 0; i < self.n; i++) {
        if (self.labels[i].ptr != null) free(self.labels[i].ptr);
    }
    free(self.labels.ptr);
    free(self.counts.ptr);
    free(self.caps.ptr);
}

fn void sort_labels(HubLabel[] labels, uint count) {
    for (uint i = 1; i < count; i++) {
        HubLabel key = labels[i];
        int j = (int)i - 1;
        while (j >= 0 && labels[j].hub > key.hub) {
            labels[j + 1] = labels[j];
            j--;
        }
        labels[j + 1] = key;
    }
}

fn uint hub_query_builders(LabelBuilder* fwd, LabelBuilder* bwd, uint s, uint t) {
    uint best = pathfind::INF;
    for (uint i = 0; i < fwd.counts[s]; i++) {
        uint h = fwd.labels[s][i].hub;
        uint d1 = fwd.labels[s][i].dist;
        for (uint j = 0; j < bwd.counts[t]; j++) {
            if (bwd.labels[t][j].hub == h) {
                uint d = d1 + bwd.labels[t][j].dist;
                if (d < best) best = d;
                break;
            }
        }
    }
    return best;
}

// ============================================================
// Reusable Dijkstra workspace (allocate once, reuse V times)
// ============================================================

struct DijkWorkspace {
    uint[] dist;
    uint[] gen;        // generation stamp per node
    uint cur_gen;
    uint n;
}

fn DijkWorkspace dijk_ws_new(uint n) {
    DijkWorkspace ws;
    ws.dist = mem::new_array(uint, n);
    ws.gen = mem::new_array(uint, n);
    ws.cur_gen = 0;
    ws.n = n;
    for (uint i = 0; i < n; i++) ws.gen[i] = 0;
    return ws;
}

fn void DijkWorkspace.destroy(&self) {
    free(self.dist.ptr);
    free(self.gen.ptr);
}

fn void DijkWorkspace.reset(&self) {
    self.cur_gen++;
}

fn uint DijkWorkspace.get(&self, uint i) {
    if (self.gen[i] == self.cur_gen) return self.dist[i];
    return pathfind::INF;
}

fn void DijkWorkspace.set(&self, uint i, uint val) {
    self.dist[i] = val;
    self.gen[i] = self.cur_gen;
}

// Run Dijkstra from source using reusable workspace + heap.
// Returns dist via ws (caller reads ws.get(u)). Heap is reused across calls.
fn void dijkstra_reuse(pathfind::Graph* g, uint source, DijkWorkspace* ws,
                       swr_dijkstra::FibNode*[] nodes) {
    uint n = g.n;
    ws.reset();
    ws.set(source, 0);

    // Clear node pointers from previous run
    for (uint i = 0; i < n; i++) nodes[i] = null;

    swr_dijkstra::FibHeap heap = swr_dijkstra::fib_heap_new();
    nodes[source] = heap.insert(0, source);

    while (heap.n > 0) {
        swr_dijkstra::FibNode* u_node = heap.extract_min();
        if (u_node == null) break;
        uint u = u_node.value;
        uint du = u_node.key;
        nodes[u] = null;

        if (du >= pathfind::INF) break;

        for (usz i = 0; i < g.adj[u].len(); i++) {
            pathfind::Edge e = g.adj[u][i];
            uint nd = du + e.weight;
            if (nd < ws.get(e.to)) {
                ws.set(e.to, nd);
                if (nodes[e.to] == null) {
                    nodes[e.to] = heap.insert(nd, e.to);
                } else {
                    heap.decrease_key(nodes[e.to], nd);
                }
            }
        }
    }

    heap.destroy();
}

// ============================================================
// PLL: Pruned Landmark Labeling (reuses workspace)
// ============================================================

fn HubData preprocess(pathfind::Graph* g) {
    uint n = g.n;

    // Sort nodes by degree descending using heap sort (O(n log n))
    uint[] node_order = mem::new_array(uint, n);
    uint[] degrees = mem::new_array(uint, n);
    defer { free(node_order.ptr); free(degrees.ptr); }

    for (uint i = 0; i < n; i++) {
        node_order[i] = i;
        degrees[i] = (uint)g.adj[i].len();
    }

    // Heap sort descending
    // Build max-heap
    for (int i = (int)n / 2 - 1; i >= 0; i--) {
        uint idx = (uint)i;
        while (true) {
            uint largest = idx;
            uint l = 2 * idx + 1;
            uint r = 2 * idx + 2;
            if (l < n && degrees[node_order[l]] > degrees[node_order[largest]]) largest = l;
            if (r < n && degrees[node_order[r]] > degrees[node_order[largest]]) largest = r;
            if (largest == idx) break;
            uint tmp = node_order[idx]; node_order[idx] = node_order[largest]; node_order[largest] = tmp;
            idx = largest;
        }
    }
    for (uint i = n; i > 1; ) {
        i--;
        uint tmp = node_order[0]; node_order[0] = node_order[i]; node_order[i] = tmp;
        uint idx = 0;
        while (true) {
            uint largest = idx;
            uint l = 2 * idx + 1;
            uint r = 2 * idx + 2;
            if (l < i && degrees[node_order[l]] > degrees[node_order[largest]]) largest = l;
            if (r < i && degrees[node_order[r]] > degrees[node_order[largest]]) largest = r;
            if (largest == idx) break;
            uint t2 = node_order[idx]; node_order[idx] = node_order[largest]; node_order[largest] = t2;
            idx = largest;
        }
    }
    // Now node_order is sorted descending by degree (max-heap extract gives ascending,
    // but we extract into tail, so array ends up ascending = smallest first).
    // Reverse it for descending order.
    for (uint i = 0; i < n / 2; i++) {
        uint tmp = node_order[i];
        node_order[i] = node_order[n - 1 - i];
        node_order[n - 1 - i] = tmp;
    }

    // Build reverse graph
    pathfind::Graph rev = dijkstra::build_reverse(g);
    defer rev.destroy();

    LabelBuilder fwd_builder = label_builder_new(n);
    LabelBuilder bwd_builder = label_builder_new(n);
    defer { fwd_builder.destroy(); bwd_builder.destroy(); }

    // Reusable workspace — allocated ONCE
    DijkWorkspace ws_fwd = dijk_ws_new(n);
    DijkWorkspace ws_bwd = dijk_ws_new(n);
    defer { ws_fwd.destroy(); ws_bwd.destroy(); }
    swr_dijkstra::FibNode*[] nodes_buf = mem::new_array(swr_dijkstra::FibNode*, n);
    defer free(nodes_buf.ptr);

    for (uint idx = 0; idx < n; idx++) {
        uint v = node_order[idx];

        // Forward Dijkstra from v (reuse workspace)
        dijkstra_reuse(g, v, &ws_fwd, nodes_buf);
        for (uint u = 0; u < n; u++) {
            uint d = ws_fwd.get(u);
            if (d >= pathfind::INF) continue;
            uint existing = hub_query_builders(&fwd_builder, &bwd_builder, v, u);
            if (existing <= d) continue;
            bwd_builder.add(u, v, d);
        }

        // Backward Dijkstra from v on reversed graph (reuse workspace)
        dijkstra_reuse(&rev, v, &ws_bwd, nodes_buf);
        for (uint u = 0; u < n; u++) {
            uint d = ws_bwd.get(u);
            if (d >= pathfind::INF) continue;
            uint existing = hub_query_builders(&fwd_builder, &bwd_builder, u, v);
            if (existing <= d) continue;
            fwd_builder.add(u, v, d);
        }
    }

    // Sort all labels by hub ID
    for (uint i = 0; i < n; i++) {
        sort_labels(fwd_builder.labels[i], fwd_builder.counts[i]);
        sort_labels(bwd_builder.labels[i], bwd_builder.counts[i]);
    }

    // Pack into CSR
    uint[] fwd_offset = mem::new_array(uint, (usz)(n + 1));
    uint[] bwd_offset = mem::new_array(uint, (usz)(n + 1));
    uint total_fwd = 0;
    uint total_bwd = 0;
    fwd_offset[0] = 0;
    bwd_offset[0] = 0;
    for (uint i = 0; i < n; i++) {
        total_fwd += fwd_builder.counts[i];
        fwd_offset[i + 1] = total_fwd;
        total_bwd += bwd_builder.counts[i];
        bwd_offset[i + 1] = total_bwd;
    }

    HubLabel[] fwd_labels = mem::new_array(HubLabel, total_fwd > 0 ? (usz)total_fwd : (usz)1);
    HubLabel[] bwd_labels = mem::new_array(HubLabel, total_bwd > 0 ? (usz)total_bwd : (usz)1);

    for (uint i = 0; i < n; i++) {
        for (uint j = 0; j < fwd_builder.counts[i]; j++) {
            fwd_labels[fwd_offset[i] + j] = fwd_builder.labels[i][j];
        }
        for (uint j = 0; j < bwd_builder.counts[i]; j++) {
            bwd_labels[bwd_offset[i] + j] = bwd_builder.labels[i][j];
        }
    }

    return (HubData){
        .fwd_labels = fwd_labels,
        .fwd_offset = fwd_offset,
        .bwd_labels = bwd_labels,
        .bwd_offset = bwd_offset,
        .v = n
    };
}

// ============================================================
// Query: merge-join on sorted hub IDs
// ============================================================

fn uint hub_query(HubData* hub, uint source, uint target) {
    if (source >= hub.v || target >= hub.v) return pathfind::INF;
    if (source == target) return 0;

    uint fi = hub.fwd_offset[source];
    uint fe = hub.fwd_offset[source + 1];
    uint bi = hub.bwd_offset[target];
    uint be = hub.bwd_offset[target + 1];

    uint best = pathfind::INF;

    while (fi < fe && bi < be) {
        uint fh = hub.fwd_labels[fi].hub;
        uint bh = hub.bwd_labels[bi].hub;
        if (fh == bh) {
            uint d = hub.fwd_labels[fi].dist + hub.bwd_labels[bi].dist;
            if (d < best) best = d;
            fi++;
            bi++;
        } else if (fh < bh) {
            fi++;
        } else {
            bi++;
        }
    }

    return best;
}

// Full SWR routing with Hub Labeling
fn swr::RouteResult? swr_route_hub(swr::SwrState* state, swr::WeatherGrid* weather) {
    swr::update_all_weights(state, weather);

    HubData hub = preprocess(&state.prm.graph);
    defer hub.destroy();

    uint cost = hub_query(&hub, state.origin, state.destination);
    if (cost >= pathfind::INF) return swr::SWR_ERROR~;

    dijkstra::DijkResult dr = dijkstra::sssp(&state.prm.graph, state.origin)!;
    uint[] path = dijkstra::extract_path(dr.pred, state.origin, state.destination, dr.n)!;

    return (swr::RouteResult){
        .path = path,
        .path_len = path.len,
        .total_cost = cost,
        .distances = dr.dist,
        .dist_count = dr.n
    };
}
