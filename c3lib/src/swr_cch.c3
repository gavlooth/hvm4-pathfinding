module swr_cch;

import swr;
import dijkstra;
import swr_dijkstra;
import pathfind;

// ============================================================
// CCH: Customizable Contraction Hierarchies
//
// Batch-parallel contraction: each round contracts an independent
// set of non-adjacent low-priority nodes simultaneously.
// No sequential node-at-a-time dependency chain.
// ============================================================

const uint INVALID = 0xFFFFFFFF;

struct UpEdge {
    uint from;
    uint to;
    uint weight;
    uint via;
    uint e1;
    uint e2;
}

struct CchData {
    uint[] rank;
    uint[] order;
    UpEdge[] fwd_edges;
    uint[] fwd_offset;
    uint fwd_count;
    UpEdge[] bwd_edges;
    uint[] bwd_offset;
    uint bwd_count;
    uint[] customize_order;
    uint customize_count;
    uint v;
    bool initialized;
}

fn void CchData.destroy(&self) {
    if (self.rank.ptr != null) free(self.rank.ptr);
    if (self.order.ptr != null) free(self.order.ptr);
    if (self.fwd_edges.ptr != null) free(self.fwd_edges.ptr);
    if (self.fwd_offset.ptr != null) free(self.fwd_offset.ptr);
    if (self.bwd_edges.ptr != null) free(self.bwd_edges.ptr);
    if (self.bwd_offset.ptr != null) free(self.bwd_offset.ptr);
    if (self.customize_order.ptr != null) free(self.customize_order.ptr);
}

// ============================================================
// Contraction workspace
// ============================================================

struct ContractionState {
    bool[] contracted;
    uint[] level;
    uint n;
    uint[] ws_dist;
    uint[] ws_gen;
    uint ws_cur_gen;
    pathfind::Graph rev;
}

fn void ContractionState.init(&self, uint n, pathfind::Graph* g) {
    self.contracted = mem::new_array(bool, n);
    self.level = mem::new_array(uint, n);
    self.n = n;
    self.ws_dist = mem::new_array(uint, n);
    self.ws_gen = mem::new_array(uint, n);
    self.ws_cur_gen = 0;
    for (uint i = 0; i < n; i++) {
        self.contracted[i] = false;
        self.level[i] = 0;
        self.ws_gen[i] = 0;
    }
    self.rev.init(n);
    for (uint u = 0; u < n; u++) {
        for (usz i = 0; i < g.adj[u].len(); i++) {
            pathfind::Edge e = g.adj[u][i];
            self.rev.add_edge(e.to, u, e.weight);
        }
    }
}

fn void ContractionState.destroy(&self) {
    free(self.contracted.ptr);
    free(self.level.ptr);
    free(self.ws_dist.ptr);
    free(self.ws_gen.ptr);
    self.rev.destroy();
}

fn void ContractionState.ws_reset(&self) { self.ws_cur_gen++; }

fn uint ContractionState.ws_get(&self, uint i) {
    if (self.ws_gen[i] == self.ws_cur_gen) return self.ws_dist[i];
    return pathfind::INF;
}

fn void ContractionState.ws_set(&self, uint i, uint val) {
    self.ws_dist[i] = val;
    self.ws_gen[i] = self.ws_cur_gen;
}

// ============================================================
// Bounded witness search (reuses workspace)
// ============================================================

fn bool witness_search(pathfind::Graph* g, ContractionState* cs, uint v, uint w, uint limit, uint skip) {
    cs.ws_reset();
    cs.ws_set(v, 0);

    uint max_settle = 20;
    uint[256] queue;
    uint qhead = 0;
    uint qtail = 0;
    queue[qtail++ & 255] = v;

    for (uint settled = 0; settled < max_settle && qhead < qtail; settled++) {
        uint best_qi = qhead;
        uint best_d = cs.ws_get(queue[qhead & 255]);
        for (uint qi = qhead + 1; qi < qtail; qi++) {
            uint d = cs.ws_get(queue[qi & 255]);
            if (d < best_d) { best_d = d; best_qi = qi; }
        }

        if (best_qi != qhead) {
            uint tmp = queue[qhead & 255];
            queue[qhead & 255] = queue[best_qi & 255];
            queue[best_qi & 255] = tmp;
        }

        uint u = queue[qhead++ & 255];
        uint du = cs.ws_get(u);
        if (du > limit) continue;
        if (u == w) return true;
        if (cs.contracted[u] && u != v) continue;
        if (u == skip) continue;

        for (usz i = 0; i < g.adj[u].len(); i++) {
            pathfind::Edge e = g.adj[u][i];
            if (cs.contracted[e.to] || e.to == skip) continue;
            uint nd = du + e.weight;
            if (nd <= limit && nd < cs.ws_get(e.to)) {
                cs.ws_set(e.to, nd);
                if (qtail - qhead < 256) {
                    queue[qtail++ & 255] = e.to;
                }
            }
        }
    }
    return false;
}

// ============================================================
// Priority: edge-difference heuristic (uses reverse adjacency)
// ============================================================

fn int compute_priority(pathfind::Graph* g, ContractionState* cs, uint u) {
    uint shortcuts = 0;
    uint edges_removed = 0;

    for (usz i = 0; i < g.adj[u].len(); i++) {
        if (!cs.contracted[g.adj[u][i].to]) edges_removed++;
    }
    for (usz i = 0; i < cs.rev.adj[u].len(); i++) {
        if (!cs.contracted[cs.rev.adj[u][i].to]) edges_removed++;
    }

    for (usz i = 0; i < cs.rev.adj[u].len(); i++) {
        uint v = cs.rev.adj[u][i].to;
        if (cs.contracted[v]) continue;
        uint w_vu = cs.rev.adj[u][i].weight;

        for (usz j = 0; j < g.adj[u].len(); j++) {
            uint w = g.adj[u][j].to;
            if (cs.contracted[w] || w == v) continue;
            uint shortcut_dist = w_vu + g.adj[u][j].weight;

            if (!witness_search(g, cs, v, w, shortcut_dist, u)) {
                shortcuts++;
            }
        }
    }

    return (int)shortcuts - (int)edges_removed + (int)(2 * cs.level[u]);
}

// ============================================================
// Batch-parallel contraction: independent set per round
// ============================================================

fn CchData build_topology(pathfind::Graph* g) {
    uint n = g.n;

    ContractionState cs;
    cs.init(n, g);
    defer cs.destroy();

    // Compute initial priorities
    int[] priority = mem::new_array(int, n);
    defer free(priority.ptr);
    for (uint i = 0; i < n; i++) {
        priority[i] = compute_priority(g, &cs, i);
    }

    uint[] rank = mem::new_array(uint, n);
    uint[] order = mem::new_array(uint, n);
    uint rank_pos = 0;

    // Batch buffer for independent set
    uint[] batch = mem::new_array(uint, n);
    bool[] in_batch = mem::new_array(bool, n);
    defer { free(batch.ptr); free(in_batch.ptr); }

    while (rank_pos < n) {
        // Find independent set: nodes whose priority is local minimum
        // among their uncontracted neighbors (no two adjacent nodes in batch)
        uint batch_size = 0;
        for (uint i = 0; i < n; i++) in_batch[i] = false;

        for (uint i = 0; i < n; i++) {
            if (cs.contracted[i]) continue;

            // Check if i has lower priority than all uncontracted neighbors
            bool is_local_min = true;
            for (usz j = 0; j < g.adj[i].len(); j++) {
                uint nb = g.adj[i][j].to;
                if (!cs.contracted[nb] && nb != i) {
                    if (priority[nb] < priority[i] || (priority[nb] == priority[i] && nb < i)) {
                        is_local_min = false;
                        break;
                    }
                }
            }
            if (!is_local_min) continue;
            for (usz j = 0; j < cs.rev.adj[i].len(); j++) {
                uint nb = cs.rev.adj[i][j].to;
                if (!cs.contracted[nb] && nb != i) {
                    if (priority[nb] < priority[i] || (priority[nb] == priority[i] && nb < i)) {
                        is_local_min = false;
                        break;
                    }
                }
            }
            if (!is_local_min) continue;

            // Verify no neighbor already in batch (independence)
            bool neighbor_in_batch = false;
            for (usz j = 0; j < g.adj[i].len(); j++) {
                if (in_batch[g.adj[i][j].to]) { neighbor_in_batch = true; break; }
            }
            if (!neighbor_in_batch) {
                for (usz j = 0; j < cs.rev.adj[i].len(); j++) {
                    if (in_batch[cs.rev.adj[i][j].to]) { neighbor_in_batch = true; break; }
                }
            }
            if (neighbor_in_batch) continue;

            batch[batch_size++] = i;
            in_batch[i] = true;
        }

        // If no local minima found (shouldn't happen), force-contract lowest priority node
        if (batch_size == 0) {
            uint best = INVALID;
            int best_p = 0x7FFFFFFF;
            for (uint i = 0; i < n; i++) {
                if (!cs.contracted[i] && priority[i] < best_p) {
                    best_p = priority[i];
                    best = i;
                }
            }
            if (best == INVALID) break;
            batch[0] = best;
            batch_size = 1;
        }

        // Contract entire batch simultaneously
        for (uint bi = 0; bi < batch_size; bi++) {
            uint u = batch[bi];
            rank[u] = rank_pos;
            order[rank_pos] = u;
            rank_pos++;

            // Add shortcuts for (v->u, u->w) pairs
            for (usz i = 0; i < cs.rev.adj[u].len(); i++) {
                uint v = cs.rev.adj[u][i].to;
                if (cs.contracted[v] || in_batch[v]) continue;
                uint w_vu = cs.rev.adj[u][i].weight;

                for (usz j = 0; j < g.adj[u].len(); j++) {
                    uint w = g.adj[u][j].to;
                    if (cs.contracted[w] || in_batch[w] || w == v) continue;
                    uint shortcut_dist = w_vu + g.adj[u][j].weight;

                    if (!witness_search(g, &cs, v, w, shortcut_dist, u)) {
                        g.add_edge(v, w, shortcut_dist);
                        cs.rev.add_edge(w, v, shortcut_dist);
                    }
                }
            }
        }

        // Mark batch as contracted + update levels
        for (uint bi = 0; bi < batch_size; bi++) {
            uint u = batch[bi];
            cs.contracted[u] = true;

            for (usz i = 0; i < g.adj[u].len(); i++) {
                uint w = g.adj[u][i].to;
                if (!cs.contracted[w]) {
                    uint new_level = cs.level[u] + 1;
                    if (new_level > cs.level[w]) cs.level[w] = new_level;
                }
            }
        }

        // Update priorities only for neighbors of contracted batch
        for (uint bi = 0; bi < batch_size; bi++) {
            uint u = batch[bi];
            for (usz i = 0; i < g.adj[u].len(); i++) {
                uint w = g.adj[u][i].to;
                if (!cs.contracted[w]) {
                    priority[w] = compute_priority(g, &cs, w);
                }
            }
            for (usz i = 0; i < cs.rev.adj[u].len(); i++) {
                uint w = cs.rev.adj[u][i].to;
                if (!cs.contracted[w]) {
                    priority[w] = compute_priority(g, &cs, w);
                }
            }
        }
    }

    // Phase 2: Build upward graphs (unchanged)
    uint[] fwd_count_per = mem::new_array(uint, n);
    uint[] bwd_count_per = mem::new_array(uint, n);
    defer { free(fwd_count_per.ptr); free(bwd_count_per.ptr); }
    for (uint i = 0; i < n; i++) { fwd_count_per[i] = 0; bwd_count_per[i] = 0; }

    for (uint u = 0; u < n; u++) {
        for (usz i = 0; i < g.adj[u].len(); i++) {
            uint v = g.adj[u][i].to;
            if (rank[v] > rank[u]) fwd_count_per[u]++;
        }
    }

    for (uint v = 0; v < n; v++) {
        for (usz i = 0; i < g.adj[v].len(); i++) {
            uint u = g.adj[v][i].to;
            if (rank[v] > rank[u]) bwd_count_per[u]++;
        }
    }

    uint[] fwd_offset = mem::new_array(uint, (usz)(n + 1));
    uint[] bwd_offset = mem::new_array(uint, (usz)(n + 1));
    fwd_offset[0] = 0; bwd_offset[0] = 0;
    uint total_fwd = 0;
    uint total_bwd = 0;
    for (uint i = 0; i < n; i++) {
        total_fwd += fwd_count_per[i]; fwd_offset[i + 1] = total_fwd;
        total_bwd += bwd_count_per[i]; bwd_offset[i + 1] = total_bwd;
    }

    UpEdge[] fwd_edges = mem::new_array(UpEdge, total_fwd > 0 ? (usz)total_fwd : (usz)1);
    UpEdge[] bwd_edges = mem::new_array(UpEdge, total_bwd > 0 ? (usz)total_bwd : (usz)1);

    for (uint i = 0; i < n; i++) { fwd_count_per[i] = 0; bwd_count_per[i] = 0; }

    for (uint u = 0; u < n; u++) {
        for (usz i = 0; i < g.adj[u].len(); i++) {
            uint v = g.adj[u][i].to;
            if (rank[v] > rank[u]) {
                uint idx = fwd_offset[u] + fwd_count_per[u];
                fwd_edges[idx] = (UpEdge){
                    .from = u, .to = v, .weight = g.adj[u][i].weight,
                    .via = INVALID, .e1 = INVALID, .e2 = INVALID
                };
                fwd_count_per[u]++;
            }
        }
    }

    for (uint v = 0; v < n; v++) {
        for (usz i = 0; i < g.adj[v].len(); i++) {
            uint u = g.adj[v][i].to;
            if (rank[v] > rank[u]) {
                uint idx = bwd_offset[u] + bwd_count_per[u];
                bwd_edges[idx] = (UpEdge){
                    .from = u, .to = v, .weight = g.adj[v][i].weight,
                    .via = INVALID, .e1 = INVALID, .e2 = INVALID
                };
                bwd_count_per[u]++;
            }
        }
    }

    return (CchData){
        .rank = rank,
        .order = order,
        .fwd_edges = fwd_edges,
        .fwd_offset = fwd_offset,
        .fwd_count = total_fwd,
        .bwd_edges = bwd_edges,
        .bwd_offset = bwd_offset,
        .bwd_count = total_bwd,
        .customize_order = mem::new_array(uint, 0),
        .customize_count = 0,
        .v = n,
        .initialized = true
    };
}

// ============================================================
// Customization
// ============================================================

fn void customize(CchData* cch, pathfind::Graph* g) {
    for (uint i = 0; i < cch.fwd_count; i++) {
        UpEdge* e = &cch.fwd_edges[i];
        for (usz j = 0; j < g.adj[e.from].len(); j++) {
            if (g.adj[e.from][j].to == e.to) {
                e.weight = g.adj[e.from][j].weight;
                break;
            }
        }
    }
    for (uint i = 0; i < cch.bwd_count; i++) {
        UpEdge* e = &cch.bwd_edges[i];
        for (usz j = 0; j < g.adj[e.to].len(); j++) {
            if (g.adj[e.to][j].to == e.from) {
                e.weight = g.adj[e.to][j].weight;
                break;
            }
        }
    }
}

// ============================================================
// Query: bidirectional Dijkstra on upward graph
// ============================================================

fn uint cch_query(CchData* cch, uint source, uint target) {
    uint n = cch.v;
    if (source >= n || target >= n) return pathfind::INF;
    if (source == target) return 0;

    uint[] dist_f = mem::new_array(uint, n);
    uint[] dist_b = mem::new_array(uint, n);
    defer { free(dist_f.ptr); free(dist_b.ptr); }

    for (uint i = 0; i < n; i++) {
        dist_f[i] = pathfind::INF;
        dist_b[i] = pathfind::INF;
    }
    dist_f[source] = 0;
    dist_b[target] = 0;

    swr_dijkstra::FibNode*[] nodes_f = mem::new_array(swr_dijkstra::FibNode*, n);
    swr_dijkstra::FibNode*[] nodes_b = mem::new_array(swr_dijkstra::FibNode*, n);
    defer { free(nodes_f.ptr); free(nodes_b.ptr); }
    for (uint i = 0; i < n; i++) { nodes_f[i] = null; nodes_b[i] = null; }

    swr_dijkstra::FibHeap heap_f = swr_dijkstra::fib_heap_new();
    swr_dijkstra::FibHeap heap_b = swr_dijkstra::fib_heap_new();
    defer { heap_f.destroy(); heap_b.destroy(); }

    nodes_f[source] = heap_f.insert(0, source);
    nodes_b[target] = heap_b.insert(0, target);

    uint mu = pathfind::INF;

    while (heap_f.n > 0 || heap_b.n > 0) {
        if (heap_f.n > 0) {
            swr_dijkstra::FibNode* uf = heap_f.extract_min();
            if (uf != null && uf.key < mu) {
                uint u = uf.value;
                uint du = uf.key;
                nodes_f[u] = null;

                if (dist_b[u] < pathfind::INF && du + dist_b[u] < mu) mu = du + dist_b[u];

                for (uint i = cch.fwd_offset[u]; i < cch.fwd_offset[u + 1]; i++) {
                    UpEdge* e = &cch.fwd_edges[i];
                    uint nd = du + e.weight;
                    if (nd < dist_f[e.to]) {
                        dist_f[e.to] = nd;
                        if (nodes_f[e.to] == null) {
                            nodes_f[e.to] = heap_f.insert(nd, e.to);
                        } else {
                            heap_f.decrease_key(nodes_f[e.to], nd);
                        }
                        if (dist_b[e.to] < pathfind::INF && nd + dist_b[e.to] < mu) mu = nd + dist_b[e.to];
                    }
                }
            }
        }

        if (heap_b.n > 0) {
            swr_dijkstra::FibNode* ub = heap_b.extract_min();
            if (ub != null && ub.key < mu) {
                uint u = ub.value;
                uint du = ub.key;
                nodes_b[u] = null;

                if (dist_f[u] < pathfind::INF && dist_f[u] + du < mu) mu = dist_f[u] + du;

                for (uint i = cch.bwd_offset[u]; i < cch.bwd_offset[u + 1]; i++) {
                    UpEdge* e = &cch.bwd_edges[i];
                    uint nd = du + e.weight;
                    if (nd < dist_b[e.to]) {
                        dist_b[e.to] = nd;
                        if (nodes_b[e.to] == null) {
                            nodes_b[e.to] = heap_b.insert(nd, e.to);
                        } else {
                            heap_b.decrease_key(nodes_b[e.to], nd);
                        }
                        if (dist_f[e.to] < pathfind::INF && dist_f[e.to] + nd < mu) mu = dist_f[e.to] + nd;
                    }
                }
            }
        }

        uint min_f = (heap_f.n > 0 && heap_f.min != null) ? heap_f.min.key : pathfind::INF;
        uint min_b = (heap_b.n > 0 && heap_b.min != null) ? heap_b.min.key : pathfind::INF;
        if (min_f >= mu && min_b >= mu) break;
    }

    return mu;
}

fn swr::RouteResult? swr_route_cch(swr::SwrState* state, swr::WeatherGrid* weather) {
    swr::update_all_weights(state, weather);

    pathfind::Graph g_copy;
    g_copy.init(state.prm.n);
    for (uint u = 0; u < state.prm.n; u++) {
        for (usz i = 0; i < state.prm.graph.adj[u].len(); i++) {
            pathfind::Edge e = state.prm.graph.adj[u][i];
            g_copy.add_edge(u, e.to, e.weight);
        }
    }

    CchData cch = build_topology(&g_copy);
    g_copy.destroy();
    defer cch.destroy();

    customize(&cch, &state.prm.graph);

    uint cost = cch_query(&cch, state.origin, state.destination);
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
