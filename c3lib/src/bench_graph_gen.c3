module bench_graph_gen;
import bench_common;
import pathfind;

// ---------------------------------------------------------------------------
// Shared: edges_to_csr
// ---------------------------------------------------------------------------

fn CsrGraph edges_to_csr(RawEdge* edges, uint ne, uint n, bool weighted) {
    uint* rp = mem::new_array(uint, (usz)n + 2).ptr;
    for (uint i = 0; i < n + 2; i++) rp[i] = 0;
    for (uint i = 0; i < ne; i++) rp[edges[i].u + 1]++;
    for (uint i = 1; i <= n; i++) rp[i] += rp[i - 1];

    uint* ci = mem::new_array(uint, ne).ptr;
    uint* wt = weighted ? mem::new_array(uint, ne).ptr : null;
    uint* pos = mem::new_array(uint, (usz)n + 1).ptr;
    for (uint i = 0; i <= n; i++) pos[i] = rp[i];

    for (uint i = 0; i < ne; i++) {
        uint u = edges[i].u;
        uint p = pos[u];
        pos[u] = p + 1;
        ci[p] = edges[i].v;
        if (wt) wt[p] = edges[i].w;
    }

    free(pos);
    CsrGraph result;
    result.row_ptr = rp;
    result.col_idx = ci;
    result.weight = wt;
    result.v = n;
    result.e = ne;
    return result;
}

// ---------------------------------------------------------------------------
// gen_random: sparse connected graph with chain + random edges
// Used by: bf, delta_step
// ---------------------------------------------------------------------------

fn CsrGraph gen_random(uint n, uint epn, uint seed) {
    Lcg rng = { .s = seed };
    uint target_e = n * epn;
    uint max_e = target_e + n;
    RawEdge* edges = mem::new_array(RawEdge, (usz)max_e).ptr;
    uint ne = 0;

    // Chain for connectivity
    for (uint i = 0; i + 1 < n; i++) {
        uint w = rng.next() % 10 + 1;
        edges[ne].u = i;
        edges[ne].v = i + 1;
        edges[ne].w = w;
        ne++;
    }

    // Random extra edges
    uint attempts = n * (epn - 1) * 2;
    for (uint a = 0; a < attempts && ne < target_e; a++) {
        uint u = rng.next() % n;
        uint v = rng.next() % n;
        if (u == v) continue;
        uint w = rng.next() % 20 + 1;
        edges[ne].u = u;
        edges[ne].v = v;
        edges[ne].w = w;
        ne++;
    }

    CsrGraph g = edges_to_csr(edges, ne, n, true);
    free(edges);
    return g;
}

// ---------------------------------------------------------------------------
// gen_dag: forward-only DAG (all edges u < v)
// Used by: ch, dag_dp
// ---------------------------------------------------------------------------

fn CsrGraph gen_dag(uint n, uint epn, uint seed) {
    Lcg rng = { .s = seed };
    uint target_e = n * epn;
    uint max_e = target_e + n;
    RawEdge* edges = mem::new_array(RawEdge, (usz)max_e).ptr;
    uint ne = 0;

    // Chain
    for (uint i = 0; i + 1 < n; i++) {
        uint w = rng.next() % 10 + 1;
        edges[ne].u = i;
        edges[ne].v = i + 1;
        edges[ne].w = w;
        ne++;
    }

    // Random forward edges
    uint attempts = n * (epn - 1) * 3;
    for (uint a = 0; a < attempts && ne < target_e; a++) {
        uint u = rng.next() % n;
        uint v = rng.next() % n;
        if (u >= v) continue;
        uint w = rng.next() % 20 + 1;
        edges[ne].u = u;
        edges[ne].v = v;
        edges[ne].w = w;
        ne++;
    }

    CsrGraph g = edges_to_csr(edges, ne, n, true);
    free(edges);
    return g;
}

// ---------------------------------------------------------------------------
// gen_dense: dense directed weighted graph (~50% edge probability)
// Used by: apsp
// Returns: CsrGraph + dense V*V adjacency matrix (INF for no-edge, 0 on diag)
// ---------------------------------------------------------------------------

const uint APSP_INF = 999_999;

struct DenseResult {
    CsrGraph csr;
    uint* adj;  // n*n flat matrix
}

fn DenseResult gen_dense(uint n, uint seed) {
    Lcg rng = { .s = seed };

    uint* adj = mem::new_array(uint, (usz)n * n).ptr;
    for (uint i = 0; i < n; i++) {
        for (uint j = 0; j < n; j++) {
            adj[i * n + j] = (i == j) ? 0 : APSP_INF;
        }
    }

    uint ne = 0;
    for (uint i = 0; i < n; i++) {
        for (uint j = 0; j < n; j++) {
            if (i == j) continue;
            if (rng.next() % 2 == 0) {
                uint w = rng.next() % 20 + 1;
                adj[i * n + j] = w;
                ne++;
            }
        }
    }

    // Build CSR from adj
    uint* rp = mem::new_array(uint, (usz)n + 2).ptr;
    for (uint i = 0; i < n + 2; i++) rp[i] = 0;
    for (uint i = 0; i < n; i++) {
        for (uint j = 0; j < n; j++) {
            if (i != j && adj[i * n + j] < APSP_INF) rp[i + 1]++;
        }
    }
    for (uint i = 1; i <= n; i++) rp[i] += rp[i - 1];

    uint* ci = mem::new_array(uint, ne).ptr;
    uint* wt = mem::new_array(uint, ne).ptr;
    uint* pos = mem::new_array(uint, (usz)n + 1).ptr;
    for (uint i = 0; i <= n; i++) pos[i] = rp[i];

    for (uint i = 0; i < n; i++) {
        for (uint j = 0; j < n; j++) {
            if (i != j && adj[i * n + j] < APSP_INF) {
                uint p = pos[i];
                pos[i] = p + 1;
                ci[p] = j;
                wt[p] = adj[i * n + j];
            }
        }
    }

    free(pos);
    CsrGraph csr;
    csr.row_ptr = rp;
    csr.col_idx = ci;
    csr.weight = wt;
    csr.v = n;
    csr.e = ne;
    DenseResult result;
    result.csr = csr;
    result.adj = adj;
    return result;
}

// ---------------------------------------------------------------------------
// gen_dense_bool: dense directed boolean graph (~50% edge probability)
// Used by: closure
// Returns: CsrGraph + boolean adj matrix (1 on diag, 1 = edge)
// ---------------------------------------------------------------------------

struct BoolDenseResult {
    CsrGraph csr;
    uint* adj;  // n*n flat, 0 or 1
}

fn BoolDenseResult gen_dense_bool(uint n, uint seed) {
    Lcg rng = { .s = seed };

    uint* adj = mem::new_array(uint, (usz)n * n).ptr;
    for (uint i = 0; i < n; i++) {
        for (uint j = 0; j < n; j++) {
            adj[i * n + j] = (i == j) ? 1 : 0;
        }
    }

    uint ne = 0;
    for (uint i = 0; i < n; i++) {
        for (uint j = 0; j < n; j++) {
            if (i == j) continue;
            if (rng.next() % 2 == 0) {
                adj[i * n + j] = 1;
                ne++;
            }
        }
    }

    // Build CSR
    uint* rp = mem::new_array(uint, (usz)n + 2).ptr;
    for (uint i = 0; i < n + 2; i++) rp[i] = 0;
    for (uint i = 0; i < n; i++) {
        for (uint j = 0; j < n; j++) {
            if (i != j && adj[i * n + j] != 0) rp[i + 1]++;
        }
    }
    for (uint i = 1; i <= n; i++) rp[i] += rp[i - 1];

    uint* ci = mem::new_array(uint, ne).ptr;
    uint* wt = mem::new_array(uint, ne).ptr;
    uint* pos = mem::new_array(uint, (usz)n + 1).ptr;
    for (uint i = 0; i <= n; i++) pos[i] = rp[i];

    for (uint i = 0; i < n; i++) {
        for (uint j = 0; j < n; j++) {
            if (i != j && adj[i * n + j] != 0) {
                uint p = pos[i];
                pos[i] = p + 1;
                ci[p] = j;
                wt[p] = 1;
            }
        }
    }

    free(pos);
    CsrGraph csr;
    csr.row_ptr = rp;
    csr.col_idx = ci;
    csr.weight = wt;
    csr.v = n;
    csr.e = ne;
    BoolDenseResult result;
    result.csr = csr;
    result.adj = adj;
    return result;
}

// ---------------------------------------------------------------------------
// gen_undirected: undirected graph (both directions stored in CSR)
// Used by: bidir_bfs (weights = 1)
// ---------------------------------------------------------------------------

fn CsrGraph gen_undirected(uint n, uint epn, uint seed) {
    Lcg rng = { .s = seed };
    uint target_e = n * epn;
    uint max_e = target_e * 2 + 2 * n;
    RawEdge* edges = mem::new_array(RawEdge, (usz)max_e).ptr;
    uint ne = 0;

    // Chain (both directions)
    for (uint i = 0; i + 1 < n; i++) {
        edges[ne].u = i;
        edges[ne].v = i + 1;
        edges[ne].w = 1;
        ne++;
        edges[ne].u = i + 1;
        edges[ne].v = i;
        edges[ne].w = 1;
        ne++;
    }

    // Random edges (both directions)
    uint attempts = n * (epn - 1) * 2;
    for (uint a = 0; a < attempts && ne < target_e * 2; a++) {
        uint u = rng.next() % n;
        uint v = rng.next() % n;
        if (u == v) continue;
        edges[ne].u = u;
        edges[ne].v = v;
        edges[ne].w = 1;
        ne++;
        edges[ne].u = v;
        edges[ne].v = u;
        edges[ne].w = 1;
        ne++;
    }

    // CSR
    uint* rp = mem::new_array(uint, (usz)n + 2).ptr;
    for (uint i = 0; i < n + 2; i++) rp[i] = 0;
    for (uint i = 0; i < ne; i++) rp[edges[i].u + 1]++;
    for (uint i = 1; i <= n; i++) rp[i] += rp[i - 1];

    uint* ci = mem::new_array(uint, ne).ptr;
    uint* pos = mem::new_array(uint, (usz)n + 1).ptr;
    for (uint i = 0; i <= n; i++) pos[i] = rp[i];

    for (uint i = 0; i < ne; i++) {
        uint u = edges[i].u;
        uint p = pos[u];
        pos[u] = p + 1;
        ci[p] = edges[i].v;
    }

    free(edges);
    free(pos);
    CsrGraph result;
    result.row_ptr = rp;
    result.col_idx = ci;
    result.weight = null;
    result.v = n;
    result.e = ne;
    return result;
}

// ---------------------------------------------------------------------------
// gen_spine_dag: sparse DAG with spine + skip edges (exponential paths)
// Used by: path_enum
// ---------------------------------------------------------------------------

struct SpineResult {
    CsrGraph csr;
    uint* spine;
    uint h;
}

fn SpineResult gen_spine_dag(uint v_count, uint k) {
    Lcg rng = { .s = 12345 + v_count + k };

    uint h = k + 5;
    if (h < 4) h = 4;
    if (h > v_count) h = v_count;

    uint* spine = mem::new_array(uint, h).ptr;
    for (uint i = 0; i < h; i++) {
        spine[i] = (uint)((ulong)i * (ulong)(v_count - 1) / (ulong)(h - 1));
    }
    spine[0] = 0;
    spine[h - 1] = v_count - 1;

    uint max_edges = (h - 1) + k;
    RawEdge* edges = mem::new_array(RawEdge, (usz)max_edges).ptr;
    uint ne = 0;

    // Spine edges
    for (uint i = 0; i + 1 < h; i++) {
        edges[ne].u = spine[i];
        edges[ne].v = spine[i + 1];
        edges[ne].w = rng.next() % 10 + 1;
        ne++;
    }

    // Skip edges: spine[i] -> spine[i+2]
    for (uint i = 0; i < k && i + 2 < h; i++) {
        edges[ne].u = spine[i];
        edges[ne].v = spine[i + 2];
        edges[ne].w = rng.next() % 10 + 1;
        ne++;
    }

    CsrGraph g = edges_to_csr(edges, ne, v_count, true);
    free(edges);
    SpineResult result;
    result.csr = g;
    result.spine = spine;
    result.h = h;
    return result;
}

// ---------------------------------------------------------------------------
// reverse_csr: build reverse CSR from forward CSR
// Used by: ch (backward graph)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// gen_geo_grid: geographic grid with 4-neighbor connectivity, haversine weights
// ---------------------------------------------------------------------------

struct GeoGridResult {
    CsrGraph csr;
    pathfind::LatLon* coords;
}

fn GeoGridResult gen_geo_grid(uint rows, uint cols,
                               double lat0, double lon0,
                               double lat1, double lon1) {
    uint n = rows * cols;
    pathfind::LatLon* coords = mem::new_array(pathfind::LatLon, n).ptr;

    double dlat = (lat1 - lat0) / (double)(rows > 1 ? rows - 1 : 1);
    double dlon = (lon1 - lon0) / (double)(cols > 1 ? cols - 1 : 1);

    for (uint r = 0; r < rows; r++) {
        for (uint c = 0; c < cols; c++) {
            uint id = r * cols + c;
            coords[id] = (pathfind::LatLon){
                .lat = lat0 + (double)r * dlat,
                .lon = lon0 + (double)c * dlon
            };
        }
    }

    // Count edges: 4-neighbor grid (right + down, bidirectional)
    uint max_e = 2 * (rows * (cols - 1) + (rows - 1) * cols) * 2;
    RawEdge* edges = mem::new_array(RawEdge, (usz)max_e).ptr;
    uint ne = 0;

    for (uint r = 0; r < rows; r++) {
        for (uint c = 0; c < cols; c++) {
            uint id = r * cols + c;
            // Right neighbor
            if (c + 1 < cols) {
                uint right = r * cols + (c + 1);
                uint w = pathfind::haversine_u32(coords[id], coords[right]);
                if (w < 1) w = 1;
                edges[ne].u = id; edges[ne].v = right; edges[ne].w = w; ne++;
                edges[ne].u = right; edges[ne].v = id; edges[ne].w = w; ne++;
            }
            // Down neighbor
            if (r + 1 < rows) {
                uint down = (r + 1) * cols + c;
                uint w = pathfind::haversine_u32(coords[id], coords[down]);
                if (w < 1) w = 1;
                edges[ne].u = id; edges[ne].v = down; edges[ne].w = w; ne++;
                edges[ne].u = down; edges[ne].v = id; edges[ne].w = w; ne++;
            }
        }
    }

    CsrGraph csr = edges_to_csr(edges, ne, n, true);
    free(edges);

    GeoGridResult result;
    result.csr = csr;
    result.coords = coords;
    return result;
}

// ---------------------------------------------------------------------------
// gen_geo_random: random geographic points with KNN connectivity
// ---------------------------------------------------------------------------

struct GeoRandomResult {
    CsrGraph csr;
    pathfind::LatLon* coords;
}

fn GeoRandomResult gen_geo_random(uint n, uint k, uint seed,
                                    double lat0, double lon0,
                                    double lat1, double lon1) {
    Lcg rng = { .s = seed };

    pathfind::LatLon* coords = mem::new_array(pathfind::LatLon, n).ptr;
    double lat_range = lat1 - lat0;
    double lon_range = lon1 - lon0;

    for (uint i = 0; i < n; i++) {
        coords[i] = (pathfind::LatLon){
            .lat = lat0 + (double)rng.next() / 2147483648.0 * lat_range,
            .lon = lon0 + (double)rng.next() / 2147483648.0 * lon_range
        };
    }

    // Build edges via KNN using a GeoGraph
    pathfind::GeoGraph geo;
    geo.init(n);

    for (uint i = 0; i < n; i++) {
        geo.add_node(coords[i].lat, coords[i].lon);
    }
    geo.connect_knn(k);

    // Extract edges from GeoGraph into CSR
    uint ne = 0;
    for (uint i = 0; i < n; i++) {
        ne += (uint)geo.graph.adj[i].len();
    }

    RawEdge* edges = mem::new_array(RawEdge, (usz)ne).ptr;
    uint idx = 0;
    for (uint i = 0; i < n; i++) {
        for (usz j = 0; j < geo.graph.adj[i].len(); j++) {
            pathfind::Edge e = geo.graph.adj[i][j];
            edges[idx].u = i;
            edges[idx].v = e.to;
            edges[idx].w = e.weight;
            idx++;
        }
    }

    CsrGraph csr = edges_to_csr(edges, ne, n, true);
    free(edges);
    geo.destroy();

    GeoRandomResult result;
    result.csr = csr;
    result.coords = coords;
    return result;
}

// ---------------------------------------------------------------------------
// reverse_csr: build reverse CSR from forward CSR
// Used by: ch (backward graph)
// ---------------------------------------------------------------------------

fn CsrGraph reverse_csr(CsrGraph* fwd) {
    uint n = fwd.v;
    uint ne = fwd.e;

    uint* rrp = mem::new_array(uint, (usz)n + 2).ptr;
    for (uint i = 0; i < n + 2; i++) rrp[i] = 0;
    for (uint u = 0; u < n; u++) {
        for (uint e = fwd.row_ptr[u]; e < fwd.row_ptr[u + 1]; e++) {
            rrp[fwd.col_idx[e] + 1]++;
        }
    }
    for (uint i = 1; i <= n; i++) rrp[i] += rrp[i - 1];

    uint* rci = mem::new_array(uint, ne).ptr;
    uint* rwt = mem::new_array(uint, ne).ptr;
    uint* pos = mem::new_array(uint, (usz)n + 1).ptr;
    for (uint i = 0; i <= n; i++) pos[i] = rrp[i];

    for (uint u = 0; u < n; u++) {
        for (uint e = fwd.row_ptr[u]; e < fwd.row_ptr[u + 1]; e++) {
            uint v = fwd.col_idx[e];
            uint p = pos[v];
            pos[v] = p + 1;
            rci[p] = u;
            rwt[p] = fwd.weight[e];
        }
    }

    free(pos);
    CsrGraph result;
    result.row_ptr = rrp;
    result.col_idx = rci;
    result.weight = rwt;
    result.v = n;
    result.e = ne;
    return result;
}
