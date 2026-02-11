module pathfind;
import std::collections;

const uint INF = 999_999;

faultdef INVALID_NODE, NO_PATH;

// ----- Data types -----

struct Edge {
    uint to;
    uint weight;
}

alias EdgeList = list::List {Edge};
alias UintList = list::List {uint};

struct Graph {
    uint n;
    EdgeList[] adj;
}

struct PathResult {
    uint[] weights;
    usz    count;
}

// ----- Graph lifecycle -----

fn void Graph.init(&self, uint node_count) {
    self.n = node_count;
    self.adj = mem::new_array(EdgeList, node_count);
}

fn void Graph.add_edge(&self, uint from, uint to, uint weight) {
    self.adj[from].push((Edge){ .to = to, .weight = weight });
}

fn void Graph.add_biedge(&self, uint a, uint b, uint weight) {
    self.add_edge(a, b, weight);
    self.add_edge(b, a, weight);
}

fn void Graph.destroy(&self) {
    for (uint i = 0; i < self.n; i++) {
        self.adj[i].free();
    }
    free(self.adj.ptr);
}

// ----- Internal helpers -----

fn void relax(uint[] dist, uint u, uint v, uint w) @inline {
    if (dist[u] >= INF) return;
    uint nd = dist[u] + w;
    if (nd < dist[v]) {
        dist[v] = nd;
    }
}

fn uint[] new_dist(uint n, uint source) {
    uint[] d = mem::new_array(uint, n);
    for (uint i = 0; i < n; i++) d[i] = INF;
    d[source] = 0;
    return d;
}

// ============================================================
// 1. Bellman-Ford SSSP
//    V-1 rounds of edge relaxation over all edges.
// ============================================================

fn uint[]? bellman_ford(Graph* g, uint source) {
    uint n = g.n;
    if (source >= n) return INVALID_NODE~;

    uint[] dist = new_dist(n, source);

    for (uint round = 0; round + 1 < n; round++) {
        for (uint u = 0; u < n; u++) {
            for (usz j = 0; j < g.adj[u].len(); j++) {
                Edge e = g.adj[u][j];
                relax(dist, u, e.to, e.weight);
            }
        }
    }
    return dist;
}

// ============================================================
// 2. Delta-Stepping SSSP
//    Light edges (w <= delta) relaxed twice per round,
//    heavy edges (w > delta) once.
// ============================================================

fn uint[]? delta_stepping(Graph* g, uint source, uint delta) {
    uint n = g.n;
    if (source >= n) return INVALID_NODE~;

    uint[] dist = new_dist(n, source);

    for (uint round = 0; round + 1 < n; round++) {
        // Light pass x2
        for (int pass = 0; pass < 2; pass++) {
            for (uint u = 0; u < n; u++) {
                for (usz j = 0; j < g.adj[u].len(); j++) {
                    Edge e = g.adj[u][j];
                    if (e.weight <= delta) {
                        relax(dist, u, e.to, e.weight);
                    }
                }
            }
        }
        // Heavy pass x1
        for (uint u = 0; u < n; u++) {
            for (usz j = 0; j < g.adj[u].len(); j++) {
                Edge e = g.adj[u][j];
                if (e.weight > delta) {
                    relax(dist, u, e.to, e.weight);
                }
            }
        }
    }
    return dist;
}

// ============================================================
// 3. Bidirectional BFS
//    Alternating frontier expansion; unweighted distance.
// ============================================================

fn bool list_contains(UintList* haystack, uint needle) {
    for (usz i = 0; i < haystack.len(); i++) {
        if ((*haystack)[i] == needle) return true;
    }
    return false;
}

fn bool frontiers_overlap(UintList* a, UintList* b) {
    for (usz i = 0; i < a.len(); i++) {
        if (list_contains(b, (*a)[i])) return true;
    }
    return false;
}

fn UintList expand_frontier(Graph* g, UintList* frontier) {
    UintList next;
    for (usz i = 0; i < frontier.len(); i++) {
        uint node = (*frontier)[i];
        for (usz j = 0; j < g.adj[node].len(); j++) {
            next.push(g.adj[node][j].to);
        }
    }
    return next;
}

fn uint? bidir_bfs(Graph* g, uint source, uint target) {
    if (source >= g.n || target >= g.n) return INVALID_NODE~;
    if (source == target) return 0;

    UintList fwd;
    UintList bwd;
    fwd.push(source);
    bwd.push(target);

    uint max_d = g.n * 2;

    for (uint d = 0; d <= max_d; d++) {
        if (frontiers_overlap(&fwd, &bwd)) {
            fwd.free();
            bwd.free();
            return d;
        }
        UintList new_fwd = expand_frontier(g, &fwd);
        fwd.free();
        fwd = bwd;
        bwd = new_fwd;
    }

    fwd.free();
    bwd.free();
    return NO_PATH~;
}

// ============================================================
// 4. Contraction Hierarchy Query
//    Bidirectional upward-only relaxation, meet in middle.
// ============================================================

fn uint? contraction_query(Graph* fwd_g, Graph* bwd_g, uint source, uint target, uint node_count) {
    if (source >= node_count || target >= node_count) return INVALID_NODE~;

    uint[] df = new_dist(node_count, source);
    uint[] db = new_dist(node_count, target);
    defer { free(df.ptr); free(db.ptr); }

    // Forward: rank order 0..n-1
    for (uint v = 0; v < node_count; v++) {
        if (df[v] >= INF) continue;
        for (usz j = 0; j < fwd_g.adj[v].len(); j++) {
            Edge e = fwd_g.adj[v][j];
            relax(df, v, e.to, e.weight);
        }
    }

    // Backward: reverse rank order n-1..0
    for (uint v = node_count; v > 0; ) {
        v--;
        if (db[v] >= INF) continue;
        for (usz j = 0; j < bwd_g.adj[v].len(); j++) {
            Edge e = bwd_g.adj[v][j];
            relax(db, v, e.to, e.weight);
        }
    }

    // Meet: min(df[v] + db[v])
    uint best = INF;
    for (uint v = 0; v < node_count; v++) {
        if (df[v] < INF && db[v] < INF) {
            uint s = df[v] + db[v];
            if (s < best) best = s;
        }
    }

    if (best >= INF) return NO_PATH~;
    return best;
}

// ============================================================
// 5. Algebraic APSP (Tropical Semiring)
//    Matrix squaring under (min, +).
// ============================================================

fn uint trop_add(uint a, uint b) @inline {
    return a < b ? a : b;
}

fn uint trop_mul(uint a, uint b) @inline {
    if (a >= INF || b >= INF) return INF;
    return a + b;
}

fn uint[]? algebraic_apsp(Graph* g) {
    uint n = g.n;
    uint[] mat = mem::new_array(uint, (usz)n * n);
    uint[] tmp = mem::new_array(uint, (usz)n * n);
    defer free(tmp.ptr);

    // Build initial distance matrix
    for (uint i = 0; i < n * n; i++) mat[i] = INF;
    for (uint i = 0; i < n; i++) mat[i * n + i] = 0;

    for (uint u = 0; u < n; u++) {
        for (usz j = 0; j < g.adj[u].len(); j++) {
            Edge e = g.adj[u][j];
            uint idx = u * n + e.to;
            if (e.weight < mat[idx]) mat[idx] = e.weight;
        }
    }

    // Repeated squaring ceil(log2(n)) times
    uint rounds = 0;
    uint nn = n;
    while (nn > 1) { rounds++; nn = (nn + 1) / 2; }
    if (rounds == 0) rounds = 1;

    for (uint r = 0; r < rounds; r++) {
        for (uint i = 0; i < n; i++) {
            for (uint j = 0; j < n; j++) {
                uint val = INF;
                for (uint k = 0; k < n; k++) {
                    val = trop_add(val, trop_mul(mat[i * n + k], mat[k * n + j]));
                }
                tmp[i * n + j] = val;
            }
        }
        for (uint i = 0; i < n * n; i++) mat[i] = tmp[i];
    }

    return mat;
}

// ============================================================
// 6. Path Enumeration (all source-to-sink paths in a DAG)
// ============================================================

fn void enumerate_dfs(Graph* g, uint node, uint sink, uint weight, UintList* results) {
    if (node == sink) {
        results.push(weight);
        return;
    }
    for (usz j = 0; j < g.adj[node].len(); j++) {
        Edge e = g.adj[node][j];
        enumerate_dfs(g, e.to, sink, weight + e.weight, results);
    }
}

fn PathResult? enumerate_paths(Graph* g, uint source, uint sink) {
    if (source >= g.n || sink >= g.n) return INVALID_NODE~;

    UintList results;
    enumerate_dfs(g, source, sink, 0, &results);

    usz count = results.len();
    uint[] arr = mem::new_array(uint, count);
    for (usz i = 0; i < count; i++) {
        arr[i] = results[i];
    }
    results.free();

    return (PathResult){ .weights = arr, .count = count };
}
