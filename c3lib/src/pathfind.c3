module pathfind;
import std::collections;
import std::core::dstring;
import std::io;

const uint INF = 999_999;

faultdef INVALID_NODE, NO_PATH, HVM_ERROR;

// ----- FFI declarations -----

extern fn void hvm4_lib_init() @cname("hvm4_lib_init");
extern fn void hvm4_lib_cleanup() @cname("hvm4_lib_cleanup");
extern fn void hvm4_lib_reset() @cname("hvm4_lib_reset");
extern fn int hvm4_run(char* source, int collapse_limit, uint* out, int max_out) @cname("hvm4_run");

// ----- Data types -----

struct Edge {
    uint to;
    uint weight;
}

alias EdgeList = list::List{Edge};

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

// ----- Init/cleanup wrappers -----

fn void runtime_init() {
    hvm4_lib_init();
}

fn void runtime_cleanup() {
    hvm4_lib_cleanup();
}

// ----- DString helpers -----

alias DStr = dstring::DString;

fn DStr new_dstr() {
    return dstring::new(mem);
}

// ============================================================
// Helper: compute ceil(log2(n)) for repeated squaring rounds
// ============================================================

fn uint ceil_log2(uint n) {
    if (n <= 1) return 1;
    uint rounds = 0;
    uint nn = n;
    while (nn > 1) {
        rounds++;
        nn = (nn + 1) / 2;
    }
    if (rounds == 0) rounds = 1;
    return rounds;
}

// ============================================================
// 1. Bellman-Ford SSSP
//    Generates HVM4 source with assoc-list helpers and edge
//    relaxation, calls hvm4_run(), extracts results.
// ============================================================

fn uint[]? bellman_ford(Graph* g, uint source) {
    uint n = g.n;
    if (source >= n) return INVALID_NODE~;

    hvm4_lib_reset();

    DStr ds = new_dstr();
    defer ds.free();

    // --- Constants ---
    ds.append_string("@INF = 999999\n");

    // --- Helpers (matching path_bellman_ford.hvm4 exactly) ---
    ds.append_string("@assoc_get = \xce\xbb&key. \xce\xbb&def. \xce\xbb{[]: def; <>: \xce\xbb&h. \xce\xbbt. \xce\xbb{#KV: \xce\xbb&k. \xce\xbbv. \xce\xbb{0: @assoc_get(key, def, t); \xce\xbbn. v}(k == key)}(h)}\n");
    ds.append_string("@assoc_set = \xce\xbb&key. \xce\xbb&val. \xce\xbb{[]: [#KV{key, val}]; <>: \xce\xbb&h. \xce\xbb&t. \xce\xbb{#KV: \xce\xbb&k. \xce\xbb&v. \xce\xbb{0: #KV{k, v} <> @assoc_set(key, val, t); \xce\xbbn. #KV{k, val} <> t}(k == key)}(h)}\n");
    ds.append_string("@min = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: b; \xce\xbbn. a}(a < b)\n");
    ds.append_string("@relax_edge = \xce\xbb&dist. \xce\xbb{#E3: \xce\xbb&u. \xce\xbb&v. \xce\xbbw. ! &du = @assoc_get(u, @INF, dist); ! &new_d = du + w; ! &dv = @assoc_get(v, @INF, dist); \xce\xbb{0: dist; \xce\xbbn. @assoc_set(v, new_d, dist)}(new_d < dv)}\n");
    ds.append_string("@foldl = \xce\xbb&f. \xce\xbb&acc. \xce\xbb{[]: acc; <>: \xce\xbbh. \xce\xbbt. @foldl(f, f(acc, h), t)}\n");
    ds.append_string("@relax_round = \xce\xbbdist. @foldl(@relax_edge, dist, @edges)\n");
    ds.append_string("@repeat = \xce\xbb&f. \xce\xbb&x. \xce\xbb{0: x; \xce\xbbn. @repeat(f, f(x), n - 1)}\n");

    // --- Generated edge list ---
    ds.append_string("@edges = [");
    bool first = true;
    for (uint u = 0; u < n; u++) {
        for (usz j = 0; j < g.adj[u].len(); j++) {
            Edge e = g.adj[u][j];
            if (!first) ds.append_string(", ");
            ds.appendf("#E3{%d,%d,%d}", u, e.to, e.weight);
            first = false;
        }
    }
    ds.append_string("]\n");

    // --- Initial distance ---
    ds.appendf("@init_dist = [#KV{%d, 0}]\n", source);

    // --- Run V-1 rounds ---
    uint rounds = n > 1 ? n - 1 : 1;
    ds.appendf("@bf = @repeat(@relax_round, @init_dist, %d)\n", rounds);

    // --- Extract results ---
    ds.append_string("@main = [");
    for (uint i = 0; i < n; i++) {
        if (i > 0) ds.append_string(", ");
        ds.appendf("@assoc_get(%d, @INF, @bf)", i);
    }
    ds.append_string("]\n");

    // --- Run HVM4 ---
    uint[] out_buf = mem::new_array(uint, n);
    int count = hvm4_run(ds.zstr_view(), 0, out_buf.ptr, (int)n);

    if (count < 0) {
        free(out_buf.ptr);
        return HVM_ERROR~;
    }

    return out_buf;
}

// ============================================================
// 2. Delta-Stepping SSSP
//    Light/heavy edge classification with repeated relaxation.
// ============================================================

fn uint[]? delta_stepping(Graph* g, uint source, uint delta) {
    uint n = g.n;
    if (source >= n) return INVALID_NODE~;

    hvm4_lib_reset();

    DStr ds = new_dstr();
    defer ds.free();

    // --- Constants ---
    ds.append_string("@INF = 999999\n");

    // --- Helpers ---
    ds.append_string("@assoc_get = \xce\xbb&key. \xce\xbb&default. \xce\xbb{[]: default; <>: \xce\xbbh. \xce\xbbt. \xce\xbb{#KV: \xce\xbb&k. \xce\xbbv. \xce\xbb{0: @assoc_get(key, default, t); \xce\xbbn. v}(k == key)}(h)}\n");
    ds.append_string("@assoc_set = \xce\xbb&key. \xce\xbb&val. \xce\xbb{[]: [#KV{key, val}]; <>: \xce\xbb&h. \xce\xbb&t. \xce\xbb{#KV: \xce\xbb&k. \xce\xbbv. \xce\xbb{0: h <> @assoc_set(key, val, t); \xce\xbbn. #KV{k, val} <> t}(k == key)}(h)}\n");
    ds.append_string("@min = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: b; \xce\xbbn. a}(a < b)\n");
    ds.append_string("@relax_edge = \xce\xbb&dist. \xce\xbb{#E3: \xce\xbb&u. \xce\xbb&v. \xce\xbbw. ! &du = @assoc_get(u, @INF, dist); ! &new_d = du + w; ! &dv = @assoc_get(v, @INF, dist); \xce\xbb{0: dist; \xce\xbbn. @assoc_set(v, new_d, dist)}(new_d < dv)}\n");
    ds.append_string("@relax_list = \xce\xbb&dist. \xce\xbb{[]: dist; <>: \xce\xbbh. \xce\xbbt. @relax_list(@relax_edge(dist, h), t)}\n");
    ds.append_string("@repeat = \xce\xbb&f. \xce\xbb&x. \xce\xbb{0: x; \xce\xbbn. @repeat(f, f(x), n - 1)}\n");

    // --- Light and heavy edge lists ---
    ds.append_string("@light_edges = [");
    bool first_l = true;
    for (uint u = 0; u < n; u++) {
        for (usz j = 0; j < g.adj[u].len(); j++) {
            Edge e = g.adj[u][j];
            if (e.weight <= delta) {
                if (!first_l) ds.append_string(", ");
                ds.appendf("#E3{%d, %d, %d}", u, e.to, e.weight);
                first_l = false;
            }
        }
    }
    ds.append_string("]\n");

    ds.append_string("@heavy_edges = [");
    bool first_h = true;
    for (uint u = 0; u < n; u++) {
        for (usz j = 0; j < g.adj[u].len(); j++) {
            Edge e = g.adj[u][j];
            if (e.weight > delta) {
                if (!first_h) ds.append_string(", ");
                ds.appendf("#E3{%d, %d, %d}", u, e.to, e.weight);
                first_h = false;
            }
        }
    }
    ds.append_string("]\n");

    // --- One delta-stepping round ---
    ds.append_string("@one_round = \xce\xbbdist. ! &d1 = @relax_list(dist, @light_edges); ! &d2 = @relax_list(d1, @light_edges); @relax_list(d2, @heavy_edges)\n");

    // --- Initial distance ---
    ds.appendf("@init_dist = [#KV{%d, 0}]\n", source);

    // --- Run V-1 rounds ---
    uint rounds = n > 1 ? n - 1 : 1;
    ds.appendf("@result = @repeat(@one_round, @init_dist, %d)\n", rounds);

    // --- Extract results ---
    ds.append_string("@main = [");
    for (uint i = 0; i < n; i++) {
        if (i > 0) ds.append_string(", ");
        ds.appendf("@assoc_get(%d, @INF, @result)", i);
    }
    ds.append_string("]\n");

    // --- Run HVM4 ---
    uint[] out_buf = mem::new_array(uint, n);
    int count = hvm4_run(ds.zstr_view(), 0, out_buf.ptr, (int)n);

    if (count < 0) {
        free(out_buf.ptr);
        return HVM_ERROR~;
    }

    return out_buf;
}

// ============================================================
// 3. Bidirectional BFS
//    Alternating frontier expansion; unweighted distance.
// ============================================================

fn uint? bidir_bfs(Graph* g, uint source, uint target) {
    if (source >= g.n || target >= g.n) return INVALID_NODE~;
    if (source == target) return 0;

    uint n = g.n;

    hvm4_lib_reset();

    DStr ds = new_dstr();
    defer ds.free();

    // --- Adjacency function ---
    ds.append_string("@adj = \xce\xbb{");
    for (uint u = 0; u < n; u++) {
        ds.appendf("%d: [", u);
        for (usz j = 0; j < g.adj[u].len(); j++) {
            if (j > 0) ds.append_string(", ");
            ds.appendf("%d", g.adj[u][j].to);
        }
        ds.append_string("]; ");
    }
    ds.append_string("\xce\xbbn. []}\n");

    // --- Helpers ---
    ds.append_string("@member = \xce\xbb&x. \xce\xbb{[]: 0; <>: \xce\xbb&h. \xce\xbbt. \xce\xbb{0: @member(x, t); \xce\xbbn. 1}(h == x)}\n");
    ds.append_string("@any_in = \xce\xbb&ys. \xce\xbb{[]: 0; <>: \xce\xbb&h. \xce\xbbt. \xce\xbb{0: @any_in(ys, t); \xce\xbbn. 1}(@member(h, ys))}\n");
    ds.append_string("@append = \xce\xbb{[]: \xce\xbbys. ys; <>: \xce\xbbh. \xce\xbbt. \xce\xbbys. h <> @append(t, ys)}\n");
    ds.append_string("@concat_map = \xce\xbb&f. \xce\xbb{[]: []; <>: \xce\xbbh. \xce\xbbt. @append(f(h), @concat_map(f, t))}\n");
    ds.append_string("@expand = \xce\xbbfrontier. @concat_map(@adj, frontier)\n");

    // --- BFS ---
    uint max_d = n * 2;
    ds.append_string("@bfs = \xce\xbb&fwd. \xce\xbb&bwd. \xce\xbb&dist. \xce\xbb&max. \xce\xbb{0: \xce\xbb{0: ! &new_fwd = @expand(fwd); @bfs(bwd, new_fwd, dist + 1, max); \xce\xbbn. dist}(@any_in(bwd, fwd)); \xce\xbbn. 999}(dist > max)\n");

    ds.appendf("@main = @bfs([%d], [%d], 0, %d)\n", source, target, max_d);

    // --- Run HVM4 ---
    uint[1] out_buf;
    int count = hvm4_run(ds.zstr_view(), 0, &out_buf[0], 1);

    if (count < 0) return HVM_ERROR~;

    uint result = out_buf[0];
    if (result >= 999) return NO_PATH~;
    return result;
}

// ============================================================
// 4. Contraction Hierarchy Query
//    Bidirectional upward-only relaxation, meet in middle.
// ============================================================

fn uint? contraction_query(Graph* fwd_g, Graph* bwd_g, uint source, uint target, uint node_count) {
    if (source >= node_count || target >= node_count) return INVALID_NODE~;

    hvm4_lib_reset();

    DStr ds = new_dstr();
    defer ds.free();

    uint n = node_count;

    // --- Constants ---
    ds.append_string("@INF = 999999\n");
    ds.append_string("@min = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: b; \xce\xbbn. a}(a < b)\n");

    // --- Helpers ---
    ds.append_string("@assoc_get = \xce\xbb&key. \xce\xbb&def. \xce\xbb{[]: def; <>: \xce\xbb&h. \xce\xbbt. \xce\xbb{#KV: \xce\xbb&k. \xce\xbbv. \xce\xbb{0: @assoc_get(key, def, t); \xce\xbbn. v}(k == key)}(h)}\n");
    ds.append_string("@assoc_set = \xce\xbb&key. \xce\xbb&val. \xce\xbb{[]: [#KV{key, val}]; <>: \xce\xbb&h. \xce\xbb&t. \xce\xbb{#KV: \xce\xbb&k. \xce\xbb&v. \xce\xbb{0: #KV{k, v} <> @assoc_set(key, val, t); \xce\xbbn. #KV{k, val} <> t}(k == key)}(h)}\n");
    ds.append_string("@foldl = \xce\xbb&f. \xce\xbb&acc. \xce\xbb{[]: acc; <>: \xce\xbbh. \xce\xbbt. @foldl(f, f(acc, h), t)}\n");

    // --- Forward adjacency ---
    ds.append_string("@fwd_adj = \xce\xbb{");
    for (uint u = 0; u < n; u++) {
        if (fwd_g.adj[u].len() > 0) {
            ds.appendf("%d: [", u);
            for (usz j = 0; j < fwd_g.adj[u].len(); j++) {
                if (j > 0) ds.append_string(", ");
                Edge e = fwd_g.adj[u][j];
                ds.appendf("#E{%d, %d}", e.to, e.weight);
            }
            ds.append_string("]; ");
        }
    }
    ds.append_string("\xce\xbbn. []}\n");

    // --- Backward adjacency ---
    ds.append_string("@bwd_adj = \xce\xbb{");
    for (uint u = 0; u < n; u++) {
        if (bwd_g.adj[u].len() > 0) {
            ds.appendf("%d: [", u);
            for (usz j = 0; j < bwd_g.adj[u].len(); j++) {
                if (j > 0) ds.append_string(", ");
                Edge e = bwd_g.adj[u][j];
                ds.appendf("#E{%d, %d}", e.to, e.weight);
            }
            ds.append_string("]; ");
        }
    }
    ds.append_string("\xce\xbbn. []}\n");

    // --- relax_edges ---
    ds.append_string("@relax_edges = \xce\xbb&src_d. \xce\xbb&dist. \xce\xbb{[]: dist; <>: \xce\xbbh. \xce\xbb&t. \xce\xbb{#E: \xce\xbb&tgt. \xce\xbbw. ! &new_d = src_d + w; ! &old_d = @assoc_get(tgt, @INF, dist); \xce\xbb{0: @relax_edges(src_d, dist, t); \xce\xbbn. @relax_edges(src_d, @assoc_set(tgt, new_d, dist), t)}(new_d < old_d)}(h)}\n");

    // --- process_node ---
    ds.append_string("@process_node = \xce\xbb&adj_fn. \xce\xbb&dist. \xce\xbb&node. ! &d = @assoc_get(node, @INF, dist); \xce\xbb{0: @relax_edges(d, dist, adj_fn(node)); \xce\xbbn. dist}(d == @INF)\n");

    // --- Forward pass: nodes in rank order [0, 1, ..., n-1] ---
    ds.append_string("@fwd_nodes = [");
    for (uint i = 0; i < n; i++) {
        if (i > 0) ds.append_string(", ");
        ds.appendf("%d", i);
    }
    ds.append_string("]\n");
    ds.appendf("@fwd_dist = @foldl(\xce\xbb&dist. \xce\xbbnode. @process_node(@fwd_adj, dist, node), [#KV{%d, 0}], @fwd_nodes)\n", source);

    // --- Backward pass: nodes in reverse rank order [n-1, ..., 0] ---
    ds.append_string("@bwd_nodes = [");
    for (uint i = n; i > 0; ) {
        i--;
        if (i + 1 < n) ds.append_string(", ");
        ds.appendf("%d", i);
    }
    ds.append_string("]\n");
    ds.appendf("@bwd_dist = @foldl(\xce\xbb&dist. \xce\xbbnode. @process_node(@bwd_adj, dist, node), [#KV{%d, 0}], @bwd_nodes)\n", target);

    // --- Meet ---
    ds.append_string("@all_nodes = [");
    for (uint i = 0; i < n; i++) {
        if (i > 0) ds.append_string(", ");
        ds.appendf("%d", i);
    }
    ds.append_string("]\n");
    ds.append_string("@meet = \xce\xbb&fd. \xce\xbb&bd. @foldl(\xce\xbb&best. \xce\xbb&v. ! &df = @assoc_get(v, @INF, fd); ! &db = @assoc_get(v, @INF, bd); @min(best, df + db), @INF, @all_nodes)\n");
    ds.append_string("@main = @meet(@fwd_dist, @bwd_dist)\n");

    // --- Run HVM4 ---
    uint[1] out_buf;
    int count = hvm4_run(ds.zstr_view(), 0, &out_buf[0], 1);

    if (count < 0) return HVM_ERROR~;

    uint result = out_buf[0];
    if (result >= INF) return NO_PATH~;
    return result;
}

// ============================================================
// 5. Algebraic APSP (Tropical Semiring)
//    Matrix squaring under (min, +) via HVM4.
// ============================================================

fn uint[]? algebraic_apsp(Graph* g) {
    uint n = g.n;

    hvm4_lib_reset();

    DStr ds = new_dstr();
    defer ds.free();

    // --- Constants ---
    ds.append_string("@INF = 999999\n");

    // --- Tropical operations ---
    ds.append_string("@trop_add = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: b; \xce\xbbn. a}(a < b)\n");
    ds.append_string("@trop_mul = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: \xce\xbb{0: a + b; \xce\xbbn. @INF}(b == @INF); \xce\xbbn. @INF}(a == @INF)\n");

    // --- List utilities ---
    ds.append_string("@map = \xce\xbb&f. \xce\xbb{[]: []; <>: \xce\xbbh. \xce\xbbt. f(h) <> @map(f, t)}\n");
    ds.append_string("@foldl = \xce\xbb&f. \xce\xbb&acc. \xce\xbb{[]: acc; <>: \xce\xbbh. \xce\xbbt. @foldl(f, f(acc, h), t)}\n");
    ds.append_string("@zip_with = \xce\xbb&f. \xce\xbbxs. \xce\xbbys. \xce\xbb{[]: []; <>: \xce\xbbhx. \xce\xbbtx. \xce\xbb{[]: []; <>: \xce\xbbhy. \xce\xbbty. f(hx, hy) <> @zip_with(f, tx, ty)}(ys)}(xs)\n");
    ds.append_string("@nth = \xce\xbb&n. \xce\xbb{[]: 0; <>: \xce\xbb&h. \xce\xbbt. \xce\xbb{0: h; \xce\xbbm. @nth(n - 1, t)}(n)}\n");
    ds.append_string("@col = \xce\xbb&j. \xce\xbb{[]: []; <>: \xce\xbb&row. \xce\xbbt. @nth(j, row) <> @col(j, t)}\n");
    ds.append_string("@dot = \xce\xbb&row. \xce\xbb&col. @foldl(\xce\xbb&a. \xce\xbbb. @trop_add(a, b), @INF, @zip_with(\xce\xbba. \xce\xbbb. @trop_mul(a, b), row, col))\n");

    // --- mat_mul: generated dynamically for size N ---
    ds.append_string("@mat_mul = \xce\xbb&a. \xce\xbb&b. ");
    for (uint j = 0; j < n; j++) {
        ds.appendf("! &c%d = @col(%d, b); ", j, j);
    }
    ds.append_string("@map(\xce\xbb&r. [");
    for (uint j = 0; j < n; j++) {
        if (j > 0) ds.append_string(", ");
        ds.appendf("@dot(r, c%d)", j);
    }
    ds.append_string("], a)\n");

    // --- Distance matrix ---
    // Build NxN initial distance matrix from graph adjacency
    ds.append_string("@D = [");
    for (uint i = 0; i < n; i++) {
        if (i > 0) ds.append_string(", ");
        ds.append_string("[");
        for (uint j = 0; j < n; j++) {
            if (j > 0) ds.append_string(", ");
            if (i == j) {
                ds.append_string("0");
            } else {
                // Find edge weight from i to j (or INF)
                uint w = INF;
                for (usz k = 0; k < g.adj[i].len(); k++) {
                    Edge e = g.adj[i][k];
                    if (e.to == j && e.weight < w) {
                        w = e.weight;
                    }
                }
                ds.appendf("%d", w);
            }
        }
        ds.append_string("]");
    }
    ds.append_string("]\n");

    // --- Repeated squaring ---
    uint rounds = ceil_log2(n);
    ds.append_string("@repeat = \xce\xbb&f. \xce\xbb&x. \xce\xbb{0: x; \xce\xbbn. @repeat(f, f(x), n - 1)}\n");
    ds.appendf("@main = @repeat(\xce\xbb&m. @mat_mul(m, m), @D, %d)\n", rounds);

    // --- Run HVM4 ---
    usz total = (usz)n * n;
    uint[] out_buf = mem::new_array(uint, total);
    int count = hvm4_run(ds.zstr_view(), 0, out_buf.ptr, (int)total);

    if (count < 0) {
        free(out_buf.ptr);
        return HVM_ERROR~;
    }

    return out_buf;
}

// ============================================================
// 6. Path Enumeration (all source-to-sink paths in a DAG)
//    Uses HVM4 superpositions (SUP) to enumerate paths.
// ============================================================

fn PathResult? enumerate_paths(Graph* g, uint source, uint sink) {
    if (source >= g.n || sink >= g.n) return INVALID_NODE~;

    uint n = g.n;

    hvm4_lib_reset();

    DStr ds = new_dstr();
    defer ds.free();

    // --- Generate @explore function with SUP labels ---
    // Each node with >1 outgoing edges gets a unique SUP label.
    // SUP labels use &A, &B, &C, etc.
    ds.append_string("@explore = \xce\xbb{");

    // Track which SUP label to assign next
    char sup_label = 'A';

    for (uint u = 0; u < n; u++) {
        usz edge_count = g.adj[u].len();

        ds.appendf("%d: ", u);

        if (u == sink) {
            ds.append_string("0");
        } else if (edge_count == 0) {
            ds.append_string("0");
        } else if (edge_count == 1) {
            Edge e = g.adj[u][0];
            ds.appendf("%d + @explore(%d)", e.weight, e.to);
        } else if (edge_count == 2) {
            Edge e0 = g.adj[u][0];
            Edge e1 = g.adj[u][1];
            ds.appendf("&%c{%d + @explore(%d), %d + @explore(%d)}",
                sup_label, e0.weight, e0.to, e1.weight, e1.to);
            sup_label++;
        } else {
            // For >2 edges, nest SUPs pairwise
            for (usz j = 0; j + 1 < edge_count; j++) {
                Edge ej = g.adj[u][j];
                ds.appendf("&%c{%d + @explore(%d), ", sup_label, ej.weight, ej.to);
                sup_label++;
            }
            // Last edge (innermost right)
            Edge elast = g.adj[u][edge_count - 1];
            ds.appendf("%d + @explore(%d)", elast.weight, elast.to);
            // Close all the nested SUPs
            for (usz j = 0; j + 1 < edge_count; j++) {
                ds.append_string("}");
            }
        }

        ds.append_string("; ");
    }
    ds.append_string("\xce\xbbn. 0}\n");

    ds.appendf("@main = @explore(%d)\n", source);

    // --- Run HVM4 in collapse mode ---
    int max_out = 1000;
    uint[] out_buf = mem::new_array(uint, max_out);
    int count = hvm4_run(ds.zstr_view(), max_out, out_buf.ptr, max_out);

    if (count < 0) {
        free(out_buf.ptr);
        return HVM_ERROR~;
    }

    // Copy results to right-sized array
    uint[] result = mem::new_array(uint, (usz)count);
    for (int i = 0; i < count; i++) {
        result[i] = out_buf[i];
    }
    free(out_buf.ptr);

    return (PathResult){ .weights = result, .count = (usz)count };
}
