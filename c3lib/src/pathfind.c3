module pathfind;
import std::collections;
import std::core::dstring;
import std::io;
import std::math;
import libc;

const uint INF = 999_999;

faultdef INVALID_NODE, NO_PATH, HVM_ERROR;

// ----- FFI declarations -----

extern fn void hvm4_lib_init() @cname("hvm4_lib_init");
extern fn void hvm4_lib_cleanup() @cname("hvm4_lib_cleanup");
extern fn void hvm4_lib_reset() @cname("hvm4_lib_reset");
extern fn int hvm4_run(char* source, int collapse_limit, uint* out, int max_out) @cname("hvm4_run");
extern fn void hvm4_graph_setup(uint* row_ptr, uint* col_idx, uint* weight, uint v) @cname("hvm4_graph_setup");
extern fn void hvm4_weather_setup(
    double* dist_m, double* heading,
    double* wind_speed, double* wind_dir, double* wave_height,
    double* current_speed, double* current_dir,
    double base_speed, double max_wave_ht, double fuel_base, double fuel_per_kt,
    uint cost_mode, double alpha, double safety_penalty, uint scale,
    uint edge_count
) @cname("hvm4_weather_setup");

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

// ----- Geography primitives -----

const double PI = 3.14159265358979323846;
const double EARTH_RADIUS_M = 6_371_000.0;
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;

struct LatLon {
    double lat;
    double lon;
}

fn double haversine(LatLon a, LatLon b) {
    double dlat = (b.lat - a.lat) * DEG_TO_RAD;
    double dlon = (b.lon - a.lon) * DEG_TO_RAD;
    double la = a.lat * DEG_TO_RAD;
    double lb = b.lat * DEG_TO_RAD;
    double s = math::sin(dlat / 2.0);
    double c = math::sin(dlon / 2.0);
    double h = s * s + math::cos(la) * math::cos(lb) * c * c;
    return 2.0 * EARTH_RADIUS_M * math::asin(math::sqrt(h));
}

fn double bearing(LatLon a, LatLon b) {
    double la = a.lat * DEG_TO_RAD;
    double lb = b.lat * DEG_TO_RAD;
    double dlon = (b.lon - a.lon) * DEG_TO_RAD;
    double y = math::sin(dlon) * math::cos(lb);
    double x = math::cos(la) * math::sin(lb) - math::sin(la) * math::cos(lb) * math::cos(dlon);
    double brg = math::atan2(y, x) * RAD_TO_DEG;
    if (brg < 0.0) brg += 360.0;
    return brg;
}

fn uint haversine_u32(LatLon a, LatLon b) {
    return (uint)haversine(a, b);
}

// ----- GeoGraph -----

struct GeoGraph {
    Graph graph;
    LatLon[] coords;
    uint node_count;
    uint capacity;
}

fn void GeoGraph.init(&self, uint capacity) {
    self.graph.init(capacity);
    self.coords = mem::new_array(LatLon, capacity);
    self.node_count = 0;
    self.capacity = capacity;
}

fn void GeoGraph.destroy(&self) {
    self.graph.destroy();
    if (self.coords.ptr != null) free(self.coords.ptr);
}

fn uint GeoGraph.add_node(&self, double lat, double lon) {
    uint id = self.node_count;
    self.coords[id] = (LatLon){ .lat = lat, .lon = lon };
    self.node_count++;
    return id;
}

fn void GeoGraph.add_edge(&self, uint from, uint to, uint weight) {
    self.graph.add_edge(from, to, weight);
}

fn void GeoGraph.add_geo_edge(&self, uint from, uint to) {
    uint w = haversine_u32(self.coords[from], self.coords[to]);
    if (w < 1) w = 1;
    self.graph.add_edge(from, to, w);
}

fn void GeoGraph.add_geo_biedge(&self, uint a, uint b) {
    uint w = haversine_u32(self.coords[a], self.coords[b]);
    if (w < 1) w = 1;
    self.graph.add_biedge(a, b, w);
}

fn uint GeoGraph.nearest_node(&self, double lat, double lon) {
    LatLon target = { .lat = lat, .lon = lon };
    uint best = 0;
    double best_dist = 1e18;
    for (uint i = 0; i < self.node_count; i++) {
        double d = haversine(target, self.coords[i]);
        if (d < best_dist) {
            best_dist = d;
            best = i;
        }
    }
    return best;
}

fn void GeoGraph.connect_knn(&self, uint k) {
    uint n = self.node_count;
    double[] dists = mem::new_array(double, n);
    uint[] indices = mem::new_array(uint, n);
    defer { free(dists.ptr); free(indices.ptr); }

    for (uint i = 0; i < n; i++) {
        for (uint j = 0; j < n; j++) {
            dists[j] = haversine(self.coords[i], self.coords[j]);
            indices[j] = j;
        }
        dists[i] = 1e18;

        uint found = 0;
        for (uint pass = 0; pass < n - 1 && found < k; pass++) {
            uint min_idx = pass;
            for (uint q = pass + 1; q < n; q++) {
                if (dists[q] < dists[min_idx]) min_idx = q;
            }
            double tmp_d = dists[pass]; dists[pass] = dists[min_idx]; dists[min_idx] = tmp_d;
            uint tmp_i = indices[pass]; indices[pass] = indices[min_idx]; indices[min_idx] = tmp_i;

            uint j = indices[pass];
            double dist = dists[pass];
            if (dist >= 1e17) break;

            self.add_geo_biedge(i, j);
            found++;
        }
    }
}

fn void GeoGraph.connect_radius(&self, double radius_m) {
    uint n = self.node_count;
    for (uint i = 0; i < n; i++) {
        for (uint j = i + 1; j < n; j++) {
            double d = haversine(self.coords[i], self.coords[j]);
            if (d <= radius_m) {
                uint w = (uint)d;
                if (w < 1) w = 1;
                self.graph.add_biedge(i, j, w);
            }
        }
    }
}

// ----- GeoGraph file loader -----

extern fn CFile fopen(ZString path, ZString mode) @cname("fopen");
extern fn int fclose(CFile f) @cname("fclose");
extern fn int fscanf(CFile f, ZString fmt, ...) @cname("fscanf");
extern fn char* fgets(char* buf, int size, CFile f) @cname("fgets");

fn GeoGraph? load_geo_file(ZString path) {
    CFile f = fopen(path, "r");
    if (f == null) return HVM_ERROR~;

    char[256] line_buf;
    uint node_count = 0;
    uint edge_count = 0;

    // Read NODES header
    while (fgets(&line_buf, 256, f) != null) {
        if (line_buf[0] == '#' || line_buf[0] == '\n') continue;
        if (fscanf(f, "") >= 0) {} // consume
        // Try to parse "NODES <n>"
        int scanned = fscanf(f, "");
        // Rewind — just use sscanf approach via manual parse
        break;
    }

    // Simpler: rewind and use fscanf directly
    fclose(f);
    f = fopen(path, "r");
    if (f == null) return HVM_ERROR~;
    defer fclose(f);

    // Skip comment lines, find NODES
    while (fgets(&line_buf, 256, f) != null) {
        if (line_buf[0] == '#' || line_buf[0] == '\n' || line_buf[0] == '\r') continue;
        // Check for "NODES"
        if (line_buf[0] == 'N') {
            // Parse "NODES <n>"
            uint n = 0;
            for (int i = 6; i < 256 && line_buf[i] != 0; i++) {
                if (line_buf[i] >= '0' && line_buf[i] <= '9') {
                    n = n * 10 + (uint)(line_buf[i] - '0');
                }
            }
            node_count = n;
            break;
        }
    }

    if (node_count == 0) return HVM_ERROR~;

    GeoGraph geo;
    geo.init(node_count);

    // Read node coordinates
    uint nodes_read = 0;
    while (nodes_read < node_count && fgets(&line_buf, 256, f) != null) {
        if (line_buf[0] == '#' || line_buf[0] == '\n' || line_buf[0] == '\r') continue;
        double lat = 0;
        double lon = 0;
        if (fscanf(f, "") >= 0) {} // dummy
        // Parse two doubles from line_buf
        // Use a simple manual parse
        int pos = 0;
        // skip whitespace
        while (line_buf[pos] == ' ' || line_buf[pos] == '\t') pos++;
        // parse lat
        bool neg = false;
        if (line_buf[pos] == '-') { neg = true; pos++; }
        double whole = 0;
        while (line_buf[pos] >= '0' && line_buf[pos] <= '9') {
            whole = whole * 10.0 + (double)(line_buf[pos] - '0');
            pos++;
        }
        double frac = 0;
        double div = 1.0;
        if (line_buf[pos] == '.') {
            pos++;
            while (line_buf[pos] >= '0' && line_buf[pos] <= '9') {
                frac = frac * 10.0 + (double)(line_buf[pos] - '0');
                div *= 10.0;
                pos++;
            }
        }
        lat = whole + frac / div;
        if (neg) lat = -lat;

        // skip whitespace/comma
        while (line_buf[pos] == ' ' || line_buf[pos] == '\t' || line_buf[pos] == ',') pos++;

        // parse lon
        neg = false;
        if (line_buf[pos] == '-') { neg = true; pos++; }
        whole = 0;
        while (line_buf[pos] >= '0' && line_buf[pos] <= '9') {
            whole = whole * 10.0 + (double)(line_buf[pos] - '0');
            pos++;
        }
        frac = 0;
        div = 1.0;
        if (line_buf[pos] == '.') {
            pos++;
            while (line_buf[pos] >= '0' && line_buf[pos] <= '9') {
                frac = frac * 10.0 + (double)(line_buf[pos] - '0');
                div *= 10.0;
                pos++;
            }
        }
        lon = whole + frac / div;
        if (neg) lon = -lon;

        geo.add_node(lat, lon);
        nodes_read++;
    }

    // Find EDGES header
    while (fgets(&line_buf, 256, f) != null) {
        if (line_buf[0] == '#' || line_buf[0] == '\n' || line_buf[0] == '\r') continue;
        if (line_buf[0] == 'E') {
            uint m = 0;
            for (int i = 6; i < 256 && line_buf[i] != 0; i++) {
                if (line_buf[i] >= '0' && line_buf[i] <= '9') {
                    m = m * 10 + (uint)(line_buf[i] - '0');
                }
            }
            edge_count = m;
            break;
        }
    }

    // Read edges
    uint edges_read = 0;
    while (edges_read < edge_count && fgets(&line_buf, 256, f) != null) {
        if (line_buf[0] == '#' || line_buf[0] == '\n' || line_buf[0] == '\r') continue;
        // Parse two uints: from to
        int pos = 0;
        while (line_buf[pos] == ' ' || line_buf[pos] == '\t') pos++;
        uint from_id = 0;
        while (line_buf[pos] >= '0' && line_buf[pos] <= '9') {
            from_id = from_id * 10 + (uint)(line_buf[pos] - '0');
            pos++;
        }
        while (line_buf[pos] == ' ' || line_buf[pos] == '\t') pos++;
        uint to_id = 0;
        while (line_buf[pos] >= '0' && line_buf[pos] <= '9') {
            to_id = to_id * 10 + (uint)(line_buf[pos] - '0');
            pos++;
        }

        if (from_id < geo.node_count && to_id < geo.node_count) {
            geo.add_geo_edge(from_id, to_id);
        }
        edges_read++;
    }

    return geo;
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
// Helper: emit tree operations for Bend-style parallel code
//   @tz — tree zip (element-wise op on two same-shape trees)
//   @tf — tree fold (parallel reduction)
//   @tm — tree map (apply f to every leaf)
//   @tg — tree get (extract element at index)
//   @bld — bifurcation builder (build tree from index→value fn)
//   @bld_inf — build tree of all @INF
// ============================================================

fn void emit_tree_ops(DStr* ds) {
    // Tree zip: element-wise operation on two trees of same shape
    // @tz(0, f, a, b) = f(a, b)  — leaf case
    // @tz(n, f, #P{al,ar}, #P{bl,br}) = #P{@tz(n-1,f,al,bl), @tz(n-1,f,ar,br)}
    ds.append_string("@tz = \xce\xbb{0: \xce\xbbf. \xce\xbba. \xce\xbbb. f(a, b); \xce\xbbn. \xce\xbb&f. \xce\xbba. \xce\xbbb. @tz_go(n, f, a, b)}\n");
    ds.append_string("@tz_go = \xce\xbb&n. \xce\xbb&f. \xce\xbb{#P: \xce\xbb&al. \xce\xbbar. \xce\xbb{#P: \xce\xbbbl. \xce\xbbbr. #P{@tz(n - 1, f, al, bl), @tz(n - 1, f, ar, br)}}}\n");

    // Tree fold: parallel reduction
    // @tf(0, op, leaf) = leaf
    // @tf(n, op, #P{l,r}) = op(@tf(n-1,op,l), @tf(n-1,op,r))
    ds.append_string("@tf = \xce\xbb{0: \xce\xbbop. \xce\xbbleaf. leaf; \xce\xbbn. ! &d = n; \xce\xbb&op. \xce\xbb{#P: \xce\xbbl. \xce\xbbr. op(@tf(d - 1, op, l), @tf(d - 1, op, r))}}\n");

    // Tree map: apply f to every leaf
    // @tm(0, f, leaf) = f(leaf)
    // @tm(n, f, #P{l,r}) = #P{@tm(n-1,f,l), @tm(n-1,f,r)}
    ds.append_string("@tm = \xce\xbb{0: \xce\xbbf. \xce\xbbleaf. f(leaf); \xce\xbbn. ! &d = n; \xce\xbb&f. \xce\xbb{#P: \xce\xbbl. \xce\xbbr. #P{@tm(d - 1, f, l), @tm(d - 1, f, r)}}}\n");

    // Tree get: extract element at index (binary decomposition)
    // Index bits: bit 0 selects L/R at depth 0, bit 1 at depth 1, etc.
    // @tg(0, idx, leaf) = leaf
    // @tg(n, idx, #P{l,r}) = if idx%2==0 then @tg(n-1, idx/2, l) else @tg(n-1, idx/2, r)
    ds.append_string("@tg = \xce\xbb{0: \xce\xbbidx. \xce\xbbleaf. leaf; \xce\xbbn. \xce\xbb&idx. \xce\xbb{#P: \xce\xbb&l. \xce\xbbr. @tg_sel(idx % 2, idx / 2, n, l, r)}}\n");
    ds.append_string("@tg_sel = \xce\xbb{0: \xce\xbbidx. \xce\xbbd. \xce\xbbl. \xce\xbbr. @tg(d - 1, idx, l); \xce\xbbm. \xce\xbbidx. \xce\xbbd. \xce\xbbl. \xce\xbbr. @tg(d - 1, idx, r)}\n");

    // Bifurcation builder: build tree of values from index→value function
    // @bld(0, i, lim, f) = if i<lim then f(i) else @INF
    // @bld(n, i, lim, f) = #P{@bld(n-1, i*2, lim, f), @bld(n-1, i*2+1, lim, f)}
    // @bld(d, i, step, lim, f): bit-interleaved layout matching @tg's idx%2 addressing
    ds.append_string("@bld = \xce\xbb{0: \xce\xbb&i. \xce\xbb&s. \xce\xbb&lim. \xce\xbb&f. \xce\xbb{0: @INF; \xce\xbbm. f(i)}(i < lim); \xce\xbbn. ! &d = n; \xce\xbb&i. \xce\xbb&s. \xce\xbb&lim. \xce\xbb&f. ! &ns = s * 2; #P{@bld(d - 1, i, ns, lim, f), @bld(d - 1, i + s, ns, lim, f)}}\n");

    // Build tree of all @INF (for padding)
    ds.append_string("@bld_inf = \xce\xbb{0: @INF; \xce\xbbn. ! &d = n; #P{@bld_inf(d - 1), @bld_inf(d - 1)}}\n");
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

    uint depth = ceil_log2(n);

    // --- Constants ---
    ds.append_string("@INF = 999999\n");
    ds.appendf("@TD = %d\n", depth);
    ds.appendf("@N = %d\n", n);

    // --- Tropical operations ---
    ds.append_string("@trop_add = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: b; \xce\xbbn. a}(a < b)\n");
    ds.append_string("@trop_mul = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: \xce\xbb{0: a + b; \xce\xbbn. @INF}(b == @INF); \xce\xbbn. @INF}(a == @INF)\n");

    // --- Tree operations (Bend-style parallel) ---
    emit_tree_ops(&ds);

    // --- Dot product: zip with trop_mul, fold with trop_add ---
    ds.append_string("@dot = \xce\xbb&row. \xce\xbb&cv. @tf(@TD, \xce\xbba. \xce\xbbb. @trop_add(a, b), @tz(@TD, \xce\xbba. \xce\xbbb. @trop_mul(a, b), row, cv))\n");

    // --- Extract column j: map over rows, get element j from each ---
    ds.append_string("@col = \xce\xbb&j. \xce\xbb&mat. @tm(@TD, \xce\xbbrow. @tg(@TD, j, row), mat)\n");

    // --- Build result row using bifurcation (bit-interleaved) ---
    ds.append_string("@brr = \xce\xbb{0: \xce\xbb&i. \xce\xbb&s. \xce\xbb&lim. \xce\xbb&row. \xce\xbb&B. \xce\xbb{0: @INF; \xce\xbbm. @dot(row, @col(i, B))}(i < lim); \xce\xbbn. ! &d = n; \xce\xbb&i. \xce\xbb&s. \xce\xbb&lim. \xce\xbb&row. \xce\xbb&B. ! &ns = s * 2; #P{@brr(d - 1, i, ns, lim, row, B), @brr(d - 1, i + s, ns, lim, row, B)}}\n");

    // --- Full mat_mul: map over rows of A, build result row for each ---
    ds.append_string("@mat_mul = \xce\xbb&A. \xce\xbb&B. @tm(@TD, \xce\xbb&row. @brr(@TD, 0, 1, @N, row, B), A)\n");

    // --- Build initial distance matrix as tree ---
    // Precompute adjacency matrix in C3
    uint* adj_mat = (uint*)mem::calloc((usz)n * n * uint.sizeof);
    defer free(adj_mat);
    for (uint i = 0; i < n * n; i++) adj_mat[i] = INF;
    for (uint i = 0; i < n; i++) {
        adj_mat[i * n + i] = 0;
        for (usz k = 0; k < g.adj[i].len(); k++) {
            Edge e = g.adj[i][k];
            if (e.weight < adj_mat[i * n + e.to]) {
                adj_mat[i * n + e.to] = e.weight;
            }
        }
    }

    // Emit row value lookup: @rv_i = switch on column index
    for (uint i = 0; i < n; i++) {
        ds.appendf("@rv_%d = \xce\xbb{", i);
        for (uint j = 0; j < n; j++) {
            ds.appendf("%d: %d; ", j, adj_mat[i * n + j]);
        }
        ds.appendf("\xce\xbbn. @INF}\n");
    }

    // Emit row builders using bifurcation
    for (uint i = 0; i < n; i++) {
        ds.appendf("@row_%d = @bld(@TD, 0, 1, @N, @rv_%d)\n", i, i);
    }

    // Matrix row lookup: switch on row index
    ds.append_string("@mrv = \xce\xbb{");
    for (uint i = 0; i < n; i++) {
        ds.appendf("%d: @row_%d; ", i, i);
    }
    ds.append_string("\xce\xbbn. @bld_inf(@TD)}\n");

    // Build matrix tree via bifurcation over rows
    ds.append_string("@D_init = @bld(@TD, 0, 1, @N, @mrv)\n");

    // --- Repeated squaring ---
    uint rounds = ceil_log2(n);
    ds.append_string("@repeat = \xce\xbb&f. \xce\xbb&x. \xce\xbb{0: x; \xce\xbbn. @repeat(f, f(x), n - 1)}\n");
    ds.appendf("@result = @repeat(\xce\xbb&m. @mat_mul(m, m), @D_init, %d)\n", rounds);

    // --- Extract results as flat list ---
    // Direct extraction: @tg(depth, j, @tg(depth, i, mat))
    ds.append_string("@main = [");
    for (uint i = 0; i < n; i++) {
        for (uint j = 0; j < n; j++) {
            if (i > 0 || j > 0) ds.append_string(", ");
            ds.appendf("@tg(@TD, %d, @tg(@TD, %d, @result))", j, i);
        }
    }
    ds.append_string("]\n");

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

// ============================================================
// 7. DAG-DP (shortest path via nested let-bindings)
//    Graph structure embedded directly in HVM4 let-binding chain.
//    Each node becomes a `! &eXXX = ...` binding; @min reduces edges.
//    O(E) work, O(V+E) heap. No trie needed.
//    Requires DAG input: edges must go from lower to higher node index.
// ============================================================

// B64 alphabet for collision-free 4-char variable names (HVM4 nick encoding)
const ZString B64 = "_abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789$";

fn void append_node_name(DStr* ds, uint n) {
    ds.append_char('e');
    ds.append_char(B64[(n >> 12) & 63]);
    ds.append_char(B64[(n >> 6) & 63]);
    ds.append_char(B64[n & 63]);
}

fn uint? dag_dp(Graph* g, uint source, uint sink) {
    if (source >= g.n || sink >= g.n) return INVALID_NODE~;

    uint n = g.n;

    hvm4_lib_reset();

    DStr ds = new_dstr();
    defer ds.free();

    ds.append_string("@min = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: b; \xce\xbbn. a}(a < b)\n");
    ds.append_string("@INF = 999999\n");
    ds.append_string("@main =\n");

    // Destination binding: dist = 0
    ds.append_string("  ! &");
    append_node_name(&ds, sink);
    ds.append_string(" = 0;\n");

    // Non-source, non-sink nodes in reverse topological order
    for (int u = (int)n - 2; u >= 0; u--) {
        if ((uint)u == source) continue;
        if ((uint)u == sink) continue;

        usz deg = g.adj[(uint)u].len();
        if (deg == 0) {
            ds.append_string("  ! &");
            append_node_name(&ds, (uint)u);
            ds.append_string(" = @INF;\n");
        } else if (deg == 1) {
            Edge e = g.adj[(uint)u][0];
            ds.append_string("  ! &");
            append_node_name(&ds, (uint)u);
            ds.appendf(" = %d + ", e.weight);
            append_node_name(&ds, e.to);
            ds.append_string(";\n");
        } else {
            // Multiple edges: @min(w1 + t1, @min(w2 + t2, ... wk + tk))
            ds.append_string("  ! &");
            append_node_name(&ds, (uint)u);
            ds.append_string(" = ");
            for (usz i = 0; i < deg - 1; i++) {
                Edge e = g.adj[(uint)u][i];
                ds.appendf("@min(%d + ", e.weight);
                append_node_name(&ds, e.to);
                ds.append_string(", ");
            }
            Edge elast = g.adj[(uint)u][deg - 1];
            ds.appendf("%d + ", elast.weight);
            append_node_name(&ds, elast.to);
            for (usz i = 0; i < deg - 1; i++) ds.append_string(")");
            ds.append_string(";\n");
        }
    }

    // Source node: return expression (not bound)
    usz src_deg = g.adj[source].len();
    if (src_deg == 0) {
        ds.append_string("  @INF\n");
    } else if (src_deg == 1) {
        Edge e = g.adj[source][0];
        ds.appendf("  %d + ", e.weight);
        append_node_name(&ds, e.to);
        ds.append_string("\n");
    } else {
        ds.append_string("  ");
        for (usz i = 0; i < src_deg - 1; i++) {
            Edge e = g.adj[source][i];
            ds.appendf("@min(%d + ", e.weight);
            append_node_name(&ds, e.to);
            ds.append_string(", ");
        }
        Edge elast = g.adj[source][src_deg - 1];
        ds.appendf("%d + ", elast.weight);
        append_node_name(&ds, elast.to);
        for (usz i = 0; i < src_deg - 1; i++) ds.append_string(")");
        ds.append_string("\n");
    }

    // Run HVM4 (no collapse needed — single numeric result)
    uint result;
    int count = hvm4_run(ds.zstr_view(), 0, &result, 1);

    if (count < 0) return HVM_ERROR~;
    if (result >= INF) return NO_PATH~;

    return result;
}

// ============================================================
// 8. Connected Components (Label Propagation)
//    Tree-parallel: each round, every node takes min of its
//    own label and all neighbors' labels. O(log V) rounds.
//    Graph should have edges in both directions for undirected CC.
//    Returns uint[] of N labels (label = min node index in component).
// ============================================================

fn uint[]? connected_components(Graph* g) {
    uint n = g.n;
    if (n == 0) return INVALID_NODE~;

    hvm4_lib_reset();

    DStr ds = new_dstr();
    defer ds.free();

    uint depth = ceil_log2(n);
    uint rounds = depth;  // O(log V) rounds sufficient for convergence

    // Constants
    ds.append_string("@INF = 999999\n");
    ds.appendf("@TD = %d\n", depth);
    ds.appendf("@N = %d\n", n);

    // Min helper
    ds.append_string("@min = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: b; \xce\xbbn. a}(a < b)\n");

    // Tree operations
    emit_tree_ops(&ds);

    // Per-node update: @upd(node_idx, labels) = min(self_label, neighbor_labels...)
    ds.append_string("@upd = \xce\xbb{");
    for (uint i = 0; i < n; i++) {
        usz deg = g.adj[i].len();
        ds.appendf("%d: \xce\xbb&labels. ", i);

        if (deg == 0) {
            // Isolated node: keep own label
            ds.appendf("@tg(@TD, %d, labels)", i);
        } else {
            // @min(self, @min(n0, @min(n1, ... nk)))
            ds.append_string("@min(");
            ds.appendf("@tg(@TD, %d, labels), ", i);
            for (usz k = 0; k + 1 < deg; k++) {
                Edge e = g.adj[i][k];
                ds.append_string("@min(");
                ds.appendf("@tg(@TD, %d, labels), ", e.to);
            }
            Edge elast = g.adj[i][deg - 1];
            ds.appendf("@tg(@TD, %d, labels)", elast.to);
            for (usz k = 0; k < deg; k++) ds.append_string(")");
        }

        ds.append_string("; ");
    }
    ds.append_string("\xce\xbbn. \xce\xbblabels. n}\n");

    // One round of label propagation
    ds.append_string("@round = \xce\xbb&labels. @bld(@TD, 0, 1, @N, \xce\xbb&i. @upd(i, labels))\n");

    // Repeated rounds
    ds.append_string("@repeat = \xce\xbb{0: \xce\xbbf. \xce\xbbx. x; \xce\xbbn. ! &d = n; \xce\xbb&f. \xce\xbbx. @repeat(d - 1, f, f(x))}\n");

    // Initial labels: each node = its own index
    ds.append_string("@init = @bld(@TD, 0, 1, @N, \xce\xbbi. i)\n");

    // Run label propagation
    ds.appendf("@res = @repeat(%d, @round, @init)\n", rounds);

    // Extract results as flat list
    ds.append_string("@main = [");
    for (uint i = 0; i < n; i++) {
        if (i > 0) ds.append_string(", ");
        ds.appendf("@tg(@TD, %d, @res)", i);
    }
    ds.append_string("]\n");

    // Run HVM4
    uint[] out_buf = mem::new_array(uint, n);
    int count = hvm4_run(ds.zstr_view(), 0, out_buf.ptr, (int)n);

    if (count < 0) {
        free(out_buf.ptr);
        return HVM_ERROR~;
    }

    return out_buf;
}

