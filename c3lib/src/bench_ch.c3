module bench_ch;
import bench_ffi;
import bench_common;
import bench_graph_gen;
import bench_reference;
import bench_hvm4_src;
import std::io;
import std::core::dstring;

// ---------------------------------------------------------------------------
// Min-heap priority queue (mode-parameterized)
// ---------------------------------------------------------------------------

struct PqEntry {
    uint dist;
    uint node;
}

PqEntry*[2] g_pq;
uint[2] g_pq_size;
uint[2] g_pq_cap;

fn void ch_pq_swap(uint m, uint i, uint j) {
    PqEntry tmp = g_pq[m][i];
    g_pq[m][i] = g_pq[m][j];
    g_pq[m][j] = tmp;
}

fn void ch_pq_siftup(uint m, uint i) {
    while (i > 0) {
        uint p = (i - 1) / 2;
        if (g_pq[m][i].dist >= g_pq[m][p].dist) break;
        ch_pq_swap(m, i, p);
        i = p;
    }
}

fn void ch_pq_siftdown(uint m, uint i) {
    uint n = g_pq_size[m];
    for (;;) {
        uint s = i;
        uint l = 2 * i + 1;
        uint r = 2 * i + 2;
        if (l < n && g_pq[m][l].dist < g_pq[m][s].dist) s = l;
        if (r < n && g_pq[m][r].dist < g_pq[m][s].dist) s = r;
        if (s == i) break;
        ch_pq_swap(m, i, s);
        i = s;
    }
}

fn void ch_pq_push(uint m, uint dist, uint node) {
    if (g_pq_size[m] >= g_pq_cap[m]) {
        g_pq_cap[m] = g_pq_cap[m] != 0 ? g_pq_cap[m] * 2 : 256;
        g_pq[m] = (PqEntry*)realloc(g_pq[m], (usz)g_pq_cap[m] * PqEntry.sizeof);
    }
    g_pq[m][g_pq_size[m]].dist = dist;
    g_pq[m][g_pq_size[m]].node = node;
    ch_pq_siftup(m, g_pq_size[m]);
    g_pq_size[m]++;
}

fn bool ch_pq_pop(uint m, PqEntry* out) {
    if (g_pq_size[m] == 0) return false;
    *out = g_pq[m][0];
    g_pq_size[m]--;
    if (g_pq_size[m] > 0) {
        g_pq[m][0] = g_pq[m][g_pq_size[m]];
        ch_pq_siftdown(m, 0);
    }
    return true;
}

// ---------------------------------------------------------------------------
// Module-level state for FFI callbacks
// ---------------------------------------------------------------------------

uint*[2] g_ch_rp;
uint*[2] g_ch_ci;
uint*[2] g_ch_wt;
uint g_ch_v;
uint*[2] g_ch_dist;

// %ch_deg(mode, v)
fn bench_ffi::Term prim_ch_deg(bench_ffi::Term* args) {
    uint m = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[0]));
    uint v = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[1]));
    if (v >= g_ch_v || m > 1) return bench_ffi::hvm4_term_new_num(0);
    return bench_ffi::hvm4_term_new_num(g_ch_rp[m][v + 1] - g_ch_rp[m][v]);
}

// %ch_target(mode, v, i)
fn bench_ffi::Term prim_ch_target(bench_ffi::Term* args) {
    uint m = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[0]));
    uint v = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[1]));
    uint i = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[2]));
    uint edge = g_ch_rp[m][v] + i;
    return bench_ffi::hvm4_term_new_num(g_ch_ci[m][edge]);
}

// %ch_weight(mode, v, i)
fn bench_ffi::Term prim_ch_weight(bench_ffi::Term* args) {
    uint m = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[0]));
    uint v = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[1]));
    uint i = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[2]));
    uint edge = g_ch_rp[m][v] + i;
    return bench_ffi::hvm4_term_new_num(g_ch_wt[m][edge]);
}

// %ch_get(mode, v)
fn bench_ffi::Term prim_ch_get(bench_ffi::Term* args) {
    uint m = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[0]));
    uint v = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[1]));
    if (v >= g_ch_v || m > 1) return bench_ffi::hvm4_term_new_num(bench_common::INF);
    return bench_ffi::hvm4_term_new_num(g_ch_dist[m][v]);
}

// %ch_next(mode, dummy)
fn bench_ffi::Term prim_ch_next(bench_ffi::Term* args) {
    uint m = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[0]));
    bench_ffi::hvm4_wnf(args[1]);
    PqEntry e;
    while (ch_pq_pop(m, &e)) {
        if (e.dist == g_ch_dist[m][e.node]) {
            return bench_ffi::hvm4_term_new_num(e.node);
        }
    }
    return bench_ffi::hvm4_term_new_num(g_ch_v);
}

// %ch_relax(mode, u, nd)
fn bench_ffi::Term prim_ch_relax(bench_ffi::Term* args) {
    uint m = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[0]));
    uint u = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[1]));
    uint nd = bench_ffi::hvm4_term_val(bench_ffi::hvm4_wnf(args[2]));
    if (u < g_ch_v && m <= 1 && nd < g_ch_dist[m][u]) {
        g_ch_dist[m][u] = nd;
        ch_pq_push(m, nd, u);
        return bench_ffi::hvm4_term_new_num(1);
    }
    return bench_ffi::hvm4_term_new_num(0);
}

// %ch_meet(dummy)
fn bench_ffi::Term prim_ch_meet(bench_ffi::Term* args) {
    bench_ffi::hvm4_wnf(args[0]);
    uint best = bench_common::INF;
    for (uint v = 0; v < g_ch_v; v++) {
        uint sum = g_ch_dist[0][v] + g_ch_dist[1][v];
        if (sum < best) best = sum;
    }
    return bench_ffi::hvm4_term_new_num(best);
}

fn void ch_setup(uint v, uint src, uint dest,
                 uint* fwd_rp, uint* fwd_ci, uint* fwd_wt,
                 uint* bwd_rp, uint* bwd_ci, uint* bwd_wt) {
    g_ch_v = v;
    g_ch_rp[0] = fwd_rp; g_ch_ci[0] = fwd_ci; g_ch_wt[0] = fwd_wt;
    g_ch_rp[1] = bwd_rp; g_ch_ci[1] = bwd_ci; g_ch_wt[1] = bwd_wt;

    for (uint m = 0; m < 2; m++) {
        g_ch_dist[m] = mem::new_array(uint, v).ptr;
        for (uint i = 0; i < v; i++) g_ch_dist[m][i] = bench_common::INF;
        g_pq[m] = null;
        g_pq_size[m] = 0;
        g_pq_cap[m] = 0;
    }

    g_ch_dist[0][src] = 0;
    ch_pq_push(0, 0, src);
    g_ch_dist[1][dest] = 0;
    ch_pq_push(1, 0, dest);

    bench_ffi::hvm4_prim_register_ext("ch_deg", 6, 2, &prim_ch_deg);
    bench_ffi::hvm4_prim_register_ext("ch_target", 9, 3, &prim_ch_target);
    bench_ffi::hvm4_prim_register_ext("ch_weight", 9, 3, &prim_ch_weight);
    bench_ffi::hvm4_prim_register_ext("ch_get", 6, 2, &prim_ch_get);
    bench_ffi::hvm4_prim_register_ext("ch_next", 7, 2, &prim_ch_next);
    bench_ffi::hvm4_prim_register_ext("ch_relax", 8, 3, &prim_ch_relax);
    bench_ffi::hvm4_prim_register_ext("ch_meet", 7, 1, &prim_ch_meet);
}

fn void ch_cleanup() {
    for (uint m = 0; m < 2; m++) {
        free(g_ch_dist[m]);
        free(g_pq[m]);
    }
}

// ---------------------------------------------------------------------------
// Run benchmark
// ---------------------------------------------------------------------------

fn bool run(uint v, uint epn) {
    CsrGraph fwd = bench_graph_gen::gen_dag(v, epn, 42 + v);
    CsrGraph bwd = bench_graph_gen::reverse_csr(&fwd);

    // Reference: BF on forward graph, check dist[src->dest]
    uint src = 0;
    uint dest = v - 1;
    uint* ref = mem::new_array(uint, v).ptr;
    bench_reference::bf_reference(v, fwd.row_ptr, fwd.col_idx, fwd.weight, src, ref);
    uint ref_dist = ref[dest];

    dstring::DString hvm_src = bench_hvm4_src::gen_ch_source(v);

    bench_ffi::hvm4_lib_reset();
    ch_setup(v, src, dest,
             fwd.row_ptr, fwd.col_idx, fwd.weight,
             bwd.row_ptr, bwd.col_idx, bwd.weight);

    uint result;
    int count = bench_ffi::hvm4_run(hvm_src.zstr_view(), 0, &result, 1);

    bool ok = count >= 1 && result == ref_dist;

    ch_cleanup();
    hvm_src.free();
    free(ref);
    bwd.destroy();
    fwd.destroy();

    return ok;
}
