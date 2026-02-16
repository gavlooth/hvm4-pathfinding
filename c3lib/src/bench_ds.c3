module bench_ds;
import bench_ffi;
import bench_common;
import bench_graph_gen;
import bench_reference;
import bench_hvm4_src;
import std::io;
import std::core::dstring;

// ---------------------------------------------------------------------------
// Min-heap priority queue
// ---------------------------------------------------------------------------

struct PqEntry {
    uint dist;
    uint node;
}

PqEntry* g_pq;
uint g_pq_size;
uint g_pq_cap;

fn void pq_swap(uint i, uint j) {
    PqEntry tmp = g_pq[i];
    g_pq[i] = g_pq[j];
    g_pq[j] = tmp;
}

fn void pq_siftup(uint i) {
    while (i > 0) {
        uint p = (i - 1) / 2;
        if (g_pq[i].dist >= g_pq[p].dist) break;
        pq_swap(i, p);
        i = p;
    }
}

fn void pq_siftdown(uint i) {
    for (;;) {
        uint s = i;
        uint l = 2 * i + 1;
        uint r = 2 * i + 2;
        if (l < g_pq_size && g_pq[l].dist < g_pq[s].dist) s = l;
        if (r < g_pq_size && g_pq[r].dist < g_pq[s].dist) s = r;
        if (s == i) break;
        pq_swap(i, s);
        i = s;
    }
}

fn void pq_push(uint dist, uint node) {
    if (g_pq_size >= g_pq_cap) {
        g_pq_cap = g_pq_cap != 0 ? g_pq_cap * 2 : 256;
        g_pq = (PqEntry*)realloc(g_pq, (usz)g_pq_cap * PqEntry.sizeof);
    }
    g_pq[g_pq_size].dist = dist;
    g_pq[g_pq_size].node = node;
    pq_siftup(g_pq_size);
    g_pq_size++;
}

fn bool pq_pop(PqEntry* out) {
    if (g_pq_size == 0) return false;
    *out = g_pq[0];
    g_pq_size--;
    if (g_pq_size > 0) {
        g_pq[0] = g_pq[g_pq_size];
        pq_siftdown(0);
    }
    return true;
}

// ---------------------------------------------------------------------------
// Module-level state for FFI callbacks
// ---------------------------------------------------------------------------

uint* g_dist;
uint g_dist_v;

// %dist_get(v) -> distance of node v
fn bench_ffi::Term prim_dist_get(bench_ffi::Term* args) {
    bench_ffi::Term v = bench_ffi::hvm4_wnf(args[0]);
    uint node = bench_ffi::hvm4_term_val(v);
    return bench_ffi::hvm4_term_new_num(node < g_dist_v ? g_dist[node] : bench_common::INF);
}

// %ds_next(dummy) -> pop min from PQ (skip stale), return V if empty
fn bench_ffi::Term prim_ds_next(bench_ffi::Term* args) {
    bench_ffi::hvm4_wnf(args[0]);
    PqEntry e;
    while (pq_pop(&e)) {
        if (e.dist == g_dist[e.node]) {
            return bench_ffi::hvm4_term_new_num(e.node);
        }
    }
    return bench_ffi::hvm4_term_new_num(g_dist_v);
}

// %ds_relax(u, nd) -> if nd < dist[u], update + push to PQ, return 1 else 0
fn bench_ffi::Term prim_ds_relax(bench_ffi::Term* args) {
    bench_ffi::Term u_term = bench_ffi::hvm4_wnf(args[0]);
    bench_ffi::Term nd_term = bench_ffi::hvm4_wnf(args[1]);
    uint u = bench_ffi::hvm4_term_val(u_term);
    uint nd = bench_ffi::hvm4_term_val(nd_term);
    if (u < g_dist_v && nd < g_dist[u]) {
        g_dist[u] = nd;
        pq_push(nd, u);
        return bench_ffi::hvm4_term_new_num(1);
    }
    return bench_ffi::hvm4_term_new_num(0);
}

fn void ds_setup(uint* dist, uint v) {
    g_dist = dist;
    g_dist_v = v;
    g_pq = null;
    g_pq_size = 0;
    g_pq_cap = 0;

    // Seed source node
    dist[0] = 0;
    pq_push(0, 0);

    bench_ffi::hvm4_prim_register_ext("dist_get", 8, 1, &prim_dist_get);
    bench_ffi::hvm4_prim_register_ext("ds_next", 7, 1, &prim_ds_next);
    bench_ffi::hvm4_prim_register_ext("ds_relax", 8, 2, &prim_ds_relax);
}

// ---------------------------------------------------------------------------
// Run benchmark
// ---------------------------------------------------------------------------

fn bool run(uint v, uint epn) {
    CsrGraph g = bench_graph_gen::gen_random(v, epn, 42 + v);

    // C reference
    uint* ref = mem::new_array(uint, v).ptr;
    bench_reference::bf_reference(v, g.row_ptr, g.col_idx, g.weight, 0, ref);

    uint dest = v - 1;
    dstring::DString hvm_src = bench_hvm4_src::gen_ds_source(v, dest);

    bench_ffi::hvm4_lib_reset();
    bench_ffi::hvm4_graph_setup(g.row_ptr, g.col_idx, g.weight, v);

    // Initialize C-side distance array
    uint* dist = mem::new_array(uint, v).ptr;
    for (uint i = 0; i < v; i++) dist[i] = bench_common::INF;
    ds_setup(dist, v);

    // Run HVM4
    uint result;
    int count = bench_ffi::hvm4_run(hvm_src.zstr_view(), 0, &result, 1);

    bool ok = count >= 1 && result == ref[dest];

    hvm_src.free();
    free(dist);
    free(ref);
    free(g_pq);
    g.destroy();

    return ok;
}
