module bench_bf;
import bench_ffi;
import bench_common;
import bench_graph_gen;
import bench_reference;
import bench_hvm4_src;
import std::io;
import std::core::dstring;

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

// %dist_upd(v, d) -> min-update: if d < dist[v], set and return 1, else 0
fn bench_ffi::Term prim_dist_upd(bench_ffi::Term* args) {
    bench_ffi::Term v = bench_ffi::hvm4_wnf(args[0]);
    bench_ffi::Term d = bench_ffi::hvm4_wnf(args[1]);
    uint node = bench_ffi::hvm4_term_val(v);
    uint val = bench_ffi::hvm4_term_val(d);
    if (node < g_dist_v && val < g_dist[node]) {
        g_dist[node] = val;
        return bench_ffi::hvm4_term_new_num(1);
    }
    return bench_ffi::hvm4_term_new_num(0);
}

fn void dist_setup(uint* dist, uint v) {
    g_dist = dist;
    g_dist_v = v;
    bench_ffi::hvm4_prim_register_ext("dist_get", 8, 1, &prim_dist_get);
    bench_ffi::hvm4_prim_register_ext("dist_upd", 8, 2, &prim_dist_upd);
}

// ---------------------------------------------------------------------------
// Run benchmark
// ---------------------------------------------------------------------------

fn bool run(uint v, uint epn) {
    CsrGraph g = bench_graph_gen::gen_random(v, epn, 42 + v);

    // C reference
    uint* ref = mem::new_array(uint, v).ptr;
    bench_reference::bf_reference(v, g.row_ptr, g.col_idx, g.weight, 0, ref);

    // Generate HVM4 source
    uint dest = v - 1;
    dstring::DString hvm_src = bench_hvm4_src::gen_bf_source(v, dest);

    bench_ffi::hvm4_lib_reset();
    bench_ffi::hvm4_graph_setup(g.row_ptr, g.col_idx, g.weight, v);

    // Initialize C-side distance array
    uint* dist = mem::new_array(uint, v).ptr;
    for (uint i = 0; i < v; i++) dist[i] = bench_common::INF;
    dist[0] = 0;
    dist_setup(dist, v);

    // Run HVM4
    uint result;
    int count = bench_ffi::hvm4_run(hvm_src.zstr_view(), 0, &result, 1);

    // Validate all distances
    bool ok = count >= 1;
    if (ok) {
        for (uint i = 0; i < v; i++) {
            if (dist[i] != ref[i]) {
                ok = false;
                break;
            }
        }
    }

    hvm_src.free();
    free(dist);
    free(ref);
    g.destroy();

    return ok;
}
