module bench_bidir_bfs;
import bench_ffi;
import bench_common;
import bench_graph_gen;
import bench_reference;
import bench_hvm4_src;
import std::io;
import std::core::dstring;

fn bool run(uint v, uint epn) {
    CsrGraph g = bench_graph_gen::gen_undirected(v, epn, 42 + v);

    uint src = 0;
    uint dest = v - 1;
    uint ref = bench_reference::bfs_reference(v, g.row_ptr, g.col_idx, src, dest);

    // graph_weight not used for BFS but bridge requires it
    uint* wt = mem::new_array(uint, g.e).ptr;
    for (uint i = 0; i < g.e; i++) wt[i] = 1;

    dstring::DString hvm_src = bench_hvm4_src::gen_bidir_bfs_source(v, src, dest);

    bench_ffi::hvm4_lib_reset();
    bench_ffi::hvm4_graph_setup(g.row_ptr, g.col_idx, wt, v);

    uint result;
    int count = bench_ffi::hvm4_run(hvm_src.zstr_view(), 0, &result, 1);

    hvm_src.free();
    free(wt);
    g.destroy();

    return count >= 1 && result == ref;
}
