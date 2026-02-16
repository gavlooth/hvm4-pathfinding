module bench_apsp;
import bench_ffi;
import bench_common;
import bench_graph_gen;
import bench_reference;
import bench_hvm4_src;
import std::io;
import std::core::dstring;

fn bool run(uint v) {
    DenseResult dr = bench_graph_gen::gen_dense(v, 42 + v);

    uint src = 0;
    uint dest = v - 1;
    uint* ref_dist = mem::new_array(uint, (usz)v * v).ptr;
    bench_reference::floyd_warshall(v, dr.adj, ref_dist);
    uint ref = ref_dist[src * v + dest];

    dstring::DString hvm_src = bench_hvm4_src::gen_apsp_source(v, src, dest);

    bench_ffi::hvm4_lib_reset();
    bench_ffi::hvm4_graph_setup(dr.csr.row_ptr, dr.csr.col_idx, dr.csr.weight, v);

    uint result;
    int count = bench_ffi::hvm4_run(hvm_src.zstr_view(), 0, &result, 1);

    hvm_src.free();
    free(ref_dist);
    free(dr.adj);
    dr.csr.destroy();

    return count >= 1 && result == ref;
}
