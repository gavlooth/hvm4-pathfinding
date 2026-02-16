module bench_closure;
import bench_ffi;
import bench_common;
import bench_graph_gen;
import bench_reference;
import bench_hvm4_src;
import std::io;
import std::core::dstring;

fn bool run(uint v) {
    BoolDenseResult dr = bench_graph_gen::gen_dense_bool(v, 42 + v);

    uint src = 0;
    uint dest = v - 1;
    uint* ref_reach = mem::new_array(uint, (usz)v * v).ptr;
    bench_reference::warshall(v, dr.adj, ref_reach);
    uint ref = ref_reach[src * v + dest];

    dstring::DString hvm_src = bench_hvm4_src::gen_closure_source(v, src, dest);

    bench_ffi::hvm4_lib_reset();
    bench_ffi::hvm4_graph_setup(dr.csr.row_ptr, dr.csr.col_idx, dr.csr.weight, v);

    uint result;
    int count = bench_ffi::hvm4_run(hvm_src.zstr_view(), 0, &result, 1);

    hvm_src.free();
    free(ref_reach);
    free(dr.adj);
    dr.csr.destroy();

    return count >= 1 && result == ref;
}
