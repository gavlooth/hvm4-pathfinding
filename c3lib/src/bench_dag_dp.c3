module bench_dag_dp;
import bench_ffi;
import bench_common;
import bench_graph_gen;
import bench_reference;
import bench_hvm4_src;
import std::io;
import std::core::dstring;

fn bool run(uint v, uint epn) {
    CsrGraph g = bench_graph_gen::gen_dag(v, epn, 42 + v);

    uint src = 0;
    uint dest = v - 1;
    uint ref = bench_reference::dag_dp_reference(v, g.row_ptr, g.col_idx, g.weight, src, dest);

    dstring::DString hvm_src = bench_hvm4_src::gen_dag_dp_source(v, g.row_ptr, g.col_idx, g.weight, src, dest);

    bench_ffi::hvm4_lib_reset();

    uint result;
    int count = bench_ffi::hvm4_run(hvm_src.zstr_view(), 0, &result, 1);

    hvm_src.free();
    g.destroy();

    return count >= 1 && result == ref;
}
