module bench_path_enum;
import bench_ffi;
import bench_common;
import bench_graph_gen;
import bench_reference;
import bench_hvm4_src;
import std::io;
import std::core::dstring;

fn bool run(uint v, uint k) {
    SpineResult sr = bench_graph_gen::gen_spine_dag(v, k);
    uint dest = v - 1;

    // C reference DFS
    int max_paths = 1 << (k < 20 ? k : 20);
    uint* ref_paths = mem::new_array(uint, (usz)max_paths).ptr;
    DfsState state;
    state.rp = sr.csr.row_ptr;
    state.ci = sr.csr.col_idx;
    state.wt = sr.csr.weight;
    state.paths = ref_paths;
    state.path_count = 0;
    state.path_max = max_paths;
    bench_reference::c_dfs(&state, 0, dest, 0);
    int ref_count = state.path_count;
    bench_common::sort_uint(ref_paths, ref_count < max_paths ? (usz)ref_count : (usz)max_paths);

    // Generate HVM4 source
    dstring::DString hvm_src = bench_hvm4_src::gen_path_enum_source(dest);

    bench_ffi::hvm4_lib_reset();
    bench_ffi::hvm4_graph_setup(sr.csr.row_ptr, sr.csr.col_idx, sr.csr.weight, v);

    // Run with collapse mode (lists → printed values)
    uint* hvm_results = mem::new_array(uint, (usz)max_paths).ptr;
    int hvm_count = bench_ffi::hvm4_run(hvm_src.zstr_view(), 0, hvm_results, max_paths);

    hvm_src.free();

    // Compare sorted results
    bool ok = hvm_count == ref_count;
    if (ok && hvm_count > 0) {
        int cmp_count = hvm_count < max_paths ? hvm_count : max_paths;
        bench_common::sort_uint(hvm_results, (usz)cmp_count);
        for (int i = 0; i < cmp_count; i++) {
            if (hvm_results[i] != ref_paths[i]) {
                ok = false;
                break;
            }
        }
    }

    free(hvm_results);
    free(ref_paths);
    free(sr.spine);
    sr.csr.destroy();

    return ok;
}
