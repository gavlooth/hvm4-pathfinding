module bench_test;
import bench_apsp;
import bench_closure;
import bench_path_enum;
import bench_bidir_bfs;
import bench_dag_dp;
import bench_graph_gen;
import std::io;

fn void run_bench_tests(int* pass, int* fail) {
    io::printn("\n=== Bench Tests ===");

    // APSP: V=10 (O(V^3 log V), keep small)
    if (bench_apsp::run(10)) {
        io::printn("PASS  bench_apsp"); (*pass)++;
    } else {
        io::printn("FAIL  bench_apsp"); (*fail)++;
    }

    // Closure: V=10
    if (bench_closure::run(10)) {
        io::printn("PASS  bench_closure"); (*pass)++;
    } else {
        io::printn("FAIL  bench_closure"); (*fail)++;
    }

    // Path Enumeration: V=100, K=8
    if (bench_path_enum::run(100, 8)) {
        io::printn("PASS  bench_path_enum"); (*pass)++;
    } else {
        io::printn("FAIL  bench_path_enum"); (*fail)++;
    }

    // Bidirectional BFS: V=50, 4 edges/node
    if (bench_bidir_bfs::run(50, 4)) {
        io::printn("PASS  bench_bidir_bfs"); (*pass)++;
    } else {
        io::printn("FAIL  bench_bidir_bfs"); (*fail)++;
    }

    // DAG-DP: V=50, 4 edges/node
    if (bench_dag_dp::run(50, 4)) {
        io::printn("PASS  bench_dag_dp"); (*pass)++;
    } else {
        io::printn("FAIL  bench_dag_dp"); (*fail)++;
    }

    // Geo Grid: 5x5 grid over Europe
    {
        bench_graph_gen::GeoGridResult gg = bench_graph_gen::gen_geo_grid(
            5, 5, 48.0, 2.0, 52.0, 10.0);
        defer {
            free(gg.csr.row_ptr);
            free(gg.csr.col_idx);
            if (gg.csr.weight != null) free(gg.csr.weight);
            free(gg.coords);
        }
        bool ok = gg.csr.v == 25 && gg.csr.e > 0;
        if (ok) {
            io::printn("PASS  bench_geo_grid"); (*pass)++;
        } else {
            io::printn("FAIL  bench_geo_grid"); (*fail)++;
        }
    }

}
