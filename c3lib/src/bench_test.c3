module bench_test;
import bench_graph_gen;
import std::io;

fn void run_bench_tests(int* pass, int* fail) {
    io::printn("\n=== Bench Tests ===");

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
