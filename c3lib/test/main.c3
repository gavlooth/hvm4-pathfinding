module test_main;
import pathfind;
import swr_test;
import bench_test;
import std::io;

fn void main() {
    int pass = 0;
    int fail = 0;

    // === 1. GeoGraph KNN ===
    {
        pathfind::GeoGraph geo;
        geo.init(5);
        defer geo.destroy();

        // 5 European cities
        geo.add_node(51.5074, -0.1278);  // London
        geo.add_node(48.8566, 2.3522);   // Paris
        geo.add_node(50.8503, 4.3517);   // Brussels
        geo.add_node(52.3676, 4.9041);   // Amsterdam
        geo.add_node(48.2082, 16.3738);  // Vienna

        geo.connect_knn(2);

        // Every node should have at least one edge
        bool all_have_edges = true;
        for (uint i = 0; i < 5; i++) {
            if (geo.graph.adj[i].len() == 0) {
                all_have_edges = false;
                break;
            }
        }
        if (all_have_edges) {
            io::printn("PASS  geograph_knn"); pass++;
        } else {
            io::printn("FAIL  geograph_knn"); fail++;
        }
    }

    // Run SWR tests
    swr_test::run_swr_tests(&pass, &fail);

    // Run bench tests
    bench_test::run_bench_tests(&pass, &fail);

    io::printfn("\nResults: %d passed, %d failed", pass, fail);
}
