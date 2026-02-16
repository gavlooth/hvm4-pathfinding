module test_main;
import pathfind;
import swr_test;
import bench_test;
import std::io;

fn void print_dist(uint[] d) {
    io::printf("[");
    for (usz i = 0; i < d.len; i++) {
        if (i > 0) io::printf(",");
        io::printf("%d", d[i]);
    }
    io::printn("]");
}

fn void print_matrix(uint[] m, uint n) {
    io::printf("[");
    for (uint i = 0; i < n; i++) {
        if (i > 0) io::printf(",");
        io::printf("[");
        for (uint j = 0; j < n; j++) {
            if (j > 0) io::printf(",");
            io::printf("%d", m[i * n + j]);
        }
        io::printf("]");
    }
    io::printn("]");
}

fn void main() {
    int pass = 0;
    int fail = 0;

    // Initialize HVM4 runtime
    pathfind::runtime_init();
    defer pathfind::runtime_cleanup();

    // === 1. Algebraic APSP ===
    {
        pathfind::Graph g;
        g.init(3);
        defer g.destroy();
        g.add_biedge(0, 1, 2);
        g.add_biedge(1, 2, 3);
        g.add_biedge(0, 2, 8);

        uint[] mat = pathfind::algebraic_apsp(&g)!!;
        defer free(mat.ptr);

        // Expected: [[0,2,5],[2,0,3],[5,3,0]]
        bool ok = mat[0]==0 && mat[1]==2 && mat[2]==5
               && mat[3]==2 && mat[4]==0 && mat[5]==3
               && mat[6]==5 && mat[7]==3 && mat[8]==0;
        if (ok) {
            io::printn("PASS  algebraic_apsp"); pass++;
        } else {
            io::printn("FAIL  algebraic_apsp");
            io::printf("  got: "); print_matrix(mat, 3); fail++;
        }
    }

    // === 2. Path Enumeration ===
    // Note: eval_collapse returns SUP branches in nondeterministic order
    // when multi-threaded, so we sort before comparing.
    {
        pathfind::Graph g;
        g.init(6);
        defer g.destroy();
        g.add_edge(0, 1, 2); g.add_edge(0, 2, 3);
        g.add_edge(1, 3, 1); g.add_edge(1, 4, 4);
        g.add_edge(2, 3, 5); g.add_edge(2, 4, 2);
        g.add_edge(3, 5, 3);
        g.add_edge(4, 5, 1);

        pathfind::PathResult pr = pathfind::enumerate_paths(&g, 0, 5)!!;
        defer free(pr.weights.ptr);

        uint[4] expect = { 6, 6, 7, 11 };
        // Sort results for order-independent comparison
        uint[] got = pr.weights;
        for (usz i = 1; i < got.len; i++) {
            uint key = got[i];
            isz j = (isz)i - 1;
            while (j >= 0 && got[j] > key) {
                got[j + 1] = got[j];
                j--;
            }
            got[j + 1] = key;
        }
        bool ok = pr.count == 4;
        if (ok) {
            for (usz i = 0; i < 4; i++) {
                if (got[i] != expect[i]) { ok = false; break; }
            }
        }
        if (ok) {
            io::printn("PASS  enumerate_paths"); pass++;
        } else {
            io::printfn("FAIL  enumerate_paths (count=%d)", (int)pr.count);
            if (pr.count > 0) {
                io::printf("  got: "); print_dist(pr.weights);
            }
            fail++;
        }
    }

    // === 3. DAG-DP ===
    {
        // DAG: 0->1 (w=2), 0->2 (w=4), 1->3 (w=3), 2->3 (w=1)
        // Shortest path 0->3: 0->1->3 = 2+3 = 5, or 0->2->3 = 4+1 = 5
        pathfind::Graph g;
        g.init(4);
        defer g.destroy();
        g.add_edge(0, 1, 2);
        g.add_edge(0, 2, 4);
        g.add_edge(1, 3, 3);
        g.add_edge(2, 3, 1);

        uint dist = pathfind::dag_dp(&g, 0, 3)!!;

        if (dist == 5) {
            io::printn("PASS  dag_dp"); pass++;
        } else {
            io::printfn("FAIL  dag_dp (got %d, expected 5)", dist); fail++;
        }
    }

    // === 4. Connected Components ===
    {
        // Graph with 6 nodes, 2 components:
        //   Component A: 0-1-2 (bidirectional)
        //   Component B: 3-4-5 (bidirectional)
        pathfind::Graph g;
        g.init(6);
        defer g.destroy();
        g.add_biedge(0, 1, 1);
        g.add_biedge(1, 2, 1);
        g.add_biedge(3, 4, 1);
        g.add_biedge(4, 5, 1);

        uint[] labels = pathfind::connected_components(&g)!!;
        defer free(labels.ptr);

        // Component A: nodes 0,1,2 should all have label 0
        // Component B: nodes 3,4,5 should all have label 3
        bool ok = labels[0] == 0 && labels[1] == 0 && labels[2] == 0
               && labels[3] == 3 && labels[4] == 3 && labels[5] == 3;
        if (ok) {
            io::printn("PASS  connected_components"); pass++;
        } else {
            io::printf("FAIL  connected_components  got: ");
            print_dist(labels); fail++;
        }
    }

    // === 5. GeoGraph KNN ===
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
