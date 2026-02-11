module test_main;
import pathfind;
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

    // === 1. Bellman-Ford ===
    {
        pathfind::Graph g;
        g.init(5);
        defer g.destroy();
        g.add_edge(0, 1, 4);
        g.add_edge(0, 2, 2);
        g.add_edge(1, 3, 3);
        g.add_edge(2, 1, 1);
        g.add_edge(2, 3, 5);
        g.add_edge(3, 4, 1);

        uint[] dist = pathfind::bellman_ford(&g, 0)!!;
        defer free(dist.ptr);

        if (dist[0]==0 && dist[1]==3 && dist[2]==2 && dist[3]==6 && dist[4]==7) {
            io::printn("PASS  bellman_ford"); pass++;
        } else {
            io::printn("FAIL  bellman_ford");
            io::printf("  got: "); print_dist(dist); fail++;
        }
    }

    // === 2. Delta-Stepping ===
    {
        pathfind::Graph g;
        g.init(5);
        defer g.destroy();
        g.add_edge(0, 1, 1);
        g.add_edge(0, 2, 5);
        g.add_edge(1, 2, 2);
        g.add_edge(1, 3, 6);
        g.add_edge(2, 3, 2);
        g.add_edge(2, 4, 1);
        g.add_edge(3, 4, 3);

        uint[] dist = pathfind::delta_stepping(&g, 0, 3)!!;
        defer free(dist.ptr);

        if (dist[0]==0 && dist[1]==1 && dist[2]==3 && dist[3]==5 && dist[4]==4) {
            io::printn("PASS  delta_stepping"); pass++;
        } else {
            io::printn("FAIL  delta_stepping");
            io::printf("  got: "); print_dist(dist); fail++;
        }
    }

    // === 3. Bidirectional BFS ===
    {
        pathfind::Graph g;
        g.init(7);
        defer g.destroy();
        g.add_biedge(0, 1, 1);
        g.add_biedge(0, 3, 1);
        g.add_biedge(1, 2, 1);
        g.add_biedge(2, 3, 1);
        g.add_biedge(3, 4, 1);
        g.add_biedge(3, 6, 1);
        g.add_biedge(4, 5, 1);
        g.add_biedge(5, 6, 1);

        uint d = pathfind::bidir_bfs(&g, 0, 6)!!;
        if (d == 2) {
            io::printn("PASS  bidir_bfs"); pass++;
        } else {
            io::printfn("FAIL  bidir_bfs (got %d, expected 2)", d); fail++;
        }
    }

    // === 4. Contraction Hierarchy ===
    {
        pathfind::Graph fwd;
        fwd.init(6);
        defer fwd.destroy();
        fwd.add_edge(0, 1, 2); fwd.add_edge(0, 2, 6);
        fwd.add_edge(1, 2, 3); fwd.add_edge(1, 3, 5);
        fwd.add_edge(2, 4, 4);
        fwd.add_edge(3, 4, 2); fwd.add_edge(3, 5, 8);
        fwd.add_edge(4, 5, 1);

        pathfind::Graph bwd;
        bwd.init(6);
        defer bwd.destroy();
        bwd.add_edge(5, 4, 1); bwd.add_edge(5, 3, 8);
        bwd.add_edge(4, 2, 4); bwd.add_edge(4, 3, 2);
        bwd.add_edge(3, 1, 5);
        bwd.add_edge(2, 0, 6); bwd.add_edge(2, 1, 3);
        bwd.add_edge(1, 0, 2);

        uint d = pathfind::contraction_query(&fwd, &bwd, 0, 5, 6)!!;
        if (d == 10) {
            io::printn("PASS  contraction_hierarchy"); pass++;
        } else {
            io::printfn("FAIL  contraction_hierarchy (got %d, expected 10)", d); fail++;
        }
    }

    // === 5. Algebraic APSP ===
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

    // === 6. Path Enumeration ===
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

        uint[4] expect = { 6, 7, 11, 6 };
        bool ok = pr.count == 4;
        if (ok) {
            for (usz i = 0; i < 4; i++) {
                if (pr.weights[i] != expect[i]) { ok = false; break; }
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

    io::printfn("\nResults: %d passed, %d failed", pass, fail);
}
