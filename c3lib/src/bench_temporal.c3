module bench_temporal;
import bench_common;
import swr;
import swr_temporal;
import swr_alt;
import swr_cch;
import swr_hub;
import dijkstra;
import pathfind;
import std::io;
import std::math;

// Stress test for temporal routing — ALL algorithms.
//
// Builds a synthetic PRM with V spatial nodes and ~epn edges/node,
// creates a forecast with T time steps, constructs the time-expanded
// graph (V*T nodes), runs Dijkstra/ALT/CCH/Hub, validates agreement.
//
// Usage: run(V_spatial, T_steps, edges_per_node)
//   e.g. run(10000, 10, 4) → 100k temporal nodes

fn bool run(uint v, uint t_steps, uint epn) {
    if (v < 2 || t_steps < 2) return false;

    bench_common::Lcg rng = { .s = 42 };

    // ----------------------------------------------------------------
    // 1. Build synthetic PRM (bypass O(V²) KNN)
    // ----------------------------------------------------------------
    uint max_edges = v * epn + v;  // random + chain
    uint* e_from = mem::new_array(uint, max_edges).ptr;
    uint* e_to = mem::new_array(uint, max_edges).ptr;
    double* e_dist = mem::new_array(double, max_edges).ptr;
    double* e_head = mem::new_array(double, max_edges).ptr;
    uint* e_wt = mem::new_array(uint, max_edges).ptr;
    uint ne = 0;

    // Chain for connectivity
    for (uint i = 0; i + 1 < v && ne < max_edges; i++) {
        e_from[ne] = i;
        e_to[ne] = i + 1;
        e_dist[ne] = 18520.0;  // ~10 NM
        e_head[ne] = 90.0;
        e_wt[ne] = rng.next() % 10 + 1;
        ne++;
    }

    // Random edges
    uint attempts = v * (epn - 1) * 2;
    for (uint a = 0; a < attempts && ne < v * epn; a++) {
        uint u = rng.next() % v;
        uint vi = rng.next() % v;
        if (u == vi) continue;
        e_from[ne] = u;
        e_to[ne] = vi;
        e_dist[ne] = (double)(rng.next() % 50 + 5) * 1852.0;
        e_head[ne] = (double)(rng.next() % 360);
        e_wt[ne] = rng.next() % 20 + 1;
        ne++;
    }

    // Build adjacency list graph
    pathfind::Graph g;
    g.init(v);
    for (uint i = 0; i < ne; i++) {
        g.add_edge(e_from[i], e_to[i], e_wt[i]);
    }

    // Waypoints along a line
    swr::Waypoint* wps = mem::new_array(swr::Waypoint, v).ptr;
    for (uint i = 0; i < v; i++) {
        wps[i] = (swr::Waypoint){
            .id = i,
            .pos = {
                .lat = 37.0 + 2.0 * (double)i / (double)v,
                .lon = 23.0 + 4.0 * (double)i / (double)v
            }
        };
    }

    // Assemble SwrState
    bool[1] grid_data = { false };
    swr::SwrState state;
    state.land = (swr::LandMask){
        .bounds = { .sw = { .lat = 37.0, .lon = 23.0 }, .ne = { .lat = 39.0, .lon = 27.0 } },
        .rows = 1, .cols = 1, .grid = &grid_data
    };
    state.ship = (swr::ShipModel){
        .base_speed = 15.0, .max_wave_ht = 4.0,
        .fuel_rate_base = 100.0, .fuel_rate_per_kt = 5.0
    };
    state.cost_cfg = (swr::CostConfig){
        .mode = swr::CostMode.TIME_ONLY, .alpha = 1.0,
        .safety_penalty = 100.0, .scale = 1000
    };
    state.prm.n = v;
    state.prm.graph = g;
    state.prm.waypoints = wps[:v];
    state.prm.edge_count = ne;
    state.prm.edge_capacity = ne;
    state.prm.edge_from = e_from[:ne];
    state.prm.edge_to = e_to[:ne];
    state.prm.edge_dist_m = e_dist[:ne];
    state.prm.edge_heading = e_head[:ne];
    state.prm.edge_weight = e_wt[:ne];
    state.origin = 0;
    state.destination = v - 1;

    // ----------------------------------------------------------------
    // 2. Build forecast (varying weather across time steps)
    //    Mediterranean-realistic: calm→building→storm progression
    //    Each time step gets its OWN cells array (no shared pointer bug)
    // ----------------------------------------------------------------
    swr::WeatherGrid* grids = mem::new_array(swr::WeatherGrid, t_steps).ptr;
    swr::WeatherCell** all_cells = mem::new_array(swr::WeatherCell*, t_steps).ptr;

    for (uint t = 0; t < t_steps; t++) {
        swr::WeatherCell* cells = mem::new_array(swr::WeatherCell, 4).ptr;
        all_cells[t] = cells;
        double phase = (double)t / (double)t_steps;
        // Realistic Med Sea range: wind 5-25kt, waves 0.3-4m, current 0.2-1.5kt
        for (int i = 0; i < 4; i++) {
            cells[i] = (swr::WeatherCell){
                .wind_speed = 5.0 + 20.0 * phase,
                .wind_dir = 180.0 + 90.0 * phase,
                .wave_height = 0.3 + 3.7 * phase,
                .wave_dir = 200.0 + 30.0 * phase,
                .current_speed = 0.2 + 1.3 * phase,
                .current_dir = 90.0 + 45.0 * phase
            };
        }
        grids[t] = (swr::WeatherGrid){
            .bounds = { .sw = { .lat = 37.0, .lon = 23.0 }, .ne = { .lat = 39.0, .lon = 27.0 } },
            .rows = 2, .cols = 2,
            .cells = cells[:4],
            .timestamp = (double)t * 6.0
        };
    }

    swr::WeatherForecast forecast = {
        .grids = grids[:t_steps], .count = t_steps,
        .interval_hours = 6, .wait_cost = 100
    };

    uint total_nodes = v * t_steps;

    // ----------------------------------------------------------------
    // 3. Build temporal graph (timed)
    // ----------------------------------------------------------------
    io::printfn("  [temporal] V=%d T=%d epn=%d → %d temporal nodes, %d spatial edges",
        v, t_steps, epn, total_nodes, ne);

    bench_common::Timespec t0 = bench_common::time_now();
    pathfind::Graph tg = swr_temporal::build_temporal_graph(&state, &forecast);
    double build_ms = bench_common::time_elapsed_s(t0) * 1000.0;

    uint tg_edges = 0;
    for (uint i = 0; i < tg.n; i++) {
        tg_edges += (uint)tg.adj[i].len();
    }
    io::printfn("  [temporal] build: %.1f ms, %d nodes, %d edges", build_ms, tg.n, tg_edges);

    uint src = state.origin * t_steps;
    bool all_ok = true;

    // ----------------------------------------------------------------
    // 4a. Dijkstra (reference)
    // ----------------------------------------------------------------
    bench_common::Timespec t1 = bench_common::time_now();
    dijkstra::DijkResult dr = dijkstra::sssp(&tg, src)!!;
    double dijk_ms = bench_common::time_elapsed_s(t1) * 1000.0;

    uint ref_cost = pathfind::INF;
    uint best_t = 0;
    for (uint t = 0; t < t_steps; t++) {
        uint idx = state.destination * t_steps + t;
        if (dr.dist[idx] < ref_cost) {
            ref_cost = dr.dist[idx];
            best_t = t;
        }
    }
    io::printfn("  [dijkstra]  %.1f ms  cost=%d  t=%d", dijk_ms, ref_cost, best_t);

    // Extract path
    bool path_ok = false;
    if (ref_cost < pathfind::INF) {
        uint dst_temporal = state.destination * t_steps + best_t;
        uint[] temporal_path = dijkstra::extract_path(
            dr.pred, src, dst_temporal, total_nodes)!!;
        uint[] spatial_path = swr_temporal::extract_spatial_path(
            temporal_path, temporal_path.len, t_steps);

        path_ok = spatial_path.len >= 2
                && spatial_path[0] == state.origin
                && spatial_path[spatial_path.len - 1] == state.destination;

        io::printfn("  [dijkstra]  path: %d temporal hops → %d spatial nodes",
            temporal_path.len, spatial_path.len);

        free(temporal_path.ptr);
        free(spatial_path.ptr);
    }
    dr.destroy();
    if (!path_ok) all_ok = false;

    // ----------------------------------------------------------------
    // 4b. ALT (A* with landmarks)
    // ----------------------------------------------------------------
    bench_common::Timespec t_alt0 = bench_common::time_now();
    swr_alt::AltData alt = swr_alt::preprocess(&tg, 4);
    double alt_pre_ms = bench_common::time_elapsed_s(t_alt0) * 1000.0;

    bench_common::Timespec t_alt1 = bench_common::time_now();
    uint alt_cost = pathfind::INF;
    for (uint t = 0; t < t_steps; t++) {
        uint dst = state.destination * t_steps + t;
        swr::RouteResult ar = swr_alt::query(&alt, &tg, src, dst) ?? (swr::RouteResult){};
        if (ar.total_cost > 0 && ar.total_cost < alt_cost) alt_cost = ar.total_cost;
        ar.destroy();
    }
    double alt_q_ms = bench_common::time_elapsed_s(t_alt1) * 1000.0;
    alt.destroy();

    bool alt_match = alt_cost == ref_cost;
    io::printfn("  [ALT]       pre=%.1f ms  query=%.1f ms  cost=%d  %s",
        alt_pre_ms, alt_q_ms, alt_cost, alt_match ? "MATCH" : "MISMATCH");
    if (!alt_match) all_ok = false;

    // ----------------------------------------------------------------
    // 4c. CCH (Customizable Contraction Hierarchies)
    //     O(V²) preprocessing — skip above 5000 temporal nodes
    // ----------------------------------------------------------------
    double cch_pre_ms = 0;
    double cch_q_ms = 0;
    if (total_nodes <= 500) {
        pathfind::Graph tg_copy;
        tg_copy.init(tg.n);
        for (uint u = 0; u < tg.n; u++) {
            for (usz i = 0; i < tg.adj[u].len(); i++) {
                pathfind::Edge e = tg.adj[u][i];
                tg_copy.add_edge(u, e.to, e.weight);
            }
        }

        bench_common::Timespec t_cch0 = bench_common::time_now();
        swr_cch::CchData cch = swr_cch::build_topology(&tg_copy);
        tg_copy.destroy();
        swr_cch::customize(&cch, &tg);
        cch_pre_ms = bench_common::time_elapsed_s(t_cch0) * 1000.0;

        bench_common::Timespec t_cch1 = bench_common::time_now();
        uint cch_cost = pathfind::INF;
        for (uint t = 0; t < t_steps; t++) {
            uint dst = state.destination * t_steps + t;
            uint c = swr_cch::cch_query(&cch, src, dst);
            if (c < cch_cost) cch_cost = c;
        }
        cch_q_ms = bench_common::time_elapsed_s(t_cch1) * 1000.0;
        cch.destroy();

        bool cch_match = cch_cost == ref_cost;
        io::printfn("  [CCH]       pre=%.1f ms  query=%.1f ms  cost=%d  %s",
            cch_pre_ms, cch_q_ms, cch_cost, cch_match ? "MATCH" : "MISMATCH");
        if (!cch_match) all_ok = false;
    } else {
        io::printfn("  [CCH]       SKIPPED (V*T=%d > 500, O(V²) preprocessing)", total_nodes);
    }

    // ----------------------------------------------------------------
    // 4d. Hub Labeling
    //     O(V²) preprocessing — skip above 5000 temporal nodes
    // ----------------------------------------------------------------
    double hub_pre_ms = 0;
    double hub_q_ms = 0;
    if (total_nodes <= 500) {
        bench_common::Timespec t_hub0 = bench_common::time_now();
        swr_hub::HubData hub = swr_hub::preprocess(&tg);
        hub_pre_ms = bench_common::time_elapsed_s(t_hub0) * 1000.0;

        bench_common::Timespec t_hub1 = bench_common::time_now();
        uint hub_cost = pathfind::INF;
        for (uint t = 0; t < t_steps; t++) {
            uint dst = state.destination * t_steps + t;
            uint c = swr_hub::hub_query(&hub, src, dst);
            if (c < hub_cost) hub_cost = c;
        }
        hub_q_ms = bench_common::time_elapsed_s(t_hub1) * 1000.0;
        hub.destroy();

        bool hub_match = hub_cost == ref_cost;
        io::printfn("  [Hub]       pre=%.1f ms  query=%.1f ms  cost=%d  %s",
            hub_pre_ms, hub_q_ms, hub_cost, hub_match ? "MATCH" : "MISMATCH");
        if (!hub_match) all_ok = false;
    } else {
        io::printfn("  [Hub]       SKIPPED (V*T=%d > 500, O(V²) preprocessing)", total_nodes);
    }

    // ----------------------------------------------------------------
    // Summary
    // ----------------------------------------------------------------
    double total_ms = build_ms + dijk_ms + alt_pre_ms + alt_q_ms + cch_pre_ms + cch_q_ms + hub_pre_ms + hub_q_ms;
    long rss_kb = bench_common::peak_rss_kb_proc();
    io::printfn("  [temporal] total: %.1f ms, RSS: %d KB (%.1f MB)",
        total_ms, rss_kb, (double)rss_kb / 1024.0);
    io::printfn("  [temporal] all algorithms agree: %s", all_ok ? "YES" : "NO");

    // ----------------------------------------------------------------
    // Cleanup
    // ----------------------------------------------------------------
    tg.destroy();
    for (uint t = 0; t < t_steps; t++) free(all_cells[t]);
    free(all_cells);
    free(grids);
    g.destroy();
    free(wps);
    free(e_from);
    free(e_to);
    free(e_dist);
    free(e_head);
    free(e_wt);

    return ref_cost < pathfind::INF && all_ok;
}
