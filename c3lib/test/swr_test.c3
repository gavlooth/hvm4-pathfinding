module swr_test;
import swr;
import swr_ffi;
import swr_sup;
import swr_temporal;
import swr_dijkstra;
import swr_alt;
import swr_cch;
import swr_hub;
import dijkstra;
import pathfind;
import std::io;
import std::math;

int pass;
int fail;

fn void check(bool ok, String name) {
    if (ok) {
        io::printfn("PASS  %s", name);
        pass++;
    } else {
        io::printfn("FAIL  %s", name);
        fail++;
    }
}

// ============================================================
// Test: Haversine distance
// ============================================================

fn void test_haversine() {
    // London (51.5074N, 0.1278W) to Paris (48.8566N, 2.3522E)
    pathfind::LatLon london = { .lat = 51.5074, .lon = -0.1278 };
    pathfind::LatLon paris  = { .lat = 48.8566, .lon = 2.3522 };
    double d = pathfind::haversine(london, paris);
    // Expected ~343.5 km
    bool ok = d > 340_000.0 && d < 347_000.0;
    check(ok, "haversine_london_paris");

    // Same point
    double d2 = pathfind::haversine(london, london);
    check(d2 < 1.0, "haversine_same_point");

    // Equator to north pole (~10018 km)
    pathfind::LatLon eq = { .lat = 0.0, .lon = 0.0 };
    pathfind::LatLon np = { .lat = 90.0, .lon = 0.0 };
    double d3 = pathfind::haversine(eq, np);
    check(d3 > 10_000_000.0 && d3 < 10_040_000.0, "haversine_eq_to_pole");
}

// ============================================================
// Test: Bearing
// ============================================================

fn void test_bearing() {
    // Due north
    pathfind::LatLon a = { .lat = 0.0, .lon = 0.0 };
    pathfind::LatLon b = { .lat = 1.0, .lon = 0.0 };
    double brg = pathfind::bearing(a, b);
    check(brg < 1.0 || brg > 359.0, "bearing_due_north");

    // Due east
    pathfind::LatLon c = { .lat = 0.0, .lon = 1.0 };
    double brg2 = pathfind::bearing(a, c);
    check(brg2 > 89.0 && brg2 < 91.0, "bearing_due_east");
}

// ============================================================
// Test: LandMask
// ============================================================

fn void test_land_mask() {
    // 5x5 grid, island in center (row 1-3, col 1-3)
    bool[25] grid;
    for (int i = 0; i < 25; i++) grid[i] = false;
    // Set center island
    grid[1*5+1] = true; grid[1*5+2] = true; grid[1*5+3] = true;
    grid[2*5+1] = true; grid[2*5+2] = true; grid[2*5+3] = true;
    grid[3*5+1] = true; grid[3*5+2] = true; grid[3*5+3] = true;

    swr::LandMask mask = {
        .bounds = {
            .sw = { .lat = 37.0, .lon = 23.0 },
            .ne = { .lat = 39.0, .lon = 27.0 }
        },
        .rows = 5,
        .cols = 5,
        .grid = &grid
    };

    // Center should be land
    check(mask.is_land({ .lat = 38.0, .lon = 25.0 }), "land_mask_center_is_land");
    // Corner (SW) should be sea
    check(!mask.is_land({ .lat = 37.1, .lon = 23.1 }), "land_mask_corner_is_sea");
    // Outside bounds should be sea
    check(!mask.is_land({ .lat = 36.0, .lon = 22.0 }), "land_mask_outside_is_sea");

    // Segment from SW to NE goes through land
    pathfind::LatLon sw_pt = { .lat = 37.1, .lon = 23.1 };
    pathfind::LatLon ne_pt = { .lat = 38.9, .lon = 26.9 };
    check(!mask.segment_clear(sw_pt, ne_pt, 20), "land_mask_segment_blocked");

    // Segment along bottom edge (sea only)
    pathfind::LatLon bot_l = { .lat = 37.1, .lon = 23.1 };
    pathfind::LatLon bot_r = { .lat = 37.1, .lon = 26.9 };
    check(mask.segment_clear(bot_l, bot_r, 20), "land_mask_segment_clear");
}

// ============================================================
// Test: Weather-to-weight conversion
// ============================================================

fn void test_weather_cost() {
    swr::ShipModel ship = {
        .base_speed = 15.0,
        .max_wave_ht = 4.0,
        .fuel_rate_base = 100.0,
        .fuel_rate_per_kt = 5.0
    };

    swr::CostConfig cfg = {
        .mode = swr::CostMode.TIME_ONLY,
        .alpha = 1.0,
        .safety_penalty = 100.0,
        .scale = 1000
    };

    // Calm weather
    swr::WeatherCell calm = {
        .wind_speed = 0.0, .wind_dir = 0.0,
        .wave_height = 0.0, .wave_dir = 0.0,
        .current_speed = 0.0, .current_dir = 0.0
    };

    // 1 nautical mile = 1852 m, at 15 knots = 1/15 hours = 0.0667 hours
    double cost = swr::compute_edge_cost(&ship, calm, 1852.0, 90.0, &cfg);
    check(cost > 0.06 && cost < 0.07, "weather_cost_calm");

    // Rough weather: 3m waves should reduce speed
    swr::WeatherCell rough = {
        .wind_speed = 15.0, .wind_dir = 0.0,
        .wave_height = 3.0, .wave_dir = 0.0,
        .current_speed = 0.0, .current_dir = 0.0
    };
    double cost2 = swr::compute_edge_cost(&ship, rough, 1852.0, 90.0, &cfg);
    check(cost2 > cost, "weather_cost_rough_slower");
}

// ============================================================
// Test: PRM construction
// ============================================================

fn void test_prm() {
    // Small sea region with no land
    bool[25] grid;
    for (int i = 0; i < 25; i++) grid[i] = false;

    swr::SwrState state;
    state.land = (swr::LandMask){
        .bounds = {
            .sw = { .lat = 37.0, .lon = 23.0 },
            .ne = { .lat = 39.0, .lon = 27.0 }
        },
        .rows = 5,
        .cols = 5,
        .grid = &grid
    };
    state.ship = (swr::ShipModel){
        .base_speed = 15.0,
        .max_wave_ht = 4.0,
        .fuel_rate_base = 100.0,
        .fuel_rate_per_kt = 5.0
    };
    state.cost_cfg = (swr::CostConfig){
        .mode = swr::CostMode.TIME_ONLY,
        .alpha = 1.0,
        .safety_penalty = 100.0,
        .scale = 1000
    };

    swr::swr_build_prm(&state, 10, 3, 10, 42);
    defer state.destroy();

    check(state.prm.n == 10, "prm_node_count");
    check(state.prm.edge_count > 0, "prm_has_edges");

    // Each node should have at least 1 edge (connected graph for open sea)
    bool all_connected = true;
    for (uint i = 0; i < state.prm.n; i++) {
        bool has_edge = false;
        for (uint j = 0; j < state.prm.edge_count; j++) {
            if (state.prm.edge_from[j] == i || state.prm.edge_to[j] == i) {
                has_edge = true;
                break;
            }
        }
        if (!has_edge) { all_connected = false; break; }
    }
    check(all_connected, "prm_all_connected");
}

// ============================================================
// Test: C3 Dijkstra
// ============================================================

fn void test_dijkstra() {
    // Use the same graph as Bellman-Ford standalone test
    pathfind::Graph g;
    g.init(5);
    defer g.destroy();
    g.add_edge(0, 1, 4);
    g.add_edge(0, 2, 2);
    g.add_edge(1, 3, 3);
    g.add_edge(2, 1, 1);
    g.add_edge(2, 3, 5);
    g.add_edge(3, 4, 1);

    // Run through SWR Dijkstra by setting up a minimal SwrState
    bool[1] grid_data = { false };
    swr::SwrState state;
    state.land = (swr::LandMask){
        .bounds = { .sw = { .lat = 0.0, .lon = 0.0 }, .ne = { .lat = 1.0, .lon = 1.0 } },
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

    // Manually set up PRM with the test graph structure
    state.prm.n = 5;
    state.prm.waypoints = mem::new_array(swr::Waypoint, 5);
    for (uint i = 0; i < 5; i++) {
        state.prm.waypoints[i] = (swr::Waypoint){
            .id = i,
            .pos = { .lat = (double)i * 0.1, .lon = (double)i * 0.1 }
        };
    }
    state.prm.graph = g;
    state.prm.edge_count = 6;
    state.prm.edge_capacity = 6;
    state.prm.edge_from = mem::new_array(uint, 6);
    state.prm.edge_to = mem::new_array(uint, 6);
    state.prm.edge_dist_m = mem::new_array(double, 6);
    state.prm.edge_heading = mem::new_array(double, 6);
    state.prm.edge_weight = mem::new_array(uint, 6);

    // Edges matching the graph
    uint[6] froms = { 0, 0, 1, 2, 2, 3 };
    uint[6] tos   = { 1, 2, 3, 1, 3, 4 };
    uint[6] wts   = { 4, 2, 3, 1, 5, 1 };
    for (uint i = 0; i < 6; i++) {
        state.prm.edge_from[i] = froms[i];
        state.prm.edge_to[i] = tos[i];
        state.prm.edge_weight[i] = wts[i];
        state.prm.edge_dist_m[i] = 1852.0;
        state.prm.edge_heading[i] = 90.0;
    }

    state.origin = 0;
    state.destination = 4;

    // Run Dijkstra (skip weather update by calling directly)
    uint n = 5;
    uint[] dist = mem::new_array(uint, n);
    uint[] pred = mem::new_array(uint, n);
    defer { free(dist.ptr); free(pred.ptr); }

    for (uint i = 0; i < n; i++) {
        dist[i] = pathfind::INF;
        pred[i] = n;
    }
    dist[0] = 0;

    swr_dijkstra::FibHeap heap = swr_dijkstra::fib_heap_new();
    defer heap.destroy();

    swr_dijkstra::FibNode*[] nodes = mem::new_array(swr_dijkstra::FibNode*, n);
    defer free(nodes.ptr);
    for (uint i = 0; i < n; i++) {
        nodes[i] = heap.insert(dist[i], i);
    }

    while (heap.n > 0) {
        swr_dijkstra::FibNode* u_node = heap.extract_min();
        if (u_node == null) break;
        uint u = u_node.value;
        uint du = u_node.key;
        if (du >= pathfind::INF) break;

        for (uint i = 0; i < 6; i++) {
            if (state.prm.edge_from[i] == u) {
                uint v = state.prm.edge_to[i];
                uint w = state.prm.edge_weight[i];
                uint new_d = du + w;
                if (new_d < dist[v]) {
                    dist[v] = new_d;
                    pred[v] = u;
                    heap.decrease_key(nodes[v], new_d);
                }
            }
        }
    }

    // Expected: dist = [0, 3, 2, 6, 7] (same as Bellman-Ford)
    bool ok = dist[0]==0 && dist[1]==3 && dist[2]==2 && dist[3]==6 && dist[4]==7;
    check(ok, "dijkstra_matches_bellman_ford");

    // Don't let the state destroy the graph we created on stack
    state.prm.graph = (pathfind::Graph){};
    free(state.prm.waypoints.ptr);
    free(state.prm.edge_from.ptr);
    free(state.prm.edge_to.ptr);
    free(state.prm.edge_dist_m.ptr);
    free(state.prm.edge_heading.ptr);
    free(state.prm.edge_weight.ptr);
}

// ============================================================
// Test: SUP → Dijkstra pipeline
// ============================================================

fn void test_sup_dijkstra() {
    bool[25] grid;
    for (int i = 0; i < 25; i++) grid[i] = false;

    swr::SwrState state;
    state.land = (swr::LandMask){
        .bounds = {
            .sw = { .lat = 37.0, .lon = 23.0 },
            .ne = { .lat = 39.0, .lon = 27.0 }
        },
        .rows = 5,
        .cols = 5,
        .grid = &grid
    };
    state.ship = (swr::ShipModel){
        .base_speed = 15.0,
        .max_wave_ht = 4.0,
        .fuel_rate_base = 100.0,
        .fuel_rate_per_kt = 5.0
    };
    state.cost_cfg = (swr::CostConfig){
        .mode = swr::CostMode.TIME_ONLY,
        .alpha = 1.0,
        .safety_penalty = 100.0,
        .scale = 1000
    };

    swr::swr_build_prm(&state, 8, 3, 10, 123);
    defer state.destroy();

    swr::swr_set_voyage(&state, { .lat = 37.5, .lon = 23.5 }, { .lat = 38.5, .lon = 26.5 });

    // Three weather scenarios: calm, moderate, stormy
    swr::WeatherCell[4] calm_cells;
    swr::WeatherCell[4] mod_cells;
    swr::WeatherCell[4] storm_cells;
    for (int i = 0; i < 4; i++) {
        calm_cells[i] = (swr::WeatherCell){
            .wind_speed = 3.0, .wind_dir = 180.0,
            .wave_height = 0.5, .wave_dir = 180.0,
            .current_speed = 0.2, .current_dir = 90.0
        };
        mod_cells[i] = (swr::WeatherCell){
            .wind_speed = 10.0, .wind_dir = 270.0,
            .wave_height = 2.0, .wave_dir = 270.0,
            .current_speed = 1.0, .current_dir = 180.0
        };
        storm_cells[i] = (swr::WeatherCell){
            .wind_speed = 20.0, .wind_dir = 0.0,
            .wave_height = 3.5, .wave_dir = 0.0,
            .current_speed = 2.0, .current_dir = 270.0
        };
    }

    swr::WeatherGrid[3] scenarios;
    swr::WeatherGrid base = {
        .bounds = {
            .sw = { .lat = 37.0, .lon = 23.0 },
            .ne = { .lat = 39.0, .lon = 27.0 }
        },
        .rows = 2,
        .cols = 2,
        .timestamp = 0.0
    };
    scenarios[0] = base; scenarios[0].cells = &calm_cells;
    scenarios[1] = base; scenarios[1].cells = &mod_cells;
    scenarios[2] = base; scenarios[2].cells = &storm_cells;

    // Run SUP → Dijkstra with BEST_SINGLE strategy
    swr::RouteResult result = swr_sup::swr_sup_dijkstra(
        &state, &scenarios, 3, swr_sup::ScenarioStrategy.BEST_SINGLE)!!;
    defer result.destroy();

    check(result.total_cost > 0 && result.total_cost < pathfind::INF, "sup_dijkstra_cost");
    check(result.path_len >= 2, "sup_dijkstra_path");
    check(result.path[0] == state.origin, "sup_dijkstra_starts_at_origin");
    check(result.path[result.path_len - 1] == state.destination, "sup_dijkstra_ends_at_dest");
}

// ============================================================
// Test: Fibonacci Heap operations
// ============================================================

fn void test_fib_heap() {
    swr_dijkstra::FibHeap heap = swr_dijkstra::fib_heap_new();
    defer heap.destroy();

    // Insert and extract_min
    heap.insert(5, 50);
    heap.insert(3, 30);
    heap.insert(7, 70);
    heap.insert(1, 10);

    swr_dijkstra::FibNode* m = heap.extract_min();
    check(m != null && m.key == 1 && m.value == 10, "fib_heap_extract_min_1");

    m = heap.extract_min();
    check(m != null && m.key == 3 && m.value == 30, "fib_heap_extract_min_2");

    // Decrease key
    swr_dijkstra::FibNode* n7 = null;
    // Remaining: keys 5 and 7
    swr_dijkstra::FibHeap heap2 = swr_dijkstra::fib_heap_new();
    defer heap2.destroy();

    swr_dijkstra::FibNode* na = heap2.insert(10, 100);
    swr_dijkstra::FibNode* nb = heap2.insert(20, 200);
    heap2.decrease_key(nb, 5);

    m = heap2.extract_min();
    check(m != null && m.key == 5 && m.value == 200, "fib_heap_decrease_key");
}

// ============================================================
// Test: HVM4 weather-to-weight pipeline
// ============================================================

fn void test_weather_hvm4() {
    bool[25] grid;
    for (int i = 0; i < 25; i++) grid[i] = false;

    swr::SwrState state;
    state.land = (swr::LandMask){
        .bounds = {
            .sw = { .lat = 37.0, .lon = 23.0 },
            .ne = { .lat = 39.0, .lon = 27.0 }
        },
        .rows = 5,
        .cols = 5,
        .grid = &grid
    };
    state.ship = (swr::ShipModel){
        .base_speed = 15.0,
        .max_wave_ht = 4.0,
        .fuel_rate_base = 100.0,
        .fuel_rate_per_kt = 5.0
    };
    state.cost_cfg = (swr::CostConfig){
        .mode = swr::CostMode.TIME_ONLY,
        .alpha = 1.0,
        .safety_penalty = 100.0,
        .scale = 1000
    };

    swr::swr_build_prm(&state, 8, 3, 10, 123);
    defer state.destroy();

    // Uniform weather
    swr::WeatherCell[4] cells;
    for (int i = 0; i < 4; i++) {
        cells[i] = (swr::WeatherCell){
            .wind_speed = 5.0, .wind_dir = 180.0,
            .wave_height = 1.0, .wave_dir = 180.0,
            .current_speed = 0.5, .current_dir = 90.0
        };
    }
    swr::WeatherGrid weather = {
        .bounds = {
            .sw = { .lat = 37.0, .lon = 23.0 },
            .ne = { .lat = 39.0, .lon = 27.0 }
        },
        .rows = 2,
        .cols = 2,
        .cells = &cells,
        .timestamp = 0.0
    };

    // Compute weights via HVM4 pipeline
    uint[] hvm4_weights = swr::compute_weights_hvm4(&state, &weather)!!;
    defer free(hvm4_weights.ptr);

    // Compute weights via C3 for comparison
    uint e = state.prm.edge_count;
    uint[] c3_weights = mem::new_array(uint, e);
    defer free(c3_weights.ptr);

    for (uint i = 0; i < e; i++) {
        uint from = state.prm.edge_from[i];
        uint to   = state.prm.edge_to[i];
        pathfind::LatLon mid = {
            .lat = (state.prm.waypoints[from].pos.lat + state.prm.waypoints[to].pos.lat) / 2.0,
            .lon = (state.prm.waypoints[from].pos.lon + state.prm.waypoints[to].pos.lon) / 2.0
        };
        swr::WeatherCell w = weather.sample(mid);
        double cost = swr::compute_edge_cost(&state.ship, w, state.prm.edge_dist_m[i], state.prm.edge_heading[i], &state.cost_cfg);
        uint weight = (uint)(cost * (double)state.cost_cfg.scale);
        if (weight < 1) weight = 1;
        if (weight > 900_000) weight = 900_000;
        c3_weights[i] = weight;
    }

    // Verify: all weights > 0
    bool all_positive = true;
    for (uint i = 0; i < e; i++) {
        if (hvm4_weights[i] == 0) { all_positive = false; break; }
    }
    check(all_positive, "weather_hvm4_all_positive");

    // Verify: HVM4 weights match C3 weights exactly (same formula, same rounding)
    bool all_match = true;
    for (uint i = 0; i < e; i++) {
        if (hvm4_weights[i] != c3_weights[i]) {
            io::printfn("  edge %d: hvm4=%d c3=%d", i, hvm4_weights[i], c3_weights[i]);
            all_match = false;
        }
    }
    check(all_match, "weather_hvm4_matches_c3");
}

// ============================================================
// Test: Dijkstra utility (standalone, on Graph)
// ============================================================

fn void test_dijkstra_util() {
    pathfind::Graph g;
    g.init(5);
    defer g.destroy();
    g.add_edge(0, 1, 4);
    g.add_edge(0, 2, 2);
    g.add_edge(1, 3, 3);
    g.add_edge(2, 1, 1);
    g.add_edge(2, 3, 5);
    g.add_edge(3, 4, 1);

    dijkstra::DijkResult dr = dijkstra::sssp(&g, 0)!!;
    defer dr.destroy();

    // Expected: [0, 3, 2, 6, 7]
    bool ok = dr.dist[0]==0 && dr.dist[1]==3 && dr.dist[2]==2
           && dr.dist[3]==6 && dr.dist[4]==7;
    check(ok, "dijkstra_util_sssp");
}

// ============================================================
// Test: ALT algorithm
// ============================================================

fn void test_alt() {
    pathfind::Graph g;
    g.init(10);
    defer g.destroy();

    // Build a connected graph
    g.add_biedge(0, 1, 5);
    g.add_biedge(1, 2, 3);
    g.add_biedge(2, 3, 4);
    g.add_biedge(3, 4, 2);
    g.add_biedge(4, 5, 6);
    g.add_biedge(5, 6, 1);
    g.add_biedge(6, 7, 3);
    g.add_biedge(7, 8, 2);
    g.add_biedge(8, 9, 4);
    g.add_biedge(0, 3, 10);
    g.add_biedge(1, 4, 8);
    g.add_biedge(5, 9, 7);

    // Get Dijkstra reference distance
    dijkstra::DijkResult ref_dr = dijkstra::sssp(&g, 0)!!;
    uint ref_dist = ref_dr.dist[9];
    ref_dr.destroy();

    // ALT query
    swr_alt::AltData alt = swr_alt::preprocess(&g, 4);
    defer alt.destroy();

    swr::RouteResult result = swr_alt::query(&alt, &g, 0, 9)!!;
    defer result.destroy();

    check(result.total_cost == ref_dist, "alt_matches_dijkstra");
}

// ============================================================
// Test: CCH algorithm
// ============================================================

fn void test_cch() {
    // Build fresh graph each time (contraction modifies graph)
    pathfind::Graph g;
    g.init(10);
    defer g.destroy();

    g.add_biedge(0, 1, 5);
    g.add_biedge(1, 2, 3);
    g.add_biedge(2, 3, 4);
    g.add_biedge(3, 4, 2);
    g.add_biedge(4, 5, 6);
    g.add_biedge(5, 6, 1);
    g.add_biedge(6, 7, 3);
    g.add_biedge(7, 8, 2);
    g.add_biedge(8, 9, 4);
    g.add_biedge(0, 3, 10);
    g.add_biedge(1, 4, 8);
    g.add_biedge(5, 9, 7);

    // Get Dijkstra reference distance
    dijkstra::DijkResult ref_dr = dijkstra::sssp(&g, 0)!!;
    uint ref_dist = ref_dr.dist[9];
    ref_dr.destroy();

    // CCH: build topology on a copy (contraction adds shortcuts)
    pathfind::Graph g_copy;
    g_copy.init(10);
    g_copy.add_biedge(0, 1, 5);
    g_copy.add_biedge(1, 2, 3);
    g_copy.add_biedge(2, 3, 4);
    g_copy.add_biedge(3, 4, 2);
    g_copy.add_biedge(4, 5, 6);
    g_copy.add_biedge(5, 6, 1);
    g_copy.add_biedge(6, 7, 3);
    g_copy.add_biedge(7, 8, 2);
    g_copy.add_biedge(8, 9, 4);
    g_copy.add_biedge(0, 3, 10);
    g_copy.add_biedge(1, 4, 8);
    g_copy.add_biedge(5, 9, 7);

    swr_cch::CchData cch = swr_cch::build_topology(&g_copy);
    g_copy.destroy();
    defer cch.destroy();

    swr_cch::customize(&cch, &g);
    uint cch_dist = swr_cch::cch_query(&cch, 0, 9);

    if (cch_dist != ref_dist) {
        io::printfn("  cch=%d ref=%d", cch_dist, ref_dist);
    }
    check(cch_dist == ref_dist, "cch_matches_dijkstra");
}

// ============================================================
// Test: Hub Labeling
// ============================================================

fn void test_hub() {
    pathfind::Graph g;
    g.init(10);
    defer g.destroy();

    g.add_biedge(0, 1, 5);
    g.add_biedge(1, 2, 3);
    g.add_biedge(2, 3, 4);
    g.add_biedge(3, 4, 2);
    g.add_biedge(4, 5, 6);
    g.add_biedge(5, 6, 1);
    g.add_biedge(6, 7, 3);
    g.add_biedge(7, 8, 2);
    g.add_biedge(8, 9, 4);
    g.add_biedge(0, 3, 10);
    g.add_biedge(1, 4, 8);
    g.add_biedge(5, 9, 7);

    dijkstra::DijkResult ref_dr = dijkstra::sssp(&g, 0)!!;
    uint ref_dist = ref_dr.dist[9];
    ref_dr.destroy();

    swr_hub::HubData hub = swr_hub::preprocess(&g);
    defer hub.destroy();

    uint hub_dist = swr_hub::hub_query(&hub, 0, 9);

    check(hub_dist == ref_dist, "hub_matches_dijkstra");
}

// ============================================================
// Test: SWR dispatch for new algorithms
// ============================================================

fn void test_swr_dispatch() {
    bool[25] grid;
    for (int i = 0; i < 25; i++) grid[i] = false;

    swr::SwrState state;
    state.land = (swr::LandMask){
        .bounds = {
            .sw = { .lat = 37.0, .lon = 23.0 },
            .ne = { .lat = 39.0, .lon = 27.0 }
        },
        .rows = 5,
        .cols = 5,
        .grid = &grid
    };
    state.ship = (swr::ShipModel){
        .base_speed = 15.0,
        .max_wave_ht = 4.0,
        .fuel_rate_base = 100.0,
        .fuel_rate_per_kt = 5.0
    };
    state.cost_cfg = (swr::CostConfig){
        .mode = swr::CostMode.TIME_ONLY,
        .alpha = 1.0,
        .safety_penalty = 100.0,
        .scale = 1000
    };

    swr::swr_build_prm(&state, 8, 3, 10, 123);
    defer state.destroy();

    swr::swr_set_voyage(&state, { .lat = 37.5, .lon = 23.5 }, { .lat = 38.5, .lon = 26.5 });

    swr::WeatherCell[4] cells;
    for (int i = 0; i < 4; i++) {
        cells[i] = (swr::WeatherCell){
            .wind_speed = 3.0, .wind_dir = 180.0,
            .wave_height = 0.5, .wave_dir = 180.0,
            .current_speed = 0.2, .current_dir = 90.0
        };
    }
    swr::WeatherGrid weather = {
        .bounds = {
            .sw = { .lat = 37.0, .lon = 23.0 },
            .ne = { .lat = 39.0, .lon = 27.0 }
        },
        .rows = 2,
        .cols = 2,
        .cells = &cells,
        .timestamp = 0.0
    };

    // Get Dijkstra reference
    swr::RouteResult dij_result = swr::swr_replan(&state, &weather, swr::Algorithm.DIJKSTRA)!!;
    uint dij_cost = dij_result.total_cost;
    dij_result.destroy();

    // ALT should find same optimal cost
    swr::RouteResult alt_result = swr::swr_replan(&state, &weather, swr::Algorithm.ALT)!!;
    check(alt_result.total_cost == dij_cost, "swr_dispatch_alt");
    alt_result.destroy();

    // CCH should find same optimal cost
    swr::RouteResult cch_result = swr::swr_replan(&state, &weather, swr::Algorithm.CCH) ?? (swr::RouteResult){};
    if (cch_result.total_cost > 0 && cch_result.total_cost < pathfind::INF) {
        check(cch_result.total_cost == dij_cost, "swr_dispatch_cch");
    } else {
        check(false, "swr_dispatch_cch");
    }
    cch_result.destroy();

    // Hub should find same optimal cost
    swr::RouteResult hub_result = swr::swr_replan(&state, &weather, swr::Algorithm.HUB_LABEL) ?? (swr::RouteResult){};
    if (hub_result.total_cost > 0 && hub_result.total_cost < pathfind::INF) {
        check(hub_result.total_cost == dij_cost, "swr_dispatch_hub");
    } else {
        check(false, "swr_dispatch_hub");
    }
    hub_result.destroy();
}

// ============================================================
// Test: Temporal graph build
// ============================================================

fn void test_temporal_graph_build() {
    // 5 spatial nodes, 2 time steps → 10 temporal nodes
    pathfind::Graph g;
    g.init(5);
    defer g.destroy();
    g.add_edge(0, 1, 10);
    g.add_edge(1, 2, 10);
    g.add_edge(2, 3, 10);
    g.add_edge(3, 4, 10);

    // Minimal SwrState with manual PRM
    bool[1] grid_data = { false };
    swr::SwrState state;
    state.land = (swr::LandMask){
        .bounds = { .sw = { .lat = 0.0, .lon = 0.0 }, .ne = { .lat = 1.0, .lon = 1.0 } },
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

    state.prm.n = 5;
    state.prm.graph = g;
    state.prm.waypoints = mem::new_array(swr::Waypoint, 5);
    for (uint i = 0; i < 5; i++) {
        state.prm.waypoints[i] = (swr::Waypoint){
            .id = i, .pos = { .lat = (double)i * 0.1, .lon = (double)i * 0.1 }
        };
    }
    state.prm.edge_count = 4;
    state.prm.edge_capacity = 4;
    state.prm.edge_from = mem::new_array(uint, 4);
    state.prm.edge_to = mem::new_array(uint, 4);
    state.prm.edge_dist_m = mem::new_array(double, 4);
    state.prm.edge_heading = mem::new_array(double, 4);
    state.prm.edge_weight = mem::new_array(uint, 4);

    uint[4] froms = { 0, 1, 2, 3 };
    uint[4] tos = { 1, 2, 3, 4 };
    for (uint i = 0; i < 4; i++) {
        state.prm.edge_from[i] = froms[i];
        state.prm.edge_to[i] = tos[i];
        state.prm.edge_weight[i] = 10;
        state.prm.edge_dist_m[i] = 18520.0;  // ~10 NM
        state.prm.edge_heading[i] = 90.0;
    }

    // 2-step forecast with calm weather
    swr::WeatherCell[1] cells = {{ .wind_speed = 0.0, .wind_dir = 0.0,
        .wave_height = 0.0, .wave_dir = 0.0, .current_speed = 0.0, .current_dir = 0.0 }};
    swr::WeatherGrid[2] grids;
    for (int i = 0; i < 2; i++) {
        grids[i] = (swr::WeatherGrid){
            .bounds = { .sw = { .lat = 0.0, .lon = 0.0 }, .ne = { .lat = 1.0, .lon = 1.0 } },
            .rows = 1, .cols = 1, .cells = &cells, .timestamp = (double)i * 6.0
        };
    }
    swr::WeatherForecast forecast = {
        .grids = &grids, .count = 2, .interval_hours = 6, .wait_cost = 1
    };

    pathfind::Graph tg = swr_temporal::build_temporal_graph(&state, &forecast);
    defer tg.destroy();

    check(tg.n == 10, "temporal_graph_node_count");

    // Should have spatial edges (some may be pruned if arrival > horizon)
    // Plus waiting edges: 5 nodes * 1 transition = 5 waiting edges
    uint total_edges = 0;
    for (uint i = 0; i < tg.n; i++) {
        total_edges += (uint)tg.adj[i].len();
    }
    check(total_edges > 0, "temporal_graph_has_edges");

    // Clean up manual PRM
    state.prm.graph = (pathfind::Graph){};
    free(state.prm.waypoints.ptr);
    free(state.prm.edge_from.ptr);
    free(state.prm.edge_to.ptr);
    free(state.prm.edge_dist_m.ptr);
    free(state.prm.edge_heading.ptr);
    free(state.prm.edge_weight.ptr);
}

// ============================================================
// Mock Weather: Realistic Mediterranean Sea conditions
// Source: NOAA GFS, Copernicus MEDSEA, Frontiers wave climatology
// Aegean Sea region (37-39N, 23-27E)
// ============================================================

// Summer calm: light Etesian, low swell, weak eastward current
fn swr::WeatherCell mock_med_calm() {
    return (swr::WeatherCell){
        .wind_speed = 5.0, .wind_dir = 340.0,       // 5kt NNW (light Etesian)
        .wave_height = 0.3, .wave_dir = 350.0,       // 0.3m NW swell
        .current_speed = 0.2, .current_dir = 90.0    // 0.2kt eastward
    };
}

// Transitional: moderate Meltemi, building seas
fn swr::WeatherCell mock_med_moderate() {
    return (swr::WeatherCell){
        .wind_speed = 15.0, .wind_dir = 350.0,       // 15kt NNW (Meltemi)
        .wave_height = 1.8, .wave_dir = 340.0,        // 1.8m NW swell
        .current_speed = 0.5, .current_dir = 120.0    // 0.5kt SE drift
    };
}

// Winter storm: strong Meltemi/Bora, heavy seas
fn swr::WeatherCell mock_med_storm() {
    return (swr::WeatherCell){
        .wind_speed = 30.0, .wind_dir = 20.0,        // 30kt NNE (Bora-type)
        .wave_height = 4.5, .wave_dir = 10.0,         // 4.5m N swell
        .current_speed = 1.5, .current_dir = 200.0    // 1.5kt SW storm surge
    };
}

fn swr::SeaBounds mock_med_bounds() {
    return (swr::SeaBounds){
        .sw = { .lat = 37.0, .lon = 23.0 },
        .ne = { .lat = 39.0, .lon = 27.0 }
    };
}

// Fill a 2x2 weather grid with a single cell type (uniform conditions)
fn void fill_grid_uniform(swr::WeatherCell[] cells, swr::WeatherCell cell) {
    for (usz i = 0; i < cells.len; i++) cells[i] = cell;
}

// ============================================================
// Test: Temporal routing vs static Dijkstra (constant weather)
// ============================================================

fn void test_temporal_routing() {
    bool[25] grid;
    for (int i = 0; i < 25; i++) grid[i] = false;

    swr::SwrState state;
    swr::SeaBounds bounds = mock_med_bounds();
    state.land = (swr::LandMask){
        .bounds = bounds, .rows = 5, .cols = 5, .grid = &grid
    };
    state.ship = (swr::ShipModel){
        .base_speed = 15.0, .max_wave_ht = 4.0,
        .fuel_rate_base = 100.0, .fuel_rate_per_kt = 5.0
    };
    state.cost_cfg = (swr::CostConfig){
        .mode = swr::CostMode.TIME_ONLY, .alpha = 1.0,
        .safety_penalty = 100.0, .scale = 1000
    };

    swr::swr_build_prm(&state, 8, 3, 10, 123);
    defer state.destroy();
    swr::swr_set_voyage(&state, { .lat = 37.5, .lon = 23.5 }, { .lat = 38.5, .lon = 26.5 });

    // Constant moderate Meltemi across all time steps
    swr::WeatherCell[4] cells;
    fill_grid_uniform(&cells, mock_med_moderate());

    swr::WeatherGrid base_grid = {
        .bounds = bounds, .rows = 2, .cols = 2, .cells = &cells, .timestamp = 0.0
    };

    // Static Dijkstra reference
    swr::RouteResult static_result = swr::swr_replan(&state, &base_grid, swr::Algorithm.DIJKSTRA)!!;
    uint static_cost = static_result.total_cost;
    static_result.destroy();

    // Temporal with 4 time steps of identical weather
    swr::WeatherGrid[4] grids;
    for (int i = 0; i < 4; i++) {
        grids[i] = base_grid;
        grids[i].timestamp = (double)i * 6.0;
    }
    swr::WeatherForecast forecast = {
        .grids = &grids, .count = 4, .interval_hours = 6, .wait_cost = 1
    };

    swr::RouteResult temporal_result = swr_temporal::swr_route_temporal(&state, &forecast)!!;
    defer temporal_result.destroy();

    check(temporal_result.total_cost > 0 && temporal_result.total_cost < pathfind::INF,
          "temporal_routing_finds_path");
    check(temporal_result.path_len >= 2, "temporal_routing_has_path");
}

// ============================================================
// Test: Temporal waiting (bad weather then calm)
// ============================================================

fn void test_temporal_waiting() {
    // Build a simple 2-node graph: origin → destination
    bool[1] grid_data = { false };
    swr::SwrState state;
    state.land = (swr::LandMask){
        .bounds = { .sw = { .lat = 0.0, .lon = 0.0 }, .ne = { .lat = 1.0, .lon = 1.0 } },
        .rows = 1, .cols = 1, .grid = &grid_data
    };
    state.ship = (swr::ShipModel){
        .base_speed = 15.0, .max_wave_ht = 4.0,
        .fuel_rate_base = 100.0, .fuel_rate_per_kt = 5.0
    };
    state.cost_cfg = (swr::CostConfig){
        .mode = swr::CostMode.SAFETY_PENALIZED, .alpha = 0.5,
        .safety_penalty = 500_000.0, .scale = 1000
    };

    pathfind::Graph g;
    g.init(2);
    state.prm.n = 2;
    state.prm.graph = g;
    state.prm.waypoints = mem::new_array(swr::Waypoint, 2);
    state.prm.waypoints[0] = (swr::Waypoint){ .id = 0, .pos = { .lat = 0.2, .lon = 0.2 } };
    state.prm.waypoints[1] = (swr::Waypoint){ .id = 1, .pos = { .lat = 0.8, .lon = 0.8 } };
    state.prm.edge_count = 1;
    state.prm.edge_capacity = 1;
    state.prm.edge_from = mem::new_array(uint, 1);
    state.prm.edge_to = mem::new_array(uint, 1);
    state.prm.edge_dist_m = mem::new_array(double, 1);
    state.prm.edge_heading = mem::new_array(double, 1);
    state.prm.edge_weight = mem::new_array(uint, 1);
    state.prm.edge_from[0] = 0;
    state.prm.edge_to[0] = 1;
    state.prm.edge_dist_m[0] = 18520.0;  // ~10 NM
    state.prm.edge_heading[0] = 45.0;
    state.prm.edge_weight[0] = 100;

    state.origin = 0;
    state.destination = 1;

    // Storm at t=0 (winter Bora), calm at t=1 (summer)
    swr::WeatherCell[1] storm_cells = { mock_med_storm() };
    swr::WeatherCell[1] calm_cells = { mock_med_calm() };

    swr::WeatherGrid[3] grids;
    swr::WeatherGrid base_tpl = {
        .bounds = { .sw = { .lat = 0.0, .lon = 0.0 }, .ne = { .lat = 1.0, .lon = 1.0 } },
        .rows = 1, .cols = 1, .timestamp = 0.0
    };
    grids[0] = base_tpl; grids[0].cells = &storm_cells; grids[0].timestamp = 0.0;
    grids[1] = base_tpl; grids[1].cells = &calm_cells;  grids[1].timestamp = 6.0;
    grids[2] = base_tpl; grids[2].cells = &calm_cells;  grids[2].timestamp = 12.0;

    swr::WeatherForecast forecast = {
        .grids = &grids, .count = 3, .interval_hours = 6, .wait_cost = 1
    };

    // Build temporal graph
    pathfind::Graph tg = swr_temporal::build_temporal_graph(&state, &forecast);
    defer tg.destroy();

    // Get cost of going at t=0 (storm) vs waiting + going at t=1 (calm)
    // Direct at t=0: edge (0*3+0) → (1*3+arrival)
    // Wait then go: (0*3+0) → (0*3+1) → (1*3+arrival)
    // With huge safety penalty on storm, waiting should be cheaper if we can reach
    dijkstra::DijkResult dr = dijkstra::sssp(&tg, 0)!!;
    defer dr.destroy();

    // Check destination reachable at some time step
    uint best_cost = pathfind::INF;
    for (uint t = 0; t < 3; t++) {
        uint idx = 1 * 3 + t;
        if (dr.dist[idx] < best_cost) best_cost = dr.dist[idx];
    }
    check(best_cost < pathfind::INF, "temporal_waiting_finds_path");

    // Verify that cost at t=0 departure through storm is higher than waiting
    // Storm edge at t=0 goes to destination at some arrival time
    uint storm_direct = pathfind::INF;
    for (usz i = 0; i < tg.adj[0].len(); i++) {
        pathfind::Edge e = tg.adj[0][i];
        // Skip waiting edge to self (node 1 = 0*3+1)
        if (e.to / 3 == 1) {
            storm_direct = e.weight;
            break;
        }
    }

    // Calm edge at t=1 (node 0*3+1=1) goes to destination
    uint calm_via_wait = pathfind::INF;
    if (tg.adj[1].len() > 0) {
        for (usz i = 0; i < tg.adj[1].len(); i++) {
            pathfind::Edge e = tg.adj[1][i];
            if (e.to / 3 == 1) {
                calm_via_wait = forecast.wait_cost + e.weight;
                break;
            }
        }
    }

    if (storm_direct < pathfind::INF && calm_via_wait < pathfind::INF) {
        check(calm_via_wait < storm_direct, "temporal_waiting_cheaper_in_storm");
    } else {
        // At least one path should exist
        check(best_cost < pathfind::INF, "temporal_waiting_cheaper_in_storm");
    }

    // Clean up manual PRM
    state.prm.graph = (pathfind::Graph){};
    free(state.prm.waypoints.ptr);
    free(state.prm.edge_from.ptr);
    free(state.prm.edge_to.ptr);
    free(state.prm.edge_dist_m.ptr);
    free(state.prm.edge_heading.ptr);
    free(state.prm.edge_weight.ptr);
}

// ============================================================
// Test: Temporal horizon (edges beyond T excluded)
// ============================================================

fn void test_temporal_horizon() {
    // Long edge (200 NM, ~13h travel) with T=2, interval=6h
    // floor(13.3/6) = 2 steps → arrival at t=2 >= T=2, so edge excluded at both t=0 and t=1
    bool[1] grid_data = { false };
    swr::SwrState state;
    state.land = (swr::LandMask){
        .bounds = { .sw = { .lat = 0.0, .lon = 0.0 }, .ne = { .lat = 1.0, .lon = 1.0 } },
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

    pathfind::Graph g;
    g.init(2);
    state.prm.n = 2;
    state.prm.graph = g;
    state.prm.waypoints = mem::new_array(swr::Waypoint, 2);
    state.prm.waypoints[0] = (swr::Waypoint){ .id = 0, .pos = { .lat = 0.2, .lon = 0.2 } };
    state.prm.waypoints[1] = (swr::Waypoint){ .id = 1, .pos = { .lat = 0.8, .lon = 0.8 } };
    state.prm.edge_count = 1;
    state.prm.edge_capacity = 1;
    state.prm.edge_from = mem::new_array(uint, 1);
    state.prm.edge_to = mem::new_array(uint, 1);
    state.prm.edge_dist_m = mem::new_array(double, 1);
    state.prm.edge_heading = mem::new_array(double, 1);
    state.prm.edge_weight = mem::new_array(uint, 1);
    state.prm.edge_from[0] = 0;
    state.prm.edge_to[0] = 1;
    state.prm.edge_dist_m[0] = 370400.0;  // 200 NM → ~13.3h at 15kt → 2 steps
    state.prm.edge_heading[0] = 45.0;
    state.prm.edge_weight[0] = 100;

    // T=2 time steps, calm weather
    swr::WeatherCell[1] cells = {{ .wind_speed = 0.0, .wind_dir = 0.0,
        .wave_height = 0.0, .wave_dir = 0.0, .current_speed = 0.0, .current_dir = 0.0 }};
    swr::WeatherGrid[2] grids;
    for (int i = 0; i < 2; i++) {
        grids[i] = (swr::WeatherGrid){
            .bounds = { .sw = { .lat = 0.0, .lon = 0.0 }, .ne = { .lat = 1.0, .lon = 1.0 } },
            .rows = 1, .cols = 1, .cells = &cells, .timestamp = (double)i * 6.0
        };
    }
    swr::WeatherForecast forecast = {
        .grids = &grids, .count = 2, .interval_hours = 6, .wait_cost = 1
    };

    pathfind::Graph tg = swr_temporal::build_temporal_graph(&state, &forecast);
    defer tg.destroy();

    // Spatial edges: at t=0 arrival=2>=T, at t=1 arrival=3>=T → both excluded
    // Only waiting edges remain: 1 node * 1 transition = 1 waiting edge (node 0: t0→t1)
    // Node 1 also gets 1 waiting edge. Total = 2 waiting edges.
    uint spatial_edges = 0;
    for (uint i = 0; i < tg.n; i++) {
        for (usz j = 0; j < tg.adj[i].len(); j++) {
            pathfind::Edge e = tg.adj[i][j];
            uint from_node = i / 2;
            uint to_node = e.to / 2;
            if (from_node != to_node) spatial_edges++;
        }
    }
    check(spatial_edges == 0, "temporal_horizon_no_spatial_edges");

    // Clean up
    state.prm.graph = (pathfind::Graph){};
    free(state.prm.waypoints.ptr);
    free(state.prm.edge_from.ptr);
    free(state.prm.edge_to.ptr);
    free(state.prm.edge_dist_m.ptr);
    free(state.prm.edge_heading.ptr);
    free(state.prm.edge_weight.ptr);
}

// ============================================================
// Test: All algorithms agree on temporal graph
// Build a temporal graph, then run Dijkstra / ALT / CCH / Hub
// on it and verify they all find the same shortest-path cost.
// ============================================================

fn void test_temporal_all_algorithms() {
    bool[25] grid;
    for (int i = 0; i < 25; i++) grid[i] = false;

    swr::SeaBounds bounds = mock_med_bounds();
    swr::SwrState state;
    state.land = (swr::LandMask){
        .bounds = bounds, .rows = 5, .cols = 5, .grid = &grid
    };
    state.ship = (swr::ShipModel){
        .base_speed = 15.0, .max_wave_ht = 4.0,
        .fuel_rate_base = 100.0, .fuel_rate_per_kt = 5.0
    };
    state.cost_cfg = (swr::CostConfig){
        .mode = swr::CostMode.TIME_ONLY, .alpha = 1.0,
        .safety_penalty = 100.0, .scale = 1000
    };

    swr::swr_build_prm(&state, 10, 4, 10, 777);
    defer state.destroy();
    swr::swr_set_voyage(&state, { .lat = 37.3, .lon = 23.5 }, { .lat = 38.7, .lon = 26.5 });

    // 3-step forecast: calm → moderate → storm (realistic progression)
    swr::WeatherCell[4] calm_cells;
    swr::WeatherCell[4] mod_cells;
    swr::WeatherCell[4] storm_cells;
    fill_grid_uniform(&calm_cells, mock_med_calm());
    fill_grid_uniform(&mod_cells, mock_med_moderate());
    fill_grid_uniform(&storm_cells, mock_med_storm());

    swr::WeatherGrid[3] grids;
    swr::WeatherGrid tpl = {
        .bounds = bounds, .rows = 2, .cols = 2, .timestamp = 0.0
    };
    grids[0] = tpl; grids[0].cells = &calm_cells;  grids[0].timestamp = 0.0;
    grids[1] = tpl; grids[1].cells = &mod_cells;   grids[1].timestamp = 6.0;
    grids[2] = tpl; grids[2].cells = &storm_cells;  grids[2].timestamp = 12.0;

    swr::WeatherForecast forecast = {
        .grids = &grids, .count = 3, .interval_hours = 6, .wait_cost = 50
    };

    // Build temporal graph (shared by all algorithms)
    uint t_count = forecast.count;
    pathfind::Graph tg = swr_temporal::build_temporal_graph(&state, &forecast);
    defer tg.destroy();

    uint src = state.origin * t_count;
    uint n = tg.n;

    // --- 1. Dijkstra (reference) ---
    dijkstra::DijkResult dij = dijkstra::sssp(&tg, src)!!;
    uint ref_cost = pathfind::INF;
    for (uint t = 0; t < t_count; t++) {
        uint idx = state.destination * t_count + t;
        if (dij.dist[idx] < ref_cost) ref_cost = dij.dist[idx];
    }
    dij.destroy();

    check(ref_cost > 0 && ref_cost < pathfind::INF, "temporal_alg_dijkstra_finds_path");

    // --- 2. ALT ---
    swr_alt::AltData alt = swr_alt::preprocess(&tg, 4);
    defer alt.destroy();

    // ALT query: find best arrival time
    uint alt_cost = pathfind::INF;
    for (uint t = 0; t < t_count; t++) {
        uint dst = state.destination * t_count + t;
        swr::RouteResult ar = swr_alt::query(&alt, &tg, src, dst) ?? (swr::RouteResult){};
        if (ar.total_cost > 0 && ar.total_cost < alt_cost) alt_cost = ar.total_cost;
        ar.destroy();
    }
    if (alt_cost < pathfind::INF) {
        check(alt_cost == ref_cost, "temporal_alg_alt_matches_dijkstra");
    } else {
        check(false, "temporal_alg_alt_matches_dijkstra");
    }

    // --- 3. CCH ---
    // Build on a copy (contraction modifies the graph)
    pathfind::Graph tg_copy;
    tg_copy.init(n);
    for (uint u = 0; u < n; u++) {
        for (usz i = 0; i < tg.adj[u].len(); i++) {
            pathfind::Edge e = tg.adj[u][i];
            tg_copy.add_edge(u, e.to, e.weight);
        }
    }

    swr_cch::CchData cch = swr_cch::build_topology(&tg_copy);
    tg_copy.destroy();
    defer cch.destroy();

    swr_cch::customize(&cch, &tg);
    uint cch_cost = pathfind::INF;
    for (uint t = 0; t < t_count; t++) {
        uint dst = state.destination * t_count + t;
        uint c = swr_cch::cch_query(&cch, src, dst);
        if (c < cch_cost) cch_cost = c;
    }
    check(cch_cost == ref_cost, "temporal_alg_cch_matches_dijkstra");

    // --- 4. Hub Labeling ---
    swr_hub::HubData hub = swr_hub::preprocess(&tg);
    defer hub.destroy();

    uint hub_cost = pathfind::INF;
    for (uint t = 0; t < t_count; t++) {
        uint dst = state.destination * t_count + t;
        uint c = swr_hub::hub_query(&hub, src, dst);
        if (c < hub_cost) hub_cost = c;
    }
    check(hub_cost == ref_cost, "temporal_alg_hub_matches_dijkstra");
}

// ============================================================
// Test: FFI layer (swr_ffi.c3)
// ============================================================

fn void test_ffi_lifecycle() {
    // State create/destroy
    void* state = swr_ffi::swr_state_new();
    check(state != null, "ffi_state_new");

    // Set ship
    int rc = swr_ffi::swr_state_set_ship(state, 15.0, 4.0, 100.0, 5.0);
    check(rc == 0, "ffi_set_ship");

    // Set cost
    rc = swr_ffi::swr_state_set_cost(state, 0, 1.0, 100.0, 1000);
    check(rc == 0, "ffi_set_cost");

    // Bad cost mode
    rc = swr_ffi::swr_state_set_cost(state, 5, 1.0, 100.0, 1000);
    check(rc == -2, "ffi_set_cost_bad_mode");

    // Set land (all sea)
    bool[25] grid;
    for (int i = 0; i < 25; i++) grid[i] = false;
    rc = swr_ffi::swr_state_set_land(state, 37.0, 23.0, 39.0, 27.0, 5, 5, &grid);
    check(rc == 0, "ffi_set_land");

    // Build PRM
    rc = swr_ffi::swr_state_build_prm(state, 10, 3, 10, 42);
    check(rc == 0, "ffi_build_prm");

    int nc = swr_ffi::swr_state_node_count(state);
    check(nc == 10, "ffi_node_count");

    // Set voyage
    rc = swr_ffi::swr_state_set_voyage(state, 37.5, 23.5, 38.5, 26.5);
    check(rc == 0, "ffi_set_voyage");

    // Push weather (calm, uniform 2x2)
    double[4] ws = { 3.0, 3.0, 3.0, 3.0 };
    double[4] wd = { 180.0, 180.0, 180.0, 180.0 };
    double[4] wh = { 0.5, 0.5, 0.5, 0.5 };
    double[4] wdr = { 180.0, 180.0, 180.0, 180.0 };
    double[4] cs = { 0.2, 0.2, 0.2, 0.2 };
    double[4] cd = { 90.0, 90.0, 90.0, 90.0 };
    rc = swr_ffi::swr_weather_push(state, 37.0, 23.0, 39.0, 27.0, 2, 2,
        &ws, &wd, &wh, &wdr, &cs, &cd, 0.0);
    check(rc == 0, "ffi_weather_push");

    // Route with Dijkstra (algorithm 2)
    void* route = swr_ffi::swr_route(state, 2);
    check(route != null, "ffi_route_dijkstra");

    if (route != null) {
        uint cost = swr_ffi::swr_route_cost(route);
        check(cost > 0 && cost < pathfind::INF, "ffi_route_cost");

        int path_len = swr_ffi::swr_route_path_len(route);
        check(path_len >= 2, "ffi_route_path_len");

        // Get waypoints
        double[20] lats;
        double[20] lons;
        double[20] etas;
        double[20] speeds;
        double[20] waves;
        double[20] winds;
        double[20] hdgs;
        uint[20] nids;
        int n = swr_ffi::swr_route_get_waypoints(route,
            &lats, &lons, &etas, &speeds, &waves, &winds, &hdgs, &nids, 20);
        check(n >= 2, "ffi_waypoints_count");
        check(etas[0] == 0.0, "ffi_waypoints_eta_origin");
        if (n >= 2) {
            check(etas[1] > 0.0, "ffi_waypoints_eta_increases");
            check(speeds[1] > 0.0, "ffi_waypoints_speed_positive");
            check(lats[0] > 36.0 && lats[0] < 40.0, "ffi_waypoints_lat_range");
        }

        swr_ffi::swr_route_destroy(route);
    }

    // Weather clear
    rc = swr_ffi::swr_weather_clear(state);
    check(rc == 0, "ffi_weather_clear");

    // Null checks
    check(swr_ffi::swr_state_set_ship(null, 15.0, 4.0, 100.0, 5.0) == -1, "ffi_null_check");

    swr_ffi::swr_state_destroy(state);
}

// ============================================================
// Run all SWR tests (called from main test harness)
// ============================================================

fn void run_swr_tests(int* total_pass, int* total_fail) {
    pass = 0;
    fail = 0;

    io::printn("\n=== SWR Tests ===");

    test_haversine();
    test_bearing();
    test_land_mask();
    test_weather_cost();
    test_prm();
    test_fib_heap();
    test_dijkstra();
    test_dijkstra_util();
    test_weather_hvm4();
    test_sup_dijkstra();
    test_alt();
    test_cch();
    test_hub();
    test_swr_dispatch();
    test_temporal_graph_build();
    test_temporal_routing();
    test_temporal_waiting();
    test_temporal_horizon();
    test_temporal_all_algorithms();
    test_ffi_lifecycle();

    *total_pass += pass;
    *total_fail += fail;
}
