module swr_test;
import swr;
import swr_ffi;
import swr_dijkstra;
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
// Test: SWR dispatch (Dijkstra only)
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

    // Dijkstra via dispatch
    swr::RouteResult dij_result = swr::swr_replan(&state, &weather, swr::Algorithm.DIJKSTRA)!!;
    check(dij_result.total_cost > 0 && dij_result.total_cost < pathfind::INF, "swr_dispatch_dijkstra");
    check(dij_result.path_len >= 2, "swr_dispatch_dijkstra_path");
    dij_result.destroy();
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

    // Route with Dijkstra (algorithm index 0)
    void* route = swr_ffi::swr_route(state, 0);
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
    test_swr_dispatch();
    test_ffi_lifecycle();

    *total_pass += pass;
    *total_fail += fail;
}
