module swr_sup;
import swr;
import swr_dijkstra;
import pathfind;
import std::core::dstring;

alias DStr = dstring::DString;

// ============================================================
// Multi-Scenario Superposition
// ============================================================

enum ScenarioStrategy : char {
    BEST_EXPECTED,
    MINIMAX,
    BEST_SINGLE
}

// Run Bellman-Ford once with superposed edge weights for N weather scenarios.
// Returns N costs (one per scenario) via HVM4 collapse.
fn uint[]? swr_replan_multi(
    swr::SwrState* state,
    swr::WeatherGrid[] scenarios,
    uint n_scenarios
) {
    if (n_scenarios == 0 || n_scenarios > 16) return swr::SWR_ERROR~;

    uint edge_count = state.prm.edge_count;
    uint n = state.prm.n;

    // Compute edge weights for each scenario
    uint[][] scenario_weights = mem::new_array(uint[], n_scenarios);
    defer {
        for (uint s = 0; s < n_scenarios; s++) {
            if (scenario_weights[s].ptr != null) free(scenario_weights[s].ptr);
        }
        free(scenario_weights.ptr);
    }

    for (uint s = 0; s < n_scenarios; s++) {
        scenario_weights[s] = mem::new_array(uint, edge_count);
        for (uint i = 0; i < edge_count; i++) {
            uint from = state.prm.edge_from[i];
            uint to = state.prm.edge_to[i];
            swr::LatLon mid = {
                .lat = (state.prm.waypoints[from].pos.lat + state.prm.waypoints[to].pos.lat) / 2.0,
                .lon = (state.prm.waypoints[from].pos.lon + state.prm.waypoints[to].pos.lon) / 2.0
            };
            swr::WeatherCell w = scenarios[s].sample(mid);
            double cost = swr::compute_edge_cost(
                &state.ship, w, state.prm.edge_dist_m[i],
                state.prm.edge_heading[i], &state.cost_cfg);
            uint weight = (uint)(cost * (double)state.cost_cfg.scale);
            if (weight < 1) weight = 1;
            if (weight > 900_000) weight = 900_000;
            scenario_weights[s][i] = weight;
        }
    }

    // Generate HVM4 source with superposed edge weights
    pathfind::hvm4_lib_reset();

    DStr ds = dstring::new(mem);
    defer ds.free();

    uint depth = pathfind::ceil_log16(n);

    // Constants
    ds.append_string("@INF = 999999\n");
    ds.appendf("@DEPTH = %d\n", depth);

    // Trie definitions
    pathfind::emit_trie_defs(&ds);

    // Helpers (same as bellman_ford)
    ds.append_string("@min = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: b; \xce\xbbn. a}(a < b)\n");
    ds.append_string("@relax_edge = \xce\xbb&dist. \xce\xbb{#E3: \xce\xbb&u. \xce\xbb&v. \xce\xbbw. ! &du = @trie_get(u, @DEPTH, dist); ! &new_d = du + w; ! &dv = @trie_get(v, @DEPTH, dist); \xce\xbb{0: dist; \xce\xbbn. @trie_set(v, new_d, @DEPTH, dist)}(new_d < dv)}\n");
    ds.append_string("@foldl = \xce\xbb&f. \xce\xbb&acc. \xce\xbb{[]: acc; <>: \xce\xbbh. \xce\xbbt. @foldl(f, f(acc, h), t)}\n");
    ds.append_string("@relax_round = \xce\xbbdist. @foldl(@relax_edge, dist, @edges)\n");
    ds.append_string("@repeat = \xce\xbb&f. \xce\xbb&x. \xce\xbb{0: x; \xce\xbbn. @repeat(f, f(x), n - 1)}\n");

    // Edge list with superposed weights
    ds.append_string("@edges = [");
    for (uint i = 0; i < edge_count; i++) {
        if (i > 0) ds.append_string(", ");
        ds.appendf("#E3{%d,%d,", state.prm.edge_from[i], state.prm.edge_to[i]);
        emit_sup_weight(&ds, scenario_weights, i, n_scenarios);
        ds.append_string("}");
    }
    ds.append_string("]\n");

    // Initial distance
    ds.appendf("@init_dist = @trie_set(%d, 0, @DEPTH, #HE{})\n", state.origin);

    // Run V-1 rounds
    uint rounds = n > 1 ? n - 1 : 1;
    ds.appendf("@bf = @repeat(@relax_round, @init_dist, %d)\n", rounds);

    // Extract destination distance (superposed result)
    ds.appendf("@main = @trie_get(%d, @DEPTH, @bf)\n", state.destination);

    // Run HVM4 with collapse to get N scenario results
    uint max_out = n_scenarios * 2;
    uint[] out_buf = mem::new_array(uint, max_out);
    int count = pathfind::hvm4_run(ds.zstr_view(), (int)max_out, out_buf.ptr, (int)max_out);

    if (count < 0) {
        free(out_buf.ptr);
        return pathfind::HVM_ERROR~;
    }

    // Copy to right-sized array
    uint[] result = mem::new_array(uint, (usz)count);
    for (int i = 0; i < count; i++) {
        result[i] = out_buf[i];
    }
    free(out_buf.ptr);

    return result;
}

// Encode N scenario weights as nested SUPs: &A{w0, &B{w1, &C{w2, ...}}}
fn void emit_sup_weight(DStr* ds, uint[][] weights, uint edge_idx, uint n) {
    if (n == 1) {
        ds.appendf("%d", weights[0][edge_idx]);
        return;
    }
    for (uint s = 0; s < n - 1; s++) {
        char label = (char)('A' + s);
        ds.appendf("&%c{%d, ", label, weights[s][edge_idx]);
    }
    ds.appendf("%d", weights[n - 1][edge_idx]);
    for (uint s = 0; s < n - 1; s++) {
        ds.append_string("}");
    }
}

// Select best route from N scenario costs based on strategy.
fn uint select_route(uint[] costs, uint n_scenarios, ScenarioStrategy strategy) {
    if (n_scenarios == 0) return pathfind::INF;

    if (strategy == BEST_EXPECTED) {
        uint sum = 0;
        for (uint i = 0; i < n_scenarios; i++) {
            sum += costs[i];
        }
        return sum / n_scenarios;
    }
    if (strategy == MINIMAX) {
        uint worst = 0;
        for (uint i = 0; i < n_scenarios; i++) {
            if (costs[i] > worst) worst = costs[i];
        }
        return worst;
    }
    // BEST_SINGLE: return minimum cost
    uint best = pathfind::INF;
    for (uint i = 0; i < n_scenarios; i++) {
        if (costs[i] < best) best = costs[i];
    }
    return best;
}

// Select best scenario INDEX based on strategy.
fn uint select_scenario_idx(uint[] costs, uint n_scenarios, ScenarioStrategy strategy) {
    if (n_scenarios == 0) return 0;

    if (strategy == BEST_EXPECTED) {
        // Expected value — all scenarios contribute equally, pick the one closest to average
        uint sum = 0;
        for (uint i = 0; i < n_scenarios; i++) sum += costs[i];
        uint avg = sum / n_scenarios;
        uint best_idx = 0;
        uint best_diff = pathfind::INF;
        for (uint i = 0; i < n_scenarios; i++) {
            uint diff = costs[i] > avg ? costs[i] - avg : avg - costs[i];
            if (diff < best_diff) { best_diff = diff; best_idx = i; }
        }
        return best_idx;
    }
    if (strategy == MINIMAX) {
        // Worst-case — pick scenario with highest cost, then Dijkstra routes around it
        uint worst_idx = 0;
        uint worst = 0;
        for (uint i = 0; i < n_scenarios; i++) {
            if (costs[i] > worst) { worst = costs[i]; worst_idx = i; }
        }
        return worst_idx;
    }
    // BEST_SINGLE: pick scenario with minimum cost
    uint best_idx = 0;
    uint best = pathfind::INF;
    for (uint i = 0; i < n_scenarios; i++) {
        if (costs[i] < best) { best = costs[i]; best_idx = i; }
    }
    return best_idx;
}

// ============================================================
// SUP → Dijkstra Pipeline
// ============================================================
//
// 1. HVM4 superposition evaluates all weather scenarios in one run
//    (via swr_replan_multi), exploiting shared computation.
// 2. Strategy selects which scenario to route for.
// 3. C3 Dijkstra (Fibonacci heap) computes the actual optimal path
//    with predecessor tracking for the selected scenario.
//
// This gives the best of both worlds:
//   - HVM4 SUP for fast multi-scenario evaluation
//   - C3 Dijkstra for optimal path extraction with O((V+E)log V)

fn swr::RouteResult? swr_sup_dijkstra(
    swr::SwrState* state,
    swr::WeatherGrid[] scenarios,
    uint n_scenarios,
    ScenarioStrategy strategy
) {
    // Step 1: Run HVM4 SUP to evaluate all scenarios at once
    uint[] costs = swr_replan_multi(state, scenarios, n_scenarios)!!;
    defer free(costs.ptr);

    // Step 2: Select best scenario based on strategy
    uint best_idx = select_scenario_idx(costs, (uint)costs.len, strategy);

    // Step 3: Run C3 Dijkstra on the selected scenario's weather
    return swr_dijkstra::swr_dijkstra(state, &scenarios[best_idx]);
}
