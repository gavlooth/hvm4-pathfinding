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

// Run APSP once with superposed edge weights for N weather scenarios.
// Returns N costs (one per scenario) via HVM4 collapse.
// Uses tree-parallel matrix squaring under tropical (min, +) semiring.
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
            pathfind::LatLon mid = {
                .lat = (state.prm.waypoints[from].pos.lat + state.prm.waypoints[to].pos.lat) / 2.0,
                .lon = (state.prm.waypoints[from].pos.lon + state.prm.waypoints[to].pos.lon) / 2.0
            };
            swr::WeatherCell w = scenarios[s].sample(mid);
            double cost = swr::compute_edge_cost(
                &state.ship, w, state.prm.edge_dist_m[i],
                state.prm.edge_heading[i], &state.cost_cfg);
            uint weight = (uint)(cost * (double)state.cost_cfg.scale);
            if (weight < 1) weight = 1;
            if (weight > swr::MAX_EDGE_WEIGHT) weight = swr::MAX_EDGE_WEIGHT;
            scenario_weights[s][i] = weight;
        }
    }

    // Generate HVM4 source with tree-parallel APSP + SUP weights
    pathfind::hvm4_lib_reset();

    DStr ds = dstring::new(mem);
    defer ds.free();

    uint depth = pathfind::ceil_log2(n);

    // Constants
    ds.append_string("@INF = 999999\n");
    ds.appendf("@TD = %d\n", depth);
    ds.appendf("@N = %d\n", n);

    // Tropical operations
    ds.append_string("@trop_add = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: b; \xce\xbbn. a}(a < b)\n");
    ds.append_string("@trop_mul = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: \xce\xbb{0: a + b; \xce\xbbn. @INF}(b == @INF); \xce\xbbn. @INF}(a == @INF)\n");

    // Tree operations (Bend-style parallel)
    pathfind::emit_tree_ops(&ds);

    // Dot product, column extraction, row builder, mat_mul
    ds.append_string("@dot = \xce\xbb&row. \xce\xbb&cv. @tf(@TD, \xce\xbba. \xce\xbbb. @trop_add(a, b), @tz(@TD, \xce\xbba. \xce\xbbb. @trop_mul(a, b), row, cv))\n");
    ds.append_string("@col = \xce\xbb&j. \xce\xbb&mat. @tm(@TD, \xce\xbbrow. @tg(@TD, j, row), mat)\n");
    ds.append_string("@brr = \xce\xbb{0: \xce\xbb&i. \xce\xbb&s. \xce\xbb&lim. \xce\xbb&row. \xce\xbb&B. \xce\xbb{0: @INF; \xce\xbbm. @dot(row, @col(i, B))}(i < lim); \xce\xbbn. ! &d = n; \xce\xbb&i. \xce\xbb&s. \xce\xbb&lim. \xce\xbb&row. \xce\xbb&B. ! &ns = s * 2; #P{@brr(d - 1, i, ns, lim, row, B), @brr(d - 1, i + s, ns, lim, row, B)}}\n");
    ds.append_string("@mat_mul = \xce\xbb&A. \xce\xbb&B. @tm(@TD, \xce\xbb&row. @brr(@TD, 0, 1, @N, row, B), A)\n");

    // Build adjacency matrix with SUP-labeled weights
    // Precompute: for each (i,j), find the edge index (or -1)
    int* edge_idx = (int*)mem::calloc((usz)n * n * int.sizeof);
    defer free(edge_idx);
    for (uint i = 0; i < n * n; i++) edge_idx[i] = -1;
    for (uint e = 0; e < edge_count; e++) {
        uint u = state.prm.edge_from[e];
        uint v = state.prm.edge_to[e];
        edge_idx[u * n + v] = (int)e;
    }

    // Emit row value lookup with SUP weights
    for (uint i = 0; i < n; i++) {
        ds.appendf("@rv_%d = \xce\xbb{", i);
        for (uint j = 0; j < n; j++) {
            ds.appendf("%d: ", j);
            if (i == j) {
                ds.append_string("0");
            } else if (edge_idx[i * n + j] >= 0) {
                emit_sup_weight(&ds, scenario_weights, (uint)edge_idx[i * n + j], n_scenarios);
            } else {
                ds.append_string("@INF");
            }
            ds.append_string("; ");
        }
        ds.append_string("\xce\xbbn. @INF}\n");
    }

    // Row builders and matrix
    for (uint i = 0; i < n; i++) {
        ds.appendf("@row_%d = @bld(@TD, 0, 1, @N, @rv_%d)\n", i, i);
    }
    ds.append_string("@mrv = \xce\xbb{");
    for (uint i = 0; i < n; i++) {
        ds.appendf("%d: @row_%d; ", i, i);
    }
    ds.append_string("\xce\xbbn. @bld_inf(@TD)}\n");
    ds.append_string("@D_init = @bld(@TD, 0, 1, @N, @mrv)\n");

    // Repeated squaring
    uint rounds = pathfind::ceil_log2(n);
    ds.append_string("@repeat = \xce\xbb&f. \xce\xbb&x. \xce\xbb{0: x; \xce\xbbn. @repeat(f, f(x), n - 1)}\n");
    ds.appendf("@result = @repeat(\xce\xbb&m. @mat_mul(m, m), @D_init, %d)\n", rounds);

    // Extract result[origin][destination] — a superposed value
    ds.appendf("@main = @tg(@TD, %d, @tg(@TD, %d, @result))\n",
        state.destination, state.origin);

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
