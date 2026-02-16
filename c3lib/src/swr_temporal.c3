module swr_temporal;
import swr;
import dijkstra;
import pathfind;
import std::math;

// ============================================================
// Time-Expanded Graph Construction
// ============================================================
//
// Given V spatial nodes and T time steps (interval_hours buckets),
// total nodes = V * T.  Node u at time step t → index u * T + t.
//
// Spatial edges: For PRM edge (u,v) at time step t, compute travel
// cost via swr::compute_edge_cost() using forecast.grids[t], snap
// arrival to next time bucket, add edge in temporal graph.
//
// Waiting edges: (u*T + t) → (u*T + t+1) with configurable cost.

fn pathfind::Graph build_temporal_graph(swr::SwrState* state, swr::WeatherForecast* forecast) {
    uint v = state.prm.n;
    uint t_count = forecast.count;
    uint total = v * t_count;

    pathfind::Graph tg;
    tg.init(total);

    // Spatial edges: for each time step, for each PRM edge
    for (uint t = 0; t < t_count; t++) {
        swr::WeatherGrid* grid = &forecast.grids[t];

        for (uint ei = 0; ei < state.prm.edge_count; ei++) {
            uint from = state.prm.edge_from[ei];
            uint to = state.prm.edge_to[ei];

            // Sample weather at edge midpoint
            pathfind::LatLon mid = {
                .lat = (state.prm.waypoints[from].pos.lat + state.prm.waypoints[to].pos.lat) / 2.0,
                .lon = (state.prm.waypoints[from].pos.lon + state.prm.waypoints[to].pos.lon) / 2.0
            };
            swr::WeatherCell w = grid.sample(mid);

            double cost = swr::compute_edge_cost(
                &state.ship, w,
                state.prm.edge_dist_m[ei],
                state.prm.edge_heading[ei],
                &state.cost_cfg
            );

            // Snap arrival to time bucket based on travel time
            double cost_hours = cost;
            if (state.cost_cfg.mode != swr::CostMode.TIME_ONLY) {
                // For non-time modes, estimate travel hours from distance and speed
                double dist_nm = state.prm.edge_dist_m[ei] / swr::NM_TO_M;
                cost_hours = dist_nm / state.ship.base_speed;
            }
            // Floor: fast edges (< interval) stay in same time bucket
            uint steps = (uint)(cost_hours / (double)forecast.interval_hours);
            uint arrival_t = t + steps;
            if (arrival_t >= t_count) continue; // beyond horizon

            uint weight = (uint)(cost * (double)state.cost_cfg.scale);
            if (weight < 1) weight = 1;
            if (weight > swr::MAX_EDGE_WEIGHT) weight = swr::MAX_EDGE_WEIGHT;

            tg.add_edge(from * t_count + t, to * t_count + arrival_t, weight);
        }
    }

    // Waiting edges: stay at same node, advance one time step
    for (uint u = 0; u < v; u++) {
        for (uint t = 0; t + 1 < t_count; t++) {
            tg.add_edge(u * t_count + t, u * t_count + t + 1, forecast.wait_cost);
        }
    }

    return tg;
}

// ============================================================
// Spatial Path Extraction
// ============================================================

fn uint[] extract_spatial_path(uint[] temporal_path, usz path_len, uint t_count) {
    if (path_len == 0) {
        uint[] empty = mem::new_array(uint, 1);
        return empty[:0];
    }

    uint[] spatial = mem::new_array(uint, path_len);
    uint slen = 0;

    uint prev_node = temporal_path[0] / t_count;
    spatial[slen++] = prev_node;

    for (usz i = 1; i < path_len; i++) {
        uint node = temporal_path[i] / t_count;
        if (node != prev_node) {
            spatial[slen++] = node;
            prev_node = node;
        }
    }

    return spatial[:slen];
}

// ============================================================
// Temporal Routing Entry Point
// ============================================================

fn swr::RouteResult? swr_route_temporal(swr::SwrState* state, swr::WeatherForecast* forecast) {
    if (forecast == null || forecast.count == 0) return swr::SWR_ERROR~;

    uint v = state.prm.n;
    uint t_count = forecast.count;
    uint total = v * t_count;

    // Build time-expanded graph
    pathfind::Graph tg = build_temporal_graph(state, forecast);
    defer tg.destroy();

    // Run Dijkstra from origin at t=0
    uint src = state.origin * t_count; // origin at time 0
    dijkstra::DijkResult dr = dijkstra::sssp(&tg, src)!;

    // Find minimum cost across all arrival times at destination
    uint best_cost = pathfind::INF;
    uint best_t = 0;
    for (uint t = 0; t < t_count; t++) {
        uint idx = state.destination * t_count + t;
        if (dr.dist[idx] < best_cost) {
            best_cost = dr.dist[idx];
            best_t = t;
        }
    }

    if (best_cost >= pathfind::INF) {
        dr.destroy();
        return swr::SWR_ERROR~;
    }

    // Extract temporal path
    uint dst_temporal = state.destination * t_count + best_t;
    uint[] temporal_path = dijkstra::extract_path(dr.pred, src, dst_temporal, total)!;
    usz temporal_len = temporal_path.len;

    // Convert to spatial path (deduplicate waiting)
    uint[] spatial_path = extract_spatial_path(temporal_path, temporal_len, t_count);
    free(temporal_path.ptr);

    // Transfer dist ownership to result, free pred separately
    uint[] dist = dr.dist;
    free(dr.pred.ptr);

    return (swr::RouteResult){
        .path = spatial_path,
        .path_len = spatial_path.len,
        .total_cost = best_cost,
        .distances = dist,
        .dist_count = total
    };
}
