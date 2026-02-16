module swr;
import pathfind;
import swr_dijkstra;
import swr_alt;
import swr_cch;
import swr_hub;
import swr_temporal;
import std::math;
import std::io;

// ============================================================
// Constants
// ============================================================

const double NM_TO_M = 1852.0;
const uint MAX_EDGE_WEIGHT = 900_000;

faultdef SWR_ERROR, PRM_DISCONNECTED, LAND_COLLISION;

// ============================================================
// Core Types
// ============================================================

alias LatLon = pathfind::LatLon;

struct Waypoint {
    uint id;
    LatLon pos;
}

struct SeaBounds {
    LatLon sw;
    LatLon ne;
}

struct LandMask {
    SeaBounds bounds;
    uint rows;
    uint cols;
    bool[] grid;  // row-major, true = land
}

struct WeatherCell {
    double wind_speed;
    double wind_dir;
    double wave_height;
    double wave_dir;
    double current_speed;
    double current_dir;
}

struct WeatherGrid {
    SeaBounds bounds;
    uint rows;
    uint cols;
    WeatherCell[] cells;
    double timestamp;
}

struct WeatherForecast {
    WeatherGrid[] grids;
    uint count;
    uint interval_hours;  // typically 6
    uint wait_cost;       // cost of waiting one time step at a node
}

fn void WeatherForecast.destroy(&self) {
    if (self.grids.ptr != null) free(self.grids.ptr);
}

struct ShipModel {
    double base_speed;
    double max_wave_ht;
    double fuel_rate_base;
    double fuel_rate_per_kt;
}

enum CostMode : char {
    TIME_ONLY,
    FUEL_ONLY,
    WEIGHTED_SUM,
    SAFETY_PENALIZED
}

struct CostConfig {
    CostMode mode;
    double alpha;
    double safety_penalty;
    uint scale;
}

enum Algorithm : char {
    APSP,
    ENUMERATE,
    DIJKSTRA,
    ALT,
    CCH,
    HUB_LABEL,
    TEMPORAL
}

struct RouteResult {
    uint[] path;
    usz path_len;
    uint total_cost;
    uint[] distances;
    usz dist_count;
}

// ============================================================
// LCG Random Number Generator
// ============================================================

struct Lcg {
    ulong state;
}

fn Lcg lcg_new(ulong seed) {
    return (Lcg){ .state = seed | 1 };
}

fn uint Lcg.next(&self) {
    self.state = self.state * 6364136223846793005 + 1442695040888963407;
    return (uint)(self.state >> 33);
}

fn double Lcg.next_double(&self) {
    return (double)self.next() / 2147483648.0;
}

// ============================================================
// PRM Graph
// ============================================================

struct PrmGraph {
    Waypoint[] waypoints;
    uint n;
    uint k_neighbors;
    pathfind::Graph graph;
    uint[] edge_from;
    uint[] edge_to;
    double[] edge_dist_m;
    double[] edge_heading;
    uint[] edge_weight;
    uint edge_count;
    uint edge_capacity;
}

fn void PrmGraph.destroy(&self) {
    self.graph.destroy();
    if (self.waypoints.ptr != null) free(self.waypoints.ptr);
    if (self.edge_from.ptr != null) free(self.edge_from.ptr);
    if (self.edge_to.ptr != null) free(self.edge_to.ptr);
    if (self.edge_dist_m.ptr != null) free(self.edge_dist_m.ptr);
    if (self.edge_heading.ptr != null) free(self.edge_heading.ptr);
    if (self.edge_weight.ptr != null) free(self.edge_weight.ptr);
}

// ============================================================
// SWR State
// ============================================================

struct SwrState {
    PrmGraph prm;
    ShipModel ship;
    LandMask land;
    CostConfig cost_cfg;
    uint origin;
    uint destination;
    WeatherForecast* forecast;
}

fn void SwrState.destroy(&self) {
    self.prm.destroy();
}

fn void RouteResult.destroy(&self) {
    if (self.path.ptr != null) free(self.path.ptr);
    if (self.distances.ptr != null) free(self.distances.ptr);
}

// ============================================================
// Helpers
// ============================================================

fn double dabs(double x) {
    return x < 0.0 ? -x : x;
}

fn double bilerp(double q00, double q01, double q10, double q11, double dr, double dc) {
    double top = q00 * (1.0 - dc) + q01 * dc;
    double bot = q10 * (1.0 - dc) + q11 * dc;
    return top * (1.0 - dr) + bot * dr;
}

fn double clamp_d(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// ============================================================
// Geography (delegates to pathfind module)
// ============================================================

fn double haversine(LatLon a, LatLon b) {
    return pathfind::haversine(a, b);
}

fn double bearing(LatLon a, LatLon b) {
    return pathfind::bearing(a, b);
}

// ============================================================
// LandMask Methods
// ============================================================

fn bool LandMask.is_land(&self, LatLon pos) {
    if (pos.lat < self.bounds.sw.lat || pos.lat > self.bounds.ne.lat ||
        pos.lon < self.bounds.sw.lon || pos.lon > self.bounds.ne.lon) {
        return false;
    }
    double frac_row = (pos.lat - self.bounds.sw.lat) / (self.bounds.ne.lat - self.bounds.sw.lat) * (double)(self.rows - 1);
    double frac_col = (pos.lon - self.bounds.sw.lon) / (self.bounds.ne.lon - self.bounds.sw.lon) * (double)(self.cols - 1);
    uint row = (uint)frac_row;
    uint col = (uint)frac_col;
    if (row >= self.rows) row = self.rows - 1;
    if (col >= self.cols) col = self.cols - 1;
    return self.grid[row * self.cols + col];
}

fn bool LandMask.segment_clear(&self, LatLon a, LatLon b, uint steps) {
    for (uint i = 0; i <= steps; i++) {
        double t = (double)i / (double)steps;
        LatLon p = {
            .lat = a.lat + t * (b.lat - a.lat),
            .lon = a.lon + t * (b.lon - a.lon)
        };
        if (self.is_land(p)) return false;
    }
    return true;
}

// ============================================================
// WeatherGrid Methods
// ============================================================

fn WeatherCell WeatherGrid.sample(&self, LatLon pos) {
    double frac_row = (pos.lat - self.bounds.sw.lat) / (self.bounds.ne.lat - self.bounds.sw.lat) * (double)(self.rows - 1);
    double frac_col = (pos.lon - self.bounds.sw.lon) / (self.bounds.ne.lon - self.bounds.sw.lon) * (double)(self.cols - 1);

    frac_row = clamp_d(frac_row, 0.0, (double)(self.rows - 1) - 0.001);
    frac_col = clamp_d(frac_col, 0.0, (double)(self.cols - 1) - 0.001);

    uint r0 = (uint)frac_row;
    uint c0 = (uint)frac_col;
    uint r1 = r0 + 1;
    uint c1 = c0 + 1;
    if (r1 >= self.rows) r1 = self.rows - 1;
    if (c1 >= self.cols) c1 = self.cols - 1;

    double dr = frac_row - (double)r0;
    double dc = frac_col - (double)c0;

    WeatherCell* q00 = &self.cells[r0 * self.cols + c0];
    WeatherCell* q01 = &self.cells[r0 * self.cols + c1];
    WeatherCell* q10 = &self.cells[r1 * self.cols + c0];
    WeatherCell* q11 = &self.cells[r1 * self.cols + c1];

    return (WeatherCell){
        .wind_speed    = bilerp(q00.wind_speed, q01.wind_speed, q10.wind_speed, q11.wind_speed, dr, dc),
        .wind_dir      = bilerp(q00.wind_dir, q01.wind_dir, q10.wind_dir, q11.wind_dir, dr, dc),
        .wave_height   = bilerp(q00.wave_height, q01.wave_height, q10.wave_height, q11.wave_height, dr, dc),
        .wave_dir      = bilerp(q00.wave_dir, q01.wave_dir, q10.wave_dir, q11.wave_dir, dr, dc),
        .current_speed = bilerp(q00.current_speed, q01.current_speed, q10.current_speed, q11.current_speed, dr, dc),
        .current_dir   = bilerp(q00.current_dir, q01.current_dir, q10.current_dir, q11.current_dir, dr, dc)
    };
}

// ============================================================
// PRM Construction
// ============================================================

fn void swr_build_prm(SwrState* state, uint n_samples, uint k_neighbors, uint collision_steps, ulong seed) {
    Lcg rng = lcg_new(seed);

    Waypoint[] wps = mem::new_array(Waypoint, n_samples);
    uint count = 0;
    uint max_attempts = n_samples * 10;

    double lat_range = state.land.bounds.ne.lat - state.land.bounds.sw.lat;
    double lon_range = state.land.bounds.ne.lon - state.land.bounds.sw.lon;

    for (uint attempt = 0; attempt < max_attempts && count < n_samples; attempt++) {
        LatLon pos = {
            .lat = state.land.bounds.sw.lat + rng.next_double() * lat_range,
            .lon = state.land.bounds.sw.lon + rng.next_double() * lon_range
        };
        if (!state.land.is_land(pos)) {
            wps[count] = (Waypoint){ .id = count, .pos = pos };
            count++;
        }
    }

    state.prm.waypoints = wps;
    state.prm.n = count;
    state.prm.k_neighbors = k_neighbors;
    state.prm.graph.init(count);

    connect_knn(state, k_neighbors, collision_steps);
}

fn void connect_knn(SwrState* state, uint k, uint collision_steps) {
    uint n = state.prm.n;

    uint max_edges = n * k * 2;
    state.prm.edge_from = mem::new_array(uint, max_edges);
    state.prm.edge_to = mem::new_array(uint, max_edges);
    state.prm.edge_dist_m = mem::new_array(double, max_edges);
    state.prm.edge_heading = mem::new_array(double, max_edges);
    state.prm.edge_weight = mem::new_array(uint, max_edges);
    state.prm.edge_count = 0;
    state.prm.edge_capacity = max_edges;

    double[] dists = mem::new_array(double, n);
    uint[] indices = mem::new_array(uint, n);
    defer { free(dists.ptr); free(indices.ptr); }

    for (uint i = 0; i < n; i++) {
        for (uint j = 0; j < n; j++) {
            dists[j] = haversine(state.prm.waypoints[i].pos, state.prm.waypoints[j].pos);
            indices[j] = j;
        }
        dists[i] = 1e18;

        uint found = 0;
        for (uint pass = 0; pass < n - 1 && found < k; pass++) {
            uint min_idx = pass;
            for (uint q = pass + 1; q < n; q++) {
                if (dists[q] < dists[min_idx]) min_idx = q;
            }
            double tmp_d = dists[pass]; dists[pass] = dists[min_idx]; dists[min_idx] = tmp_d;
            uint tmp_i = indices[pass]; indices[pass] = indices[min_idx]; indices[min_idx] = tmp_i;

            uint j = indices[pass];
            double dist = dists[pass];
            if (dist >= 1e17) break;

            if (!state.land.segment_clear(state.prm.waypoints[i].pos, state.prm.waypoints[j].pos, collision_steps)) {
                continue;
            }

            add_prm_edge(state, i, j, dist);
            found++;
        }
    }

    for (uint i = 0; i < state.prm.edge_count; i++) {
        state.prm.graph.add_edge(state.prm.edge_from[i], state.prm.edge_to[i], state.prm.edge_weight[i]);
    }
}

fn void add_prm_edge(SwrState* state, uint from, uint to, double dist_m) {
    for (uint i = 0; i < state.prm.edge_count; i++) {
        if (state.prm.edge_from[i] == from && state.prm.edge_to[i] == to) return;
    }
    if (state.prm.edge_count + 1 >= state.prm.edge_capacity) return;

    uint idx = state.prm.edge_count;
    state.prm.edge_from[idx] = from;
    state.prm.edge_to[idx] = to;
    state.prm.edge_dist_m[idx] = dist_m;
    state.prm.edge_heading[idx] = bearing(state.prm.waypoints[from].pos, state.prm.waypoints[to].pos);
    state.prm.edge_weight[idx] = 1;
    state.prm.edge_count++;

    // Reverse edge
    for (uint i = 0; i < state.prm.edge_count; i++) {
        if (state.prm.edge_from[i] == to && state.prm.edge_to[i] == from) return;
    }
    if (state.prm.edge_count >= state.prm.edge_capacity) return;

    uint idx2 = state.prm.edge_count;
    state.prm.edge_from[idx2] = to;
    state.prm.edge_to[idx2] = from;
    state.prm.edge_dist_m[idx2] = dist_m;
    state.prm.edge_heading[idx2] = bearing(state.prm.waypoints[to].pos, state.prm.waypoints[from].pos);
    state.prm.edge_weight[idx2] = 1;
    state.prm.edge_count++;
}

fn uint find_nearest_waypoint(PrmGraph* prm, LatLon pos) {
    uint best = 0;
    double best_dist = 1e18;
    for (uint i = 0; i < prm.n; i++) {
        double d = haversine(pos, prm.waypoints[i].pos);
        if (d < best_dist) {
            best_dist = d;
            best = i;
        }
    }
    return best;
}

fn void swr_set_voyage(SwrState* state, LatLon origin, LatLon destination) {
    state.origin = find_nearest_waypoint(&state.prm, origin);
    state.destination = find_nearest_waypoint(&state.prm, destination);
}

// ============================================================
// Weather-to-Weight Conversion
// ============================================================

fn double compute_edge_cost(ShipModel* ship, WeatherCell weather, double dist_m, double heading_deg, CostConfig* cfg) {
    double wind_angle = dabs(weather.wind_dir - heading_deg);
    if (wind_angle > 180.0) wind_angle = 360.0 - wind_angle;

    double wave_ratio = weather.wave_height / ship.max_wave_ht;
    if (wave_ratio > 1.0) wave_ratio = 1.0;
    double reduction = 1.0 - 0.5 * wave_ratio * wave_ratio;
    reduction = clamp_d(reduction, 0.1, 1.0);

    double current_angle = (weather.current_dir - heading_deg) * pathfind::DEG_TO_RAD;
    double current_along = weather.current_speed * math::cos(current_angle) * 1.94384;

    double eff_speed = ship.base_speed * reduction + current_along;
    if (eff_speed < 0.1) eff_speed = 0.1;

    double dist_nm = dist_m / NM_TO_M;
    double time_h = dist_nm / eff_speed;
    double fuel = time_h * (ship.fuel_rate_base + ship.fuel_rate_per_kt * eff_speed);

    double cost;
    if (cfg.mode == TIME_ONLY) {
        cost = time_h;
    } else if (cfg.mode == FUEL_ONLY) {
        cost = fuel;
    } else if (cfg.mode == WEIGHTED_SUM) {
        cost = cfg.alpha * time_h + (1.0 - cfg.alpha) * fuel;
    } else {
        cost = cfg.alpha * time_h + (1.0 - cfg.alpha) * fuel;
        if (weather.wave_height > ship.max_wave_ht) {
            cost += cfg.safety_penalty;
        }
    }

    return cost;
}

fn void update_all_weights(SwrState* state, WeatherGrid* weather) {
    for (uint i = 0; i < state.prm.edge_count; i++) {
        uint from = state.prm.edge_from[i];
        uint to = state.prm.edge_to[i];

        LatLon mid = {
            .lat = (state.prm.waypoints[from].pos.lat + state.prm.waypoints[to].pos.lat) / 2.0,
            .lon = (state.prm.waypoints[from].pos.lon + state.prm.waypoints[to].pos.lon) / 2.0
        };
        WeatherCell w = weather.sample(mid);

        double cost = compute_edge_cost(&state.ship, w, state.prm.edge_dist_m[i], state.prm.edge_heading[i], &state.cost_cfg);

        uint weight = (uint)(cost * (double)state.cost_cfg.scale);
        if (weight < 1) weight = 1;
        if (weight > MAX_EDGE_WEIGHT) weight = MAX_EDGE_WEIGHT;
        state.prm.edge_weight[i] = weight;
    }

    rebuild_graph(state);
}

// ============================================================
// HVM4 Weather-to-Weight Pipeline (Pass 1)
// ============================================================

fn uint[]? compute_weights_hvm4(SwrState* state, WeatherGrid* weather) {
    uint e = state.prm.edge_count;
    if (e == 0) return SWR_ERROR~;

    // Allocate per-edge weather sample arrays
    double[] wind_speed = mem::new_array(double, e);
    double[] wind_dir   = mem::new_array(double, e);
    double[] wave_ht    = mem::new_array(double, e);
    double[] cur_speed  = mem::new_array(double, e);
    double[] cur_dir    = mem::new_array(double, e);
    defer {
        free(wind_speed.ptr);
        free(wind_dir.ptr);
        free(wave_ht.ptr);
        free(cur_speed.ptr);
        free(cur_dir.ptr);
    }

    // Sample weather at each edge midpoint
    for (uint i = 0; i < e; i++) {
        uint from = state.prm.edge_from[i];
        uint to   = state.prm.edge_to[i];
        LatLon mid = {
            .lat = (state.prm.waypoints[from].pos.lat + state.prm.waypoints[to].pos.lat) / 2.0,
            .lon = (state.prm.waypoints[from].pos.lon + state.prm.waypoints[to].pos.lon) / 2.0
        };
        WeatherCell w = weather.sample(mid);
        wind_speed[i] = w.wind_speed;
        wind_dir[i]   = w.wind_dir;
        wave_ht[i]    = w.wave_height;
        cur_speed[i]  = w.current_speed;
        cur_dir[i]    = w.current_dir;
    }

    // Register weather FFI
    pathfind::hvm4_lib_reset();
    pathfind::hvm4_weather_setup(
        state.prm.edge_dist_m.ptr, state.prm.edge_heading.ptr,
        wind_speed.ptr, wind_dir.ptr, wave_ht.ptr,
        cur_speed.ptr, cur_dir.ptr,
        state.ship.base_speed, state.ship.max_wave_ht,
        state.ship.fuel_rate_base, state.ship.fuel_rate_per_kt,
        (uint)state.cost_cfg.mode, state.cost_cfg.alpha,
        state.cost_cfg.safety_penalty, state.cost_cfg.scale,
        e
    );

    // Generate HVM4 source: balanced binary tree of %edge_cost(i) calls
    // Uses bifurcation pattern (bend) for parallel work-stealing.
    // @build(depth, index) forks into two halves at each level;
    // leaves call %edge_cost(i) via FFI, @cat merges result lists.
    pathfind::DStr ds = pathfind::new_dstr();
    defer ds.free();

    // Compute tree depth: ceil(log2(e))
    uint depth = 0;
    uint cap = 1;
    while (cap < e) { depth++; cap *= 2; }

    ds.append_string("@cat = \xce\xbb{[]: \xce\xbbys. ys; <>: \xce\xbbh. \xce\xbbt. \xce\xbbys. h <> @cat(t, ys)}\n");
    ds.append_string("@build = \xce\xbb{0: \xce\xbb&i. \xce\xbblimit. @build_base(i < limit, i);\n");
    ds.append_string("            \xce\xbbn. \xce\xbbi. \xce\xbblimit. @build_step(n, i, limit)}\n");
    ds.append_string("@build_base = \xce\xbb{0: \xce\xbbi. []; \xce\xbbm. \xce\xbbi. [%edge_cost(i)]}\n");
    ds.append_string("@build_step = \xce\xbb&d. \xce\xbb&i. \xce\xbb&limit. @cat(@build(d - 1, i * 2, limit), @build(d - 1, i * 2 + 1, limit))\n");
    ds.appendf("@main = @build(%d, 0, %d)\n", depth, e);

    // Run HVM4
    uint[] out_buf = mem::new_array(uint, e);
    int count = pathfind::hvm4_run(ds.zstr_view(), 0, out_buf.ptr, (int)e);

    if (count < 0 || (uint)count != e) {
        free(out_buf.ptr);
        return pathfind::HVM_ERROR~;
    }

    return out_buf;
}

fn void update_weights_hvm4(SwrState* state, WeatherGrid* weather) {
    uint[] weights = compute_weights_hvm4(state, weather)!!;
    for (uint i = 0; i < state.prm.edge_count; i++) {
        state.prm.edge_weight[i] = weights[i];
    }
    free(weights.ptr);
    rebuild_graph(state);
}

fn void rebuild_graph(SwrState* state) {
    state.prm.graph.destroy();
    state.prm.graph.init(state.prm.n);
    for (uint i = 0; i < state.prm.edge_count; i++) {
        state.prm.graph.add_edge(state.prm.edge_from[i], state.prm.edge_to[i], state.prm.edge_weight[i]);
    }
}

// ============================================================
// Route Extraction
// ============================================================

fn uint[] extract_path(SwrState* state, uint[] dist) {
    uint[] path = mem::new_array(uint, state.prm.n);
    uint len = 0;
    uint current = state.destination;

    path[len++] = current;

    uint max_steps = state.prm.n;
    while (current != state.origin && max_steps > 0) {
        max_steps--;
        bool found = false;

        for (uint i = 0; i < state.prm.edge_count; i++) {
            if (state.prm.edge_to[i] == current) {
                uint u = state.prm.edge_from[i];
                uint w = state.prm.edge_weight[i];
                if (dist[u] + w == dist[current]) {
                    current = u;
                    path[len++] = current;
                    found = true;
                    break;
                }
            }
        }

        if (!found) break;
    }

    for (uint i = 0; i < len / 2; i++) {
        uint tmp = path[i];
        path[i] = path[len - 1 - i];
        path[len - 1 - i] = tmp;
    }

    return path[:len];
}

// ============================================================
// Per-Algorithm Routing Functions
// ============================================================

fn RouteResult? swr_route_apsp(SwrState* state, WeatherGrid* weather) {
    update_weights_hvm4(state, weather);

    uint[] mat = pathfind::algebraic_apsp(&state.prm.graph)!;
    uint n = state.prm.n;

    uint cost = mat[state.origin * n + state.destination];

    return (RouteResult){
        .path = {},
        .path_len = 0,
        .total_cost = cost,
        .distances = mat,
        .dist_count = (usz)(n * n)
    };
}

fn RouteResult? swr_route_enumerate(SwrState* state, WeatherGrid* weather) {
    update_weights_hvm4(state, weather);

    pathfind::PathResult pr = pathfind::enumerate_paths(&state.prm.graph, state.origin, state.destination)!;

    uint min_cost = pathfind::INF;
    for (usz i = 0; i < pr.count; i++) {
        if (pr.weights[i] < min_cost) min_cost = pr.weights[i];
    }

    return (RouteResult){
        .path = {},
        .path_len = 0,
        .total_cost = min_cost,
        .distances = pr.weights,
        .dist_count = pr.count
    };
}

// ============================================================
// Unified Replan
// ============================================================

fn RouteResult? swr_replan(SwrState* state, WeatherGrid* weather, Algorithm algo) {
    switch (algo) {
        case APSP: return swr_route_apsp(state, weather);
        case ENUMERATE: return swr_route_enumerate(state, weather);
        case DIJKSTRA: return swr_dijkstra::swr_dijkstra(state, weather);
        case ALT: return swr_alt::swr_route_alt(state, weather);
        case CCH: return swr_cch::swr_route_cch(state, weather);
        case HUB_LABEL: return swr_hub::swr_route_hub(state, weather);
        case TEMPORAL: return swr_temporal::swr_route_temporal(state, state.forecast);
    }
}
