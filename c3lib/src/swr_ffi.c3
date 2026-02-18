module swr_ffi;
import swr;
import swr_dijkstra;
import pathfind;
import std::math;

// ============================================================
// Result Codes
// ============================================================

const int SWR_OK         =  0;
const int SWR_NULL       = -1;
const int SWR_BAD_PARAM  = -2;
const int SWR_ALLOC_FAIL = -3;
const int SWR_NO_ROUTE   = -4;

// ============================================================
// Internal Types (opaque to FFI callers)
// ============================================================

struct FfiState {
    swr::SwrState state;
    swr::WeatherGrid[] forecast_grids;
    swr::WeatherCell*[] forecast_cells;  // backing cell arrays
    uint forecast_count;
    uint forecast_capacity;
    uint interval_hours;
    uint wait_cost;
    bool[] land_copy;  // owned copy of land grid
}

struct FfiRoute {
    swr::RouteResult result;
    FfiState* parent;
    int weather_idx;  // which grid was used (-1 = temporal)
}

struct LegInfo {
    double eff_speed_knots;
    double time_hours;
    swr::WeatherCell weather;
}

// ============================================================
// Helpers
// ============================================================

fn LegInfo compute_leg_info(swr::ShipModel* ship, swr::WeatherCell weather,
                             double dist_m, double heading_deg) {
    double wind_angle = swr::dabs(weather.wind_dir - heading_deg);
    if (wind_angle > 180.0) wind_angle = 360.0 - wind_angle;

    double wave_ratio = weather.wave_height / ship.max_wave_ht;
    if (wave_ratio > 1.0) wave_ratio = 1.0;
    double reduction = 1.0 - 0.5 * wave_ratio * wave_ratio;
    reduction = swr::clamp_d(reduction, 0.1, 1.0);

    double current_angle = (weather.current_dir - heading_deg) * pathfind::DEG_TO_RAD;
    double current_along = weather.current_speed * math::cos(current_angle) * 1.94384;

    double eff_speed = ship.base_speed * reduction + current_along;
    if (eff_speed < 0.1) eff_speed = 0.1;

    double dist_nm = dist_m / swr::NM_TO_M;
    double time_h = dist_nm / eff_speed;

    return (LegInfo){
        .eff_speed_knots = eff_speed,
        .time_hours = time_h,
        .weather = weather
    };
}

fn int find_edge_idx(swr::PrmGraph* prm, uint from, uint to) {
    for (uint i = 0; i < prm.edge_count; i++) {
        if (prm.edge_from[i] == from && prm.edge_to[i] == to) return (int)i;
    }
    return -1;
}

fn void ffi_state_free_forecast(FfiState* fs) {
    for (uint i = 0; i < fs.forecast_count; i++) {
        if (fs.forecast_cells[i] != null) {
            free(fs.forecast_cells[i]);
        }
    }
    fs.forecast_count = 0;
}

// ============================================================
// Lifecycle
// ============================================================

fn void swr_init() @export("swr_init") {
    // No-op: HVM4 runtime removed
}

fn void swr_cleanup() @export("swr_cleanup") {
    // No-op: HVM4 runtime removed
}

// ============================================================
// State Management
// ============================================================

fn void* swr_state_new() @export("swr_state_new") {
    FfiState* fs = mem::new(FfiState);
    if (fs == null) return null;
    *fs = (FfiState){};
    // Pre-allocate forecast buffer
    fs.forecast_capacity = 16;
    fs.forecast_grids = mem::new_array(swr::WeatherGrid, 16);
    fs.forecast_cells = mem::new_array(swr::WeatherCell*, 16);
    for (uint i = 0; i < 16; i++) fs.forecast_cells[i] = null;
    fs.interval_hours = 6;
    fs.wait_cost = 50;
    return (void*)fs;
}

fn void swr_state_destroy(void* handle) @export("swr_state_destroy") {
    if (handle == null) return;
    FfiState* fs = (FfiState*)handle;
    ffi_state_free_forecast(fs);
    if (fs.forecast_grids.ptr != null) free(fs.forecast_grids.ptr);
    if (fs.forecast_cells.ptr != null) free(fs.forecast_cells.ptr);
    if (fs.land_copy.ptr != null) free(fs.land_copy.ptr);
    fs.state.destroy();
    free(fs);
}

fn int swr_state_set_ship(void* handle, double base_speed, double max_wave,
                           double fuel_base, double fuel_per_kt) @export("swr_state_set_ship") {
    if (handle == null) return SWR_NULL;
    FfiState* fs = (FfiState*)handle;
    fs.state.ship = (swr::ShipModel){
        .base_speed = base_speed,
        .max_wave_ht = max_wave,
        .fuel_rate_base = fuel_base,
        .fuel_rate_per_kt = fuel_per_kt
    };
    return SWR_OK;
}

fn int swr_state_set_cost(void* handle, int mode, double alpha,
                           double safety_penalty, uint scale) @export("swr_state_set_cost") {
    if (handle == null) return SWR_NULL;
    if (mode < 0 || mode > 3) return SWR_BAD_PARAM;
    FfiState* fs = (FfiState*)handle;
    fs.state.cost_cfg = (swr::CostConfig){
        .mode = (swr::CostMode)(char)mode,
        .alpha = alpha,
        .safety_penalty = safety_penalty,
        .scale = scale
    };
    return SWR_OK;
}

fn int swr_state_set_land(void* handle, double sw_lat, double sw_lon,
                           double ne_lat, double ne_lon,
                           uint rows, uint cols, bool* grid) @export("swr_state_set_land") {
    if (handle == null) return SWR_NULL;
    if (grid == null || rows == 0 || cols == 0) return SWR_BAD_PARAM;
    FfiState* fs = (FfiState*)handle;

    // Free previous land copy
    if (fs.land_copy.ptr != null) free(fs.land_copy.ptr);

    // Copy grid data
    uint n = rows * cols;
    fs.land_copy = mem::new_array(bool, n);
    mem::copy(fs.land_copy.ptr, grid, n * bool.sizeof);

    fs.state.land = (swr::LandMask){
        .bounds = {
            .sw = { .lat = sw_lat, .lon = sw_lon },
            .ne = { .lat = ne_lat, .lon = ne_lon }
        },
        .rows = rows,
        .cols = cols,
        .grid = fs.land_copy
    };
    return SWR_OK;
}

fn int swr_state_build_prm(void* handle, uint n_samples, uint k_neighbors,
                             uint collision_steps, ulong seed) @export("swr_state_build_prm") {
    if (handle == null) return SWR_NULL;
    if (n_samples == 0 || k_neighbors == 0) return SWR_BAD_PARAM;
    FfiState* fs = (FfiState*)handle;

    // Destroy previous PRM if any
    fs.state.prm.destroy();
    fs.state.prm = (swr::PrmGraph){};

    swr::swr_build_prm(&fs.state, n_samples, k_neighbors, collision_steps, seed);
    return SWR_OK;
}

fn int swr_state_set_graph(void* handle,
    uint n_nodes, double* lats, double* lons,
    uint n_edges, uint* edge_from, uint* edge_to,
    double* edge_dist_m, double* edge_heading
) @export("swr_state_set_graph") {
    if (handle == null) return SWR_NULL;
    if (n_nodes == 0 || lats == null || lons == null) return SWR_BAD_PARAM;
    if (n_edges > 0 && (edge_from == null || edge_to == null ||
        edge_dist_m == null || edge_heading == null)) return SWR_BAD_PARAM;

    FfiState* fs = (FfiState*)handle;

    // Destroy previous PRM if any
    fs.state.prm.destroy();
    fs.state.prm = (swr::PrmGraph){};

    // Allocate and populate waypoints
    swr::Waypoint[] wps = mem::new_array(swr::Waypoint, n_nodes);
    if (wps.ptr == null) return SWR_ALLOC_FAIL;
    for (uint i = 0; i < n_nodes; i++) {
        wps[i] = (swr::Waypoint){ .id = i, .pos = { .lat = lats[i], .lon = lons[i] } };
    }
    fs.state.prm.waypoints = wps;
    fs.state.prm.n = n_nodes;

    // Allocate and copy edge arrays
    fs.state.prm.edge_from = mem::new_array(uint, n_edges);
    fs.state.prm.edge_to = mem::new_array(uint, n_edges);
    fs.state.prm.edge_dist_m = mem::new_array(double, n_edges);
    fs.state.prm.edge_heading = mem::new_array(double, n_edges);
    fs.state.prm.edge_weight = mem::new_array(uint, n_edges);
    fs.state.prm.edge_count = n_edges;
    fs.state.prm.edge_capacity = n_edges;

    for (uint i = 0; i < n_edges; i++) {
        fs.state.prm.edge_from[i] = edge_from[i];
        fs.state.prm.edge_to[i] = edge_to[i];
        fs.state.prm.edge_dist_m[i] = edge_dist_m[i];
        fs.state.prm.edge_heading[i] = edge_heading[i];
        fs.state.prm.edge_weight[i] = 1;
    }

    // Build adjacency list
    swr::rebuild_graph(&fs.state);
    return SWR_OK;
}

fn int swr_state_set_voyage(void* handle, double orig_lat, double orig_lon,
                              double dest_lat, double dest_lon) @export("swr_state_set_voyage") {
    if (handle == null) return SWR_NULL;
    FfiState* fs = (FfiState*)handle;
    if (fs.state.prm.n == 0) return SWR_BAD_PARAM;
    swr::swr_set_voyage(&fs.state,
        (pathfind::LatLon){ .lat = orig_lat, .lon = orig_lon },
        (pathfind::LatLon){ .lat = dest_lat, .lon = dest_lon });
    return SWR_OK;
}

fn int swr_state_node_count(void* handle) @export("swr_state_node_count") {
    if (handle == null) return -1;
    FfiState* fs = (FfiState*)handle;
    return (int)fs.state.prm.n;
}

// ============================================================
// Weather (Push Model)
// ============================================================

fn int swr_weather_push(void* handle,
                         double sw_lat, double sw_lon,
                         double ne_lat, double ne_lon,
                         uint rows, uint cols,
                         double* wind_speed, double* wind_dir,
                         double* wave_height, double* wave_dir,
                         double* current_speed, double* current_dir,
                         double timestamp) @export("swr_weather_push") {
    if (handle == null) return SWR_NULL;
    if (rows == 0 || cols == 0) return SWR_BAD_PARAM;
    if (wind_speed == null || wind_dir == null || wave_height == null ||
        wave_dir == null || current_speed == null || current_dir == null) return SWR_BAD_PARAM;

    FfiState* fs = (FfiState*)handle;

    // Grow forecast buffer if needed
    if (fs.forecast_count >= fs.forecast_capacity) {
        uint new_cap = fs.forecast_capacity * 2;
        swr::WeatherGrid[] new_grids = mem::new_array(swr::WeatherGrid, new_cap);
        swr::WeatherCell*[] new_cells = mem::new_array(swr::WeatherCell*, new_cap);
        mem::copy(new_grids.ptr, fs.forecast_grids.ptr, fs.forecast_count * swr::WeatherGrid.sizeof);
        mem::copy(new_cells.ptr, fs.forecast_cells.ptr, fs.forecast_count * swr::WeatherCell*.sizeof);
        for (uint i = fs.forecast_count; i < new_cap; i++) new_cells[i] = null;
        free(fs.forecast_grids.ptr);
        free(fs.forecast_cells.ptr);
        fs.forecast_grids = new_grids;
        fs.forecast_cells = new_cells;
        fs.forecast_capacity = new_cap;
    }

    // Convert flat arrays → WeatherCell[]
    uint n = rows * cols;
    swr::WeatherCell* cells = (swr::WeatherCell*)mem::calloc(n * swr::WeatherCell.sizeof);
    if (cells == null) return SWR_ALLOC_FAIL;

    for (uint i = 0; i < n; i++) {
        cells[i] = (swr::WeatherCell){
            .wind_speed    = wind_speed[i],
            .wind_dir      = wind_dir[i],
            .wave_height   = wave_height[i],
            .wave_dir      = wave_dir[i],
            .current_speed = current_speed[i],
            .current_dir   = current_dir[i]
        };
    }

    uint idx = fs.forecast_count;
    fs.forecast_cells[idx] = cells;
    fs.forecast_grids[idx] = (swr::WeatherGrid){
        .bounds = {
            .sw = { .lat = sw_lat, .lon = sw_lon },
            .ne = { .lat = ne_lat, .lon = ne_lon }
        },
        .rows = rows,
        .cols = cols,
        .cells = ((swr::WeatherCell*)cells)[:n],
        .timestamp = timestamp
    };
    fs.forecast_count++;
    return SWR_OK;
}

fn int swr_weather_set_interval(void* handle, uint hours, uint wait_cost) @export("swr_weather_set_interval") {
    if (handle == null) return SWR_NULL;
    if (hours == 0) return SWR_BAD_PARAM;
    FfiState* fs = (FfiState*)handle;
    fs.interval_hours = hours;
    fs.wait_cost = wait_cost;
    return SWR_OK;
}

fn int swr_weather_clear(void* handle) @export("swr_weather_clear") {
    if (handle == null) return SWR_NULL;
    FfiState* fs = (FfiState*)handle;
    ffi_state_free_forecast(fs);
    return SWR_OK;
}

// ============================================================
// Routing
// ============================================================

fn void* swr_route(void* state_handle, int algorithm) @export("swr_route") {
    if (state_handle == null) return null;
    FfiState* fs = (FfiState*)state_handle;
    if (fs.state.prm.n == 0) return null;
    if (fs.forecast_count == 0) return null;
    if (algorithm < 0 || algorithm > 0) return null;

    // Use last pushed grid
    uint grid_idx = fs.forecast_count - 1;
    swr::WeatherGrid* weather = &fs.forecast_grids[grid_idx];

    swr::Algorithm algo = (swr::Algorithm)(char)algorithm;
    swr::RouteResult result = swr::swr_replan(&fs.state, weather, algo) ?? (swr::RouteResult){};
    if (result.total_cost == 0 && result.path_len == 0 && result.dist_count == 0) return null;

    FfiRoute* fr = mem::new(FfiRoute);
    if (fr == null) {
        result.destroy();
        return null;
    }
    *fr = (FfiRoute){
        .result = result,
        .parent = fs,
        .weather_idx = (int)grid_idx
    };
    return (void*)fr;
}

fn void swr_route_destroy(void* route_handle) @export("swr_route_destroy") {
    if (route_handle == null) return;
    FfiRoute* fr = (FfiRoute*)route_handle;
    fr.result.destroy();
    free(fr);
}

// ============================================================
// Result Access
// ============================================================

fn uint swr_route_cost(void* route_handle) @export("swr_route_cost") {
    if (route_handle == null) return 0;
    FfiRoute* fr = (FfiRoute*)route_handle;
    return fr.result.total_cost;
}

fn int swr_route_path_len(void* route_handle) @export("swr_route_path_len") {
    if (route_handle == null) return 0;
    FfiRoute* fr = (FfiRoute*)route_handle;
    return (int)fr.result.path_len;
}

fn int swr_route_get_waypoints(void* route_handle,
                                double* lats, double* lons,
                                double* eta_hours, double* speed_knots,
                                double* wave_heights, double* wind_speeds,
                                double* headings, uint* node_ids,
                                int max) @export("swr_route_get_waypoints") {
    if (route_handle == null) return SWR_NULL;
    FfiRoute* fr = (FfiRoute*)route_handle;
    FfiState* fs = fr.parent;
    if (fs == null) return SWR_NULL;

    int n = (int)fr.result.path_len;
    if (n == 0) return 0;
    if (n > max) n = max;

    // Select weather grid for sampling
    swr::WeatherGrid* weather = null;
    if (fr.weather_idx >= 0 && (uint)fr.weather_idx < fs.forecast_count) {
        weather = &fs.forecast_grids[fr.weather_idx];
    } else if (fs.forecast_count > 0) {
        weather = &fs.forecast_grids[fs.forecast_count - 1];
    }

    double cumulative_eta = 0.0;

    for (int i = 0; i < n; i++) {
        uint nid = fr.result.path[i];
        pathfind::LatLon pos = fs.state.prm.waypoints[nid].pos;

        lats[i] = pos.lat;
        lons[i] = pos.lon;
        if (node_ids != null) node_ids[i] = nid;

        if (i == 0) {
            // Origin waypoint
            eta_hours[i] = 0.0;
            speed_knots[i] = fs.state.ship.base_speed;
            if (weather != null) {
                swr::WeatherCell w = weather.sample(pos);
                wave_heights[i] = w.wave_height;
                wind_speeds[i] = w.wind_speed;
            } else {
                wave_heights[i] = 0.0;
                wind_speeds[i] = 0.0;
            }
            headings[i] = 0.0;
        } else {
            uint prev_nid = fr.result.path[i - 1];
            pathfind::LatLon prev_pos = fs.state.prm.waypoints[prev_nid].pos;

            // Find edge data
            int ei = find_edge_idx(&fs.state.prm, prev_nid, nid);
            double dist_m;
            double heading_deg;
            if (ei >= 0) {
                dist_m = fs.state.prm.edge_dist_m[ei];
                heading_deg = fs.state.prm.edge_heading[ei];
            } else {
                // Fallback: compute from coordinates
                dist_m = pathfind::haversine(prev_pos, pos);
                heading_deg = pathfind::bearing(prev_pos, pos);
            }

            // Sample weather at edge midpoint
            swr::WeatherCell w = {};
            if (weather != null) {
                // For temporal routes, select grid by current time
                swr::WeatherGrid* grid_to_sample = weather;
                if (fr.weather_idx < 0 && fs.forecast_count > 0 && fs.interval_hours > 0) {
                    uint t_idx = (uint)(cumulative_eta / (double)fs.interval_hours);
                    if (t_idx >= fs.forecast_count) t_idx = fs.forecast_count - 1;
                    grid_to_sample = &fs.forecast_grids[t_idx];
                }
                pathfind::LatLon mid = {
                    .lat = (prev_pos.lat + pos.lat) / 2.0,
                    .lon = (prev_pos.lon + pos.lon) / 2.0
                };
                w = grid_to_sample.sample(mid);
            }

            LegInfo leg = compute_leg_info(&fs.state.ship, w, dist_m, heading_deg);
            cumulative_eta += leg.time_hours;

            eta_hours[i] = cumulative_eta;
            speed_knots[i] = leg.eff_speed_knots;
            wave_heights[i] = leg.weather.wave_height;
            wind_speeds[i] = leg.weather.wind_speed;
            headings[i] = heading_deg;
        }
    }

    return n;
}
