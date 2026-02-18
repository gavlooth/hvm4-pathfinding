#!/usr/bin/env julia
"""
Maritime routing REST API server.

Start: julia server.jl [port]
Default port: 8080
"""

include("H3Grid.jl")
include("ShipRouting.jl")
include("VineRouting.jl")
include("KwonSpeedLoss.jl")
using .H3Grid
using .ShipRouting
using .VineRouting
using .KwonSpeedLoss
using HTTP
using JSON3
using Dates

# ============================================================
# Server State
# ============================================================

mutable struct ServerState
    routing::Union{RoutingState, Nothing}
    result::Union{RouteResult, Nothing}
    scale::UInt32
    initialized::Bool
    h3_grid::Union{H3Grid.HexGrid, Nothing}
    vine::Union{VineRouting.VineState, Nothing}
    ship::KwonSpeedLoss.ShipParams
    cost_mode::Symbol  # :time, :fuel, :weighted, :safety
    # Cached CSR topology (fixed) and weights (recomputed with weather)
    csr_row_ptr::Union{Vector{UInt32}, Nothing}
    csr_col_idx::Union{Vector{UInt32}, Nothing}
    csr_weights::Union{Vector{UInt32}, Nothing}
    # Per-edge geometry (fixed, needed for weight recomputation)
    csr_dist_m::Union{Vector{Float64}, Nothing}
    csr_heading::Union{Vector{Float64}, Nothing}
    csr_midpoints::Union{Vector{Tuple{Float64,Float64}}, Nothing}
    # Calm-weather weights (cached, no weather penalty)
    csr_weights_calm::Union{Vector{UInt32}, Nothing}
    last_weather_update::String
end

const STATE = ServerState(
    nothing, nothing, UInt32(1000), false, nothing, nothing,
    KwonSpeedLoss.ShipParams(),  # default ship
    :time,                       # default: minimize time
    nothing, nothing, nothing, nothing, nothing, nothing,
    nothing, ""
)

const GRID_CACHE = joinpath(@__DIR__, "data", "aegean_grid.txt")

# ============================================================
# Helpers
# ============================================================

function json_response(data; status=200)
    body = JSON3.write(data)
    HTTP.Response(status, cors_headers("application/json"), body)
end

function error_response(msg::String; status=400)
    json_response(Dict(:error => msg); status)
end

function cors_headers(content_type="application/json")
    [
        "Content-Type" => content_type,
        "Access-Control-Allow-Origin" => "*",
        "Access-Control-Allow-Methods" => "GET, POST, OPTIONS",
        "Access-Control-Allow-Headers" => "Content-Type",
    ]
end

function parse_body(req)
    JSON3.read(String(req.body))
end

function wp_to_dict(wp::WaypointInfo)
    Dict(
        :lat => wp.lat,
        :lon => wp.lon,
        :eta_hours => wp.eta_hours,
        :speed_knots => wp.speed_knots,
        :wave_height => wp.wave_height,
        :wind_speed => wp.wind_speed,
        :heading_deg => wp.heading_deg,
        :node_id => wp.node_id,
    )
end

# ============================================================
# Route Handlers
# ============================================================

function handle_init(req)
    body = parse_body(req)

    ship_speed = Float64(get(body, :ship_speed, 15.0))
    max_wave = Float64(get(body, :max_wave, 4.0))
    graph_mode = String(get(body, :graph_mode, "prm"))

    # Create state
    scale = UInt32(get(body, :scale, 1000))
    STATE.routing = create_state(ship_speed=ship_speed, max_wave=max_wave, scale=scale)
    STATE.scale = scale
    STATE.result = nothing

    if graph_mode == "h3"
        # Load cached H3 grid
        if STATE.h3_grid === nothing
            if !isfile(GRID_CACHE)
                return error_response("H3 grid cache not found at $GRID_CACHE — run trip.jl --build-grid first")
            end
            @info "Loading cached H3 grid from $GRID_CACHE"
            STATE.h3_grid = H3Grid.load_grid(GRID_CACHE)
        end
        grid = STATE.h3_grid

        # Compute bounding box from grid centers
        lats_g = [c[1] for c in grid.centers]
        lons_g = [c[2] for c in grid.centers]
        sw_lat = minimum(lats_g) - 0.1
        sw_lon = minimum(lons_g) - 0.1
        ne_lat = maximum(lats_g) + 0.1
        ne_lon = maximum(lons_g) + 0.1

        # Minimal land mask for weather bounds (all-water, H3 handles land avoidance)
        set_land!(STATE.routing, LatLon(sw_lat, sw_lon), LatLon(ne_lat, ne_lon), falses(5, 5))

        # Inject H3 graph
        H3Grid.set_graph_from_h3!(STATE.routing, grid, ShipRouting_mod=ShipRouting)
    else
        # Original PRM path
        land = body[:land]
        sw = [Float64(land[:sw][1]), Float64(land[:sw][2])]
        ne = [Float64(land[:ne][1]), Float64(land[:ne][2])]
        rows = Int(land[:rows])
        cols = Int(land[:cols])
        grid_data = get(land, :grid, [])
        if isempty(grid_data)
            grid = falses(rows, cols)
        else
            grid = reshape(Bool.(collect(grid_data)), rows, cols)
        end
        set_land!(STATE.routing, LatLon(sw[1], sw[2]), LatLon(ne[1], ne[2]), grid)

        prm = body[:prm]
        n_samples = Int(get(prm, :n_samples, 200))
        k_neighbors = Int(get(prm, :k_neighbors, 8))
        seed = Int(get(prm, :seed, 42))
        build_prm!(STATE.routing, n_samples=n_samples, k_neighbors=k_neighbors, seed=seed)
    end

    STATE.initialized = true
    nc = ShipRouting.node_count(STATE.routing)

    json_response(Dict(:ok => true, :node_count => nc, :graph_mode => graph_mode))
end

function handle_voyage(req)
    STATE.initialized || return error_response("not initialized")
    body = parse_body(req)
    origin = body[:origin]
    destination = body[:destination]
    set_voyage!(STATE.routing,
        LatLon(Float64(origin[1]), Float64(origin[2])),
        LatLon(Float64(destination[1]), Float64(destination[2])))
    json_response(Dict(:ok => true))
end

function handle_weather(req)
    STATE.initialized || return error_response("not initialized")
    body = parse_body(req)

    sw = body[:sw]; ne = body[:ne]
    rows = UInt32(body[:rows]); cols = UInt32(body[:cols])
    n = Int(rows) * Int(cols)

    grid = WeatherGrid(
        LatLon(Float64(sw[1]), Float64(sw[2])),
        LatLon(Float64(ne[1]), Float64(ne[2])),
        rows, cols,
        Float64.(collect(body[:wind_speed])),
        Float64.(collect(body[:wind_dir])),
        Float64.(collect(body[:wave_height])),
        Float64.(collect(body[:wave_dir])),
        Float64.(collect(body[:current_speed])),
        Float64.(collect(body[:current_dir])),
        Float64(get(body, :timestamp, 0.0)),
    )
    push_weather!(STATE.routing, grid)
    json_response(Dict(:ok => true))
end

function handle_weather_clear(req)
    STATE.initialized || return error_response("not initialized")
    clear_weather!(STATE.routing)
    json_response(Dict(:ok => true))
end

function handle_route(req)
    STATE.initialized || return error_response("not initialized")
    body = parse_body(req)
    alg_idx = Int(get(body, :algorithm, 0))
    alg = Algorithm(alg_idx)

    STATE.result = route(STATE.routing, algorithm=alg)
    c = cost(STATE.result)
    pl = path_length(STATE.result)
    wps = waypoints(STATE.result)

    json_response(Dict(
        :cost => c,
        :cost_real => Float64(c) / Float64(STATE.scale),
        :path_length => pl,
        :waypoints => [wp_to_dict(wp) for wp in wps],
    ))
end

"""Build CSR topology + per-edge geometry (fixed). Weights computed separately."""
function build_csr_cache!(grid)
    n = length(grid.cells)
    row_ptr = zeros(UInt32, n + 1)
    for (i, nbs) in enumerate(grid.neighbors)
        row_ptr[i + 1] = row_ptr[i] + UInt32(length(nbs))
    end
    total_edges = Int(row_ptr[end])
    col_idx = Vector{UInt32}(undef, total_edges)
    dist_m = Vector{Float64}(undef, total_edges)
    heading = Vector{Float64}(undef, total_edges)
    midpoints = Vector{Tuple{Float64,Float64}}(undef, total_edges)

    idx = 1
    for (i, nbs) in enumerate(grid.neighbors)
        lat_i, lon_i = grid.centers[i]
        for j in nbs
            lat_j, lon_j = grid.centers[j]
            d = H3Grid._haversine_m(lat_i, lon_i, lat_j, lon_j)
            # Heading: bearing from i to j (degrees, 0=N, 90=E)
            hdg = _bearing_deg(lat_i, lon_i, lat_j, lon_j)
            col_idx[idx] = UInt32(j - 1)  # 0-indexed
            dist_m[idx] = d
            heading[idx] = hdg
            midpoints[idx] = ((lat_i + lat_j) / 2.0, (lon_i + lon_j) / 2.0)
            idx += 1
        end
    end
    STATE.csr_row_ptr = row_ptr
    STATE.csr_col_idx = col_idx
    STATE.csr_dist_m = dist_m
    STATE.csr_heading = heading
    STATE.csr_midpoints = midpoints
    @info "CSR topology built: $n nodes, $total_edges edges"

    # Initial weights: calm weather (distance-based, no weather penalty)
    recompute_csr_weights!()
    STATE.csr_weights_calm = copy(STATE.csr_weights)
end

"""Bearing from (lat1,lon1) to (lat2,lon2) in degrees (0=N, 90=E)."""
function _bearing_deg(lat1, lon1, lat2, lon2)
    φ1 = deg2rad(lat1); φ2 = deg2rad(lat2)
    Δλ = deg2rad(lon2 - lon1)
    x = sin(Δλ) * cos(φ2)
    y = cos(φ1) * sin(φ2) - sin(φ1) * cos(φ2) * cos(Δλ)
    θ = atan(x, y)
    return mod(rad2deg(θ), 360.0)
end

"""
Recompute CSR edge weights using the Kwon model + current weather.
If no weather is set, uses calm conditions (BN=0, no speed loss).
Call this whenever weather forecast changes.
"""
function recompute_csr_weights!(; wind_speed_ms=0.0, wind_dir_deg=0.0,
                                  current_speed_ms=0.0, current_dir_deg=0.0,
                                  weather_grid=nothing)
    dist_m = STATE.csr_dist_m
    hdg = STATE.csr_heading
    ship = STATE.ship
    scale = Int(STATE.scale)
    mode = STATE.cost_mode
    n_edges = length(dist_m)
    weights = Vector{UInt32}(undef, n_edges)

    for i in 1:n_edges
        # Per-edge weather: sample from grid if available, else uniform
        ws, wd, cs, cd = wind_speed_ms, wind_dir_deg, current_speed_ms, current_dir_deg
        # TODO: if weather_grid !== nothing, sample at midpoints[i]

        weights[i] = KwonSpeedLoss.kwon_edge_cost(
            ship, dist_m[i], hdg[i], ws, wd, cs, cd;
            mode=mode, scale=scale
        )
    end
    STATE.csr_weights = weights
    @info "CSR weights recomputed: $(n_edges) edges, mode=$(mode)"
end

"""Convert vine route result path to waypoint dicts."""
function path_to_waypoints(path, grid, scale)
    wps = Dict{Symbol, Any}[]
    for nid in path
        lat, lon = grid.centers[nid + 1]  # 1-indexed in Julia
        push!(wps, Dict(
            :lat => lat, :lon => lon,
            :eta_hours => 0.0, :speed_knots => 0.0,
            :wave_height => 0.0, :wind_speed => 0.0,
            :heading_deg => 0.0, :node_id => UInt32(nid),
        ))
    end
    wps
end

function handle_vine_route(req)
    STATE.initialized || return error_response("not initialized")
    body = parse_body(req)
    algo = String(get(body, :algorithm, "dijkstra"))
    workers = Int(get(body, :workers, 4))

    grid = STATE.h3_grid
    if grid === nothing
        return error_response("H3 grid not loaded — Vine routing requires H3 mode")
    end

    # Initialize Vine FFI state lazily
    if STATE.vine === nothing
        @info "Initializing Vine IVM (FFI mode)..."
        try
            STATE.vine = VineRouting.vine_init_ffi()
        catch e
            return error_response("Vine init failed: $(sprint(showerror, e))"; status=500)
        end
        @info "Vine IVM ready."
    end

    # Build CSR topology + weights lazily
    if STATE.csr_row_ptr === nothing
        build_csr_cache!(grid)
    end

    # Get origin/destination
    if !haskey(body, :origin) || !haskey(body, :destination)
        return error_response("Vine route requires 'origin' and 'destination' in request body")
    end
    origin = body[:origin]
    destination = body[:destination]
    src_idx = H3Grid.find_nearest(grid, Float64(origin[1]), Float64(origin[2]))
    tgt_idx = H3Grid.find_nearest(grid, Float64(destination[1]), Float64(destination[2]))
    src_node = src_idx - 1  # 0-indexed for Vine
    tgt_node = tgt_idx - 1

    # Map algorithm name to numeric ID: 0=dijkstra, 1=delta_stepping
    algo_id = algo == "delta_stepping" ? 1 : 0

    # Weather parameters (optional — default to calm)
    wind_speed = Float64(get(body, :wind_speed, 0.0))
    wind_dir = Float64(get(body, :wind_dir, 0.0))
    current_speed = Float64(get(body, :current_speed, 0.0))
    current_dir = Float64(get(body, :current_dir, 0.0))
    has_weather = wind_speed > 0 || current_speed > 0

    scale = Int(STATE.scale)
    timestamp = Dates.format(now(UTC), "yyyy-mm-ddTHH:MM:SSZ")

    # --- Calm route (always use cached calm weights) ---
    t0 = time()
    calm_output = VineRouting.vine_route_graph(
        STATE.vine,
        STATE.csr_row_ptr, STATE.csr_col_idx, STATE.csr_weights_calm,
        Int(src_node), Int(tgt_node);
        algorithm=algo_id, workers=workers)
    calm_elapsed = time() - t0
    calm_result = VineRouting.parse_route_output(calm_output)
    calm_wps = path_to_waypoints(calm_result.path, grid, scale)

    # --- Weather route ---
    if has_weather
        recompute_csr_weights!(wind_speed_ms=wind_speed, wind_dir_deg=wind_dir,
                               current_speed_ms=current_speed, current_dir_deg=current_dir)
        STATE.last_weather_update = timestamp
    end
    # Use weather weights (or calm weights if no weather — same result)
    weather_weights = has_weather ? STATE.csr_weights : STATE.csr_weights_calm

    t1 = time()
    wx_output = VineRouting.vine_route_graph(
        STATE.vine,
        STATE.csr_row_ptr, STATE.csr_col_idx, weather_weights,
        Int(src_node), Int(tgt_node);
        algorithm=algo_id, workers=workers)
    wx_elapsed = time() - t1
    wx_result = VineRouting.parse_route_output(wx_output)
    wx_wps = path_to_waypoints(wx_result.path, grid, scale)

    total_elapsed = calm_elapsed + wx_elapsed

    json_response(Dict(
        :cost => wx_result.cost,
        :cost_real => Float64(wx_result.cost) / Float64(scale),
        :path_length => length(wx_result.path),
        :waypoints => wx_wps,
        :elapsed_ms => round(total_elapsed * 1000, digits=1),
        :algorithm => algo,
        :engine => "vine",
        :timestamp => timestamp,
        :weather => Dict(
            :wind_speed => wind_speed,
            :wind_dir => wind_dir,
            :current_speed => current_speed,
            :current_dir => current_dir,
        ),
        :calm_route => Dict(
            :cost => calm_result.cost,
            :cost_real => Float64(calm_result.cost) / Float64(scale),
            :path_length => length(calm_result.path),
            :waypoints => calm_wps,
        ),
    ))
end

function handle_state(req)
    nc = STATE.initialized ? ShipRouting.node_count(STATE.routing) : 0
    json_response(Dict(
        :initialized => STATE.initialized,
        :node_count => nc,
    ))
end

# ============================================================
# Router
# ============================================================

const ROUTER = HTTP.Router()

function route_handler(req)
    # CORS preflight
    if req.method == "OPTIONS"
        return HTTP.Response(204, cors_headers())
    end

    target = req.target
    method = req.method

    try
        if method == "POST" && target == "/api/init"
            return handle_init(req)
        elseif method == "POST" && target == "/api/voyage"
            return handle_voyage(req)
        elseif method == "POST" && target == "/api/weather"
            return handle_weather(req)
        elseif method == "POST" && target == "/api/weather/clear"
            return handle_weather_clear(req)
        elseif method == "POST" && target == "/api/route"
            return handle_route(req)
        elseif method == "POST" && target == "/api/vine/route"
            return handle_vine_route(req)
        elseif method == "GET" && target == "/api/state"
            return handle_state(req)
        elseif method == "GET" && (target == "/" || target == "/map")
            indexfile = joinpath(@__DIR__, "..", "frontend", "public", "index.html")
            if isfile(indexfile)
                return HTTP.Response(200, ["Content-Type" => "text/html"], read(indexfile, String))
            else
                return HTTP.Response(200, ["Content-Type" => "text/html"],
                    """<!DOCTYPE html><html><body style="font-family:sans-serif;background:#1a1a2e;color:#eee;padding:40px">
                    <h1 style="color:#4fc3f7">Maritime Router API</h1>
                    <p>index.html not found. <a href="/api/state" style="color:#4fc3f7">API state</a></p>
                    </body></html>""")
            end
        else
            return error_response("not found"; status=404)
        end
    catch e
        msg = sprint(showerror, e)
        @error "Handler error" exception=(e, catch_backtrace())
        return error_response(msg; status=500)
    end
end

# ============================================================
# Main
# ============================================================

function main()
    port = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 8080

    println("Initializing routing runtime...")
    ShipRouting.init()

    println("Starting server on http://0.0.0.0:$port")
    println("Endpoints:")
    println("  POST /api/init          — initialize routing state")
    println("  POST /api/voyage        — set origin/destination")
    println("  POST /api/weather       — push weather grid")
    println("  POST /api/weather/clear — clear weather grids")
    println("  POST /api/route         — compute route (Dijkstra)")
    println("  POST /api/vine/route    — compute route (Vine IVM)")
    println("  GET  /api/state         — server state")

    HTTP.serve(route_handler, "0.0.0.0", port)
end

main()
