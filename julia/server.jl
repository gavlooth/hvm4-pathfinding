#!/usr/bin/env julia
"""
Maritime routing REST API server.

Start: julia server.jl [port]
Default port: 8080
"""

include("H3Grid.jl")
include("ShipRouting.jl")
using .H3Grid
using .ShipRouting
using HTTP
using JSON3

# ============================================================
# Server State
# ============================================================

mutable struct ServerState
    routing::Union{RoutingState, Nothing}
    result::Union{RouteResult, Nothing}
    scale::UInt32
    initialized::Bool
    h3_grid::Union{H3Grid.HexGrid, Nothing}
end

const STATE = ServerState(nothing, nothing, UInt32(1000), false, nothing)

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
    alg_idx = Int(get(body, :algorithm, 2))
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
        elseif method == "GET" && target == "/api/state"
            return handle_state(req)
        elseif method == "GET" && (target == "/" || target == "/map")
            mapfile = joinpath(@__DIR__, "..", "frontend", "public", "map.html")
            if isfile(mapfile)
                return HTTP.Response(200, ["Content-Type" => "text/html"], read(mapfile, String))
            else
                return HTTP.Response(200, ["Content-Type" => "text/html"],
                    """<!DOCTYPE html><html><body style="font-family:sans-serif;background:#1a1a2e;color:#eee;padding:40px">
                    <h1 style="color:#4fc3f7">Maritime Router API</h1>
                    <p>map.html not found. <a href="/api/state" style="color:#4fc3f7">API state</a></p>
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
    println("  GET  /api/state         — server state")

    HTTP.serve(route_handler, "0.0.0.0", port)
end

main()
