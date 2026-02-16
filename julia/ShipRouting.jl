"""
    ShipRouting

Julia wrapper for the HVM4 Ship Weather Routing (SWR) FFI library.

Provides type-safe access to 7 routing algorithms (APSP, Enumerate, Dijkstra, ALT,
CCH, Hub-Label, Temporal) with full ship physics, weather interpolation, PRM graph
construction, and per-waypoint metadata (lat/lon/ETA/speed/wave/wind/heading).

# Quick Start
```julia
using ShipRouting

ShipRouting.init()
state = create_state(ship_speed=15.0, max_wave=4.0)
set_land!(state, LatLon(37.0, 23.0), LatLon(39.0, 27.0), falses(10, 10))
build_prm!(state, n_samples=200, k_neighbors=8)
set_voyage!(state, LatLon(37.5, 23.5), LatLon(38.5, 26.5))
push_weather!(state, grid)
result = route(state, algorithm=DIJKSTRA)
wps = waypoints(result)
ShipRouting.cleanup()
```
"""
module ShipRouting

export LatLon, WaypointInfo, WeatherGrid, CostMode, Algorithm
export TIME_ONLY, FUEL_ONLY, WEIGHTED_SUM, SAFETY_PENALIZED
export APSP, ENUMERATE, DIJKSTRA, ALT, CCH, HUB_LABEL, TEMPORAL
export RoutingState, RouteResult
export create_state, set_land!, build_prm!, set_graph!, set_voyage!
export push_weather!, set_forecast_interval!, clear_weather!
export route, route_temporal, cost, path_length, waypoints

# ============================================================
# Library Path
# ============================================================

const LIBPATH = joinpath(@__DIR__, "..", "c3lib", "build", "pathfind-so.so")

# ============================================================
# Types
# ============================================================

struct LatLon
    lat::Float64
    lon::Float64
end

struct WaypointInfo
    lat::Float64
    lon::Float64
    eta_hours::Float64
    speed_knots::Float64
    wave_height::Float64
    wind_speed::Float64
    heading_deg::Float64
    node_id::UInt32
end

struct WeatherGrid
    sw::LatLon
    ne::LatLon
    rows::UInt32
    cols::UInt32
    wind_speed::Vector{Float64}
    wind_dir::Vector{Float64}
    wave_height::Vector{Float64}
    wave_dir::Vector{Float64}
    current_speed::Vector{Float64}
    current_dir::Vector{Float64}
    timestamp::Float64
end

@enum CostMode begin
    TIME_ONLY       = 0
    FUEL_ONLY       = 1
    WEIGHTED_SUM    = 2
    SAFETY_PENALIZED = 3
end

@enum Algorithm begin
    APSP       = 0
    ENUMERATE  = 1
    DIJKSTRA   = 2
    ALT        = 3
    CCH        = 4
    HUB_LABEL  = 5
    TEMPORAL   = 6
end

# ============================================================
# Error Codes
# ============================================================

const SWR_OK         =  0
const SWR_NULL       = -1
const SWR_BAD_PARAM  = -2
const SWR_ALLOC_FAIL = -3
const SWR_NO_ROUTE   = -4
const SWR_HVM_ERROR  = -5

struct SwrError <: Exception
    code::Int
    msg::String
end

function check_rc(rc::Integer, context::String)
    rc == SWR_OK && return
    msgs = Dict(
        SWR_NULL      => "null handle",
        SWR_BAD_PARAM => "bad parameter",
        SWR_ALLOC_FAIL => "allocation failed",
        SWR_NO_ROUTE  => "no route found",
        SWR_HVM_ERROR => "HVM4 error"
    )
    throw(SwrError(Int(rc), "$context: $(get(msgs, Int(rc), "unknown error ($rc)"))"))
end

# ============================================================
# Opaque Handles with Finalizers
# ============================================================

mutable struct RoutingState
    ptr::Ptr{Cvoid}
    function RoutingState(ptr::Ptr{Cvoid})
        obj = new(ptr)
        finalizer(obj) do s
            if s.ptr != C_NULL
                ccall((:swr_state_destroy, LIBPATH), Cvoid, (Ptr{Cvoid},), s.ptr)
                s.ptr = C_NULL
            end
        end
        obj
    end
end

Base.unsafe_convert(::Type{Ptr{Cvoid}}, s::RoutingState) = s.ptr

mutable struct RouteResult
    ptr::Ptr{Cvoid}
    function RouteResult(ptr::Ptr{Cvoid})
        obj = new(ptr)
        finalizer(obj) do r
            if r.ptr != C_NULL
                ccall((:swr_route_destroy, LIBPATH), Cvoid, (Ptr{Cvoid},), r.ptr)
                r.ptr = C_NULL
            end
        end
        obj
    end
end

Base.unsafe_convert(::Type{Ptr{Cvoid}}, r::RouteResult) = r.ptr

# ============================================================
# Lifecycle
# ============================================================

"""
    init()

Initialize the HVM4 runtime. Must be called before any other function.
Idempotent — safe to call multiple times.
"""
function init()
    ccall((:swr_init, LIBPATH), Cvoid, ())
end

"""
    cleanup()

Shut down the HVM4 runtime and free global resources.
"""
function cleanup()
    ccall((:swr_cleanup, LIBPATH), Cvoid, ())
end

# ============================================================
# State Setup
# ============================================================

"""
    create_state(; ship_speed=15.0, max_wave=4.0, fuel_base=100.0, fuel_per_kt=5.0,
                   cost_mode=TIME_ONLY, alpha=1.0, safety_penalty=100.0, scale=1000)

Create a new routing state with ship parameters and cost configuration.
Returns an opaque `RoutingState` handle (automatically freed on GC).
"""
function create_state(;
    ship_speed::Float64  = 15.0,
    max_wave::Float64    = 4.0,
    fuel_base::Float64   = 100.0,
    fuel_per_kt::Float64 = 5.0,
    cost_mode::CostMode  = TIME_ONLY,
    alpha::Float64       = 1.0,
    safety_penalty::Float64 = 100.0,
    scale::UInt32        = UInt32(1000)
)
    ptr = ccall((:swr_state_new, LIBPATH), Ptr{Cvoid}, ())
    ptr == C_NULL && error("swr_state_new: allocation failed")
    state = RoutingState(ptr)

    rc = ccall((:swr_state_set_ship, LIBPATH), Cint,
        (Ptr{Cvoid}, Cdouble, Cdouble, Cdouble, Cdouble),
        state, ship_speed, max_wave, fuel_base, fuel_per_kt)
    check_rc(rc, "set_ship")

    rc = ccall((:swr_state_set_cost, LIBPATH), Cint,
        (Ptr{Cvoid}, Cint, Cdouble, Cdouble, Cuint),
        state, Int32(Integer(cost_mode)), alpha, safety_penalty, scale)
    check_rc(rc, "set_cost")

    state
end

"""
    set_land!(state, sw::LatLon, ne::LatLon, grid::AbstractMatrix{Bool})

Set the land mask. `grid` is row-major (rows x cols), `true` = land.
The library copies the data, so `grid` can be freed after this call.
"""
function set_land!(state::RoutingState, sw::LatLon, ne::LatLon, grid::AbstractMatrix{<:Union{Bool,Integer}})
    rows, cols = size(grid)
    # C3 expects row-major bool array (1 byte per cell)
    flat = UInt8.(vec(permutedims(Matrix(grid))))
    GC.@preserve flat begin
        rc = ccall((:swr_state_set_land, LIBPATH), Cint,
            (Ptr{Cvoid}, Cdouble, Cdouble, Cdouble, Cdouble, Cuint, Cuint, Ptr{UInt8}),
            state, sw.lat, sw.lon, ne.lat, ne.lon,
            UInt32(rows), UInt32(cols), pointer(flat))
        check_rc(rc, "set_land")
    end
end

"""
    build_prm!(state; n_samples=200, k_neighbors=8, collision_steps=10, seed=42)

Build the Probabilistic Roadmap (PRM) graph.
"""
function build_prm!(state::RoutingState;
    n_samples::Integer      = 200,
    k_neighbors::Integer    = 8,
    collision_steps::Integer = 10,
    seed::Integer           = 42
)
    rc = ccall((:swr_state_build_prm, LIBPATH), Cint,
        (Ptr{Cvoid}, Cuint, Cuint, Cuint, Culonglong),
        state, UInt32(n_samples), UInt32(k_neighbors),
        UInt32(collision_steps), UInt64(seed))
    check_rc(rc, "build_prm")
end

"""
    set_graph!(state; lats, lons, edge_from, edge_to, edge_dist_m, edge_heading)

Inject a pre-built graph directly, bypassing PRM random sampling.
Node indices are 0-based (C convention). All vectors must be Float64 or UInt32.
"""
function set_graph!(state::RoutingState;
    lats::Vector{Float64},
    lons::Vector{Float64},
    edge_from::Vector{UInt32},
    edge_to::Vector{UInt32},
    edge_dist_m::Vector{Float64},
    edge_heading::Vector{Float64}
)
    n_nodes = UInt32(length(lats))
    n_edges = UInt32(length(edge_from))
    @assert length(lons) == length(lats) "lats/lons length mismatch"
    @assert length(edge_to) == length(edge_from) "edge_from/edge_to length mismatch"
    @assert length(edge_dist_m) == length(edge_from) "edge_dist_m length mismatch"
    @assert length(edge_heading) == length(edge_from) "edge_heading length mismatch"

    GC.@preserve lats lons edge_from edge_to edge_dist_m edge_heading begin
        rc = ccall((:swr_state_set_graph, LIBPATH), Cint,
            (Ptr{Cvoid}, Cuint, Ptr{Cdouble}, Ptr{Cdouble},
             Cuint, Ptr{Cuint}, Ptr{Cuint}, Ptr{Cdouble}, Ptr{Cdouble}),
            state, n_nodes, lats, lons,
            n_edges, edge_from, edge_to, edge_dist_m, edge_heading)
        check_rc(rc, "set_graph")
    end
end

"""
    set_voyage!(state, origin::LatLon, destination::LatLon)

Set origin and destination (snaps to nearest PRM nodes).
"""
function set_voyage!(state::RoutingState, origin::LatLon, destination::LatLon)
    rc = ccall((:swr_state_set_voyage, LIBPATH), Cint,
        (Ptr{Cvoid}, Cdouble, Cdouble, Cdouble, Cdouble),
        state, origin.lat, origin.lon, destination.lat, destination.lon)
    check_rc(rc, "set_voyage")
end

"""
    node_count(state) → Int

Return the number of PRM nodes.
"""
function node_count(state::RoutingState)
    n = ccall((:swr_state_node_count, LIBPATH), Cint, (Ptr{Cvoid},), state)
    n < 0 && error("node_count: null state")
    Int(n)
end

# ============================================================
# Weather (Push Model)
# ============================================================

"""
    push_weather!(state, grid::WeatherGrid)

Push a weather grid into the forecast buffer. For static routing, only the
last pushed grid is used. For temporal routing, all pushed grids form the
forecast sequence.
"""
function push_weather!(state::RoutingState, g::WeatherGrid)
    n = Int(g.rows) * Int(g.cols)
    @assert length(g.wind_speed)    == n "wind_speed length mismatch"
    @assert length(g.wind_dir)      == n "wind_dir length mismatch"
    @assert length(g.wave_height)   == n "wave_height length mismatch"
    @assert length(g.wave_dir)      == n "wave_dir length mismatch"
    @assert length(g.current_speed) == n "current_speed length mismatch"
    @assert length(g.current_dir)   == n "current_dir length mismatch"

    GC.@preserve g begin
        rc = ccall((:swr_weather_push, LIBPATH), Cint,
            (Ptr{Cvoid},
             Cdouble, Cdouble, Cdouble, Cdouble,
             Cuint, Cuint,
             Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble},
             Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble},
             Cdouble),
            state,
            g.sw.lat, g.sw.lon, g.ne.lat, g.ne.lon,
            g.rows, g.cols,
            g.wind_speed, g.wind_dir, g.wave_height,
            g.wave_dir, g.current_speed, g.current_dir,
            g.timestamp)
        check_rc(rc, "weather_push")
    end
end

"""
    set_forecast_interval!(state; interval_hours=6, wait_cost=50)

Configure temporal routing parameters.
"""
function set_forecast_interval!(state::RoutingState;
    interval_hours::Integer = 6,
    wait_cost::Integer      = 50
)
    rc = ccall((:swr_weather_set_interval, LIBPATH), Cint,
        (Ptr{Cvoid}, Cuint, Cuint),
        state, UInt32(interval_hours), UInt32(wait_cost))
    check_rc(rc, "weather_set_interval")
end

"""
    clear_weather!(state)

Remove all pushed weather grids from the forecast buffer.
"""
function clear_weather!(state::RoutingState)
    rc = ccall((:swr_weather_clear, LIBPATH), Cint, (Ptr{Cvoid},), state)
    check_rc(rc, "weather_clear")
end

# ============================================================
# Routing
# ============================================================

"""
    route(state; algorithm=DIJKSTRA) → RouteResult

Run a static routing algorithm using the last pushed weather grid.
Returns an opaque `RouteResult` handle (automatically freed on GC).

Available algorithms: `APSP`, `ENUMERATE`, `DIJKSTRA`, `ALT`, `CCH`, `HUB_LABEL`.
"""
function route(state::RoutingState; algorithm::Algorithm = DIJKSTRA)
    ptr = ccall((:swr_route, LIBPATH), Ptr{Cvoid},
        (Ptr{Cvoid}, Cint), state, Int32(Integer(algorithm)))
    ptr == C_NULL && error("route: routing failed (no route or bad state)")
    RouteResult(ptr)
end

"""
    route_temporal(state) → RouteResult

Run temporal routing over all pushed weather grids.
Requires at least 2 pushed grids. Call `set_forecast_interval!` first.
"""
function route_temporal(state::RoutingState)
    ptr = ccall((:swr_route_temporal, LIBPATH), Ptr{Cvoid}, (Ptr{Cvoid},), state)
    ptr == C_NULL && error("route_temporal: routing failed (need ≥2 grids)")
    RouteResult(ptr)
end

# ============================================================
# Result Access
# ============================================================

"""
    cost(result) → UInt32

Total route cost (scaled integer, divide by `scale` for real units).
"""
function cost(result::RouteResult)
    ccall((:swr_route_cost, LIBPATH), Cuint, (Ptr{Cvoid},), result)
end

"""
    path_length(result) → Int

Number of waypoints in the route path.
"""
function path_length(result::RouteResult)
    Int(ccall((:swr_route_path_len, LIBPATH), Cint, (Ptr{Cvoid},), result))
end

"""
    waypoints(result) → Vector{WaypointInfo}

Extract per-waypoint metadata: lat, lon, ETA (hours from origin),
effective speed (knots), wave height (m), wind speed (knots),
heading (degrees), and PRM node ID.
"""
function waypoints(result::RouteResult)
    n = path_length(result)
    n <= 0 && return WaypointInfo[]

    lats   = Vector{Float64}(undef, n)
    lons   = Vector{Float64}(undef, n)
    etas   = Vector{Float64}(undef, n)
    speeds = Vector{Float64}(undef, n)
    waves  = Vector{Float64}(undef, n)
    winds  = Vector{Float64}(undef, n)
    hdgs   = Vector{Float64}(undef, n)
    nids   = Vector{UInt32}(undef, n)

    got = ccall((:swr_route_get_waypoints, LIBPATH), Cint,
        (Ptr{Cvoid},
         Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble},
         Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cuint},
         Cint),
        result,
        lats, lons, etas, speeds, waves, winds, hdgs, nids,
        Int32(n))

    got < 0 && error("get_waypoints failed: $got")

    [WaypointInfo(lats[i], lons[i], etas[i], speeds[i],
                  waves[i], winds[i], hdgs[i], nids[i])
     for i in 1:got]
end

end # module
