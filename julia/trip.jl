#!/usr/bin/env julia
"""
End-to-end maritime trip generation for the Aegean Sea.

Usage:
  julia trip.jl                          # default: Piraeus → Mykonos
  julia trip.jl --from 37.94,23.64 --to 37.45,25.33  # custom coordinates
  julia trip.jl --build-grid             # rebuild H3 grid (slow, saves to cache)
  julia trip.jl --list-routes            # show preset routes

Steps:
  1. Load (or build) H3 hex grid with GSHHG coastline
  2. Convert hex grid → land mask for ShipRouting
  3. Initialize ShipRouting state
  4. Set voyage (origin/destination)
  5. Push weather (synthetic or from API)
  6. Route using HVM4 algorithm
  7. Print waypoints with full metadata
"""

include("H3Grid.jl")
include("ShipRouting.jl")
using .H3Grid
using .ShipRouting
using Printf

# ============================================================
# Preset Routes
# ============================================================

const ROUTES = Dict(
    "piraeus-mykonos"    => ((37.9475, 23.6378), (37.4467, 25.3289), "Piraeus → Mykonos"),
    "piraeus-santorini"  => ((37.9475, 23.6378), (36.3932, 25.4615), "Piraeus → Santorini"),
    "piraeus-heraklion"  => ((37.9475, 23.6378), (35.3387, 25.1442), "Piraeus → Heraklion"),
    "thessaloniki-chios" => ((40.6401, 22.9444), (38.3722, 26.1367), "Thessaloniki → Chios"),
    "rhodes-piraeus"     => ((36.4510, 28.2278), (37.9475, 23.6378), "Rhodes → Piraeus"),
    "volos-skiathos"     => ((39.3666, 22.9425), (39.1622, 23.4903), "Volos → Skiathos"),
    "kavala-limnos"      => ((40.9397, 24.4014), (39.8742, 25.0636), "Kavala → Limnos"),
    "patras-corfu"       => ((38.2466, 21.7346), (39.6243, 19.9217), "Patras → Corfu"),
)

# ============================================================
# Config
# ============================================================

const GRID_CACHE = joinpath(@__DIR__, "data", "aegean_grid.txt")
const COASTLINE  = joinpath(@__DIR__, "data", "gshhg", "GSHHS_shp", "h", "GSHHS_h_L1.shp")
const AEGEAN_BBOX = (34.5, 19.0, 41.5, 30.0)  # wider bbox to include Ionian + Aegean

# ============================================================
# Grid Management
# ============================================================

function get_grid(; force_rebuild=false)
    if !force_rebuild && isfile(GRID_CACHE)
        println("Loading cached H3 grid from $GRID_CACHE")
        grid = H3Grid.load_grid(GRID_CACHE)
        H3Grid.grid_stats(grid)
        return grid
    end

    println("Building H3 grid (this takes a few minutes)...")
    grid = H3Grid.build_aegean_grid(
        coastline_path = COASTLINE,
        base_res = 6,       # ~3.2 km in open water (smoother routes)
        coast_res = 8,      # ~0.5 km near islands (precise coastal navigation)
        refine_dist_km = 5.0,
        bbox = AEGEAN_BBOX,
    )
    H3Grid.grid_stats(grid)

    mkpath(dirname(GRID_CACHE))
    H3Grid.save_grid(GRID_CACHE, grid)
    grid
end

# ============================================================
# Trip Pipeline
# ============================================================

function run_trip(;
    origin::Tuple{Float64,Float64},
    destination::Tuple{Float64,Float64},
    algorithm::ShipRouting.Algorithm = ShipRouting.DIJKSTRA,
    ship_speed::Float64 = 15.0,
    max_wave::Float64 = 4.0,
    weather_hour::Int = 0,
    grid::Union{H3Grid.HexGrid, Nothing} = nothing,
)
    # Step 1: Grid
    if grid === nothing
        grid = get_grid()
    end

    # Step 2: Compute bounding box for weather grid from H3 centers
    lats_grid = [c[1] for c in grid.centers]
    lons_grid = [c[2] for c in grid.centers]
    sw = (minimum(lats_grid) - 0.1, minimum(lons_grid) - 0.1)
    ne = (maximum(lats_grid) + 0.1, maximum(lons_grid) + 0.1)

    # Step 3: Initialize ShipRouting
    println("\nInitializing routing engine...")
    ShipRouting.init()
    state = ShipRouting.create_state(ship_speed=ship_speed, max_wave=max_wave)

    # Minimal land mask for weather interpolation bounds (all-water since H3 handles land)
    ShipRouting.set_land!(state,
        ShipRouting.LatLon(sw[1], sw[2]),
        ShipRouting.LatLon(ne[1], ne[2]),
        falses(5, 5))

    # Inject H3 graph directly (bypasses PRM)
    println("\nInjecting H3 graph...")
    H3Grid.set_graph_from_h3!(state, grid, ShipRouting_mod=ShipRouting)
    nc = ShipRouting.node_count(state)
    println("  Graph nodes: $nc")

    # Step 4: Set voyage
    println("\nVoyage: ($(origin[1]), $(origin[2])) → ($(destination[1]), $(destination[2]))")
    ShipRouting.set_voyage!(state,
        ShipRouting.LatLon(origin[1], origin[2]),
        ShipRouting.LatLon(destination[1], destination[2]))

    # Step 5: Push weather
    println("Pushing weather (hour offset: $weather_hour)...")
    base_wave = 0.5 + weather_hour * 0.05
    base_wind = 3.0 + weather_hour * 0.3
    n_wx = 25  # 5x5 grid
    weather = ShipRouting.WeatherGrid(
        ShipRouting.LatLon(sw[1], sw[2]),
        ShipRouting.LatLon(ne[1], ne[2]),
        UInt32(5), UInt32(5),
        fill(base_wind, n_wx),
        fill(180.0, n_wx),
        fill(base_wave, n_wx),
        fill(180.0, n_wx),
        fill(0.2, n_wx),
        fill(90.0, n_wx),
        Float64(weather_hour),
    )
    ShipRouting.push_weather!(state, weather)

    # Step 6: Route!
    alg_names = Dict(
        ShipRouting.DIJKSTRA => "Dijkstra",
    )
    println("\nRouting with $(get(alg_names, algorithm, "?"))...")
    result = ShipRouting.route(state, algorithm=algorithm)

    c = ShipRouting.cost(result)
    pl = ShipRouting.path_length(result)
    wps = ShipRouting.waypoints(result)

    # Step 7: Print results
    println("\n" * "="^70)
    println("ROUTE RESULT")
    println("="^70)
    println("  Algorithm:   $(get(alg_names, algorithm, "?"))")
    println("  Cost:        $c (scaled) = $(round(Float64(c)/1000.0, digits=2)) hours")
    println("  Waypoints:   $pl")
    if length(wps) >= 2
        println("  Distance:    ~$(round(_total_distance(wps), digits=1)) nm")
    end
    println()

    println("  #  │  Latitude  │ Longitude │   ETA   │ Speed │ Wave │ Wind │  Hdg  │ Node")
    println("─────┼────────────┼───────────┼─────────┼───────┼──────┼──────┼───────┼─────")
    for (i, wp) in enumerate(wps)
        @printf(" %3d │ %9.4f  │ %8.4f  │ %5.1f h │ %4.1f kt│ %3.1f m│ %4.1f │ %5.0f° │ %d\n",
            i, wp.lat, wp.lon, wp.eta_hours, wp.speed_knots,
            wp.wave_height, wp.wind_speed, wp.heading_deg, wp.node_id)
    end

    # Cleanup
    ShipRouting.cleanup()

    println("\nRoute GeoJSON (for map display):")
    _print_geojson(wps)

    (result=result, waypoints=wps, cost=c, grid=grid)
end

function _total_distance(wps)
    d = 0.0
    for i in 2:length(wps)
        d += _haversine_nm(wps[i-1].lat, wps[i-1].lon, wps[i].lat, wps[i].lon)
    end
    d
end

function _haversine_nm(lat1, lon1, lat2, lon2)
    R_nm = 3440.065  # Earth radius in nautical miles
    dlat = deg2rad(lat2 - lat1)
    dlon = deg2rad(lon2 - lon1)
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2
    2 * R_nm * asin(sqrt(a))
end

function _print_geojson(wps)
    coords = ["[$(wp.lon), $(wp.lat)]" for wp in wps]
    println("""{"type":"Feature","geometry":{"type":"LineString","coordinates":[$(join(coords, ","))]}}""")
end

# ============================================================
# CLI
# ============================================================

function main()
    if "--list-routes" in ARGS
        println("Preset routes:")
        for (key, (from, to, name)) in sort(collect(ROUTES))
            @printf("  %-20s  %s  (%.2f,%.2f) → (%.2f,%.2f)\n",
                key, name, from[1], from[2], to[1], to[2])
        end
        return
    end

    force_rebuild = "--build-grid" in ARGS

    # Parse origin/destination
    origin = (37.9475, 23.6378)       # Piraeus default
    destination = (37.4467, 25.3289)  # Mykonos default

    for (i, arg) in enumerate(ARGS)
        if arg == "--from" && i < length(ARGS)
            parts = split(ARGS[i+1], ",")
            origin = (parse(Float64, parts[1]), parse(Float64, parts[2]))
        elseif arg == "--to" && i < length(ARGS)
            parts = split(ARGS[i+1], ",")
            destination = (parse(Float64, parts[1]), parse(Float64, parts[2]))
        elseif haskey(ROUTES, arg)
            origin, destination, name = ROUTES[arg]
            println("Using preset: $name")
        end
    end

    grid = get_grid(force_rebuild=force_rebuild)

    run_trip(
        origin = origin,
        destination = destination,
        algorithm = ShipRouting.DIJKSTRA,
        ship_speed = 15.0,
        max_wave = 4.0,
        weather_hour = 0,
        grid = grid,
    )
end

main()
