#!/usr/bin/env julia
"""
Smoke test for ShipRouting.jl — exercises the full FFI pipeline:
  init → state → land → PRM → voyage → weather → route → waypoints → cleanup
"""

include("ShipRouting.jl")
using .ShipRouting
using Printf

passed = 0
failed = 0

function check(ok::Bool, name::String)
    global passed, failed
    if ok
        println("PASS  $name")
        passed += 1
    else
        println("FAIL  $name")
        failed += 1
    end
end

# --- Lifecycle ---
ShipRouting.init()

# --- State creation ---
state = create_state(ship_speed=15.0, max_wave=4.0)
check(state.ptr != C_NULL, "create_state")

# --- Land mask (all sea) ---
set_land!(state, LatLon(37.0, 23.0), LatLon(39.0, 27.0), falses(5, 5))
check(true, "set_land")  # no exception = pass

# --- PRM ---
build_prm!(state, n_samples=10, k_neighbors=3, seed=42)
nc = ShipRouting.node_count(state)
check(nc == 10, "node_count=$nc")

# --- Voyage ---
set_voyage!(state, LatLon(37.5, 23.5), LatLon(38.5, 26.5))
check(true, "set_voyage")

# --- Weather (calm, uniform 2x2) ---
grid = WeatherGrid(
    LatLon(37.0, 23.0), LatLon(39.0, 27.0),
    UInt32(2), UInt32(2),
    fill(3.0, 4),    # wind_speed
    fill(180.0, 4),  # wind_dir
    fill(0.5, 4),    # wave_height
    fill(180.0, 4),  # wave_dir
    fill(0.2, 4),    # current_speed
    fill(90.0, 4),   # current_dir
    0.0              # timestamp
)
push_weather!(state, grid)
check(true, "push_weather")

# --- Route with Dijkstra ---
result = route(state, algorithm=DIJKSTRA)
check(result.ptr != C_NULL, "route_dijkstra")

c = cost(result)
check(c > 0 && c < 999_999, "route_cost=$c")

pl = path_length(result)
check(pl >= 2, "path_length=$pl")

# --- Waypoints ---
wps = waypoints(result)
check(length(wps) >= 2, "waypoints_count=$(length(wps))")
check(wps[1].eta_hours == 0.0, "waypoint_origin_eta")

if length(wps) >= 2
    check(wps[2].eta_hours > 0.0, "waypoint_eta_increases")
    check(wps[2].speed_knots > 0.0, "waypoint_speed_positive")
    check(37.0 < wps[1].lat < 40.0, "waypoint_lat_range")
end

# --- Print waypoints ---
println("\nRoute waypoints:")
for (i, wp) in enumerate(wps)
    @printf("  %d: (%.4f, %.4f) ETA=%.2fh speed=%.1fkt wave=%.1fm wind=%.1fkt hdg=%.0f° node=%d\n",
        i, wp.lat, wp.lon, wp.eta_hours, wp.speed_knots,
        wp.wave_height, wp.wind_speed, wp.heading_deg, wp.node_id)
end

# --- Weather clear ---
clear_weather!(state)
check(true, "clear_weather")

# --- Cleanup ---
ShipRouting.cleanup()

# --- Results ---
println("\nResults: $passed passed, $failed failed")
exit(failed > 0 ? 1 : 0)
