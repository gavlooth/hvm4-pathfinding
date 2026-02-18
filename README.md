# Maritime Ship Weather Routing

Optimal ship routing through weather on an H3 hexagonal grid. Multiple pathfinding engines (C3 Dijkstra, Vine IVM parallel delta-stepping), Leaflet map frontend.

## Core Idea: Metric Space Transformation

**Algorithms are metric-agnostic.** The domain knowledge — weather physics, ship hydrodynamics, fuel economics — lives entirely in the **edge weight computation**. Pathfinding algorithms see only numbers.

```
                         Weather Forecast (BN, direction)
                                    |
  H3 Grid (topology)               v
  nodes + edges -------> [ Kwon Speed Loss Model ] -------> weighted CSR
       (fixed)           ship params, weather, heading       (time or fuel cost)
                                                                    |
                                                                    v
                                                        [ Any shortest-path algorithm ]
                                                        Dijkstra, Delta-Stepping, ...
                                                                    |
                                                                    v
                                                            Optimal route
```

The transformation: **distance (meters) -> cost (hours or kg fuel)** via the Kwon model. Change the weather forecast, recompute weights, re-route. Algorithms don't change.

For dynamic routing (re-routing every 3 hours): recompute edge weights with the updated forecast, run the algorithm from the ship's current position. The algorithm is the same — only the metric changes.

## Speed Loss Model: Kwon (2008)

Based on: Kwon, Y.J. "Speed loss due to added resistance in wind and waves." *The Naval Architect*, March 2008. Extension of Townsin-Kwon (1983) formulae validated against 60 hull forms and full-scale data.

### Percentage Speed Loss

```
ΔV/V = α · μ · (head weather speed loss)
```

Where:
- **V** = design service speed (knots)
- **ΔV** = speed reduction due to weather (knots)
- **α** = correction factor for hull form and Froude number
- **μ** = weather direction reduction factor

### Head Weather Speed Loss

The base speed loss in **head seas** (0-30 degrees from bow):

**Laden condition** (C_B = 0.75, 0.80, 0.85):
```
ΔV/V × 100 = 0.5·BN + BN^6.5 / (2.7·∇^(2/3))
```

**Ballast condition**:
```
ΔV/V × 100 = 0.7·BN + BN^6.5 / (2.7·∇^(2/3))
```

**Normal condition** (containerships, C_B = 0.55-0.70):
```
ΔV/V × 100 = 0.7·BN + BN^6.5 / (22·∇^(2/3))
```

Parameters:
- **BN** = Beaufort number (derived from wind speed)
- **∇** = displacement volume (m³)

### Correction Factor α

Depends on block coefficient C_B, Froude number F_n, and loading condition.

| C_B  | Condition | α                                |
|------|-----------|----------------------------------|
| 0.55 | normal    | 1.7 - 1.4·F_n - 7.4·F_n²        |
| 0.60 | normal    | 2.2 - 2.5·F_n - 9.7·F_n²        |
| 0.65 | normal    | 2.6 - 3.7·F_n - 11.6·F_n²       |
| 0.70 | normal    | 3.1 - 5.3·F_n - 12.4·F_n²       |
| 0.75 | laden     | 2.4 - 10.6·F_n - 9.5·F_n²       |
| 0.80 | laden     | 2.6 - 13.1·F_n - 15.1·F_n²      |
| 0.85 | laden     | 3.1 - 18.7·F_n + 28·F_n²        |
| 0.75 | ballast   | 2.6 - 12.5·F_n - 13.5·F_n²      |
| 0.80 | ballast   | 3.0 - 16.3·F_n - 21.6·F_n²      |
| 0.85 | ballast   | 3.4 - 20.9·F_n + 31.8·F_n²      |

Where:
- **C_B** = block coefficient (hull fullness: 0.55 fine, 0.85 full)
- **F_n** = Froude number = V / sqrt(g·L), range 0.05-0.30
- **L** = waterline length (m)

### Weather Direction Reduction Factor μ

Reduces the speed loss effect based on the angle between weather and ship heading.

| Angle from bow | μ formula                        |
|----------------|----------------------------------|
| 0° - 30°       | μ = 1.0 (head sea, full effect)  |
| 30° - 60°      | 2μ = 1.7 - 0.03·(BN - 4)²       |
| 60° - 150°     | 2μ = 0.9 - 0.06·(BN - 6)²       |
| 150° - 180°    | 2μ = 0.4 - 0.03·(BN - 8)²       |

Following seas (150-180°) can actually *increase* speed at low Beaufort numbers.

### Beaufort Number from Wind Speed

| BN | Wind Speed (m/s) | Description    |
|----|-------------------|----------------|
| 0  | 0 - 0.2           | Calm           |
| 1  | 0.3 - 1.5         | Light air      |
| 2  | 1.6 - 3.3         | Light breeze   |
| 3  | 3.4 - 5.4         | Gentle breeze  |
| 4  | 5.5 - 7.9         | Moderate       |
| 5  | 8.0 - 10.7        | Fresh          |
| 6  | 10.8 - 13.8       | Strong         |
| 7  | 13.9 - 17.1       | Near gale      |
| 8  | 17.2 - 20.7       | Gale           |

Continuous approximation: `BN ≈ (wind_speed_ms / 0.836)^(2/3)`

### Edge Cost Computation

For each edge (u → v) in the graph:

```
1. Sample weather at edge midpoint:
   wind_speed, wind_dir from forecast grid (bilinear interpolation)

2. Compute Beaufort number:
   BN = (wind_speed / 0.836)^(2/3)

3. Compute head weather speed loss:
   loss% = 0.5·BN + BN^6.5 / (2.7·∇^(2/3))    [laden]

4. Apply correction factor α:
   α = f(C_B, F_n, condition)                    [from table]

5. Compute weather-heading angle:
   θ = |wind_dir - edge_heading|

6. Apply direction factor μ:
   μ = f(θ, BN)                                  [from table]

7. Effective speed:
   V_eff = V_design × (1 - α·μ·loss%/100)
   V_eff = max(V_eff, 0.5)                       [minimum steerage]

8. Add current effect:
   V_eff += current_speed × cos(current_dir - heading) × 1.94384

9. Edge cost (time):
   cost_hours = (dist_m / 1852) / V_eff

10. Edge cost (fuel):
    fuel_kg = cost_hours × (fuel_rate_base + fuel_rate_per_kt × V_eff)

11. Edge weight (integer for algorithm):
    weight = round(cost × scale)
```

### Cost Modes

| Mode              | Objective                              |
|-------------------|----------------------------------------|
| TIME_ONLY         | Minimize total voyage time (hours)     |
| FUEL_ONLY         | Minimize total fuel consumption (kg)   |
| WEIGHTED_SUM      | α·time + (1-α)·fuel                   |
| SAFETY_PENALIZED  | Weighted sum + penalty if BN > threshold |

## Dynamic Re-routing

For voyages spanning multiple forecast windows:

1. Compute route using current forecast (t=0)
2. Ship sails for 3 hours along the route
3. New forecast arrives → recompute edge weights from ship's current position
4. Re-route with updated weights
5. Repeat until arrival

The algorithm is unchanged at each step — only the edge weights (metric) change with each new forecast.

## Architecture

```
frontend/public/index.html    Leaflet map UI (single-file, no build step)
julia/server.jl               HTTP server (port 8080)
julia/ShipRouting.jl           C3 Dijkstra wrapper
julia/VineRouting.jl           Vine IVM wrapper (CSR graph injection)
julia/H3Grid.jl               H3 hexagonal grid (multi-res, land avoidance)
bridge/src/lib.rs              Rust cdylib — embedded IVM with FFI extrinsics
vine/pathfind/                 Vine pathfinding library
  dijkstra_ffi.vi              Sequential Dijkstra (Map-based PQ)
  delta_stepping_ffi.vi        Parallel delta-stepping (atomic dist/prev in Rust)
c3lib/src/                     C3 routing engine
  swr_dijkstra.c3              Dijkstra with Fibonacci heap
  swr.c3                       Edge cost model, weather sampling
```

### Edge Weight Pipeline

```
julia/server.jl                 julia/H3Grid.jl
build_csr_cache!()              grid topology (nodes, edges, haversine distances)
        |                              |
        v                              v
  [ Weather Sampling ]  +  [ Kwon Speed Loss ]  =  CSR weights (UInt32[])
        |
        v
  VineRouting.vine_route_graph(row_ptr, col_idx, weights, src, tgt, algorithm)
        |
        v
  bridge/src/lib.rs → IVM → vine/pathfind/*.vi
        |
        v
  COST + PATH (node IDs → lat/lon waypoints)
```

## H3 Grid

- Multi-resolution: base res-6 (~3.2 km open water), coast res-8 (~0.5 km near islands)
- GSHHG coastline data for land/water classification
- Land-crossing edge filter via segment sampling
- ~84k nodes, ~587k edges, avg degree ~7
- Grid cached at `julia/data/aegean_grid.txt`

## Build & Run

```bash
# Build Vine IVM bridge
cd bridge && cargo build --release

# Build C3 shared library
cd c3lib && c3c build pathfind-so

# Start server
julia julia/server.jl

# Open http://localhost:8080
```

## Algorithms

| Algorithm | Engine | Parallelism | Time (84k nodes) |
|-----------|--------|-------------|-------------------|
| Dijkstra (Fibonacci heap) | C3 | Sequential | ~5 ms |
| Dijkstra (Map PQ) | Vine IVM | Sequential | ~14 s |
| Delta-Stepping | Vine IVM | Tree-parallel (atomic FFI) | ~266 ms |

Delta-stepping: all mutable state (dist/prev arrays, bucket queues) in Rust with `AtomicU64`. Vine dispatches tree-parallel vertex relaxations via IVM work-stealing.

## References

1. Kwon, Y.J. "Speed loss due to added resistance in wind and waves." *The Naval Architect*, March 2008.
2. Townsin, R.L. and Kwon, Y.J. "Approximate Formulae for the Speed Loss Due to Added Resistance in Wind and Waves." *Trans RINA*, Vol 125, 1983.
3. ISO 15016:2002. "Guidelines for the Assessment of Speed and Power Performance by Analysis of Speed Trial Data."
