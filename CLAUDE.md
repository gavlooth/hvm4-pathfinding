# CLAUDE.md - Maritime Routing Project

## Project Overview

Ship Weather Routing (SWR) engine with H3 hexagonal grid, multiple pathfinding algorithms, and Leaflet map frontend.

## Build & Test

```bash
cd c3lib && c3c build pathfind-so   # Build C3 shared library
julia julia/server.jl               # Start HTTP server (port 8080)
julia julia/trip.jl                  # CLI: Piraeus → Mykonos
julia julia/trip.jl --build-grid     # Rebuild H3 grid cache
```

## Project Structure

```
c3lib/src/       — C3 routing engine (Dijkstra, ALT, CCH, Hub Label, Temporal)
julia/           — Julia wrappers (ShipRouting.jl, H3Grid.jl, server.jl, trip.jl)
frontend/public/ — Leaflet map UI (map.html)
```

## Algorithms

| # | Algorithm | Description |
|---|-----------|-------------|
| 2 | Dijkstra  | Fibonacci heap shortest path |
| 3 | ALT       | A* with landmark triangle inequality |
| 4 | CCH       | Customizable Contraction Hierarchies |
| 5 | Hub Label | Pruned landmark labeling |
| 6 | Temporal  | Time-dependent multi-weather routing |

## H3 Grid

- Multi-resolution: base res-6 (~3.2 km open water), coast res-8 (~0.5 km near islands)
- GSHHG coastline data for land/water classification
- Land-crossing edge filter via segment sampling
- Grid cached at `julia/data/aegean_grid.txt`
