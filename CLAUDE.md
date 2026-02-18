# CLAUDE.md - Maritime Routing Project

## Project Overview

Ship Weather Routing (SWR) engine with H3 hexagonal grid, multiple pathfinding engines (C3 Dijkstra + Vine IVM), and Leaflet map frontend.

## Build & Test

```bash
cd bridge && cargo build --release   # Build Vine IVM bridge (cdylib)
cd c3lib && c3c build pathfind-so    # Build C3 shared library
julia julia/server.jl                # Start HTTP server (port 8080)
julia julia/trip.jl                  # CLI: Piraeus → Mykonos
julia julia/trip.jl --build-grid     # Rebuild H3 grid cache
```

## Project Structure

```
bridge/          — Rust cdylib embedding Vine IVM (FFI graph injection)
vine/            — Vine pathfinding library (Dijkstra + parallel delta-stepping)
c3lib/src/       — C3 routing engine (Dijkstra with Fibonacci heap)
julia/           — Julia wrappers (ShipRouting.jl, H3Grid.jl, VineRouting.jl, server.jl)
frontend/public/ — Leaflet map UI (index.html)
```

## H3 Grid

- Multi-resolution: base res-6 (~3.2 km open water), coast res-8 (~0.5 km near islands)
- GSHHG coastline data for land/water classification
- Land-crossing edge filter via segment sampling
- Grid cached at `julia/data/aegean_grid.txt`
