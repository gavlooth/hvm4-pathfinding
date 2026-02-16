"""
    H3Grid

Multi-resolution H3 hexagonal grid for maritime routing.
Uses GSHHG coastline data to classify hexes as land/water,
then builds a sparse adjacency graph over water hexes.

The graph feeds directly into ShipRouting.jl via set_land!/build_prm!
or as a custom graph replacement.
"""
module H3Grid

using H3_jll
using Shapefile

export build_aegean_grid, HexGrid, hex_center, hex_neighbors
export load_coastline, is_land, grid_to_land_mask
export grid_stats, save_grid, load_grid, set_graph_from_h3!

const LIB = H3_jll.libh3

# ============================================================
# H3 Low-Level Wrappers
# ============================================================

struct LatLng
    lat::Float64  # radians
    lng::Float64  # radians
end

function latlng_to_cell(lat_deg::Float64, lng_deg::Float64, res::Int)::UInt64
    ll = LatLng(deg2rad(lat_deg), deg2rad(lng_deg))
    h = Ref{UInt64}(0)
    err = ccall((:latLngToCell, LIB), UInt32, (Ref{LatLng}, Cint, Ref{UInt64}),
                Ref(ll), res, h)
    err == 0 || error("latLngToCell failed: $err")
    h[]
end

function cell_to_latlng(h::UInt64)
    ll = Ref{LatLng}(LatLng(0.0, 0.0))
    err = ccall((:cellToLatLng, LIB), UInt32, (UInt64, Ref{LatLng}), h, ll)
    err == 0 || error("cellToLatLng failed: $err")
    (rad2deg(ll[].lat), rad2deg(ll[].lng))
end

function grid_disk(h::UInt64, k::Int)::Vector{UInt64}
    # Max size for grid disk of radius k
    max_size = Ref{Int64}(0)
    ccall((:maxGridDiskSize, LIB), UInt32, (Cint, Ref{Int64}), k, max_size)
    out = Vector{UInt64}(undef, max_size[])
    err = ccall((:gridDisk, LIB), UInt32, (UInt64, Cint, Ptr{UInt64}), h, k, out)
    err == 0 || error("gridDisk failed: $err")
    filter(!=(UInt64(0)), out)
end

function grid_ring(h::UInt64, k::Int)::Vector{UInt64}
    # Ring at exactly distance k (6*k cells for k>0, 1 for k=0)
    n = k == 0 ? 1 : 6 * k
    out = Vector{UInt64}(undef, n)
    err = ccall((:gridRingUnsafe, LIB), UInt32, (UInt64, Cint, Ptr{UInt64}), h, k, out)
    if err != 0
        # Fallback: use gridDisk and subtract inner
        all_k = Set(grid_disk(h, k))
        if k > 0
            inner = Set(grid_disk(h, k - 1))
            return collect(setdiff(all_k, inner))
        else
            return collect(all_k)
        end
    end
    filter(!=(UInt64(0)), out)
end

function cell_to_children(h::UInt64, child_res::Int)::Vector{UInt64}
    n = Ref{Int64}(0)
    ccall((:cellToChildrenSize, LIB), UInt32, (UInt64, Cint, Ref{Int64}), h, child_res, n)
    out = Vector{UInt64}(undef, n[])
    err = ccall((:cellToChildren, LIB), UInt32, (UInt64, Cint, Ptr{UInt64}), h, child_res, out)
    err == 0 || error("cellToChildren failed: $err")
    out
end

function cell_to_parent(h::UInt64, parent_res::Int)::UInt64
    out = Ref{UInt64}(0)
    err = ccall((:cellToParent, LIB), UInt32, (UInt64, Cint, Ref{UInt64}), h, parent_res, out)
    err == 0 || error("cellToParent failed: $err")
    out[]
end

function get_resolution(h::UInt64)::Int
    Int(ccall((:getResolution, LIB), Cint, (UInt64,), h))
end

function are_neighbor_cells(a::UInt64, b::UInt64)::Bool
    out = Ref{Cint}(0)
    err = ccall((:areNeighborCells, LIB), UInt32, (UInt64, UInt64, Ref{Cint}), a, b, out)
    err == 0 || return false
    out[] != 0
end

function get_hexagon_edge_length_avg(res::Int)::Float64
    out = Ref{Cdouble}(0.0)
    ccall((:getHexagonEdgeLengthAvgKm, LIB), UInt32, (Cint, Ref{Cdouble}), res, out)
    out[]
end

# ============================================================
# Coastline Loading (GSHHG Shapefile)
# ============================================================

struct Coastline
    polygons::Vector{Vector{Tuple{Float64,Float64}}}  # [(lon,lat), ...]
end

"""
    load_coastline(shapefile_path; bbox=nothing)

Load GSHHG L1 shapefile. Optionally filter to bounding box (sw_lat, sw_lon, ne_lat, ne_lon).
"""
function load_coastline(path::String; bbox=nothing)
    table = Shapefile.Table(path)
    polys = Vector{Vector{Tuple{Float64,Float64}}}()

    for row in table
        geom = Shapefile.shape(row)
        for ring in _extract_rings(geom)
            if bbox !== nothing
                sw_lat, sw_lon, ne_lat, ne_lon = bbox
                # Quick bounding box check
                lons = [p[1] for p in ring]
                lats = [p[2] for p in ring]
                if maximum(lons) < sw_lon || minimum(lons) > ne_lon ||
                   maximum(lats) < sw_lat || minimum(lats) > ne_lat
                    continue
                end
            end
            push!(polys, ring)
        end
    end

    println("Loaded $(length(polys)) coastline polygons")
    Coastline(polys)
end

function _extract_rings(geom::Shapefile.Polygon)
    rings = Vector{Vector{Tuple{Float64,Float64}}}()
    points = geom.points
    for i in 1:length(geom.parts)
        start_idx = geom.parts[i] + 1  # 0-indexed to 1-indexed
        end_idx = i < length(geom.parts) ? geom.parts[i+1] : length(points)
        ring = [(Float64(points[j].x), Float64(points[j].y)) for j in start_idx:end_idx]
        push!(rings, ring)
    end
    rings
end

function _extract_rings(geom)
    # Fallback for other geometry types
    Vector{Vector{Tuple{Float64,Float64}}}()
end

"""
    is_land(coastline, lat, lon) → Bool

Ray casting point-in-polygon test. Returns true if point is inside any coastline polygon.
"""
function is_land(coast::Coastline, lat::Float64, lon::Float64)::Bool
    for poly in coast.polygons
        if _point_in_polygon(lon, lat, poly)
            return true
        end
    end
    false
end

function _point_in_polygon(x::Float64, y::Float64, poly::Vector{Tuple{Float64,Float64}})::Bool
    n = length(poly)
    inside = false
    j = n
    for i in 1:n
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)
            inside = !inside
        end
        j = i
    end
    inside
end

# ============================================================
# Multi-Resolution H3 Grid
# ============================================================

struct HexGrid
    cells::Vector{UInt64}           # all water hex IDs
    centers::Vector{Tuple{Float64,Float64}}  # (lat, lon) per cell
    neighbors::Vector{Vector{Int}}  # adjacency list (1-indexed into cells)
    cell_to_idx::Dict{UInt64,Int}   # hex ID → index
    base_res::Int
    coast_res::Int
end

"""
    build_aegean_grid(; coastline_path, base_res=5, coast_res=7, refine_dist_km=5.0,
                       bbox=(34.5, 22.0, 41.5, 30.0))

Build multi-resolution H3 grid over the Aegean Sea.

1. Tile bounding box at base_res
2. Refine hexes near coastline to coast_res
3. Discard land hexes
4. Build adjacency graph
"""
function build_aegean_grid(;
    coastline_path::String,
    base_res::Int = 5,
    coast_res::Int = 7,
    refine_dist_km::Float64 = 5.0,
    bbox::NTuple{4,Float64} = (34.5, 22.0, 41.5, 30.0)  # Aegean: SW_lat, SW_lon, NE_lat, NE_lon
)
    sw_lat, sw_lon, ne_lat, ne_lon = bbox

    println("Loading coastline...")
    coast = load_coastline(coastline_path; bbox=bbox)

    println("Generating base hex grid at resolution $base_res...")
    base_cells = _tile_bbox(sw_lat, sw_lon, ne_lat, ne_lon, base_res)
    println("  $(length(base_cells)) base cells")

    println("Classifying land/water at base resolution...")
    water_base = UInt64[]
    land_base = UInt64[]
    coast_base = UInt64[]  # cells near coastline — candidates for refinement

    for h in base_cells
        lat, lon = cell_to_latlng(h)
        if is_land(coast, lat, lon)
            push!(land_base, h)
        else
            # Check if any neighbor is land (= near coast)
            near_coast = false
            for nb in grid_ring(h, 1)
                nb_lat, nb_lon = cell_to_latlng(nb)
                if is_land(coast, nb_lat, nb_lon)
                    near_coast = true
                    break
                end
            end
            if near_coast
                push!(coast_base, h)
            else
                push!(water_base, h)
            end
        end
    end
    println("  $(length(water_base)) open water, $(length(coast_base)) near-coast, $(length(land_base)) land")

    println("Refining near-coast cells to resolution $coast_res...")
    refined_water = UInt64[]
    for h in coast_base
        children = cell_to_children(h, coast_res)
        for child in children
            lat, lon = cell_to_latlng(child)
            if !is_land(coast, lat, lon)
                push!(refined_water, child)
            end
        end
    end
    println("  $(length(refined_water)) refined water cells")

    # Combine: open water at base_res + refined water at coast_res
    all_water = vcat(water_base, refined_water)
    println("Total water cells: $(length(all_water))")

    println("Building adjacency graph (with land-crossing filter)...")
    grid = _build_adjacency(all_water, base_res, coast_res; coast=coast)

    println("Grid complete: $(length(grid.cells)) nodes, $(sum(length.(grid.neighbors))) edges")
    grid
end

function _tile_bbox(sw_lat, sw_lon, ne_lat, ne_lon, res)
    # Sample points on a fine regular grid, convert to H3, deduplicate
    edge_km = get_hexagon_edge_length_avg(res)
    step_deg = edge_km / 111.0 * 0.8  # ~80% of edge length in degrees

    cells = Set{UInt64}()
    lat = sw_lat
    while lat <= ne_lat
        lon = sw_lon
        while lon <= ne_lon
            h = latlng_to_cell(lat, lon, res)
            push!(cells, h)
            lon += step_deg
        end
        lat += step_deg
    end
    collect(cells)
end

"""
    _segment_clear(coast, lat1, lon1, lat2, lon2; steps=5) → Bool

Check that the line segment between two points doesn't cross land.
Samples `steps` intermediate points along the great-circle path.
"""
function _segment_clear(coast::Coastline, lat1, lon1, lat2, lon2; steps::Int=5)
    for k in 1:steps
        t = k / (steps + 1)
        mlat = lat1 + t * (lat2 - lat1)
        mlon = lon1 + t * (lon2 - lon1)
        if is_land(coast, mlat, mlon)
            return false
        end
    end
    true
end

function _build_adjacency(cells::Vector{UInt64}, base_res::Int, coast_res::Int;
                          coast::Union{Coastline,Nothing}=nothing)
    cell_set = Set(cells)
    cell_to_idx = Dict{UInt64,Int}()
    for (i, h) in enumerate(cells)
        cell_to_idx[h] = i
    end

    centers = [cell_to_latlng(h) for h in cells]
    neighbors = [Int[] for _ in 1:length(cells)]
    n_filtered = Ref(0)

    function try_add_edge!(i, j)
        if coast !== nothing
            lat1, lon1 = centers[i]
            lat2, lon2 = centers[j]
            dist = _haversine_km(lat1, lon1, lat2, lon2)
            # Fine-res edges (<2 km) skip check — too short to cross land
            # Base-res and cross-res edges get checked
            if dist > 2.0 && !_segment_clear(coast, lat1, lon1, lat2, lon2)
                n_filtered[] += 1
                return
            end
        end
        if j ∉ neighbors[i]
            push!(neighbors[i], j)
        end
    end

    for (i, h) in enumerate(cells)
        res = get_resolution(h)
        # Direct neighbors at same resolution
        for nb in grid_ring(h, 1)
            if haskey(cell_to_idx, nb)
                j = cell_to_idx[nb]
                try_add_edge!(i, j)
            end
        end

        # Cross-resolution edges: if this is a base_res cell,
        # connect to coast_res cells that are children of our neighbors
        if res == base_res
            for nb in grid_ring(h, 1)
                # Check if neighbor was refined (not in our set but its children might be)
                if !haskey(cell_to_idx, nb)
                    # This neighbor was refined — find its children that are in our set
                    for child in cell_to_children(nb, coast_res)
                        if haskey(cell_to_idx, child)
                            j = cell_to_idx[child]
                            child_lat, child_lon = centers[j]
                            my_lat, my_lon = centers[i]
                            dist = _haversine_km(my_lat, my_lon, child_lat, child_lon)
                            edge_km = get_hexagon_edge_length_avg(base_res)
                            if dist < edge_km * 2.0
                                try_add_edge!(i, j)
                                try_add_edge!(j, i)
                            end
                        end
                    end
                end
            end
        end
    end

    if n_filtered[] > 0
        println("  Filtered $(n_filtered[]) land-crossing edges")
    end

    HexGrid(cells, centers, neighbors, cell_to_idx, base_res, coast_res)
end

function _haversine_km(lat1, lon1, lat2, lon2)
    R = 6371.0
    dlat = deg2rad(lat2 - lat1)
    dlon = deg2rad(lon2 - lon1)
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2
    2 * R * asin(sqrt(a))
end

# ============================================================
# Utilities
# ============================================================

function hex_center(grid::HexGrid, idx::Int)
    grid.centers[idx]
end

function hex_neighbors(grid::HexGrid, idx::Int)
    grid.neighbors[idx]
end

function grid_stats(grid::HexGrid)
    n = length(grid.cells)
    e = sum(length.(grid.neighbors))
    base_count = count(h -> get_resolution(h) == grid.base_res, grid.cells)
    fine_count = n - base_count
    avg_deg = e / n

    println("H3 Grid Statistics:")
    println("  Total nodes:    $n")
    println("  Total edges:    $e (directed)")
    println("  Base res $(grid.base_res): $base_count nodes")
    println("  Fine res $(grid.coast_res): $fine_count nodes")
    println("  Avg degree:     $(round(avg_deg, digits=1))")
    println("  Edge length (base): $(round(get_hexagon_edge_length_avg(grid.base_res), digits=1)) km")
    println("  Edge length (fine): $(round(get_hexagon_edge_length_avg(grid.coast_res), digits=1)) km")
end

"""
    grid_to_land_mask(grid; rows=100, cols=100) → (Matrix{Bool}, sw, ne)

Convert hex grid into a regular land mask for ShipRouting.set_land!
True = land (no water hex covers this cell).
"""
function grid_to_land_mask(grid::HexGrid; rows::Int=100, cols::Int=100)
    # Compute bounding box from grid centers
    lats = [c[1] for c in grid.centers]
    lons = [c[2] for c in grid.centers]
    sw_lat = minimum(lats) - 0.1
    sw_lon = minimum(lons) - 0.1
    ne_lat = maximum(lats) + 0.1
    ne_lon = maximum(lons) + 0.1

    mask = trues(rows, cols)  # start all land

    for (lat, lon) in grid.centers
        # Map to grid cell
        r = clamp(round(Int, (lat - sw_lat) / (ne_lat - sw_lat) * (rows - 1)) + 1, 1, rows)
        c = clamp(round(Int, (lon - sw_lon) / (ne_lon - sw_lon) * (cols - 1)) + 1, 1, cols)
        mask[r, c] = false  # water
    end

    (mask, (sw_lat, sw_lon), (ne_lat, ne_lon))
end

"""
    find_nearest(grid, lat, lon) → index

Find the grid cell closest to the given lat/lon.
"""
function find_nearest(grid::HexGrid, lat::Float64, lon::Float64)::Int
    best_i = 1
    best_d = Inf
    for (i, (clat, clon)) in enumerate(grid.centers)
        d = _haversine_km(lat, lon, clat, clon)
        if d < best_d
            best_d = d
            best_i = i
        end
    end
    best_i
end

"""
    save_grid(path, grid)

Save grid to a simple text format for reuse.
"""
function save_grid(path::String, grid::HexGrid)
    open(path, "w") do f
        println(f, "H3GRID v1 $(grid.base_res) $(grid.coast_res)")
        println(f, "CELLS $(length(grid.cells))")
        for h in grid.cells
            println(f, string(h, base=16))
        end
        println(f, "ADJACENCY")
        for nbs in grid.neighbors
            println(f, join(nbs, ","))
        end
    end
    println("Saved grid to $path")
end

"""
    load_grid(path) → HexGrid

Load a previously saved grid.
"""
function load_grid(path::String)::HexGrid
    lines = readlines(path)
    header = split(lines[1])
    base_res = parse(Int, header[3])
    coast_res = parse(Int, header[4])

    n = parse(Int, split(lines[2])[2])
    cells = UInt64[]
    for i in 3:(2+n)
        push!(cells, parse(UInt64, lines[i], base=16))
    end

    # Skip "ADJACENCY" header
    neighbors = Vector{Int}[]
    for i in (3+n+1):(2+2n+1)
        if i > length(lines) || isempty(strip(lines[i]))
            push!(neighbors, Int[])
        else
            push!(neighbors, parse.(Int, split(strip(lines[i]), ",")))
        end
    end

    centers = [cell_to_latlng(h) for h in cells]
    cell_to_idx = Dict{UInt64,Int}(h => i for (i, h) in enumerate(cells))

    HexGrid(cells, centers, neighbors, cell_to_idx, base_res, coast_res)
end

# ============================================================
# Direct Graph Injection into ShipRouting
# ============================================================

function _haversine_m(lat1, lon1, lat2, lon2)
    _haversine_km(lat1, lon1, lat2, lon2) * 1000.0
end

function _bearing(lat1, lon1, lat2, lon2)
    φ1 = deg2rad(lat1); φ2 = deg2rad(lat2)
    Δλ = deg2rad(lon2 - lon1)
    y = sin(Δλ) * cos(φ2)
    x = cos(φ1) * sin(φ2) - sin(φ1) * cos(φ2) * cos(Δλ)
    θ = atan(y, x)
    mod(rad2deg(θ) + 360.0, 360.0)
end

"""
    set_graph_from_h3!(state, grid::HexGrid; ShipRouting_mod=nothing)

Flatten the HexGrid adjacency into edge arrays and inject into the SWR engine
via ShipRouting.set_graph!. Bypasses PRM entirely.

`ShipRouting_mod` should be the ShipRouting module reference (auto-detected if
loaded via `include("ShipRouting.jl"); using .ShipRouting`).
"""
function set_graph_from_h3!(state, grid::HexGrid; ShipRouting_mod=nothing)
    # Auto-detect ShipRouting module from Main
    if ShipRouting_mod === nothing
        ShipRouting_mod = Base.Main.ShipRouting
    end

    n = length(grid.cells)
    lats = Float64[c[1] for c in grid.centers]
    lons = Float64[c[2] for c in grid.centers]

    # Flatten adjacency into directed edge arrays
    edge_from = UInt32[]
    edge_to = UInt32[]
    edge_dist = Float64[]
    edge_hdg = Float64[]

    for (i, nbs) in enumerate(grid.neighbors)
        for j in nbs
            push!(edge_from, UInt32(i - 1))  # 0-indexed for C
            push!(edge_to, UInt32(j - 1))
            d = _haversine_m(lats[i], lons[i], lats[j], lons[j])
            h = _bearing(lats[i], lons[i], lats[j], lons[j])
            push!(edge_dist, d)
            push!(edge_hdg, h)
        end
    end

    println("Injecting H3 graph: $n nodes, $(length(edge_from)) directed edges")
    ShipRouting_mod.set_graph!(state,
        lats=lats, lons=lons,
        edge_from=edge_from, edge_to=edge_to,
        edge_dist_m=edge_dist, edge_heading=edge_hdg)
end

end # module
