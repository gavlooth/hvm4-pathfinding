"""
    VineRouting

Julia wrapper for Vine pathfinding via the embedded IVM cdylib bridge.

Two modes:
- Text protocol: `vine_route(state, input)` — parses graph from text (slow for large graphs)
- Graph FFI: `vine_route_graph(state, csr, src, tgt)` — CSR arrays via extrinsics (fast)
"""
module VineRouting

export vine_init, vine_init_ffi, vine_route, vine_route_graph, vine_destroy, parse_route_output

const LIBPATH = joinpath(@__DIR__, "..", "bridge", "target", "release", "libvine_pathfind.so")
const VINE_LIB_PATH = joinpath(@__DIR__, "..", "vine", "pathfind")
const VINE_MAIN_PATH = joinpath(@__DIR__, "..", "vine", "main.vi")
const VINE_MAIN_FFI_PATH = joinpath(@__DIR__, "..", "vine", "main_ffi.vi")

# Opaque state pointer
mutable struct VineState
    ptr::Ptr{Cvoid}
end

"""
    vine_init(; lib_path, main_path, root_path=C_NULL) -> VineState

Compile Vine pathfinding sources and return cached state.
Uses text-protocol main by default.
"""
function vine_init(;
    lib_path::String = VINE_LIB_PATH,
    main_path::String = VINE_MAIN_PATH,
    root_path = C_NULL
)
    _vine_init_impl(lib_path, main_path, root_path)
end

"""
    vine_init_ffi(; lib_path, main_path, root_path=C_NULL) -> VineState

Compile Vine FFI pathfinding sources (graph data via extrinsics, no text parsing).
"""
function vine_init_ffi(;
    lib_path::String = VINE_LIB_PATH,
    main_path::String = VINE_MAIN_FFI_PATH,
    root_path = C_NULL
)
    _vine_init_impl(lib_path, main_path, root_path)
end

function _vine_init_impl(lib_path, main_path, root_path)
    if !isfile(LIBPATH)
        error("Vine bridge not found at $LIBPATH — run: cd bridge && cargo build --release")
    end

    rp = root_path === C_NULL ? C_NULL : pointer(Vector{UInt8}(codeunits(root_path * "\0")))

    ptr = ccall((:vine_init, LIBPATH), Ptr{Cvoid},
        (Cstring, Cstring, Ptr{Cvoid}),
        lib_path, main_path, rp)

    ptr == C_NULL && error("vine_init failed — check stderr for compilation errors")

    state = VineState(ptr)
    finalizer(vine_destroy, state)
    state
end

"""
    vine_destroy(state::VineState)

Free the compiled Vine state.
"""
function vine_destroy(state::VineState)
    if state.ptr != C_NULL
        ccall((:vine_destroy, LIBPATH), Cvoid, (Ptr{Cvoid},), state.ptr)
        state.ptr = C_NULL
    end
end

"""
    vine_route(state, input; workers=1) -> String

Execute a pathfinding query using text protocol.
"""
function vine_route(state::VineState, input::String; workers::Integer = 1)
    state.ptr == C_NULL && error("VineState has been destroyed")

    out_buf = Vector{UInt8}(undef, 1048576)
    out_written = Ref{Cuint}(0)

    rc = GC.@preserve out_buf begin
        ccall((:vine_route, LIBPATH), Cint,
            (Ptr{Cvoid}, Cstring, Cuint, Ptr{UInt8}, Cuint, Ptr{Cuint}),
            state.ptr, input, Cuint(workers),
            pointer(out_buf), Cuint(length(out_buf)), out_written)
    end

    rc != 0 && error("vine_route failed with code $rc")
    String(out_buf[1:out_written[]])
end

"""
    vine_route_graph(state, row_ptr, col_idx, weights, source, target; algorithm=0, workers=4) -> String

Execute pathfinding with graph data passed directly as CSR arrays (no text parsing).
- `row_ptr`: UInt32 array of length V+1
- `col_idx`: UInt32 array of length E (edge targets)
- `weights`: UInt32 array of length E (edge weights)
- `source`, `target`: 0-indexed node IDs
- `algorithm`: 0=dijkstra, 1=delta_stepping (parallel)
"""
function vine_route_graph(
    state::VineState,
    row_ptr::Vector{UInt32},
    col_idx::Vector{UInt32},
    weights::Vector{UInt32},
    source::Integer,
    target::Integer;
    algorithm::Integer = 0,
    workers::Integer = 4
)
    state.ptr == C_NULL && error("VineState has been destroyed")

    node_count = UInt32(length(row_ptr) - 1)
    out_buf = Vector{UInt8}(undef, 1048576)
    out_written = Ref{Cuint}(0)

    rc = GC.@preserve row_ptr col_idx weights out_buf begin
        ccall((:vine_route_graph, LIBPATH), Cint,
            (Ptr{Cvoid}, Cuint, Ptr{Cuint}, Ptr{Cuint}, Ptr{Cuint},
             Cuint, Cuint, Cuint, Cuint,
             Ptr{UInt8}, Cuint, Ptr{Cuint}),
            state.ptr, node_count,
            pointer(row_ptr), pointer(col_idx), pointer(weights),
            Cuint(source), Cuint(target), Cuint(algorithm), Cuint(workers),
            pointer(out_buf), Cuint(length(out_buf)), out_written)
    end

    rc != 0 && error("vine_route_graph failed with code $rc")
    String(out_buf[1:out_written[]])
end

"""
    parse_route_output(output) -> (cost::Int, path::Vector{Int})

Parse Vine route output text into cost and path.
"""
function parse_route_output(output::String)
    cost = -1
    path = Int[]
    for line in split(strip(output), '\n')
        if startswith(line, "COST ")
            cost = parse(Int, split(line)[2])
        elseif startswith(line, "PATH ")
            parts = split(line)[2:end]
            path = [parse(Int, p) for p in parts]
        end
    end
    (cost=cost, path=path)
end

end # module
