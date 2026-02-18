use std::ffi::CStr;
use std::io::{Cursor, Read};
use std::os::raw::{c_char, c_int, c_uint};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use ivy::ast::Nets;
use ivy::host::Host;
use ivy::optimize::Optimizer;
use ivm::ext::{ExtList, Extrinsics};
use ivm::heap::Heap;
use ivm::port::{Port, Tag};
use ivm::IVM;
use vine::compiler::Compiler;
use vine::components::loader::{Loader, RealFS};
use vine::structures::ast::Ident;

/// Compiled Vine state cached across route calls.
pub struct VineState {
    nets: Nets,
}

/// CSR graph data passed from Julia/C.
/// SAFETY: Pointers must remain valid for the duration of the vine_route_graph call.
struct GraphCSR {
    node_count: u32,
    row_ptr: *const u32,
    col_idx: *const u32,
    weights: *const u32,
    source: u32,
    target: u32,
}

unsafe impl Send for GraphCSR {}
unsafe impl Sync for GraphCSR {}

impl GraphCSR {
    fn degree(&self, node: u32) -> u32 {
        if node >= self.node_count {
            return 0;
        }
        unsafe {
            let start = *self.row_ptr.add(node as usize);
            let end = *self.row_ptr.add(node as usize + 1);
            end - start
        }
    }

    fn adj_target(&self, node: u32, idx: u32) -> u32 {
        unsafe {
            let start = *self.row_ptr.add(node as usize);
            *self.col_idx.add((start + idx) as usize)
        }
    }

    fn adj_weight(&self, node: u32, idx: u32) -> u32 {
        unsafe {
            let start = *self.row_ptr.add(node as usize);
            *self.weights.add((start + idx) as usize)
        }
    }
}

/// Initialize: compile Vine pathfinding sources and cache the nets.
///
/// # Safety
/// `lib_path` and `main_path` must be valid null-terminated C strings.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vine_init(
    lib_path: *const c_char,
    main_path: *const c_char,
    root_path: *const c_char,
) -> *mut VineState {
    let lib_path_str = unsafe { CStr::from_ptr(lib_path) }.to_str().unwrap();
    let main_path_str = unsafe { CStr::from_ptr(main_path) }.to_str().unwrap();

    // Determine Vine root stdlib path
    let root = if root_path.is_null() {
        // Try VINE_ROOT_PATH env, then fall back to vine's git checkout
        match std::env::var("VINE_ROOT_PATH") {
            Ok(p) => PathBuf::from(p),
            Err(_) => match find_vine_root() {
                Some(p) => p,
                None => {
                    eprintln!("vine_init: cannot find Vine root stdlib. Set VINE_ROOT_PATH.");
                    return std::ptr::null_mut();
                }
            },
        }
    } else {
        PathBuf::from(unsafe { CStr::from_ptr(root_path) }.to_str().unwrap())
    };

    let mut compiler = Compiler::new(false, Default::default());

    {
        let mut loader = Loader::new(&mut compiler, RealFS, None);
        loader.load_mod(Ident::new("root").unwrap(), root);
        loader.load_mod(Ident::new("pathfind").unwrap(), PathBuf::from(lib_path_str));
        loader.load_main_mod(Ident::new("main").unwrap(), PathBuf::from(main_path_str));
    }

    let mut nets = match compiler.compile(()) {
        Ok(nets) => nets,
        Err(_) => {
            eprintln!("vine_init: compilation failed");
            for e in &compiler.diags.errors {
                eprintln!("  error: {e}");
            }
            for w in &compiler.diags.warnings {
                eprintln!("  warning: {w}");
            }
            return std::ptr::null_mut();
        }
    };

    Optimizer::default().optimize(&mut nets);

    Box::into_raw(Box::new(VineState { nets }))
}

/// Free VineState.
///
/// # Safety
/// `state` must be a valid pointer returned by `vine_init`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vine_destroy(state: *mut VineState) {
    if !state.is_null() {
        drop(unsafe { Box::from_raw(state) });
    }
}

/// Run a pathfinding query using text protocol. Returns 0 on success, negative on error.
///
/// # Safety
/// All pointers must be valid. `out_buf` must have capacity `out_len`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vine_route(
    state: *const VineState,
    input: *const c_char,
    workers: c_uint,
    out_buf: *mut u8,
    out_len: c_uint,
    out_written: *mut c_uint,
) -> c_int {
    let state = unsafe { &*state };
    let input_str = match unsafe { CStr::from_ptr(input) }.to_str() {
        Ok(s) => s,
        Err(_) => return -1,
    };

    match run_vine_text(&state.nets, input_str, workers as usize) {
        Ok(output) => write_output(output.as_bytes(), out_buf, out_len, out_written),
        Err(e) => {
            eprintln!("vine_route error: {e}");
            -2
        }
    }
}

/// Run a pathfinding query with CSR graph data (no text parsing).
/// Returns 0 on success, negative on error.
///
/// # Safety
/// All pointers must be valid. CSR arrays must be consistent.
/// `row_ptr` has `node_count + 1` elements.
/// `col_idx` and `weights` have `row_ptr[node_count]` elements.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vine_route_graph(
    state: *const VineState,
    node_count: c_uint,
    row_ptr: *const c_uint,
    col_idx: *const c_uint,
    weights: *const c_uint,
    source: c_uint,
    target: c_uint,
    workers: c_uint,
    out_buf: *mut u8,
    out_len: c_uint,
    out_written: *mut c_uint,
) -> c_int {
    let state = unsafe { &*state };

    let graph = Arc::new(GraphCSR {
        node_count,
        row_ptr,
        col_idx,
        weights,
        source,
        target,
    });

    match run_vine_graph(&state.nets, graph, workers as usize) {
        Ok(output) => write_output(output.as_bytes(), out_buf, out_len, out_written),
        Err(e) => {
            eprintln!("vine_route_graph error: {e}");
            -2
        }
    }
}

/// Copy output bytes to C buffer, null-terminate.
fn write_output(bytes: &[u8], out_buf: *mut u8, out_len: c_uint, out_written: *mut c_uint) -> c_int {
    if out_len == 0 {
        unsafe { *out_written = 0 };
        return 0;
    }
    let copy_len = bytes.len().min(out_len as usize - 1);
    unsafe {
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), out_buf, copy_len);
        *out_buf.add(copy_len) = 0;
        *out_written = copy_len as c_uint;
    }
    0
}

/// Run compiled Vine with text-based buffer IO (original path).
fn run_vine_text(nets: &Nets, input: &str, workers: usize) -> Result<String, String> {
    let heap = Heap::new();
    let mut host = &mut Host::default();
    let mut extrinsics = Extrinsics::default();

    host.register_default_extrinsics(&mut extrinsics);

    let input_buf = Arc::new(Mutex::new(Cursor::new(input.as_bytes().to_vec())));
    let output_buf: Arc<Mutex<Vec<u8>>> = Arc::new(Mutex::new(Vec::new()));
    register_buffer_runtime(&mut *host, &mut extrinsics, input_buf, output_buf.clone());

    host.insert_nets(nets);

    let main = host.get("::").ok_or("missing main function")?;
    let mut ivm = IVM::new(&heap, &extrinsics);

    let node = unsafe {
        ivm.new_node(
            Tag::Comb,
            Host::label_to_u16("x", &mut host.comb_labels),
        )
    };
    ivm.link_wire(node.1, Port::new_ext_val(host.new_io()));
    ivm.link(Port::new_global(main), node.0);

    if workers > 1 {
        ivm.normalize_parallel(workers);
    } else {
        ivm.normalize();
    }

    let out = ivm.follow(Port::new_wire(node.2));
    let no_io =
        out.tag() != Tag::ExtVal || unsafe { out.as_ext_val() }.bits() != host.new_io().bits();

    if no_io || !ivm.flags.success() {
        return Err("IVM execution failed".into());
    }

    let output = output_buf.lock().unwrap();
    String::from_utf8(output.clone()).map_err(|e| e.to_string())
}

/// Run compiled Vine with graph FFI extrinsics (no text parsing).
fn run_vine_graph(nets: &Nets, graph: Arc<GraphCSR>, workers: usize) -> Result<String, String> {
    let heap = Heap::new();
    let mut host = &mut Host::default();
    let mut extrinsics = Extrinsics::default();

    host.register_default_extrinsics(&mut extrinsics);

    // Output buffer for results (COST/PATH lines — small)
    let output_buf: Arc<Mutex<Vec<u8>>> = Arc::new(Mutex::new(Vec::new()));
    // Empty input — FFI mode doesn't read stdin
    let input_buf = Arc::new(Mutex::new(Cursor::new(Vec::new())));
    register_buffer_runtime(&mut *host, &mut extrinsics, input_buf, output_buf.clone());

    // Register graph FFI extrinsics
    register_graph_extrinsics(&mut *host, &mut extrinsics, graph);

    host.insert_nets(nets);

    let main = host.get("::").ok_or("missing main function")?;
    let mut ivm = IVM::new(&heap, &extrinsics);

    let node = unsafe {
        ivm.new_node(
            Tag::Comb,
            Host::label_to_u16("x", &mut host.comb_labels),
        )
    };
    ivm.link_wire(node.1, Port::new_ext_val(host.new_io()));
    ivm.link(Port::new_global(main), node.0);

    if workers > 1 {
        ivm.normalize_parallel(workers);
    } else {
        ivm.normalize();
    }

    let out = ivm.follow(Port::new_wire(node.2));
    let no_io =
        out.tag() != Tag::ExtVal || unsafe { out.as_ext_val() }.bits() != host.new_io().bits();

    if no_io || !ivm.flags.success() {
        return Err("IVM execution failed".into());
    }

    let output = output_buf.lock().unwrap();
    String::from_utf8(output.clone()).map_err(|e| e.to_string())
}

/// Register graph data extrinsics backed by CSR arrays.
fn register_graph_extrinsics<'ivm>(
    host: &mut Host<'ivm>,
    extrinsics: &mut Extrinsics<'ivm>,
    graph: Arc<GraphCSR>,
) {
    let n32 = extrinsics.n32_ext_ty();

    // graph_nodes: split — input: dummy → out0: node_count, out1: erased
    {
        let g = graph.clone();
        host.register_ext_fn(
            "graph_nodes",
            extrinsics.new_split_ext_fn(move |ivm, _input, out0, out1| {
                ivm.link_wire(out0, Port::new_ext_val(n32.wrap_ext_val(g.node_count)));
                ivm.link_wire(out1, Port::ERASE);
            }),
        );
    }

    // graph_source: split — input: dummy → out0: source, out1: erased
    {
        let g = graph.clone();
        host.register_ext_fn(
            "graph_source",
            extrinsics.new_split_ext_fn(move |ivm, _input, out0, out1| {
                ivm.link_wire(out0, Port::new_ext_val(n32.wrap_ext_val(g.source)));
                ivm.link_wire(out1, Port::ERASE);
            }),
        );
    }

    // graph_target: split — input: dummy → out0: target, out1: erased
    {
        let g = graph.clone();
        host.register_ext_fn(
            "graph_target",
            extrinsics.new_split_ext_fn(move |ivm, _input, out0, out1| {
                ivm.link_wire(out0, Port::new_ext_val(n32.wrap_ext_val(g.target)));
                ivm.link_wire(out1, Port::ERASE);
            }),
        );
    }

    // graph_degree: split — input: node → out0: degree, out1: erased
    {
        let g = graph.clone();
        host.register_ext_fn(
            "graph_degree",
            extrinsics.new_split_ext_fn(move |ivm, node_val, out0, out1| {
                let Some(node) = n32.unwrap_ext_val(node_val) else {
                    eprintln!("[FFI] graph_degree: ext_generic (non-N32 input)");
                    ivm.flags.ext_generic = true;
                    ivm.link_wire(out0, Port::ERASE);
                    ivm.link_wire(out1, Port::ERASE);
                    return;
                };
                let deg = g.degree(node);
                ivm.link_wire(out0, Port::new_ext_val(n32.wrap_ext_val(deg)));
                ivm.link_wire(out1, Port::ERASE);
            }),
        );
    }

    // graph_adj_target: merge — input: (node, idx) → output: target
    {
        let g = graph.clone();
        host.register_ext_fn(
            "graph_adj_target",
            extrinsics.new_merge_ext_fn(move |ivm, node_val, idx_val, out| {
                let Some(node) = n32.unwrap_ext_val(node_val) else {
                    ivm.flags.ext_generic = true;
                    ivm.link_wire(out, Port::ERASE);
                    return;
                };
                let Some(idx) = n32.unwrap_ext_val(idx_val) else {
                    ivm.flags.ext_generic = true;
                    ivm.link_wire(out, Port::ERASE);
                    return;
                };
                let target = g.adj_target(node, idx);
                ivm.link_wire(out, Port::new_ext_val(n32.wrap_ext_val(target)));
            }),
        );
    }

    // graph_adj_weight: merge — input: (node, idx) → output: weight
    {
        let g = graph;
        host.register_ext_fn(
            "graph_adj_weight",
            extrinsics.new_merge_ext_fn(move |ivm, node_val, idx_val, out| {
                let Some(node) = n32.unwrap_ext_val(node_val) else {
                    ivm.flags.ext_generic = true;
                    ivm.link_wire(out, Port::ERASE);
                    return;
                };
                let Some(idx) = n32.unwrap_ext_val(idx_val) else {
                    ivm.flags.ext_generic = true;
                    ivm.link_wire(out, Port::ERASE);
                    return;
                };
                let weight = g.adj_weight(node, idx);
                ivm.link_wire(out, Port::new_ext_val(n32.wrap_ext_val(weight)));
            }),
        );
    }
}

/// Register all runtime extrinsics (list_*, io_*) with buffer-backed IO.
fn register_buffer_runtime<'ivm>(
    host: &mut Host<'ivm>,
    extrinsics: &mut Extrinsics<'ivm>,
    input_buf: Arc<Mutex<Cursor<Vec<u8>>>>,
    output_buf: Arc<Mutex<Vec<u8>>>,
) {
    let n32 = extrinsics.n32_ext_ty();

    // Register ext types manually (mirrors the private Host::register_ext_ty)
    let list = {
        let ty = extrinsics.new_ext_ty::<ExtList<'ivm>>();
        host.register_ext_ty_id("List".into(), ty.ty_id());
        ty
    };
    let io = {
        let ty = extrinsics.new_ext_ty::<()>();
        host.register_ext_ty_id("IO".into(), ty.ty_id());
        ty
    };

    // --- List extrinsics (same as upstream) ---

    host.register_ext_fn(
        "list_push",
        extrinsics.new_merge_ext_fn(move |ivm, l, el, out| {
            let Some(mut l) = list.unwrap_ext_val(l) else {
                ivm.flags.ext_generic = true;
                return;
            };
            l.push(el);
            ivm.link_wire(out, Port::new_ext_val(list.wrap_ext_val(l)));
        }),
    );

    host.register_ext_fn(
        "list_pop",
        extrinsics.new_split_ext_fn(move |ivm, l, out0, out1| {
            let Some(mut l) = list.unwrap_ext_val(l) else {
                ivm.flags.ext_generic = true;
                return;
            };
            let Some(el) = l.pop() else {
                ivm.flags.ext_generic = true;
                return;
            };
            ivm.link_wire(out0, Port::new_ext_val(el));
            ivm.link_wire(out1, Port::new_ext_val(list.wrap_ext_val(l)));
        }),
    );

    host.register_ext_fn(
        "list_drop",
        extrinsics.new_split_ext_fn(move |ivm, l, out0, out1| {
            match list.unwrap_ext_val(l) {
                Some(l) if !l.is_empty() => ivm.flags.ext_erase = true,
                None => ivm.flags.ext_generic = true,
                _ => {}
            }
            ivm.link_wire(out0, Port::ERASE);
            ivm.link_wire(out1, Port::ERASE);
        }),
    );

    host.register_ext_fn(
        "list_new",
        extrinsics.new_split_ext_fn(move |ivm, _unused, out0, out1| {
            ivm.link_wire(out0, Port::ERASE);
            ivm.link_wire(out1, Port::new_ext_val(list.wrap_ext_val(ExtList::default())));
        }),
    );

    host.register_ext_fn(
        "list_len",
        extrinsics.new_split_ext_fn(move |ivm, l, out0, out1| {
            let Some(l) = list.unwrap_ext_val(l) else {
                ivm.flags.ext_generic = true;
                ivm.link_wire(out0, Port::ERASE);
                ivm.link_wire(out1, Port::ERASE);
                return;
            };
            let len = l.len() as u32;
            ivm.link_wire(out0, Port::new_ext_val(n32.wrap_ext_val(len)));
            ivm.link_wire(out1, Port::new_ext_val(list.wrap_ext_val(l)));
        }),
    );

    // --- IO extrinsics (buffer-backed) ---

    host.register_ext_fn(
        "io_join",
        extrinsics.new_merge_ext_fn(move |ivm, _a, _b, out| {
            ivm.link_wire(out, Port::new_ext_val(io.wrap_ext_val(())));
        }),
    );

    host.register_ext_fn(
        "io_split",
        extrinsics.new_split_ext_fn(move |ivm, _io_val, out0, out1| {
            ivm.link_wire(out0, Port::new_ext_val(io.wrap_ext_val(())));
            ivm.link_wire(out1, Port::new_ext_val(io.wrap_ext_val(())));
        }),
    );

    host.register_ext_fn(
        "io_ready",
        extrinsics.new_split_ext_fn(move |ivm, _io_val, out0, out1| {
            ivm.link_wire(out0, Port::new_ext_val(n32.wrap_ext_val(1)));
            ivm.link_wire(out1, Port::new_ext_val(io.wrap_ext_val(())));
        }),
    );

    host.register_ext_fn(
        "io_args",
        extrinsics.new_split_ext_fn(move |ivm, _io_val, out0, out1| {
            ivm.link_wire(out0, Port::new_ext_val(list.wrap_ext_val(ExtList::default())));
            ivm.link_wire(out1, Port::new_ext_val(io.wrap_ext_val(())));
        }),
    );

    // io_print_char: write UTF-8 char to output buffer
    {
        let out = output_buf.clone();
        host.register_ext_fn(
            "io_print_char",
            extrinsics.new_merge_ext_fn(move |ivm, _io_val, b, wire| {
                let Some(b) = n32.unwrap_ext_val(b) else {
                    ivm.flags.ext_generic = true;
                    ivm.link_wire(wire, Port::ERASE);
                    return;
                };
                if let Some(c) = char::try_from(b).ok() {
                    let mut buf = [0u8; 4];
                    let s = c.encode_utf8(&mut buf);
                    out.lock().unwrap().extend_from_slice(s.as_bytes());
                }
                ivm.link_wire(wire, Port::new_ext_val(io.wrap_ext_val(())));
            }),
        );
    }

    // io_print_byte: write raw byte to output buffer
    {
        let out = output_buf.clone();
        host.register_ext_fn(
            "io_print_byte",
            extrinsics.new_merge_ext_fn(move |ivm, _io_val, b, wire| {
                let Some(b) = n32.unwrap_ext_val(b) else {
                    ivm.flags.ext_generic = true;
                    ivm.link_wire(wire, Port::ERASE);
                    return;
                };
                out.lock().unwrap().push(b as u8);
                ivm.link_wire(wire, Port::new_ext_val(io.wrap_ext_val(())));
            }),
        );
    }

    // io_flush: no-op for buffer IO
    host.register_ext_fn(
        "io_flush",
        extrinsics.new_split_ext_fn(move |ivm, _io_val, out0, out1| {
            ivm.link_wire(out0, Port::ERASE);
            ivm.link_wire(out1, Port::new_ext_val(io.wrap_ext_val(())));
        }),
    );

    // io_read_byte: read from input buffer
    {
        let inp = input_buf.clone();
        host.register_ext_fn(
            "io_read_byte",
            extrinsics.new_split_ext_fn(move |ivm, _io_val, out0, out1| {
                let mut buf = [0u8; 1];
                let byte = {
                    let mut cursor = inp.lock().unwrap();
                    match cursor.read(&mut buf) {
                        Ok(0) | Err(_) => u32::MAX,
                        Ok(_) => buf[0] as u32,
                    }
                };
                ivm.link_wire(out0, Port::new_ext_val(n32.wrap_ext_val(byte)));
                ivm.link_wire(out1, Port::new_ext_val(io.wrap_ext_val(())));
            }),
        );
    }

    // io_read_char: read UTF-8 char from input buffer
    {
        let inp = input_buf;
        host.register_ext_fn(
            "io_read_char",
            extrinsics.new_split_ext_fn(move |ivm, _io_val, out0, out1| {
                let c = {
                    let mut cursor = inp.lock().unwrap();
                    read_utf8_char(&mut *cursor)
                };
                ivm.link_wire(out0, Port::new_ext_val(n32.wrap_ext_val(c)));
                ivm.link_wire(out1, Port::new_ext_val(io.wrap_ext_val(())));
            }),
        );
    }
}

/// Find the Vine root stdlib in cargo's git checkouts.
fn find_vine_root() -> Option<PathBuf> {
    let home = std::env::var("HOME").ok()?;
    let cargo_git = PathBuf::from(&home).join(".cargo/git/checkouts");
    if !cargo_git.is_dir() {
        return None;
    }
    for entry in std::fs::read_dir(&cargo_git).into_iter().flatten().flatten() {
        if entry.file_name().to_string_lossy().starts_with("vine-") {
            if let Ok(sub_entries) = std::fs::read_dir(entry.path()) {
                for sub in sub_entries.flatten() {
                    let root_path = sub.path().join("root");
                    if root_path.is_dir() {
                        return Some(root_path);
                    }
                }
            }
        }
    }
    None
}

/// Read one UTF-8 character from a reader, returning its u32 codepoint.
/// Returns u32::MAX on EOF.
fn read_utf8_char(reader: &mut impl Read) -> u32 {
    let mut first = [0u8; 1];
    match reader.read(&mut first) {
        Ok(0) | Err(_) => return u32::MAX,
        Ok(_) => {}
    }

    let b0 = first[0];
    let (expected_len, initial_bits) = match b0.leading_ones() {
        0 => return b0 as u32,
        2 => (2, (b0 & 0x1F) as u32),
        3 => (3, (b0 & 0x0F) as u32),
        4 => (4, (b0 & 0x07) as u32),
        _ => return char::REPLACEMENT_CHARACTER as u32,
    };

    let mut codepoint = initial_bits;
    for _ in 1..expected_len {
        let mut cont = [0u8; 1];
        match reader.read(&mut cont) {
            Ok(1) if cont[0] & 0xC0 == 0x80 => {
                codepoint = (codepoint << 6) | (cont[0] & 0x3F) as u32;
            }
            _ => return char::REPLACEMENT_CHARACTER as u32,
        }
    }

    codepoint
}
