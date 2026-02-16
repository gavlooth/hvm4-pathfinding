module bench_ffi;

// HVM4 Term representation (u64)
alias Term = ulong;

// Function pointer type for FFI primitive handlers
alias PrimFn = fn Term(Term* args);

// ---------------------------------------------------------------------------
// Bridge wrappers for HVM4 static-inline functions
// (implemented in csrc/hvm4_bridge.c)
// ---------------------------------------------------------------------------

extern fn Term hvm4_wnf(Term term) @cname("hvm4_wnf");
extern fn uint hvm4_term_val(Term term) @cname("hvm4_term_val");
extern fn Term hvm4_term_new_num(uint n) @cname("hvm4_term_new_num");
extern fn uint hvm4_prim_register_ext(ZString name, uint len, uint arity, PrimFn func) @cname("hvm4_prim_register_ext");

// ---------------------------------------------------------------------------
// HVM4 runtime lifecycle and graph setup
// (also implemented in csrc/hvm4_bridge.c)
// ---------------------------------------------------------------------------

extern fn void hvm4_lib_init() @cname("hvm4_lib_init");
extern fn void hvm4_lib_cleanup() @cname("hvm4_lib_cleanup");
extern fn void hvm4_lib_reset() @cname("hvm4_lib_reset");
extern fn int hvm4_run(ZString source, int collapse_limit, uint* out, int max_out) @cname("hvm4_run");
extern fn void hvm4_graph_setup(uint* row_ptr, uint* col_idx, uint* weight, uint v) @cname("hvm4_graph_setup");
extern fn ulong hvm4_itrs_total() @cname("hvm4_itrs_total");
