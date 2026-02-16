// HVM4 Bridge for C3 FFI
// ======================
//
// This file wraps the HVM4 runtime (which uses `#define fn static inline` for
// all functions) and exports four non-static entry points callable from C3:
//
//   hvm4_lib_init()    - allocate BOOK/HEAP/TABLE, init primitives
//   hvm4_lib_cleanup() - free all runtime memory
//   hvm4_lib_reset()   - reset state between evaluations
//   hvm4_run()         - parse source, evaluate @main, extract numeric results

#include <sys/mman.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>

// HEAP_CAP is capped at 1<<32 (32 GB) — HVM4 uses u32 for heap locations.
// This is the hard architectural limit until HVM4 switches to u64 indices.

#include "../../HVM4/clang/hvm4.c"

// ---------------------------------------------------------------------------
// hvm4_lib_init: one-time runtime initialization
// ---------------------------------------------------------------------------
void hvm4_lib_init(void) {
  // Thread count: HVM4_THREADS env var, or all available cores.
  // Free-list is disabled in multi-threaded mode (cross-thread races),
  // so memory recycling relies on @compact primitive instead.
  u32 threads = 0;
  const char *env = getenv("HVM4_THREADS");
  if (env && env[0]) {
    threads = (u32)atoi(env);
  }
  if (threads == 0) {
    long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    threads = ncpu > 0 ? (u32)ncpu : 1;
  }
  thread_set_count(threads); // clamps to [1, MAX_THREADS]
  wnf_set_tid(0);
  BOOK  = calloc(BOOK_CAP, sizeof(u32));
  HEAP  = calloc(HEAP_CAP, sizeof(Term));
  TABLE = calloc(BOOK_CAP, sizeof(char*));
  if (!BOOK || !HEAP || !TABLE) {
    fprintf(stderr, "hvm4_lib_init: allocation failed\n");
    exit(1);
  }
  heap_init_slices();
  prim_init();
  DEBUG        = 0;
  SILENT       = 0;
  STEPS_ENABLE = 0;
}

// ---------------------------------------------------------------------------
// hvm4_lib_cleanup: free all runtime memory (call once at shutdown)
// ---------------------------------------------------------------------------
void hvm4_lib_cleanup(void) {
  wnf_stack_free();
  free(HEAP);
  free(BOOK);
  // Free TABLE string entries
  for (u32 i = 0; i < TABLE_LEN; i++) {
    free(TABLE[i]);
  }
  free(TABLE);
  HEAP  = NULL;
  BOOK  = NULL;
  TABLE = NULL;
}

// ---------------------------------------------------------------------------
// hvm4_lib_reset: reset state between evaluations so a new program can run
// ---------------------------------------------------------------------------
void hvm4_lib_reset(void) {
  // Free TABLE string entries
  for (u32 i = 0; i < TABLE_LEN; i++) {
    free(TABLE[i]);
  }
  TABLE_LEN = 0;

  // Reset BOOK (clear definitions)
  memset(BOOK, 0, BOOK_CAP * sizeof(u32));

  // Reset heap: use madvise to release physical pages without unmapping
  madvise(HEAP, HEAP_CAP * sizeof(Term), MADV_DONTNEED);

  // Reset free lists (stale entries would point to zeroed pages)
  heap_free_reset();

  // Re-initialize heap slices
  heap_init_slices();

  // Free PARSE_SEEN_FILES entries (they are strdup'd)
  for (u32 i = 0; i < PARSE_SEEN_FILES_LEN; i++) {
    free(PARSE_SEEN_FILES[i]);
  }

  // Reset parser globals
  PARSE_BINDS_LEN     = 0;
  PARSE_FRESH_LAB     = 0x800000;
  PARSE_SEEN_FILES_LEN = 0;
  PARSE_FORK_SIDE     = -1;
  FRESH               = 1;

  // Reset WNF state
  for (u32 t = 0; t < MAX_THREADS; t++) {
    WNF_ITRS_BANKS[t].itrs = 0;
    if (WNF_BANKS[t].stack) {
      WNF_BANKS[t].s_pos = 1;
    }
  }
  wnf_set_tid(0);

  // Clear primitive definitions and re-register (table was cleared)
  memset(PRIM_DEFS, 0, sizeof(PRIM_DEFS));
  prim_init();
}

// ---------------------------------------------------------------------------
// Graph FFI: CSR graph in C memory (outside HVM4 heap)
// ---------------------------------------------------------------------------

static uint32_t  g_graph_v;
static uint32_t *g_csr_row_ptr;  // size V+1
static uint32_t *g_csr_col_idx;  // size E
static uint32_t *g_csr_weight;   // size E

// %graph_deg(u) → NUM: outgoing degree of node u
static Term prim_graph_deg(Term *args) {
  Term u = wnf(args[0]);
  uint32_t node = term_val(u);
  if (node >= g_graph_v) return term_new_num(0);
  uint32_t deg = g_csr_row_ptr[node + 1] - g_csr_row_ptr[node];
  return term_new_num(deg);
}

// %graph_target(u, i) → NUM: i-th neighbor of node u
static Term prim_graph_target(Term *args) {
  Term u = wnf(args[0]);
  Term i = wnf(args[1]);
  uint32_t edge = g_csr_row_ptr[term_val(u)] + term_val(i);
  return term_new_num(g_csr_col_idx[edge]);
}

// %graph_weight(u, i) → NUM: weight of i-th edge from node u
static Term prim_graph_weight(Term *args) {
  Term u = wnf(args[0]);
  Term i = wnf(args[1]);
  uint32_t edge = g_csr_row_ptr[term_val(u)] + term_val(i);
  return term_new_num(g_csr_weight[edge]);
}

// Called AFTER hvm4_lib_reset(), BEFORE hvm4_run()
void hvm4_graph_setup(uint32_t *row_ptr, uint32_t *col_idx,
                      uint32_t *weight, uint32_t v) {
  g_graph_v     = v;
  g_csr_row_ptr = row_ptr;
  g_csr_col_idx = col_idx;
  g_csr_weight  = weight;
  prim_register("graph_deg",    9,  1, prim_graph_deg);
  prim_register("graph_target", 12, 2, prim_graph_target);
  prim_register("graph_weight", 12, 2, prim_graph_weight);
}

// ---------------------------------------------------------------------------
// Weather FFI: edge cost computation in C (trig requires doubles)
// ---------------------------------------------------------------------------

static uint32_t  g_weather_edge_count;
static double   *g_edge_dist_m;          // [E] haversine distance per edge
static double   *g_edge_heading;         // [E] bearing in degrees
static double   *g_weather_wind_speed;   // [E] sampled at edge midpoint
static double   *g_weather_wind_dir;     // [E]
static double   *g_weather_wave_height;  // [E]
static double   *g_weather_current_speed;// [E]
static double   *g_weather_current_dir;  // [E]
static double    g_ship_base_speed;
static double    g_ship_max_wave_ht;
static double    g_ship_fuel_base;
static double    g_ship_fuel_per_kt;
static uint32_t  g_cost_mode;            // 0=time, 1=fuel, 2=weighted, 3=safety
static double    g_cost_alpha;
static double    g_cost_safety_penalty;
static uint32_t  g_cost_scale;

static const double NM_TO_M = 1852.0;
static const double DEG_TO_RAD_C = 3.14159265358979323846 / 180.0;

// %edge_cost(edge_id) → NUM: compute weather-adjusted cost for one edge
static Term prim_edge_cost(Term *args) {
  Term id_term = wnf(args[0]);
  uint32_t i = term_val(id_term);
  if (i >= g_weather_edge_count) return term_new_num(1);

  double dist_m     = g_edge_dist_m[i];
  double heading    = g_edge_heading[i];
  double wind_speed = g_weather_wind_speed[i];
  double wind_dir   = g_weather_wind_dir[i];
  double wave_ht    = g_weather_wave_height[i];
  double cur_speed  = g_weather_current_speed[i];
  double cur_dir    = g_weather_current_dir[i];

  // Wave reduction factor
  double wave_ratio = wave_ht / g_ship_max_wave_ht;
  if (wave_ratio > 1.0) wave_ratio = 1.0;
  double reduction = 1.0 - 0.5 * wave_ratio * wave_ratio;
  if (reduction < 0.1) reduction = 0.1;
  if (reduction > 1.0) reduction = 1.0;

  // Current along heading
  double current_angle = (cur_dir - heading) * DEG_TO_RAD_C;
  double current_along = cur_speed * cos(current_angle) * 1.94384;

  // Effective speed
  double eff_speed = g_ship_base_speed * reduction + current_along;
  if (eff_speed < 0.1) eff_speed = 0.1;

  // Time and fuel
  double dist_nm = dist_m / NM_TO_M;
  double time_h  = dist_nm / eff_speed;
  double fuel    = time_h * (g_ship_fuel_base + g_ship_fuel_per_kt * eff_speed);

  // Cost based on mode
  double cost;
  if (g_cost_mode == 0) {          // TIME_ONLY
    cost = time_h;
  } else if (g_cost_mode == 1) {   // FUEL_ONLY
    cost = fuel;
  } else if (g_cost_mode == 2) {   // WEIGHTED_SUM
    cost = g_cost_alpha * time_h + (1.0 - g_cost_alpha) * fuel;
  } else {                         // SAFETY_PENALIZED
    cost = g_cost_alpha * time_h + (1.0 - g_cost_alpha) * fuel;
    if (wave_ht > g_ship_max_wave_ht) {
      cost += g_cost_safety_penalty;
    }
  }

  // Scale to integer and clamp
  uint32_t weight = (uint32_t)(cost * (double)g_cost_scale);
  if (weight < 1)      weight = 1;
  if (weight > 900000) weight = 900000;
  return term_new_num(weight);
}

// Called AFTER hvm4_lib_reset(), BEFORE hvm4_run()
void hvm4_weather_setup(
    double *dist_m, double *heading,
    double *wind_speed, double *wind_dir, double *wave_height,
    double *current_speed, double *current_dir,
    double base_speed, double max_wave_ht, double fuel_base, double fuel_per_kt,
    uint32_t cost_mode, double alpha, double safety_penalty, uint32_t scale,
    uint32_t edge_count)
{
  g_edge_dist_m            = dist_m;
  g_edge_heading           = heading;
  g_weather_wind_speed     = wind_speed;
  g_weather_wind_dir       = wind_dir;
  g_weather_wave_height    = wave_height;
  g_weather_current_speed  = current_speed;
  g_weather_current_dir    = current_dir;
  g_ship_base_speed        = base_speed;
  g_ship_max_wave_ht       = max_wave_ht;
  g_ship_fuel_base         = fuel_base;
  g_ship_fuel_per_kt       = fuel_per_kt;
  g_cost_mode              = cost_mode;
  g_cost_alpha             = alpha;
  g_cost_safety_penalty    = safety_penalty;
  g_cost_scale             = scale;
  g_weather_edge_count     = edge_count;
  prim_register("edge_cost", 9, 1, prim_edge_cost);
}

// ---------------------------------------------------------------------------
// extract_nums: recursively extract NUM values from a result term
// ---------------------------------------------------------------------------
//
// - NUM  (tag 30)         -> extract term_val() as a single value
// - C02  (tag 15)         -> cons cell, recurse head and tail
// - C00  (tag 13)         -> empty list, stop
// - other Cxx             -> recurse into all children
//
// Returns the next write position (i.e. count of values written so far).
static int extract_nums(Term term, uint32_t *out, int pos, int max_out) {
  u8 tag = term_tag(term);

  if (tag == NUM) {
    if (pos < max_out) {
      out[pos] = term_val(term);
    }
    return pos + 1;
  }

  if (tag == C00) {
    // Empty list / nullary constructor - nothing to extract
    return pos;
  }

  if (tag == C02) {
    // Cons cell: head at HEAP[val+0], tail at HEAP[val+1]
    u32 loc  = term_val(term);
    Term head = HEAP[loc];
    Term tail = HEAP[loc + 1];
    pos = extract_nums(head, out, pos, max_out);
    return extract_nums(tail, out, pos, max_out);
  }

  // For any other constructor with children, try to extract from them
  if (tag >= C01 && tag <= C16) {
    u32 ari = tag - C00;
    u32 loc = term_val(term);
    for (u32 i = 0; i < ari; i++) {
      pos = extract_nums(HEAP[loc + i], out, pos, max_out);
    }
    return pos;
  }

  return pos;
}

// ---------------------------------------------------------------------------
// hvm4_run: parse source, evaluate @main, extract numeric results
// ---------------------------------------------------------------------------
//
// Parameters:
//   source         - HVM4 source code (null-terminated)
//   collapse_limit - if >0, use eval_collapse; otherwise eval_normalize
//   out            - output buffer for extracted uint32 values
//   max_out        - capacity of the output buffer
//
// Returns:
//   >= 0  number of values written to `out`
//   -1    on error (allocation failure or @main not defined)
int hvm4_run(const char *source, int collapse_limit, uint32_t *out, int max_out) {
  // Copy source (parser needs a mutable buffer)
  size_t src_len = strlen(source);
  char *src = malloc(src_len + 1);
  if (!src) return -1;
  memcpy(src, source, src_len + 1);

  // Parse
  PState s = {
    .file = "hvm4_bridge",
    .src  = src,
    .pos  = 0,
    .len  = (u32)src_len,
    .line = 1,
    .col  = 1
  };
  parse_def(&s);
  free(src);

  // Find @main
  u32 main_id = table_find("main", 4);
  if (BOOK[main_id] == 0) {
    return -1;
  }

  Term main_ref = term_new_ref(main_id);

  if (collapse_limit > 0) {
    // Collapse mode: force single-threaded for stdout→memstream safety.
    // Worker threads would write to the redirected stdout concurrently,
    // interleaving partial output lines.
    u32 saved_threads = thread_get_count();
    thread_set_count(1);

    char *buf = NULL;
    size_t buf_len = 0;
    FILE *memf = open_memstream(&buf, &buf_len);
    if (!memf) { thread_set_count(saved_threads); return -1; }

    FILE *old_stdout = stdout;
    stdout = memf;

    eval_collapse(main_ref, collapse_limit, 0, 0);

    fflush(memf);
    stdout = old_stdout;
    fclose(memf);
    thread_set_count(saved_threads);

    // Parse numbers from captured output (one per line)
    int count = 0;
    char *line = buf;
    while (line && *line && count < max_out) {
      // Skip whitespace
      while (*line == ' ' || *line == '\t') line++;
      if (*line == '\0' || *line == '\n') {
        if (*line) line++;
        continue;
      }
      // Try to parse a number
      char *end;
      unsigned long val = strtoul(line, &end, 10);
      if (end != line) {
        out[count++] = (uint32_t)val;
      }
      // Advance to next line
      line = strchr(line, '\n');
      if (line) line++;
    }

    free(buf);
    return count;
  } else {
    // Normalize mode: evaluate and extract from term tree
    Term result = eval_normalize(main_ref);
    return extract_nums(result, out, 0, max_out);
  }
}

// ---------------------------------------------------------------------------
// Bridge wrappers: expose HVM4 static-inline functions as linkable symbols
// so C3 modules can call them and pass C3 functions as FFI primitive callbacks.
// ---------------------------------------------------------------------------

Term hvm4_wnf(Term term) {
  return wnf(term);
}

uint32_t hvm4_term_val(Term term) {
  return term_val(term);
}

Term hvm4_term_new_num(uint32_t n) {
  return term_new_num(n);
}

uint32_t hvm4_prim_register_ext(const char *name, uint32_t len,
                                uint32_t arity, Term (*handler)(Term *args)) {
  return prim_register(name, len, arity, handler);
}

uint64_t hvm4_itrs_total(void) {
  return wnf_itrs_total();
}
