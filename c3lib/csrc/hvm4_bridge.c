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

#include "../../HVM4/clang/hvm4.c"

// ---------------------------------------------------------------------------
// hvm4_lib_init: one-time runtime initialization
// ---------------------------------------------------------------------------
void hvm4_lib_init(void) {
  thread_set_count(1);
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
    // Collapse mode: capture stdout to extract printed numbers
    char *buf = NULL;
    size_t buf_len = 0;
    FILE *memf = open_memstream(&buf, &buf_len);
    if (!memf) return -1;

    FILE *old_stdout = stdout;
    stdout = memf;

    eval_collapse(main_ref, collapse_limit, 0, 0);

    fflush(memf);
    stdout = old_stdout;
    fclose(memf);

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
