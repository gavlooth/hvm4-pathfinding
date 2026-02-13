/**
 * libhvm4_graph.c - HVM4 Graph Algorithm Library Implementation
 */

#include "libhvm4_graph.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <sys/mman.h>
#include <unistd.h>

// Include HVM4 runtime after all headers to avoid conflicts
#include "../HVM4/clang/hvm4.c"

/* ========================================================================
 * Internal Types & Constants
 * ======================================================================== */

#define INF 999999

struct hvm4_graph {
    uint32_t n_nodes;
    uint32_t n_edges;
    size_t capacity;
    hvm4_edge_t *edges;
};

/* ========================================================================
 * Internal Helpers
 * ======================================================================== */

static uint32_t ceil_log2_u32(uint32_t n) {
    if (n <= 1) return 1;
    uint32_t rounds = 0;
    uint32_t nn = n;
    while (nn > 1) {
        rounds++;
        nn = (nn + 1) / 2;
    }
    return rounds > 0 ? rounds : 1;
}

static uint32_t ceil_log4_u32(uint32_t n) {
    if (n <= 4) return 1;
    uint32_t depth = 1;
    uint32_t cap = 4;
    while (cap < n) {
        depth++;
        cap *= 4;
    }
    return depth;
}

static uint32_t ceil_log16_u32(uint32_t n) {
    if (n <= 16) return 1;
    uint32_t depth = 1;
    uint32_t cap = 16;
    while (cap < n) {
        depth++;
        cap *= 16;
    }
    return depth;
}

/**
 * Dynamic string builder for HVM4 source generation
 */
typedef struct {
    char *data;
    size_t len;
    size_t capacity;
} dstring_t;

static void dstr_init(dstring_t *ds) {
    ds->capacity = 4096;
    ds->data = malloc(ds->capacity);
    ds->len = 0;
    if (ds->data) ds->data[0] = '\0';
}

static void dstr_free(dstring_t *ds) {
    free(ds->data);
    ds->data = NULL;
    ds->len = 0;
    ds->capacity = 0;
}

static void dstr_ensure(dstring_t *ds, size_t needed) {
    if (ds->len + needed + 1 > ds->capacity) {
        while (ds->capacity < ds->len + needed + 1) {
            ds->capacity *= 2;
        }
        ds->data = realloc(ds->data, ds->capacity);
    }
}

static void dstr_append(dstring_t *ds, const char *str) {
    size_t slen = strlen(str);
    dstr_ensure(ds, slen);
    memcpy(ds->data + ds->len, str, slen + 1);
    ds->len += slen;
}

static void dstr_appendf(dstring_t *ds, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    
    // Measure needed size
    va_list ap_copy;
    va_copy(ap_copy, ap);
    int needed = vsnprintf(NULL, 0, fmt, ap_copy);
    va_end(ap_copy);
    
    if (needed < 0) {
        va_end(ap);
        return;
    }
    
    dstr_ensure(ds, (size_t)needed);
    vsnprintf(ds->data + ds->len, (size_t)needed + 1, fmt, ap);
    ds->len += (size_t)needed;
    va_end(ap);
}

/* ========================================================================
 * HVM4 Source Generators
 * ======================================================================== */

/**
 * Generate adjacency list representation (node -> list of neighbors)
 */
static void gen_adjacency_list(dstring_t *ds, hvm4_graph_t *g, int include_weights) {
    dstr_append(ds, "@adj = λ{\n");
    
    for (uint32_t u = 0; u < g->n_nodes; u++) {
        dstr_appendf(ds, "  %u: [", u);
        
        int first = 1;
        for (uint32_t i = 0; i < g->n_edges; i++) {
            if (g->edges[i].src == u) {
                if (!first) dstr_append(ds, ", ");
                if (include_weights) {
                    dstr_appendf(ds, "#E{%u, %u}", 
                                g->edges[i].dst, g->edges[i].weight);
                } else {
                    dstr_appendf(ds, "%u", g->edges[i].dst);
                }
                first = 0;
            }
        }
        
        dstr_append(ds, "];\n");
    }
    
    dstr_append(ds, "  λn. []\n}\n");
}

/**
 * Generate edge list representation
 */
static void gen_edge_list(dstring_t *ds, hvm4_graph_t *g) {
    dstr_append(ds, "@edges = [");
    
    for (uint32_t i = 0; i < g->n_edges; i++) {
        if (i > 0) dstr_append(ds, ", ");
        dstr_appendf(ds, "#Edge{%u, %u, %u}", 
                    g->edges[i].src, g->edges[i].dst, g->edges[i].weight);
    }
    
    dstr_append(ds, "]\n");
}

/**
 * Generate radix-4 trie operations for tree-structured distance arrays
 */
static void gen_trie4_ops(dstring_t *ds) {
    // q4_get: O(log4(V)) lookup, returns @INF for missing keys
    dstr_append(ds, "@q4_get = λ&key. λ&depth. λ{\n");
    dstr_append(ds, "  #QE: @INF;\n");
    dstr_append(ds, "  #QL: λval. val;\n");
    dstr_append(ds, "  #Q: λ&c0. λ&c1. λ&c2. λ&c3.\n");
    dstr_append(ds, "    ! &slot = key % 4;\n");
    dstr_append(ds, "    ! &next = key / 4;\n");
    dstr_append(ds, "    ! &nd = depth - 1;\n");
    dstr_append(ds, "    λ{0: @q4_get(next,nd,c0); 1: @q4_get(next,nd,c1); ");
    dstr_append(ds, "2: @q4_get(next,nd,c2); λn. @q4_get(next,nd,c3)}(slot)\n");
    dstr_append(ds, "}\n\n");
    
    // q4_set: O(log4(V)) insert/update
    dstr_append(ds, "@q4_set = λ&key. λ&val. λ&depth. λ{\n");
    dstr_append(ds, "  #QL: λold. #QL{val};\n");
    dstr_append(ds, "  #QE: λ{0: #QL{val}; λn.\n");
    dstr_append(ds, "    ! &slot = key % 4; ! &next = key / 4; ! &nd = depth - 1;\n");
    dstr_append(ds, "    @q4_set_slot(slot, @q4_set(next, val, nd, #QE{}))}(depth);\n");
    dstr_append(ds, "  #Q: λ&c0. λ&c1. λ&c2. λ&c3.\n");
    dstr_append(ds, "    ! &slot = key % 4; ! &next = key / 4; ! &nd = depth - 1;\n");
    dstr_append(ds, "    λ{0: #Q{@q4_set(next,val,nd,c0),c1,c2,c3};\n");
    dstr_append(ds, "    1: #Q{c0,@q4_set(next,val,nd,c1),c2,c3};\n");
    dstr_append(ds, "    2: #Q{c0,c1,@q4_set(next,val,nd,c2),c3};\n");
    dstr_append(ds, "    λn. #Q{c0,c1,c2,@q4_set(next,val,nd,c3)}}(slot)\n");
    dstr_append(ds, "}\n\n");
    
    // q4_set_slot: create fresh 4-way branch with one child set
    dstr_append(ds, "@q4_set_slot = λ&slot. λ&child. λ{\n");
    dstr_append(ds, "  0: #Q{child, #QE{}, #QE{}, #QE{}};\n");
    dstr_append(ds, "  1: #Q{#QE{}, child, #QE{}, #QE{}};\n");
    dstr_append(ds, "  2: #Q{#QE{}, #QE{}, child, #QE{}};\n");
    dstr_append(ds, "  λn. #Q{#QE{}, #QE{}, #QE{}, child}\n");
    dstr_append(ds, "}(slot)\n\n");
}

/**
 * Extract numeric results from HVM4 term
 */
static int extract_nums(Term term, uint32_t *out, int pos, int max_out) {
    u8 tag = term_tag(term);
    
    if (tag == NUM) {
        if (pos < max_out) {
            out[pos] = term_val(term);
        }
        return pos + 1;
    }
    
    if (tag == C00) {
        return pos; // Empty list
    }
    
    if (tag == C02) {
        // Cons cell
        u32 loc = term_val(term);
        Term head = HEAP[loc];
        Term tail = HEAP[loc + 1];
        pos = extract_nums(head, out, pos, max_out);
        return extract_nums(tail, out, pos, max_out);
    }
    
    // Other constructors with children
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

/**
 * Run HVM4 source and extract results
 */
static int run_hvm4(const char *source, uint32_t *out, int max_out) {
    // Copy source (parser needs mutable buffer)
    size_t src_len = strlen(source);
    char *src = malloc(src_len + 1);
    if (!src) return -1;
    memcpy(src, source, src_len + 1);
    
    // Parse
    PState s = {
        .file = "libhvm4_graph",
        .src = src,
        .pos = 0,
        .len = (u32)src_len,
        .line = 1,
        .col = 1
    };
    parse_def(&s);
    free(src);
    
    // Find @main
    u32 main_id = table_find("main", 4);
    if (BOOK[main_id] == 0) {
        return -1;
    }
    
    Term main_ref = term_new_ref(main_id);
    Term result = eval_normalize(main_ref);
    
    return extract_nums(result, out, 0, max_out);
}

/**
 * Reset HVM4 state between runs
 */
static void reset_hvm4(void) {
    // Free TABLE string entries
    for (u32 i = 0; i < TABLE_LEN; i++) {
        free(TABLE[i]);
    }
    TABLE_LEN = 0;
    
    // Reset BOOK
    memset(BOOK, 0, BOOK_CAP * sizeof(u32));
    
    // Reset heap (madvise to release physical pages)
    madvise(HEAP, HEAP_CAP * sizeof(Term), MADV_DONTNEED);
    
    // Reset free lists
    heap_free_reset();
    heap_init_slices();
    
    // Free PARSE_SEEN_FILES
    for (u32 i = 0; i < PARSE_SEEN_FILES_LEN; i++) {
        free(PARSE_SEEN_FILES[i]);
    }
    
    // Reset parser globals
    PARSE_BINDS_LEN = 0;
    PARSE_FRESH_LAB = 0x800000;
    PARSE_SEEN_FILES_LEN = 0;
    PARSE_FORK_SIDE = -1;
    FRESH = 1;
    
    // Reset WNF state
    for (u32 t = 0; t < MAX_THREADS; t++) {
        WNF_ITRS_BANKS[t].itrs = 0;
        if (WNF_BANKS[t].stack) {
            WNF_BANKS[t].s_pos = 1;
        }
    }
    wnf_set_tid(0);
    
    // Clear primitive definitions and re-register
    memset(PRIM_DEFS, 0, sizeof(PRIM_DEFS));
    prim_init();
}

/* ========================================================================
 * Public API: Initialization
 * ======================================================================== */

hvm4_result_t hvm4_init(void) {
    // Thread count from env or all cores
    u32 threads = 0;
    const char *env = getenv("HVM4_THREADS");
    if (env && env[0]) {
        threads = (u32)atoi(env);
    }
    if (threads == 0) {
        long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
        threads = ncpu > 0 ? (u32)ncpu : 1;
    }
    
    thread_set_count(threads);
    wnf_set_tid(0);
    
    BOOK = calloc(BOOK_CAP, sizeof(u32));
    HEAP = calloc(HEAP_CAP, sizeof(Term));
    TABLE = calloc(BOOK_CAP, sizeof(char*));
    
    if (!BOOK || !HEAP || !TABLE) {
        return HVM4_ERR_ALLOC;
    }
    
    heap_init_slices();
    prim_init();
    
    DEBUG = 0;
    SILENT = 0;
    STEPS_ENABLE = 0;
    
    return HVM4_OK;
}

void hvm4_cleanup(void) {
    wnf_stack_free();
    free(HEAP);
    free(BOOK);
    
    for (u32 i = 0; i < TABLE_LEN; i++) {
        free(TABLE[i]);
    }
    free(TABLE);
    
    HEAP = NULL;
    BOOK = NULL;
    TABLE = NULL;
}

/* ========================================================================
 * Public API: Graph Construction
 * ======================================================================== */

hvm4_graph_t* hvm4_graph_new(uint32_t n) {
    if (n == 0) return NULL;
    
    hvm4_graph_t *g = malloc(sizeof(hvm4_graph_t));
    if (!g) return NULL;
    
    g->n_nodes = n;
    g->n_edges = 0;
    g->capacity = 16;
    g->edges = malloc(g->capacity * sizeof(hvm4_edge_t));
    
    if (!g->edges) {
        free(g);
        return NULL;
    }
    
    return g;
}

hvm4_result_t hvm4_graph_add_edge(hvm4_graph_t *g, 
                                   uint32_t src, 
                                   uint32_t dst, 
                                   uint32_t weight) {
    if (!g) return HVM4_ERR_INVALID_PARAM;
    if (src >= g->n_nodes || dst >= g->n_nodes) return HVM4_ERR_INVALID_PARAM;
    
    if (g->n_edges >= g->capacity) {
        g->capacity *= 2;
        hvm4_edge_t *new_edges = realloc(g->edges, g->capacity * sizeof(hvm4_edge_t));
        if (!new_edges) return HVM4_ERR_ALLOC;
        g->edges = new_edges;
    }
    
    g->edges[g->n_edges].src = src;
    g->edges[g->n_edges].dst = dst;
    g->edges[g->n_edges].weight = weight;
    g->n_edges++;
    
    return HVM4_OK;
}

hvm4_result_t hvm4_graph_add_biedge(hvm4_graph_t *g, 
                                     uint32_t a, 
                                     uint32_t b, 
                                     uint32_t weight) {
    hvm4_result_t r1 = hvm4_graph_add_edge(g, a, b, weight);
    if (r1 != HVM4_OK) return r1;
    
    hvm4_result_t r2 = hvm4_graph_add_edge(g, b, a, weight);
    if (r2 != HVM4_OK) return r2;
    
    return HVM4_OK;
}

void hvm4_graph_free(hvm4_graph_t *g) {
    if (!g) return;
    free(g->edges);
    free(g);
}

/* ========================================================================
 * Public API: Algorithms
 * ======================================================================== */

hvm4_result_t hvm4_closure(hvm4_graph_t *g, 
                           uint32_t depth_limit,
                           uint8_t *matrix) {
    if (!g || !matrix) return HVM4_ERR_INVALID_PARAM;
    
    reset_hvm4();
    
    dstring_t ds;
    dstr_init(&ds);
    
    // Generate adjacency list
    gen_adjacency_list(&ds, g, 0);
    
    // Helper: check if any neighbor can reach dst
    dstr_append(&ds, "\n@any_reaches = λ&dst. λ&depth. λ{\n");
    dstr_append(&ds, "  []: 0;\n");
    dstr_append(&ds, "  <>: λ&next. λrest.\n");
    dstr_append(&ds, "    λ{0: @any_reaches(dst, depth, rest); λk. 1}(@can_reach(next, dst, depth))\n");
    dstr_append(&ds, "}\n\n");
    
    // Main reachability check
    dstr_append(&ds, "@can_reach = λ&src. λ&dst. λ&depth.\n");
    dstr_append(&ds, "  λ{0: λ{0: 0; λk. 1}(src == dst); λd.\n");
    dstr_append(&ds, "    λ{0: @any_reaches(dst, depth - 1, @adj(src)); λk. 1}(src == dst)\n");
    dstr_append(&ds, "  }(depth)\n\n");
    
    // Generate matrix as flat list
    dstr_append(&ds, "@main = [");
    for (uint32_t i = 0; i < g->n_nodes; i++) {
        for (uint32_t j = 0; j < g->n_nodes; j++) {
            if (i > 0 || j > 0) dstr_append(&ds, ", ");
            dstr_appendf(&ds, "@can_reach(%u, %u, %u)", i, j, depth_limit);
        }
    }
    dstr_append(&ds, "]\n");
    
    // Run and extract
    size_t total = (size_t)g->n_nodes * g->n_nodes;
    uint32_t *out_buf = malloc(total * sizeof(uint32_t));
    if (!out_buf) {
        dstr_free(&ds);
        return HVM4_ERR_ALLOC;
    }
    
    int count = run_hvm4(ds.data, out_buf, (int)total);
    dstr_free(&ds);
    
    if (count < 0) {
        free(out_buf);
        return HVM4_ERR_HVM4_RUNTIME;
    }
    
    // Convert to uint8_t matrix
    for (size_t i = 0; i < total; i++) {
        matrix[i] = out_buf[i] ? 1 : 0;
    }
    
    free(out_buf);
    return HVM4_OK;
}

hvm4_result_t hvm4_mst_boruvka(hvm4_graph_t *g, 
                               uint32_t rounds,
                               uint32_t *mst_weight) {
    if (!g || !mst_weight) return HVM4_ERR_INVALID_PARAM;
    
    reset_hvm4();
    
    dstring_t ds;
    dstr_init(&ds);
    
    dstr_append(&ds, "@INF = 999\n\n");
    
    // List utilities
    dstr_append(&ds, "@get = λ&i. λ{[]: 0; <>: λ&h. λt. λ{0: h; λk. @get(i - 1, t)}(i)}\n");
    dstr_append(&ds, "@relabel = λ&old. λ&new. λ{[]: []; <>: λ&h. λt. λ{0: h; λk. new}(h == old) <> @relabel(old, new, t)}\n");
    dstr_append(&ds, "@edge3 = λ&f. λ{[]: f(0, 0, @INF); <>: λ&u. λ{[]: f(u, 0, @INF); <>: λ&v. λ{[]: f(u, v, @INF); <>: λ&w. λrest. f(u, v, w)}}}\n");
    dstr_append(&ds, "@xor_eq = λa. λb. λ{0: 0; λk. λ{0: 1; λk. 0}(k - 1)}(a + b)\n\n");
    
    // Find min crossing edge
    dstr_append(&ds, "@min_cross = λ&comp. λ&c. λ{[]: [0, 0, @INF]; <>: λ&edge. λrest.\n");
    dstr_append(&ds, "  ! &best = @min_cross(comp, c, rest);\n");
    dstr_append(&ds, "  @edge3(λ&u. λ&v. λ&w.\n");
    dstr_append(&ds, "    ! &cu = @get(u, comp); ! &cv = @get(v, comp);\n");
    dstr_append(&ds, "    ! &cross = @xor_eq(cu == c, cv == c);\n");
    dstr_append(&ds, "    @edge3(λ&bu. λ&bv. λ&bw.\n");
    dstr_append(&ds, "      @pick(cross, w, bw, [u, v, w], [bu, bv, bw]), best), edge)}\n\n");
    
    dstr_append(&ds, "@pick = λ&cross. λ&w. λ&bw. λ&edge. λ&best. λ{0: best; λk. λ{0: best; λk. edge}(w < bw)}(cross)\n\n");
    
    // All mins
    dstr_append(&ds, "@all_mins = λ&comp. λ&edges. λ&n. λ&c. λ{0: []; λk. @min_cross(comp, c, edges) <> @all_mins(comp, edges, n - 1, c + 1)}(n)\n\n");
    
    // Merge
    dstr_append(&ds, "@merge = λ&comp. λ&total. λ{[]: [comp, total]; <>: λ&edge. λ&rest.\n");
    dstr_append(&ds, "  @edge3(λ&u. λ&v. λ&w. ! &cu = @get(u, comp); ! &cv = @get(v, comp);\n");
    dstr_append(&ds, "    λ{0: ! &nc = @relabel(cv, cu, comp); @merge(nc, total + w, rest);\n");
    dstr_append(&ds, "    λk. @merge(comp, total, rest)}(cu == cv), edge)}\n\n");
    
    // Round
    dstr_append(&ds, "@round = λ&comp. λ&edges. λ&n. λ&total.\n");
    dstr_append(&ds, "  ! &mins = @all_mins(comp, edges, n, 0);\n");
    dstr_append(&ds, "  @merge(comp, total, mins)\n\n");
    
    // Run iterations
    dstr_append(&ds, "@run = λ&iters. λ&comp. λ&edges. λ&n. λ&total. λ{0: total; λk.\n");
    dstr_append(&ds, "  ! &state = @round(comp, edges, n, total);\n");
    dstr_append(&ds, "  λ{<>: λ&nc. λst. λ{<>: λ&nt. λnil. @run(iters - 1, nc, edges, n, nt)}(st)}(state)}(iters)\n\n");
    
    // Generate edge list as [u,v,w] triples
    dstr_append(&ds, "@edges = [");
    for (uint32_t i = 0; i < g->n_edges; i++) {
        if (i > 0) dstr_append(&ds, ", ");
        dstr_appendf(&ds, "[%u, %u, %u]", 
                    g->edges[i].src, g->edges[i].dst, g->edges[i].weight);
    }
    dstr_append(&ds, "]\n\n");
    
    // Initial component labels
    dstr_append(&ds, "@comp = [");
    for (uint32_t i = 0; i < g->n_nodes; i++) {
        if (i > 0) dstr_append(&ds, ", ");
        dstr_appendf(&ds, "%u", i);
    }
    dstr_append(&ds, "]\n\n");
    
    dstr_appendf(&ds, "@main = @run(%u, @comp, @edges, %u, 0)\n", rounds, g->n_nodes);
    
    // Run
    uint32_t out_buf[1];
    int count = run_hvm4(ds.data, out_buf, 1);
    dstr_free(&ds);
    
    if (count < 0) {
        return HVM4_ERR_HVM4_RUNTIME;
    }
    
    *mst_weight = out_buf[0];
    return HVM4_OK;
}

hvm4_result_t hvm4_shortest_path(hvm4_graph_t *g,
                                 uint32_t source,
                                 uint32_t *dist) {
    if (!g || !dist) return HVM4_ERR_INVALID_PARAM;
    if (source >= g->n_nodes) return HVM4_ERR_INVALID_PARAM;
    
    reset_hvm4();
    
    dstring_t ds;
    dstr_init(&ds);
    
    uint32_t depth = ceil_log4_u32(g->n_nodes);
    uint32_t rounds = g->n_nodes > 1 ? g->n_nodes - 1 : 1;
    
    dstr_appendf(&ds, "@INF = %u\n", INF);
    dstr_appendf(&ds, "@DEPTH = %u\n\n", depth);
    
    // Generate trie ops
    gen_trie4_ops(&ds);
    
    // Edge relaxation
    dstr_append(&ds, "@relax_edge = λ&dist. λ{#Edge: λ&u. λ&v. λw.\n");
    dstr_append(&ds, "  ! &du = @q4_get(u, @DEPTH, dist);\n");
    dstr_append(&ds, "  ! &new_d = du + w;\n");
    dstr_append(&ds, "  ! &dv = @q4_get(v, @DEPTH, dist);\n");
    dstr_append(&ds, "  λ{0: dist; λn. @q4_set(v, new_d, @DEPTH, dist)}(new_d < dv)}\n\n");
    
    // Fold over edge list
    dstr_append(&ds, "@foldl = λ&f. λ&acc. λ{[]: acc; <>: λh. λt. @foldl(f, f(acc, h), t)}\n");
    dstr_append(&ds, "@relax_round = λdist. @foldl(@relax_edge, dist, @edges)\n");
    dstr_append(&ds, "@repeat = λ&f. λ&x. λ{0: x; λn. @repeat(f, f(x), n - 1)}\n\n");
    
    // Edge list
    gen_edge_list(&ds, g);
    dstr_append(&ds, "\n");
    
    // Initial distance
    dstr_appendf(&ds, "@init_dist = @q4_set(%u, 0, @DEPTH, #QE{})\n", source);
    
    // Run rounds
    dstr_appendf(&ds, "@bf = @repeat(@relax_round, @init_dist, %u)\n\n", rounds);
    
    // Extract all distances
    dstr_append(&ds, "@main = [");
    for (uint32_t i = 0; i < g->n_nodes; i++) {
        if (i > 0) dstr_append(&ds, ", ");
        dstr_appendf(&ds, "@q4_get(%u, @DEPTH, @bf)", i);
    }
    dstr_append(&ds, "]\n");
    
    // Run
    int count = run_hvm4(ds.data, dist, (int)g->n_nodes);
    dstr_free(&ds);
    
    if (count < 0) {
        return HVM4_ERR_HVM4_RUNTIME;
    }
    
    return HVM4_OK;
}

hvm4_result_t hvm4_reachable(hvm4_graph_t *g,
                             uint32_t source,
                             uint32_t target,
                             uint32_t max_depth,
                             uint32_t *dist) {
    if (!g || !dist) return HVM4_ERR_INVALID_PARAM;
    if (source >= g->n_nodes || target >= g->n_nodes) return HVM4_ERR_INVALID_PARAM;
    
    if (source == target) {
        *dist = 0;
        return HVM4_OK;
    }
    
    reset_hvm4();
    
    dstring_t ds;
    dstr_init(&ds);
    
    // Generate adjacency list (unweighted)
    gen_adjacency_list(&ds, g, 0);
    
    // BFS helpers
    dstr_append(&ds, "\n@member = λ&x. λ{[]: 0; <>: λ&h. λt. λ{0: @member(x, t); λn. 1}(h == x)}\n");
    dstr_append(&ds, "@any_in = λ&ys. λ{[]: 0; <>: λ&h. λt. λ{0: @any_in(ys, t); λn. 1}(@member(h, ys))}\n");
    dstr_append(&ds, "@append = λ{[]: λys. ys; <>: λh. λt. λys. h <> @append(t, ys)}\n");
    dstr_append(&ds, "@concat_map = λ&f. λ{[]: []; <>: λh. λt. @append(f(h), @concat_map(f, t))}\n");
    dstr_append(&ds, "@expand = λfrontier. @concat_map(@adj, frontier)\n\n");
    
    // BFS search
    dstr_append(&ds, "@bfs = λ&fwd. λ&bwd. λ&dist. λ&max. λ{\n");
    dstr_append(&ds, "  0: λ{0: ! &new_fwd = @expand(fwd); @bfs(bwd, new_fwd, dist + 1, max);\n");
    dstr_append(&ds, "  λn. dist}(@any_in(bwd, fwd));\n");
    dstr_append(&ds, "  λn. 999}(dist > max)\n\n");
    
    dstr_appendf(&ds, "@main = @bfs([%u], [%u], 0, %u)\n", source, target, max_depth);
    
    // Run
    uint32_t out_buf[1];
    int count = run_hvm4(ds.data, out_buf, 1);
    dstr_free(&ds);
    
    if (count < 0) {
        return HVM4_ERR_HVM4_RUNTIME;
    }
    
    if (out_buf[0] >= 999) {
        return HVM4_ERR_NO_PATH;
    }
    
    *dist = out_buf[0];
    return HVM4_OK;
}
