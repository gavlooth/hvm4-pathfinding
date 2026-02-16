module bench_common;
import std::io;

const uint INF = 999_999_999;

// ---------------------------------------------------------------------------
// LCG random number generator (identical to bench/*.c)
// ---------------------------------------------------------------------------

struct Lcg {
    uint s;
}

fn uint Lcg.next(&self) {
    self.s = (uint)((ulong)self.s * 1103515245 + 12345) & 0x7FFFFFFF;
    return self.s;
}

// ---------------------------------------------------------------------------
// CSR graph representation
// ---------------------------------------------------------------------------

struct CsrGraph {
    uint* row_ptr;  // size v+1
    uint* col_idx;  // size e
    uint* weight;   // size e (null for unweighted)
    uint v;
    uint e;
}

fn void CsrGraph.destroy(&self) {
    if (self.row_ptr) { free(self.row_ptr); self.row_ptr = null; }
    if (self.col_idx) { free(self.col_idx); self.col_idx = null; }
    if (self.weight)  { free(self.weight);  self.weight = null; }
}

// ---------------------------------------------------------------------------
// Raw edge for graph generation
// ---------------------------------------------------------------------------

struct RawEdge {
    uint u;
    uint v;
    uint w;
}

// ---------------------------------------------------------------------------
// Timing via clock_gettime
// ---------------------------------------------------------------------------

struct Timespec {
    long tv_sec;
    long tv_nsec;
}

extern fn int clock_gettime(int clk_id, Timespec* tp) @cname("clock_gettime");

const int CLOCK_MONOTONIC = 1;

fn Timespec time_now() {
    Timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts;
}

fn double time_elapsed_s(Timespec start) {
    Timespec now = time_now();
    return (double)(now.tv_sec - start.tv_sec) + (double)(now.tv_nsec - start.tv_nsec) * 1e-9;
}

// ---------------------------------------------------------------------------
// Peak RSS via getrusage
// ---------------------------------------------------------------------------

struct Rusage {
    // timeval ru_utime (16 bytes)
    long ru_utime_sec;
    long ru_utime_usec;
    // timeval ru_stime (16 bytes)
    long ru_stime_sec;
    long ru_stime_usec;
    // ru_maxrss
    long ru_maxrss;
    // remaining fields (padding)
    long[11] _pad;
}

extern fn int getrusage(int who, Rusage* usage) @cname("getrusage");

fn long peak_rss_kb() {
    Rusage ru;
    getrusage(0, &ru);  // RUSAGE_SELF = 0
    return ru.ru_maxrss;
}

// ---------------------------------------------------------------------------
// Peak RSS via /proc/self/status (more portable than getrusage struct)
// ---------------------------------------------------------------------------

extern fn void* fopen(ZString path, ZString mode) @cname("fopen");
extern fn int fclose(void* stream) @cname("fclose");
extern fn char* fgets(char* buf, int size, void* stream) @cname("fgets");
extern fn int strncmp(char* a, ZString b, usz n) @cname("strncmp");
extern fn long atol(char* s) @cname("atol");

fn long peak_rss_kb_proc() {
    void* f = fopen("/proc/self/status", "r");
    if (f == null) return 0;
    char[256] buf;
    while (fgets(&buf, 256, f) != null) {
        if (strncmp(&buf, "VmHWM:", 6) == 0) {
            fclose(f);
            // Skip "VmHWM:" and whitespace
            char* p = &buf[6];
            while (*p == ' ' || *p == '\t') { p++; }
            return atol(p);
        }
    }
    fclose(f);
    return 0;
}

// ---------------------------------------------------------------------------
// Sort utilities
// ---------------------------------------------------------------------------

fn void sort_uint(uint* arr, usz n) {
    // Insertion sort (sufficient for small arrays in tests)
    for (usz i = 1; i < n; i++) {
        uint key = arr[i];
        isz j = (isz)i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}
