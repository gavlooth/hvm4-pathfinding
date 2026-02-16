module bench_main;
import bench_ffi;
import bench_common;
import bench_bf;
import bench_ds;
import bench_ch;
import bench_apsp;
import bench_closure;
import bench_path_enum;
import bench_bidir_bfs;
import bench_dag_dp;
import bench_temporal;
import std::io;

fn void usage() {
    io::printn("Usage: bench <algorithm> [V] [extra]");
    io::printn("Algorithms: bf, ds, ch, apsp, closure, path_enum, bidir_bfs, dag_dp, temporal, all");
    io::printn("  temporal: V=spatial_nodes, extra=time_steps (default 10)");
    io::printn("Output: RESULT  name  V  interactions  ms  MIPS  RSS_KB");
}

fn uint parse_uint(String s) {
    uint val = 0;
    for (usz i = 0; i < s.len; i++) {
        char c = s[i];
        if (c >= '0' && c <= '9') {
            val = val * 10 + (uint)(c - '0');
        } else {
            break;
        }
    }
    return val;
}

fn bool streq(String a, String b) {
    if (a.len != b.len) return false;
    for (usz i = 0; i < a.len; i++) {
        if (a[i] != b[i]) return false;
    }
    return true;
}

fn bool run_algo(String name, uint v, uint extra) {
    if (streq(name, "bf")) return bench_bf::run(v, extra);
    if (streq(name, "ds")) return bench_ds::run(v, extra);
    if (streq(name, "ch")) return bench_ch::run(v, extra);
    if (streq(name, "apsp")) return bench_apsp::run(v);
    if (streq(name, "closure")) return bench_closure::run(v);
    if (streq(name, "path_enum")) return bench_path_enum::run(v, extra);
    if (streq(name, "bidir_bfs")) return bench_bidir_bfs::run(v, extra);
    if (streq(name, "dag_dp")) return bench_dag_dp::run(v, extra);
    if (streq(name, "temporal")) return bench_temporal::run(v, extra != 0 ? extra : 10, 4);
    return false;
}

struct BenchResult {
    bool ok;
    ulong itrs;
    double ms;
    double mips;
    long rss_kb;
}

fn BenchResult run_measured(String name, uint v, uint extra) {
    BenchResult r;
    bench_common::Timespec t0 = bench_common::time_now();
    r.ok = run_algo(name, v, extra);
    double secs = bench_common::time_elapsed_s(t0);
    r.itrs = bench_ffi::hvm4_itrs_total();
    r.ms = secs * 1000.0;
    r.mips = secs > 0.0 ? (double)r.itrs / secs / 1000000.0 : 0.0;
    r.rss_kb = bench_common::peak_rss_kb_proc();
    return r;
}

fn void print_result(String name, uint v, BenchResult* r) {
    if (r.ok) {
        io::printf("PASS  ");
    } else {
        io::printf("FAIL  ");
    }
    io::printf("%s  ", name);
    io::printf("V=%d  ", v);
    io::printf("itrs=%d  ", r.itrs);
    io::printf("%.1f ms  ", r.ms);
    io::printf("%.2f MIPS  ", r.mips);
    io::printfn("%d KB", r.rss_kb);
}

fn void main(String[] args) {
    if (args.len < 2) {
        usage();
        return;
    }

    String algo = args[1];
    uint v = args.len >= 3 ? parse_uint(args[2]) : 50;
    uint extra = args.len >= 4 ? parse_uint(args[3]) : 0;

    bench_ffi::hvm4_lib_init();
    defer bench_ffi::hvm4_lib_cleanup();

    if (streq(algo, "all")) {
        uint epn = extra != 0 ? extra : 4;
        uint apsp_v = v > 15 ? 15 : v;

        String[9] names = { "bf", "ds", "ch", "apsp", "closure", "path_enum", "bidir_bfs", "dag_dp", "temporal" };
        int pass = 0;
        int fail = 0;

        for (usz i = 0; i < 9; i++) {
            String name = names[i];
            uint tv = (streq(name, "apsp") || streq(name, "closure")) ? apsp_v : v;
            uint te;
            if (streq(name, "apsp") || streq(name, "closure")) {
                te = 0;
            } else if (streq(name, "path_enum")) {
                te = epn != 4 ? epn : 8;
            } else {
                te = epn;
            }

            BenchResult r = run_measured(name, tv, te);
            print_result(name, tv, &r);
            if (r.ok) { pass++; } else { fail++; }
        }

        io::printfn("\nResults: %d passed, %d failed", pass, fail);
    } else {
        if (extra == 0) {
            if (streq(algo, "temporal")) {
                extra = 10;
            } else if (streq(algo, "path_enum")) {
                extra = 8;
            } else if (!streq(algo, "apsp") && !streq(algo, "closure")) {
                extra = 4;
            }
        }
        BenchResult r = run_measured(algo, v, extra);
        print_result(algo, v, &r);
    }
}
