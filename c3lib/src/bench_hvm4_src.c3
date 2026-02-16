module bench_hvm4_src;
import std::core::dstring;

alias DStr = dstring::DString;

fn DStr new_dstr() {
    return dstring::new(mem);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn uint ceil_log2(uint n) {
    uint d = 0;
    uint cap = 1;
    while (cap < n) { d++; cap *= 2; }
    return d;
}

fn uint ceil_log4(uint n) {
    if (n <= 4) return 1;
    uint depth = 1;
    uint cap = 4;
    while (cap < n) { depth++; cap *= 4; }
    return depth;
}

// B64 alphabet for dag_dp node names (matches HVM4 nick encoding)
const ZString B64 = "_abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789$";

fn void append_node_name(DStr* ds, uint n) {
    ds.append_char('e');
    ds.append_char(B64[(n >> 12) & 63]);
    ds.append_char(B64[(n >> 6) & 63]);
    ds.append_char(B64[n & 63]);
}

// ---------------------------------------------------------------------------
// 1. Bellman-Ford (FFI: %dist_get, %dist_upd, %graph_*)
// ---------------------------------------------------------------------------

fn DStr gen_bf_source(uint v, uint dest) {
    uint rounds = v > 1 ? v - 1 : 1;
    DStr ds = new_dstr();

    ds.appendf("@INF = 999999999\n@V = %d\n@DEST = %d\n@ROUNDS = %d\n",
               v, dest, rounds);

    // @rel: relax edges from node u (du = dist[u])
    ds.append_string(
        "@rel = \xce\xbbu. \xce\xbbdu. \xce\xbb&i. \xce\xbb&deg. "
        "@rgo(i < deg, u, du, i, deg)\n"

        "@rgo = \xce\xbb{"
        "0: \xce\xbbu. \xce\xbbdu. \xce\xbbi. \xce\xbbdeg. 0; "
        "\xce\xbbn. \xce\xbb&u. \xce\xbb&du. \xce\xbb&i. \xce\xbbdeg. "
        "! v = %graph_target(u, i); "
        "! w = %graph_weight(u, i); "
        "! ndv = du + w; "
        "%dist_upd(v, ndv) + @rel(u, du, i + 1, deg)}\n"
    );

    // @rall: relax all nodes 0..V-1
    ds.append_string(
        "@rall = \xce\xbb&u. @rago(u < @V, u)\n"

        "@rago = \xce\xbb{"
        "0: \xce\xbbu. 0; "
        "\xce\xbbn. \xce\xbb&u. ! &du = %dist_get(u); @rask(du < @INF, u, du)}\n"

        "@rask = \xce\xbb{"
        "0: \xce\xbbu. \xce\xbbdu. @rall(u + 1); "
        "\xce\xbbn. \xce\xbb&u. \xce\xbbdu. "
        "@rel(u, du, 0, %graph_deg(u)) + @rall(u + 1)}\n"
    );

    // @bf: main loop with early termination
    ds.append_string(
        "@bf = \xce\xbbround. @bfgo(round)\n"

        "@bfgo = \xce\xbb{"
        "0: %dist_get(@DEST); "
        "\xce\xbbk. ! changed = @rall(0); @bfet(changed, k)}\n"

        "@bfet = \xce\xbb{"
        "0: \xce\xbbk. %dist_get(@DEST); "
        "\xce\xbbn. \xce\xbbk. @bf(k - 1)}\n"
    );

    ds.append_string("@main = @bf(@ROUNDS)\n");
    return ds;
}

// ---------------------------------------------------------------------------
// 2. Delta-Stepping (FFI: %dist_get, %ds_next, %ds_relax, %graph_*)
// ---------------------------------------------------------------------------

fn DStr gen_ds_source(uint v, uint dest) {
    DStr ds = new_dstr();

    ds.appendf("@V = %d\n@DEST = %d\n", v, dest);

    // @ds_rel: relax all edges of node v
    ds.append_string(
        "@ds_rel = \xce\xbbv. \xce\xbbdv. \xce\xbb&i. \xce\xbb&deg. "
        "@ds_rgo(i < deg, v, dv, i, deg)\n"

        "@ds_rgo = \xce\xbb{"
        "0: \xce\xbbv. \xce\xbbdv. \xce\xbbi. \xce\xbbdeg. 0; "
        "\xce\xbbn. \xce\xbb&v. \xce\xbb&dv. \xce\xbb&i. \xce\xbbdeg. "
        "! u = %graph_target(v, i); "
        "! w = %graph_weight(v, i); "
        "! nd = dv + w; "
        "%ds_relax(u, nd) + @ds_rel(v, dv, i + 1, deg)}\n"
    );

    // @ds_loop: pop from PQ, relax, repeat
    ds.append_string(
        "@ds_loop = \xce\xbb{"
        "0: \xce\xbbv. %dist_get(@DEST); "
        "\xce\xbbn. \xce\xbb&v. "
        "! dv = %dist_get(v); "
        "! &v2 = %ds_next(@ds_rel(v, dv, 0, %graph_deg(v))); "
        "@ds_loop(v2 < @V, v2)}\n"
    );

    ds.append_string("@main = ! &v = %ds_next(0); @ds_loop(v < @V, v)\n");
    return ds;
}

// ---------------------------------------------------------------------------
// 3. Contraction Hierarchy (FFI: %ch_deg, %ch_target, %ch_weight,
//    %ch_get, %ch_relax, %ch_next, %ch_meet)
// ---------------------------------------------------------------------------

fn DStr gen_ch_source(uint v) {
    DStr ds = new_dstr();

    ds.appendf("@V = %d\n", v);

    // @dj_rel: relax edges of node v in direction mode
    ds.append_string(
        "@dj_rel = \xce\xbbmode. \xce\xbbv. \xce\xbbdv. \xce\xbb&i. \xce\xbb&deg. "
        "@dj_rgo(i < deg, mode, v, dv, i, deg)\n"

        "@dj_rgo = \xce\xbb{"
        "0: \xce\xbbmode. \xce\xbbv. \xce\xbbdv. \xce\xbbi. \xce\xbbdeg. 0; "
        "\xce\xbbn. \xce\xbb&mode. \xce\xbb&v. \xce\xbb&dv. \xce\xbb&i. \xce\xbbdeg. "
        "! u = %ch_target(mode, v, i); "
        "! w = %ch_weight(mode, v, i); "
        "! nd = dv + w; "
        "%ch_relax(mode, u, nd) + @dj_rel(mode, v, dv, i + 1, deg)}\n"
    );

    // @dj_loop: Dijkstra loop for one direction
    ds.append_string(
        "@dj_loop = \xce\xbb{"
        "0: \xce\xbbmode. \xce\xbbv. 0; "
        "\xce\xbbn. \xce\xbb&mode. \xce\xbb&v. "
        "! dv = %ch_get(mode, v); "
        "! &v2 = %ch_next(mode, @dj_rel(mode, v, dv, 0, %ch_deg(mode, v))); "
        "@dj_loop(v2 < @V, mode, v2)}\n"
    );

    // @dijkstra: run one direction
    ds.append_string(
        "@dijkstra = \xce\xbb&mode. "
        "! &v = %ch_next(mode, 0); "
        "@dj_loop(v < @V, mode, v)\n"
    );

    ds.append_string("@main = @dijkstra(0) + @dijkstra(1) + %ch_meet(0)\n");
    return ds;
}

// ---------------------------------------------------------------------------
// 4. APSP — Algebraic (FFI: %graph_deg, %graph_target, %graph_weight)
// ---------------------------------------------------------------------------

fn DStr gen_apsp_source(uint n, uint src, uint dest) {
    uint iters = ceil_log2(n);
    DStr ds = new_dstr();

    ds.appendf("@INF = 999999\n@N = %d\n", n);

    // min
    ds.append_string(
        "@min = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: b; \xce\xbbn. a}(a < b)\n"
    );

    // nth (list indexing)
    ds.append_string(
        "@nth = \xce\xbb&i. \xce\xbb{"
        "[]: @INF; "
        "<>: \xce\xbb&h. \xce\xbbt. @ngo(i, h, t)}\n"
        "@ngo = \xce\xbb{"
        "0: \xce\xbbh. \xce\xbbt. h; "
        "\xce\xbbk. \xce\xbbh. \xce\xbbt. @nth(k - 1, t)}\n"
    );

    // dot (tropical dot product)
    ds.append_string(
        "@dot = \xce\xbb&A. \xce\xbb&B. \xce\xbb&N. \xce\xbb&i. \xce\xbb&j. \xce\xbb&k. \xce\xbb&rem. "
        "@dgo(rem, A, B, N, i, j, k)\n"

        "@dgo = \xce\xbb{"
        "0: \xce\xbbA. \xce\xbbB. \xce\xbbN. \xce\xbbi. \xce\xbbj. \xce\xbbk. @INF; "
        "\xce\xbbr. \xce\xbb&A. \xce\xbb&B. \xce\xbb&N. \xce\xbb&i. \xce\xbb&j. \xce\xbb&k. "
        "! &aik = @nth(i * N + k, A); "
        "! &bkj = @nth(k * N + j, B); "
        "! &trm = aik + bkj; "
        "! &rst = @dot(A, B, N, i, j, k + 1, r - 1); "
        "@min(trm, rst)}\n"
    );

    // bld (build result cells for mat_mul)
    ds.append_string(
        "@bld = \xce\xbb&A. \xce\xbb&B. \xce\xbb&N. \xce\xbb&idx. \xce\xbb&rem. "
        "@blgo(rem, A, B, N, idx)\n"

        "@blgo = \xce\xbb{"
        "0: \xce\xbbA. \xce\xbbB. \xce\xbbN. \xce\xbbidx. []; "
        "\xce\xbbr. \xce\xbb&A. \xce\xbb&B. \xce\xbb&N. \xce\xbb&idx. "
        "! &i = idx / N; ! &j = idx % N; "
        "! &val = @dot(A, B, N, i, j, 0, N); "
        "val <> @bld(A, B, N, idx + 1, r - 1)}\n"
    );

    // mmul
    ds.append_string(
        "@mmul = \xce\xbb&A. \xce\xbb&B. \xce\xbb&N. @bld(A, B, N, 0, N * N)\n"
    );

    // bini (build initial matrix from FFI)
    ds.append_string(
        "@bini = \xce\xbb&idx. \xce\xbb&rem. \xce\xbb&N. "
        "@bigo(rem, idx, N)\n"

        "@bigo = \xce\xbb{"
        "0: \xce\xbbidx. \xce\xbbN. []; "
        "\xce\xbbr. \xce\xbb&idx. \xce\xbb&N. "
        "! &i = idx / N; ! &j = idx % N; "
        "! &val = @cel(i, j); "
        "val <> @bini(idx + 1, r - 1, N)}\n"
    );

    // cel (single cell: 0 on diagonal, scan edges otherwise)
    ds.append_string(
        "@cel = \xce\xbb&i. \xce\xbb&j. "
        "@ceq(i == j, i, j)\n"

        "@ceq = \xce\xbb{"
        "0: \xce\xbb&i. \xce\xbbj. @fed(i, j, 0, %graph_deg(i)); "
        "\xce\xbbk. \xce\xbbi. \xce\xbbj. 0}\n"
    );

    // fed (find edge u->j by scanning neighbors)
    ds.append_string(
        "@fed = \xce\xbb&u. \xce\xbb&j. \xce\xbb&k. \xce\xbb&deg. "
        "@fego(k < deg, u, j, k, deg)\n"

        "@fego = \xce\xbb{"
        "0: \xce\xbbu. \xce\xbbj. \xce\xbbk. \xce\xbbdeg. @INF; "
        "\xce\xbbn. \xce\xbb&u. \xce\xbb&j. \xce\xbb&k. \xce\xbbdeg. "
        "! &t = %graph_target(u, k); "
        "@fck(t == j, u, j, k, deg)}\n"

        "@fck = \xce\xbb{"
        "0: \xce\xbbu. \xce\xbbj. \xce\xbbk. \xce\xbbdeg. @fed(u, j, k + 1, deg); "
        "\xce\xbbn. \xce\xbbu. \xce\xbbj. \xce\xbbk. \xce\xbbdeg. %graph_weight(u, k)}\n"
    );

    // mmin (element-wise min)
    ds.append_string(
        "@mmin = \xce\xbba. \xce\xbbb. @mmga(a, b)\n"

        "@mmga = \xce\xbb{"
        "[]: \xce\xbbb. []; "
        "<>: \xce\xbbah. \xce\xbbat. \xce\xbbb. @mm2(b, ah, at)}\n"

        "@mm2 = \xce\xbb{"
        "[]: \xce\xbbah. \xce\xbbat. ah <> at; "
        "<>: \xce\xbbbh. \xce\xbbbt. \xce\xbbah. \xce\xbbat. "
        "! &val = @min(ah, bh); "
        "val <> @mmin(at, bt)}\n"
    );

    // clos (repeated squaring)
    ds.append_string(
        "@clos = \xce\xbb&iter. \xce\xbb&D. \xce\xbb&N. "
        "@clgo(iter, D, N)\n"

        "@clgo = \xce\xbb{"
        "0: \xce\xbbD. \xce\xbbN. D; "
        "\xce\xbbk. \xce\xbb&D. \xce\xbb&N. "
        "! &D2 = @mmul(D, D, N); "
        "! &Dnw = @mmin(D, D2); "
        "@clos(k - 1, Dnw, N)}\n"
    );

    // main
    ds.appendf(
        "@main = "
        "! &D = @bini(0, %d, @N); "
        "! &R = @clos(%d, D, @N); "
        "@nth(%d, R)\n",
        n * n, iters, src * n + dest
    );

    return ds;
}

// ---------------------------------------------------------------------------
// 5. Transitive Closure (FFI: %graph_deg, %graph_target)
// ---------------------------------------------------------------------------

fn DStr gen_closure_source(uint n, uint src, uint dest) {
    uint iters = ceil_log2(n);
    DStr ds = new_dstr();

    ds.appendf("@N = %d\n", n);

    // bor, band
    ds.append_string(
        "@bor = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: b; \xce\xbbn. 1}(a)\n"
        "@band = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: 0; \xce\xbbn. b}(a)\n"
    );

    // nth (default = 0)
    ds.append_string(
        "@nth = \xce\xbb&i. \xce\xbb{"
        "[]: 0; "
        "<>: \xce\xbb&h. \xce\xbbt. @ngo(i, h, t)}\n"
        "@ngo = \xce\xbb{"
        "0: \xce\xbbh. \xce\xbbt. h; "
        "\xce\xbbk. \xce\xbbh. \xce\xbbt. @nth(k - 1, t)}\n"
    );

    // bdot (boolean dot product)
    ds.append_string(
        "@bdot = \xce\xbb&A. \xce\xbb&B. \xce\xbb&N. \xce\xbb&i. \xce\xbb&j. \xce\xbb&k. \xce\xbb&rem. "
        "@bdgo(rem, A, B, N, i, j, k)\n"

        "@bdgo = \xce\xbb{"
        "0: \xce\xbbA. \xce\xbbB. \xce\xbbN. \xce\xbbi. \xce\xbbj. \xce\xbbk. 0; "
        "\xce\xbbr. \xce\xbb&A. \xce\xbb&B. \xce\xbb&N. \xce\xbb&i. \xce\xbb&j. \xce\xbb&k. "
        "! &aik = @nth(i * N + k, A); "
        "! &bkj = @nth(k * N + j, B); "
        "! &trm = @band(aik, bkj); "
        "! &rst = @bdot(A, B, N, i, j, k + 1, r - 1); "
        "@bor(trm, rst)}\n"
    );

    // bld
    ds.append_string(
        "@bld = \xce\xbb&A. \xce\xbb&B. \xce\xbb&N. \xce\xbb&idx. \xce\xbb&rem. "
        "@blgo(rem, A, B, N, idx)\n"

        "@blgo = \xce\xbb{"
        "0: \xce\xbbA. \xce\xbbB. \xce\xbbN. \xce\xbbidx. []; "
        "\xce\xbbr. \xce\xbb&A. \xce\xbb&B. \xce\xbb&N. \xce\xbb&idx. "
        "! &i = idx / N; ! &j = idx % N; "
        "! &val = @bdot(A, B, N, i, j, 0, N); "
        "val <> @bld(A, B, N, idx + 1, r - 1)}\n"
    );

    // mmul
    ds.append_string(
        "@mmul = \xce\xbb&A. \xce\xbb&B. \xce\xbb&N. @bld(A, B, N, 0, N * N)\n"
    );

    // bini
    ds.append_string(
        "@bini = \xce\xbb&idx. \xce\xbb&rem. \xce\xbb&N. "
        "@bigo(rem, idx, N)\n"

        "@bigo = \xce\xbb{"
        "0: \xce\xbbidx. \xce\xbbN. []; "
        "\xce\xbbr. \xce\xbb&idx. \xce\xbb&N. "
        "! &i = idx / N; ! &j = idx % N; "
        "! &val = @cel(i, j); "
        "val <> @bini(idx + 1, r - 1, N)}\n"
    );

    // cel (1 on diagonal, check edge otherwise)
    ds.append_string(
        "@cel = \xce\xbb&i. \xce\xbb&j. "
        "@ceq(i == j, i, j)\n"

        "@ceq = \xce\xbb{"
        "0: \xce\xbb&i. \xce\xbbj. @fed(i, j, 0, %graph_deg(i)); "
        "\xce\xbbk. \xce\xbbi. \xce\xbbj. 1}\n"
    );

    // fed (find edge: 1 if found, 0 if not)
    ds.append_string(
        "@fed = \xce\xbb&u. \xce\xbb&j. \xce\xbb&k. \xce\xbb&deg. "
        "@fego(k < deg, u, j, k, deg)\n"

        "@fego = \xce\xbb{"
        "0: \xce\xbbu. \xce\xbbj. \xce\xbbk. \xce\xbbdeg. 0; "
        "\xce\xbbn. \xce\xbb&u. \xce\xbb&j. \xce\xbb&k. \xce\xbbdeg. "
        "! &t = %graph_target(u, k); "
        "@fck(t == j, u, j, k, deg)}\n"

        "@fck = \xce\xbb{"
        "0: \xce\xbbu. \xce\xbbj. \xce\xbbk. \xce\xbbdeg. @fed(u, j, k + 1, deg); "
        "\xce\xbbn. \xce\xbbu. \xce\xbbj. \xce\xbbk. \xce\xbbdeg. 1}\n"
    );

    // mor (element-wise OR)
    ds.append_string(
        "@mor = \xce\xbba. \xce\xbbb. @morga(a, b)\n"

        "@morga = \xce\xbb{"
        "[]: \xce\xbbb. []; "
        "<>: \xce\xbbah. \xce\xbbat. \xce\xbbb. @mor2(b, ah, at)}\n"

        "@mor2 = \xce\xbb{"
        "[]: \xce\xbbah. \xce\xbbat. ah <> at; "
        "<>: \xce\xbbbh. \xce\xbbbt. \xce\xbbah. \xce\xbbat. "
        "! &val = @bor(ah, bh); "
        "val <> @mor(at, bt)}\n"
    );

    // clos
    ds.append_string(
        "@clos = \xce\xbb&iter. \xce\xbb&D. \xce\xbb&N. "
        "@clgo(iter, D, N)\n"

        "@clgo = \xce\xbb{"
        "0: \xce\xbbD. \xce\xbbN. D; "
        "\xce\xbbk. \xce\xbb&D. \xce\xbb&N. "
        "! &D2 = @mmul(D, D, N); "
        "! &Dnw = @mor(D, D2); "
        "@clos(k - 1, Dnw, N)}\n"
    );

    // main
    ds.appendf(
        "@main = "
        "! &D = @bini(0, %d, @N); "
        "! &R = @clos(%d, D, @N); "
        "@nth(%d, R)\n",
        n * n, iters, src * n + dest
    );

    return ds;
}

// ---------------------------------------------------------------------------
// 6. Path Enumeration (FFI: %graph_deg, %graph_target, %graph_weight)
// ---------------------------------------------------------------------------

fn DStr gen_path_enum_source(uint dest) {
    DStr ds = new_dstr();

    // apnd (list append)
    ds.append_string(
        "@apnd = \xce\xbbxs. \xce\xbb&ys. @apgo(xs, ys)\n"
        "@apgo = \xce\xbb{"
        "[]: \xce\xbbys. ys; "
        "<>: \xce\xbbh. \xce\xbbt. \xce\xbb&ys. h <> @apnd(t, ys)}\n"
    );

    // expl (DFS from node, accumulating weight)
    ds.appendf(
        "@expl = \xce\xbb&node. \xce\xbb&wt. "
        "@exgo(node == %d, node, wt)\n",
        dest
    );

    // exgo
    ds.append_string(
        "@exgo = \xce\xbb{"
        "0: \xce\xbb&node. \xce\xbbwt. @br(node, wt, 0, %graph_deg(node)); "
        "\xce\xbbn. \xce\xbbnode. \xce\xbbwt. wt <> []}\n"
    );

    // br (branch over neighbors)
    ds.append_string(
        "@br = \xce\xbb&node. \xce\xbb&wt. \xce\xbb&i. \xce\xbb&deg. "
        "@brgo(i < deg, node, wt, i, deg)\n"

        "@brgo = \xce\xbb{"
        "0: \xce\xbbnode. \xce\xbbwt. \xce\xbbi. \xce\xbbdeg. []; "
        "\xce\xbbn. \xce\xbb&node. \xce\xbb&wt. \xce\xbb&i. \xce\xbbdeg. "
        "! &v = %graph_target(node, i); "
        "! &w = %graph_weight(node, i); "
        "! &sub = @expl(v, wt + w); "
        "! &rest = @br(node, wt, i + 1, deg); "
        "@apnd(sub, rest)}\n"
    );

    ds.append_string("@main = @expl(0, 0)\n");
    return ds;
}

// ---------------------------------------------------------------------------
// 7. Bidirectional BFS (Q4 trie in HVM4, FFI: %graph_deg, %graph_target)
// ---------------------------------------------------------------------------

fn DStr gen_bidir_bfs_source(uint n, uint src, uint dest) {
    uint depth = ceil_log4(n);
    DStr ds = new_dstr();

    ds.appendf("@INF = 999999\n@DEPTH = %d\n", depth);

    // Q4 trie: get_lin + set
    ds.append_string(
        "@q4_get_lin = \xce\xbb&key. \xce\xbb&depth. \xce\xbb{"
        "#QE: #P{@INF, #QE{}}; "
        "#QL: \xce\xbb&val. #P{val, #QL{val}}; "
        "#Q: \xce\xbbc0. \xce\xbbc1. \xce\xbbc2. \xce\xbbc3. "
        "! slot = key % 4; ! next = key / 4; ! nd = depth - 1; "
        "@q4gl_Q(slot, next, nd, c0, c1, c2, c3)}\n"

        "@q4gl_Q = \xce\xbb{"
        "0: \xce\xbbnext. \xce\xbbnd. \xce\xbbc0. \xce\xbbc1. \xce\xbbc2. \xce\xbbc3. "
        "\xce\xbb{#P: \xce\xbbval. \xce\xbbnc0. #P{val, #Q{nc0, c1, c2, c3}}}(@q4_get_lin(next, nd, c0)); "
        "1: \xce\xbbnext. \xce\xbbnd. \xce\xbbc0. \xce\xbbc1. \xce\xbbc2. \xce\xbbc3. "
        "\xce\xbb{#P: \xce\xbbval. \xce\xbbnc1. #P{val, #Q{c0, nc1, c2, c3}}}(@q4_get_lin(next, nd, c1)); "
        "2: \xce\xbbnext. \xce\xbbnd. \xce\xbbc0. \xce\xbbc1. \xce\xbbc2. \xce\xbbc3. "
        "\xce\xbb{#P: \xce\xbbval. \xce\xbbnc2. #P{val, #Q{c0, c1, nc2, c3}}}(@q4_get_lin(next, nd, c2)); "
        "\xce\xbbn. \xce\xbbnext. \xce\xbbnd. \xce\xbbc0. \xce\xbbc1. \xce\xbbc2. \xce\xbbc3. "
        "\xce\xbb{#P: \xce\xbbval. \xce\xbbnc3. #P{val, #Q{c0, c1, c2, nc3}}}(@q4_get_lin(next, nd, c3))}\n"

        "@q4_set = \xce\xbb&key. \xce\xbb&val. \xce\xbb&depth. \xce\xbb{"
        "#QL: \xce\xbbold. #QL{val}; "
        "#QE: \xce\xbb{0: #QL{val}; \xce\xbbn. "
        "! slot = key % 4; ! next = key / 4; ! nd = depth - 1; "
        "@q4s_QE(slot, next, val, nd)}(depth); "
        "#Q: \xce\xbbc0. \xce\xbbc1. \xce\xbbc2. \xce\xbbc3. "
        "! slot = key % 4; ! next = key / 4; ! nd = depth - 1; "
        "@q4s_Q(slot, next, val, nd, c0, c1, c2, c3)}\n"

        "@q4s_QE = \xce\xbb{"
        "0: \xce\xbbnext. \xce\xbbval. \xce\xbbnd. #Q{@q4_set(next, val, nd, #QE{}), #QE{}, #QE{}, #QE{}}; "
        "1: \xce\xbbnext. \xce\xbbval. \xce\xbbnd. #Q{#QE{}, @q4_set(next, val, nd, #QE{}), #QE{}, #QE{}}; "
        "2: \xce\xbbnext. \xce\xbbval. \xce\xbbnd. #Q{#QE{}, #QE{}, @q4_set(next, val, nd, #QE{}), #QE{}}; "
        "\xce\xbbn. \xce\xbbnext. \xce\xbbval. \xce\xbbnd. #Q{#QE{}, #QE{}, #QE{}, @q4_set(next, val, nd, #QE{})}}\n"

        "@q4s_Q = \xce\xbb{"
        "0: \xce\xbbnext. \xce\xbbval. \xce\xbbnd. \xce\xbbc0. \xce\xbbc1. \xce\xbbc2. \xce\xbbc3. #Q{@q4_set(next, val, nd, c0), c1, c2, c3}; "
        "1: \xce\xbbnext. \xce\xbbval. \xce\xbbnd. \xce\xbbc0. \xce\xbbc1. \xce\xbbc2. \xce\xbbc3. #Q{c0, @q4_set(next, val, nd, c1), c2, c3}; "
        "2: \xce\xbbnext. \xce\xbbval. \xce\xbbnd. \xce\xbbc0. \xce\xbbc1. \xce\xbbc2. \xce\xbbc3. #Q{c0, c1, @q4_set(next, val, nd, c2), c3}; "
        "\xce\xbbn. \xce\xbbnext. \xce\xbbval. \xce\xbbnd. \xce\xbbc0. \xce\xbbc1. \xce\xbbc2. \xce\xbbc3. #Q{c0, c1, c2, @q4_set(next, val, nd, c3)}}\n"
    );

    // min helper
    ds.append_string(
        "@min = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: b; \xce\xbbn. a}(a < b)\n"
    );

    // expand_nbrs
    ds.append_string(
        "@expand_nbrs = \xce\xbb&u. \xce\xbb&i. \xce\xbb&deg. \xce\xbbfront. \xce\xbb&d. \xce\xbbvis. "
        "@enb_ck(i < deg, u, i, deg, front, d, vis)\n"

        "@enb_ck = \xce\xbb{"
        "0: \xce\xbbu. \xce\xbbi. \xce\xbbdeg. \xce\xbbfront. \xce\xbbd. \xce\xbbvis. #P{front, vis}; "
        "\xce\xbbn. \xce\xbb&u. \xce\xbb&i. \xce\xbbdeg. \xce\xbbfront. \xce\xbb&d. \xce\xbbvis. "
        "! &v = %graph_target(u, i); "
        "\xce\xbb{#P: \xce\xbb&vd. \xce\xbbvis2. "
        "@enb_go(vd < @INF, u, i, deg, front, d, v, vis2)"
        "}(@q4_get_lin(v, @DEPTH, vis))}\n"

        "@enb_go = \xce\xbb{"
        "0: \xce\xbbu. \xce\xbbi. \xce\xbbdeg. \xce\xbbfront. \xce\xbb&d. \xce\xbb&v. \xce\xbbvis. "
        "@expand_nbrs(u, i + 1, deg, v <> front, d, @q4_set(v, d, @DEPTH, vis)); "
        "\xce\xbbm. \xce\xbbu. \xce\xbbi. \xce\xbbdeg. \xce\xbbfront. \xce\xbbd. \xce\xbbv. \xce\xbbvis. "
        "@expand_nbrs(u, i + 1, deg, front, d, vis)}\n"
    );

    // expand_all
    ds.append_string(
        "@exp_all = \xce\xbb{"
        "[]: \xce\xbbnf. \xce\xbbd. \xce\xbbvis. #P{nf, vis}; "
        "<>: \xce\xbb&u. \xce\xbbrest. \xce\xbbnf. \xce\xbb&d. \xce\xbbvis. "
        "\xce\xbb{#P: \xce\xbbf2. \xce\xbbv2. "
        "@exp_all(rest, f2, d, v2)"
        "}(@expand_nbrs(u, 0, %graph_deg(u), nf, d, vis))}\n"
    );

    // check_meet
    ds.append_string(
        "@chk_mt = \xce\xbb{"
        "[]: \xce\xbbd. \xce\xbbbest. \xce\xbbopp. #P{best, opp}; "
        "<>: \xce\xbb&v. \xce\xbbrest. \xce\xbb&d. \xce\xbb&best. \xce\xbbopp. "
        "\xce\xbb{#P: \xce\xbb&od. \xce\xbbopp2. "
        "@cm_go(od < @INF, rest, d, best, od, opp2)"
        "}(@q4_get_lin(v, @DEPTH, opp))}\n"

        "@cm_go = \xce\xbb{"
        "0: \xce\xbbrest. \xce\xbbd. \xce\xbbbest. \xce\xbbod. \xce\xbbopp. @chk_mt(rest, d, best, opp); "
        "\xce\xbbm. \xce\xbbrest. \xce\xbb&d. \xce\xbbbest. \xce\xbbod. \xce\xbbopp. @chk_mt(rest, d, @min(best, d + od), opp)}\n"
    );

    // bfs_loop
    ds.append_string(
        "@bfs = \xce\xbb&fd. \xce\xbb&bd. \xce\xbb&best. \xce\xbbff. \xce\xbbbf. \xce\xbbfv. \xce\xbbbv. "
        "! &nfd = fd + 1; ! &nbd = bd + 1; "
        "\xce\xbb{#P: \xce\xbb&nff. \xce\xbbfv2. "
        "\xce\xbb{#P: \xce\xbb&bst2. \xce\xbbbv2. "
        "\xce\xbb{#P: \xce\xbb&nbf. \xce\xbbbv3. "
        "\xce\xbb{#P: \xce\xbb&bst3. \xce\xbbfv3. "
        "@bdn(bst3 < @INF, nff, nbf, fv3, bv3, nfd, nbd, bst3)"
        "}(@chk_mt(nbf, nbd, bst2, fv2))"
        "}(@exp_all(bf, [], nbd, bv2))"
        "}(@chk_mt(nff, nfd, best, bv))"
        "}(@exp_all(ff, [], nfd, fv))\n"
    );

    // bfs_done + empty checks
    ds.append_string(
        "@bdn = \xce\xbb{"
        "0: \xce\xbbff. \xce\xbbbf. \xce\xbbfv. \xce\xbbbv. \xce\xbbfd. \xce\xbbbd. \xce\xbbbst. "
        "@bmt(ff, bf, fv, bv, fd, bd); "
        "\xce\xbbm. \xce\xbbff. \xce\xbbbf. \xce\xbbfv. \xce\xbbbv. \xce\xbbfd. \xce\xbbbd. \xce\xbbbst. bst}\n"

        "@bmt = \xce\xbb{"
        "[]: \xce\xbbbf. \xce\xbbfv. \xce\xbbbv. \xce\xbbfd. \xce\xbbbd. @INF; "
        "<>: \xce\xbbh. \xce\xbbt. \xce\xbbbf. \xce\xbbfv. \xce\xbbbv. \xce\xbbfd. \xce\xbbbd. "
        "@bm2(bf, h <> t, fv, bv, fd, bd)}\n"

        "@bm2 = \xce\xbb{"
        "[]: \xce\xbbff. \xce\xbbfv. \xce\xbbbv. \xce\xbbfd. \xce\xbbbd. @INF; "
        "<>: \xce\xbbh2. \xce\xbbt2. \xce\xbbff. \xce\xbbfv. \xce\xbbbv. \xce\xbbfd. \xce\xbbbd. "
        "@bfs(fd, bd, @INF, ff, h2 <> t2, fv, bv)}\n"
    );

    // main: init visited tries and start BFS
    ds.appendf(
        "@main = "
        "! fv = @q4_set(%d, 0, @DEPTH, #QE{}); "
        "! bv = @q4_set(%d, 0, @DEPTH, #QE{}); "
        "@bfs(0, 0, @INF, [%d], [%d], fv, bv)\n",
        src, dest, src, dest
    );

    return ds;
}

// ---------------------------------------------------------------------------
// 8. DAG-DP (embedded graph in let-bindings, no FFI)
// ---------------------------------------------------------------------------

fn DStr gen_dag_dp_source(uint n, uint* rp, uint* ci, uint* wt, uint src, uint dest) {
    DStr ds = new_dstr();

    ds.append_string("@min = \xce\xbb&a. \xce\xbb&b. \xce\xbb{0: b; \xce\xbbn. a}(a < b)\n");
    ds.append_string("@INF = 999999\n");
    ds.append_string("@main =\n");

    // Destination binding
    ds.append_string("  ! &");
    append_node_name(&ds, dest);
    ds.append_string(" = 0;\n");

    // Non-source, non-dest nodes in reverse order
    for (int u = (int)n - 2; u >= 0; u--) {
        if ((uint)u == src) continue;
        if ((uint)u == dest) continue;

        uint deg = rp[u + 1] - rp[u];
        if (deg == 0) {
            ds.append_string("  ! &");
            append_node_name(&ds, (uint)u);
            ds.append_string(" = @INF;\n");
        } else if (deg == 1) {
            uint e = rp[u];
            ds.append_string("  ! &");
            append_node_name(&ds, (uint)u);
            ds.appendf(" = %d + ", wt[e]);
            append_node_name(&ds, ci[e]);
            ds.append_string(";\n");
        } else {
            ds.append_string("  ! &");
            append_node_name(&ds, (uint)u);
            ds.append_string(" = ");
            for (uint i = 0; i < deg - 1; i++) {
                uint e = rp[u] + i;
                ds.appendf("@min(%d + ", wt[e]);
                append_node_name(&ds, ci[e]);
                ds.append_string(", ");
            }
            uint e = rp[u] + deg - 1;
            ds.appendf("%d + ", wt[e]);
            append_node_name(&ds, ci[e]);
            for (uint i = 0; i < deg - 1; i++) ds.append_string(")");
            ds.append_string(";\n");
        }
    }

    // Source node: return expression
    uint src_deg = rp[src + 1] - rp[src];
    if (src_deg == 0) {
        ds.append_string("  @INF\n");
    } else if (src_deg == 1) {
        uint e = rp[src];
        ds.appendf("  %d + ", wt[e]);
        append_node_name(&ds, ci[e]);
        ds.append_string("\n");
    } else {
        ds.append_string("  ");
        for (uint i = 0; i < src_deg - 1; i++) {
            uint e = rp[src] + i;
            ds.appendf("@min(%d + ", wt[e]);
            append_node_name(&ds, ci[e]);
            ds.append_string(", ");
        }
        uint e = rp[src] + src_deg - 1;
        ds.appendf("%d + ", wt[e]);
        append_node_name(&ds, ci[e]);
        for (uint i = 0; i < src_deg - 1; i++) ds.append_string(")");
        ds.append_string("\n");
    }

    return ds;
}
