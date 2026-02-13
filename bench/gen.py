#!/usr/bin/env python3
"""Generate Bellman-Ford benchmark HVM4 files for comparing
assoc-list, radix-16 trie, and radix-32 trie at various graph sizes.

Usage: python3 bench/gen.py
Outputs: bench/bf_{assoc,trie16,trie32}_{N}.hvm4
"""
import os, math

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ---------- graph generation ----------

def lcg(seed=42):
    s = seed
    while True:
        s = (s * 1103515245 + 12345) & 0x7fffffff
        yield s

def gen_graph(n, edges_per_node=4, seed=42):
    rng = lcg(seed)
    edges = []
    seen = set()
    # Chain for connectivity
    for i in range(n - 1):
        w = next(rng) % 10 + 1
        edges.append((i, i + 1, w))
        seen.add((i, i + 1))
    # Random extra edges (deduplicated)
    attempts = n * (edges_per_node - 1) * 2
    target = n * edges_per_node
    for _ in range(attempts):
        if len(edges) >= target:
            break
        u = next(rng) % n
        v = next(rng) % n
        if u != v and (u, v) not in seen:
            w = next(rng) % 20 + 1
            edges.append((u, v, w))
            seen.add((u, v))
    return edges

def bellman_ford_py(n, edges, src=0):
    INF = 999999
    dist = [INF] * n
    dist[src] = 0
    for _ in range(n - 1):
        updated = False
        for u, v, w in edges:
            if dist[u] != INF and dist[u] + w < dist[v]:
                dist[v] = dist[u] + w
                updated = True
        if not updated:
            break
    return dist

def ceil_log(n, base):
    if n <= base:
        return 1
    d, cap = 1, base
    while cap < n:
        d += 1
        cap *= base
    return d

# ---------- HVM4 code templates ----------

APPEND_FUNC = r"""
@append = λ{
  []: λb. b;
  <>: λh. λt. λb. h <> @append(t, b)
}
"""

def fmt_edges(edges, per_line=8):
    """Format edges as HVM4 list literal, with splitting for large edge counts."""
    THRESHOLD = 3500
    CHUNK_SIZE = 2000

    if len(edges) <= THRESHOLD:
        # Single list literal
        lines = []
        for i in range(0, len(edges), per_line):
            chunk = edges[i:i+per_line]
            line = ", ".join(f"#E3{{{u},{v},{w}}}" for u, v, w in chunk)
            lines.append("  " + line)
        return "@edges = [\n" + ",\n".join(lines) + "]"

    # Split into chunks
    result_parts = []
    num_chunks = (len(edges) + CHUNK_SIZE - 1) // CHUNK_SIZE

    for chunk_idx in range(num_chunks):
        start = chunk_idx * CHUNK_SIZE
        end = min(start + CHUNK_SIZE, len(edges))
        chunk_edges = edges[start:end]

        lines = []
        for i in range(0, len(chunk_edges), per_line):
            sub_chunk = chunk_edges[i:i+per_line]
            line = ", ".join(f"#E3{{{u},{v},{w}}}" for u, v, w in sub_chunk)
            lines.append("  " + line)

        chunk_def = f"@edges_{chunk_idx} = [\n" + ",\n".join(lines) + "]"
        result_parts.append(chunk_def)

    # Build nested @append calls: @append(chunk0, @append(chunk1, chunk2))
    if num_chunks == 1:
        edges_expr = "@edges_0"
    elif num_chunks == 2:
        edges_expr = "@append(@edges_0, @edges_1)"
    else:
        # Right-associate: @append(c0, @append(c1, @append(c2, c3)))
        edges_expr = f"@edges_{num_chunks - 1}"
        for i in range(num_chunks - 2, -1, -1):
            edges_expr = f"@append(@edges_{i}, {edges_expr})"

    result_parts.append(f"@edges = {edges_expr}")
    return "\n\n".join(result_parts)

ASSOC_FUNCS = r"""
@assoc_get = λ&key. λ&def. λ{
  []: def;
  <>: λ&h. λt. λ{
    #KV: λ&k. λv. λ{0: @assoc_get(key, def, t); λn. v}(k == key)
  }(h)
}

@assoc_set = λ&key. λ&val. λ{
  []: [#KV{key, val}];
  <>: λ&h. λ&t. λ{
    #KV: λ&k. λ&v. λ{0: #KV{k, v} <> @assoc_set(key, val, t); λn. #KV{k, val} <> t}(k == key)
  }(h)
}
"""

ASSOC_RELAX = r"""
@relax_edge = λ&dist. λ{
  #E3: λ&u. λ&v. λw.
    ! &du = @assoc_get(u, @INF, dist);
    ! &new_d = du + w;
    ! &dv = @assoc_get(v, @INF, dist);
    λ{0: dist; λn. @assoc_set(v, new_d, dist)}(new_d < dv)
}
"""

TRIE16_FUNCS = r"""
@trie_get = λ&key. λ&depth. λ{
  #HE: @INF;
  #HL: λval. val;
  #H: λc0.λc1.λc2.λc3.λc4.λc5.λc6.λc7.λc8.λc9.λc10.λc11.λc12.λc13.λc14.λc15.
    ! &slot = key % 16;
    ! &next = key / 16;
    ! &nd = depth - 1;
    λ{
      0:  @trie_get(next,nd,c0);  1:  @trie_get(next,nd,c1);
      2:  @trie_get(next,nd,c2);  3:  @trie_get(next,nd,c3);
      4:  @trie_get(next,nd,c4);  5:  @trie_get(next,nd,c5);
      6:  @trie_get(next,nd,c6);  7:  @trie_get(next,nd,c7);
      8:  @trie_get(next,nd,c8);  9:  @trie_get(next,nd,c9);
      10: @trie_get(next,nd,c10); 11: @trie_get(next,nd,c11);
      12: @trie_get(next,nd,c12); 13: @trie_get(next,nd,c13);
      14: @trie_get(next,nd,c14); λn. @trie_get(next,nd,c15)
    }(slot)
}

@trie_set = λ&key. λ&val. λ&depth. λ{
  #HL: λold. #HL{val};
  #HE: λ{
    0: #HL{val};
    λn.
      ! &slot = key % 16;
      ! &next = key / 16;
      ! &nd = depth - 1;
      ! &leaf = @trie_set(next, val, nd, #HE{});
      @trie_set_slot(slot, leaf)
  }(depth);
  #H: λ&c0.λ&c1.λ&c2.λ&c3.λ&c4.λ&c5.λ&c6.λ&c7.λ&c8.λ&c9.λ&c10.λ&c11.λ&c12.λ&c13.λ&c14.λ&c15.
    ! &slot = key % 16;
    ! &next = key / 16;
    ! &nd = depth - 1;
    λ{
      0:  #H{@trie_set(next,val,nd,c0),c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15};
      1:  #H{c0,@trie_set(next,val,nd,c1),c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15};
      2:  #H{c0,c1,@trie_set(next,val,nd,c2),c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15};
      3:  #H{c0,c1,c2,@trie_set(next,val,nd,c3),c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15};
      4:  #H{c0,c1,c2,c3,@trie_set(next,val,nd,c4),c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15};
      5:  #H{c0,c1,c2,c3,c4,@trie_set(next,val,nd,c5),c6,c7,c8,c9,c10,c11,c12,c13,c14,c15};
      6:  #H{c0,c1,c2,c3,c4,c5,@trie_set(next,val,nd,c6),c7,c8,c9,c10,c11,c12,c13,c14,c15};
      7:  #H{c0,c1,c2,c3,c4,c5,c6,@trie_set(next,val,nd,c7),c8,c9,c10,c11,c12,c13,c14,c15};
      8:  #H{c0,c1,c2,c3,c4,c5,c6,c7,@trie_set(next,val,nd,c8),c9,c10,c11,c12,c13,c14,c15};
      9:  #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,@trie_set(next,val,nd,c9),c10,c11,c12,c13,c14,c15};
      10: #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,@trie_set(next,val,nd,c10),c11,c12,c13,c14,c15};
      11: #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,@trie_set(next,val,nd,c11),c12,c13,c14,c15};
      12: #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,@trie_set(next,val,nd,c12),c13,c14,c15};
      13: #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,@trie_set(next,val,nd,c13),c14,c15};
      14: #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,@trie_set(next,val,nd,c14),c15};
      λn. #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,@trie_set(next,val,nd,c15)}
    }(slot)
}

@trie_set_slot = λ&slot. λ&child. λ{
  0:  #H{child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  1:  #H{#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  2:  #H{#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  3:  #H{#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  4:  #H{#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  5:  #H{#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  6:  #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  7:  #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  8:  #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  9:  #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  10: #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{}};
  11: #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{}};
  12: #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{}};
  13: #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{}};
  14: #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{}};
  λn. #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child}
}(slot)
"""

TRIE16_RELAX = r"""
@relax_edge = λ&dist. λ{
  #E3: λ&u. λ&v. λw.
    ! &du = @trie_get(u, @DEPTH, dist);
    ! &new_d = du + w;
    ! &dv = @trie_get(v, @DEPTH, dist);
    λ{0: dist; λn. @trie_set(v, new_d, @DEPTH, dist)}(new_d < dv)
}
"""

TRIE32_FUNCS = r"""
@trie_set_slot = λ&slot. λ&child. λ{
  0:  #H{child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  1:  #H{#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  2:  #H{#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  3:  #H{#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  4:  #H{#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  5:  #H{#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  6:  #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  7:  #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  8:  #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  9:  #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{},#HE{}};
  10: #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{},#HE{}};
  11: #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{},#HE{}};
  12: #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{},#HE{}};
  13: #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{},#HE{}};
  14: #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child,#HE{}};
  λn. #H{#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},#HE{},child}
}(slot)

@trie32_get = λ&key. λ&depth. λ{
  #HE: @INF;
  #HL: λval. val;
  #H32: λ&lo. λ&hi.
    ! &slot = key % 32;
    ! &next = key / 32;
    ! &nd = depth - 1;
    λ{0:
      @trie32_get_h(next, nd, slot - 16, hi);
    λn.
      @trie32_get_h(next, nd, slot, lo)
    }(slot < 16)
}

@trie32_get_h = λ&next. λ&nd. λ&slot. λ{
  #HE: @INF;
  #H: λc0.λc1.λc2.λc3.λc4.λc5.λc6.λc7.λc8.λc9.λc10.λc11.λc12.λc13.λc14.λc15.
    λ{
      0:  @trie32_get(next,nd,c0);  1:  @trie32_get(next,nd,c1);
      2:  @trie32_get(next,nd,c2);  3:  @trie32_get(next,nd,c3);
      4:  @trie32_get(next,nd,c4);  5:  @trie32_get(next,nd,c5);
      6:  @trie32_get(next,nd,c6);  7:  @trie32_get(next,nd,c7);
      8:  @trie32_get(next,nd,c8);  9:  @trie32_get(next,nd,c9);
      10: @trie32_get(next,nd,c10); 11: @trie32_get(next,nd,c11);
      12: @trie32_get(next,nd,c12); 13: @trie32_get(next,nd,c13);
      14: @trie32_get(next,nd,c14); λn. @trie32_get(next,nd,c15)
    }(slot)
}

@trie32_set = λ&key. λ&val. λ&depth. λ{
  #HL: λold. #HL{val};
  #HE: λ{
    0: #HL{val};
    λn.
      ! &slot = key % 32;
      ! &next = key / 32;
      ! &nd = depth - 1;
      ! &leaf = @trie32_set(next, val, nd, #HE{});
      ! &lo_slot = slot % 16;
      ! &inner = @trie_set_slot(lo_slot, leaf);
      λ{0:
        #H32{#HE{}, inner};
      λn.
        #H32{inner, #HE{}}
      }(slot < 16)
  }(depth);
  #H32: λ&lo. λ&hi.
    ! &slot = key % 32;
    ! &next = key / 32;
    ! &nd = depth - 1;
    λ{0:
      #H32{lo, @trie32_set_h(next, val, nd, slot - 16, hi)};
    λn.
      #H32{@trie32_set_h(next, val, nd, slot, lo), hi}
    }(slot < 16)
}

@trie32_set_h = λ&next. λ&val. λ&nd. λ&slot. λ{
  #HE: @trie_set_slot(slot, @trie32_set(next, val, nd, #HE{}));
  #H: λ&c0.λ&c1.λ&c2.λ&c3.λ&c4.λ&c5.λ&c6.λ&c7.λ&c8.λ&c9.λ&c10.λ&c11.λ&c12.λ&c13.λ&c14.λ&c15.
    λ{
      0:  #H{@trie32_set(next,val,nd,c0),c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15};
      1:  #H{c0,@trie32_set(next,val,nd,c1),c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15};
      2:  #H{c0,c1,@trie32_set(next,val,nd,c2),c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15};
      3:  #H{c0,c1,c2,@trie32_set(next,val,nd,c3),c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15};
      4:  #H{c0,c1,c2,c3,@trie32_set(next,val,nd,c4),c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15};
      5:  #H{c0,c1,c2,c3,c4,@trie32_set(next,val,nd,c5),c6,c7,c8,c9,c10,c11,c12,c13,c14,c15};
      6:  #H{c0,c1,c2,c3,c4,c5,@trie32_set(next,val,nd,c6),c7,c8,c9,c10,c11,c12,c13,c14,c15};
      7:  #H{c0,c1,c2,c3,c4,c5,c6,@trie32_set(next,val,nd,c7),c8,c9,c10,c11,c12,c13,c14,c15};
      8:  #H{c0,c1,c2,c3,c4,c5,c6,c7,@trie32_set(next,val,nd,c8),c9,c10,c11,c12,c13,c14,c15};
      9:  #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,@trie32_set(next,val,nd,c9),c10,c11,c12,c13,c14,c15};
      10: #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,@trie32_set(next,val,nd,c10),c11,c12,c13,c14,c15};
      11: #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,@trie32_set(next,val,nd,c11),c12,c13,c14,c15};
      12: #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,@trie32_set(next,val,nd,c12),c13,c14,c15};
      13: #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,@trie32_set(next,val,nd,c13),c14,c15};
      14: #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,@trie32_set(next,val,nd,c14),c15};
      λn. #H{c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,@trie32_set(next,val,nd,c15)}
    }(slot)
}
"""

TRIE32_RELAX = r"""
@relax_edge = λ&dist. λ{
  #E3: λ&u. λ&v. λw.
    ! &du = @trie32_get(u, @DEPTH, dist);
    ! &new_d = du + w;
    ! &dv = @trie32_get(v, @DEPTH, dist);
    λ{0: dist; λn. @trie32_set(v, new_d, @DEPTH, dist)}(new_d < dv)
}
"""

BTRIE_FUNCS = r"""
@btrie_get_lin = λ&key. λ&depth. λ{
  #BE: #P{@INF, #BE{}};
  #BL: λ&val. #P{val, #BL{val}};
  #B: λl. λr.
    ! bit = key % 2;
    ! next = key / 2;
    ! nd = depth - 1;
    @btrie_get_lin_B(bit, next, nd, l, r)
}

@btrie_get_lin_B = λ{
  0: λnext. λnd. λl. λr.
    λ{#P: λval. λnew_l. #P{val, #B{new_l, r}}}(@btrie_get_lin(next, nd, l));
  λn. λnext. λnd. λl. λr.
    λ{#P: λval. λnew_r. #P{val, #B{l, new_r}}}(@btrie_get_lin(next, nd, r))
}

@btrie_get = λ&key. λ&depth. λ{
  #BE: @INF;
  #BL: λval. val;
  #B: λl. λr.
    ! bit = key % 2;
    ! next = key / 2;
    ! nd = depth - 1;
    @btrie_get_B(bit, next, nd, l, r)
}

@btrie_get_B = λ{
  0: λnext. λnd. λl. λr. @btrie_get(next, nd, l);
  λn. λnext. λnd. λl. λr. @btrie_get(next, nd, r)
}

@btrie_set = λ&key. λ&val. λ&depth. λ{
  #BL: λold. #BL{val};
  #BE: λ{
    0: #BL{val};
    λn.
      ! &bit = key % 2;
      ! &next = key / 2;
      ! &nd = depth - 1;
      @btrie_set_BE(bit, next, val, nd)
  }(depth);
  #B: λl. λr.
    ! bit = key % 2;
    ! next = key / 2;
    ! nd = depth - 1;
    @btrie_set_B(bit, next, val, nd, l, r)
}

@btrie_set_BE = λ{
  0: λnext. λval. λnd. #B{@btrie_set(next, val, nd, #BE{}), #BE{}};
  λn. λnext. λval. λnd. #B{#BE{}, @btrie_set(next, val, nd, #BE{})}
}

@btrie_set_B = λ{
  0: λnext. λval. λnd. λl. λr. #B{@btrie_set(next, val, nd, l), r};
  λn. λnext. λval. λnd. λl. λr. #B{l, @btrie_set(next, val, nd, r)}
}

@btrie_min_update = λ&key. λ&val. λ&depth. λ{
  #BL: λ&old. λ{0: #BL{old}; λn. #BL{val}}(val < old);
  #BE: λ{
    0: #BL{val};
    λn.
      ! &bit = key % 2;
      ! &next = key / 2;
      ! &nd = depth - 1;
      @btrie_mu_BE(bit, next, val, nd)
  }(depth);
  #B: λl. λr.
    ! bit = key % 2;
    ! next = key / 2;
    ! nd = depth - 1;
    @btrie_mu_B(bit, next, val, nd, l, r)
}

@btrie_mu_BE = λ{
  0: λnext. λval. λnd. #B{@btrie_min_update(next, val, nd, #BE{}), #BE{}};
  λn. λnext. λval. λnd. #B{#BE{}, @btrie_min_update(next, val, nd, #BE{})}
}

@btrie_mu_B = λ{
  0: λnext. λval. λnd. λl. λr. #B{@btrie_min_update(next, val, nd, l), r};
  λn. λnext. λval. λnd. λl. λr. #B{l, @btrie_min_update(next, val, nd, r)}
}
"""

BTRIE_RELAX = r"""
@relax_edge_lin = λdist. λ{
  #E3: λu. λv. λw.
    λ{#P: λ&du. λdist2.
      ! new_d = du + w;
      @relax_cond(du < @INF, v, new_d, dist2)
    }(@btrie_get_lin(u, @DEPTH, dist))
}

@relax_cond = λ{
  0: λv. λnew_d. λdist. dist;
  λn. λv. λnew_d. λdist. @btrie_min_update(v, new_d, @DEPTH, dist)
}
"""

BTRIE_COMMON = r"""
@foldl_lin = λf. λacc. λlist. @foldl_go(list, f, acc)

@foldl_go = λ{
  []: λf. λacc. acc;
  <>: λh. λt. λ&f. λacc. @foldl_go(t, f, f(acc, h))
}

@relax_round_lin = λdist. @foldl_lin(@relax_edge_lin, dist, @edges)

@repeat_lin = λf. λx. λn. @repeat_go(n, f, x)

@repeat_go = λ{
  0: λf. λx. x;
  λn. λ&f. λx. @repeat_go(n - 1, f, f(x))
}
"""

# ---------- Early termination btrie ----------

BTRIE_ET_FUNCS = BTRIE_FUNCS + r"""
@btrie_min_update_f = λ&key. λ&val. λ&depth. λ{
  #BL: λ&old. λ{0: #P{#BL{old}, 0}; λn. #P{#BL{val}, 1}}(val < old);
  #BE: λ{
    0: #P{#BL{val}, 1};
    λn.
      ! &bit = key % 2;
      ! &next = key / 2;
      ! &nd = depth - 1;
      @btrie_muf_BE(bit, next, val, nd)
  }(depth);
  #B: λl. λr.
    ! bit = key % 2;
    ! next = key / 2;
    ! nd = depth - 1;
    @btrie_muf_B(bit, next, val, nd, l, r)
}

@btrie_muf_BE = λ{
  0: λnext. λval. λnd.
    λ{#P: λchild. λc. #P{#B{child, #BE{}}, c}}(@btrie_min_update_f(next, val, nd, #BE{}));
  λn. λnext. λval. λnd.
    λ{#P: λchild. λc. #P{#B{#BE{}, child}, c}}(@btrie_min_update_f(next, val, nd, #BE{}))
}

@btrie_muf_B = λ{
  0: λnext. λval. λnd. λl. λr.
    λ{#P: λnew_l. λc. #P{#B{new_l, r}, c}}(@btrie_min_update_f(next, val, nd, l));
  λn. λnext. λval. λnd. λl. λr.
    λ{#P: λnew_r. λc. #P{#B{l, new_r}, c}}(@btrie_min_update_f(next, val, nd, r))
}
"""

BTRIE_ET_RELAX = r"""
@relax_edge_et = λ{
  #S: λdist. λ&changed. λ{
    #E3: λu. λv. λw.
      λ{#P: λ&du. λdist2.
        ! new_d = du + w;
        @relax_cond_et(du < @INF, v, new_d, dist2, changed)
      }(@btrie_get_lin(u, @DEPTH, dist))
  }
}

@relax_cond_et = λ{
  0: λv. λnew_d. λdist. λchanged. #S{dist, changed};
  λn. λv. λnew_d. λdist. λ&changed.
    λ{#P: λnew_dist. λc. #S{new_dist, changed + c}}(@btrie_min_update_f(v, new_d, @DEPTH, dist))
}
"""

BTRIE_ET_COMMON = r"""
@foldl_et = λf. λacc. λlist. @foldl_et_go(list, f, acc)

@foldl_et_go = λ{
  []: λf. λacc. acc;
  <>: λh. λt. λ&f. λacc. @foldl_et_go(t, f, f(acc, h))
}

@relax_round_et = λ{
  #S: λdist. λold_changed.
    @foldl_et(@relax_edge_et, #S{dist, 0}, @edges)
}

@repeat_until = λf. λx. λn. @repeat_until_go(n, f, x)

@repeat_until_go = λ{
  0: λf. λx. x;
  λn. λ&f. λstate.
    @check_continue(n, f, f(state))
}

@check_continue = λ&n. λ&f. λ{
  #S: λdist. λchanged.
    @check_go(changed, n, f, dist)
}

@check_go = λ{
  0: λn. λf. λdist. #S{dist, 0};
  λm. λn. λf. λdist. @repeat_until_go(n - 1, f, #S{dist, 1})
}
"""

COMMON_FUNCS = r"""
@min = λ&a. λ&b. λ{0: b; λn. a}(a < b)

@foldl = λ&f. λ&acc. λ{[]: acc; <>: λh. λt. @foldl(f, f(acc, h), t)}

@relax_round = λdist. @foldl(@relax_edge, dist, @edges)

@repeat = λ&f. λ&x. λ{0: x; λn. @repeat(f, f(x), n - 1)}
"""

# ---------- file generation ----------

def gen_assoc_file(n, edges, dist):
    edges_str = fmt_edges(edges)
    needs_append = len(edges) > 3500
    append_func = APPEND_FUNC if needs_append else ""

    return f"""// Bellman-Ford SSSP — assoc-list — V={n}, E={len(edges)}
// Expected: dist[{n-1}] = {dist[n-1]}

@INF = 999999
{append_func}{ASSOC_FUNCS}
{edges_str}
{ASSOC_RELAX}
{COMMON_FUNCS}
@init_dist = [#KV{{0, 0}}]

@bf = @repeat(@relax_round, @init_dist, {n-1})

@main = @assoc_get({n-1}, @INF, @bf)
//{dist[n-1]}
"""

def gen_trie16_file(n, edges, dist):
    depth = ceil_log(n, 16)
    edges_str = fmt_edges(edges)
    needs_append = len(edges) > 3500
    append_func = APPEND_FUNC if needs_append else ""

    return f"""// Bellman-Ford SSSP — radix-16 trie — V={n}, E={len(edges)}, depth={depth}
// Expected: dist[{n-1}] = {dist[n-1]}

@INF = 999999
@DEPTH = {depth}
{append_func}{TRIE16_FUNCS}
{edges_str}
{TRIE16_RELAX}
{COMMON_FUNCS}
@init_dist = @trie_set(0, 0, @DEPTH, #HE{{}})

@bf = @repeat(@relax_round, @init_dist, {n-1})

@main = @trie_get({n-1}, @DEPTH, @bf)
//{dist[n-1]}
"""

def gen_trie32_file(n, edges, dist):
    depth = ceil_log(n, 32)
    edges_str = fmt_edges(edges)
    needs_append = len(edges) > 3500
    append_func = APPEND_FUNC if needs_append else ""

    return f"""// Bellman-Ford SSSP — radix-32 trie — V={n}, E={len(edges)}, depth={depth}
// Expected: dist[{n-1}] = {dist[n-1]}

@INF = 999999
@DEPTH = {depth}
{append_func}{TRIE32_FUNCS}
{edges_str}
{TRIE32_RELAX}
{COMMON_FUNCS}
@init_dist = @trie32_set(0, 0, @DEPTH, #HE{{}})

@bf = @repeat(@relax_round, @init_dist, {n-1})

@main = @trie32_get({n-1}, @DEPTH, @bf)
//{dist[n-1]}
"""

def gen_btrie_file(n, edges, dist):
    depth = ceil_log(n, 2)
    edges_str = fmt_edges(edges)
    needs_append = len(edges) > 3500
    append_func = APPEND_FUNC if needs_append else ""

    return f"""// Bellman-Ford SSSP — linear binary trie — V={n}, E={len(edges)}, depth={depth}
// Expected: dist[{n-1}] = {dist[n-1]}

@INF = 999999
@DEPTH = {depth}
{append_func}{BTRIE_FUNCS}
{edges_str}
{BTRIE_RELAX}
{BTRIE_COMMON}
@init_dist = @btrie_set(0, 0, @DEPTH, #BE{{}})

@bf = @repeat_lin(@relax_round_lin, @init_dist, {n-1})

@main = @btrie_get({n-1}, @DEPTH, @bf)
//{dist[n-1]}
"""

def gen_btrie_et_file(n, edges, dist):
    depth = ceil_log(n, 2)
    edges_str = fmt_edges(edges)
    needs_append = len(edges) > 3500
    append_func = APPEND_FUNC if needs_append else ""

    return f"""// Bellman-Ford SSSP — linear binary trie + early termination — V={n}, E={len(edges)}, depth={depth}
// Expected: dist[{n-1}] = {dist[n-1]}

@INF = 999999
@DEPTH = {depth}
{append_func}{BTRIE_ET_FUNCS}
{edges_str}
{BTRIE_ET_RELAX}
{BTRIE_ET_COMMON}
@init_dist = @btrie_set(0, 0, @DEPTH, #BE{{}})
@init_state = #S{{@init_dist, 1}}

@bf = @repeat_until(@relax_round_et, @init_state, {n-1})

@extract = λ{{#S: λdist. λc.
  @btrie_get({n-1}, @DEPTH, dist)
}}

@main = @extract(@bf)
//{dist[n-1]}
"""

# ---------- Radix-4 trie with early termination ----------

Q4_ET_FUNCS = r"""
@q4_get_lin = λ&key. λ&depth. λ{
  #QE: #P{@INF, #QE{}};
  #QL: λ&val. #P{val, #QL{val}};
  #Q: λc0. λc1. λc2. λc3.
    ! slot = key % 4;
    ! next = key / 4;
    ! nd = depth - 1;
    @q4_get_lin_Q(slot, next, nd, c0, c1, c2, c3)
}

@q4_get_lin_Q = λ{
  0: λnext. λnd. λc0. λc1. λc2. λc3.
    λ{#P: λval. λnew_c0. #P{val, #Q{new_c0, c1, c2, c3}}}(@q4_get_lin(next, nd, c0));
  1: λnext. λnd. λc0. λc1. λc2. λc3.
    λ{#P: λval. λnew_c1. #P{val, #Q{c0, new_c1, c2, c3}}}(@q4_get_lin(next, nd, c1));
  2: λnext. λnd. λc0. λc1. λc2. λc3.
    λ{#P: λval. λnew_c2. #P{val, #Q{c0, c1, new_c2, c3}}}(@q4_get_lin(next, nd, c2));
  λn. λnext. λnd. λc0. λc1. λc2. λc3.
    λ{#P: λval. λnew_c3. #P{val, #Q{c0, c1, c2, new_c3}}}(@q4_get_lin(next, nd, c3))
}

@q4_get = λ&key. λ&depth. λ{
  #QE: @INF;
  #QL: λval. val;
  #Q: λc0. λc1. λc2. λc3.
    ! slot = key % 4;
    ! next = key / 4;
    ! nd = depth - 1;
    @q4_get_Q(slot, next, nd, c0, c1, c2, c3)
}

@q4_get_Q = λ{
  0: λnext. λnd. λc0. λc1. λc2. λc3. @q4_get(next, nd, c0);
  1: λnext. λnd. λc0. λc1. λc2. λc3. @q4_get(next, nd, c1);
  2: λnext. λnd. λc0. λc1. λc2. λc3. @q4_get(next, nd, c2);
  λn. λnext. λnd. λc0. λc1. λc2. λc3. @q4_get(next, nd, c3)
}

@q4_set = λ&key. λ&val. λ&depth. λ{
  #QL: λold. #QL{val};
  #QE: λ{
    0: #QL{val};
    λn.
      ! slot = key % 4;
      ! next = key / 4;
      ! nd = depth - 1;
      @q4_set_QE(slot, next, val, nd)
  }(depth);
  #Q: λc0. λc1. λc2. λc3.
    ! slot = key % 4;
    ! next = key / 4;
    ! nd = depth - 1;
    @q4_set_Q(slot, next, val, nd, c0, c1, c2, c3)
}

@q4_set_QE = λ{
  0: λnext. λval. λnd. #Q{@q4_set(next, val, nd, #QE{}), #QE{}, #QE{}, #QE{}};
  1: λnext. λval. λnd. #Q{#QE{}, @q4_set(next, val, nd, #QE{}), #QE{}, #QE{}};
  2: λnext. λval. λnd. #Q{#QE{}, #QE{}, @q4_set(next, val, nd, #QE{}), #QE{}};
  λn. λnext. λval. λnd. #Q{#QE{}, #QE{}, #QE{}, @q4_set(next, val, nd, #QE{})}
}

@q4_set_Q = λ{
  0: λnext. λval. λnd. λc0. λc1. λc2. λc3. #Q{@q4_set(next, val, nd, c0), c1, c2, c3};
  1: λnext. λval. λnd. λc0. λc1. λc2. λc3. #Q{c0, @q4_set(next, val, nd, c1), c2, c3};
  2: λnext. λval. λnd. λc0. λc1. λc2. λc3. #Q{c0, c1, @q4_set(next, val, nd, c2), c3};
  λn. λnext. λval. λnd. λc0. λc1. λc2. λc3. #Q{c0, c1, c2, @q4_set(next, val, nd, c3)}
}

@q4_min_update_f = λ&key. λ&val. λ&depth. λ{
  #QL: λ&old. λ{0: #P{#QL{old}, 0}; λn. #P{#QL{val}, 1}}(val < old);
  #QE: λ{
    0: #P{#QL{val}, 1};
    λn.
      ! slot = key % 4;
      ! next = key / 4;
      ! nd = depth - 1;
      @q4_muf_QE(slot, next, val, nd)
  }(depth);
  #Q: λc0. λc1. λc2. λc3.
    ! slot = key % 4;
    ! next = key / 4;
    ! nd = depth - 1;
    @q4_muf_Q(slot, next, val, nd, c0, c1, c2, c3)
}

@q4_muf_QE = λ{
  0: λnext. λval. λnd.
    λ{#P: λchild. λc. #P{#Q{child, #QE{}, #QE{}, #QE{}}, c}}(@q4_min_update_f(next, val, nd, #QE{}));
  1: λnext. λval. λnd.
    λ{#P: λchild. λc. #P{#Q{#QE{}, child, #QE{}, #QE{}}, c}}(@q4_min_update_f(next, val, nd, #QE{}));
  2: λnext. λval. λnd.
    λ{#P: λchild. λc. #P{#Q{#QE{}, #QE{}, child, #QE{}}, c}}(@q4_min_update_f(next, val, nd, #QE{}));
  λn. λnext. λval. λnd.
    λ{#P: λchild. λc. #P{#Q{#QE{}, #QE{}, #QE{}, child}, c}}(@q4_min_update_f(next, val, nd, #QE{}))
}

@q4_muf_Q = λ{
  0: λnext. λval. λnd. λc0. λc1. λc2. λc3.
    λ{#P: λnew_c0. λc. #P{#Q{new_c0, c1, c2, c3}, c}}(@q4_min_update_f(next, val, nd, c0));
  1: λnext. λval. λnd. λc0. λc1. λc2. λc3.
    λ{#P: λnew_c1. λc. #P{#Q{c0, new_c1, c2, c3}, c}}(@q4_min_update_f(next, val, nd, c1));
  2: λnext. λval. λnd. λc0. λc1. λc2. λc3.
    λ{#P: λnew_c2. λc. #P{#Q{c0, c1, new_c2, c3}, c}}(@q4_min_update_f(next, val, nd, c2));
  λn. λnext. λval. λnd. λc0. λc1. λc2. λc3.
    λ{#P: λnew_c3. λc. #P{#Q{c0, c1, c2, new_c3}, c}}(@q4_min_update_f(next, val, nd, c3))
}
"""

Q4_ET_RELAX = r"""
@relax_edge_et = λ{
  #S: λdist. λ&changed. λ{
    #E3: λu. λv. λw.
      λ{#P: λ&du. λdist2.
        ! new_d = du + w;
        @relax_cond_et(du < @INF, v, new_d, dist2, changed)
      }(@q4_get_lin(u, @DEPTH, dist))
  }
}

@relax_cond_et = λ{
  0: λv. λnew_d. λdist. λchanged. #S{dist, changed};
  λn. λv. λnew_d. λdist. λ&changed.
    λ{#P: λnew_dist. λc. #S{new_dist, changed + c}}(@q4_min_update_f(v, new_d, @DEPTH, dist))
}
"""

Q4_ET_COMMON = r"""
@foldl_et = λf. λacc. λlist. @foldl_et_go(list, f, acc)

@foldl_et_go = λ{
  []: λf. λacc. acc;
  <>: λh. λt. λ&f. λacc. @foldl_et_go(t, f, f(acc, h))
}

@relax_round_et = λ{
  #S: λdist. λold_changed.
    @foldl_et(@relax_edge_et, #S{dist, 0}, @edges)
}

@repeat_until = λf. λx. λn. @repeat_until_go(n, f, x)

@repeat_until_go = λ{
  0: λf. λx. x;
  λn. λ&f. λstate.
    @check_continue(n, f, f(state))
}

@check_continue = λ&n. λ&f. λ{
  #S: λdist. λchanged.
    @check_go(changed, n, f, dist)
}

@check_go = λ{
  0: λn. λf. λdist. #S{dist, 0};
  λm. λn. λf. λdist. @repeat_until_go(n - 1, f, #S{dist, 1})
}
"""

DELTA_STEP_ET_ROUND = r"""
@one_round_et = λ{#S: λdist. λold_changed.
  @one_round_et_2(@foldl_et(@relax_edge_et, #S{dist, 0}, @light_edges))
}
@one_round_et_2 = λ{#S: λdist. λ&c1.
  @one_round_et_3(c1, @foldl_et(@relax_edge_et, #S{dist, 0}, @light_edges))
}
@one_round_et_3 = λ&c1. λ{#S: λdist. λ&c2.
  λ{#S: λdist2. λ&c3. #S{dist2, c1 + c2 + c3}
  }(@foldl_et(@relax_edge_et, #S{dist, 0}, @heavy_edges))
}
"""

def fmt_edge_list(edges, name, per_line=8):
    """Format a named edge list, with splitting for large lists."""
    THRESHOLD = 3500
    CHUNK_SIZE = 2000

    if len(edges) <= THRESHOLD:
        lines = []
        for i in range(0, len(edges), per_line):
            chunk = edges[i:i+per_line]
            line = ", ".join(f"#E3{{{u},{v},{w}}}" for u, v, w in chunk)
            lines.append("  " + line)
        return f"@{name} = [\n" + ",\n".join(lines) + "]"

    result_parts = []
    num_chunks = (len(edges) + CHUNK_SIZE - 1) // CHUNK_SIZE
    for chunk_idx in range(num_chunks):
        start = chunk_idx * CHUNK_SIZE
        end = min(start + CHUNK_SIZE, len(edges))
        chunk_edges = edges[start:end]
        lines = []
        for i in range(0, len(chunk_edges), per_line):
            sub_chunk = chunk_edges[i:i+per_line]
            line = ", ".join(f"#E3{{{u},{v},{w}}}" for u, v, w in sub_chunk)
            lines.append("  " + line)
        chunk_def = f"@{name}_{chunk_idx} = [\n" + ",\n".join(lines) + "]"
        result_parts.append(chunk_def)

    if num_chunks == 1:
        edges_expr = f"@{name}_0"
    elif num_chunks == 2:
        edges_expr = f"@append(@{name}_0, @{name}_1)"
    else:
        edges_expr = f"@{name}_{num_chunks - 1}"
        for i in range(num_chunks - 2, -1, -1):
            edges_expr = f"@append(@{name}_{i}, {edges_expr})"

    result_parts.append(f"@{name} = {edges_expr}")
    return "\n\n".join(result_parts)

def delta_step_py(n, edges, delta, src=0):
    """Run delta-stepping in Python to get expected distances."""
    INF = 999999
    dist = [INF] * n
    dist[src] = 0
    light = [(u,v,w) for u,v,w in edges if w <= delta]
    heavy = [(u,v,w) for u,v,w in edges if w > delta]
    for _ in range(n - 1):
        changed = False
        # light twice
        for _ in range(2):
            for u,v,w in light:
                if dist[u] != INF and dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    changed = True
        # heavy once
        for u,v,w in heavy:
            if dist[u] != INF and dist[u] + w < dist[v]:
                dist[v] = dist[u] + w
                changed = True
        if not changed:
            break
    return dist

def gen_q4_et_file(n, edges, dist):
    depth = ceil_log(n, 4)
    edges_str = fmt_edges(edges)
    needs_append = len(edges) > 3500
    append_func = APPEND_FUNC if needs_append else ""

    return f"""// Bellman-Ford SSSP — linear radix-4 trie + early termination — V={n}, E={len(edges)}, depth={depth}
// Expected: dist[{n-1}] = {dist[n-1]}

@INF = 999999
@DEPTH = {depth}
{append_func}{Q4_ET_FUNCS}
{edges_str}
{Q4_ET_RELAX}
{Q4_ET_COMMON}
@init_dist = @q4_set(0, 0, @DEPTH, #QE{{}})
@init_state = #S{{@init_dist, 1}}

@bf = @repeat_until(@relax_round_et, @init_state, {n-1})

@extract = λ{{#S: λdist. λc.
  @q4_get({n-1}, @DEPTH, dist)
}}

@main = @extract(@bf)
//{dist[n-1]}
"""

def gen_q4_adj_et_file(n, edges, dist):
    depth = ceil_log(n, 4)
    # Group edges by source node into adjacency list
    adj = {}
    for u, v, w in edges:
        adj.setdefault(u, []).append((v, w))
    needs_append = False  # adjacency lists are smaller per entry
    # Format adjacency list
    adj_entries = []
    for u in sorted(adj.keys()):
        out_edges = adj[u]
        edge_strs = [f"#E2{{{v},{w}}}" for v, w in out_edges]
        # Split long outgoing edge lists
        inner_lines = []
        for i in range(0, len(edge_strs), 10):
            inner_lines.append("    " + ", ".join(edge_strs[i:i+10]))
        inner = ",\n".join(inner_lines)
        adj_entries.append(f"  #N{{{u}, [\n{inner}]}}")

    CHUNK = 3000
    if len(adj_entries) <= CHUNK:
        adj_str = "@adj = [\n" + ",\n".join(adj_entries) + "]"
        append_func = ""
    else:
        # Split adjacency list into chunks
        chunks = []
        for i in range(0, len(adj_entries), CHUNK):
            chunk = adj_entries[i:i+CHUNK]
            chunks.append(f"@adj_{len(chunks)} = [\n" + ",\n".join(chunk) + "]")
        if len(chunks) == 2:
            expr = "@append(@adj_0, @adj_1)"
        else:
            expr = f"@adj_{len(chunks)-1}"
            for i in range(len(chunks)-2, -1, -1):
                expr = f"@append(@adj_{i}, {expr})"
        chunks.append(f"@adj = {expr}")
        adj_str = "\n\n".join(chunks)
        append_func = APPEND_FUNC

    return f"""// Bellman-Ford SSSP — radix-4 trie + adjacency list + early termination — V={n}, E={len(edges)}, depth={depth}
// Adjacency list: {len(adj)} source nodes (V get_lin calls per round instead of E={len(edges)})
// Expected: dist[{n-1}] = {dist[n-1]}

@INF = 999999
@DEPTH = {depth}
{append_func}{Q4_ET_FUNCS}

{adj_str}

@relax_node_et = λ{{
  #S: λdist. λ&changed. λ{{
    #N: λu. λout.
      λ{{#P: λ&du. λdist2.
        @relax_node_go(du < @INF, du, out, dist2, changed)
      }}(@q4_get_lin(u, @DEPTH, dist))
  }}
}}

@relax_node_go = λ{{
  0: λdu. λout. λdist. λchanged. #S{{dist, changed}};
  λn. λ&du. λout. λdist. λchanged.
    @foldl_inner(@relax_out(du), #S{{dist, changed}}, out)
}}

@relax_out = λ&du. λ{{
  #S: λdist. λ&changed. λ{{
    #E2: λv. λw.
      ! new_d = du + w;
      λ{{#P: λnew_dist. λc. #S{{new_dist, changed + c}}}}(@q4_min_update_f(v, new_d, @DEPTH, dist))
  }}
}}

@foldl_inner = λf. λacc. λlist. @foldl_inner_go(list, f, acc)

@foldl_inner_go = λ{{
  []: λf. λacc. acc;
  <>: λh. λt. λ&f. λacc. @foldl_inner_go(t, f, f(acc, h))
}}

@foldl_et = λf. λacc. λlist. @foldl_et_go(list, f, acc)

@foldl_et_go = λ{{
  []: λf. λacc. acc;
  <>: λh. λt. λ&f. λacc. @foldl_et_go(t, f, f(acc, h))
}}

@relax_round_et = λ{{
  #S: λdist. λold_changed.
    @foldl_et(@relax_node_et, #S{{dist, 0}}, @adj)
}}

@repeat_until = λf. λx. λn. @repeat_until_go(n, f, x)

@repeat_until_go = λ{{
  0: λf. λx. x;
  λn. λ&f. λstate.
    @check_continue(n, f, f(state))
}}

@check_continue = λ&n. λ&f. λ{{
  #S: λdist. λchanged.
    @check_go(changed, n, f, dist)
}}

@check_go = λ{{
  0: λn. λf. λdist. #S{{dist, 0}};
  λm. λn. λf. λdist. @repeat_until_go(n - 1, f, #S{{dist, 1}})
}}

@init_dist = @q4_set(0, 0, @DEPTH, #QE{{}})
@init_state = #S{{@init_dist, 1}}

@bf = @repeat_until(@relax_round_et, @init_state, {n-1})

@extract = λ{{#S: λdist. λc.
  @q4_get({n-1}, @DEPTH, dist)
}}

@main = @extract(@bf)
//{dist[n-1]}
"""

def gen_delta_step_et_file(n, edges, dist, delta):
    depth = ceil_log(n, 2)
    light = [(u,v,w) for u,v,w in edges if w <= delta]
    heavy = [(u,v,w) for u,v,w in edges if w > delta]
    needs_append = max(len(light), len(heavy)) > 3500
    append_func = APPEND_FUNC if needs_append else ""
    light_str = fmt_edge_list(light, "light_edges")
    heavy_str = fmt_edge_list(heavy, "heavy_edges")

    return f"""// Delta-Stepping SSSP — linear binary trie + early termination — V={n}, E={len(edges)}, delta={delta}, depth={depth}
// Light edges: {len(light)}, Heavy edges: {len(heavy)}
// Expected: dist[{n-1}] = {dist[n-1]}

@INF = 999999
@DEPTH = {depth}
{append_func}{BTRIE_ET_FUNCS}

{light_str}

{heavy_str}
{BTRIE_ET_RELAX}
{BTRIE_ET_COMMON}
{DELTA_STEP_ET_ROUND}
@init_dist = @btrie_set(0, 0, @DEPTH, #BE{{}})
@init_state = #S{{@init_dist, 1}}

@result = @repeat_until(@one_round_et, @init_state, {n-1})

@extract = λ{{#S: λdist. λc.
  @btrie_get({n-1}, @DEPTH, dist)
}}

@main = @extract(@result)
//{dist[n-1]}
"""

# ---------- main ----------

# Sizes to generate.
#   V=50:  r16 depth 2, r32 depth 2 — r16 wins (less per-op overhead)
#   V=100: r16 depth 2, r32 depth 2 — r16 wins
#   V=200: r16 depth 2, r32 depth 2 — r16 wins, assoc-list very slow
#   V=256: r16 depth 2, r32 depth 2 — last size before r16 depth jump
#   V=257: r16 depth 3 (OOMs!), r32 depth 2 — r32 wins by surviving!
# Key insight: at V>256, r16 depth jumps to 3, causing OOM on HVM4's
# bump allocator.  r32 stays at depth 2 up to V=1024.
# With edge list splitting, we can now handle V>1050 (E>4200).
SIZES = [50, 100, 200, 256, 257, 500, 1000, 1500, 2000, 5000, 10000]
ASSOC_MAX = 100  # skip assoc-list above this size

def main():
    generated = []
    for n in SIZES:
        edges = gen_graph(n, edges_per_node=4, seed=42 + n)
        dist = bellman_ford_py(n, edges)
        print(f"V={n:4d}  E={len(edges):5d}  "
              f"trie16_depth={ceil_log(n,16)}  trie32_depth={ceil_log(n,32)}  "
              f"btrie_depth={ceil_log(n,2)}  q4_depth={ceil_log(n,4)}  "
              f"dist[{n-1}]={dist[n-1]}")

        # trie16
        path = os.path.join(SCRIPT_DIR, f"bf_trie16_{n}.hvm4")
        with open(path, "w") as f:
            f.write(gen_trie16_file(n, edges, dist))
        generated.append(path)

        # trie32
        path = os.path.join(SCRIPT_DIR, f"bf_trie32_{n}.hvm4")
        with open(path, "w") as f:
            f.write(gen_trie32_file(n, edges, dist))
        generated.append(path)

        # btrie (linear binary trie)
        path = os.path.join(SCRIPT_DIR, f"bf_btrie_{n}.hvm4")
        with open(path, "w") as f:
            f.write(gen_btrie_file(n, edges, dist))
        generated.append(path)

        # btrie_et (linear binary trie + early termination)
        path = os.path.join(SCRIPT_DIR, f"bf_btrie_et_{n}.hvm4")
        with open(path, "w") as f:
            f.write(gen_btrie_et_file(n, edges, dist))
        generated.append(path)

        # q4_et (linear radix-4 trie + early termination)
        path = os.path.join(SCRIPT_DIR, f"bf_q4_et_{n}.hvm4")
        with open(path, "w") as f:
            f.write(gen_q4_et_file(n, edges, dist))
        generated.append(path)

        # q4_adj_et (radix-4 trie + adjacency list + early termination)
        path = os.path.join(SCRIPT_DIR, f"bf_q4_adj_et_{n}.hvm4")
        with open(path, "w") as f:
            f.write(gen_q4_adj_et_file(n, edges, dist))
        generated.append(path)

        # delta-stepping btrie+ET (light/heavy edge split)
        delta = 5  # median-ish threshold for generated graphs (weights 1-20)
        ds_dist = delta_step_py(n, edges, delta)
        path = os.path.join(SCRIPT_DIR, f"ds_btrie_et_{n}.hvm4")
        with open(path, "w") as f:
            f.write(gen_delta_step_et_file(n, edges, ds_dist, delta))
        generated.append(path)

        # assoc-list (only for small graphs)
        if n <= ASSOC_MAX:
            path = os.path.join(SCRIPT_DIR, f"bf_assoc_{n}.hvm4")
            with open(path, "w") as f:
                f.write(gen_assoc_file(n, edges, dist))
            generated.append(path)

    print(f"\nGenerated {len(generated)} files in {SCRIPT_DIR}/")

if __name__ == "__main__":
    main()
