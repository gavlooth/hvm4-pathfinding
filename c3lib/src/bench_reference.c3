module bench_reference;
import bench_common;

// ---------------------------------------------------------------------------
// Bellman-Ford with early termination
// Used by: bf, delta_step, ch
// ---------------------------------------------------------------------------

fn void bf_reference(uint n, uint* rp, uint* ci, uint* wt, uint src, uint* dist) {
    for (uint i = 0; i < n; i++) dist[i] = bench_common::INF;
    dist[src] = 0;
    for (uint round = 0; round + 1 < n; round++) {
        int changed = 0;
        for (uint u = 0; u < n; u++) {
            if (dist[u] >= bench_common::INF) continue;
            for (uint e = rp[u]; e < rp[u + 1]; e++) {
                uint nd = dist[u] + wt[e];
                if (nd < dist[ci[e]]) {
                    dist[ci[e]] = nd;
                    changed = 1;
                }
            }
        }
        if (changed == 0) break;
    }
}

// ---------------------------------------------------------------------------
// Floyd-Warshall (APSP reference)
// ---------------------------------------------------------------------------

const uint FW_INF = 999_999;

fn void floyd_warshall(uint n, uint* adj, uint* dist) {
    for (uint i = 0; i < n * n; i++) dist[i] = adj[i];
    for (uint k = 0; k < n; k++) {
        for (uint i = 0; i < n; i++) {
            for (uint j = 0; j < n; j++) {
                uint via = dist[i * n + k] + dist[k * n + j];
                if (via < dist[i * n + j]) dist[i * n + j] = via;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Warshall's boolean transitive closure
// ---------------------------------------------------------------------------

fn void warshall(uint n, uint* adj, uint* reach) {
    for (uint i = 0; i < n * n; i++) reach[i] = adj[i];
    for (uint k = 0; k < n; k++) {
        for (uint i = 0; i < n; i++) {
            for (uint j = 0; j < n; j++) {
                if (reach[i * n + k] != 0 && reach[k * n + j] != 0) {
                    reach[i * n + j] = 1;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// BFS reference (unweighted)
// ---------------------------------------------------------------------------

fn uint bfs_reference(uint n, uint* rp, uint* ci, uint src, uint dest) {
    uint* dist = mem::new_array(uint, n).ptr;
    defer free(dist);
    for (uint i = 0; i < n; i++) dist[i] = 999_999;
    dist[src] = 0;

    uint* queue = mem::new_array(uint, n).ptr;
    defer free(queue);
    uint qh = 0;
    uint qt = 0;
    queue[qt] = src;
    qt++;

    while (qh < qt) {
        uint u = queue[qh];
        qh++;
        if (u == dest) break;
        for (uint e = rp[u]; e < rp[u + 1]; e++) {
            uint v = ci[e];
            if (dist[v] == 999_999) {
                dist[v] = dist[u] + 1;
                queue[qt] = v;
                qt++;
            }
        }
    }

    return dist[dest];
}

// ---------------------------------------------------------------------------
// DAG-DP reference (reverse topological order)
// ---------------------------------------------------------------------------

fn uint dag_dp_reference(uint n, uint* rp, uint* ci, uint* wt, uint src, uint dest) {
    uint* dist = mem::new_array(uint, n).ptr;
    defer free(dist);
    for (uint i = 0; i < n; i++) dist[i] = 999_999;
    dist[dest] = 0;

    // Reverse topological order (valid since all edges forward: u < v)
    for (int u = (int)n - 1; u >= 0; u--) {
        if ((uint)u == dest) continue;
        for (uint e = rp[(uint)u]; e < rp[(uint)u + 1]; e++) {
            uint nd = wt[e] + dist[ci[e]];
            if (nd < dist[(uint)u]) dist[(uint)u] = nd;
        }
    }

    return dist[src];
}

// ---------------------------------------------------------------------------
// DFS path enumeration reference
// ---------------------------------------------------------------------------

struct DfsState {
    uint* rp;
    uint* ci;
    uint* wt;
    uint* paths;
    int path_count;
    int path_max;
}

fn void c_dfs(DfsState* state, uint node, uint dest, uint weight) {
    if (node == dest) {
        if (state.path_count < state.path_max) {
            state.paths[state.path_count] = weight;
        }
        state.path_count++;
        return;
    }
    uint deg = state.rp[node + 1] - state.rp[node];
    for (uint i = 0; i < deg; i++) {
        uint v = state.ci[state.rp[node] + i];
        uint w = state.wt[state.rp[node] + i];
        c_dfs(state, v, dest, weight + w);
    }
}
