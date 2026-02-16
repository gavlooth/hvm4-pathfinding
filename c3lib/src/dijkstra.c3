module dijkstra;

import pathfind;
import swr_dijkstra;

// ============================================================
// Standalone Dijkstra utilities operating on pathfind::Graph
// Uses FibHeap from swr_dijkstra for O((V+E) log V)
// ============================================================

struct DijkResult {
    uint[] dist;
    uint[] pred;
    uint n;
}

fn void DijkResult.destroy(&self) {
    if (self.dist.ptr != null) free(self.dist.ptr);
    if (self.pred.ptr != null) free(self.pred.ptr);
}

// Single-source shortest path on adjacency list graph.
// Uses lazy insertion: only discovered nodes enter the heap,
// avoiding O(V) upfront inserts that overflow consolidate buffers.
fn DijkResult? sssp(pathfind::Graph* g, uint source) {
    uint n = g.n;
    if (source >= n) return pathfind::INVALID_NODE~;

    uint[] dist = mem::new_array(uint, n);
    uint[] pred = mem::new_array(uint, n);

    for (uint i = 0; i < n; i++) {
        dist[i] = pathfind::INF;
        pred[i] = n; // invalid sentinel
    }
    dist[source] = 0;

    swr_dijkstra::FibNode*[] nodes = mem::new_array(swr_dijkstra::FibNode*, n);
    defer free(nodes.ptr);
    for (uint i = 0; i < n; i++) nodes[i] = null;

    swr_dijkstra::FibHeap heap = swr_dijkstra::fib_heap_new();
    defer heap.destroy();

    nodes[source] = heap.insert(0, source);

    while (heap.n > 0) {
        swr_dijkstra::FibNode* u_node = heap.extract_min();
        if (u_node == null) break;
        uint u = u_node.value;
        uint du = u_node.key;
        nodes[u] = null; // mark as settled

        if (du >= pathfind::INF) break;

        // Traverse adjacency list: O(degree)
        for (usz i = 0; i < g.adj[u].len(); i++) {
            pathfind::Edge e = g.adj[u][i];
            uint new_d = du + e.weight;
            if (new_d < dist[e.to]) {
                dist[e.to] = new_d;
                pred[e.to] = u;
                if (nodes[e.to] == null) {
                    nodes[e.to] = heap.insert(new_d, e.to);
                } else {
                    heap.decrease_key(nodes[e.to], new_d);
                }
            }
        }
    }

    return (DijkResult){ .dist = dist, .pred = pred, .n = n };
}

// Bidirectional Dijkstra: returns shortest distance between source and target.
// fwd = original graph, bwd = reverse graph (edges flipped).
fn uint? bidirectional(pathfind::Graph* fwd, pathfind::Graph* bwd, uint source, uint target) {
    uint n = fwd.n;
    if (source >= n || target >= n) return pathfind::INVALID_NODE~;
    if (source == target) return 0;

    uint[] dist_f = mem::new_array(uint, n);
    uint[] dist_b = mem::new_array(uint, n);
    bool[] settled_f = mem::new_array(bool, n);
    bool[] settled_b = mem::new_array(bool, n);
    defer { free(dist_f.ptr); free(dist_b.ptr); free(settled_f.ptr); free(settled_b.ptr); }

    for (uint i = 0; i < n; i++) {
        dist_f[i] = pathfind::INF;
        dist_b[i] = pathfind::INF;
        settled_f[i] = false;
        settled_b[i] = false;
    }
    dist_f[source] = 0;
    dist_b[target] = 0;

    swr_dijkstra::FibNode*[] nodes_f = mem::new_array(swr_dijkstra::FibNode*, n);
    swr_dijkstra::FibNode*[] nodes_b = mem::new_array(swr_dijkstra::FibNode*, n);
    defer { free(nodes_f.ptr); free(nodes_b.ptr); }

    swr_dijkstra::FibHeap heap_f = swr_dijkstra::fib_heap_new();
    swr_dijkstra::FibHeap heap_b = swr_dijkstra::fib_heap_new();
    defer { heap_f.destroy(); heap_b.destroy(); }

    nodes_f[source] = heap_f.insert(0, source);
    nodes_b[target] = heap_b.insert(0, target);

    uint mu = pathfind::INF; // best known s-t distance

    while (heap_f.n > 0 || heap_b.n > 0) {
        // Forward step
        if (heap_f.n > 0) {
            swr_dijkstra::FibNode* uf = heap_f.extract_min();
            if (uf != null) {
                uint u = uf.value;
                uint du = uf.key;
                if (du >= mu) {
                    // Can't improve
                } else {
                    settled_f[u] = true;
                    if (settled_b[u] && du + dist_b[u] < mu) {
                        mu = du + dist_b[u];
                    }
                    for (usz i = 0; i < fwd.adj[u].len(); i++) {
                        pathfind::Edge e = fwd.adj[u][i];
                        uint nd = du + e.weight;
                        if (nd < dist_f[e.to]) {
                            dist_f[e.to] = nd;
                            if (nodes_f[e.to] == null) {
                                nodes_f[e.to] = heap_f.insert(nd, e.to);
                            } else {
                                heap_f.decrease_key(nodes_f[e.to], nd);
                            }
                            if (dist_b[e.to] < pathfind::INF && nd + dist_b[e.to] < mu) {
                                mu = nd + dist_b[e.to];
                            }
                        }
                    }
                }
            }
        }

        // Backward step
        if (heap_b.n > 0) {
            swr_dijkstra::FibNode* ub = heap_b.extract_min();
            if (ub != null) {
                uint u = ub.value;
                uint du = ub.key;
                if (du >= mu) {
                    // Can't improve
                } else {
                    settled_b[u] = true;
                    if (settled_f[u] && dist_f[u] + du < mu) {
                        mu = dist_f[u] + du;
                    }
                    for (usz i = 0; i < bwd.adj[u].len(); i++) {
                        pathfind::Edge e = bwd.adj[u][i];
                        uint nd = du + e.weight;
                        if (nd < dist_b[e.to]) {
                            dist_b[e.to] = nd;
                            if (nodes_b[e.to] == null) {
                                nodes_b[e.to] = heap_b.insert(nd, e.to);
                            } else {
                                heap_b.decrease_key(nodes_b[e.to], nd);
                            }
                            if (dist_f[e.to] < pathfind::INF && dist_f[e.to] + nd < mu) {
                                mu = dist_f[e.to] + nd;
                            }
                        }
                    }
                }
            }
        }

        // Termination: both forward and backward min-keys >= mu
        uint min_f = (heap_f.n > 0 && heap_f.min != null) ? heap_f.min.key : pathfind::INF;
        uint min_b = (heap_b.n > 0 && heap_b.min != null) ? heap_b.min.key : pathfind::INF;
        if (min_f >= mu && min_b >= mu) break;
    }

    if (mu >= pathfind::INF) return pathfind::NO_PATH~;
    return mu;
}

// A* with precomputed potential array. potential[u] = h(u, target).
// Returns SSSP result but terminates early when target is settled.
fn DijkResult? astar(pathfind::Graph* g, uint source, uint target, uint[] potential) {
    uint n = g.n;
    if (source >= n || target >= n) return pathfind::INVALID_NODE~;

    uint[] dist = mem::new_array(uint, n);
    uint[] pred = mem::new_array(uint, n);

    for (uint i = 0; i < n; i++) {
        dist[i] = pathfind::INF;
        pred[i] = n;
    }
    dist[source] = 0;

    swr_dijkstra::FibNode*[] nodes = mem::new_array(swr_dijkstra::FibNode*, n);
    defer free(nodes.ptr);
    for (uint i = 0; i < n; i++) nodes[i] = null;

    swr_dijkstra::FibHeap heap = swr_dijkstra::fib_heap_new();
    defer heap.destroy();

    // Key = dist + potential (A* f-value)
    nodes[source] = heap.insert(potential[source], source);

    while (heap.n > 0) {
        swr_dijkstra::FibNode* u_node = heap.extract_min();
        if (u_node == null) break;
        uint u = u_node.value;
        nodes[u] = null; // mark as settled — prevents stale decrease_key

        if (u == target) break; // target settled

        uint du = dist[u];
        if (du >= pathfind::INF) break;

        for (usz i = 0; i < g.adj[u].len(); i++) {
            pathfind::Edge e = g.adj[u][i];
            uint new_d = du + e.weight;
            if (new_d < dist[e.to]) {
                dist[e.to] = new_d;
                pred[e.to] = u;
                uint fval = new_d + potential[e.to];
                if (nodes[e.to] == null) {
                    nodes[e.to] = heap.insert(fval, e.to);
                } else {
                    heap.decrease_key(nodes[e.to], fval);
                }
            }
        }
    }

    return (DijkResult){ .dist = dist, .pred = pred, .n = n };
}

// Build reverse graph (flip all edges)
fn pathfind::Graph build_reverse(pathfind::Graph* g) {
    pathfind::Graph rev;
    rev.init(g.n);
    for (uint u = 0; u < g.n; u++) {
        for (usz i = 0; i < g.adj[u].len(); i++) {
            pathfind::Edge e = g.adj[u][i];
            rev.add_edge(e.to, u, e.weight);
        }
    }
    return rev;
}

// Extract path from pred array: source -> ... -> target
fn uint[]? extract_path(uint[] pred, uint source, uint target, uint n) {
    if (pred[target] == n && source != target) return pathfind::NO_PATH~;

    uint[] path = mem::new_array(uint, n);
    uint len = 0;
    uint current = target;
    while (current != source && len < n) {
        path[len++] = current;
        current = pred[current];
        if (current >= n) {
            free(path.ptr);
            return pathfind::NO_PATH~;
        }
    }
    path[len++] = source;

    // Reverse
    for (uint i = 0; i < len / 2; i++) {
        uint tmp = path[i];
        path[i] = path[len - 1 - i];
        path[len - 1 - i] = tmp;
    }

    return path[:len];
}
