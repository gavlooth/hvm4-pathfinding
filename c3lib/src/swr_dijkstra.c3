module swr_dijkstra;
import swr;
import pathfind;

// ============================================================
// Fibonacci Heap
// ============================================================

struct FibNode {
    uint key;
    uint value;
    uint degree;
    bool mark;
    FibNode* parent;
    FibNode* child;
    FibNode* left;
    FibNode* right;
}

struct FibHeap {
    FibNode* min;
    uint n;
}

fn FibHeap fib_heap_new() {
    return (FibHeap){ .min = null, .n = 0 };
}

fn FibNode* fib_node_new(uint key, uint value) {
    FibNode* node = mem::new(FibNode);
    node.key = key;
    node.value = value;
    node.degree = 0;
    node.mark = false;
    node.parent = null;
    node.child = null;
    node.left = node;
    node.right = node;
    return node;
}

fn void list_insert(FibNode* a, FibNode* b) {
    // Insert b to the right of a in circular doubly-linked list
    b.left = a;
    b.right = a.right;
    a.right.left = b;
    a.right = b;
}

fn void list_remove(FibNode* node) {
    node.left.right = node.right;
    node.right.left = node.left;
    node.left = node;
    node.right = node;
}

fn FibNode* FibHeap.insert(&self, uint key, uint value) {
    FibNode* node = fib_node_new(key, value);
    if (self.min == null) {
        self.min = node;
    } else {
        list_insert(self.min, node);
        if (key < self.min.key) {
            self.min = node;
        }
    }
    self.n++;
    return node;
}

fn FibNode* FibHeap.extract_min(&self) {
    FibNode* z = self.min;
    if (z == null) return null;

    // Add all children of z to root list
    if (z.child != null) {
        FibNode* child = z.child;
        FibNode* first = child;
        do {
            FibNode* next_child = child.right;
            child.parent = null;
            list_remove(child);
            list_insert(z, child);
            child = next_child;
        } while (child != first);
    }

    // Remove z from root list
    if (z == z.right) {
        self.min = null;
    } else {
        self.min = z.right;
        list_remove(z);
        self.consolidate();
    }

    self.n--;
    return z;
}

fn void FibHeap.decrease_key(&self, FibNode* x, uint k) {
    if (k >= x.key) return;
    x.key = k;
    FibNode* y = x.parent;
    if (y != null && x.key < y.key) {
        self.cut(x, y);
        self.cascading_cut(y);
    }
    if (x.key < self.min.key) {
        self.min = x;
    }
}

fn void FibHeap.cut(&self, FibNode* x, FibNode* y) {
    // Remove x from child list of y
    if (x == x.right) {
        y.child = null;
    } else {
        if (y.child == x) y.child = x.right;
        list_remove(x);
    }
    y.degree--;
    // Add x to root list
    list_insert(self.min, x);
    x.parent = null;
    x.mark = false;
}

fn void FibHeap.cascading_cut(&self, FibNode* y) {
    FibNode* z = y.parent;
    if (z != null) {
        if (!y.mark) {
            y.mark = true;
        } else {
            self.cut(y, z);
            self.cascading_cut(z);
        }
    }
}

fn void FibHeap.consolidate(&self) {
    // Max degree is O(log n), use 64 as safe upper bound
    FibNode*[64] degree_table;
    for (int i = 0; i < 64; i++) degree_table[i] = null;

    // Collect all root nodes first (since we modify the list)
    FibNode*[1024] roots;
    uint root_count = 0;
    FibNode* x = self.min;
    if (x != null) {
        FibNode* start = x;
        do {
            roots[root_count++] = x;
            x = x.right;
        } while (x != start && root_count < 1024);
    }

    for (uint i = 0; i < root_count; i++) {
        x = roots[i];
        uint d = x.degree;
        while (d < 64 && degree_table[d] != null) {
            FibNode* y = degree_table[d];
            if (x.key > y.key) {
                FibNode* tmp = x; x = y; y = tmp;
            }
            self.link(y, x);
            degree_table[d] = null;
            d++;
        }
        if (d < 64) degree_table[d] = x;
    }

    // Rebuild root list and find new min
    self.min = null;
    for (int i = 0; i < 64; i++) {
        if (degree_table[i] != null) {
            FibNode* node = degree_table[i];
            node.left = node;
            node.right = node;
            node.parent = null;
            if (self.min == null) {
                self.min = node;
            } else {
                list_insert(self.min, node);
                if (node.key < self.min.key) {
                    self.min = node;
                }
            }
        }
    }
}

fn void FibHeap.link(&self, FibNode* y, FibNode* x) {
    // Make y a child of x
    list_remove(y);
    y.parent = x;
    if (x.child == null) {
        x.child = y;
        y.left = y;
        y.right = y;
    } else {
        list_insert(x.child, y);
    }
    x.degree++;
    y.mark = false;
}

fn void FibHeap.destroy(&self) {
    if (self.min == null) return;
    destroy_tree(self.min);
    self.min = null;
    self.n = 0;
}

fn void destroy_tree(FibNode* node) {
    if (node == null) return;
    FibNode* current = node;
    FibNode*[1024] stack;
    uint sp = 0;
    // Collect all nodes in the circular list
    FibNode* start = current;
    do {
        stack[sp++] = current;
        current = current.right;
    } while (current != start && sp < 1024);
    // Recursively destroy children, then free
    for (uint i = 0; i < sp; i++) {
        if (stack[i].child != null) {
            destroy_tree(stack[i].child);
        }
        free(stack[i]);
    }
}

// ============================================================
// Dijkstra's Algorithm (pure C3, no HVM4)
// ============================================================

fn swr::RouteResult? swr_dijkstra(swr::SwrState* state, swr::WeatherGrid* weather) {
    swr::update_all_weights(state, weather);

    uint n = state.prm.n;
    uint src = state.origin;
    uint dst = state.destination;

    // Distance and predecessor arrays
    uint[] dist = mem::new_array(uint, n);
    uint[] pred = mem::new_array(uint, n);
    defer free(pred.ptr);

    for (uint i = 0; i < n; i++) {
        dist[i] = pathfind::INF;
        pred[i] = n; // invalid sentinel
    }
    dist[src] = 0;

    // Build node -> FibNode pointer mapping
    FibNode*[] nodes = mem::new_array(FibNode*, n);
    defer free(nodes.ptr);

    FibHeap heap = fib_heap_new();
    defer heap.destroy();

    for (uint i = 0; i < n; i++) {
        nodes[i] = heap.insert(dist[i], i);
    }

    // Dijkstra main loop
    while (heap.n > 0) {
        FibNode* u_node = heap.extract_min();
        if (u_node == null) break;
        uint u = u_node.value;
        uint du = u_node.key;

        if (du >= pathfind::INF) break; // remaining nodes unreachable

        // Relax all outgoing edges of u
        for (uint i = 0; i < state.prm.edge_count; i++) {
            if (state.prm.edge_from[i] == u) {
                uint v = state.prm.edge_to[i];
                uint w = state.prm.edge_weight[i];
                uint new_d = du + w;
                if (new_d < dist[v]) {
                    dist[v] = new_d;
                    pred[v] = u;
                    heap.decrease_key(nodes[v], new_d);
                }
            }
        }
    }

    uint cost = dist[dst];
    if (cost >= pathfind::INF) {
        free(dist.ptr);
        return swr::SWR_ERROR~;
    }

    // Extract path from predecessors
    uint[] path = mem::new_array(uint, n);
    uint len = 0;
    uint current = dst;
    while (current != src && len < n) {
        path[len++] = current;
        current = pred[current];
        if (current >= n) break; // no predecessor
    }
    path[len++] = src;

    // Reverse path
    for (uint i = 0; i < len / 2; i++) {
        uint tmp = path[i];
        path[i] = path[len - 1 - i];
        path[len - 1 - i] = tmp;
    }

    return (swr::RouteResult){
        .path = path[:len],
        .path_len = len,
        .total_cost = cost,
        .distances = dist,
        .dist_count = n
    };
}
