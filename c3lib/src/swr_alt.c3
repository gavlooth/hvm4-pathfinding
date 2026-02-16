module swr_alt;

import swr;
import dijkstra;
import pathfind;

// ============================================================
// ALT: A* + Landmarks + Triangle Inequality
//
// Preprocessing: Select K landmarks, compute SSSP from each
// landmark on both forward and reverse graphs.
// Query: A* with heuristic h(u,t) = max_L |d(L,u) - d(L,t)|
// ============================================================

struct AltData {
    uint k;              // number of landmarks
    uint[] landmarks;    // k landmark node IDs
    uint[] dist_from;    // k*V: dist_from[l*V + u] = d(landmark_l -> u)
    uint[] dist_to;      // k*V: dist_to[l*V + u] = d(u -> landmark_l)
    uint v;
}

fn void AltData.destroy(&self) {
    if (self.landmarks.ptr != null) free(self.landmarks.ptr);
    if (self.dist_from.ptr != null) free(self.dist_from.ptr);
    if (self.dist_to.ptr != null) free(self.dist_to.ptr);
}

// Farthest-first landmark selection
fn uint[] select_landmarks(pathfind::Graph* g, uint k) {
    uint n = g.n;
    if (k > n) k = n;

    uint[] lm = mem::new_array(uint, k);
    uint[] min_dist = mem::new_array(uint, n);
    defer free(min_dist.ptr);

    for (uint i = 0; i < n; i++) min_dist[i] = pathfind::INF;

    // First landmark = node 0
    lm[0] = 0;

    for (uint l = 0; l < k; l++) {
        // Update min distances from this landmark
        dijkstra::DijkResult dr = dijkstra::sssp(g, lm[l])!!;
        for (uint i = 0; i < n; i++) {
            if (dr.dist[i] < min_dist[i]) min_dist[i] = dr.dist[i];
        }
        dr.destroy();

        // Next landmark = farthest node from all existing landmarks
        if (l + 1 < k) {
            uint best = 0;
            uint best_d = 0;
            for (uint i = 0; i < n; i++) {
                if (min_dist[i] > best_d && min_dist[i] < pathfind::INF) {
                    best_d = min_dist[i];
                    best = i;
                }
            }
            lm[l + 1] = best;
        }
    }

    return lm;
}

// Precompute ALT data: forward and backward SSSP from each landmark
fn AltData preprocess(pathfind::Graph* g, uint k) {
    uint n = g.n;
    if (k > n) k = n;

    uint[] landmarks = select_landmarks(g, k);

    uint[] dist_from = mem::new_array(uint, (usz)k * n);
    uint[] dist_to = mem::new_array(uint, (usz)k * n);

    // Build reverse graph
    pathfind::Graph rev = dijkstra::build_reverse(g);
    defer rev.destroy();

    // Forward: Dijkstra from each landmark on original graph
    for (uint l = 0; l < k; l++) {
        dijkstra::DijkResult dr = dijkstra::sssp(g, landmarks[l])!!;
        for (uint i = 0; i < n; i++) {
            dist_from[(usz)l * n + i] = dr.dist[i];
        }
        dr.destroy();
    }

    // Backward: Dijkstra from each landmark on reversed graph
    // dist_to[l*V + u] = d(u -> landmark_l) = d_rev(landmark_l -> u)
    for (uint l = 0; l < k; l++) {
        dijkstra::DijkResult dr = dijkstra::sssp(&rev, landmarks[l])!!;
        for (uint i = 0; i < n; i++) {
            dist_to[(usz)l * n + i] = dr.dist[i];
        }
        dr.destroy();
    }

    return (AltData){
        .k = k,
        .landmarks = landmarks,
        .dist_from = dist_from,
        .dist_to = dist_to,
        .v = n
    };
}

// Compute ALT heuristic: h(u, target) = max over landmarks of
// max(|d(L,t) - d(L,u)|, |d(u,L) - d(t,L)|)
fn uint alt_heuristic(AltData* alt, uint u, uint target) {
    uint best = 0;
    uint v = alt.v;
    for (uint l = 0; l < alt.k; l++) {
        usz base = (usz)l * v;
        uint dl_u = alt.dist_from[base + u];
        uint dl_t = alt.dist_from[base + target];
        uint du_l = alt.dist_to[base + u];
        uint dt_l = alt.dist_to[base + target];

        // Forward bound: d(L,t) - d(L,u), clamped to 0 (directed-graph safe)
        if (dl_u < pathfind::INF && dl_t < pathfind::INF && dl_t > dl_u) {
            uint fwd = dl_t - dl_u;
            if (fwd > best) best = fwd;
        }
        // Backward bound: d(u,L) - d(t,L), clamped to 0 (directed-graph safe)
        if (du_l < pathfind::INF && dt_l < pathfind::INF && du_l > dt_l) {
            uint bwd = du_l - dt_l;
            if (bwd > best) best = bwd;
        }
    }
    return best;
}

// Build potential array for A*: potential[u] = alt_heuristic(u, target)
fn uint[] build_potential(AltData* alt, uint target) {
    uint[] pot = mem::new_array(uint, alt.v);
    for (uint u = 0; u < alt.v; u++) {
        pot[u] = alt_heuristic(alt, u, target);
    }
    return pot;
}

// Query: A* search using ALT heuristic
fn swr::RouteResult? query(AltData* alt, pathfind::Graph* g, uint source, uint target) {
    uint[] potential = build_potential(alt, target);
    defer free(potential.ptr);

    dijkstra::DijkResult dr = dijkstra::astar(g, source, target, potential)!;

    uint cost = dr.dist[target];
    if (cost >= pathfind::INF) {
        dr.destroy();
        return swr::SWR_ERROR~;
    }

    uint[] path = dijkstra::extract_path(dr.pred, source, target, dr.n)!;

    return (swr::RouteResult){
        .path = path,
        .path_len = path.len,
        .total_cost = cost,
        .distances = dr.dist,
        .dist_count = dr.n
    };
}

// Full SWR routing with ALT
fn swr::RouteResult? swr_route_alt(swr::SwrState* state, swr::WeatherGrid* weather) {
    swr::update_all_weights(state, weather);

    AltData alt = preprocess(&state.prm.graph, 8);
    defer alt.destroy();

    swr::RouteResult result = query(&alt, &state.prm.graph, state.origin, state.destination)!;

    // Free the pred array that's baked into distances (we only need dist)
    return result;
}
