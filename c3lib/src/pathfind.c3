module pathfind;
import std::collections;
import std::math;
import libc;

const uint INF = 999_999;

faultdef INVALID_NODE, NO_PATH, FILE_ERROR;

// ----- Data types -----

struct Edge {
    uint to;
    uint weight;
}

alias EdgeList = list::List{Edge};

struct Graph {
    uint n;
    EdgeList[] adj;
}

struct PathResult {
    uint[] weights;
    usz    count;
}

// ----- Geography primitives -----

const double PI = 3.14159265358979323846;
const double EARTH_RADIUS_M = 6_371_000.0;
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;

struct LatLon {
    double lat;
    double lon;
}

fn double haversine(LatLon a, LatLon b) {
    double dlat = (b.lat - a.lat) * DEG_TO_RAD;
    double dlon = (b.lon - a.lon) * DEG_TO_RAD;
    double la = a.lat * DEG_TO_RAD;
    double lb = b.lat * DEG_TO_RAD;
    double s = math::sin(dlat / 2.0);
    double c = math::sin(dlon / 2.0);
    double h = s * s + math::cos(la) * math::cos(lb) * c * c;
    return 2.0 * EARTH_RADIUS_M * math::asin(math::sqrt(h));
}

fn double bearing(LatLon a, LatLon b) {
    double la = a.lat * DEG_TO_RAD;
    double lb = b.lat * DEG_TO_RAD;
    double dlon = (b.lon - a.lon) * DEG_TO_RAD;
    double y = math::sin(dlon) * math::cos(lb);
    double x = math::cos(la) * math::sin(lb) - math::sin(la) * math::cos(lb) * math::cos(dlon);
    double brg = math::atan2(y, x) * RAD_TO_DEG;
    if (brg < 0.0) brg += 360.0;
    return brg;
}

fn uint haversine_u32(LatLon a, LatLon b) {
    return (uint)haversine(a, b);
}

// ----- GeoGraph -----

struct GeoGraph {
    Graph graph;
    LatLon[] coords;
    uint node_count;
    uint capacity;
}

fn void GeoGraph.init(&self, uint capacity) {
    self.graph.init(capacity);
    self.coords = mem::new_array(LatLon, capacity);
    self.node_count = 0;
    self.capacity = capacity;
}

fn void GeoGraph.destroy(&self) {
    self.graph.destroy();
    if (self.coords.ptr != null) free(self.coords.ptr);
}

fn uint GeoGraph.add_node(&self, double lat, double lon) {
    uint id = self.node_count;
    self.coords[id] = (LatLon){ .lat = lat, .lon = lon };
    self.node_count++;
    return id;
}

fn void GeoGraph.add_edge(&self, uint from, uint to, uint weight) {
    self.graph.add_edge(from, to, weight);
}

fn void GeoGraph.add_geo_edge(&self, uint from, uint to) {
    uint w = haversine_u32(self.coords[from], self.coords[to]);
    if (w < 1) w = 1;
    self.graph.add_edge(from, to, w);
}

fn void GeoGraph.add_geo_biedge(&self, uint a, uint b) {
    uint w = haversine_u32(self.coords[a], self.coords[b]);
    if (w < 1) w = 1;
    self.graph.add_biedge(a, b, w);
}

fn uint GeoGraph.nearest_node(&self, double lat, double lon) {
    LatLon target = { .lat = lat, .lon = lon };
    uint best = 0;
    double best_dist = 1e18;
    for (uint i = 0; i < self.node_count; i++) {
        double d = haversine(target, self.coords[i]);
        if (d < best_dist) {
            best_dist = d;
            best = i;
        }
    }
    return best;
}

fn void GeoGraph.connect_knn(&self, uint k) {
    uint n = self.node_count;
    double[] dists = mem::new_array(double, n);
    uint[] indices = mem::new_array(uint, n);
    defer { free(dists.ptr); free(indices.ptr); }

    for (uint i = 0; i < n; i++) {
        for (uint j = 0; j < n; j++) {
            dists[j] = haversine(self.coords[i], self.coords[j]);
            indices[j] = j;
        }
        dists[i] = 1e18;

        uint found = 0;
        for (uint pass = 0; pass < n - 1 && found < k; pass++) {
            uint min_idx = pass;
            for (uint q = pass + 1; q < n; q++) {
                if (dists[q] < dists[min_idx]) min_idx = q;
            }
            double tmp_d = dists[pass]; dists[pass] = dists[min_idx]; dists[min_idx] = tmp_d;
            uint tmp_i = indices[pass]; indices[pass] = indices[min_idx]; indices[min_idx] = tmp_i;

            uint j = indices[pass];
            double dist = dists[pass];
            if (dist >= 1e17) break;

            self.add_geo_biedge(i, j);
            found++;
        }
    }
}

fn void GeoGraph.connect_radius(&self, double radius_m) {
    uint n = self.node_count;
    for (uint i = 0; i < n; i++) {
        for (uint j = i + 1; j < n; j++) {
            double d = haversine(self.coords[i], self.coords[j]);
            if (d <= radius_m) {
                uint w = (uint)d;
                if (w < 1) w = 1;
                self.graph.add_biedge(i, j, w);
            }
        }
    }
}

// ----- GeoGraph file loader -----

extern fn CFile fopen(ZString path, ZString mode) @cname("fopen");
extern fn int fclose(CFile f) @cname("fclose");
extern fn int fscanf(CFile f, ZString fmt, ...) @cname("fscanf");
extern fn char* fgets(char* buf, int size, CFile f) @cname("fgets");

fn GeoGraph? load_geo_file(ZString path) {
    CFile f = fopen(path, "r");
    if (f == null) return FILE_ERROR~;

    char[256] line_buf;
    uint node_count = 0;
    uint edge_count = 0;

    // Read NODES header
    while (fgets(&line_buf, 256, f) != null) {
        if (line_buf[0] == '#' || line_buf[0] == '\n') continue;
        if (fscanf(f, "") >= 0) {} // consume
        // Try to parse "NODES <n>"
        int scanned = fscanf(f, "");
        // Rewind — just use sscanf approach via manual parse
        break;
    }

    // Simpler: rewind and use fscanf directly
    fclose(f);
    f = fopen(path, "r");
    if (f == null) return FILE_ERROR~;
    defer fclose(f);

    // Skip comment lines, find NODES
    while (fgets(&line_buf, 256, f) != null) {
        if (line_buf[0] == '#' || line_buf[0] == '\n' || line_buf[0] == '\r') continue;
        // Check for "NODES"
        if (line_buf[0] == 'N') {
            // Parse "NODES <n>"
            uint n = 0;
            for (int i = 6; i < 256 && line_buf[i] != 0; i++) {
                if (line_buf[i] >= '0' && line_buf[i] <= '9') {
                    n = n * 10 + (uint)(line_buf[i] - '0');
                }
            }
            node_count = n;
            break;
        }
    }

    if (node_count == 0) return FILE_ERROR~;

    GeoGraph geo;
    geo.init(node_count);

    // Read node coordinates
    uint nodes_read = 0;
    while (nodes_read < node_count && fgets(&line_buf, 256, f) != null) {
        if (line_buf[0] == '#' || line_buf[0] == '\n' || line_buf[0] == '\r') continue;
        double lat = 0;
        double lon = 0;
        if (fscanf(f, "") >= 0) {} // dummy
        // Parse two doubles from line_buf
        // Use a simple manual parse
        int pos = 0;
        // skip whitespace
        while (line_buf[pos] == ' ' || line_buf[pos] == '\t') pos++;
        // parse lat
        bool neg = false;
        if (line_buf[pos] == '-') { neg = true; pos++; }
        double whole = 0;
        while (line_buf[pos] >= '0' && line_buf[pos] <= '9') {
            whole = whole * 10.0 + (double)(line_buf[pos] - '0');
            pos++;
        }
        double frac = 0;
        double div = 1.0;
        if (line_buf[pos] == '.') {
            pos++;
            while (line_buf[pos] >= '0' && line_buf[pos] <= '9') {
                frac = frac * 10.0 + (double)(line_buf[pos] - '0');
                div *= 10.0;
                pos++;
            }
        }
        lat = whole + frac / div;
        if (neg) lat = -lat;

        // skip whitespace/comma
        while (line_buf[pos] == ' ' || line_buf[pos] == '\t' || line_buf[pos] == ',') pos++;

        // parse lon
        neg = false;
        if (line_buf[pos] == '-') { neg = true; pos++; }
        whole = 0;
        while (line_buf[pos] >= '0' && line_buf[pos] <= '9') {
            whole = whole * 10.0 + (double)(line_buf[pos] - '0');
            pos++;
        }
        frac = 0;
        div = 1.0;
        if (line_buf[pos] == '.') {
            pos++;
            while (line_buf[pos] >= '0' && line_buf[pos] <= '9') {
                frac = frac * 10.0 + (double)(line_buf[pos] - '0');
                div *= 10.0;
                pos++;
            }
        }
        lon = whole + frac / div;
        if (neg) lon = -lon;

        geo.add_node(lat, lon);
        nodes_read++;
    }

    // Find EDGES header
    while (fgets(&line_buf, 256, f) != null) {
        if (line_buf[0] == '#' || line_buf[0] == '\n' || line_buf[0] == '\r') continue;
        if (line_buf[0] == 'E') {
            uint m = 0;
            for (int i = 6; i < 256 && line_buf[i] != 0; i++) {
                if (line_buf[i] >= '0' && line_buf[i] <= '9') {
                    m = m * 10 + (uint)(line_buf[i] - '0');
                }
            }
            edge_count = m;
            break;
        }
    }

    // Read edges
    uint edges_read = 0;
    while (edges_read < edge_count && fgets(&line_buf, 256, f) != null) {
        if (line_buf[0] == '#' || line_buf[0] == '\n' || line_buf[0] == '\r') continue;
        // Parse two uints: from to
        int pos = 0;
        while (line_buf[pos] == ' ' || line_buf[pos] == '\t') pos++;
        uint from_id = 0;
        while (line_buf[pos] >= '0' && line_buf[pos] <= '9') {
            from_id = from_id * 10 + (uint)(line_buf[pos] - '0');
            pos++;
        }
        while (line_buf[pos] == ' ' || line_buf[pos] == '\t') pos++;
        uint to_id = 0;
        while (line_buf[pos] >= '0' && line_buf[pos] <= '9') {
            to_id = to_id * 10 + (uint)(line_buf[pos] - '0');
            pos++;
        }

        if (from_id < geo.node_count && to_id < geo.node_count) {
            geo.add_geo_edge(from_id, to_id);
        }
        edges_read++;
    }

    return geo;
}

// ----- Graph lifecycle -----

fn void Graph.init(&self, uint node_count) {
    self.n = node_count;
    self.adj = mem::new_array(EdgeList, node_count);
}

fn void Graph.add_edge(&self, uint from, uint to, uint weight) {
    self.adj[from].push((Edge){ .to = to, .weight = weight });
}

fn void Graph.add_biedge(&self, uint a, uint b, uint weight) {
    self.add_edge(a, b, weight);
    self.add_edge(b, a, weight);
}

fn void Graph.destroy(&self) {
    for (uint i = 0; i < self.n; i++) {
        self.adj[i].free();
    }
    free(self.adj.ptr);
}


