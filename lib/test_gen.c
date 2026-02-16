#include "libhvm4_graph.h"
#include <stdio.h>

int main() {
    hvm4_graph_t *g = hvm4_graph_new(4);
    hvm4_graph_add_edge(g, 0, 1, 1);
   
    uint32_t dist[4];
    hvm4_result_t res = hvm4_shortest_path(g, 0, dist);
    
    hvm4_graph_free(g);
    return 0;
}
