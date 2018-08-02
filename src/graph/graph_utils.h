#ifndef __GRAPH_UTILS_H__
#define __GRAPH_UTILS_H__

#include "graph.h"

namespace graph_utils {

    enum GraphType { DIRECTED, UNDIRECTED, MIXED };


    float eulerianCost(Graph graph, std::vector<int> skipEdges = std::vector<int>());

    GraphType detectGraphType(Graph graph);

    void refineEdges(Graph* graph, std::set<int> edges);


    void printVerticesInfo(Graph graph);
    void printEdgesInfo(Graph graph);










}




#endif