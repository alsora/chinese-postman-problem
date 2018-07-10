#ifndef __NETWORK_FLOW_H__
#define __NETWORK_FLOW_H__

#include <map>
#include "graph/graph.h"
#include "graph/graph.h"
#include "paths/shortest_paths.h"
#include "utilities.h"

namespace network_flow
{

    std::map<int, int> computeVerticesSupplyMap(const Graph graph, const  Graph::EdgeSet visitedEdges = Graph::EdgeSet());

	std::map<std::pair<int,int>, int> minCostMaxFlowMatching(Graph graph, std::map<int, int> verticesSupplyMap);

    Graph addSourceAndSink(Graph graph, std::map<int, int> verticesSupplyMap);

    void successiveShortestPath(Graph graph, std::map<std::pair<int,int>, int>& flowMatrix);

    std::pair<Graph, Graph::EdgeSet> computeResidualGraph(Graph originalGraph, std::map<std::pair<int,int>, int>& flowMatrix, std::map<int, float> potentials);

	std::vector<int> naivePairsAssignment(std::vector<int> elements, std::map<std::pair<int, int>, float> distanceMap);


}

#endif