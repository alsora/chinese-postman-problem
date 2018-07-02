#include "routing_problem.h"



std::map<int, int> RoutingProblem::computeVerticesSupplyMap(const Graph graph, const Graph::EdgeSet visitedEdges)
{
	std::map<int, int> verticesSupplyMap;

	for (Graph::VertexIDMap::const_iterator itV = graph.vertices().begin(); itV != graph.vertices().end(); itV++) {

		Graph::Vertex* v = itV->second;
		int inDegree = 0;
		int outDegree = 0;
		int degree = 0;

		for (Graph::EdgeSet::const_iterator itE = v->edges().begin(); itE != v->edges().end(); itE++) {
			Graph::Edge* e = *itE;

			if (e->undirected())
				continue;
            
			if (std::find(visitedEdges.begin(), visitedEdges.end(), e) != visitedEdges.end())
				continue;

			if (e->from() == v) {
				outDegree++;
			}
			else {
				inDegree++;
			}
		}

		degree = inDegree - outDegree;
		verticesSupplyMap.insert(std::pair<int, int>(itV->first, degree));

	}

	return verticesSupplyMap;
}