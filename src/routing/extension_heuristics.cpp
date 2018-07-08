#include "routing_problem.h"
#include "paths/shortest_paths.h"
#include "flow/network_flow.h"
#include "hungarian.h"



void RoutingProblem::evenDegreeHeuristic(Graph* graph)
{


	std::map<std::pair<int, int>, double> D;
	std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> P;
	std::vector<int> oddDegreeVertices;
	Graph allEdgesGraph = *graph;

    for (Graph::EdgeIDMap::iterator it = allEdgesGraph.edges().begin(); it != allEdgesGraph.edges().end(); it++){
        Graph::Edge* e = it->second;
        e->setUndirected(true);
    }

    for (Graph::VertexIDMap::iterator itV = allEdgesGraph.vertices().begin(); itV != allEdgesGraph.vertices().end(); itV++){

        int degree = 0;
        Graph::Vertex* v = itV->second;
        for (Graph::EdgeSet::iterator itE = v->edges().begin(); itE != v->edges().end(); itE++){
            Graph::Edge* e = *itE;
            if (_notRequiredEdges.find(e) == _notRequiredEdges.end()){
                degree ++;
            }
        }

        if (degree % 2 != 0){
            oddDegreeVertices.push_back(v->id());
        }
    }

    assert (oddDegreeVertices.size() % 2 == 0);

    if (oddDegreeVertices.empty()){
        return;
    }

    






}


std::map<int, int> RoutingProblem::simmetryHeuristic(Graph* graph)
{


	std::vector<int> verticesID;
	std::map<int, int> addedEdges;
	
	std::map<int, int> verticesSupplyMap = network_flow::computeVerticesSupplyMap(*graph, _notRequiredEdges);

	std::map<std::pair<int,int>, int> flowMatrix = network_flow::minCostMaxFlowMatching(*graph, verticesSupplyMap);

	for (std::map<std::pair<int,int>, int>::iterator itMap = flowMatrix.begin(); itMap != flowMatrix.end(); itMap++) {

		int count = itMap->second;

		assert (count >= 0);

		std::pair<int,int> element = itMap->first;
		Graph::Vertex* fromV = graph->vertex(element.first);
		Graph::Edge* e = graph->edge(element.second);
		Graph::Vertex* toV = (e->from()->id() == fromV->id()) ? e->to() : e->from();
		for (int i = 0; i < count; i ++){
			Graph::Edge* duplicatedEdge = graph->addEdge(fromV->id(), toV->id(), e->undirected(), e->cost(), e->capacity());
			duplicatedEdge->setParentId(e->parentId());
		}

		if (count != 0){
			addedEdges[e->id()] = count;
		}		

	}

	return addedEdges;
}

