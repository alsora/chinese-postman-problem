#include "routing_problem.h"


void RoutingProblem::evenDegreeHeuristic(Graph* graph)
{

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

	std::map<std::pair<int, int>, float> D;
	std::map<std::pair<int, int>, std::vector<int>> P;

	shortest_paths::mapDijkstra(allEdgesGraph, oddDegreeVertices, oddDegreeVertices, &D, &P);

	for (unsigned i = 0; i < oddDegreeVertices.size(); i++){
		int vId = oddDegreeVertices[i];
		D[std::make_pair(vId, vId)] = std::numeric_limits<float>::infinity(); 
	}


	std::vector<int> bestAssignment = network_flow::naivePairsAssignment(oddDegreeVertices, D);

	for (int j = 1; j <= oddDegreeVertices.size()/2; j++){
		int firstId = -1;
		int secondId = -1;

		for (int k = 0; k < oddDegreeVertices.size();k++){
			int label = bestAssignment[k];
			if (label == j && firstId == -1){
				firstId = k; 
			}
			else if (label == j && secondId == -1){
				secondId = k;
			}
		}

		assert (firstId >= 0 && secondId >= 0);

		std::vector<int> path = P[std::make_pair(oddDegreeVertices[firstId], oddDegreeVertices[secondId])];

		for (int eId : path){
			Graph::Edge* e = graph->edge(eId);
			Graph::Vertex* fromV = e->from();
			Graph::Vertex* toV = e->to();

			Graph::Edge* duplicatedEdge = graph->addEdge(fromV->id(), toV->id(), e->undirected(), e->cost(), e->capacity());
			duplicatedEdge->setParentId(e->parentId());
		}

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



std::tuple<std::vector<int>, std::vector<int>, std::vector<int>> RoutingProblem::fredericksonInOutDegree(Graph* graph)
{

	std::vector<int> directedElements;
	std::vector<int> undirectedElements;
	std::vector<int> addedElements;

	Graph g2 = *graph;
	Graph g3 = *graph;


	//Loop on original edges, since I'm adding edges in g2. quadruplicate edges and add arcs to directed elements
	for (Graph::EdgeIDMap::iterator it = graph->edges().begin(); it != graph->edges().end(); it++) {
		Graph::Edge* e =it->second;
		if (!e->undirected()) {
			directedElements.push_back(e->id());
			continue;
		}
			
		Graph::Vertex* vFrom = g3.vertex(e->from()->id());
		Graph::Vertex* vTo = g3.vertex(e->to()->id());
		//Substitute old edge with 2 pairs of arcs
		Graph::Edge* newArc;

		newArc = g3.addEdge(vFrom, vTo, false, e->cost(), e->capacity());
		newArc->setParentId(e->id());
		newArc = g3.addEdge(vTo, vFrom, false, e->cost(), e->capacity());
		newArc->setParentId(e->id());
		newArc = g3.addEdge(vFrom, vTo, false, 0.0, 1);
		newArc->setParentId(e->id());
		newArc = g3.addEdge(vTo, vFrom, false, 0.0, 1);
		newArc->setParentId(e->id());
		g3.removeEdge(g3.edge(e->id()));
	}

	
	std::map<int, int> newEdges = simmetryHeuristic(&g3);


	for (std::map<int, int>::iterator itMap = newEdges.begin(); itMap != newEdges.end(); itMap++) {
		//Add new edges to original graph
		int thisId = itMap->first;
		int countCopies = itMap->second;
		Graph::Edge* e = g3.edge(thisId);
		//Check that I'm considering a 0 cost directed edge
		if (countCopies == 1 && e->cost() == 0 && e->capacity() == 1) {
			//Check that I have not used the opposite direction 0 cost edge
			bool oppositeDirection = false;
			for (std::map<int, int>::iterator it = newEdges.begin(); it != newEdges.end(); it++) {
				Graph::Edge* e2 = g3.edge(it->first);

				if (it->second != 1 || e2->parentId() != e->parentId() || e2->cost() != 0 || e2->capacity() != 1)
					continue;

				if (e2->from()->id() == e->to()->id() && e2->to()->id() == e->from()->id()) {
					oppositeDirection = true;
					break;
				}
			}

			if (!oppositeDirection) {
				Graph::Vertex* vFrom = g2.vertex(e->from()->id());
				Graph::Vertex* vTo = g2.vertex(e->to()->id());
				Graph::Edge* edgeOriginal = g2.edge(e->parentId());
				Graph::Edge* new_e = g2.addEdge(vFrom, vTo, false, edgeOriginal->cost(), edgeOriginal->capacity());
				new_e->setParentId(e->parentId());
				g2.removeEdge(edgeOriginal);
				directedElements.push_back(new_e->id());

			}
			//If the opposite direction has been used, I do nothing.


		}
		else {
			Graph::Vertex* vFrom = g2.vertex(e->from()->id());
			Graph::Vertex* vTo = g2.vertex(e->to()->id());
			for (int i = 0; i < countCopies; i++) {
				//This works as duplicateLinks
				Graph::Edge* ed = g2.addEdge(vFrom, vTo, false, e->cost(), e->capacity());
				ed->setParentId(e->parentId());
				directedElements.push_back(ed->id());
				addedElements.push_back(ed->id());
			}
		}

	}

	//Insert in undirected set the remained edges
	for (Graph::EdgeIDMap::iterator it = g2.edges().begin(); it != g2.edges().end(); it++) {
		Graph::Edge* e = it->second;
		if (e->undirected()) {
			undirectedElements.push_back(e->id());
		}

	}


	*graph = g2;

	return std::tuple<std::vector<int>, std::vector<int>, std::vector<int>>(directedElements, undirectedElements, addedElements);




}
