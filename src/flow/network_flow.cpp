#include "network_flow.h"



namespace network_flow
{

	int SOURCE_ID = -1000;
    int SINK_ID = -1001;

	std::map<int, int> computeVerticesSupplyMap(const Graph graph, const Graph::EdgeSet visitedEdges)
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


	std::map<std::pair<int,int>, int> minCostMaxFlowMatching(Graph graph, std::map<int, int> verticesSupplyMap)
	{
		std::map<std::pair<int,int>, int> flowMatrix;
		if (verticesSupplyMap.empty()) {
			return flowMatrix;
		}

		Graph augmentedGraph = addSourceAndSink(graph, verticesSupplyMap);

		for (Graph::EdgeIDMap::iterator it = augmentedGraph.edges().begin(); it != augmentedGraph.edges().end(); it++) {
			Graph::Edge* e = it->second;
			Graph::PathElement forwardElement = Graph::PathElement(e->from(), e);

			flowMatrix.insert(std::make_pair(std::make_pair(e->from()->id(), e->id()), 0));
			if (e->undirected()){
				flowMatrix.insert(std::make_pair(std::make_pair(e->to()->id(), e->id()), 0));
			}
		}
		successiveShortestPath(augmentedGraph, flowMatrix);

		Graph::Vertex* sourceV = augmentedGraph.vertex(SOURCE_ID);
		Graph::Vertex* sinkV = augmentedGraph.vertex(SINK_ID);

		for (Graph::EdgeSet::iterator it = sourceV->exitingEdges().begin(); it != sourceV->exitingEdges().end(); it++) {
			Graph::Edge* e = *it;	
			std::map<std::pair<int,int>, int>::iterator itMap = flowMatrix.find(std::make_pair(SOURCE_ID, e->id()));
			if (itMap != flowMatrix.end()){
				flowMatrix.erase(itMap);			
			}
		}
		for (Graph::EdgeSet::iterator it = sinkV->enteringEdges().begin(); it != sinkV->enteringEdges().end(); it++) {
			Graph::Edge* e = *it;
			Graph::Vertex* fromV = (e->from()->id() != SINK_ID) ? e->from() : e->to();
			std::map<std::pair<int,int>, int>::iterator itMap = flowMatrix.find(std::make_pair(fromV->id(), e->id()));

			if (itMap != flowMatrix.end()){
				flowMatrix.erase(itMap);
			}
		}

		return flowMatrix;

	}


	Graph addSourceAndSink(Graph graph, std::map<int, int> verticesSupplyMap)
	{
		Graph augmentedGraph = graph;

		if (verticesSupplyMap.empty()) {
			return augmentedGraph;
		}

		Graph::Vertex* sourceV = augmentedGraph.addVertex(SOURCE_ID);
		Graph::Vertex* sinkV = augmentedGraph.addVertex(SINK_ID);

		for (std::map<int, int>::iterator it = verticesSupplyMap.begin(); it != verticesSupplyMap.end(); it++) {
		
			Graph::Vertex* v = augmentedGraph.vertex(it->first);
			int supply = it->second;

			if (supply > 0){
				augmentedGraph.addEdge(sourceV, v, false, 0.0, supply);
			}
			else if (supply < 0){
				augmentedGraph.addEdge(v, sinkV, false, 0.0, -supply);
			}

		
		}

		return augmentedGraph;

	}


	void successiveShortestPath(Graph graph, std::map<std::pair<int,int>, int>& flowMatrix)
	{

		Graph residualGraph;
		Graph::EdgeSet residualEdges;
		std::map<int, float> potentials;
		std::map<int, Graph::Edge*> predecessors;

		std::pair<Graph, Graph::EdgeSet> res = computeResidualGraph(graph, flowMatrix, potentials);
		residualGraph = res.first;
		residualEdges = res.second;

		while (true){

			shortest_paths::pathBellmanFord(residualGraph, SOURCE_ID, &potentials, &predecessors);


			std::pair<Graph, Graph::EdgeSet> res = computeResidualGraph(graph, flowMatrix, potentials);
			residualGraph = res.first;
			residualEdges = res.second;


			std::vector<int> path = shortest_paths::pathDijkstra(residualGraph, SOURCE_ID, SINK_ID);

			if (path.empty()){
				return;
			}

			int minimumCapacity = INT_MAX;
			for (int eId : path){
				Graph::Edge* e = residualGraph.edge(eId);
				if (e->capacity() < minimumCapacity){
					minimumCapacity = e->capacity();
				}
			}


			Graph::Vertex* currentV  = graph.vertex(SOURCE_ID);
			for (int eId : path){
				Graph::Edge* e = residualGraph.edge(eId);
				Graph::Edge* parentE = graph.edge(e->parentId());
				if (residualEdges.find(e) != residualEdges.end()){
					flowMatrix[std::make_pair(currentV->id(), parentE->id())] -= minimumCapacity;
				}
				else{
					flowMatrix[std::make_pair(currentV->id(), parentE->id())] += minimumCapacity;
				}

				currentV = (parentE->from()->id() == currentV->id()) ? parentE->to() : parentE->from();
			}

		}

	}



	std::pair<Graph, Graph::EdgeSet> computeResidualGraph(Graph graph, std::map<std::pair<int,int>, int>& flowMatrix, std::map<int, float> potentials)
	{
		Graph residualGraph;
		Graph::EdgeSet residualSet;

		for (Graph::VertexIDMap::iterator it = graph.vertices().begin(); it != graph.vertices().end(); it++) {
			Graph::Vertex* v = it->second;
			residualGraph.addVertex(it->first, v->position());
		}


		for (std::map<std::pair<int,int>, int>::const_iterator itMap = flowMatrix.begin(); itMap != flowMatrix.end(); itMap++) {

			std::pair<int,int> element = itMap->first;

			Graph::Edge* e = graph.edge(element.second);


			int fromID = element.first;
			int toID = (e->to()->id() != fromID) ? e->to()->id() : e->from()->id();

			Graph::Vertex* vFromResidual = residualGraph.vertex(fromID);
			Graph::Vertex* vToResidual = residualGraph.vertex(toID);

			float vFromPotential = 0;
			float vToPotential = 0;
			if (!potentials.empty()) {
				vFromPotential = potentials[fromID];
				vToPotential = potentials[toID];
			}


			float cost = e->cost() + vFromPotential - vToPotential;
			float residualCost = (potentials.empty()) ? -cost : 0.0;
			int flow = itMap->second;
			int capacity = e->capacity();
			int residualCapacity = capacity - flow;


			if (residualCapacity > 0) {
				Graph::Edge* new_e = residualGraph.addEdge(vFromResidual, vToResidual, false, cost, residualCapacity);
				new_e->setParentId(e->id());
			}
			if (flow > 0) {
				Graph::Edge* new_e = residualGraph.addEdge(vToResidual, vFromResidual, false, residualCost, flow);
				new_e->setParentId(e->id());
				residualSet.insert(e);
			}

		}

		return std::make_pair(residualGraph, residualSet);
	}



	std::vector<int> naivePairsAssignment(std::vector<int> elements, std::map<std::pair<int, int>, float> distanceMap)
	{
		

		std::vector<std::vector<int>> perms = allAssignments(elements.size(), 2);

		float minCost =  std::numeric_limits<float>::infinity(); 
		std::vector<int> best_assignment;

		for (int i = 0; i < perms.size(); i++){

			std::vector<int> assignment = perms[i];
			float cost = 0;

			for (int j = 1; j <= elements.size()/2; j++){
				int firstId = -1;
				int secondId = -1;

				for (int k = 0; k < assignment.size();k++){
					int label = assignment[k];
					if (label == j && firstId == -1){
						firstId = k; 
					}
					else if (label == j && secondId == -1){
						secondId = k;
					}
				}

				assert (firstId >= 0 && secondId >= 0);

				cost += distanceMap[std::make_pair(firstId, secondId)];
				
			}

			if (cost < minCost){
				minCost = cost;
				best_assignment = assignment;
			}

		}

		return best_assignment;
	}


}
	