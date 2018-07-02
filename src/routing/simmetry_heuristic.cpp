#include "routing_problem.h"


void RoutingProblem::simmetryHeuristic(Graph& graph)
{
	std::map<std::pair<int, int>, double> D;
	//std::map<std::pair<int, int>, std::vector<TopologicPathElement>> P;
	std::vector<int> negativeIDs;
	std::vector<int> positiveIDs;

	std::map<int, int> verticesSupplyMap = computeVerticesSupplyMap(graph, _notRequiredEdges);
	
	for (std::map<int, int>::iterator it = verticesSupplyMap.begin(); it != verticesSupplyMap.end(); it++) {

		for (int i = it->second; i > 0; i--) {
			positiveIDs.push_back(it->first);
		}
		for (int i = it->second; i < 0; i++) {
			negativeIDs.push_back(it->first);
		}
	}

	if (negativeIDs.size() != positiveIDs.size()) {
        std::cout<<"There are "<< negativeIDs.size() << " negativeIds and "<< positiveIDs.size()<< " positiveIds: ERROR"<<std::endl;
	}
    
    /*
	if (negativeIDs.size() != 0) {

		//Compute shortest paths between every possible couple (negative -> positive)
		computeDistanceMap(*graph, positiveIDs, negativeIDs, &D, &P, DIRECTED);

		//Use Hungarian algorithm to find best additional paths
		std::vector<int> assignement;

		std::vector <vector<double> > Dmatrix(positiveIDs.size(), vector<double>(negativeIDs.size(), std::numeric_limits<double>::infinity()));
		for (unsigned i = 0; i < positiveIDs.size(); i++) {
			for (unsigned j = 0; j < negativeIDs.size(); j++) {
				int fromID = positiveIDs[i];
				int toID = negativeIDs[j];
				Dmatrix[i][j] = D.at(std::make_pair(fromID, toID));
			}
		}

		m_hungarianSolver.Solve(Dmatrix, assignement);
		//Add new edges satisfiyng these paths to m_extendedGraph
		for (unsigned i = 0; i < negativeIDs.size(); i++) {
			std::vector<TopologicPathElement> path = P.at(std::make_pair(positiveIDs[i], negativeIDs[assignement[i]]));
			for (unsigned j = 0; j < path.size(); j++)
				duplicateLinks(path[j], graph);
		}

	}
    */
}