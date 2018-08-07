#include "routing/routing_problem.h"
#include "eulerian_extension.h"
#include <stack>
#include <iostream>

RoutingProblem::RoutingProblem()
{

}

RoutingProblem::~RoutingProblem()
{
    clear();
}

void RoutingProblem::init(Graph graph, int startId, int goalId, std::set<int> notRequiredEdges)
{
    _originalGraph = Graph(graph);
    _startId = startId;
    if (goalId == Graph::UnassignedId){
        _goalId = _startId;
    }
    else{
        _goalId = goalId;
    }

    _notRequiredEdges = notRequiredEdges;

    _type = graph_utils::detectGraphType(graph);
    
}


std::vector<int> RoutingProblem::solve()
{   

    return solve(_originalGraph, _type, _startId, _goalId, _notRequiredEdges);

}

std::vector<int> RoutingProblem::solve(Graph& g, graph_utils::GraphType type, int startId, int goalId, std::set<int> travelEdges)
{
    
    _eulerianExtendedGraph = Graph(g);

    std::vector<int> circuit;

    if (startId == goalId){

        eulerian_extension::extend(_eulerianExtendedGraph, type, travelEdges);

        circuit = hierholzerSolver(_eulerianExtendedGraph, startId);

    }
    else {

        //Add artificial edge
        bool undirected = _type == graph_utils::UNDIRECTED;
        Graph::Edge* virtualEdge = _eulerianExtendedGraph.addEdge(_goalId, _startId, undirected, 9999999);
        int virtualEdgeId = virtualEdge->id();

        eulerian_extension::extend(_eulerianExtendedGraph, type, travelEdges);

        circuit = hierholzerSolver(_eulerianExtendedGraph, _goalId);

        //Remove artificial edge
        circuit = adjustOpenProblem(virtualEdgeId, circuit);

    }



    return circuit;
}


std::vector<int> RoutingProblem::adjustOpenProblem(int virtualEdgeId, std::vector<int> eulerianCircuit)
{

    ptrdiff_t artificialEdgePosition = find(eulerianCircuit.begin(), eulerianCircuit.end(), virtualEdgeId) - eulerianCircuit.begin();
    assert (artificialEdgePosition < eulerianCircuit.size());

    std::vector<int> verticesCircuit = graph_utils::pathEdgesToVertices(eulerianCircuit, _eulerianExtendedGraph, _goalId);

    int fromId = verticesCircuit[artificialEdgePosition];
    int toId = verticesCircuit[artificialEdgePosition + 1];

    assert ((fromId == _goalId && toId == _startId) || (fromId == _startId && toId == _goalId));

	if (fromId == _startId) {

		//(g...s)->g transformed into (s...g)
		if (artificialEdgePosition == eulerianCircuit.size() - 1) {
			eulerianCircuit.pop_back();
			std::reverse(eulerianCircuit.begin(), eulerianCircuit.end());
		}
		//(g...s)->g...g transformed into (s...g) ...g
		else {
			std::vector<int> firstPart = std::vector<int>(eulerianCircuit.begin(), eulerianCircuit.begin() + artificialEdgePosition);
			std::vector<int> secondPart = std::vector<int>(eulerianCircuit.begin() + artificialEdgePosition + 1, eulerianCircuit.end());

			std::reverse(firstPart.begin(), firstPart.end());

			eulerianCircuit = firstPart;
			eulerianCircuit.insert(eulerianCircuit.end(), secondPart.begin(), secondPart.end());
		}
	}
	else if (fromId == _goalId) {

		//g->(s...g) transformed into (s...g)
		if (artificialEdgePosition == 0) {
			eulerianCircuit.erase(eulerianCircuit.begin());
		}
		//g...g->(s...g) transformed into (s...g) ...g
		else {
			std::vector<int> firstPart = std::vector<int>(eulerianCircuit.begin(), eulerianCircuit.begin() + artificialEdgePosition);
			std::vector<int> secondPart = std::vector<int>(eulerianCircuit.begin() + artificialEdgePosition + 1, eulerianCircuit.end());

			eulerianCircuit = secondPart;
			eulerianCircuit.insert(eulerianCircuit.end(), firstPart.begin(), firstPart.end());
		}

	}



    return eulerianCircuit;
    
}



std::vector<int> RoutingProblem::hierholzerSolver(Graph& graph, int startId){

    std::vector<int> circuit;
    std::stack<int> currentPath;

    Graph requiredGraph = graph;

    for (Graph::EdgeIDMap::iterator it = graph.edges().begin(); it != graph.edges().end();it++){
        if (_notRequiredEdges.find(it->second->id()) != _notRequiredEdges.end()){
            int eId = it->first;
            Graph::Edge* e = requiredGraph.edge(eId);
            requiredGraph.removeEdge(e);
        }
    }

    int vId = startId;

    do {
        Graph::Vertex* currentV = requiredGraph.vertex(vId);
        Graph::EdgeSet edges = currentV->exitingEdges();

        if (edges.size() > 0){
            Graph::Edge* e = (*edges.begin());
            currentPath.push(e->id());
            vId = (e->from()->id() == currentV->id()) ? e->to()->id() : e->from()->id();
            requiredGraph.removeEdge(e);
            
        }
        else if (!currentPath.empty()) {
            int previousEdgeId = currentPath.top();
            currentPath.pop();

            Graph::Edge* previousEdge = graph.edge(previousEdgeId);
            vId = (previousEdge->from()->id() == currentV->id()) ? previousEdge->to()->id() : previousEdge->from()->id();
            circuit.push_back(previousEdge->parentId());
        }


    } while (!currentPath.empty());


    std::reverse(circuit.begin(), circuit.end());


    return circuit;

}


void RoutingProblem::clear()
{
    //delete &_originalGraph;
    //delete &_eulerianExtendedGraph;
    _notRequiredEdges.clear();

    _startId = Graph::UnassignedId;
    _goalId = Graph::UnassignedId;
}