#include "routing/routing_problem.h"


RoutingProblem::RoutingProblem()
{

}

RoutingProblem::~RoutingProblem()
{
    clear();
}

void RoutingProblem::init(Graph graph, int startId, int goalId, Graph::EdgeSet notRequiredEdges)
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

    _eulerianExtendedGraph = Graph(graph);

    _type = detectGraphType(graph);
    
}

std::vector<int> RoutingProblem::solve()
{
    if (_type == DIRECTED && _notRequiredEdges.empty() && _startId == _goalId){
        std::cout<<"The graph is directed"<<std::endl;

        simmetryHeuristic(&_eulerianExtendedGraph);

        return hierholzerSolver(_eulerianExtendedGraph, _startId, _goalId);
    }
    
    if (_type == UNDIRECTED && _notRequiredEdges.empty() && _startId == _goalId){
        std::cout<<"The graph is undirected"<<std::endl;
    }

    if (_type == MIXED  && _notRequiredEdges.empty() && _startId == _goalId){
        std::cout<<"The graph is mixed"<<std::endl;

    }


    return std::vector<int>();

}

std::vector<int> RoutingProblem::hierholzerSolver(Graph& graph, int startId, int goalId){

    std::vector<int> circuit;
    std::stack<int> currentPath;

    Graph requiredGraph = graph;

    for (Graph::EdgeIDMap::iterator it = graph.edges().begin(); it != graph.edges().end();it++){
        if (_notRequiredEdges.find(it->second) != _notRequiredEdges.end()){
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


GraphType RoutingProblem::detectGraphType(Graph graph)
{
    bool hasEdge = false;
    bool hasArc = false;

    Graph::EdgeIDMap edges = graph.edges();
    for (Graph::EdgeIDMap::const_iterator it = edges.begin(); it != edges.end(); it++){
        Graph::Edge* e = it->second;

        if (e->undirected()){
            hasEdge = true;
        }
        else {
            hasArc = true;
        }

    }

    if (!hasArc)
        return UNDIRECTED;
    else if (hasArc && !hasEdge)
        return DIRECTED;
    else
        return MIXED;

}

void RoutingProblem::clear()
{
    delete &_originalGraph;
    delete &_eulerianExtendedGraph;
    _notRequiredEdges.clear();

    _startId = Graph::UnassignedId;
    _goalId = Graph::UnassignedId;
}