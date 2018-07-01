#include "routing/routing_problem.h"


RoutingProblem::RoutingProblem()
{

}

RoutingProblem::~RoutingProblem()
{

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

    }
    
    if (_type == UNDIRECTED && _notRequiredEdges.empty() && _startId == _goalId){
        std::cout<<"The graph is undirected"<<std::endl;
    }

    if (_type == MIXED  && _notRequiredEdges.empty() && _startId == _goalId){

    }


    return std::vector<int>();

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