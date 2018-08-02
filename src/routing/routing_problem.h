#ifndef __ROUTING_PROBLEM_H__
#define __ROUTING_PROBLEM_H__

#include "graph/graph.h"
#include "graph/graph_utils.h"



class RoutingProblem
{

    public:
        RoutingProblem();
        ~RoutingProblem();
        void clear();

        void init(const Graph graph, const int startId, int goalId = Graph::UnassignedId, std::set<int> travelEdges = std::set<int>());

        std::vector<int> solve();

    private: 

      

        std::vector<int> hierholzerSolver(Graph& graph, int startNodeId, int goalNodeId = Graph::UnassignedId);


        int _startId;
        int _goalId;
        std::set<int> _notRequiredEdges;

        Graph _originalGraph;
        graph_utils::GraphType _type;
        Graph _eulerianExtendedGraph;

};



#endif