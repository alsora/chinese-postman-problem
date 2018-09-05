

#include "graph/graph.h"
#include "graph/graph_utils.h"
#include <map>


namespace eulerian_extension
{

        
    Graph extend(Graph& graph, graph_utils::GraphType type, std::set<int> travelEdges);

    void heuristicExtension(Graph* graph, graph_utils::GraphType type, std::set<int> travelEdges = std::set<int>());

    void evenDegreeHeuristic(Graph* graph, std::set<int> not_required) ;
    
    std::map<int, int> simmetryHeuristic(Graph* graph, std::set<int> not_required);

    std::tuple<std::vector<int>, std::vector<int>, std::vector<int>> fredericksonInOutDegree(Graph* graph, std::set<int> notRequiredEdges);

    std::set<int> ruralSolver(Graph* graph, std::set<int> notRequired, graph_utils::GraphType type);



}