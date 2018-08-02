

#include "graph/graph.h"
#include "graph/graph_utils.h"
#include <map>


namespace eulerian_extension
{

        
    void evenDegreeHeuristic(Graph* graph, std::set<int> not_required) ;
    
    std::map<int, int> simmetryHeuristic(Graph* graph, std::set<int> not_required);

    std::tuple<std::vector<int>, std::vector<int>, std::vector<int>> fredericksonInOutDegree(Graph* graph, std::set<int> notRequiredEdges);

    std::set<int> ruralSolver(Graph* graph, std::set<int> notRequired, graph_utils::GraphType type);



}