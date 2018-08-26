#include <iostream>
#include "graph/graph.h"
#include "routing/routing_problem.h"
#include "graph/graph_factory.h"
#include "graph/graph_utils.h"
#include "utilities.h"
#ifdef OPENCV_LIBRARY_FOUND
#include "opencv_utilities.h"
#endif
using namespace Eigen;

int main()
{

    std::pair<Graph, std::set<int>> res = buildRandomGraph();
    Graph graph = res.first;
    std::set<int> eset = res.second;
    
    graph_utils::printEdgesInfo(graph);

    int startId = 1;
    int goalId = 1;
    
    RoutingProblem routing = RoutingProblem();
    routing.init(graph, startId, goalId, eset);

    std::vector<int> circuit = routing.solve();

    std::vector<int> verticesCircuit = graph_utils::pathEdgesToVertices(circuit, graph, startId);
    for (int vId : verticesCircuit){
        std::cout<<vId <<std::endl;
    }

    #ifdef OPENCV_LIBRARY_FOUND
    animatePath(graph, circuit, startId);
    #endif

    
}