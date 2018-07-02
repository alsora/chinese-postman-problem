#include <iostream>
#include <eigen3/Eigen/Dense>
#include "graph/graph.h"
#include "routing/routing_problem.h"

using namespace Eigen;

int main()
{

    Graph graph;

    graph.addVertex(1, Vector2f(0, 10));
    graph.addVertex(2, Vector2f(10, 10));
    graph.addVertex(3, Vector2f(5, 2.5));
    graph.addVertex(4, Vector2f(0, 0));
    graph.addVertex(5, Vector2f(10, 0));

    graph.addEdge(3, 1);
    graph.addEdge(1, 2);
    graph.addEdge(2, 3);
    graph.addEdge(3, 4);
    graph.addEdge(5, 2);
    graph.addEdge(5, 3);
    graph.addEdge(4, 5);


    /**
     *
     * 1------->2
     *  ^      /^
     *   \    / |
     *    \  <  |
     *     3    |
     *    / ^   |
     *  <     \ |
     * 4------->5
     **/


    RoutingProblem routing = RoutingProblem();
    routing.init(graph, 1);
    routing.solve();

}