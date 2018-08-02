#include <iostream>
#include <eigen3/Eigen/Dense>
#include "graph/graph.h"
#include "routing/routing_problem.h"
#include "opencv_utilities.h"

using namespace Eigen;

int main()
{

    Graph graph;

    graph.addVertex(1, Vector2f(0, 10));
    graph.addVertex(2, Vector2f(10, 10));
    graph.addVertex(3, Vector2f(5, 2.5));
    graph.addVertex(4, Vector2f(0, 0));
    graph.addVertex(5, Vector2f(10, 0));

    bool undirected = true;

    graph.addEdge(3, 1, undirected);
    graph.addEdge(1, 2, undirected);
    graph.addEdge(2, 3, undirected);
    graph.addEdge(3, 4, undirected);
    Graph::Edge* e = graph.addEdge(5, 2, undirected);
    graph.addEdge(5, 3, undirected);
    graph.addEdge(4, 5, undirected);


    std::set<int> eset;
    eset.insert(e->id());

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


    int startId = 1;
    
    RoutingProblem routing = RoutingProblem();
    routing.init(graph, startId, startId, eset);
    std::vector<int> circuit = routing.solve();
    std::cout<<"Eulerian circuit:"<<std::endl;
    for (int eId : circuit){
        Graph::Edge* e = graph.edge(eId);
        std::cout<<e->from()->id()<<" "<<e->to()->id()<<std::endl;
    }


    animatePath(graph, circuit, startId);

}