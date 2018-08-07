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

    bool undirected = false;

    Graph::Edge* e1 = graph.addEdge(3, 1, undirected);
    Graph::Edge* e2 = graph.addEdge(1, 2, undirected);
    Graph::Edge* e3 = graph.addEdge(2, 3, undirected);
    Graph::Edge* e4 = graph.addEdge(3, 4, undirected);
    Graph::Edge* e5 = graph.addEdge(5, 2, undirected);
    Graph::Edge* e6 = graph.addEdge(5, 3, undirected);
    Graph::Edge* e7 = graph.addEdge(4, 5, undirected);


    std::set<int> eset;
    eset.insert(e5->id());

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


    int startId = 3;
    int goalId = 5;
    
    RoutingProblem routing = RoutingProblem();
    routing.init(graph, startId, goalId, eset);
    std::vector<int> circuit = routing.solve();

    std::vector<int> verticesCircuit = graph_utils::pathEdgesToVertices(circuit, graph, startId);
    std::cout<<"Eulerian circuit:"<<std::endl;
    for (int vId : verticesCircuit){
        std::cout<<vId <<std::endl;
    }


    animatePath(graph, circuit, startId);

}