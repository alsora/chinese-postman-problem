#include "graph_factory.h"
#include <eigen3/Eigen/Dense>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include "graph/graph_utils.h"

using namespace Eigen;



std::pair<Graph, std::set<int>> buildGraph1()
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


    return std::make_pair(graph, eset);

}


std::pair<Graph, std::set<int>> buildRandomGraph()
{

    Graph graph;

    int nVertices = 20;
    int nEdges =40;
    int nVisited = 0;
    bool undirected = false;



    float maxX = 20;
    float minX = 0;
    float maxY = 20;
    float minY = 0;


    for (int i = 1; i <= nVertices; i++){

        float x = minX + (rand() % static_cast<int>(maxX - minX + 1));
        float y = minY + (rand() % static_cast<int>(maxY - minY + 1));
        //float x = minX + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxX-minX)));
        //float y = minY + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxY-minY)));

        graph.addVertex(i, Vector2f(x, y));

    }


    for (int i = 0; i < nEdges; ){

        int v1 = 1 + (rand() % static_cast<int>(nVertices - 1 + 1));
        int v2 = 1 + (rand() % static_cast<int>(nVertices - 1 + 1));

        if (v1 == v2){
            continue;
        }

        std::set<int> edges = graph.getEdgesBetweenVertices(v1, v2);

        if (edges.empty()){
            graph.addEdge(v1, v2, undirected);
            i++;
        }

    }


    int k = 0;
    for (int i = 1; i <= nVertices; i++){
         Graph::Vertex* v = graph.vertex(i);
        if (v->edges().empty()){
            graph.removeVertex(v);
        }

    }

    std::set<int> eset;
    for (auto it = graph.edges().begin(); it != graph.edges().end(); it++){
        if (eset.size()>=nVisited){
            break;
        }
        eset.insert((*it).first);
    }



    return std::make_pair(graph, eset);

}




