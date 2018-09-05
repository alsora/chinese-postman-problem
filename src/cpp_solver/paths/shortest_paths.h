#ifndef __SHORTEST_PATHS_H__
#define __SHORTEST_PATHS_H__


#include <map>

namespace shortest_paths
{

    std::vector<int> pathDijkstra(Graph graph, int startId, int goalId);
    void mapDijkstra(const Graph graph, std::vector<int> startVerticesID, std::vector<int> endVerticesID, std::map<std::pair<int, int>, float>* D, std::map<std::pair<int, int>, std::vector<int>>* P);

    void pathBellmanFord(Graph graph, int startId, std::map<int, float>* D, std::map<int, Graph::Edge*>* P);


    void mapFloydWarshall(Graph graph, std::vector<int> verticesID, std::map<std::pair<int,int>, float>* D2, std::map<std::pair<int, int>, std::vector<int>>* H2);


}

#endif