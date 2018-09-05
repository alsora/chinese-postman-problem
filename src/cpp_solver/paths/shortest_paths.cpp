

#include <unordered_map>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>
#include "graph/graph.h"

#include "shortest_paths.h"

namespace shortest_paths
{
    using namespace std;

    vector<int> pathDijkstra(Graph graph, int startId, int goalId)
    {
        unordered_map<int, float> distances;
        unordered_map<int, Graph::PathElement> previous;

        vector<int> vertices;
        vector<int> path;

        auto comparator = [&] (int left, int right) {return distances[left] > distances[right]; };

	    for (Graph::VertexIDMap::iterator itV = graph.vertices().begin(); itV != graph.vertices().end(); itV++) {
            
            if (itV->first == startId){
                distances[itV->first] = 0;
            }
            else{
                distances[itV->first] = numeric_limits<float>::max();
            }

            vertices.push_back(itV->first);
            push_heap(begin(vertices), end(vertices), comparator);
        }

        while (!vertices.empty()){
            pop_heap(begin(vertices), end(vertices), comparator);
            int smallestId = vertices.back();
            vertices.pop_back();

            if (smallestId == goalId){
                while(previous.find(smallestId) != end(previous)){
                    Graph::PathElement prevElement = previous[smallestId];
                    Graph::Vertex* prevV = prevElement.first;
                    Graph::Edge* prevE = prevElement.second; 
                    path.push_back(prevE->id());
                    smallestId = prevV->id();
                }

                break;
            }

            if (distances[smallestId] == numeric_limits<float>::max()){
                break;
            }

            for (Graph::PathElement reachablePair : graph.vertex(smallestId)->reachable()){
                Graph::Vertex* v = reachablePair.first;
                Graph::Edge* e = reachablePair.second;
                float alternativePath = distances[smallestId] + e->cost();
                if (alternativePath < distances[v->id()]){
                    distances[v->id()] = alternativePath;
                    previous[v->id()] = Graph::PathElement(graph.vertex(smallestId), e);
                    make_heap(begin(vertices), end(vertices), comparator);
                }

            }

        }


        std::reverse(path.begin(), path.end());


        return path;
    }



    void mapDijkstra(const Graph graph, vector<int> startVerticesId, vector<int> endVerticesId, map<pair<int, int>, float>* D, map<pair<int, int>, vector<int>>* P)
    {
        
        for (int startId : startVerticesId){

            for (int endId : endVerticesId){

                vector<int> path = pathDijkstra(graph, startId, endId);
                float cost = 0.0;
                for (int eId : path){
                    cost += graph.edge(eId)->cost();
                }

                if (path.empty() && startId != endId){
                    cost = numeric_limits<float>::infinity();
                }
                (*D)[make_pair(startId, endId)] = cost;
                (*P)[make_pair(startId, endId)] = path;

            }

        }

    }



    void pathBellmanFord(Graph graph, int startId, map<int, float>* D, std::map<int, Graph::Edge*>* P)
    {

        Graph::VertexIDMap vertices = graph.vertices();
        Graph::EdgeIDMap edges = graph.edges();

        for (Graph::VertexIDMap::iterator it = vertices.begin(); it != vertices.end(); it++){
            (*D)[it->first] = std::numeric_limits<float>::infinity();
            (*P)[it->first] = nullptr;
        }
        
        (*D)[startId] = 0.0;

        for (Graph::VertexIDMap::iterator itV = vertices.begin(); itV != vertices.end(); itV++){

            for (Graph::EdgeIDMap::iterator itE = edges.begin(); itE != edges.end(); itE ++){

                Graph::Edge* e = itE->second;
                int fromId = e->from()->id();
                int toId = e->to()->id();

                if ((*D)[fromId] + e->cost() < (*D)[toId]){
                    (*D)[toId] = (*D)[fromId] + e->cost();
                    (*P)[toId] = e;
                }
                if (e->undirected() && ((*D)[toId] + e->cost() < (*D)[fromId])){
                    (*D)[fromId] = (*D)[toId] + e->cost();
                    (*P)[fromId] = e;
                }

            }

        }

        for (Graph::EdgeIDMap::iterator itE = edges.begin(); itE != edges.end(); itE ++){
            
            Graph::Edge* e = itE->second;
            int fromId = e->from()->id();
            int toId = e->to()->id();

            assert((*D)[fromId] + e->cost() >= (*D)[toId]);

            assert(!e->undirected() || ((*D)[toId] + e->cost() >= (*D)[fromId]));

        }

    }



    void mapFloydWarshall(Graph graph, std::vector<int> verticesID, std::map<std::pair<int,int>, float>* D2, std::map<std::pair<int, int>, std::vector<int>>* H2)
    {

        float inf = std::numeric_limits<float>::infinity();

        std::vector<std::vector<float>> D;
        std::vector<std::vector<int>> H;

        //Initialize distance map to infinite
        D = std::vector<std::vector<float>>(verticesID.size(), vector<float>(verticesID.size(), inf));
        H = std::vector<std::vector<int>>(verticesID.size(), vector<int>(verticesID.size(), Graph::UnassignedId));

        //Set values for existing edges
        for (auto it = graph.edges().begin(); it != graph.edges().end(); it++) {

            Graph::Edge* e = it->second;

            int fromID = e->from()->id();
            int toID = e->to()->id();

            ptrdiff_t fromPos = find(verticesID.begin(), verticesID.end(), fromID) - verticesID.begin();
            ptrdiff_t toPos = find(verticesID.begin(), verticesID.end(), toID) - verticesID.begin();

            D[fromPos][toPos] = e->cost();
            H[fromPos][toPos] = it->first;

            if (e->undirected()) {
                D[toPos][fromPos] = e->cost();
                H[toPos][fromPos] = it->first;
            }

        }
        //Set value from a node to itself
        for (unsigned i = 0; i < verticesID.size(); i++) {
            D[i][i] = 0;
        }

        //Floyd Warshall algorithm
        for (unsigned k = 0; k < verticesID.size(); k++) {
            for (unsigned i = 0; i < verticesID.size(); i++) {
                for (unsigned j = 0; j < verticesID.size(); j++) {

                    if ((D[k][j] != inf) && (D[j][i] != inf) && (D[j][j] < 0)) {
                        D[k][i] = -inf;
                        continue;
                    }

                    if ((D[i][k] + D[k][j]) < D[i][j]) {
                        D[i][j] = D[i][k] + D[k][j];
                        H[i][j] = H[k][j];
                    }

                }
            }
        }


        for (int i = 0; i < verticesID.size(); i ++){
            for (int j = 0; j < verticesID.size();j++){


                int fromID = verticesID[i];
                int toID = verticesID[j];


                if (i == j){
                    (*H2)[std::make_pair(fromID, toID)] = std::vector<int>();
                    (*D2)[std::make_pair(fromID, toID)] = 0;
                    continue;
                }

                int fromPos = i;
                int toPos = j;
                
                (*D2)[std::make_pair(fromID, toID)] = D[i][j];

                std::vector<int> path;
                int predecessorID = Graph::UnassignedId;
                int successiveID = Graph::UnassignedId;
                int predecessorEdgeHash;

                while (predecessorID != fromID) {

                    predecessorEdgeHash = H[fromPos][toPos];

                    Graph::Edge* e = graph.edge(predecessorEdgeHash);

                    if (!e->undirected() || (e->to()->id() == predecessorID) || (e->to()->id() == toID)) {
                        predecessorID = e->from()->id();
                        successiveID = e->to()->id();
                    }
                    else {
                        predecessorID = e->to()->id();
                        successiveID = e->from()->id();
                    }

                    toPos = find(verticesID.begin(), verticesID.end(), predecessorID) - verticesID.begin();

                    path.push_back(predecessorEdgeHash);

                }

                std::reverse(path.begin(), path.end());

                (*H2)[std::make_pair(fromID, toID)] = path;

            }
        }
    

    }



}



