#include "graph_utils.h"
#include "iostream"
#include "paths/shortest_paths.h"



namespace graph_utils
{
    using namespace std;


    float eulerianCost(Graph graph, std::vector<int> skipEdges)
    {

        float cost = 0;

        for (Graph::EdgeIDMap::iterator it = graph.edges().begin(); it != graph.edges().end(); it++) {
            Graph::Edge* e = it->second;
            if (std::find(skipEdges.begin(), skipEdges.end(), e->id()) == skipEdges.end()) 
                cost += e->cost();
        }

        return cost;

    }


    GraphType detectGraphType(Graph graph)
    {
        bool hasEdge = false;
        bool hasArc = false;

        Graph::EdgeIDMap edges = graph.edges();
        for (Graph::EdgeIDMap::const_iterator it = edges.begin(); it != edges.end(); it++){
            Graph::Edge* e = it->second;

            if (e->undirected()){
                hasEdge = true;
            }
            else {
                hasArc = true;
            }

        }

        if (!hasArc)
            return UNDIRECTED;
        else if (hasArc && !hasEdge)
            return DIRECTED;
        else
            return MIXED;

    }


    void refineEdges(Graph* graph, std::set<int> edges)
    {
        std::vector<std::pair<int,int>> optsExtremes;

        Graph noOtpGraph = *graph;

        for (int edgeId : edges){
            Graph::Edge* otp = noOtpGraph.edge(edgeId);
            if (otp == nullptr){
                continue;
            }
            noOtpGraph.removeEdge(otp);
        }

        for (int edgeId : edges){

            Graph::Edge* edgeHere = graph->edge(edgeId);
            Graph::Edge* edgeThere = noOtpGraph.edge(edgeId);
            if (edgeThere == nullptr){
                continue;
            }
            
            int fromId = edgeHere->from()->id();
            int toId = edgeHere->to()->id();
            bool undirected = edgeHere->undirected();

            std::vector<int> path = shortest_paths::pathDijkstra(noOtpGraph, fromId, toId);

            for (int eId : path){
                Graph::Edge* eP = noOtpGraph.edge(eId);
                Graph::Vertex* fromV = eP->from();
                Graph::Vertex* toV = eP->to();

                Graph::Edge* duplicatedEdge = noOtpGraph.addEdge(fromV->id(), toV->id(), eP->undirected(), eP->cost(), eP->capacity());
                duplicatedEdge->setParentId(eP->parentId());
            }


        }

    }



    void printVerticesInfo(Graph g)
    {
        for (Graph::VertexIDMap::const_iterator it = g.vertices().begin(); it != g.vertices().end(); it++){
            Graph::Vertex* v = it->second;
            std::cout<< "Vertex " << v->id() << " at " << v->position().transpose();
            if (v->enteringEdges().size() > 0){
                std::cout <<" +1:";
                for (Graph::Edge* e : v->enteringEdges()){
                    std::cout<<" e " << e->from()->id()<< " " << e->to()->id();
                }
            }
            if (v->exitingEdges().size() > 0){
                std::cout <<" -1:";
                for (Graph::Edge* e : v->exitingEdges()){
                    std::cout<<" e " << e->from()->id()<< " " << e->to()->id();
                }
            }
            std::cout<<std::endl;
        }
    }


    void printEdgesInfo(Graph g)
    {
        for (Graph::EdgeIDMap::const_iterator it = g.edges().begin(); it != g.edges().end(); it++){
            Graph::Edge* e = it->second;
            std::cout<< "Edge " << e->from()->id() << " "<< e->to()->id() << (e->undirected() ? " Undirected" : " Directed") << " Cost: " << e->cost() << " Capacity: " << (e->capacity() == INT_MAX ? std::numeric_limits<float>::infinity() : e->capacity()) << " Id " << e->id();
            if (e->parentId() != e->id()){
                std::cout << " ParentId: " << e->parentId();
            }
            std::cout<<std::endl;
        }
    }







}