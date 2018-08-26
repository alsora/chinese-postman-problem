#include "eulerian_extension.h"
#include "paths/shortest_paths.h"
#include "paths/network_flow.h"
#include "branch_bound.h"
#include <queue>

namespace eulerian_extension
{


        
    Graph extend(Graph& graph, graph_utils::GraphType type, std::set<int> travelEdges)
    {

        if (travelEdges.empty()){

            heuristicExtension(&graph, type, travelEdges);

        }
        else {

            if (type == graph_utils::MIXED){

            }
            else {
                std::set<int> otps = eulerian_extension::ruralSolver(&graph, travelEdges, type);

                graph_utils::refineEdges(&graph, otps);
            }
        }

        return graph;

    }


    void heuristicExtension(Graph* graph, graph_utils::GraphType type, std::set<int> travelEdges)
    {
            
         if (type == graph_utils::DIRECTED){

            eulerian_extension::simmetryHeuristic(graph, travelEdges);
        }                
        else if (type == graph_utils::UNDIRECTED){
            eulerian_extension::evenDegreeHeuristic(graph, travelEdges);

        }
        else if (type == graph_utils::MIXED){
        
        }

    }


    void evenDegreeHeuristic(Graph* graph, std::set<int> notRequiredEdges)
    {

        std::vector<int> oddDegreeVertices;
        Graph allEdgesGraph = *graph;

        for (Graph::EdgeIDMap::iterator it = allEdgesGraph.edges().begin(); it != allEdgesGraph.edges().end(); it++){
            Graph::Edge* e = it->second;
            e->setUndirected(true);
        }

        for (Graph::VertexIDMap::iterator itV = allEdgesGraph.vertices().begin(); itV != allEdgesGraph.vertices().end(); itV++){

            int degree = 0;
            Graph::Vertex* v = itV->second;
            for (Graph::EdgeSet::iterator itE = v->edges().begin(); itE != v->edges().end(); itE++){
                Graph::Edge* e = *itE;
                if (notRequiredEdges.find(e->id()) == notRequiredEdges.end()){
                    degree ++;
                }
            }

            if (degree % 2 != 0){
                oddDegreeVertices.push_back(v->id());
            }
        }

        assert (oddDegreeVertices.size() % 2 == 0);

        if (oddDegreeVertices.empty()){
            return;
        }

        std::map<std::pair<int, int>, float> D;
        std::map<std::pair<int, int>, std::vector<int>> P;
        
        shortest_paths::mapDijkstra(allEdgesGraph, oddDegreeVertices, oddDegreeVertices, &D, &P);

        for (unsigned i = 0; i < oddDegreeVertices.size(); i++){
            int vId = oddDegreeVertices[i];
            D[std::make_pair(vId, vId)] = std::numeric_limits<float>::infinity(); 
        }


        std::vector<int> bestAssignment = network_flow::naivePairsAssignment(oddDegreeVertices, D);

        for (int j = 0; j < bestAssignment.size(); j += 2){
            
            int firstElement = bestAssignment[j];
            int firstId = oddDegreeVertices[firstElement];
            int secondElement = bestAssignment[j + 1];
            int secondId = oddDegreeVertices[secondElement];

            assert (firstId >= 0 && secondId >= 0);

            std::vector<int> path = P[std::make_pair(firstId, secondId)];

            for (int eId : path){
                Graph::Edge* e = graph->edge(eId);
                Graph::Vertex* fromV = graph->vertex(e->from()->id());
                Graph::Vertex* toV = graph->vertex(e->to()->id());
                Graph::Edge* duplicatedEdge = graph->addEdge(fromV->id(), toV->id(), e->undirected(), e->cost(), e->capacity());
                duplicatedEdge->setParentId(e->parentId());
            }

        }

    }

    std::map<int, int> simmetryHeuristic(Graph* graph, std::set<int> notRequiredEdges)
    {

        std::vector<int> verticesID;
        std::map<int, int> addedEdges;
        
        std::map<int, int> verticesSupplyMap = network_flow::computeVerticesSupplyMap(*graph, notRequiredEdges);

        std::map<std::pair<int,int>, int> flowMatrix = network_flow::minCostMaxFlowMatching(*graph, verticesSupplyMap);

        for (std::map<std::pair<int,int>, int>::iterator itMap = flowMatrix.begin(); itMap != flowMatrix.end(); itMap++) {

            int count = itMap->second;

            assert (count >= 0);

            std::pair<int,int> element = itMap->first;
            Graph::Vertex* fromV = graph->vertex(element.first);
            Graph::Edge* e = graph->edge(element.second);
            Graph::Vertex* toV = (e->from()->id() == fromV->id()) ? e->to() : e->from();
            for (int i = 0; i < count; i ++){
                Graph::Edge* duplicatedEdge = graph->addEdge(fromV->id(), toV->id(), e->undirected(), e->cost(), e->capacity());
                duplicatedEdge->setParentId(e->parentId());
            }

            if (count != 0){
                addedEdges[e->id()] = count;
            }		

        }

        return addedEdges;
    }



    std::tuple<std::vector<int>, std::vector<int>, std::vector<int>> fredericksonInOutDegree(Graph* graph, std::set<int> notRequiredEdges)
    {

        std::vector<int> directedElements;
        std::vector<int> undirectedElements;
        std::vector<int> addedElements;

        Graph g2 = *graph;
        Graph g3 = *graph;


        //Loop on original edges, since I'm adding edges in g2. quadruplicate edges and add arcs to directed elements
        for (Graph::EdgeIDMap::iterator it = graph->edges().begin(); it != graph->edges().end(); it++) {
            Graph::Edge* e =it->second;
            if (!e->undirected()) {
                directedElements.push_back(e->id());
                continue;
            }
                
            Graph::Vertex* vFrom = g3.vertex(e->from()->id());
            Graph::Vertex* vTo = g3.vertex(e->to()->id());
            //Substitute old edge with 2 pairs of arcs
            Graph::Edge* newArc;

            newArc = g3.addEdge(vFrom, vTo, false, e->cost(), e->capacity());
            newArc->setParentId(e->id());
            newArc = g3.addEdge(vTo, vFrom, false, e->cost(), e->capacity());
            newArc->setParentId(e->id());
            newArc = g3.addEdge(vFrom, vTo, false, 0.0, 1);
            newArc->setParentId(e->id());
            newArc = g3.addEdge(vTo, vFrom, false, 0.0, 1);
            newArc->setParentId(e->id());
            g3.removeEdge(g3.edge(e->id()));
        }

        
        std::map<int, int> newEdges = simmetryHeuristic(&g3, notRequiredEdges);


        for (std::map<int, int>::iterator itMap = newEdges.begin(); itMap != newEdges.end(); itMap++) {
            //Add new edges to original graph
            int thisId = itMap->first;
            int countCopies = itMap->second;
            Graph::Edge* e = g3.edge(thisId);
            //Check that I'm considering a 0 cost directed edge
            if (countCopies == 1 && e->cost() == 0 && e->capacity() == 1) {
                //Check that I have not used the opposite direction 0 cost edge
                bool oppositeDirection = false;
                for (std::map<int, int>::iterator it = newEdges.begin(); it != newEdges.end(); it++) {
                    Graph::Edge* e2 = g3.edge(it->first);

                    if (it->second != 1 || e2->parentId() != e->parentId() || e2->cost() != 0 || e2->capacity() != 1)
                        continue;

                    if (e2->from()->id() == e->to()->id() && e2->to()->id() == e->from()->id()) {
                        oppositeDirection = true;
                        break;
                    }
                }

                if (!oppositeDirection) {
                    Graph::Vertex* vFrom = g2.vertex(e->from()->id());
                    Graph::Vertex* vTo = g2.vertex(e->to()->id());
                    Graph::Edge* edgeOriginal = g2.edge(e->parentId());
                    Graph::Edge* new_e = g2.addEdge(vFrom, vTo, false, edgeOriginal->cost(), edgeOriginal->capacity());
                    new_e->setParentId(e->parentId());
                    g2.removeEdge(edgeOriginal);
                    directedElements.push_back(new_e->id());

                }
                //If the opposite direction has been used, I do nothing.


            }
            else {
                Graph::Vertex* vFrom = g2.vertex(e->from()->id());
                Graph::Vertex* vTo = g2.vertex(e->to()->id());
                for (int i = 0; i < countCopies; i++) {
                    //This works as duplicateLinks
                    Graph::Edge* ed = g2.addEdge(vFrom, vTo, false, e->cost(), e->capacity());
                    ed->setParentId(e->parentId());
                    directedElements.push_back(ed->id());
                    addedElements.push_back(ed->id());
                }
            }

        }

        //Insert in undirected set the remained edges
        for (Graph::EdgeIDMap::iterator it = g2.edges().begin(); it != g2.edges().end(); it++) {
            Graph::Edge* e = it->second;
            if (e->undirected()) {
                undirectedElements.push_back(e->id());
            }

        }


        *graph = g2;

        return std::tuple<std::vector<int>, std::vector<int>, std::vector<int>>(directedElements, undirectedElements, addedElements);

    }




    std::set<int> ruralSolver(Graph* graph, std::set<int> notRequiredEdges, graph_utils::GraphType type)
    {

        //Vertices incident only to required edges
        std::vector<int> requiredVertices;
        //Vertices incident only to already visited edges
        std::vector<int> travelVertices;
        //Vertices incident to at least 1 required and 1 visited edges
        std::vector<int> borderVertices;


        for (Graph::VertexIDMap::const_iterator itVertices = graph->vertices().begin(); itVertices != graph->vertices().end(); itVertices++) {
            bool incidentToRequired = false;
            bool incidentToVisited = false;
            Graph::Vertex* v = dynamic_cast<Graph::Vertex*>(itVertices->second);

            for (Graph::EdgeSet::iterator itEdges = v->edges().begin(); itEdges != v->edges().end(); itEdges++) {
                Graph::Edge* e = dynamic_cast<Graph::Edge*>(*itEdges);

                if (notRequiredEdges.find(e->id()) != notRequiredEdges.end())
                    incidentToVisited = true;
                else 
                    incidentToRequired = true;
                
            }

            if (incidentToVisited && incidentToRequired) {
                borderVertices.push_back(itVertices->first);
            }
            else if (incidentToRequired) {
                requiredVertices.push_back(itVertices->first);
            }
            else if (incidentToVisited) {
                travelVertices.push_back(itVertices->first);
            }

        }

        std::map<std::pair<int, int>, float> D;
        std::map<std::pair<int, int>, std::vector<int>> P;

        //Compute all shortest paths between border vertices
        shortest_paths::mapDijkstra(*graph, borderVertices, borderVertices, &D, &P);


        std::map<std::pair<int, int>, std::vector<int>> optimalTravelPaths = P;
        std::map<std::pair<int, int>, float> shortestPathsD = D;
        //Remove paths containing not only visited edges
        for (int fromId : borderVertices) {
            for (int toId : borderVertices) {

                if (fromId == toId) {
                    optimalTravelPaths.at(std::make_pair(fromId, toId)).clear();
                    shortestPathsD.at(std::make_pair(fromId, toId)) = std::numeric_limits<float>::infinity();
                    continue;
                }

                std::vector<int> path = P.at(std::make_pair(fromId, toId));
                for (unsigned i = 0; i < path.size(); i++) {

                    if (notRequiredEdges.find(path[i]) != notRequiredEdges.end()){
                        optimalTravelPaths.at(std::make_pair(fromId, toId)).clear();
                        shortestPathsD.at(std::make_pair(fromId, toId)) = std::numeric_limits<float>::infinity();
                        break;
                    }
                }

            }
        }


        std::set<int> otpEdges;
        Graph optimalGraph = *graph;

        bool undirected = type == graph_utils::UNDIRECTED;
        //Add artificial OTPs edges
        for (std::map<std::pair<int, int>, std::vector<int>>::const_iterator it = optimalTravelPaths.begin(); it != optimalTravelPaths.end(); it++) {

            if ((it->second).empty())
                continue;

            int fromId = it->first.first;
            int toId = it->first.second;

            Graph::Vertex* vFrom = optimalGraph.vertex(fromId);
            Graph::Vertex* vTo = optimalGraph.vertex(toId);

            Graph::Edge* optEdge = optimalGraph.addEdge(vFrom, vTo, undirected, 0);

            otpEdges.insert(optEdge->id());
            notRequiredEdges.insert(optEdge->id());

        }

        BranchNBoundStruct lowerBound = BranchNBoundStruct(optimalGraph, otpEdges);

        std::priority_queue<BranchNBoundStruct, std::vector<BranchNBoundStruct>, std::greater<BranchNBoundStruct>> pq;

        pq.push(lowerBound);

        while (!pq.empty()) {
            //Extract top (lower cost) element from the priority queue
            BranchNBoundStruct currentStruct = pq.top();
            pq.pop();
            //If the lower cost element has no unlabelled OTP edges -> I have found a solution
            if (currentStruct.remainedElements.empty()) {
                heuristicExtension(&currentStruct.graph, type, notRequiredEdges);
                *graph = currentStruct.graph;
                return otpEdges;
            }

            int branchEdgeId = Graph::UnassignedId;
            float realCost =  0;

            for (int edgeId : currentStruct.remainedElements) {

                Graph::Edge* e = currentStruct.graph.edge(edgeId);
                int fromId = e->from()->id();
                int toId =  e->to()->id();

                float c = shortestPathsD.at(std::make_pair(fromId, toId));

                if (c >= realCost){
                    realCost = c;
                    branchEdgeId = edgeId;
                }

            }

            Graph g;
            BranchNBoundStruct structA = currentStruct;
            Graph::Edge* branchEdgeA = structA.graph.edge(branchEdgeId);

            structA.graph.removeEdge(branchEdgeA);
            structA.remainedElements.erase(branchEdgeId);
            g = structA.graph;
            heuristicExtension(&g, type, notRequiredEdges);

            structA.cost = graph_utils::eulerianCost(structA.graph);
            pq.push(structA);

            BranchNBoundStruct structB = currentStruct;
            structB.graph.edge(branchEdgeId)->setCost(realCost);
            structB.remainedElements.erase(branchEdgeId);

            g = structB.graph;
            heuristicExtension(&g, type, notRequiredEdges);

            structB.cost = graph_utils::eulerianCost(structB.graph);
            pq.push(structB);

        }

    }


}