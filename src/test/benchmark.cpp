#include <iostream>
#include "graph/graph.h"
#include "routing/routing_problem.h"
#include "graph/graph_factory.h"
#include "graph/graph_utils.h"
#include "utilities.h"
#include <chrono>

using namespace std::chrono;
using namespace Eigen;

int main()
{


    int num_tests = 1000;

    std::vector<int> vals;

    srand(time(0));

    Graph graph;
    std::set<int> eset;

    while (vals.size() < num_tests){

        std::pair<Graph, std::set<int>> res = buildRandomGraph();
        graph = res.first;
        eset = res.second;
        
        int startId = 2;
        int goalId = 2;

        if (graph.vertex(startId) == nullptr || graph.vertex(goalId) == nullptr){
            continue;
        }        

        RoutingProblem routing = RoutingProblem();
        routing.init(graph, startId, goalId, eset);

        high_resolution_clock::time_point t1 = high_resolution_clock::now();

        std::vector<int> circuit = routing.solve();

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>( t2 - t1 ).count();

        if (duration > 1040597){
            continue;
        }

        vals.push_back(duration);
    }


    float mean = vector_mean(vals);
    float std = vector_std_dev(vals);


    std::cout<<"Mean: "<< mean <<" std_dev: "<<std<<std::endl;

}
