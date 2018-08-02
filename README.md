# Chinese-Postman-Problem
Framework for creating graph structures and computing Eulerian circuits on them, using the Chinese Postman Problem.

Solver for all types of graphs or CPP variants: directed graph, undirected graph, mixed graph, closed problem, rural problem, open problem.

![eulerian circuit animation](https://github.com/alsora/chinese-postman-problem/blob/master/routing.gif)
Animation of the Eulerian circuit computed on a simple directed graph.
Traversed edges are coloured in orange.

### Requirements

 - Eigen
 - OpenCV (only for visualizing the results)



 ### Usage

 Build the project

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make 



Run the test scripts in order to try the framework

    $ cd ../bin

Create a graph structure

    $ ./create_graph
   
Solve a CPP problem and compute an eulerian circuit
     
    $ ./routing

 
### References

Here you can find a reference to the theory behind the CPP and its solvers implemented in this repository.

|   | Directed | Undirected | Mixed |
| ------------- | ------------- | ------------- |------------- |
| **Standard**  | Simmetry Heuristic | Even Degree Heuristic | 3/2 Bbounded Solution|
| **Rural**  | Branch & Bound  | Branch & Bound | Minimum Spanning Tree|
| **Open**  | Directed Artificial Edge | Undirected Artificial Edge | Directed Artificial Edge |


Pull Requests with modern algorithms (there's a lot of stuff with genetic algorithms for example!) are very welcomed.


The graph representation has been created starting from the [g2o](https://github.com/RainerKuemmerle/g2o) hypergraph. Credits to R. Kuemmerle and G. Grisetti.

