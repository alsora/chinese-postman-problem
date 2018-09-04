# Chinese-Postman-Problem
Framework for creating graph structures and computing Eulerian circuits on them, using Chinese Postman Problem algorithms.

## What's the Chinese Postman Problem (CPP)


An Eulerian circuit is a path which allows the agent to visit all the edges of a graph at least once.
The CPP uses combinatorial optimization techniques in order to find the minimum cost Eulerian circuit.

This solver can handle all the possible combinations of graph's types, CPP's variants and output path requirements. 
Graph's edges can be either all directed, all undirected or a mixture of the two.
The standard CPP problem require all the edges to be visited, while in the rural one it is possible to specify a subset of edges which is not compulsory to be visited.
The output can be a circuit, i.e. a closed path where the start and end nodes are coincident, or a generic open path.

![eulerian circuit animation](https://github.com/alsora/chinese-postman-problem/blob/master/routing.gif)

Animation of the Eulerian circuit computed on a simple directed graph.
Traversed edges are coloured in orange.

## Requirements

 - Eigen
 - OpenCV (only for visualizing the results)



 ## Usage

 Build the project

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make 



Run the test scripts in order to try the framework

    $ cd ../bin

For example: to solve a CPP problem and compute an eulerian path
     
    $ ./routing

 
## References

Here you can find a reference to the theory behind the CPP and its solvers implemented in this repository.

|   | Directed | Undirected | Mixed |
| ------------- | ------------- | ------------- |------------- |
| **Standard**  | [Simmetry Heuristic](https://www3.cs.stonybrook.edu/~algorith/implement/cpp/distrib/SPAEcpp.pdf) | [Even Degree Heuristic](http://web.mit.edu/urban_or_book/www/book/chapter6/6.4.2.html) | [Mixed Approach 3/2](https://pdfs.semanticscholar.org/bbbe/3546347a4a15cb6b51a6fbaf6cec4cc1ad17.pdf)|
| **Rural**  | [Branch & Bound](http://www.roboticsproceedings.org/rss06/p21.pdf)  | [Branch & Bound](http://www.roboticsproceedings.org/rss06/p21.pdf) | [Minimum Spanning Tree](https://www.ri.cmu.edu/pub_files/2011/8/thesis_xu.pdf)|

An open path between any two vertices can be computed adding an artificial edge between them.

Pull Requests with modern algorithms (there's a lot of stuff with genetic algorithms for example!) are very welcomed.


The graph representation has been created starting from the [g2o](https://github.com/RainerKuemmerle/g2o) hypergraph. Credits to R. Kuemmerle and G. Grisetti.

