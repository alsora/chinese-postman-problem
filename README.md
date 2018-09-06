# Chinese-Postman-Problem
Chinese Postman Problem solver library.

## Repository details

This repository contains a library which allows to create traversable graph structures.
The shortest route which visits all the edges of the graph is computed solving a Chinese Postman Problem.

This library is used to create a ROS exploration plugin for the Nav2d ROS package.
This plugin allows the user to draw a graph on Rviz.
Then it's possible to explore it while building a map of the environment using the Nav2d SLAM algorithms.

Compute Eulerian circuits and paths to traverse graph structures.

ROS implementation for autonomous robotic exploration and navigation.


![eulerian circuit animation](https://github.com/alsora/chinese-postman-problem/blob/master/routing.gif)

Animation of the Eulerian circuit computed on a simple directed graph.
Traversed edges are coloured in orange.



## Build

### Requirements

 - Eigen REQUIRED
 - OpenCV (only for visualization)
 - ROS kinetic (only for the ROS plugin)
 - Nav2d (only for the ROS plugin)

### Instructions

You can decide wether to build only the library (and its tests) or also the ROS exploration plugin.

##### Library only

    $ mkdir build
    $ cd build
    $ cmake -DROS=0 ..
    $ make 

##### ROS build

Run the command in a catkin workspace.

    $ catkin_make

## Usage

##### Library tests

The executables are in the bin folder.
To generate a random graph and solve a CPP on it:

    $ ./routing

##### ROS exploration

Run the commands in a catkin workspace.

    $ source devel/setup.bash
    $ roslaunch cpp_solver exploration.launch

Then it's possible to draw the graph in Rviz using the `publish point` tool and following the prompted instructions.

Once the graph is complete, to start the exploration run in another terminal

    $ rosservice call \StartMapping
    $ rosservice call \StartExploration




## What's the Chinese Postman Problem (CPP)


An Eulerian circuit is a path which allows the agent to visit all the edges of a graph at least once.
The CPP uses combinatorial optimization techniques in order to find the minimum cost Eulerian circuit.

This solver can handle all the possible combinations of graph's types, CPP's variants and output path requirements. 
Graph's edges can be either all directed, all undirected or a mixture of the two.
The standard CPP problem require all the edges to be visited, while in the rural one it is possible to specify a subset of edges which is not compulsory to be visited.
The output can be a circuit, i.e. a closed path where the start and end nodes are coincident, or a generic open path.

Here you can find a reference to the theory behind the CPP and its solvers implemented in this repository.

|   | Directed | Undirected | Mixed |
| ------------- | ------------- | ------------- |------------- |
| **Standard**  | [Simmetry Heuristic](https://www3.cs.stonybrook.edu/~algorith/implement/cpp/distrib/SPAEcpp.pdf) | [Even Degree Heuristic](http://web.mit.edu/urban_or_book/www/book/chapter6/6.4.2.html) | [Mixed Approach 3/2](https://pdfs.semanticscholar.org/bbbe/3546347a4a15cb6b51a6fbaf6cec4cc1ad17.pdf)|
| **Rural**  | [Branch & Bound](http://www.roboticsproceedings.org/rss06/p21.pdf)  | [Branch & Bound](http://www.roboticsproceedings.org/rss06/p21.pdf) | [Minimum Spanning Tree](https://www.ri.cmu.edu/pub_files/2011/8/thesis_xu.pdf)|

An open path between any two vertices can be computed adding an artificial edge between them.

Pull Requests with modern algorithms (there's a lot of stuff with genetic algorithms for example!) are very welcomed.


The graph representation has been created starting from the [g2o](https://github.com/RainerKuemmerle/g2o) hypergraph. Credits to R. Kuemmerle and G. Grisetti.

