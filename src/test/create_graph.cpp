#include <iostream>
#include <eigen3/Eigen/Dense>
#include "graph/graph.h"
#include "graph/graph_utils.h"
#include "graph/graph_factory.h"
#ifdef OPENCV_LIBRARY_FOUND
#include "opencv2/highgui/highgui.hpp"
#include "visualization/opencv_utilities.h"
#endif


using namespace Eigen;

int main()
{

    Graph graph;

    graph.addVertex(1, Vector2f(0, 10));
    graph.addVertex(2, Vector2f(10, 10));
    graph.addVertex(3, Vector2f(5, 2.5));
    graph.addVertex(4, Vector2f(0, 0));
    graph.addVertex(5, Vector2f(10, 0));

    graph.addEdge(3, 1);
    graph.addEdge(1, 2);
    graph.addEdge(2, 3);
    graph.addEdge(3, 4);
    graph.addEdge(5, 2);
    graph.addEdge(5, 3);
    graph.addEdge(4, 5);

    graph_utils::printVerticesInfo(graph);
    graph_utils::printEdgesInfo(graph);


    #ifdef OPENCV_LIBRARY_FOUND
    cv::Mat img = drawGraph(graph);

    cv::namedWindow("my window", CV_WINDOW_AUTOSIZE);
    cv::imshow("my window", img);
    cv::waitKey(0);
    #endif



}