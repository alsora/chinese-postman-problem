#ifndef __OPENCV_UTILITIES_H__
#define __OPENCV_UTILITIES_H__

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include <eigen3/Eigen/Dense>
#include "graph/graph.h"

using namespace Eigen;




cv::Point worldToImage(Vector2f wp, cv::Mat img, Vector2f min, float resolution);

Vector2f imageToWorld(cv::Point ip, cv::Mat img, Vector2f min, float resolution);

void addGrid(cv::Mat* img, int step, cv::Scalar color = cv::Scalar(0, 0, 0));

void rotate90n(cv::Mat const &src, cv::Mat &dst, int angle);

std::vector<cv::Point> sampleSegment(cv::Point a, cv::Point b, int step);


cv::Mat drawGraph(Graph graph, float offset = 5, float resolution = 0.05);

void animatePath(Graph graph, std::vector<int> path, int startId);

void drawVertex(cv::Mat img, Graph::Vertex* v, Vector2f min, float resolution, cv::Scalar color = cv::Scalar(255,0,0));

void drawEdge(cv::Mat* img, Graph::Edge* e, Vector2f min, float resolution, cv::Scalar color = cv::Scalar(0,255,0));


std::pair<Vector2f, Vector2f> computeGraphExtremes(Graph graph);

cv::Mat createImg(Vector2f min, Vector2f max, float offset = 5, float resolution = 0.05);


#endif