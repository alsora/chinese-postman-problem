#include "opencv_utilities.h"

using namespace Eigen;


cv::Point worldToImage(Vector2f wp, cv::Mat img, Vector2f min, float resolution)
{

    cv::Point pt;

    pt.x = (int)lrint((wp.x() - min.x()) / resolution);
    pt.y = (int)lrint((wp.y() - min.y()) / resolution);

    pt.x = std::min(pt.x, img.cols - 1);
    pt.y = std::min((img.rows - pt.y), img.rows - 1);

    return pt;

}



Vector2f imageToWorld(cv::Point ip, cv::Mat img, Vector2f min, float resolution)
{

    Vector2f pt;

    pt.x() = ip.x * resolution + min.x();
    pt.y() = (img.rows - ip.y) * resolution + min.y();


    return pt;

}


void addGrid(cv::Mat* img, int step, cv::Scalar color)
{

	int width = img->size().width;
	int height = img->size().height;

	for (int i = 0; i<height; i += step)
		cv::line(*img, cv::Point(0, i), cv::Point(width, i), color);

	for (int i = 0; i<width; i += step)
		cv::line(*img, cv::Point(i, 0), cv::Point(i, height), color);

}

void rotate90n(cv::Mat const &src, cv::Mat &dst, int angle)
{

	CV_Assert(angle % 90 == 0 && angle <= 360 && angle >= -360);
	if (angle == 270 || angle == -90) {
		// Rotate clockwise 270 degrees
		cv::transpose(src, dst);
		cv::flip(dst, dst, 0);
	}
	else if (angle == 180 || angle == -180) {
		// Rotate clockwise 180 degrees
		cv::flip(src, dst, -1);
	}
	else if (angle == 90 || angle == -270) {
		// Rotate clockwise 90 degrees
		cv::transpose(src, dst);
		cv::flip(dst, dst, 1);
	}
	else if (angle == 360 || angle == 0 || angle == -360) {
		if (src.data != dst.data) {
			src.copyTo(dst);
		}
	}


}


std::vector<cv::Point> sampleSegment(cv::Point a, cv::Point b, int step)
{

    std::vector<cv::Point> sampledPoints;

    cv::Point difference = b - a;
    float length = cv::norm(difference);

    Vector2f slope = Vector2f(difference.x / length, difference.y / length);

    for (int i = 0; i <= length; i += step){

        cv::Point delta = cv::Point((int)lrint(i * slope.x()), (int)lrint(i * slope.y()));
        
        cv::Point s = a + delta;

        sampledPoints.push_back(s); 
    }

    return sampledPoints;

}


std::pair<Vector2f, Vector2f> computeGraphExtremes(Graph graph)
{

    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::min();
    float maxY = std::numeric_limits<float>::min();

    for (Graph::VertexIDMap::iterator it = graph.vertices().begin(); it != graph.vertices().end(); it ++){

        Vector2f position = it->second->position();

        if (position.x() < minX){
            minX = position.x();
        }
        if (position.x() > maxX ){
            maxX = position.x();
        }
        if (position.y() < minY){
            minY = position.y();
        }
        if (position.y() > maxY) {
            maxY = position.y();
        }
    }


    Vector2f min = Vector2f(minX, minY);
    Vector2f max = Vector2f(maxX, maxY);


    return std::make_pair(min, max);

}

cv::Mat createImg(Vector2f min, Vector2f max, float offset, float resolution)
{


    int cols = (max.x() - min.x()) / resolution;
    int rows = (max.y() - min.y()) / resolution; 


    cv::Mat img = cv::Mat::zeros(rows, cols, CV_8UC3);

    return img;

}

void drawVertex(cv::Mat img, Graph::Vertex* v, Vector2f min, float resolution,cv::Scalar color){

    Vector2f position = v->position();

    cv::Point pt = worldToImage(position, img, min, resolution);
    
    cv::circle(img, pt, 4, color, -1);

}


void drawEdge(cv::Mat* img, Graph::Edge* e, Vector2f min, float resolution, cv::Scalar color){

    Vector2f fromPosition = e->from()->position();
    Vector2f toPosition = e->to()->position();

    bool undirected = e->undirected();

    cv::Point fromPt = worldToImage(fromPosition, *img, min, resolution);
    cv::Point toPt = worldToImage(toPosition, *img, min, resolution);
    
    double length = cv::norm(fromPt - toPt);

    double arrowScale = 100 * 0.15  / length;

    cv::arrowedLine(*img, fromPt, toPt, color, 1, 8, 0, arrowScale);

    if (undirected){
        cv::arrowedLine(*img, toPt, fromPt, color, 1, 8, 0, arrowScale);
    }

}


cv::Mat drawGraph(Graph graph, float offset, float resolution)
{


    std::pair<Vector2f, Vector2f> extremes = computeGraphExtremes(graph);

    Vector2f min = extremes.first;
    Vector2f max = extremes.second;

    max.x() += offset;
    max.y() += offset;
    min.x() -= offset;
    min.y() -= offset;

    cv::Mat img = createImg(min, max, offset, resolution);


    for (Graph::VertexIDMap::iterator it = graph.vertices().begin(); it != graph.vertices().end(); it ++){
        drawVertex(img, it->second, min, resolution, cv::Scalar(255,0,0));
    }

    for (Graph::EdgeIDMap::iterator it = graph.edges().begin(); it != graph.edges().end(); it ++){
        drawEdge(&img, it->second, min, resolution, cv::Scalar(0, 255, 0));
    }


    return img;

}




void animatePath(Graph graph, std::vector<int> path, int startId)
{   

    cv::namedWindow("my window", CV_WINDOW_AUTOSIZE);
    float offset = 5;
    float resolution = 0.05;

    std::pair<Vector2f, Vector2f> extremes = computeGraphExtremes(graph);

    Vector2f min = extremes.first;
    Vector2f max = extremes.second;

    max.x() += offset;
    max.y() += offset;
    min.x() -= offset;
    min.y() -= offset;

    cv::Mat img = createImg(min, max, offset, resolution);

    for (Graph::VertexIDMap::iterator it = graph.vertices().begin(); it != graph.vertices().end(); it ++){
        drawVertex(img, it->second, min, resolution, cv::Scalar(255,0,0));
    }

    for (Graph::EdgeIDMap::iterator it = graph.edges().begin(); it != graph.edges().end(); it ++){
        drawEdge(&img, it->second, min, resolution, cv::Scalar(0, 255, 0));
    }

    int previousId = startId;

    for (int eId : path){

        Graph::Edge* e = graph.edge(eId);

        Vector2f fromPosition = e->from()->position();
        Vector2f toPosition = e->to()->position();
        if (e->undirected() && (e->from()->id() != previousId) ){
            fromPosition = e->to()->position();
            toPosition = e->from()->position();
            previousId = e->from()->id();
        }
        else {
            previousId = e->to()->id();
        }

        cv::Point fromPt = worldToImage(fromPosition, img, min, resolution);
        cv::Point toPt = worldToImage(toPosition, img, min, resolution);

        std::vector<cv::Point> sampledPts = sampleSegment(fromPt, toPt, 5);

        int w = 15;
        int h = 15;

        for (cv::Point pt : sampledPts){

            cv::Mat tmpImg = img.clone();

            cv::rectangle( tmpImg, 
                        cv::Point(pt.x-w/2,pt.y-h/2),
                        cv::Point(pt.x+w/2,pt.y+h/2),
                        cv::Scalar(0,0, 255), 3, 8
                        );

            cv::imshow("my window", tmpImg);
            cv::waitKey(0);

        }


        drawEdge(&img, e, min, resolution, cv::Scalar(0, 165, 255));

    }


}


