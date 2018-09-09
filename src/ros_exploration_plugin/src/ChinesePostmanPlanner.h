#ifndef CHINESEPOSTMANPLANNER_H
#define CHINESEPOSTMANPLANNER_H

#include <nav2d_navigator/ExplorationPlanner.h>
#include "graph/graph.h"

class ChinesePostmanPlanner : public ExplorationPlanner
{
	public:
		ChinesePostmanPlanner();
		~ChinesePostmanPlanner();

		int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal);

	private:

		void drawVertex(int id);
		void drawEdge(int Id);

		void rvizToGrid(GridMap* map, geometry_msg::Point &pt, unsigned int &x, unsigned int &y);
		void gridToRviz(GridMap* map, unsigned int &x, unsigned int &y, geometry_msg::Point &pt);


		int unique_markers_id;


		Graph _graph;

        ros::NodeHandle _nh;
        
		ros::Publisher _pubMarkers;
    	ros::Subscriber _subMapFrame;
        ros::Subscriber _subRobotFrame;
        ros::Subscriber _subMarkers;

};

#endif // CHINESEPOSTMANPLANNER_H
