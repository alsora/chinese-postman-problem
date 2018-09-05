#ifndef CHINESEPOSTMANPLANNER_H
#define CHINESEPOSTMANPLANNER_H

#include <nav2d_navigator/ExplorationPlanner.h>

class ChinesePostmanPlanner : public ExplorationPlanner
{
	public:
		ChinesePostmanPlanner();
		~ChinesePostmanPlanner();

		int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal);

	private:

		std::vector<std::pair<double, double> > goals;
		int c;


		int goals_count;
        ros::NodeHandle _nh;
        
		ros::Publisher _pubMarkers;
    	ros::Subscriber _subMapFrame;
        ros::Subscriber _subRobotFrame;
        ros::Subscriber _subMarkers;

};

#endif // CHINESEPOSTMANPLANNER_H
