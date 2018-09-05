#include "ChinesePostmanPlanner.h"

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <string> 
#include <sstream>
#include "graph/graph.h"

typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;

bool new_msg = false;
 geometry_msgs::Point last_point;

using namespace ros;

void chatterCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{

	double x = msg->point.x;
	double y = msg->point.x;
	double z = msg->point.z;

	last_point = msg->point;

	std::cout<<"I heard "<< x<<" " << y<<" " << z<<std::endl;

//  ROS_INFO("I heard: [%d %d %d]", x, y, z);
	new_msg = true;
}



ChinesePostmanPlanner::ChinesePostmanPlanner()
{	


	Graph g;
	
	
	goals_count = 0;
	
	_subMarkers = _nh.subscribe("clicked_point", 1000, chatterCallback);

	_pubMarkers = _nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);


	ROS_INFO("THIS IS CPP PROBLEM");	

	ROS_INFO("WAITING FOR POINTS TO BE PUBLISHED");	

	int unique_markers_id = 0;
	std::vector<geometry_msgs::Point> graph_vertices;


	int i = 0;

	while (i < 5)
	{
		spinOnce();

		if (!new_msg){ continue;}

		i++;
		unique_markers_id++;
		
		graph_vertices.push_back(last_point);

		visualization_msgs::Marker points;
		points.type = visualization_msgs::Marker::POINTS;
		points.id = unique_markers_id;
		// POINTS markers use x and y scale for width/height respectively
		points.scale.x = 0.6;
		points.scale.y = 0.6;
		// Points are green
		points.color.g = 1.0f;
		points.color.a = 1.0;
		points.header.frame_id = "/map";
		points.points.push_back(last_point);


		visualization_msgs::Marker text;
		text.header.frame_id = "/map";
		text.id = unique_markers_id;
		text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

		text.pose.position = last_point;
		
		std::stringstream ss;
		ss << i;
		text.text = ss.str();


		unique_markers_id++;
		text.id = unique_markers_id;

		text.scale.x = 4;
		text.scale.y = 4;
		text.scale.z = 1;

		text.color.r = 0.0f;
		text.color.g = 0.0f;
		text.color.b = 0.0f;
		text.color.a = 1.0;

		_pubMarkers.publish(text);
		_pubMarkers.publish(points);

		new_msg = false;
	

	}

	ROS_INFO("Graph's vertices created! Now define the edges");	
	ROS_INFO("Write the id of the two vertices you want to connect. Write \"-1 -1\" to stop.");	

	while (true){

		std::string text;
		std::vector<int> vertices_pair;

		getline (std::cin, text);
		
		std::cout<<"You typed: "<< text <<std::endl;


		int n;
		std::stringstream stream(text);
		while(stream >> n){
			vertices_pair.push_back(n);
		}

		if (vertices_pair.size() != 2){

			//std::ostringstream error_stream;
			//error_stream << "Error. Expected exactly 2 vertices id. Parsed "<<vertices_pair.size()<<" in: \""<< text << "\"";
			//std::string error_string = error_stream.str();
			ROS_ERROR("You must write 2 numbers or \"-1 -1\" to stop.");	
		}

		int from = vertices_pair[0];
		int to = vertices_pair[1];	

		if (from <= 0 && to <=0){
			break;	
		}

		if (from > graph_vertices.size() || to > graph_vertices.size()){
			ROS_ERROR("One of the entered id is not present in the graph.");
			continue;	
		}

		if (from <= 0 || to <= 0){
			ROS_ERROR("One of the entered id is negative.");
			continue;
		}

		bool undirected = true;

		unique_markers_id++;

		visualization_msgs::Marker edge;
		edge.type = visualization_msgs::Marker::ARROW;
		edge.id = unique_markers_id;
		edge.header.frame_id = "/map";
 		edge.header.stamp = ros::Time::now();
		edge.scale.x = 0.05;
		edge.scale.y = 0.1;
		edge.color.r = 1.0;
		edge.color.a = 1.0;
		edge.points.push_back(graph_vertices[from -1]);
		edge.points.push_back(graph_vertices[to -1]);

		_pubMarkers.publish(edge);


		ROS_INFO("Define another edge or write \"-1 -1\" to stop.");	

	}

	ROS_INFO("Defined all edges.");	

	

}

ChinesePostmanPlanner::~ChinesePostmanPlanner()
{
	
}

int ChinesePostmanPlanner::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal)
{

	ROS_INFO("THIS IS CPP PROBLEM");

	/*
	if (goals_count == 0){


		std::pair<double, double> new_goal = std::make_pair(-7.5, -7.5);

		double resolution = map->getResolution();

		std::pair<unsigned int, unsigned int> cells = std::make_pair(new_goal.first / resolution, new_goal.second / resolution);

		unsigned int index;
		
		map->getIndex(cells.first, cells.second, index);

		goal = index;

		goals_count++;
		return EXPL_TARGET_SET;
	}
	else{

		return EXPL_FINISHED;
	}
	*/

	// Create some workspace for the wavefront algorithm
	unsigned int mapSize = map->getSize();
	double* plan = new double[mapSize];
	for(unsigned int i = 0; i < mapSize; i++)
	{
		plan[i] = -1;
	}
	
	// Initialize the queue with the robot position
	Queue queue;
	Entry startPoint(0.0, start);
	queue.insert(startPoint);
	plan[start] = 0;
	
	Queue::iterator next;
	double distance;
	double linear = map->getResolution();
	bool foundFrontier = false;
	int cellCount = 0;
	
	// Do full search with weightless Dijkstra-Algorithm
	while(!queue.empty())
	{
		cellCount++;
		// Get the nearest cell from the queue
		next = queue.begin();
		distance = next->first;
		unsigned int index = next->second;
		queue.erase(next);
		
		// Add all adjacent cells
		if(map->isFrontier(index))
		{
			// We reached the border of the map, which is unexplored terrain as well:
			foundFrontier = true;
			unsigned int x;
			unsigned int y;
			
			map->getCoordinates(x, y, index);
			
			goal = index;
			break;
		}else
		{
			unsigned int ind[4];

			ind[0] = index - 1;               // left
			ind[1] = index + 1;               // right
			ind[2] = index - map->getWidth(); // up
			ind[3] = index + map->getWidth(); // down
			
			for(unsigned int it = 0; it < 4; it++)
			{
				unsigned int i = ind[it];
				if(map->isFree(i) && plan[i] == -1)
				{
					queue.insert(Entry(distance+linear, i));
					plan[i] = distance+linear;
				}
			}
		}
	}

	//cell 94 131 at I: 24591

	unsigned int x = 60;
	unsigned int y = 131;

	unsigned int index;
	
	map->getIndex(x, y, index);

	std::cout<<"GOAL AT: "<< x <<" "<< y <<" with index "<< index<<std::endl;

	goal = index;
	

	ROS_DEBUG("Checked %d cells.", cellCount);	
	delete[] plan;

	if (goals_count == 0){

		return EXPL_TARGET_SET;
		goals_count ++;
	
	}
	else {
		return EXPL_FINISHED;
	}
	
	if(foundFrontier)
	{
		return EXPL_FINISHED;
	}else
	{
		if(cellCount > 50)
			return EXPL_FINISHED;
		else
			return EXPL_FAILED;
	}


}
