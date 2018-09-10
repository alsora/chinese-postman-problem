#include "ChinesePostmanPlanner.h"

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <string> 
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <math.h> 
#include "graph/graph_utils.h"



typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;

bool new_msg = false;
 geometry_msgs::Point last_point;

using namespace ros;

void chatterCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{

	last_point = msg->point;

	last_point.y -= 2;

	ROS_INFO("Clicked point: [%f %f]", last_point.x, last_point.y);
	new_msg = true;
}



ChinesePostmanPlanner::ChinesePostmanPlanner()
{	

	unique_markers_id = 0;

	_routing = RoutingProblem();
	
	_subMarkers = _nh.subscribe("clicked_point", 1000, chatterCallback);

	_pubMarkers = _nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	parseInputGraph();


	ROS_INFO("Generating CPP instance...");	


	_routing.init(_graph,1,1);

	_circuit = _routing.solve();

	_circuit = graph_utils::pathEdgesToVertices(_circuit, _graph, 1);
	
	ROS_INFO("Solved CPP");	

	for (int i : _circuit){
		std::cout<< i <<" ";
	}
	std::cout<<std::endl;


}

ChinesePostmanPlanner::~ChinesePostmanPlanner()
{
	
}

int ChinesePostmanPlanner::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal)
{

	float goal_tolerance = 25;

	if (_circuit.empty()){
		goal = start;
		return EXPL_FINISHED;
	}

	unsigned int start_x;
	unsigned int start_y;
	map->getCoordinates(start_x, start_y, start);

	while (true){

		int new_goal_id = _circuit.front();
		Eigen::Vector2f new_goal = _graph.vertex(new_goal_id)->position();

		auto goal_x = ((new_goal.x() - map->getOriginX())/map->getResolution());
		auto goal_y = (((new_goal.y() + 2) - map->getOriginY())/map->getResolution());

		float distance = (goal_x - start_x)* (goal_x - start_x) + (goal_y - start_y)* (goal_y - start_y);

		if (distance < goal_tolerance){
			_circuit.erase(_circuit.begin());
			continue;
		}


		map->getIndex(goal_x, goal_y, goal);

		return EXPL_TARGET_SET;
	}

	
}





void ChinesePostmanPlanner::drawVertex(int id)
{

		unique_markers_id++;

		Eigen::Vector2f vPos = _graph.vertex(id)->position();
		
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
		geometry_msgs::Point contour_point;
		contour_point.x = vPos.x();
		contour_point.y = vPos.y() + 2;

		points.points.push_back(contour_point);

		visualization_msgs::Marker text;
		text.header.frame_id = "/map";
		text.id = unique_markers_id;
		text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

		text.pose.position = contour_point;
		text.pose.position.y -= 0.1;

		std::stringstream ss;
		ss << _graph.vertices().size();
		text.text = ss.str();


		unique_markers_id++;
		text.id = unique_markers_id;

		text.scale.x = 1;
		text.scale.y = 1;
		text.scale.z = 0.75;

		text.color.r = 0.0f;
		text.color.g = 0.0f;
		text.color.b = 0.0f;
		text.color.a = 1.0;

		_pubMarkers.publish(text);
		_pubMarkers.publish(points);



}

void ChinesePostmanPlanner::drawEdge(int id)
{

	unique_markers_id++;

	Graph::Edge* e = _graph.edge(id);

	Graph::Vertex* fromV = e->from();
	Graph::Vertex* toV = e->to();
	bool undirected = e->undirected();

	visualization_msgs::Marker edge;
	edge.type = visualization_msgs::Marker::ARROW;
	edge.id = unique_markers_id;
	edge.header.frame_id = "/map";
	edge.header.stamp = ros::Time::now();
	edge.scale.x = 0.05;
	edge.scale.y = 0.1;
	edge.color.r = 1.0;
	edge.color.a = 1.0;


	Eigen::Vector2f fromPos = fromV->position();
	Eigen::Vector2f toPos = toV->position();

	geometry_msgs::Point fromPt;
	geometry_msgs::Point toPt;

	fromPt.x = fromPos.x();
	fromPt.y = fromPos.y() + 2;

	toPt.x = toPos.x();
	toPt.y = toPos.y() + 2;

	edge.points.push_back(fromPt);
	edge.points.push_back(toPt);

	_pubMarkers.publish(edge);

	if (undirected){
		unique_markers_id++;
		visualization_msgs::Marker opposite_edge = edge;
		opposite_edge.id = unique_markers_id;
		opposite_edge.points.clear();
		opposite_edge.points.push_back(toPt);
		opposite_edge.points.push_back(fromPt);
		_pubMarkers.publish(opposite_edge);

	}


}

void ChinesePostmanPlanner::parseInputGraph()
{

	parseInputVertices();

	ROS_INFO("Graph's vertices created! Now define the edges");	

	std::vector<std::vector<int>> connectedComponents = std::vector<std::vector<int>>();

	while (connectedComponents.size() != 1){

		parseInputEdges();

		connectedComponents = graph_utils::tarjanConnectedComponents(_graph);
		if (connectedComponents.size() != 1){
			ROS_ERROR("The defined graph is not fully connected! It has %d connected components", connectedComponents.size());
			for (int i = 0; i < connectedComponents.size(); i ++){
				std::stringstream component_text("");
				component_text<<"Components";
				component_text<< i + 1;
				component_text<<": ";
				for (int j = 0; j < connectedComponents[i].size(); j ++){
					component_text << connectedComponents[i][j];
					component_text << " ";
				}
				
				ROS_INFO("%s\n", component_text.str());
			}

			ROS_INFO("Select one option. 0 for adding new eges to this graph. 1 for aborting");
			
			std::string text;
			getline (std::cin, text);
					
			std::stringstream ss(text);
			int i;
			if ((ss >> i).fail() || !(ss >> std::ws).eof())
			{
				ROS_ERROR("Your selection is not valid. Aborting");
			}
			if (i != 0 && i != 1){
				ROS_ERROR("Your selection is not valid. Aborting");
			}
			if (i == 0){
				continue;
			}
			
		}
	}



	ROS_INFO("Defined all edges.");	
	ROS_INFO("Graph succesfully created.");	


}





void ChinesePostmanPlanner::parseInputVertices()
{

	ROS_INFO("Waiting for graph vertices to be published..");	


	while (_graph.vertices().size() < 4)
	{
		spinOnce();

		if (!new_msg){ continue;}

		int vertex_id = _graph.vertices().size() + 1;
		_graph.addVertex(vertex_id, Eigen::Vector2f(last_point.x, last_point.y));

		drawVertex(vertex_id);

		new_msg = false;
	
	}


}



void ChinesePostmanPlanner::parseInputEdges()
{

	ROS_INFO("Write the id of the two vertices you want to connect. Write \"-1 -1\" to stop.");	

	while (true){

		std::string text;
		std::vector<int> vertices_pair;

		getline (std::cin, text);
		
		int n;
		std::stringstream stream(text);
		while(stream >> n){
			vertices_pair.push_back(n);
		}

		if (vertices_pair.size() != 2){
			ROS_ERROR("You must write 2 numbers or \"-1 -1\" to stop.");
			continue;	
		}

		int fromId = vertices_pair[0];
		int toId = vertices_pair[1];	
		bool undirected = true;


		if (fromId == -1 && toId == -1){
			break;	
		}

		Graph::Vertex* fromV = _graph.vertex(fromId);
		Graph::Vertex* toV = _graph.vertex(toId);


		if (fromV == nullptr || toV == nullptr){
			ROS_ERROR("One of the entered id is not present in the graph.");
			continue;
		}

		Graph::Edge* e = _graph.addEdge(fromId, toId, undirected);

		drawEdge(e->id());

		ROS_INFO("Define another edge or write \"-1 -1\" to stop.");	

	}

}


void ChinesePostmanPlanner::rvizToGrid(GridMap* map, geometry_msgs::Point &pt, unsigned int &x, unsigned int &y)
{

	x = ((pt.x - map->getOriginX())/map->getResolution());
	y = (((pt.y + 2) - map->getOriginY())/map->getResolution());



}


void ChinesePostmanPlanner::gridToRviz(GridMap* map, unsigned int &x, unsigned int &y, geometry_msgs::Point &pt)
{

	pt.x = map->getOriginX() + (((double)x+0.5) * map->getResolution());
	pt.y = map->getOriginY() + (((double)y+0.5) * map->getResolution()) - 2;


}
