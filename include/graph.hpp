#include <limits>
#include <queue>
#include <list>
#include <map>
#include <complex>
#include <iostream>
#include <unordered_map>


#include <opencv2/imgproc/imgproc.hpp>
#include "visualization_msgs/Marker.h"
#include "nav_msgs/GetMap.h"



////Graph as adjacency list nested class
class node_info{
	public:
	int label;
	int region_label;
	int sub_region;
	float distance_from_origin;
	std::complex<double> position;
	
	node_info();
};

//////////////////////////
class edge_info{
	public:
	float distance;
	int label;
	
	edge_info();
};
/////////////////////
class Node;

class Edge{
	public:
	edge_info info;
	Node*  from;
	Node*  to;
};

///////////////////
struct Connections{
	Edge* linker;
	Node* to;
};

///////////////////////////
class Node{
	public:

	node_info info;	
	Node* predecesor;	
	std::vector<Connections> connected;
	
	Node();	
	void print_node_label_and_pos();
};


/////////////////////////////////////////////////
/////REGION GRAPH

class Region_Node; //forward declaration
class Region_Edge{
	public:
	std::vector <Edge*> Edges_in_border;
	std::vector<cv::Point> frontier;
	float shortest_distance;
	std::set<int> Nodes_ids;
	
	Region_Node*  First_Region;
	Region_Node*  Second_Region;
};
///////////////////////////
class Region_Node{	
	public:
	int id;
	Node* Node_Center;
	std::list <Node*> nodes_inside;
	std::list < std::list <Node*> > sub_graphs;
	float distance_from_origin;
	Region_Node* predecesor;	
	std::vector<cv::Point> contour;
	std::vector<Region_Edge*> connected;
	
	std::set< Edge*> marker_in;
	std::set< Edge*> marker_out;
};
////////////////////////////////////////////////////









////////////////////////////////////////////////////

typedef std::list<Node*>::iterator Node_iter;
typedef std::list<Edge*>::iterator Edge_iter;

///////////////////////////////////////
///////////////////////////////////////

typedef std::map < std::set<int> , std::vector<cv::Point>   > edge_points_mapper;
typedef std::map < int , std::vector<cv::Point>   > region_points_mapper;

typedef	std::unordered_map <int,Node*> NodeMapper;
typedef	std::unordered_map <int,Edge*> EdgeMapper;
typedef	std::unordered_map <int,Region_Node*> RegionNodeMapper;
typedef	std::map < std::set<int> ,Region_Edge*> RegionEdgeMapper;
	

//////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

class RegionGraph{
	public:
	
	~RegionGraph();
	int build_Region_Graph(std::vector<geometry_msgs::Point> edge_markers, nav_msgs::MapMetaData info, cv::Mat  Tag_image, cv::Mat  original_image);
	friend std::ostream& operator<<(std::ostream& os, RegionGraph& Graph);
	void Tremaux_data();
	
	protected:
	void extract_subgraph();
	void evaluate_list_connectivity(std::list <Node*> list_in, int name);
	void build_region_graph(cv::Mat  Tag_image, cv::Mat  original_image);
	void find_edges_between_regions();
	void find_center_of_regions();

	int current_node_id;
	
	std::unordered_map <int,Node*> Nodes_Map;
	std::unordered_map <int,Edge*> Edges_Map;
	
	std::unordered_map <int,Region_Node*> Region_Nodes_Map;
	std::map < std::set<int> ,Region_Edge*> Region_Edges_Map;
	
		
};

