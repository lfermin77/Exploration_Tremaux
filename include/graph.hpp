#include <limits>
#include <queue>
#include <list>
#include <map>
#include <complex>
#include <iostream>
//#include "visualization_msgs/Marker.h"




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



////////////////////////////////////////////////////

typedef std::list<Node*>::iterator Node_iter;
typedef std::list<Edge*>::iterator Edge_iter;

class UtilityGraph{
	public:
	std::list <Node*> Nodes;
	std::list <Edge*> Edges;

	~UtilityGraph();
		
	void print_nodes();
	Node_iter find_point_in_node(std::complex<double> query_position);
	int update_distances(std::complex<double> current_position);

	std::list <Edge*> find_edges_between_regions();	
	void evaluate_regions_connectivity();
	void evaluate_list_connectivity(std::list <Node*> list_in, int name);
	void Closest_subregions(std::list <Node*> node_pair, int region);
	std::pair <Node*,Node*> closest_node_subregions(std::vector<Node*> path_1, std::vector<Node*> path_2);
		
};



