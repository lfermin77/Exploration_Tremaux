#include <limits>
#include <queue>
#include "visualization_msgs/Marker.h"




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

///////////////////////////
class Node{
	public:

	class nested_Edge{
		public:
		edge_info info;
		Node*  from;
		Node*  to;
	};
	
	struct nested_connections{
		nested_Edge* linker;
		Node* to;
	};

	node_info info;	
	Node* predecesor;	
	std::vector<nested_connections> connected;
	
	Node();	
	void print_node_label_and_pos();
};
typedef Node::nested_Edge         Edge;
typedef Node::nested_connections  Connections;


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
	void build_graph_from_edges(std::vector<geometry_msgs::Point> edge_markers);
	int update_distances(geometry_msgs::Point current_pos);

	std::list <Edge*> find_edges_between_regions();	
	void evaluate_regions_connectivity(int number_of_regions);
	void evaluate_list_connectivity(std::list <Node*> list_in, int name);

		
};



