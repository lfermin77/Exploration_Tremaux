#include <limits>
#include <queue>
#include <list>
#include <map>
#include <complex>
#include <iostream>
//#include "visualization_msgs/Marker.h"




////Graph as adjacency list nested class



class Node;
class Region_Edge{
	private:
	int info;
	Node*  from;
	Node*  to;
};
///////////////////////////
class Region_Node{	
	private:
	int info;	
	Node* predecesor;	
	std::vector<Edge> connected;
};


////////////////////////////////////////////////////


class Region_Graph{
	public:
	std::list <Region_Node*> Nodes;
	std::list <Region_Edge*> Edges;		
};



