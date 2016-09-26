#include <limits>
#include <queue>
#include <list>
#include <map>
#include <complex>
#include <iostream>
//#include "visualization_msgs/Marker.h"




////Graph as adjacency list nested class

class Region_Node; // forward declaration

class Region_Edge{
	private:
	int info;
	Region_Node*  from;
	Region_Node*  to;
};
///////////////////////////
class Region_Node{	
	private:
	int info;	
	Region_Node* predecesor;	
	std::vector<Region_Edge*> connected;
};
////////////////////////////////////////////////////

class Region_Graph{
	public:
	std::list <Region_Node*> Nodes;
	std::list <Region_Edge*> Edges;		
};



