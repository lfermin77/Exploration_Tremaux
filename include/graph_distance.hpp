#include <limits>
#include <queue>
#include <list>
#include <map>
#include <set>
#include <iostream>
#include <unordered_map>




////Graph as an incidence list 
///////////////////////////

struct Arc{
	std::set<int> labels;
	float distance=-1;
};

struct Vertex{
	int label;
	float distance_in_this_iter=-1;	
	Vertex* predecesor=NULL;	
	std::vector<Arc*> connections;
};

///////////////////////////
class Graph_Distance{
	public:
	
	~Graph_Distance();
	friend std::ostream& operator<<(std::ostream& os, Graph_Distance& Graph);	
	void insert_first_pair_of_nodes(int label_first, int label_second, float distance);
	void insert_new_node(int label, std::vector< std::pair<int, float> > Connections_label_distance);
	void insert_new_edge(int label_1, int label_2,  float distance );
	
	protected:
	//functions
	void clean_class();
	
	// variables
	std::unordered_map<int, Vertex*> Vertices_map;
	std::map<std::set<int>, Arc*> Arcs_map;
	
	std::vector <std::vector <float> > distance_matrix;
};
