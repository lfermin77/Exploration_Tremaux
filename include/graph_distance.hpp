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
	float distance_in_this_iter = std::numeric_limits<float>::infinity();
	std::vector<Arc*> connections;
};

///////////////////////////
class Graph_Distance{
	public:
	
	~Graph_Distance();
	friend std::ostream& operator<<(std::ostream& os, Graph_Distance& Graph);	

	int insert_new_node(int label, std::vector< std::pair<int, float> > Connections_label_distance);
	int insert_new_edge(int label_1, int label_2,  float distance );

	
	
	protected:
	//functions
	void insert_first_edge(int label_first, int label_second, float distance);
	void clean_class();
	int update_distances(	int label_new_node );
	int update_distance_matrix(int label_new_node);
	int extract_central_vertex_label();
		
	// variables
	std::unordered_map<int, Vertex*>   Vertices_map;
	std::map<std::set<int>, Arc*>      Arcs_map;
	
	std::vector <std::vector <float> > distance_matrix;

	std::unordered_map <int, std::unordered_map <int,float> >  map_distance_matrix;
};





