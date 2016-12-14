#include "graph_distance.hpp"


///////////////////

void Graph_Distance::clean_class(){
	for(std::unordered_map<int, Vertex*>::iterator map_iter = Vertices_map.begin(); map_iter != Vertices_map.end(); map_iter ++){
		Vertex* ptr = map_iter->second;
	    delete  ptr;	
	}
	Vertices_map.clear();

	for(std::map<std::set<int>, Arc*>::iterator map_iter = Arcs_map.begin(); map_iter != Arcs_map.end(); map_iter ++){
		Arc* ptr = map_iter->second;
	    delete  ptr;	
	}
	Arcs_map.clear();
}
//
Graph_Distance::~Graph_Distance(){
	clean_class();
}


std::ostream& operator<<(std::ostream& os, Graph_Distance& Graph){
	os << "\n";	
	os << "Number of Nodes "<< Graph.Vertices_map.size() << "\n";
	os << "Number of Edges "<< Graph.Vertices_map.size() << "\n";

    return os;
}




///////////////////////////////////////



///////
void Graph_Distance::insert_first_pair_of_nodes(int label_first, int label_second, float distance){
	clean_class();
	Vertex* first_vertex = new Vertex;
	Vertex* second_vertex = new Vertex;
	
	Vertices_map[label_first]  = first_vertex;
	Vertices_map[label_second] = second_vertex;
		
	first_vertex->label  = label_first;
	second_vertex->label = label_second;

	Arc* first_arc = new Arc;

	first_arc->labels={label_first, label_second};
	first_arc->distance =distance;
	
	first_vertex ->connections.push_back(first_arc);
	second_vertex->connections.push_back(first_arc);
	
	Arcs_map[first_arc->labels] = first_arc;


	
}
//////


void Graph_Distance::insert_new_node(int label, std::vector< std::pair<int, float> > Connections_label_distance ){
	std::unordered_map<int, Vertex*>::iterator found_ptr =  Vertices_map.find(label);
	if(found_ptr != Vertices_map.end() ){
		std::cout << "Vertex label already in graph" << std::endl;
		return;
	}
	

	Vertex* vertex_new = new Vertex;
	Vertices_map[label]  = vertex_new;
	
	vertex_new->label=label;
	
	for(int i=0; i < Connections_label_distance.size(); i++){
		Arc* current_arc = new Arc;
		int TO_label = Connections_label_distance[i].first;
		
		current_arc->labels={label, TO_label};
		current_arc->distance = Connections_label_distance[i].second;
		
		vertex_new            ->connections.push_back(current_arc);
		Vertices_map[TO_label]->connections.push_back(current_arc);		
		
		Arcs_map[current_arc->labels] = current_arc;
	}
	////
}
//////


void Graph_Distance::insert_new_edge(int label_1, int label_2,  float distance ){
	std::unordered_map<int, Vertex*>::iterator found_ptr =  Vertices_map.find(label_1);
	if(found_ptr == Vertices_map.end() ){
		std::cout << "Vertex 1 label NOT in graph" << std::endl;
		return;
	}
	found_ptr =  Vertices_map.find(label_2);
	if(found_ptr == Vertices_map.end() ){
		std::cout << "Vertex 2 label NOT in graph" << std::endl;
		return;
	}
	
	Arc* current_arc = new Arc;
	
	current_arc->labels={label_1, label_2};
	current_arc->distance = distance;
	
	Vertices_map[label_1]->connections.push_back(current_arc);		
	Vertices_map[label_2]->connections.push_back(current_arc);		
	
	Arcs_map[current_arc->labels] = current_arc;
	
	
}







