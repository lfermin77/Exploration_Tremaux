#include "graph.hpp"



////Graph as adjacency list nested class

node_info::node_info(){
	distance_from_origin= std::numeric_limits<float>::infinity();
	label=-1;
	region_label=-1;
	sub_region=-1;
}
/////////////////////////////

edge_info::edge_info(){
	distance = -1;
	label=-1;
}


///////////////////
	
Node::Node(){
	predecesor=NULL;
}


void Node::print_node_label_and_pos(){
	std::cout << "Node "<< info.label<< ", region "<< info.region_label << std::endl;
	for(int i=0; i < connected.size();i++){
		std::cout << "connected to "<< (connected[i].to)->info.label<<std::endl;
		std::cout << "connected in ("<< (connected[i].linker)->from->info.label<<","<< (connected[i].linker)->to->info.label << ")"<<std::endl;
	}
}

/////////////////////	

UtilityGraph::~UtilityGraph(){
	for(Node_iter it = Nodes.begin(); it != Nodes.end(); it++)
	    delete *it; 
	Nodes.clear();

	for(Edge_iter it = Edges.begin(); it != Edges.end(); it++)
	    delete *it;
	Edges.clear();

}



void UtilityGraph::print_nodes(){
	for(Node_iter it = Nodes.begin(); it != Nodes.end(); it++){
//		(*it)->print_node_label_and_pos();
		std::cout << "Labels "<<(*it)->info.label<< " at distance " <<  (*it)->info.distance_from_origin << ", region "<< (*it)->info.region_label <<  std::endl;
	}
}



Node_iter UtilityGraph::find_point_in_node(std::complex<double> query_position){
	Node_iter first = Nodes.begin();
	Node_iter last = Nodes.end();

	while (first!=last) {
		if ((*first)->info.position == query_position) return first;
		++first;
	}
	return last;
}

int UtilityGraph::update_distances(	std::complex<double> current_position ){
	std::list <Node*> Unvisited_Nodes = Nodes; //List to work

	//// Find source node
	Node* Minimal_Node;
	Node_iter Source_iter = Unvisited_Nodes.end();
	for(Node_iter it = Unvisited_Nodes.begin(); it != Unvisited_Nodes.end(); it++){
		if((*it)->info.position ==  current_position){
			Minimal_Node = (*it);
			Source_iter = it;
		}
	}	
	Minimal_Node->predecesor = NULL;
	Minimal_Node->info.distance_from_origin = 0;
	
	std::cout << "Minimun distace found, is  " << Minimal_Node->info.label << std::endl;

	while(Unvisited_Nodes.size() > 0){
		Node_iter eliminate_iter = Unvisited_Nodes.begin();
		float min_distance = std::numeric_limits<float>::infinity();

		for(Node_iter it = Unvisited_Nodes.begin(); it != Unvisited_Nodes.end(); it++){
			if( ((*it)->info.distance_from_origin ) < min_distance ){
				eliminate_iter = it;
				min_distance = (*it)->info.distance_from_origin;
			}
		}
		Minimal_Node = (*eliminate_iter);

		Unvisited_Nodes.erase(eliminate_iter);
		
		for(int i=0; i < Minimal_Node->connected.size();i++){
			float new_distance = Minimal_Node->info.distance_from_origin  +  Minimal_Node->connected[i].linker->info.distance;

			if(new_distance < Minimal_Node->connected[i].to->info.distance_from_origin  ){
				Minimal_Node->connected[i].to->info.distance_from_origin = new_distance;
				Minimal_Node->connected[i].to->predecesor = Minimal_Node;
			}
		}
	}
	return -1;
}


std::list <Edge*> UtilityGraph::find_edges_between_regions(){
	std::list <Edge*> connecting_edges;
	std::cout << "Connecting Edges "<< std::endl;
		
	for(Edge_iter it = Edges.begin(); it != Edges.end(); it++){
		int region_from = (*it)->from->info.region_label;
		int region_to   = (*it)->to->info.region_label;

			
		if( region_from != region_to ){
			connecting_edges.push_back((*it) );
//			std::cout <<"Edge "<< (*it)->info.label << " connect the region " << region_from  <<" with "<< region_to << std::endl;
		}
		
		
	}
	return connecting_edges;
}


void UtilityGraph::evaluate_regions_connectivity(){
	std::map < int, std::list <Node*> > labels_to_nodes;
	
	std::vector < std::vector < std::list <Node*> > >   Regions_in_Graph;
	std::vector < std::list <Node*> >    Sub_Regions_in_Regions;
	Regions_in_Graph.clear();	
	
	for(Node_iter it = Nodes.begin(); it != Nodes.end(); it++){
		int region_label = (*it)->info.region_label;
		labels_to_nodes[region_label].push_back(*it);
	}
		
	for(std::map < int, std::list <Node*> >::iterator map_it = labels_to_nodes.begin(); map_it != labels_to_nodes.end() ; map_it++){
		int sub_region=0;
		std::list <Node*> Remaining_Nodes, List_of_Nodes = map_it->second, Nodes_in_sub_Region;
		
		Sub_Regions_in_Regions.clear();
		
		while( List_of_Nodes.size() > 0){// Breadth first
			evaluate_list_connectivity(List_of_Nodes, sub_region);
			Remaining_Nodes.clear();
			Nodes_in_sub_Region.clear();

			for(Node_iter it = map_it->second.begin(); it != map_it->second.end(); it++){		
				if( (*it)->info.sub_region == -1 ){
					Remaining_Nodes.push_back(*it);
				}
				if( (*it)->info.sub_region == sub_region ){
					Nodes_in_sub_Region.push_back(*it);
				}
			}
			List_of_Nodes = Remaining_Nodes;
			sub_region++;
			Sub_Regions_in_Regions.push_back(Nodes_in_sub_Region);
		}

		Regions_in_Graph.push_back(Sub_Regions_in_Regions);
	}

	std::cout << "The graph is decomposed in " << Regions_in_Graph.size()<<" Regions" <<std::endl;
		std::map < int, std::list <Node*> >::iterator map_it = labels_to_nodes.begin();
	for(int i=0; i< Regions_in_Graph.size(); i++){
		
		std::cout << " Region: " << map_it->first <<" with "<<Regions_in_Graph[i].size()<<" subregions" <<std::endl;
		std::vector < std::list <Node*> > Subregions_Inside = Regions_in_Graph[i];
		
		for(int j=0; j< Subregions_Inside.size(); j++){
			std::cout << "   Sub_Region: " << j <<" with "<< Subregions_Inside[j].size()<<" nodes"<<std::endl;
		}
		map_it++;
	}

}

void UtilityGraph::evaluate_list_connectivity(std::list <Node*> list_in, int name){

	list_in.front()->info.sub_region = name;
	
	int current_region_label = list_in.front()->info.region_label;

	std::queue<Node*> Q;
	Q.push( list_in.front() );

	
	while (!Q.empty()){
		Node* current = Q.front();		Q.pop();

		for (int i=0;i< current->connected.size();i++ ){
			Node* destiny =current->connected[i].to;

			if(destiny->info.region_label == current_region_label){
				if(destiny->info.sub_region != name ){	
					destiny->info.sub_region = name;
					Q.push(destiny);
				}
			}
		}
	} 
}




