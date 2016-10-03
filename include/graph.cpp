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





///////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////


RegionGraph::~RegionGraph(){

	for(NodeMapper::iterator it = Nodes_Map.begin(); it != Nodes_Map.end(); it++){
		Node* ptr = (*it).second;
	    delete  ptr;
	}
	Nodes_Map.clear();

	for(EdgeMapper::iterator it = Edges_Map.begin(); it != Edges_Map.end(); it++){
		Edge* ptr = (*it).second;
	    delete  ptr;
	}
	Edges_Map.clear();

	for(RegionNodeMapper::iterator it = Region_Nodes_Map.begin(); it != Region_Nodes_Map.end(); it++){
		Region_Node* ptr = (*it).second;
	    delete  ptr;
	}
	Region_Nodes_Map.clear();
	
	for(RegionEdgeMapper::iterator it = Region_Edges_Map.begin(); it != Region_Edges_Map.end(); it++){
		Region_Edge* ptr = (*it).second;
	    delete  ptr;
	}
	Region_Edges_Map.clear();


}

///////////////////////
std::ostream& operator<<(std::ostream& os, RegionGraph& Graph){
	os << "\n"<< "\n";
	
	os << "Number of Nodes "<< Graph.Nodes_Map.size() << "\n";
	os << "Number of Edges "<< Graph.Edges_Map.size() << "\n";
	os << "Number of Regions "<< Graph.Region_Nodes_Map.size() << "\n" << "\n";

//	os << "Number of Nodes per Region "<< "\n";
	for (RegionNodeMapper::iterator Region_iter = Graph.Region_Nodes_Map.begin(); Region_iter != Graph.Region_Nodes_Map.end(); Region_iter++){
		os << "   Region "<< (*Region_iter).first  <<": Number of Nodes: "<< (*Region_iter).second->nodes_inside.size() ;
		os << "        Subgraphs "<<  (*Region_iter).second->sub_graphs.size() << "\n";
		for( std::list < std::list <Node*> >::iterator it = (*Region_iter).second->sub_graphs.begin();it != (*Region_iter).second->sub_graphs.end(); it++ ){
			os << "        Subgraphs size "<<  (*it).size() << " nodes" <<"\n";
		}
		os << "   Connections "<< "\n";
		for(int i=0; i<  (*Region_iter).second->connected.size(); i++){
			std::set <int> link = (*Region_iter).second->connected[i]->Nodes_ids;
			os << "     ("<< (*link.begin()) <<","<<  (*link.rbegin()) <<  ")  " ;			
		}
		os << "\n";			

	}
	/////////////
	os << "Region Edges "<< "\n";
	for (RegionEdgeMapper::iterator Region_Edge_iter = Graph.Region_Edges_Map.begin(); Region_Edge_iter != Graph.Region_Edges_Map.end(); Region_Edge_iter++){
		std::set<int> link = (*Region_Edge_iter).second->Nodes_ids;
		os << "    Path ("<< (*link.begin()) <<","<<  (*link.rbegin()) <<  ")  has " << (*Region_Edge_iter).second->Edges_in_border.size()  << " connections" << "\n";	
	}
    return os;
}


//////////////////////////////



int RegionGraph::build_Region_Graph(std::vector<geometry_msgs::Point> edge_markers, nav_msgs::MapMetaData info, cv::Mat  Tag_image, cv::Mat  original_image){
	
	int number_of_edges = edge_markers.size()/2;				
	
	Node* from_Node_ptr; 
	Node* TO_Node_ptr;   
		
	std::complex<double> origin(info.origin.position.x, info.origin.position.y);	
	
	
	Region_Node* Unexplored_region = new Region_Node;
	Unexplored_region->id=-1;
	Region_Nodes_Map[-1] = Unexplored_region;	
	
	current_node_id = 0;
	
	for (int i=0; i < number_of_edges;i++){
		std::complex<double> FROM_position( edge_markers[2*i].x , edge_markers[2*i].y  );
		std::complex<double> TO_position  (edge_markers[2*i+1].x, edge_markers[2*i+1].y);

		int FROM_label = edge_markers[2*i].z*100;
		int TO_label   = edge_markers[2*i+1].z*100;

		current_node_id = std::max(current_node_id , FROM_label);
		current_node_id = std::max(current_node_id , TO_label);

		NodeMapper::iterator FROM_iter = Nodes_Map.find (FROM_label);
		NodeMapper::iterator TO_iter = Nodes_Map.find (TO_label);


//		std::cout << "FROM NODE"<< std::endl;
		if(FROM_iter == Nodes_Map.end() ){ //not found
			from_Node_ptr = new Node;
			from_Node_ptr->info.position = FROM_position;
			from_Node_ptr->info.label = FROM_label;
			
			////Find Region label
			FROM_position = (FROM_position - origin)/(double)info.resolution;
			int x = round(FROM_position.real() );
			int y = round(FROM_position.imag() );
			
			int region_tag = Tag_image.at<uchar>(cv::Point(x,info.height - y)) - 1;
			
			from_Node_ptr->info.region_label = region_tag;
			
			
			// Insert Region Node
			{
				RegionNodeMapper::iterator Region_iter = Region_Nodes_Map.find(region_tag);			
				if(Region_iter == Region_Nodes_Map.end() ){// not found
					Region_Node* current_region = new Region_Node;
					current_region->nodes_inside.push_back(from_Node_ptr);
					current_region->id = region_tag;
					Region_Nodes_Map[region_tag] = current_region;				
				}
				else{
					Region_Nodes_Map[region_tag]->nodes_inside.push_back(from_Node_ptr);
				}
			//////////
			}
			Nodes_Map[FROM_label] = from_Node_ptr;	//Inser Region Map
		}
		else {
			from_Node_ptr = (*FROM_iter).second;
		}



//		std::cout << "TO NODE"<< std::endl;
		if(TO_iter == Nodes_Map.end() ){ //not found
			TO_Node_ptr = new Node;
			TO_Node_ptr->info.position = TO_position;
			TO_Node_ptr->info.label = TO_label;
			
			////Find Region label
			TO_position = (TO_position - origin)/(double)info.resolution;
			int x = round(TO_position.real() );
			int y = round(TO_position.imag() );
			
			int region_tag = Tag_image.at<uchar>(cv::Point(x,info.height - y)) - 1;
			
			TO_Node_ptr->info.region_label = region_tag;
			
			
			// Insert Node to Regions
			{
				RegionNodeMapper::iterator Region_iter = Region_Nodes_Map.find(region_tag);			
				if(Region_iter == Region_Nodes_Map.end() ){// not found
					Region_Node* current_region = new Region_Node;
					current_region->nodes_inside.push_back(TO_Node_ptr);
					current_region->id = region_tag;
					Region_Nodes_Map[region_tag] = current_region;				
				}
				else{
					Region_Nodes_Map[region_tag]->nodes_inside.push_back(TO_Node_ptr);
				}
			//////////
			}
			Nodes_Map[TO_label] = TO_Node_ptr;	//Inser Region Map
		}
		else {
			TO_Node_ptr = (*TO_iter).second;
		}
		
		
//		std::cout << "Insert EDGES"<< std::endl;
		Edge* current_edge = new Edge;
		current_edge->info.distance = abs(FROM_position - TO_position);
		current_edge->info.label = i;
		
		current_edge->from = from_Node_ptr;
		current_edge->to   = TO_Node_ptr;  

		Edges_Map[i] = current_edge;


//		std::cout << "Insert Linked Information"<< std::endl;
		Connections connecting_from;
		connecting_from.linker = current_edge;			
		connecting_from.to = TO_Node_ptr;
		from_Node_ptr->connected.push_back(connecting_from);
		
		Connections connecting_to;
		connecting_to.linker = current_edge;			
		connecting_to.to = from_Node_ptr;
		TO_Node_ptr->connected.push_back(connecting_to);
		//Edges
		current_edge->from = from_Node_ptr;
		current_edge->to = TO_Node_ptr;
	}

//	std::cout << "   Everything inserted "<< std::endl;

	extract_subgraph();
//	std::cout << "   Subgraph extracted "<< std::endl;

	build_region_graph(Tag_image, original_image);
//	std::cout << "   Region Graph Built "<< std::endl;
	
	find_edges_between_regions();
//	std::cout << "   Edges Between Regions Added "<< std::endl;
	///////
	return 1;
}

/////////////////


void RegionGraph::extract_subgraph(){
	
	for (RegionNodeMapper::iterator Region_iter = Region_Nodes_Map.begin(); Region_iter != Region_Nodes_Map.end(); Region_iter++){
		std::list <Node*> Remaining_Nodes = (*Region_iter).second->nodes_inside, Nodes_in_sub_Region;
		int sub_region=0;

		while( Remaining_Nodes.size() > 0){// Breadth first
			evaluate_list_connectivity(Remaining_Nodes, sub_region);
			Remaining_Nodes.clear();
			Nodes_in_sub_Region.clear();

			for(Node_iter it = (*Region_iter).second->nodes_inside.begin(); it != (*Region_iter).second->nodes_inside.end(); it++){		
				if( (*it)->info.sub_region == -1 ){
					Remaining_Nodes.push_back(*it);
				}
				else if( (*it)->info.sub_region == sub_region ){
					Nodes_in_sub_Region.push_back(*it);
				}
			}
			sub_region++;
			(*Region_iter).second->sub_graphs.push_back(Nodes_in_sub_Region);
		}
		////		
	}
}
//////////
void RegionGraph::evaluate_list_connectivity(std::list <Node*> list_in, int name){

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




void RegionGraph::build_region_graph(cv::Mat  Tag_image, cv::Mat  original_image){
	
	int window_size=1;
	
	
	edge_points_mapper mapping_set_to_point_array, mapping_frontier_to_point_array;
	region_points_mapper mapping_region_to_point_array;
	
	for (int i=window_size;i < Tag_image.size().width- window_size ;i++){
		for (int j=window_size;j < Tag_image.size().height - window_size ;j++){
		
			/////////////////////
			cv::Point window_center(i,j);
			int center_tag = Tag_image.at<uchar>(window_center);
			
			std::set<int>  connections_in_region, frontier_connections;
			//check neigbourhood for frontiers and connection
			for(int x=-window_size; x <= window_size; x++){
				for(int y=-window_size; y <= window_size; y++){
					cv::Point delta(x,y); 							
					cv::Point current_point = window_center + delta;
					int tag = Tag_image.at<uchar>(current_point);
					int frontier = original_image.at<uchar>(current_point);
					
					if (tag>0){
						connections_in_region.insert( tag -1 );
						frontier_connections.insert( tag -1 );
					}
					if ( frontier==255 &&  tag==0) frontier_connections.insert( -1 );

				}
			}
			//////////////////////////////
			if (connections_in_region.size()==2 || (frontier_connections.size()==2 &&  ( (*frontier_connections.begin())==-1) ) ){
				mapping_region_to_point_array[center_tag-1].push_back(window_center);
			}

			//////////////////
			if(connections_in_region.size()==2){					
				mapping_set_to_point_array[connections_in_region].push_back(window_center);
				
			}
			if(frontier_connections.size()==2 &&  ( (*frontier_connections.begin())==-1) ){					
				mapping_frontier_to_point_array[frontier_connections].push_back(window_center);
	//					Frontier_image.at<uchar>( window_center ) = 255;
			}
		}
	}
	//////////////	
	
	// std::cout << "      Image processed "<< std::endl;
	
	for (region_points_mapper::iterator it2 = mapping_region_to_point_array.begin(); it2 != mapping_region_to_point_array.end(); it2 ++){
		int region_tag = (*it2).first;		
		RegionNodeMapper::iterator Region_iter = Region_Nodes_Map.find(region_tag);			
		if(Region_iter == Region_Nodes_Map.end() ){// not found
			Region_Node* current_region = new Region_Node;
			current_region->id = region_tag;
			Region_Nodes_Map[region_tag] = current_region;				
		}				
		Region_Nodes_Map[region_tag]->contour = (*it2).second;		
	}
	//	std::cout << "      Contour Extracted "<< std::endl;
		


	for (edge_points_mapper::iterator it2 = mapping_set_to_point_array.begin(); it2 != mapping_set_to_point_array.end(); it2 ++){
		Region_Edge *InsideEdge;
		InsideEdge = new Region_Edge;
		
		InsideEdge->frontier = it2->second;
		std::set<int> conection =it2->first;
		InsideEdge->First_Region  = Region_Nodes_Map[ (*conection.begin()) ];
		InsideEdge->Second_Region = Region_Nodes_Map[ (*conection.rbegin()) ];
		InsideEdge->Nodes_ids = conection;
		
		Region_Nodes_Map[ (*conection.begin()) ]->connected.push_back(InsideEdge);
		Region_Nodes_Map[ (*conection.rbegin()) ]->connected.push_back(InsideEdge);
		
		Region_Edges_Map[conection] =InsideEdge;	
	}
	// std::cout << "      Edge Extracted "<< std::endl;

	for (edge_points_mapper::iterator it2 = mapping_frontier_to_point_array.begin(); it2 != mapping_frontier_to_point_array.end(); it2 ++){
		Region_Edge *InsideEdge;
		InsideEdge = new Region_Edge;
		
		InsideEdge->frontier = it2->second;
		std::set<int> conection =it2->first;
		InsideEdge->First_Region  = Region_Nodes_Map[ (*conection.begin()) ];
		InsideEdge->Second_Region = Region_Nodes_Map[ (*conection.rbegin()) ];
		InsideEdge->Nodes_ids = conection;
		
		Region_Nodes_Map[ (*conection.begin()) ]->connected.push_back(InsideEdge);
		Region_Nodes_Map[ (*conection.rbegin()) ]->connected.push_back(InsideEdge);
		
		Region_Edges_Map[conection] =InsideEdge;	
	}
	// std::cout << "      Frontier Extracted "<< std::endl;


}



void RegionGraph::find_edges_between_regions(){
	std::list <Edge*> connecting_edges;
		
	for(EdgeMapper::iterator it = Edges_Map.begin(); it != Edges_Map.end(); it++){

		int region_from = (*it).second->from->info.region_label;
		int region_to   = (*it).second->to->info.region_label;
		
		std::set<int> connection;
		connection.insert(region_from);
		connection.insert(region_to);

			
		if( region_from != region_to ){
			Region_Edges_Map[connection]->Edges_in_border.push_back( (*it).second );
		}
		
	}

}


void RegionGraph::Tremaux_data(){	
	std::cout << "Current Node id is  "<< current_node_id << std::endl;
	Region_Node* current_Region = Region_Nodes_Map[ Nodes_Map[current_node_id]->info.region_label ];

	std::cout << "Current Region is  "<< current_Region->id << std::endl;

	std::cout << "Paths  " << std::endl;
	for( std::vector<Region_Edge*>::iterator region_edge_iter = current_Region->connected.begin(); region_edge_iter != current_Region->connected.end();region_edge_iter++){
		std::set<int> connections =  (*region_edge_iter)->Nodes_ids;
		std::cout << "  ( " << *(connections.begin() )<< " , "<< *(connections.rbegin() )  << " )"<< std::endl;
	}
	



	int a=1;
}





/*
Tremaux

*path is unvisited, marked once or twice

1. Remember direction taken

2. Is noded marked (has been visited)?
  2.1 Not marked -> choose a random direction and mark
  2.2 Is marked,
     2.2.1  if direction is marked once return the same direction
     2.2.2  else, pick direction with fewest marks (random if more than once)

*/







