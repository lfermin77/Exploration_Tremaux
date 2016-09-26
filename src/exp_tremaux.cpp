//ROS
#include "ros/ros.h"
#include "nav_msgs/GetMap.h"

#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseArray.h"


//openCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

//Graph
#include "graph.hpp"
#include "graph_Regions.hpp"





class ROS_handler
{
	ros::NodeHandle n;
	
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;	
	cv_bridge::CvImagePtr cv_ptr;
		
	std::string mapname_;
	ros::Subscriber map_sub_;	
	ros::Subscriber graph_sub_;	
	ros::Subscriber Uncertainty_sub_;
	ros::Subscriber trajectory_sub_;
	
    ros::Publisher  pose_array_pub_;	
	ros::Timer timer;
	
	nav_msgs::MapMetaData map_info;				
	std::vector<geometry_msgs::Point> edges;	
	geometry_msgs::Point Last_node;
	
	cv::Mat image_map, image_tagged;
	
	bool map_received, path_received, graph_received, tagged_image_received;
	
	public:
		ROS_handler(const std::string& mapname) : mapname_(mapname),  it_(n)
		{
			ROS_INFO("Waiting for the map");
			map_sub_ = n.subscribe("map", 2, &ROS_handler::mapCallback, this); //mapname_ to include different name
			trajectory_sub_ = n.subscribe("trajectory", 1, &ROS_handler::trajectoryCallback, this);			
			graph_sub_ = n.subscribe("SLAM_Graph", 10, &ROS_handler::graphCallback, this);			

			timer = n.createTimer(ros::Duration(0.5), &ROS_handler::metronomeCallback, this);
			image_pub_ = it_.advertise("/processed_image", 1);
			image_sub_ = it_.subscribe("tagged_image", 1,  &ROS_handler::imageCallback, this);
			
			cv_ptr.reset (new cv_bridge::CvImage);
//			cv_ptr->encoding = "mono8";
			
			Uncertainty_sub_ = n.subscribe("query_Uncertainty", 10, &ROS_handler::UncertaintyCallback, this);
			pose_array_pub_  = n.advertise<geometry_msgs::PoseArray>("query_Poses", 10);
			
			map_received = path_received = graph_received = tagged_image_received = false;
			
		}



/////////////////////////////	
// ROS CALLBACKS			
////////////////////////////////		

		void mapCallback(const nav_msgs::OccupancyGridConstPtr& map){
			map_info = map->info;		
			
			///////////////////////Occupancy to image	
			cv::Mat grad, img(map->info.height, map->info.width, CV_8U);
			img.data = (unsigned char *)(&(map->data[0]) );
			cv::flip(img,img,0);
			image_map = img.clone();
			map_received = true;
			
		}


/////////////////		
		void metronomeCallback(const ros::TimerEvent&)
		{
//		  ROS_INFO("tic tac");
		  publish_Image();
		}

////////////////
		void trajectoryCallback(const geometry_msgs::PoseArray &msg)
		{
			/*
			for(int i=0; i< msg.poses.size();i++){
				std::cout << "Pose is "<< msg.poses[i].position.x  <<std::endl;
			}//*/
			Last_node = msg.poses.front().position;
			path_received = true;
		}

////////////////
		void graphCallback(const visualization_msgs::Marker& graph_msg)
		{
			edges = graph_msg.points;
			graph_received = true;
		}

///////////////
		void UncertaintyCallback(const geometry_msgs::PoseWithCovariance& msg)
		{
			int a=1;
		}
//////////////
		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			cv_bridge::CvImagePtr temporal_ptr;

			try
			{
			  temporal_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
			}
			catch (cv_bridge::Exception& e)
			{
			  ROS_ERROR("cv_bridge exception: %s", e.what());
			  return;
			}
			
			image_tagged = temporal_ptr->image.clone();
			tagged_image_received = true;
			
			image_tagged.convertTo(image_tagged, CV_8UC1);

			bool data_ready = map_received & path_received & graph_received & tagged_image_received;	
			cv::Mat grad;
			
			UtilityGraph GraphSLAM;
			build_graph_from_edges(edges, GraphSLAM);

			if(data_ready){
				cv::Mat occupancy_image = image_map.clone();

				grad =graph2image(GraphSLAM, map_info, image_tagged);				
				find_contour_connectivity_and_frontier(image_tagged, occupancy_image);
//				GraphSLAM.print_nodes();
				GraphSLAM.find_edges_between_regions();

				std::complex<double> complex_last_node(Last_node.x, Last_node.y);				
				GraphSLAM.update_distances(	complex_last_node );
				
				GraphSLAM.evaluate_regions_connectivity();

				double min, max;
				cv::minMaxLoc(image_tagged, &min, &max);
				
				std::list <Node*> node_pairs;
				for(int i=0;i < max; i++){
					std::cout << "Region "<< i << " analysis " << std::endl;
					GraphSLAM.Closest_subregions(node_pairs,i);
				}
			}
			else{
				grad = image_tagged;
			}
			
			cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			grad.convertTo(grad, CV_32F);
			grad.copyTo(cv_ptr->image);////most important
					
		}


////////////////////////
// PUBLISHING METHODS		
////////////////////////////		
		void publish_Image(){
			image_pub_.publish(cv_ptr->toImageMsg());
		}



/////////////////////////
//// UTILITY
/////////////////////////
		cv::Mat graph2image(UtilityGraph &GraphSLAM, nav_msgs::MapMetaData info, cv::Mat  Tag_image ){
			cv::Mat  Node_image  = cv::Mat::zeros(info.height, info.width, CV_8UC1);
			cv::Mat  image_test  = cv::Mat::zeros(info.height, info.width, CV_8UC1);
			
			std::complex<double> origin(info.origin.position.x, info.origin.position.y);	

			for(Node_iter it = GraphSLAM.Nodes.begin(); it != GraphSLAM.Nodes.end(); it++ ){
				std::complex<double> current_node_position = (*it)->info.position;

				current_node_position = (current_node_position - origin)/(double)info.resolution;
				int x = round(current_node_position.real() );
				int y = round(current_node_position.imag() );

				int current_tag = Tag_image.at<uchar>(cv::Point(x,info.height - y));

				Node_image.at<uchar>(cv::Point(x, info.height - y)) = current_tag;
				
				(*it)->info.region_label = current_tag -1;
			}

			if(false){
				return  Node_image;
			}
			else{
				Node_image=Node_image>0;
				cv::dilate(Node_image, Node_image, cv::Mat(), cv::Point(-1,-1), 3, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );			
				Tag_image.copyTo(image_test , ~Node_image);
				return  image_test;
//				return  Node_image;
			}
		}
/////////////////						
		typedef std::map < std::set<int> , std::vector<cv::Point>   > edge_points_mapper;
		
		cv::Mat find_contour_connectivity_and_frontier(cv::Mat  Tag_image, cv::Mat  original_image){
			
			UtilityGraph Region_Graph;
			int window_size=1;
			
			cv::Mat  Frontier_image  = cv::Mat::zeros(original_image.size(), CV_8UC1);
			
			edge_points_mapper mapping_set_to_point_array, mapping_frontier_to_point_array;
			for (int i=window_size;i < Tag_image.size().width- window_size ;i++){
				for (int j=window_size;j < Tag_image.size().height - window_size ;j++){
				
					/////////////////////
					cv::Point window_center(i,j);
					
					std::set<int>  connections_in_region, frontier_connections;
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
			
			
//*
			for (edge_points_mapper::iterator it2 = mapping_set_to_point_array.begin(); it2 != mapping_set_to_point_array.end(); it2 ++){

				std::cout << "Connections  are: ";
				std::set<int> current_connections_set = it2->first ;
				for (std::set<int>::iterator it = current_connections_set.begin(); it != current_connections_set.end(); it ++){
					std::cout <<" " << *it;
				}
				
				std::vector<cv::Point> current_points = it2->second;
				cv::Point average_point(0,0);

				for (std::vector<cv::Point>::iterator it = current_points.begin(); it != current_points.end(); it ++){
					average_point += *it;
				}		
				std::cout <<" at position (" << average_point.x/current_points.size() << " , " << average_point.y/current_points.size() << ")";
				
				std::cout << std::endl;
			}
			//*/
			
//*
			std::cout << "Frontiers size is: "<<  mapping_frontier_to_point_array.size() << std::endl;
			for (edge_points_mapper::iterator it2 = mapping_frontier_to_point_array.begin(); it2 != mapping_frontier_to_point_array.end(); it2 ++){
				
				if(it2->first.size() <3){
					std::cout << "Frontiers  are: ";
					std::set<int> current_connections_set = it2->first ;
					for (std::set<int>::iterator it = current_connections_set.begin(); it != current_connections_set.end(); it ++){
						std::cout <<" " << *it;
					}
					
					std::vector<cv::Point> current_points = it2->second;
					cv::Point average_point(0,0);	
					for (std::vector<cv::Point>::iterator it = current_points.begin(); it != current_points.end(); it ++){
						average_point += *it;
						Frontier_image.at<uchar>( *it ) = 255;
					}		
					std::cout <<" at position (" << average_point.x/current_points.size() << " , " << average_point.y/current_points.size() << ") size: "<< current_points.size();
					
					std::cout << std::endl;

				}

			}
			return Frontier_image;
//			return original_image;
			//*/



				

		}

/////////////////////////////////////////////////////////////////
		int build_graph_from_edges(std::vector<geometry_msgs::Point> edge_markers, UtilityGraph& UGraph){
			
			int number_of_edges = edge_markers.size()/2;				
			int labeler=0;
			
			Node* from_Node_ptr; 
			Node* to_Node_ptr;   
					
			for (int i=0; i < number_of_edges;i++){
			
		//		cout << "Insert nodes"<< endl;
				// FROM
				std::complex<double> FROM_position(edge_markers[2*i].x, edge_markers[2*i].y);
				std::cout << "label " << 100*edge_markers[2*i].z << std::endl;
							
				Node_iter FROM_node_iter = UGraph.find_point_in_node(FROM_position);		
				if(FROM_node_iter == UGraph.Nodes.end() ){//couldn't find, insert new node
					labeler++;
					from_Node_ptr =new Node;
					from_Node_ptr->info.position = FROM_position;
					from_Node_ptr->info.label = labeler;
					UGraph.Nodes.push_back(from_Node_ptr);	
				}			
				else from_Node_ptr = *FROM_node_iter;
				
				// TO
				std::complex<double> TO_position(edge_markers[2*i+1].x, edge_markers[2*i+1].y);			
				Node_iter TO_node_iter = UGraph.find_point_in_node(TO_position);			
				if(TO_node_iter == UGraph.Nodes.end() ){//couldn't find, insert new node
					labeler++;
					to_Node_ptr = new Node;
					to_Node_ptr->info.position = TO_position;
					to_Node_ptr->info.label = labeler;
					UGraph.Nodes.push_back(to_Node_ptr);				
				}			
				else to_Node_ptr = *TO_node_iter;
		
		
		//		cout << "Insert Edges"<<endl;
				Edge* current_edge = new Edge;
				current_edge->info.distance = abs(FROM_position - TO_position);
				current_edge->info.label = i;
				
				current_edge->from = from_Node_ptr;
				current_edge->to   = to_Node_ptr;  
				
				UGraph.Edges.push_back(current_edge);
			
		//		cout << "Insert Linked Information"<<endl;
				Connections connecting_from;
				connecting_from.linker = current_edge;			
				connecting_from.to = to_Node_ptr;
				from_Node_ptr->connected.push_back(connecting_from);
				
				Connections connecting_to;
				connecting_to.linker = current_edge;			
				connecting_to.to = from_Node_ptr;
				to_Node_ptr->connected.push_back(connecting_to);
				//Edges
				current_edge->from = from_Node_ptr;
				UGraph.Edges.back()->to = to_Node_ptr;
			}
			std::cout << "Found " << labeler  <<" nodes"<< std::endl;

			return 1;
		}






};










int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Exploration_Tremaux");
	
	std::string mapname = "map";	
	ROS_handler mg(mapname);
	ros::spin();
	
	return 0;
}
