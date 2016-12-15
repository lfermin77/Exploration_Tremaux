//ROS
#include "ros/ros.h"
#include "nav_msgs/GetMap.h"

#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"

#include <ros/package.h>

//openCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

//Graph
#include "graph.hpp"

// Standard
#include "time.h"
#include <fstream>

#include "graph_distance.hpp"


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
	ros::Subscriber ground_truth_sub_;
	
    ros::Publisher  pose_array_pub_;	
    ros::Publisher  markers_pub_;	
    ros::Publisher  goal_pub_;	
	ros::Timer timer;
	
	nav_msgs::MapMetaData map_info;				
	std::vector<geometry_msgs::Point> edges;	
	geometry_msgs::Point Last_node;
	int counter;
	
	cv::Mat image_map, image_tagged;
	
	bool map_received, path_received, graph_received, tagged_image_received, GT_received;
	geometry_msgs::PoseStamped pose_to_publish; 
	geometry_msgs::PoseStamped Last_goal; 
	bool trying_to_connect;


	//Statistics
	float distance;
	float cum_time;
	float time_counter;	
	std::ofstream myfile;
	ros::Subscriber save_sub_;	
	
	std::vector <geometry_msgs::Point> ground_truth;
	geometry_msgs::Point current_ground_truth;
	geometry_msgs::Point *first_pose;

	std::map<int, std::set<int> > simplified_graph;
	
	Graph_Distance Distances;

	
	
	public:
		ROS_handler(const std::string& mapname) : mapname_(mapname),  it_(n)
		{
			ROS_INFO("Waiting for the map");
			map_sub_	 		= n.subscribe("map", 2, &ROS_handler::mapCallback, this); //mapname_ to include different name
			trajectory_sub_ 	= n.subscribe("trajectory", 1, &ROS_handler::trajectoryCallback, this);			
			ground_truth_sub_ 	= n.subscribe("GT_poses", 1, &ROS_handler::ground_truth_Callback, this);			
			graph_sub_ 			= n.subscribe("SLAM_Graph", 10, &ROS_handler::graphCallback, this);			

			timer = n.createTimer(ros::Duration(0.5), &ROS_handler::metronomeCallback, this);
			image_pub_ = it_.advertise("/processed_image", 1);
			image_sub_ = it_.subscribe("tagged_image", 1,  &ROS_handler::imageCallback, this);
			
			markers_pub_     =  n.advertise<visualization_msgs::Marker>( "Exploration_Choices", 10 );
			
			cv_ptr.reset (new cv_bridge::CvImage);
//			cv_ptr->encoding = "mono8";
			
			Uncertainty_sub_ = n.subscribe("query_Uncertainty", 10, &ROS_handler::UncertaintyCallback, this);
			pose_array_pub_  = n.advertise<geometry_msgs::PoseArray>("query_Poses", 10);
			goal_pub_	  	 = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
			
			counter =0;
			map_received = path_received = graph_received = tagged_image_received = GT_received = false;
			trying_to_connect=false;
			
			//Statistics
			save_sub_ = n.subscribe("save_file", 10, &ROS_handler::UncertaintyCallback, this);
			Last_goal.header.seq = -1;
			distance = 0;
			time_counter = 0;
			cum_time=0;
			first_pose=NULL;
						
			std::string path = ros::package::getPath("exp_tremaux");
			path.append("/results/Results.txt");
			myfile.open(path);  
			myfile << "Iteration, Processing_Time, Distance, Area, LoopClosures, Current Error \n";

			
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

			geometry_msgs::Point New_node = msg.poses.front().position;
			
			float difference_x = New_node.x - Last_node.x;
			float difference_y = New_node.y - Last_node.y;
			float delta_distance =	sqrt( difference_x*difference_x +  difference_y*difference_y);
			
			distance += delta_distance;
			// UNSTUCK
			{			
				if( delta_distance  < 0.05){
					counter++;
				}
				else{
					counter=0;
				}
				
				if(counter > 20){
//				if(false){
					geometry_msgs::PoseStamped pose_out;
					pose_out.pose.orientation = msg.poses.front().orientation;
					double angle = 2*atan2(pose_out.pose.orientation.z, pose_out.pose.orientation.w);
	
					float dist = 0.15;
					pose_out.pose.position.x = New_node.x + dist*cos(angle);
					pose_out.pose.position.y = New_node.y + dist*sin(angle);
					
					publish_goal(pose_out);		
					
					//Reset last goal

					Last_goal.header.seq=-1;
					trying_to_connect=false;
					//*/		
				}
			}
			Last_node = New_node;
			path_received = true;
		}

///////////////////////
		void ground_truth_Callback(const visualization_msgs::Marker& GT_msg){
			ground_truth= GT_msg.points;
			GT_received= true;
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
			std::cout << "... ";
			
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

			bool data_ready = map_received & path_received & graph_received & tagged_image_received & GT_received;	
			cv::Mat grad;
			
			


			if(data_ready){
				cv::Mat occupancy_image = image_map.clone();
				std::cout << std::endl;
//				std::cout << "\033[2J\033[1;1H";				// clean whole console


				RegionGraph Tremaux_Graph;

				clock_t time_before = clock();
				Tremaux_Graph.build_Region_Graph(edges, map_info, image_tagged, occupancy_image);
				clock_t time_after = clock();

				float time_elapsed = 1000*((float)(time_after - time_before) )/CLOCKS_PER_SEC;

				std::map<int, std::set<int> >  new_graph = Tremaux_Graph.extract_simplified_graph();
				//*
				//Compare graphs

				{
					std::set< std::set<int> > new_edges;
					std::set<int> new_nodes;
					for(std::map<int, std::set<int> >::iterator graph_iter = new_graph.begin(); graph_iter != new_graph.end(); graph_iter ++){
						int new_label = graph_iter-> first;
						std::set<int> new_set   = graph_iter-> second;
						
						std::map<int, std::set<int> >::iterator old_node_iter = simplified_graph.find(new_label);
	
						if (old_node_iter == simplified_graph.end() ){
							std::cout << "  New node "<< new_label << std::endl;
							new_nodes.insert(new_label);
							//insert them all
							for(std::set<int>::iterator set_iter =  new_set.begin(); set_iter !=  new_set.end(); set_iter ++ ){
								std::set<int> current_arc={new_label, *set_iter};
								new_edges.insert(current_arc);
							}							
						}
						else{
							//check edges
							std::set<int> old_set = old_node_iter-> second;
							if(old_set != new_set){
							//	std::cout << "  Edge variation in node: "<< new_label << std::endl;
								for(std::set<int>::iterator set_iter =  new_set.begin(); set_iter !=  new_set.end(); set_iter ++ ){
									std::set<int>::iterator query_iter = old_set.find( *set_iter);
									if(query_iter == old_set.end() ){
										// new connection
										std::set<int> current_arc = {new_label, *set_iter};
										new_edges.insert(current_arc);
									//	std::cout << "     edge "<< new_label << " with "<< *set_iter << std::endl;
									}
	
								}							
								
							}
						}
						//
					}
					//////
					for( std::set< std::set<int> >::iterator set_set_iter = new_edges.begin(); set_set_iter != new_edges.end(); set_set_iter ++ ){ 
						std::set<int> connection = *set_set_iter;
						std::cout << "  New Edges: "<<*connection.begin() << " with " <<  *connection.rbegin()  <<   std::endl;
					}
					
					std::set< std::set<int> > processed_edge_set = new_edges;
					if(new_graph.size() == -1){
						std::cout << "  FIRST GRAPH "<< std::endl;
						float edge_distance_1 = Tremaux_Graph.get_edge_distance(0,1);
						Distances.insert_first_edge(0,1, Tremaux_Graph.get_edge_distance(0,1) );

						std::cout << Distances << std::endl;
						
						int label_new_node=2;
						std::vector< std::pair<int, float> > edges_in_new_node;
//						std::set< std::set<int> > remaining_edges;
						processed_edge_set.clear();
						
						for( std::set< std::set<int> >::iterator set_set_iter = new_edges.begin(); set_set_iter != new_edges.end(); set_set_iter ++ ){ 
							std::set<int> connection = *set_set_iter;
							if( *connection.begin() == label_new_node || *connection.rbegin() == label_new_node ){
								std::pair<int, float> pair_to_include;
								pair_to_include.second = Tremaux_Graph.get_edge_distance(*connection.begin() , *connection.rbegin() );
								
								pair_to_include.first =(*connection.begin() == label_new_node) ? *connection.rbegin() : *connection.begin() ;
								/*
								pair_to_include.first = *connection.begin();								
								if( *connection.begin() == label_new_node){
									pair_to_include.first = *connection.rbegin();
								}
								//*/
								edges_in_new_node.push_back(pair_to_include);
							}
							else{
								processed_edge_set.insert(connection);
							}
						//
						}
						//
						
						Distances.insert_new_node(label_new_node,  edges_in_new_node);
						std::cout << Distances << std::endl;

						processed_edge_set.clear();
						new_nodes.clear();
					}
					if(new_nodes.size() > 0){

						
						for(std::set<int>::iterator set_iter = new_nodes.begin(); set_iter != new_nodes.end(); set_iter ++ ){
							int label_new_node = *set_iter;
							std::vector< std::pair<int, float> > edges_in_new_node;
							
							std::set< std::set<int> > remaining_edges = processed_edge_set;
							processed_edge_set.clear();
							//Find edges of the node
							for( std::set< std::set<int> >::iterator set_set_iter = remaining_edges.begin(); set_set_iter != remaining_edges.end(); set_set_iter ++ ){ 
								std::set<int> connection = *set_set_iter;
								bool label_in_set = ( *connection.begin() == label_new_node) || (*connection.rbegin() == label_new_node);

								if(  label_in_set ){
									std::pair<int, float> pair_to_include;
									pair_to_include.second = Tremaux_Graph.get_edge_distance(*connection.begin() , *connection.rbegin() );

									pair_to_include.first =    (*connection.begin() == label_new_node)?  *connection.rbegin() : *connection.begin() ;

									if(pair_to_include.first < label_new_node)      edges_in_new_node.push_back(pair_to_include);
									else  processed_edge_set.insert(connection);
								}
								else{
									processed_edge_set.insert(connection);
								}
								//
							}

							//
							Distances.insert_new_node(label_new_node,  edges_in_new_node);
							std::cout << Distances << std::endl;
						}
					}
//					std::cout << "  size of remaining edges: "<< processed_edge_set.size()  <<   std::endl;
					if(processed_edge_set.size() !=0){
						std::cout << "  MEMORY REMAINS: "  <<   std::endl;

					}
				}


				
				
				
			

				//*/
				simplified_graph = new_graph;
				/*
				// Print Graph
				for(std::map<int, std::set<int> >::iterator graph_iter = simplified_graph.begin(); graph_iter != simplified_graph.end(); graph_iter ++){
					std::set<int> connection_set = graph_iter->second;
					std::cout << "  node " << graph_iter->first << ": ";										
					for(std::set<int>::iterator set_iter = connection_set.begin(); set_iter != connection_set.end(); set_iter ++){
						int value = *set_iter; 
						std::cout << "  " << *set_iter;
					}
					std::cout << "  " << std::endl;
				}
				//*/



//				std::cout << "Elapsed time in building: "<< time_elapsed <<" ms" << std::endl<< std::endl;
//				std::cout << Tremaux_Graph;			

//				time_before = clock();
//				Tremaux_Graph.choose_goal(pose_to_publish) ;  
//				int region_completed = Tremaux_Graph.Tremaux_data(pose_to_publish) ;  

				geometry_msgs::PoseStamped center_goal; 
				int valid_goal = Tremaux_Graph.connect_inside_region_closer(center_goal);
				//*
				int last_goal_reached = Tremaux_Graph.check_if_old_goal_is_in_current_sub_graph(center_goal);
				if(last_goal_reached >0){
					std::cout << "goal in path "<< std::endl;
				}
				else{
					std::cout << "goal NOT in path "<< std::endl;
				}
				//*/
				/////
				if(valid_goal > 0){
					std::cout << "isNOTconnected "<< std::endl;
				}
				else{
					std::cout << " isconnected "<< std::endl;
				}
				


				
				int region_completed = Tremaux_Graph.choose_goal(pose_to_publish) ;  
				time_after = clock();
				time_elapsed = 1000*((float)(time_after - time_before) )/CLOCKS_PER_SEC;
				{// RESULTS
					std::cout << std::endl<< std::endl;
					std::cout << "Current distance traveled: "<< distance <<" m" << std::endl;
									
					time_counter ++;				cum_time += time_elapsed;
					float Area = (map_info.resolution)*(map_info.resolution)* (cv::countNonZero(image_tagged) ) ;
	
					std::cout << "Elapsed time average: "<< cum_time/time_counter <<" ms" << std::endl;
	//				std::cout << "Number of Loop Closures: "<< Tremaux_Graph.number_loop_closures()  << std::endl;
					std::cout << "Area: "<< Area  << std::endl;
					

					
					float average_error;
					std::vector<float> error_list = Tremaux_Graph.extract_error_per_node(ground_truth, average_error);

					float current_error = 		error_list.back();			
					std::cout << "Current_error: "<< current_error  << std::endl;
					std::cout << "Average_error: "<< average_error  << std::endl;
					
					std::cout<< std::endl;
					
					
					myfile << time_counter <<", "<< time_elapsed <<", "<<  distance <<", "<< Area <<", "<<  Tremaux_Graph.number_loop_closures()<<", "<< current_error <<"\n";
				}



				/*
				if (region_completed < 0  && Last_goal.header.seq != -1){ //return to center of region
//				if (is_connected > 0  && Last_goal.header.seq != -1){ // return till connected

					float difference_x = Last_goal.pose.position.x - Last_node.x;
					float difference_y = Last_goal.pose.position.y - Last_node.y;
					
					float diff = sqrt( difference_x*difference_x +  difference_y*difference_y);
					Last_goal.header.seq = 0;
						
					if(diff < 0.1){				// Is it near the pose?
						publish_goal(pose_to_publish);
						Last_goal.pose = pose_to_publish.pose;
						Last_goal.header.seq = -1;
					}
					else{
						std::cout << " Trying to connect Sub-Graphs" << std::endl;		
					}
					
				}
				else
				//*/
				
				
				//*
				if(trying_to_connect){
					std::cout << "Trying_to_connect "<< std::endl;
					int last_goal_reached = Tremaux_Graph.check_if_old_goal_is_in_current_sub_graph(Last_goal);
//					if (last_goal_reached > 0){
					if (valid_goal < 0){
						std::cout << "Region is now considered connected "<< std::endl;
						trying_to_connect = false;
						publish_goal(pose_to_publish);						
						Last_goal.pose = pose_to_publish.pose;
					}
					else{
						// do nothing, keep on connecting
						std::cout << "keep on connecting "<< std::endl;
						trying_to_connect= true;
					}
					////
				}
				else{ // NOT connecting
					std::cout << "not connecting "<< std::endl;
					int is_center_connected = Tremaux_Graph.check_if_center_is_in_current_sub_graph();
					
					
//					if(is_center_connected < 0){ // CHECK IF THE RIGHT FUNCTION
					if(valid_goal > 0){ // CHECK IF THE RIGHT FUNCTION
//						center_goal = Tremaux_Graph.region_center();
						publish_goal(center_goal);
						trying_to_connect= true;
						std::cout << "will try to connect "<< std::endl;
						Last_goal.pose = pose_to_publish.pose;
					}
					else{
						publish_goal(pose_to_publish);
						std::cout << "regular exploring "<< std::endl;
					}						
				}
				//*/
				
				
				
				cv::Mat edge_image = Tremaux_Graph.segment_current_frontier ( image_tagged );
				grad = edge_image.clone();

				
				data_ready = map_received = path_received = graph_received = tagged_image_received = GT_received = false;
//				ground_truth.push_back(current_ground_truth);
				std::cout<<"Ground Truth size "<< ground_truth.size() <<std::endl;
				
//				std::cout << Tremaux_Graph;			
				
//				publish_markers(Tremaux_Graph.collect_all_frontiers());
				publish_markers(Tremaux_Graph.exploration_status());
				
				std::cout << std::endl << std::endl;	
				
			}
			else{
//				grad = image_tagged;
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

		
		int publish_markers(std::vector < std::complex <double> >  region_frontier) {
			
			visualization_msgs::Marker marker;
		
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time();

			
		
			marker.ns = "my_namespace";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::POINTS;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = 0;
			marker.pose.position.y = 0;
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.2;
			marker.scale.y = 0.2;
			marker.scale.z = 0.2;
			marker.color.a = 1.0; // Don't forget to set the alpha!
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
		
			
		
			marker.points.clear();

			for(int i=0;i< region_frontier.size();i++){
				geometry_msgs::Point point_from;
				
				point_from.x = region_frontier[i].real();
				point_from.y = region_frontier[i].imag();
				point_from.z = 0;
				
				marker.points.push_back(point_from);
			}
			
			
		
		
			markers_pub_.publish( marker );	
			
			
			return 0;
		}
		
		int publish_goal(geometry_msgs::PoseStamped  goal_msg) {
			goal_msg.header.frame_id = "map";
			goal_msg.header.stamp = ros::Time();
			
			goal_pub_.publish(goal_msg);
		}

/////////////////////////
//// UTILITY
/////////////////////////


		typedef std::map < std::set<int> , std::vector<cv::Point>   > edge_points_mapper;





};










int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Exploration_Tremaux");
	
	std::string mapname = "map";	
	ROS_handler mg(mapname);
	ros::spin();
	
	return 0;
}
