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
	
	cv::Mat image_received, image_tagged;
	
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
			cv_ptr->encoding = "mono8";
			
			Uncertainty_sub_ = n.subscribe("query_Uncertainty", 10, &ROS_handler::UncertaintyCallback, this);
			pose_array_pub_  = n.advertise<geometry_msgs::PoseArray>("query_Poses", 10);
			
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
			image_received = img;
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
		}

////////////////
		void graphCallback(const visualization_msgs::Marker& graph_msg)
		{
			edges = graph_msg.points;

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
			  temporal_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& e)
			{
			  ROS_ERROR("cv_bridge exception: %s", e.what());
			  return;
			}
			
			image_tagged = temporal_ptr->image;
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


		void edges2Graph(){
			int a=1;
			std::pair<int,int> xyz;
		}

		void graph_iteration(){
			std::cout << "Edges received "<< edges.size() << std::endl;
			UtilityGraph GraphSLAM;
			GraphSLAM.build_graph_from_edges(edges);
			if(edges.size() > 0){
				GraphSLAM.update_distances(Last_node);
				GraphSLAM.print_nodes();
			}
//			cout << "Label of closest node "<< GraphSLAM.update_distances(Last_node) << endl;			
//			GraphSLAM.print_nodes();
		}


		cv::Mat graph2image(UtilityGraph &GraphSLAM, nav_msgs::MapMetaData info, cv::Mat  Tag_image ){
			cv::Mat  Node_image  = cv::Mat::zeros(info.height, info.width, CV_8UC1);
			cv::Mat  image_test  = cv::Mat::zeros(info.height, info.width, CV_8UC1);
			
			std::complex<double> origin(info.origin.position.x, info.origin.position.y);	

			for(Node_iter it = GraphSLAM.Nodes.begin(); it != GraphSLAM.Nodes.end(); it++ ){
				std::complex<double> current_node_position = (*it)->info.position;

				current_node_position = (current_node_position - origin)/(double)info.resolution;
				int x = round(current_node_position.real() );
				int y = round(current_node_position.imag() );

				Node_image.at<uchar>(cv::Point(x, info.height - y)) = Tag_image.at<uchar>(cv::Point(x,y));
				
				(*it)->info.region_label = Tag_image.at<uchar>(cv::Point(x,y)) -1;
			}

			if(false){
				return  Node_image;
			}
			else{
				Node_image=Node_image>0;
				cv::dilate(Node_image, Node_image, cv::Mat(), cv::Point(-1,-1), 3, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );			
				cv::flip(Tag_image,Tag_image,0);
				Tag_image.copyTo(image_test , ~Node_image);
				return  image_test;
			}
		}
/////////////////						
		typedef std::map < std::set<int> , std::vector<cv::Point>   > edge_points_mapper;
		void find_contour_connectivity_and_frontier(cv::Mat  Tag_image, cv::Mat  original_image){
			
			UtilityGraph Region_Graph;
			int window_size=3;
			

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
							if ( frontier==255) frontier_connections.insert( -1 );

						}
					}
					//////////////////
				if(connections_in_region.size()>1){					
					mapping_set_to_point_array[connections_in_region].push_back(window_center);
				}
				if(frontier_connections.size()>1 &&  ( (*frontier_connections.begin())==-1)  ){					
					mapping_frontier_to_point_array[frontier_connections].push_back(window_center);
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
			for (edge_points_mapper::iterator it2 = mapping_frontier_to_point_array.begin(); it2 != mapping_frontier_to_point_array.end(); it2 ++){

				std::cout << "Frontiers  are: ";
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
