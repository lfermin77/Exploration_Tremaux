//ROS
#include "ros/ros.h"
#include "ros/package.h"
#include "nav_msgs/GetMap.h"

#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"


//openCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

//Graph
#include "graph.hpp"

// Standard
#include "time.h"
#include <fstream>



class ROS_handler
{
	ros::NodeHandle n;
	
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;	
	cv_bridge::CvImagePtr cv_ptr;
	
	cv_bridge::CvImagePtr cv_ptr_color;
	std::vector <cv::Vec3b> colormap;
		
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

	//Statistics
	float distance;
	float cum_time;
	float time_counter;	
	std::ofstream myfile;
	ros::Subscriber save_sub_;	
	
	std::vector <geometry_msgs::Point> ground_truth;
	geometry_msgs::Point current_ground_truth;
	geometry_msgs::Point *first_pose;
	
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
			cv_ptr_color.reset (new cv_bridge::CvImage);
//			cv_ptr_color->encoding = "mono8";
			cv_ptr_color->encoding = "bgr8";
			
			
			image_sub_ = it_.subscribe("tagged_image", 1,  &ROS_handler::imageCallback, this);
			
			markers_pub_     =  n.advertise<visualization_msgs::Marker>( "Exploration_Choices", 10 );
			
			cv_ptr.reset (new cv_bridge::CvImage);
//			cv_ptr->encoding = "mono8";
			
			Uncertainty_sub_ = n.subscribe("query_Uncertainty", 10, &ROS_handler::UncertaintyCallback, this);
			pose_array_pub_  = n.advertise<geometry_msgs::PoseArray>("query_Poses", 10);
			goal_pub_	  	 = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
//			goal_pub_	  	 = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal_2", 10);
			
			
			
			
			
			cv::Vec3b black(208, 208, 208);
			colormap.push_back(black);
			for(int i=0;i<= 500; i++){
				cv::Vec3b color(rand() % 255,rand() % 255,rand() % 255);
				colormap.push_back(color);
			}
			
			
			
			
			
			
			
			
			counter =0;
			map_received = path_received = graph_received = tagged_image_received = GT_received = false;
			
			//Statistics
			save_sub_ = n.subscribe("save_file", 10, &ROS_handler::UncertaintyCallback, this);
			Last_goal.header.seq = -1;
			distance = 0;
			time_counter = 0;
			cum_time=0;
			first_pose=NULL;
			
			std::string path_exploration =ros::package::getPath ("exp_tremaux");
			
			path_exploration += "/results/Results.txt";

			std::cout << "Path "<< path_exploration << std::endl<< std::endl<< std::endl;
			
			myfile.open(path_exploration);  

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
		//			std::cout << "Counter is " << counter << std::endl;
				}
				else{
					counter=0;
				}
				
				if(counter > 20){
					geometry_msgs::PoseStamped pose_out;
					pose_out.pose.orientation = msg.poses.front().orientation;
					double angle = 2*atan2(pose_out.pose.orientation.z, pose_out.pose.orientation.w);
	
					float dist = 0.15;
					pose_out.pose.position.x = New_node.x + dist*cos(angle);
					pose_out.pose.position.y = New_node.y + dist*sin(angle);
					
					publish_goal(pose_out);		
					
					//Reset last goal

					Last_goal.header.seq=-1;
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
			std::cout << "New Image"<< std::endl;
			cv_bridge::CvImagePtr temporal_ptr;

			try
			{
			  temporal_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
//			  temporal_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
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
			
//			grad = paint_image_colormap(image_tagged, colormap);
			
//			grad = image_tagged;
//			cv_ptr->encoding = cv_ptr->encoding = "bgr8";			grad.convertTo(grad, CV_32F);

			if(data_ready){
				cv::Mat occupancy_image = image_map.clone();
				


				RegionGraph Tremaux_Graph;

				clock_t time_before = clock();
				Tremaux_Graph.build_Region_Graph(edges, map_info, image_tagged, occupancy_image);
				clock_t time_after = clock();

				float time_elapsed = 1000*((float)(time_after - time_before) )/CLOCKS_PER_SEC;
//				std::cout << "Elapsed time in building: "<< time_elapsed <<" ms" << std::endl<< std::endl;
//				std::cout << Tremaux_Graph;			

//				time_before = clock();
				int region_completed = Tremaux_Graph.Tremaux_data(pose_to_publish) ;  
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


				//*
				if (region_completed < 0  & Last_goal.header.seq != -1){
//					Tremaux_Graph.connect_inside_region(pose_to_publish);

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
				{
					publish_goal(pose_to_publish);
					Last_goal.pose = pose_to_publish.pose;
					Last_goal.header.seq = 0;
				}	
				publish_markers(Tremaux_Graph.collect_all_frontiers());
				
				cv::Mat edge_image = Tremaux_Graph.segment_current_frontier ( image_tagged );
//				grad = edge_image.clone();

				
				data_ready = map_received = path_received = graph_received = tagged_image_received = GT_received = false;
//				ground_truth.push_back(current_ground_truth);
				std::cout<<"Ground Truth size "<< ground_truth.size() <<std::endl;
				std::cout << std::endl << std::endl;
				
//				std::vector<std::complex<int> > node_center_in_pixels =Tremaux_Graph.extract_region_node_centers();	
				
							grad = paint_image_colormap(image_tagged, colormap, &Tremaux_Graph);
				
			}
			else{
				grad = image_tagged;
			}
			
//			cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;			grad.convertTo(grad, CV_32F);
			cv_ptr->encoding = "bgr8";			//grad.convertTo(grad, CV_32F);
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

	cv::Mat paint_image_colormap(cv::Mat image_in, std::vector <cv::Vec3b> color_vector, RegionGraph* Tremaux_Graph_Draw){



		cv::Mat image_float = cv::Mat::zeros(image_in.size(), CV_8UC3);


		// Color the regions
		for(int i=0; i < image_in.rows; i++){
			for(int j=0;j< image_in.cols; j++){
				int color_index = image_in.at<uchar>(i,j);
				image_float.at<cv::Vec3b>(i,j) = color_vector[color_index];
			}
		}
		
		
				// Paint Edges
		std::vector<std::complex<int> > pixel_from_vec;
		std::vector<std::complex<int> > pixel_to_vec;
		std::vector<int> traverse_code;
		
		Tremaux_Graph_Draw->extract_regions_edges(&pixel_from_vec, &pixel_to_vec, &traverse_code);
//		std::cerr << " Size " <<pixel_from_vec.size() << std::endl;
		
		for(int i=0;i < pixel_from_vec.size();i++){			
			cv::Point Start_link (pixel_from_vec[i].real(), pixel_from_vec[i].imag()   );
			cv::Point End_link(pixel_to_vec[i].real(), pixel_to_vec[i].imag());

			
			if(traverse_code[i]==0)
				cv::line( image_float, Start_link, End_link, cv::Scalar( 0, 255, 0 ), 3, 8);
			if(traverse_code[i]==1)
				cv::line( image_float, Start_link, End_link, cv::Scalar( 0, 0, 255 ), 3, 8);
			if(traverse_code[i]==2)
				cv::line( image_float, Start_link, End_link, cv::Scalar( 128, 128, 128 ), 3, 8);


		}
			
			
			


		// Paint the nodes
		std::vector<std::complex<int> > node_center_in_pixels =Tremaux_Graph_Draw->extract_region_node_centers();	
		for(int i = 0; i < node_center_in_pixels.size();i++){
			cv::Point flip_centroid;
			flip_centroid.x = node_center_in_pixels[i].real();
			flip_centroid.y = node_center_in_pixels[i].imag();
			
//			circle( image_float,flip_centroid,5,cv::Scalar(0, 255, 0),-1,8);
			circle( image_float,flip_centroid,6,cv::Scalar(0, 255, 0),-1,8);
			circle( image_float,flip_centroid,3,cv::Scalar(0, 0, 255),-1,8);
		 }



		//Paint edge contour
		std::vector<cv::Point > edge_contour;
		Tremaux_Graph_Draw->extract_edges_contour(edge_contour);			
		for(int i = 0; i < edge_contour.size();i++){
			circle( image_float,edge_contour[i],1,cv::Scalar(255,0, 0),-1,8);
		 }

//		cv::flip(image_float,image_float,0);
		
		
		
		
		
		/*
//		std::cerr<<"Stable.Region_centroid.size()"<<Stable.Region_centroid.size()<<std::endl;

		std::cerr << "Current edges " << Stable.diagonal_connections.size() << std::endl;
		
		//*
		for(int i=0;i < Stable.diagonal_centroid.size();i++){
			int region_from =*(Stable.diagonal_connections[i].begin() );
			int region_to =*(Stable.diagonal_connections[i].rbegin() );
			
			cv::Point Start_link = Stable.Region_centroid[region_from];
			cv::Point End_link = Stable.Region_centroid[region_to];

			std::cerr << "    from "<<region_from << ", to " << region_to << std::endl;
			
			cv::line( image_float, Start_link, Stable.diagonal_centroid[i], cv::Scalar( 0, 255, 0 ), 3, 8);
			cv::line( image_float, Stable.diagonal_centroid[i], End_link, cv::Scalar( 0, 255, 0 ), 3, 8);
			
		}
		//
		std::cerr << "Current frontier " << Stable.region_frontier.size() << std::endl;
		
		for(int i=0;i < Stable.region_frontier.size();i++){
			int region = Stable.region_frontier[i];
			
			cv::Point Start_link = Stable.Region_centroid[region];
			cv::Point End_link = Stable.center_of_frontier[i];

			std::cerr << "    from "<<region << std::endl;
			std::cerr << "    Start_link "<<Start_link << std::endl;
			std::cerr << "    End_link "<<End_link << std::endl;
			
			cv::line( image_float, Start_link, End_link, cv::Scalar( 0, 0, 255 ), 3, 8);

			
		}






		for(int i = 0; i < Stable.Region_centroid.size();i++){
			cv::Point flip_centroid;
			flip_centroid.x = Stable.Region_centroid[i].x;
			flip_centroid.y = Stable.image_size.height - Stable.Region_centroid[i].y;
			
//			circle( image_float,flip_centroid,5,cv::Scalar(0, 255, 0),-1,8);
			circle( image_float,Stable.Region_centroid[i],6,cv::Scalar(0, 255, 0),-1,8);
			circle( image_float,Stable.Region_centroid[i],3,cv::Scalar(0, 0, 255),-1,8);
		 }
		
		
		//*/
		
		
//		cv::flip(image_float,image_float,0);
		
		return image_float;
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
