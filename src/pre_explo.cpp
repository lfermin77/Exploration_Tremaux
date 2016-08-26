//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/MarkerArray.h"

//openCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

//DuDe
#include "inc_decomp.hpp"




class ROS_handler
{
	ros::NodeHandle n;
	
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;	
	cv_bridge::CvImagePtr cv_ptr;
		
	std::string mapname_;
	ros::Subscriber map_sub_;	
	ros::Timer timer;
		
	ros::Subscriber odom_sub_;
	nav_msgs::Odometry Odom_Info_;
	
	float robot_position_[2];
	cv::Point robot_position_image_;
	std::vector <cv::Point> path_;
	cv::Point position_cm_;
	float distance;
	
	std::vector<std::vector<cv::Point> > contour_vector;
	
	float Decomp_threshold_;
	Incremental_Decomposer inc_decomp;
	Stable_graph Stable;

	std::vector <double> time_vector;
	
	public:
		ROS_handler(const std::string& mapname, float threshold) : mapname_(mapname),  it_(n), Decomp_threshold_(threshold)
		{


			ROS_INFO("Waiting for the map");
			map_sub_ = n.subscribe("map", 1, &ROS_handler::mapCallback, this);
			ros::Subscriber chatter_sub_ = n.subscribe("chatter", 1000, &ROS_handler::chatterCallback, this);
			odom_sub_ = n.subscribe("pose_corrected", 1, &ROS_handler::odomCallback, this);
			timer = n.createTimer(ros::Duration(0.5), &ROS_handler::metronomeCallback, this);
			image_pub_ = it_.advertise("/image_frontier", 1);
			
			cv_ptr.reset (new cv_bridge::CvImage);
			cv_ptr->encoding = "mono8";

			position_cm_ = cv::Point(0,0); 
			distance=0;
		}

		~ROS_handler()	{
		}


/////////////////////////////	
// ROS CALLBACKS			
////////////////////////////////		
		void chatterCallback(const std_msgs::String::ConstPtr& msg)
		{
		  ROS_INFO("I heard: [%s]", msg->data.c_str());  
		}
		
//////////////////////////////////		
		void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
		{
			double begin_process, end_process, begin_whole;
			begin_whole = begin_process = getTime();
			
			ROS_INFO("Received a %d X %d map @ %.3f m/pix", map->info.width, map->info.height, map->info.resolution);

		///////////////////////Occupancy to clean image	
			cv::Mat grad, img(map->info.height, map->info.width, CV_8U);
			img.data = (unsigned char *)(&(map->data[0]) );
			
			float pixel_Tau = Decomp_threshold_ / map->info.resolution;				
			cv_ptr->header = map->header;
			cv::Point2f origin = cv::Point2f(map->info.origin.position.x, map->info.origin.position.y);

			cv::Mat black_image, image_cleaned = clean_image(img, black_image);

			end_process = getTime();	cout << "Cleaning last "<< end_process - begin_process <<" ms" << endl;

		///////////////////////// Decompose Image
			begin_process = getTime();
			
			Stable = inc_decomp.decompose_image(image_cleaned, pixel_Tau, origin, map->info.resolution);
			
			end_process = getTime();	cout << "Decomposition last "<< (int) (end_process - begin_process) <<" ms" << endl;
			//time_vector.push_back(end_process - begin_process);

		////////////Draw Image & publish
			begin_process = getTime();

			grad = Stable.draw_stable_contour();

			cv_ptr->encoding = "32FC1";
			grad.convertTo(grad, CV_32F);
			grad.copyTo(cv_ptr->image);////most important
			
			end_process = getTime();	cout << "Drawing and publishing last "<< end_process - begin_process <<" ms" << endl;
			time_vector.push_back(end_process - begin_whole);

			cout << "Time Vector size "<< time_vector.size() << endl;
			for(int i=0; i < time_vector.size(); i++){
				cout << time_vector[i] << endl;
			}
		/////////////////////////	
		}
			

/////////////////////////
		void odomCallback(const nav_msgs::Odometry& msg){		
			robot_position_[0] =  msg.pose.pose.position.x;
			robot_position_[1] =  msg.pose.pose.position.y;
			
			cv::Point temp_Point(100*robot_position_[0], 100*robot_position_[1]);
			if(path_.size()>0) distance += cv::norm(temp_Point - position_cm_);

			path_.push_back(temp_Point);
			position_cm_ = temp_Point;
		}
		
/////////////////		
		void metronomeCallback(const ros::TimerEvent&)
		{
//		  ROS_INFO("tic tac");
		  publish_Image();
		}


////////////////////////
// PUBLISHING METHODS		
////////////////////////////		
		void publish_Image(){
			image_pub_.publish(cv_ptr->toImageMsg());
//			cv::imshow("OPENCV_WINDOW", cv_ptr->image);
//			cv::waitKey(3);
		}



/////////////////////////
//// UTILITY
/////////////////////////

		cv::Mat clean_image(cv::Mat Occ_Image, cv::Mat &black_image){
			//////////////////////////////	
			//Occupancy Image to Free Space	
			std::cout << "Cleaning Image..... "; 		double start_cleaning = getTime();
			cv::Mat open_space = Occ_Image<10;
			black_image = Occ_Image>90 & Occ_Image<=100;		
			cv::Mat Median_Image, Image_in, cut_image ;
			{
//				cout << "Entering........ ";
				cv::dilate(black_image, black_image, cv::Mat(), cv::Point(-1,-1), 4, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );			
//				cout << "dilated........ ";
//				cv::medianBlur(open_space, Median_Image, 15);
				
				//*
				cv::Mat temp;
				cv::boxFilter(open_space, temp, -1, cv::Size(10,10), cv::Point(-1,-1), false, cv::BORDER_DEFAULT );
				Median_Image = temp>50; 
				//cv::medianBlur(Median_Image, Median_Image, 1); //*/
				
//				cout << "Median Blur........ ";
				Image_in = Median_Image & ~black_image;
//				cout << "And........ ";
				Image_in.copyTo(cut_image);			
//				cout << "copy........ ";
			}
			double end_cleaning = getTime();  cout << "done, it last "<<(end_cleaning-start_cleaning)<< " ms"  << endl;	
			return cut_image;
		}


};










int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Inc_Dual_Decomposer");
	
	std::string mapname = "map";
	
	float decomp_th=3;
	if (argc ==2){ decomp_th = atof(argv[1]); }	
	
	ROS_handler mg(mapname, decomp_th);
	ros::spin();
	
	return 0;
}
