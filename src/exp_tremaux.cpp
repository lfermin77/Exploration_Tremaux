#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "nav_msgs/Odometry.h"


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;


  ros::Publisher pose_pub = node.advertise<nav_msgs::Odometry>( "pose_corrected", 10 );

  tf::TransformListener listener;

//	std::cout<<"Entre "<< std::endl;
	
	

  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_footprint",  ros::Time(0), transform);
 //     	std::cout<<"Entre adentro "<< std::endl;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    nav_msgs::Odometry msg;
    
    msg.header.frame_id = "map";
    msg.header.stamp = transform.stamp_;
	msg.pose.pose.position.x = transform.getOrigin().x();
	msg.pose.pose.position.y = transform.getOrigin().y();
	msg.pose.pose.position.z = transform.getOrigin().z();
	
	tf::Quaternion q = transform.getRotation();
	q= q.normalized();
	
	tf::quaternionTFToMsg (q, msg.pose.pose.orientation);

		
	pose_pub.publish(msg);


  }
  return 0;
};
