
#include <ros/ros.h>
#include <turtlesim/Pose.h> 
#include <iomanip> // for std::setprecision and std::fixed

// A callback function . Executed each time a new pose message arrives.
void poseMessageReceived(const turtlesim::Pose& msg) {
ROS_INFO_STREAM(std::setprecision(2)<<std::fixed
<< "position = ("<<msg.x<<", "<<msg.y<<")"
<< " direction="<<msg.theta);
}

int main ( int argc, char ** argv) {
	// Initialize the ROS system and become a node.
	ros::init (argc, argv, "subscribe_to_pose");
	ros::NodeHandle nh;
	
	// Create a publisher object.
	ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &poseMessageReceived);
	while (ros::ok()){
		ros::Rate rate(2);
		rate.sleep();		
		// Let ROS take over
		ros::spinOnce();
	}
}
