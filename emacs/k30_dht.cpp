
#include <ros/ros.h>
#include <turtlesim/Pose.h> 
#include <turtlesim/k30.h> 
#include <iomanip> // for std::setprecision and std::fixed

//~ // A callback function . Executed each time a new pose message arrives.
//~ void poseMessageReceived(const turtlesim::Pose& msg) {
	//~ ROS_INFO_STREAM(std::setprecision(2)<<std::fixed
	//~ << "position = ("<<msg.x<<", "<<msg.y<<")"
	//~ << " direction="<<msg.theta);
//~ }

// A callback function . Executed each time a new k30 message arrives.
void envMessageReceived(const turtlesim::Pose& msg) {
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed
	<< "CO2 % ppm = "<< msg.k30 
	<< " temp C = "<<msg.tempc
	<< " humid % = "<<msg.humid);
}

int main ( int argc, char ** argv) {
	// Initialize the ROS system and become a node.
	ros::init (argc, argv, "subscribe_to_pose");
	ros::NodeHandle nh;
	
	// Create a publisher object.
	ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &envMessageReceived);
	while (ros::ok()){
		ros::Rate rate();
		rate.sleep();		
		// Let ROS take over
		ros::spinOnce();
	}
}
