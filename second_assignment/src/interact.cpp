#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // header of the cmd_vel topic
#include "std_srvs/Empty.h" // header of the reset_position server
#include "assignment_srv/Velocity_assignment.h" // header of the velocity custom server

ros::ServiceClient client_vel;
ros::ServiceClient client_res_pos;

void ui_velocity(){
// Ask if the user desires to modify the linear or angular velocity of the robot and call /velocity custom service

	char c;
	assignment_srv::Velocity_assignment k;
	
	ROS_INFO("Do you want to modify linear velocity (L) or angular velocity (A)? %c\n");
	std::cin >> c;
	if(c=='L'){
		ROS_INFO("Do you want to increase (I) or decrease (D) the linear velocity? If neither of them, just press 'X': %c\n");
		std::cin >> k.request.c;
	}
	else if(c=='A'){
		ROS_INFO("Do you want to increase (i) or decrease (d) the angular velocity? If neither of them, just press 'x': %c\n");
		std::cin >> k.request.c;
	}
		
	client_vel.waitForExistence();
	client_vel.call(k);
}

void ui_position(){
// Ask if the user desires to reset the robot position and call /reset_positions service
	char p;
	
	ROS_INFO("Do you want to reset the robot position? If so, press 'Y' otherwise 'N': %c\n");
	std::cin >> p;
	if(p=='Y'){
		std_srvs::Empty srv_res_pos;
			
		client_res_pos.waitForExistence();
		client_res_pos.call(srv_res_pos);
	}
}

int main (int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "stageros_interact");
	ros::NodeHandle nh;
	
	// Define the clients for /velocity custom service and /reset_positions service
	client_vel = nh.serviceClient<assignment_srv::Velocity_assignment>("/velocity");
	client_res_pos = nh.serviceClient<std_srvs::Empty>("/reset_positions");
	
	// Constantly ask for an user input to call the associated services
	while (ros::ok){
		
		ui_velocity(); // call for an input regarding the velocity
		ui_position(); // call for an input regarding the position
		
		ros::spinOnce();
	}
	
	return 0;
}
