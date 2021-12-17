#include "ros/ros.h"
#include "assignment_srv/Velocity_assignment.h"

bool callback_srv(assignment_srv::Velocity_assignment::Request &req, assignment_srv::Velocity_assignment::Response &res)
{
	/* In this callback function the server_node control the multiplication factor of the 
	   robot velocity in order to increase or decrease it accordingly to the user input.*/
	  
	// Linear velocity modifying code
	if(req.c=='I'){ // user want to increase it
		res.x = 1;
		res.k = 'I';
	}
	else if(req.c=='D'){ // user want to decrease it
		res.x = -0.5;
		res.k = 'D';
	}
	
	// Angular velocity modifying code
	else if(req.c=='i'){ // user want to increase it
		res.z = 1;
		res.k = 'i';
	}
	else if(req.c=='d'){// user want to decrease it
		res.z = -0.5;
		res.k = 'd';
	}
	
	// No velocity is modified
	else{ // user doesn't want to modify it
		res.x = 0;
		res.z = 0;
	}
		
	return true;
}

int main(int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "server_node");
	ros::NodeHandle ns;
	
	//ros::Rate loop_rate(10);
	
	// Define the server for /velocity custom service
	ros::ServiceServer service = ns.advertiseService("/velocity", callback_srv);
	
	ros::spin();
	//loop_rate.sleep();
	
	return 0;
}
