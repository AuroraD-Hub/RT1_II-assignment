#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // header of the cmd_vel topic
#include "sensor_msgs/LaserScan.h" // header of the base_scan topic
#include "assignment_srv/Velocity_assignment.h" // header of the velocity custom server

ros::Publisher pub_vel;
assignment_srv::Velocity_assignment k;
double min_r, min_c, min_l; // variables for minimum distances scanned from the edges
double min_th=1.5; // minimum distance threshold from the edges of the circuit

void get_k(){
/* In this function the control node uses the custom service /velocity to define the
   multiplication factor to modify the robot velocity */
   	
	// Linear velocity modifying code
	if(k.response.k=='I'){ // user want to increase it
		k.response.x = 1;
		k.response.k = 'I';
	}
	else if(k.response.k=='D'){ // user want to decrease it
		k.response.x = -0.5;
		k.response.k = 'D';
	}
	
	// Angular velocity modifying code
	else if(k.response.k=='i'){ // user want to increase it
		k.response.z = 1;
		k.response.k = 'i';
	}
	else if(k.response.k=='d'){// user want to decrease it
		k.response.z = -0.5;
		k.response.k = 'd';
	}
	
	// No velocity is modified
	else {// user doesn't want to modify it
		k.response.z = 0;
		k.response.x = 0;
	}
	    
	ROS_INFO("K factor changed by user input: [%f, %f]", k.response.x, k.response.z);
}

void choose_path()
{
/* In this function the control node find a clear path to follow, set the velocity 
   and then publish it into the /cmd_vel topic. */
   
	geometry_msgs::Twist vel;
	
	if(min_c>min_th){ // there is a clear path ahed of the robot
		vel.linear.x = (1+k.response.x)*5;
		vel.angular.z = 0;
	}
	else{
		if(min_r>min_l){ // there is a clear path to the right of the robot
			vel.linear.x = 1;
			vel.angular.z = -(1+k.response.z)*2;
		}
		else if(min_l>min_r){ // there is a clear path to the left of the robot
			vel.linear.x = 1;
			vel.angular.z = (1+k.response.z)*2;
		}
		else{
			vel.linear.x = 5;
			vel.angular.z = 0;
		}
	}
	
	pub_vel.publish(vel);	
}

void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
/* In this function the control_node acquires information from the laser scanner
   of the robot and uses them to move the robot in the circuit. The 'ranges' array, which
   is of size 721 and containes the information needed, is divided in three sections to 
   cover respectively [0 60]° (right section), [60 120]° (central section) and [120 180]° 
   (left section). */
   
	ROS_INFO("Robot laser scan subscriber", msg->ranges);
	
	/* Get information about the environment with /base_scan topic: find the minimum
   	   distances from the edges in the three sections. */

	min_r = msg->ranges[0];
	min_c = msg->ranges[240];
	min_l = msg->ranges[480];
	
	// Look for the minimum distances from the nearest obstacles
	for(int i=1; i<=240; i++){
		if(msg->ranges[i]<min_r)
			min_r = msg->ranges[i];
		if(msg->ranges[i+240]<min_c)
			min_c = msg->ranges[i+240];
		if(msg->ranges[i+480]<min_l)
			min_l = msg->ranges[i+480];
	}
	
	/* Control the velocity of the robot given the information previously acquired.*/
	
	get_k(); // define the k velocity factor
	choose_path(); // choose a clear path to follow
}


int main (int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "stageros_control");
	ros::NodeHandle nh;
	
	/* Define the publisher to /cmd_vel, the subscriber to /base_scan robot topics and the client of 
	   /velocity custom service*/
	pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Subscriber sub_scan = nh.subscribe("/base_scan", 1, callback_scan);
	ros::ServiceClient client_vel = nh.serviceClient<assignment_srv::Velocity_assignment>("/velocity");
	client_vel.call(k);
	
	ros::spin();
	
	return 0;
}
