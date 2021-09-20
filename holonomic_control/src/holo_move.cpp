#include "ros/ros.h"
#include "holonomic_control/Random_target.h" //Service for target position
#include "geometry_msgs/Twist.h" //Message cointaining velocity
#include "nav_msgs/Odometry.h" //Message cointaining estimated position of robot
#include <iostream>
#include <cmath>


using namespace std;

float x_trgt=0.0, y_trgt=0.0; //variables for target position
float d_th=0.1; //Treshold distance
float dist_x = 0.0; //distance on x
float dist_y = 0.0; //distance on y
float k; //proportional costant to obtain a certain speed

/*Declaring globally the publisher and the client*/
ros::Publisher pub;
ros::ServiceClient client;

/**************************************************************************************//*
* Callback publishing velocity after reading the position. 
* This function computes the velocity published on
* the topic "/cmd_vel" based on the estimated
* position retrieved from the messages in the 
* "/odom" odometry topic. When the target position
* is reached a call to the Service "/random_target"
* is made in order to retrieve a new target
* position.
*
* \param h_state (/nav_msgs::Odometry::ConstPtr&):
* 	pointer to a message read from topic
* 	"/odom", used to obtain estimated
* 	current position of the holonomic robot 
*******************************************************************************************/

void positionCallback(const nav_msgs::Odometry::ConstPtr& h_state)
{
  
  geometry_msgs::Twist vel;
  holonomic_control::Random_target target;

//Check if the target is reached or not through the d_th
//If the distance is < d_th, the target is reached and i call for a new one
//If not, i set the velocity as vel = k*(position of target-position of robot)
  if(dist_x < d_th && dist_y < d_th){

  	target.request.min = -6.0;
  	target.request.max = 6.0;
  	client.call(target);
  	x_trgt = target.response.x;
  	y_trgt = target.response.y;
	cout << "Chief, i have received the target position: "<<x_trgt<<" "<<y_trgt<<"\n";
	cout << "Call for a new target\n";
	dist_x = x_trgt - h_state->pose.pose.position.x;
	dist_y = y_trgt - h_state->pose.pose.position.y;
	}


  else if(dist_x >= d_th || dist_y >= d_th){

  	vel.linear.x = k*(x_trgt - h_state->pose.pose.position.x);
	vel.linear.y = k*(y_trgt - h_state->pose.pose.position.y);
	dist_x = x_trgt - h_state->pose.pose.position.x;
	dist_y = y_trgt - h_state->pose.pose.position.y;
	cout << "I can't reach the goal, MOVE!\n";
	}

  ROS_INFO("Robot position: [%f, %f]", h_state->pose.pose.position.x, h_state->pose.pose.position.y);
  ROS_INFO("Robot velocity: [%f, %f]",dist_x, dist_y);
  pub.publish(vel);

  
}


int main(int argc, char **argv)
{  
   //First insert the proportional costant for a certain velocity
   cout << "Insert a k value to determine the velocity :\n";
   cin >> k;
   //Initialization of node
   ros::init(argc, argv, "holo_move");
   ros::NodeHandle n;

   //client service, publisher advertise and subscriber
   client=n.serviceClient<holonomic_control::Random_target>("/random_target");

   pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

   ros::Subscriber sub = n.subscribe("/odom", 1000, positionCallback);
   ros::spin();

   return 0;
}
