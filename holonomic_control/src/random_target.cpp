#include "ros/ros.h"
#include "holonomic_control/Random_target.h" 

/***************************************************************************************//*
* This function returns a random number for a giving range (M, N)
* This is the same function used in the lectures by professor (i.e. velocity_server.ccp 
* in my_srv package for the turtlebot_controller's exercises)
****************************************************************************************/
double randMToN(double M, double N){
	return M + (rand() / (RAND_MAX / (N-M)));
}


/**************************************************************************************//*
* target_pose is a service callback that replies to a giving request
* Here i get x,y as random values from a min and max number.
****************************************************************************************/
bool target_pose(holonomic_control::Random_target::Request &req, holonomic_control::Random_target::Response &res){
	res.x = randMToN(req.min, req.max);
	res.y = randMToN(req.min, req.max);
	return true;
}


int main(int argc, char **argv){
	//Service node initialization and advertise 
	ros::init(argc, argv, "random_server");
	ros::NodeHandle n;
	ros::ServiceServer service= n.advertiseService("/random_target", target_pose);
	ros::spin();

	return 0;
} 
