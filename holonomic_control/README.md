# Research Track 1 - First Assignment


This package was built in order to control a holonomic robot in 2d space.


## Robot Behaviour


1. The robot asks for a random target, with both coordinates in the interval (-6.0, 6.0).
2. The robot reaches the target.
3. Go to step **1**

The target is reached if the distance between the target position and the one retrieved by odometry is below the distance treshold *d_th=0.1*, otherwise the robot's velocity is set by the equation *vel=k(target position - robot position)*.
*k* is a proportional costant given by the user as input to analyze different behaviours.
It can see that:
- if **k>>1** the robot is fast and always able to reach the target
- if **k<<1** the robot is slow and has difficult in reaching the goal


## Running the code


First of all do the *$catkin_make* command in the workspace for building the package.

Then for launching all nodes toghether

```bash

$roslaunch holonomic_control holo_control.launch

```

Otherwise, you can run individually 

```bash

$rosrun holonomic_control random_server
$rosrun holonomic_control holo_move

```

## Nodes 


**random_server**

Service node serving *Random_target*, for the service *"random_target"*. When a request with the two floats min e max is issued, it fills the *response* field of the Service with a random position, a composed of two random *x* and *y* coordinates

**holo_move**

This node publishes velocity data on the topic *"/cmd_vel"*. It does so by reading the estimated position of the robot (messages present in *"/odom"*) and, if the current target hasn't been reached, it set the velocities according to the formula described in section *Robot Behaviour*. If, otherwise, the target position has been reached the node will call the Service *Random_target* which will yield in its response field the coordinates of a new point to reach on the map.


## Custom Services


**Random_target**

Service used to retrieve the random position of the target.
The *request* part is composed by *float32 min* and *float32 max*.
The *response* part is composed by *float32 x* and *float32 min*.


## Computational graph of the system

![rqt_graph](/holonomic_control/rosgraph.png)


- **holo_move** is subscribed to *"/odom"* and publishes in *"/cmd_vel"*, while calling Services *"/random_server"*.
- **random_server** serves for Service *"/random_target"*
