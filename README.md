# ROS simulation in c++
This second assignment consists in a robot simulation based on ROS environment and in c++ language. The arena and its CMakeLists.txt has been forked by [Prof. Carmine Recchiuto's repository](https://github.com/CarmineD8/second_assignment) for Robotics Engineering students of UNIGE.

For this assignment a Docker Image (`carms84/noetic_ros2`) has been used.

## Installing and running
At first, to install ROS add this line to the .bashrc file `source /opt/ros/noetic/setup.bash` if the Docker Image is used, otherwise follow the instruction on [wiki.ros.org](https://wiki.ros.org).

Then, a ROS workspace need to be built and so digit in the terminal:
```
mkdir -p <workspace_name>/src
cd <worksapce_name>
catkin_make
```

At last, it is necessary to add the line `source [workspace_path]/devel/setup.bash` in your .bashrc file, which can be opened with `gedit .bashrc` command in your root folder, and build again the workspace with `catkin_make`.

Once the workspace is built, it is possible both to fork this package or to download it with `git clone` command inside the newly created workspace.

NOTE: `catkin_make` should always be done in the root folder of the workspace.

Now, to run the simulation digit on the terminal:
```
roscore &
rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world
```
You should see the following arena:![stage-000000](https://user-images.githubusercontent.com/72380912/146173933-1902c107-ac68-41ad-8a43-cf7b48c94b53.png)
To run the assignment open two other terminals and digit in them respectively:
```
rosrun second_assignment control_node
rosrun second_assignment interact_node
```
## Assignment
For this assignment the robot (blue dot in the previous image) has to move autonomously inside the arena. In fact, it is endowed with lasers scanners that gives information about the environment and the edges of the circuit. Moreover, an external user should be able to increase/decrease the velocity and to reset the robot position.

These behaviours can be achied by initializing two nodes: `control_node` and `interact_node`. The first one subscribes to `/base_scan` topic, which gives information about the obtacles scanned by the lasers, and then publish to `/cmd_vel` topic, which modify the linear and angular velocity of the robot in order to let it move autonomously inside the circuit. The latter, awaits for input from the user and accordingly modifies the velocity or reset the position by calling the service `/reset_positions`.

The following flowchart describes the scripts of one possible solution implemented:

## 'stageros' robot
### Laser scanner ###
By subscribing to the `/base_scan` topic, the node has access to many fields among which `rages` is used to get the information needed: it is an array of 721 elements which contains the distances from the nearest obstacles in a [0 180]° vision range.

This topic type is `sensor_msgs::LaserScan`.

### Velocity ###
By publishing into the `/cmd_vel` topic, the node can modify its fileds:
* *linear*: linear velocity array
  * *x*,*y*: direction of the linear velocity
* *angular*: angular velocity array
  * *z*: direction of the angular velocity 

This topic type is `geometry_msgs::Twist`.

### Position ###
By calling `/reset_positions` service, the node initializes a client with empty request and expect an empty response whenever the user decides to reset the robot position. 

This service type is `std_srvs::Empty`.
