# Navigation #


## Install ##

ROS navigation packages


    sudo apt-get install ros-kinetic-navigation



ROS package: `gradient_based_navigation`


    git clone https://github.com/Imperoli/gradient_based_navigation


add to catkin workspace and compile.


## Run ##

### 1. Start a robot ###

See [robot](https://bitbucket.org/iocchi/marrtino_apps/src/master/robot/) or 
[stage](https://bitbucket.org/iocchi/marrtino_apps/src/master/stage/) sections.

### 2. Start navigation ###

To launch navigation modules, use the script 

        python runnav.py <MAP_NAME> <X> <Y> <TH_DEG>


(X,Y,TH) is the initial pose of the robot in map coordinates.


Example:

        python runnav.py map 0 0 0


#### Alternative manual launch

To start localizer and navigation modules manually

        roslaunch srrg_localizer.launch mapsdir:=<MAPS_DIR> map_name:=<MAP> initial_pose_x:=<X> initial_pose_y:=<Y> initial_pose_a:=<TH_RAD>
        roslaunch move_base.launch


### 3. Send target goals


To send target goals to the robot, use the script

        python move.py <GX> <GY> <GTH_DEG>

(GX,GY,GTH): target pose in map coordinates


Example:

        python move.py 6 -1 270

### 4. Navigation path

Write a navigation file containing a sequence of navigation commands

        moveTo(<GX>,<GY>,<GTH_DEG>)
        forward(<M>)
        turn(<DEG>,"ABS"|"REL")


Run the navigation file with

        python nav.py <navfile>

Example:

        python nav.py test.nav


### 5. Recovery

Launch the navigation recovery procedure

        python recovery.py

Note: to define specific recovery procedures for your environment, copy `recovery.py`
and add other recovery behaviors.


## Other functionalities

### Obstacle avoidance behavior only ###

To launch only the obstacle avoidance behavior:

    roslaunch obstacle_avoidance.launch


You need a controller (e.g., joystick teleop) to drive the robot.

Note: Controllers must publish on /desired_cmd_vel instead of /cmd_vel


### Visualization ###
    
    rosrun rviz rviz -d nav.rviz


### Quit ###

    rosnode kill -a



