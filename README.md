# Robot-Programming_Project

## Installation

Make sure you have followed the installation instructions in [http://wiki.ros.org/Robots/TIAGo/Tutorials](http://wiki.ros.org/Robots/TIAGo/Tutorials), either installing directly on Ubuntu 20.04 or through Docker. 

Follow These Steps:
1. Create a catkin workspace with an src folder within it.
2. Clone the repo into the src folder.
3. Come back to the workspace where you see the src folder ann source the /devel/setup.bash file inside tiago_public_ws whereever it is.
4. Build in Release Mode with: ```bash catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release```
5. ```bash catkin build```
6. Source the /devel/setup.bash file (for every terminal) taht now is inside the workspace.

## Usage

### On the move simulations

First run the Gazebo world:
```bash
roslaunch rp_tiago_gazebo tiago_gazebo.launch public_sim:=true end_effector:=pal-gripper world:=post_ball_polygonbox
```

Then run the node to execute simoultaneously the base and the arm controller
```bash
roslaunch tiago_grasping_on_the_move tiago_grasping_on_the_move.launch
```



PROVAPROVAPROVAPROVA