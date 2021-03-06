# KUKA iiwa model for grasping in Gazebo

## Instructions for installing
1) Official KUKA iiwa repo

https://github.com/IFL-CAMP/iiwa_stack

2) Repo for robotiq grippers and MimicPlugin

https://github.com/ros-industrial/robotiq
https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins

3) Realsense model and plugin

https://github.com/be2rlab/myicub_ros/tree/master/models/realsense_camera
https://github.com/pal-robotics/realsense_gazebo_plugin

4) Gazebo Grasp Plugin

https://github.com/JenniferBuehler/gazebo-pkgs

5) Segmentation

https://github.com/deyakovleva/yolact_ros

6) GPD repo 

https://github.com/deyakovleva/gpd_ros

7) Contact-GraspNet repo

https://github.com/deyakovleva/contact_graspnet/tree/dev_ros

## Usage

1) Run Gazebo and MoveIt
```
roslaunch iiwa_moveit moveit_planning_execution.launch
```
2) Run segmentation node
```
roslaunch launch/launch_them_all.launch
```
Run script for choosing object
```
python3 command_node.py
```
3.1) If you want to use Grasp Pose Detection run:
```
roslaunch gpd_ros ur5.launch
```
3.2) If you want to use Contact-GraspNet go https://github.com/deyakovleva/contact_graspnet/tree/dev_ros

There you'll find all the instructions.

4) Run manipulation node
```
roslaunch iiwa_manipulation ns_launch.launch
```
## Results

### Grasp Pose Detection
Sometimes GPD sets strange poses for grasping, so that only the best poses were chosen for video.

![grasping](src/iiwa_stack/results/box.gif)
![grasping](src/iiwa_stack/results/coke.gif)

### Contact-GraspNet
##### Scene
<p align="center">
  <img src="src/iiwa_stack/results/scene_observation.gif" width="640" title="Scene"/>
</p>

##### Grasp candidates
<p align="center">
  <img src="src/iiwa_stack/results/plot.gif" width="640" title="Plot"/>
</p>

##### Manipulation
<p align="center">
  <img src="src/iiwa_stack/results/cup_cgn.gif" width="640" title="cup_cgn"/>
</p>
<p align="center">
  <img src="src/iiwa_stack/results/cola_cgn.gif" width="640" title="cola_cgn"/>
</p>
<p align="center">
  <img src="src/iiwa_stack/results/box_cgn.gif" width="640" title="box_cgn"/>
</p>
<p align="center">
  <img src="src/iiwa_stack/results/ball_cgn.gif" width="640" title="ball_cgn"/>
</p>
<p align="center">
  <img src="src/iiwa_stack/results/dino_cgn.gif" width="640" title="dino_cgn"/>
</p>
