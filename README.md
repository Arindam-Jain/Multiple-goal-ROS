# Multi_goal_navigation
Multiple goals navigation which is set by YAML file and the goals are send to Move_base with actionclient and Hinge is controlled by conditional loops.

# How to Launch
Launch in same order after adding to workspace

1) To launch the world
roslaunch marker_description marker_world.launch

2) To launch rviz+move_base
roslaunch marker_description move_base.launch

3) To launch multi-navigation-goals
multi_goal_driver multi_goal_driver.launch 

![image](https://user-images.githubusercontent.com/40122399/140578992-ab2e655a-6969-47e9-b8dc-7361f1939163.png)

https://drive.google.com/file/d/1-Ltms2kTFQGwbizYWhXSirhCImQOBE3U/view?usp=sharing


# Video
[![IMAGE ALT TEXT HERE](![image](https://user-images.githubusercontent.com/40122399/140579490-9a4eb331-f017-48b1-b298-dc594f32431c.png)](https://drive.google.com/file/d/1-Ltms2kTFQGwbizYWhXSirhCImQOBE3U/view?usp=sharing)

This video is the prototype simulation of trolley carrying from Dispatch point to storage point

# Author
Arindam Jain 
