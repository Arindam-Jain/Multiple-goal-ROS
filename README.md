# Multi_goal_navigation
Multiple goals navigation which is set by YAML file and the goals are send to Move_base with actionclient and Hinge is controlled by conditional loops.

# How to Launch
Launch in same order after adding to workspace

1) To launch the world
roslaunch marker_description hinge_down.launch

2) To launch rviz+move_base
roslaunch marker_description move_base.launch

3) To launch multi-navigation-goals
multi_goal_driver multi_goal_driver.launch 

# Author
Arindam Jain https://github.com/Arindam-Jain 
