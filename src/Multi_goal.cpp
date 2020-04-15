// In this code following things takes place in sequence
// hinge up-> going to point A with trolley -> going to point B -> Then unlocking of trolley takes place
// plus goals can be easily added
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <string.h>
#include <map>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include <iostream>
using namespace std;

// defining global variable for hinge down
int flag_hinge=0;

struct Pose
{
  double x, y, theta;
  std::string frame;
  
};


// controlling hinge down
void hinge_down(ros::NodeHandle n_down)
{
  cout<<"2"<<endl;
  ros::Publisher hinge_publisher_down = n_down.advertise<std_msgs::Float64>("simple_model/base_to_second_joint_position_controller/command", 10);

  ros::Rate r(10);

  std_msgs::Float64 h;
  h.data = 0.0;
  int ctrl=0;
  cout<<"3"<<endl;
  while(ctrl==0){
    cout<<"4"<<endl;
    int connetctions=hinge_publisher_down.getNumSubscribers();
    if(connetctions>0){
      hinge_publisher_down.publish(h);
      ROS_INFO("%f", h.data);
      ctrl=1;
    }
    else{
      r.sleep();
    }
  }
}

class Multigoal
{
private:
  void callActionServer(move_base_msgs::MoveBaseGoal goal);
  void getGoals();
  void setGoals(Pose final_pose,double goal_num);
  void run(int status);
  
  int goal_count;
  bool goal_reached ,goal_sended, operation_started;

  
  double real_start_time, real_end_time;
  double new_pose_x , new_pose_y , old_pose_x , old_pose_y, dist;

  int i , j;
  ros::NodeHandle n_down;
  ros::NodeHandle nh_;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  ros::Subscriber odom_sub;
  move_base_msgs::MoveBaseGoal goal;
  move_base_msgs::MoveBaseGoal goal2;
  
  int check_status(int status);
  double get_start_time(double start_time);
  double get_end_time(double end_time);
  double calculate_distance(double curr_pos, double last_pos);
  int goal_status;
  // std::map<std::string, Pose> goal_map_;
  

public:
  void resultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::Ptr &msg);
  Multigoal(ros::NodeHandle nh);
  ~Multigoal();
};

Multigoal::Multigoal(ros::NodeHandle nh)
{
  i = 0;
  j = 0;
  getGoals();
  odom_sub = nh.subscribe("/odom",1,&Multigoal::odomCallback,this);
  sub = nh.subscribe("/move_base/status",1,&Multigoal::resultCallback,this);
  goal_count = 0;
  goal_status = 0;
  operation_started = false;
  old_pose_x = 0; old_pose_y = 0;
  dist = 0;
}

Multigoal::~Multigoal()
{
}

void Multigoal::run(int status)
{ 
  cout<<"goal_count="<<goal_count<<"  "<<"status="<<status<<endl; 
  if (status == 3 && goal_count == 0) { // this is the first stage 
    
    // still in the first stage but already in some position from previous action
    goal_reached = false;
    callActionServer(goal);
    goal_count = goal_count + 1;
    
  }
  if(goal_count == 1 && status == 1)
  {
    // already succesfully sending the goal.
    goal_count = goal_count + 1;
    
  }
  if (goal_count == 2 && status == 3) {
    // already send the goal and the first goal is reached
    callActionServer(goal2);
    goal_count = goal_count + 1;
    
  }
  if (goal_count == 3 && status == 1) {
    // already successfully sending the second goal.
    goal_count = goal_count + 1;
    
  }
  if (goal_count == 4 && status == 3) {
    ROS_INFO("all goal has reaced succesfully!");
    goal_count = goal_count + 1;
    operation_started = false;
    ROS_INFO("total distance traveled : %f",dist);
  }
  if (goal_count == 6 && status == 3) {
    ROS_INFO("hinge down!!");
    goal_count = goal_count+1;
    flag_hinge=1;
    cout<<"flag_hinge loop wala="<<flag_hinge<<endl;
    hinge_down(n_down);
  }

  else
  {
    //do nothing
  }
  
  
}

int Multigoal::check_status(int status)
{
  goal_status = status;
  // ROS_INFO("goal_status %i",goal_status);
  if (goal_status == 1) {
    // ROS_INFO("goal_sended ");
    goal_sended = true;
    goal_reached = false;
  }
  if (goal_status == 3)
  {
    // ROS_INFO("goal reached");
    goal_reached = true;
    goal_sended = false;
  }
  run(goal_status);
  return goal_status;
  // check the current status

}
double Multigoal::get_start_time(double start_time)
{
  
  if (start_time > 0 && i < 1 ) {
    real_start_time = start_time;
    operation_started = true;
    std::cout << "real start time is :" << real_start_time << std::endl;
    return real_start_time;
    
  }
  
}
double Multigoal::get_end_time(double end_time){

  if (end_time > 0 && j < 1 ) {
    real_end_time = end_time;
    std::cout << "real end time is :" << real_end_time << std::endl;
    return real_end_time;
    
  }

}

void Multigoal::odomCallback(const nav_msgs::Odometry::Ptr & msg)
{

  if (operation_started) {
  new_pose_x = msg->pose.pose.position.x;
  new_pose_y = msg->pose.pose.position.y;
  double diff_x , diff_y;

  diff_x = new_pose_x - old_pose_x;
  diff_y = new_pose_y - old_pose_y;
  dist = calculate_distance(diff_x,diff_y) + dist;
  old_pose_x = new_pose_x; old_pose_y = new_pose_y;
  
  }
  else
  {
    //
  }
  
}

double Multigoal::calculate_distance(double diff_x, double diff_y)
{
  // calculate the distance 
  double result = hypot (diff_x, diff_y);
  return result;
}

void Multigoal::resultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg){
// check if goal is reached 

int goal_stat;
if (msg->status_list.empty()) {
  goal_stat = 3;
}
else
{
  goal_stat = msg->status_list[0].status;
}

check_status(goal_stat);
double start_time , finish_time;

if (goal_count == 1)
    {
      // status is not clear, no goal is sended yet!
      start_time = msg->header.stamp.toSec(); // get the time from message
      get_start_time(start_time);
      i = i + 1;
      
      // getGoals();
     
    }
if ( goal_count == 5)
    {
      finish_time = msg->header.stamp.toSec(); // get the time from message
      get_end_time(finish_time);
      j = j + 1;
      double total_time = abs(real_start_time - real_end_time);
      ROS_INFO("total time = %f",total_time);
      goal_count = goal_count + 1;
    }
    
}


void Multigoal::getGoals()
{

std::string param_name;
  
  if (nh_.searchParam("/multi_goal/multi_goal_driver/goals", param_name))
  {
    XmlRpc::XmlRpcValue goals;
    if (!nh_.hasParam("/multi_goal/multi_goal_driver/goals"))
      {
        ROS_ERROR("No stations on parameterserver");
      }
  
    nh_.getParam("/multi_goal/multi_goal_driver/goals", goals);
    for (size_t i = 0 ; i < goals.size(); i++)
    {
      XmlRpc::XmlRpcValue goal = goals[i];
      Pose final_pose;
      XmlRpc::XmlRpcValue poses = goal["poses"];
      std::string frame = goal["frame_id"];
      XmlRpc::XmlRpcValue pose_back = poses[poses.size()-1];
      cout<<pose_back;
      final_pose.x = pose_back[0];
      final_pose.y = pose_back[1];
      final_pose.theta = pose_back[2];
      final_pose.frame = frame;
      setGoals(final_pose,i);
    }
    // after the goal has been obtained, now run the code to go to the destination
  
  }
  else
  {
    ROS_INFO("No param 'goals' found in an upward search");
  }

}

void Multigoal::setGoals(Pose final_pose,double goal_num)
{
  
  if (goal_num == 0) {
  geometry_msgs::PoseStamped target_pose;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = final_pose.frame;
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = final_pose.x;
  goal.target_pose.pose.orientation.w = final_pose.theta;
  std::cout << "first goal is :  " << final_pose.x << ", " << final_pose.y << ", " << final_pose.theta << ", " << final_pose.frame << std::endl;
  
  
  }
  if (goal_num == 1)
  {
  geometry_msgs::PoseStamped target_pose;

  //we'll send a goal to the robot to move 1 meter forward
  goal2.target_pose.header.frame_id = final_pose.frame;
  goal2.target_pose.header.stamp = ros::Time::now();
  
  goal2.target_pose.pose.position.x = final_pose.x;
  goal2.target_pose.pose.orientation.w = final_pose.theta;
  
  std::cout << " second goal is: " << final_pose.x << ", " << final_pose.y << ", " << final_pose.theta << ", " << final_pose.frame << std::endl;
    
    
  }
  
}

void Multigoal::callActionServer(move_base_msgs::MoveBaseGoal goal)
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
   MoveBaseClient ac("/move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
  ROS_INFO("Waiting for the move_base action server to come up");
    
  }
  //we'll send a goal to the robot the goal we get from previous function
  ac.sendGoal(goal);
  
  ROS_INFO("Sending goal");  
}


// controlling hinge up
void hinge_up(ros::NodeHandle n_up)
{
  cout<<"2"<<endl;
  ros::Publisher hinge_publisher_up = n_up.advertise<std_msgs::Float64>("simple_model/base_to_second_joint_position_controller/command", 10);

  ros::Rate r(10);

  std_msgs::Float64 h;
  h.data = 0.11;
  int ctrl=0;
  cout<<"3"<<endl;
  while(ctrl==0){
    cout<<"4"<<endl;
    int connetctions=hinge_publisher_up.getNumSubscribers();
    if(connetctions>0){
      hinge_publisher_up.publish(h);
      ROS_INFO("%f", h.data);
      ctrl=1;
    }
    else{
      r.sleep();
    }
  }
}



int main(int argc, char** argv){
    ros::init(argc, argv, "pub_hinge_up");
    ros::NodeHandle n_up;
    cout<<"1"<<endl;
    hinge_up(n_up);
    hinge_up(n_up);
    cout<<"5"<<endl;
    ros::NodeHandle n_down;
    cout<<"flag_hinge="<<flag_hinge<<endl;
    if(flag_hinge==1){
      cout<<"flag_hinge inside="<<flag_hinge<<endl;
      hinge_down(n_down);
      hinge_down(n_down);
    } 
    ros::init(argc,argv,"multi_goal_driver");
    ros::NodeHandle nh;

    Multigoal Multigoal(nh); 
    
    
    ros::Rate rate(5);
    ros::spin();

    return 0;

  //wait for the action server to come up
}



