#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "posli_ruku_niekde");
  ros::NodeHandle node_handle;

  // Start an asynchronous spinner to allow MoveIt to process callbacks
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Define the planning group
  static const std::string PLANNING_GROUP = "cr5_arm";

  // Create a MoveGroupInterface for the planning group
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // Set the planner and planning time
  move_group_interface.setPlannerId("RRTConnectkConfigDefault");
  move_group_interface.setPlanningTime(5.0);

  // Get the current robot state
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

  // Set the start state to the current state
  move_group_interface.setStartState(*current_state);

  // Define the target pose (move up by 10 cm)
  geometry_msgs::Pose target_pose = move_group_interface.getCurrentPose().pose;
  target_pose.position.z -= 0.05;  // move up by 10 cm
  //target_pose.position.y -= 0.1; 

  // Set the target pose
  move_group_interface.setPoseTarget(target_pose);

  // Plan the motion
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    // Visualize the planned trajectory in RViz
    move_group_interface.move();
  }
  else
  {
    ROS_WARN("Failed to plan the motion");
  }

  ros::shutdown();
  return 0;
}

