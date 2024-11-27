#pragma once

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <vector>

class MagniGoals
{
public:
  using clientType = std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>;

  MagniGoals();
  ~MagniGoals() = default;
  MagniGoals(const MagniGoals& other) = delete;
  MagniGoals& operator=(const MagniGoals& other) = delete;
  MagniGoals(MagniGoals&& other) = delete;
  MagniGoals& operator=(MagniGoals&& other) = delete;
  void run();

private:
  void setPosition(const double& x, const double& y);
  void setOrientation(const double& yaw);
  void sendAngularSpeed(const double& angular);
  void goToNextWaypoint(const std::array<double, 3>& waypoint);
  void turnAround(const double& desired_yaw);
  void doStep();

  const double PI = 3.1415926535898;
  double step_size_;
  double time_at_step_;
  double step_vel_;
  ros::Publisher vel_pub_;
  move_base_msgs::MoveBaseGoal goal_;
  clientType action_client_;
  std::vector<std::array<double, 3>> waypoints_;
};