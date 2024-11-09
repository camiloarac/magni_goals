#include "magni_goals/magni_goals.h"

#include <algorithm>

MagniGoals::MagniGoals() {
    ros::NodeHandle nh("~");
    vel_pub_ =
        ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));
    action_client_.reset(
        new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
            "move_base", true));

    nh.param<double>("turning_time", turning_time_, 20.0);

    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.pose.position.z = 0.0;

    waypoints_ = {{6.0, -6.0, PI},
                  {-0.9, -6.0, PI / 2.0},
                  {-0.9, -2.0, 0.0},
                  {6.0, -2.0, -PI / 2.0}};
}

void MagniGoals::setPosition(const double& x, const double& y) {
    goal_.target_pose.pose.position.x = x;
    goal_.target_pose.pose.position.y = y;
}

void MagniGoals::setOrientation(const double& yaw) {
    tf2::Quaternion q;
    q.setRPY(0., 0., yaw);
    goal_.target_pose.pose.orientation.x = q.getX();
    goal_.target_pose.pose.orientation.y = q.getY();
    goal_.target_pose.pose.orientation.z = q.getZ();
    goal_.target_pose.pose.orientation.w = q.getW();
}

void MagniGoals::goToNextWaypoint(const std::array<double, 3>& waypoint) {
    setPosition(waypoint[0], waypoint[1]);
    setOrientation(waypoint[2]);
    // send a goal to the action
    action_client_->sendGoal(goal_);
    // wait for the action to return
    bool finished = action_client_->waitForResult(ros::Duration(30.0));
    if (finished) {
        actionlib::SimpleClientGoalState state = action_client_->getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not finish before the time out.");
        action_client_->cancelGoal();
    }
}

void MagniGoals::sendAngularSpeed(const double& angular) {
    geometry_msgs::Twist msg;
    msg.angular.z = angular;
    msg.linear.x = 0.0;

    vel_pub_.publish(msg);
}

void MagniGoals::turnAround(const double& desired_yaw) {
    ros::Time time_to_stop{ros::Time::now() + ros::Duration(turning_time_)};
    // TODO: rotation_angle should be the difference between the current yaw and
    // the desired_yaw
    double rotation_angle{2 * PI};
    double speed{rotation_angle / turning_time_};
    while (ros::Time::now() < time_to_stop) sendAngularSpeed(speed);
    sendAngularSpeed(0.0);
    return;
}

void MagniGoals::run() {
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    action_client_->waitForServer();  // will wait for infinite time

    ROS_INFO("Action server started, sending goals.");
    int seconds{10};
    int n_steps{5};
    for (const auto& elm : waypoints_) {
        goToNextWaypoint(elm);
        turnAround(elm[2]);
    }
}
