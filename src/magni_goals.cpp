#include "magni_goals/magni_goals.h"

#include <algorithm>

#include "magni_goals/read_csv.h"

MagniGoals::MagniGoals() {
    ros::NodeHandle nh("~");
    vel_pub_ =
        ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));
    action_client_.reset(
        new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
            "move_base", true));

    double step_size_deg{};
    double step_vel_deg{};
    nh.param<double>("step_size", step_size_deg, 90.0);
    nh.param<double>("time_at_step", time_at_step_, 1.0);
    nh.param<double>("step_vel", step_vel_deg, 90.0);

    step_size_ = step_size_deg * PI / 180.0;
    step_vel_ = step_vel_deg * PI / 180.0;

    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.pose.position.z = 0.0;

    std::ifstream file("/workspace/waypoints.csv");

    CsvRow row;
    std::array<double, 3> waypoint;
    while (row.readNextRow(file)) {
        auto toDouble = [](const std::string_view& input) -> double {
            return std::stod(static_cast<std::string>(input));
        };
        waypoints_.push_back(
            {toDouble(row[0]), toDouble(row[1]), toDouble(row[2])});
    }
}

void MagniGoals::setPosition(const double& x, const double& y) {
    goal_.target_pose.pose.position.x = x;
    goal_.target_pose.pose.position.y = y;
}

void MagniGoals::setOrientation(const double& yaw) {
    tf2::Quaternion q;
    q.setRPY(0., 0., yaw * PI / 180.0);
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

void MagniGoals::doStep() {
    double time_for_step = step_size_ / step_vel_;
    ROS_INFO("Time for the step: %f", time_for_step);
    ros::Time time_to_stop{ros::Time::now() + ros::Duration(time_for_step)};
    while (ros::Time::now() < time_to_stop) {
        sendAngularSpeed(step_vel_);
        ros::Duration(0.05).sleep();
    }
    sendAngularSpeed(0.0);
    return;
}

void MagniGoals::turnAround(const double& desired_yaw) {
    for (double i = 0.; i < 1.99 * PI; i += step_size_) {
        doStep();
        ros::Duration(time_at_step_).sleep();
    }
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
        if (!ros::ok()) break;
    }
}
