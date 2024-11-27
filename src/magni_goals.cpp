#include <geometry_msgs/Transform.h>

#include <algorithm>
#include <cmath>

#include "magni_goals/magni_goals.h"
#include "magni_goals/read_csv.h"

MagniGoals::MagniGoals()
    : tf_buffer_(ros::Duration(3.0)), tf_listener_(tf_buffer_) {
    ros::NodeHandle nh("~");
    vel_pub_ =
        ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));
    action_client_.reset(
        new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
            "move_base", true));

    double step_size_deg{};
    double max_angular_vel_deg{};
    nh.param<double>("step_size", step_size_deg, 90.0);
    nh.param<double>("time_at_step", time_at_step_, 1.0);
    nh.param<double>("max_angular_vel", max_angular_vel_deg, 90.0);

    step_size_ = step_size_deg * PI / 180.0;
    max_angular_vel_ = max_angular_vel_deg * PI / 180.0;

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

void MagniGoals::doRotationStep(double yaw) {
    ros::Rate r(50);

    double current_yaw{getCurrentYaw()};
    double vel{};
    ROS_INFO("Yaw before normalization: %f", yaw);
    normalizeAngle(yaw);
    ROS_INFO("Yaw after normalization: %f", yaw);
    double error{yaw - current_yaw};
    ROS_INFO("Error before loop: %f", error);
    while ((std::abs(error) > kAngularErrorThreshold) && ros::ok()) {
        r.sleep();
        current_yaw = getCurrentYaw();
        error = yaw - current_yaw;
        normalizeAngle(error);
        double sign = std::signbit(error) ? -1.0 : 1.0;
        vel = std::min(max_angular_vel_, std::abs(error)) * sign;
        ROS_INFO("reference: %f, current: %f, error: %f, vel: %f",
                 yaw * 180.0 / PI, current_yaw * 180.0 / PI, error * 180.0 / PI,
                 vel);
        sendAngularSpeed(vel);
    }
    sendAngularSpeed(0.0);
    return;
}

double MagniGoals::getCurrentYaw() {
    tf2::Transform tf{};
    std::string to_frame{"map"};
    std::string from_frame{"base_footprint"};
    double roll{};
    double pitch{};
    double yaw{};
    geometry_msgs::TransformStamped tfs =
        tf_buffer_.lookupTransform(to_frame, from_frame, ros::Time(0));
    // tf2::Quaternion q{tfs.transform.rotation.x, tfs.transform.rotation.y,
    //                   tfs.transform.rotation.z, tfs.transform.rotation.w};
    geometry_msgs::Transform geom_tf{tfs.transform};
    tf2::fromMsg(geom_tf, tf);
    tf.getBasis().getRPY(roll, pitch, yaw);
    return yaw;
}

void MagniGoals::turnAround(const double& desired_yaw) {
    for (double i = step_size_; i < 1.999 * PI; i += step_size_) {
        ROS_INFO("Final goal: %f, current goal: %f", desired_yaw,
                 desired_yaw + i);
        doRotationStep(desired_yaw + i);
        ros::Duration(time_at_step_).sleep();
    }
    return;
}

void MagniGoals::normalizeAngle(double& yaw) {
    if (yaw < -PI) {
        yaw += 2 * PI;
    }
    if (yaw > PI) {
        yaw -= 2 * PI;
    }
}

void MagniGoals::run() {
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    action_client_->waitForServer();  // will wait for infinite time

    ROS_INFO("Action server started, sending goals.");
    int seconds{10};
    int n_steps{5};
    for (const auto& elm : waypoints_) {
        ROS_INFO("Waypoint: %f, %f, %f", elm[0], elm[1], elm[2]);
        goToNextWaypoint(elm);
        turnAround(elm[2] * PI / 180.0);
        if (!ros::ok()) break;
    }
}
