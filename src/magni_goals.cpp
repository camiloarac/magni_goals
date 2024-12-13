#include "magni_goals/magni_goals.h"

#include <geometry_msgs/Transform.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Transform.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <thread>

#include "magni_goals/read_csv.h"

MagniGoals::MagniGoals()
    : tf_buffer_(ros::Duration(3.0)), tf_listener_(tf_buffer_) {
    ros::NodeHandle nh("~");
    vel_pub_ =
        ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));
    digital_output_pub_ =
        ros::Publisher(nh.advertise<std_msgs::Bool>("/digital_output", 10));
    action_client_.reset(
        new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
            "move_base", true));

    double step_size_deg{};
    double max_angular_vel_deg{};
    nh.param<double>("step_size", step_size_deg, 90.0);
    nh.param<double>("time_at_rotation_step", time_at_rotation_step_, 1.0);
    nh.param<double>("max_angular_vel", max_angular_vel_deg, 90.0);

    step_size_ = step_size_deg * PI / 180.0;
    max_angular_vel_ = max_angular_vel_deg * PI / 180.0;

    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.pose.position.z = 0.0;

    running_ = true;
    std::ifstream file("/workspace/src/magni_goals/waypoints.csv");

    CsvRow row;
    std::array<double, 3> waypoint;
    while (row.readNextRow(file)) {
        auto toDouble = [](const std::string_view& input) -> double {
            return std::stod(static_cast<std::string>(input));
        };
        auto toInt = [](const std::string_view& input) -> int {
            int val = std::stoi(static_cast<std::string>(input));
            if (val <= 0) return 1;
            return val;
        };
        auto toBool = [](std::string_view input) -> bool {
            input.remove_prefix(std::min(input.find_first_not_of(" "), input.size()));
            if (input == "on")
                return true;
            else
                return false;
        };
        waypoints_.push_back(
            {toDouble(row[0]), toDouble(row[1]), toDouble(row[2])});
        n_steps_.push_back(toInt(row[3]));
        time_at_linear_step_.push_back(toDouble(row[4]));
        output_values_.push_back(toBool(row[5]));
    }
    for (const bool& output : output_values_) {
        ROS_INFO("Output: %d", output);
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
        // ROS_INFO("reference: %f, current: %f, error: %f, vel: %f",
        //          yaw * 180.0 / PI, current_yaw * 180.0 / PI, error * 180.0 /
        //          PI, vel);
        sendAngularSpeed(vel);
    }
    sendAngularSpeed(0.0);
    return;
}

tf2::Transform MagniGoals::getTransform() {
    tf2::Transform tf{};
    std::string to_frame{"map"};
    std::string from_frame{"base_footprint"};
    double roll{};
    double pitch{};
    double yaw{};
    geometry_msgs::TransformStamped tfs =
        tf_buffer_.lookupTransform(to_frame, from_frame, ros::Time(0));
    geometry_msgs::Transform geom_tf{tfs.transform};
    tf2::fromMsg(geom_tf, tf);
    return tf;
}

double MagniGoals::getCurrentYaw() {
    tf2::Transform tf{getTransform()};
    double roll{};
    double pitch{};
    double yaw{};
    tf.getBasis().getRPY(roll, pitch, yaw);
    return yaw;
}

auto MagniGoals::getCurrentXY() -> std::pair<double, double> {
    tf2::Transform tf{getTransform()};
    double x{tf.getOrigin()[0]};
    double y{tf.getOrigin()[1]};
    return std::make_pair(x, y);
}

void MagniGoals::turnAround(const double& desired_yaw) {
    for (double i = step_size_; i < 1.999 * PI; i += step_size_) {
        // ROS_INFO("Final goal: %f, current goal: %f", desired_yaw,
        //          desired_yaw + i);
        doRotationStep(desired_yaw + i);
        ros::Duration(time_at_rotation_step_).sleep();
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
    std::thread thr([this]() {
        ros::Rate r(10);
        std_msgs::Bool msg;
        while (ros::ok() && running_) {
            msg.data = output_;
            digital_output_pub_.publish(msg);
            r.sleep();
        }
    });
    ROS_INFO("Action server started, sending goals.");
    std::array<double, 3> waypoint{};
    double delta_x{};
    double delta_y{};
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        output_ = output_values_[i];
        waypoint = waypoints_[i];
        auto [x_ini, y_ini] = getCurrentXY();
        ROS_INFO("\nWaypoint: %f, %f, %f", waypoint[0], waypoint[1],
                 waypoint[2]);
        ROS_INFO("Current position: %f, %f\n", x_ini, y_ini);
        delta_x = (waypoint[0] - x_ini) / static_cast<double>(n_steps_[i]);
        delta_y = (waypoint[1] - y_ini) / static_cast<double>(n_steps_[i]);
        double x{x_ini + delta_x};
        double y{y_ini + delta_y};
        double yaw_ini{atan2(delta_y, delta_x) * 180.0 / PI};
        double yaw{waypoint[2]};
        for (size_t j = 0; j < n_steps_[i]; ++j) {
            waypoint[0] = x;
            waypoint[1] = y;
            if (j + 1 == n_steps_[i])
                waypoint[2] = yaw;
            else
                waypoint[2] = yaw_ini;
            ROS_INFO("Waypoint in the loop: %f, %f, %f", waypoint[0],
                     waypoint[1], waypoint[2]);
            goToNextWaypoint(waypoint);
            ros::Duration(time_at_linear_step_[i]).sleep();
            x += delta_x;
            y += delta_y;
        }
        turnAround(waypoint[2] * PI / 180.0);
        if (!ros::ok()) break;
    }
    output_ = false;
    // Wait until the digital output is turned off
    running_ = false;
    ros::Duration(1.0).sleep();
    thr.join();
}
