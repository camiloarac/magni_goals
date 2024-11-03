#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>

#include "magni_goals/magni_goals.h"

#include <vector>
#include <array>

constexpr double PI = 3.1415926535898;

void setPosition(const double& x, const double& y, move_base_msgs::MoveBaseGoal& goal)
{
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
}

void setOrientation(const double& yaw, move_base_msgs::MoveBaseGoal& goal)
{
    tf2::Quaternion q;
    q.setRPY(0., 0., yaw);
    goal.target_pose.pose.orientation.x = q.getX();
    goal.target_pose.pose.orientation.y = q.getY();
    goal.target_pose.pose.orientation.z = q.getZ();
    goal.target_pose.pose.orientation.w = q.getW();
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "magni_goals");

    std::vector<std::array<double, 3>> waypoints {
        { 6.0,  0.0, -PI/2.0},
        { 6.0, -6.0,      PI},
        {-0.9, -6.0,  PI/2.0},
        {-0.9, -2.0,     0.0},
        { 6.0, -2.0,  PI/2.0}
    };

    // create the action client
    // true causes the client to spin its own thread
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> cli;
    cli.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ("move_base", true));

    // Initialize goal object
    move_base_msgs::MoveBaseGoal goal; 
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.z = 0.0;
    
    
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    cli->waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goals.");

    bool finished{};
    for (const auto& elm : waypoints) {
        setPosition(elm[0], elm[1], goal);
        setOrientation(elm[2], goal);
        // send a goal to the action
        cli->sendGoal(goal);
        //wait for the action to return
        finished = cli->waitForResult(ros::Duration(30.0));
        if (finished)
        {
            actionlib::SimpleClientGoalState state = cli->getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
        {
            ROS_INFO("Action did not finish before the time out.");
            cli->cancelGoal();
        }
    }
    
    //exit
    return 0;
}