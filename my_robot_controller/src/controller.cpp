#include <my_robot_controller/controller.hpp>


Controller::Controller() : Node("controller")
{
    m_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    m_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("set_joint_trajectory", 10);
    m_vel_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Controller::timerCmdVelCallback, this));
    m_trajectory_timer = this->create_wall_timer(std::chrono::seconds(2), std::bind(&Controller::timerTrajectoryCallback, this));
}

void Controller::timerCmdVelCallback()
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.1;
    msg.angular.z = 0.0;
    m_vel_publisher->publish(msg);
}

void Controller::timerTrajectoryCallback()
{
    auto msg = trajectory_msgs::msg::JointTrajectory();
    msg.header.frame_id = "base_footprint";
    msg.joint_names = {"arm_forearm_joint", "forearm_hand_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.1, 0.2};
    msg.points.push_back(point);
    m_trajectory_publisher->publish(msg);
}
