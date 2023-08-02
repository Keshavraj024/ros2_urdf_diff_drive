#ifndef _CONTROLLER_HPP_
#define _CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class Controller : public rclcpp::Node
{
    public:
        Controller();
    private:
        /**
         * @brief publish to /cmd_vel topic at certain frequency
        */
        void timerCmdVelCallback();

        /**
         * @brief publish to /set_joint_trajectory topic at certain frequency
        */
        void timerTrajectoryCallback();

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_vel_publisher;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_trajectory_publisher;
        rclcpp::TimerBase::SharedPtr m_vel_timer;
        rclcpp::TimerBase::SharedPtr m_trajectory_timer;




};

#endif