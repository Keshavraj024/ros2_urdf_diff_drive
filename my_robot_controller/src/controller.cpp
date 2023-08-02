#include <my_robot_controller/controller.hpp>

Controller::Controller() : Node("controller")
{
    this->declare_parameter("linear_vel_x", 0.0);
    this->declare_parameter("angular_yaw", 0.0);
    this->declare_parameter("forearm_position", 0.0);
    this->declare_parameter("hand_position", 0.0);

    m_linear_vel_x = this->get_parameter("linear_vel_x").as_double();
    m_angular_yaw = this->get_parameter("angular_yaw").as_double();
    m_forearm_position = this->get_parameter("forearm_position").as_double();
    m_hand_position = this->get_parameter("hand_position").as_double();

    m_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    m_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("set_joint_trajectory", 10);
    m_vel_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Controller::timerCmdVelCallback, this));
    m_trajectory_timer = this->create_wall_timer(std::chrono::seconds(2), std::bind(&Controller::timerTrajectoryCallback, this));
}

void Controller::timerCmdVelCallback()
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = m_linear_vel_x;
    msg.angular.z = m_angular_yaw;
    m_vel_publisher->publish(msg);
}

void Controller::timerTrajectoryCallback()
{
    auto msg = trajectory_msgs::msg::JointTrajectory();
    msg.header.frame_id = "base_footprint";
    msg.joint_names = {"arm_forearm_joint", "forearm_hand_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {m_forearm_position, m_hand_position};
    msg.points.push_back(point);
    m_trajectory_publisher->publish(msg);
}
