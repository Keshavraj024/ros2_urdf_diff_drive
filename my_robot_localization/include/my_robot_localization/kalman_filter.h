#ifndef _KALMAN_FILTER__HPP_
#define _KALMAN_FILTER__HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

class KalmanFilter : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the KalmanFilter class.
     */
    KalmanFilter();

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_noisy_odom_pub;

    double m_mean;
    double m_variance;
    double m_imu_angular_z;
    double m_last_angular_z;
    bool m_is_first_odom;
    double m_motion;
    nav_msgs::msg::Odometry m_kalman_odom;
    nav_msgs::msg::Odometry m_noisy_odom;
    double m_motion_variance;
    double m_meaurement_variance;
    double m_odom_noise_factor{0.1};

    /**
     * @brief Callback function for Odometry messages.
     *
     * @param msg The received Odometry message.
     */
    void odomCallback(const nav_msgs::msg::Odometry &);
    /**
     * @brief Callback function for IMU messages.
     *
     * @param msg The received IMU message.
     */
    void imuCallback(const sensor_msgs::msg::Imu &);
    /**
     * @brief Perform the measurement update step of the Kalman filter.
     */
    void measurementUpdate();
    /**
     * @brief Perform the state prediction step of the Kalman filter.
     */
    void statePrediction();
};

#endif