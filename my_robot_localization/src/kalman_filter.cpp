#include <my_robot_localization/kalman_filter.h>

KalmanFilter::KalmanFilter() : Node("kalman_filter"),
                               m_mean{0},
                               m_variance{2000.0},
                               m_imu_angular_z{0.0},
                               m_last_angular_z{0.0},
                               m_is_first_odom{true},
                               m_motion{0.0},
                               m_motion_variance{4.0},
                               m_meaurement_variance{0.5}
{
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1000,
                                                                    std::bind(&KalmanFilter::odomCallback, this, std::placeholders::_1));

    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 1000,
                                                                 std::bind(&KalmanFilter::imuCallback, this, std::placeholders::_1));

    m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("my_robot_controller/odom_kalman", 10);

    m_noisy_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("my_robot_controller/nosiy_odom", 10);
}

void KalmanFilter::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    m_kalman_odom = odom;
    m_noisy_odom = odom;
    const auto noisy_odom = odom.twist.twist.angular.z  + m_odom_noise_factor;
    if (m_is_first_odom)
    {
        m_mean =noisy_odom;
        m_last_angular_z =noisy_odom;
        m_is_first_odom = false;
        return;
    }

    m_motion =noisy_odom - m_last_angular_z;
    statePrediction();
    measurementUpdate();
    m_last_angular_z =noisy_odom;
    
    m_kalman_odom.twist.twist.angular.z = m_mean;
    m_odom_pub->publish(m_kalman_odom);
    
    m_noisy_odom.twist.twist.angular.z = noisy_odom;
    m_noisy_odom_pub->publish(m_noisy_odom);
}

void KalmanFilter::imuCallback(const sensor_msgs::msg::Imu &imu)
{
    m_imu_angular_z = imu.angular_velocity.z;
}

void KalmanFilter::measurementUpdate()
{
    m_mean = ((m_meaurement_variance * m_mean) + (m_variance * m_imu_angular_z)) / (m_meaurement_variance + m_variance);
    m_variance = (m_meaurement_variance * m_variance) / (m_meaurement_variance + m_variance);
}

void KalmanFilter::statePrediction()
{
    m_mean = m_mean + m_motion;
    m_variance = m_variance + m_motion_variance;
}
