#include <my_robot_localization/kalman_filter.h>
#include <memory>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}