#include <my_robot_controller/controller.hpp>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}