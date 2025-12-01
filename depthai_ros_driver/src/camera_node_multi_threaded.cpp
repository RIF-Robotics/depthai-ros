#include "depthai_ros_driver/camera.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto cam = std::make_shared<depthai_ros_driver::Camera>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(cam);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
