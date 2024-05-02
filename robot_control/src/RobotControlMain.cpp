#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "robot_control/RobotControlNode.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto robot_control_node = std::make_shared<RobotControlNode>();

    robot_control_node->initMoveGroup();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(robot_control_node);

    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    // Start mainLoop in a separate thread
    std::thread mainLoopThread([&]() {
        rclcpp::Rate rate(5); // Adjust the frequency as needed
        while (rclcpp::ok()) {
            robot_control_node->mainLoop();
            rate.sleep();
        }
    });

//    // Use rclcpp::spin to handle callbacks in the main thread
//    rclcpp::spin(robot_control_node);
//
//    // Wait for the mainLoop thread to finish before shutting down
//    if (mainLoopThread.joinable()) {
//        mainLoopThread.join();
//    }

    mainLoopThread.join();
    executor.cancel();
    if (executor_thread.joinable()) {
        executor_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}
