#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "another_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("another_node");

  // Next step goes here

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  move_group_interface.setEndEffectorLink("tool0");

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = -0.043154485523700714;
    msg.orientation.y = 0.9961686134338379;
    msg.orientation.z = 0.07547704875469208;
    msg.orientation.w = -0.009561617858707905;
    msg.position.x =  1.1692674160003662;
    msg.position.y = 0.24401788413524628;
    msg.position.z = 0.5697827935218811;
    return msg;
  }();

//    msg.orientation.x = -0.04;
//    msg.orientation.y = 1.0;
//    msg.orientation.z = 0.07;
//    msg.orientation.w = 0.009;
//    msg.position.x =  1.169;
//    msg.position.y = 0.24;
//    msg.position.z = 0.569;


//    msg.orientation.w = 1.0;
//    msg.position.x = 1.1691778898239136;
//    msg.position.y = 0.24397680163383484;
//    msg.position.z = 0.5697410702705383;

//    msg.position.x = 0.28;
//    msg.position.y = 0.2;
//    msg.position.z = 0.5;
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}