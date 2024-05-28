#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

extern "C"{
#include "LeapC.h"
#include "ExampleConnection.h"
}

class GestureInterfaceNode : public rclcpp::Node
{
public:
    GestureInterfaceNode() : Node("gesture_interface_node"), tf_broadcaster_(this)
    {
        // Initialize LeapC connection
        OpenConnection();
        while (!IsConnected)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(100)); // Wait for the connection to complete
        }

        RCLCPP_INFO(this->get_logger(), "Connected.");
        LEAP_DEVICE_INFO* deviceProps = GetDeviceProperties();
        if (deviceProps)
        {
            RCLCPP_INFO(this->get_logger(), "Using device %s.", deviceProps->serial);
        }

        palm_position_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("leap_gesture/palm_position", 10);
    }

    void run()
{
    rclcpp::Rate rate(10); // For example, 10Hz rate
    int64_t lastFrameID = 0;

    // Default transform setup
    double default_x = 0.68;  // Default x position of leap_hands in meters
    double default_y = 0.0;  // Default y position
    double default_z = 0.0;  // Default z position

    int degrees = -30;
    // Define the rotation angle in radians
    double angle_rad = degrees * M_PI / 180; // Convert 45 degrees to radians

    // Rotation matrix for a 45-degree rotation about the x-axis
    double Rx[3][3] = {
        {1, 0, 0},
        {0, cos(angle_rad), -sin(angle_rad)},
        {0, sin(angle_rad), cos(angle_rad)}
    };

    while (rclcpp::ok())
    {
        LEAP_TRACKING_EVENT *frame = GetFrame();
        if (frame && (frame->tracking_frame_id > lastFrameID))
        {
            lastFrameID = frame->tracking_frame_id;
            RCLCPP_INFO(this->get_logger(), "Frame %lli with %i hands.", (long long int)frame->tracking_frame_id, frame->nHands);

            for (uint32_t h = 0; h < frame->nHands; h++)
            {
                LEAP_HAND* hand = &frame->pHands[h];
                RCLCPP_INFO(this->get_logger(), "Hand id %i is a %s hand with position (%f, %f, %f).",
                            hand->id,
                            (hand->type == eLeapHandType_Left ? "left" : "right"),
                            hand->palm.position.x,
                            hand->palm.position.y,
                            hand->palm.position.z);

                // Original palm position coordinates adjusted from sensor data
                double original_x = default_x + (hand->palm.position.x / 1000);
                double original_y = default_y - (hand->palm.position.z / 1000);
                double original_z = default_z + (hand->palm.position.y / 1000);

                // Apply the rotation matrix to the original coordinates
                geometry_msgs::msg::Vector3 palm_position;
                palm_position.x = original_x; // x-coordinate remains unchanged
                palm_position.y = original_y * Rx[1][1] + original_z * Rx[1][2];
                palm_position.z = original_y * Rx[2][1] + original_z * Rx[2][2];

                if(hand->type == eLeapHandType_Right || hand->type == eLeapHandType_Left)
                {
                    palm_position_pub_->publish(palm_position);
                }

                // Rotate orientation by 45 degrees around the x-axis
                tf2::Quaternion original_orientation(hand->palm.orientation.x, -(hand->palm.orientation.z), hand->palm.orientation.y, hand->palm.orientation.w);
                tf2::Quaternion rotation_quaternion;
                rotation_quaternion.setRPY(degrees * M_PI / 180, 0, 0);  // Rotation of 45 degrees about the x-axis

                // Combine original orientation with the rotation
                tf2::Quaternion new_orientation = rotation_quaternion * original_orientation;
                new_orientation.normalize();

                // Broadcast transform for the leap_hands frame with an offset
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = this->get_clock()->now();
                transformStamped.header.frame_id = "world";  // or your robot's base frame
                transformStamped.child_frame_id = "leap_hands";
                transformStamped.transform.translation.x = palm_position.x;
                transformStamped.transform.translation.y = palm_position.y;
                transformStamped.transform.translation.z = palm_position.z;
                transformStamped.transform.rotation.x = new_orientation.x();
                transformStamped.transform.rotation.y = new_orientation.y();
                transformStamped.transform.rotation.z = new_orientation.z();
                transformStamped.transform.rotation.w = new_orientation.w();
                tf_broadcaster_.sendTransform(transformStamped);
            }
        }
        rate.sleep();
    }
}

private:
    // ... (any private member variables or methods you may need)
    //Publisher variables
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr palm_position_pub_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;  // TF broadcaster
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GestureInterfaceNode>();
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
