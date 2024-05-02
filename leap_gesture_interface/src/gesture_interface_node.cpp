#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/vector3.hpp>

extern "C"
{
#include "LeapC.h"
#include "ExampleConnection.h"
}

class GestureInterfaceNode : public rclcpp::Node
{
public:
    // Enumeration for the gestures
    enum class Gesture
    {
        FIST,
        THUMB,
        POINT,
        PINKY,
        PISTOL,
        VICTORY,
        HORNS,
        SHAKA,
        THREE,
        ROCKER,
        FOUR,
        FIVE,
        UNKNOWN
    };

    GestureInterfaceNode() : Node("gesture_interface_node")
    {
        current_gestureL = Gesture::UNKNOWN;
        current_gestureR = Gesture::UNKNOWN;
        previous_gestureL = Gesture::UNKNOWN;
        previous_gestureR = Gesture::UNKNOWN;

        init_gesture = Gesture::FIST;
        gesture_counterL = 0;
        gesture_counterR = 0;
        init_gesture_counterR = 0;
        init_gesture_counterL = 0;
        filter_frames = 5;

        /***********LEAP MOTION STUFF***********/
        // Initialize LeapC connection
        OpenConnection();
        while (!IsConnected)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait a bit to let the connection complete
        }
        RCLCPP_INFO(this->get_logger(), "Connected.");

        LEAP_DEVICE_INFO *deviceProps = GetDeviceProperties();
        if (deviceProps)
        {
            RCLCPP_INFO(this->get_logger(), "Using device %s.", deviceProps->serial);
        }

        /***********ROS STUFF***********/
        // Declare the "publish_rate" parameter with a default value of 10.0 Hz
        double publish_rate = this->declare_parameter<double>("publish_rate", 10.0);

        // Initialize the publishers
        gesture_pub_ = this->create_publisher<std_msgs::msg::String>("leap_gesture", 10);
        gesture_pub_L_ = this->create_publisher<std_msgs::msg::String>("leap_gesture/left", 10);
        gesture_pub_R_ = this->create_publisher<std_msgs::msg::String>("leap_gesture/right", 10);
        gesture_filtered_pub_ = this->create_publisher<std_msgs::msg::String>("leap_gesture/filtered", 10);
        gesture_filtered_pub_L_ = this->create_publisher<std_msgs::msg::String>("leap_gesture/filtered_left", 10);
        gesture_filtered_pub_R_ = this->create_publisher<std_msgs::msg::String>("leap_gesture/filtered_right", 10);
        gesture_double_gesture_pub_ = this->create_publisher<std_msgs::msg::String>("leap_gesture/double", 10);
        gesture_double_filtered_gesture_pub_ = this->create_publisher<std_msgs::msg::String>("leap_gesture/double_filtered", 10);
        gesture_double_initiated_gesture_pub_ = this->create_publisher<std_msgs::msg::String>("leap_gesture/double_initiated", 10);
        gesture_initiated_gesture_pub_ = this->create_publisher<std_msgs::msg::String>("leap_gesture/gesture_initiated", 10);
        gesture_initiated_gesture_pub_L_ = this->create_publisher<std_msgs::msg::String>("leap_gesture/gesture_initiated_left", 10);
        gesture_initiated_gesture_pub_R_ = this->create_publisher<std_msgs::msg::String>("leap_gesture/gesture_initiated_right", 10);
        rpy_pub_L_= this->create_publisher<geometry_msgs::msg::Vector3>("leap_gesture/raw_rpy_left", 10);
        normalized_rpy_pub_L_ = this->create_publisher<geometry_msgs::msg::Vector3>("leap_gesture/normalized_rpy_left", 10);
        rpy_pub_R_ = this->create_publisher<geometry_msgs::msg::Vector3>("leap_gesture/raw_rpy_right", 10);
        normalized_rpy_pub_R_ = this->create_publisher<geometry_msgs::msg::Vector3>("leap_gesture/normalized_rpy_right", 10);

        // Timer callback to process gestures
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / 10)),
                                         std::bind(&GestureInterfaceNode::processGesture, this));
    }

private:
    //Function for basic gesture recognition
    Gesture recognizeGesture(const LEAP_HAND &hand)
    {
        int extended_fingers = 0;
        for (int f = 0; f < 5; f++)
        {
            if (hand.digits[f].is_extended)
            {
                extended_fingers++;
            }
        }

        switch (extended_fingers)
        {
        case 0:
            return Gesture::FIST;
        case 1:
            // Check if only the thumb is extended
            if (hand.digits[0].is_extended) // Assuming 0 is the index for thumb
            {
                return Gesture::THUMB;
            }
            // Check for if only the index is extended
            else if (hand.digits[1].is_extended) // Assuming 1 is the index for index
            {
                return Gesture::POINT;
            }
            else if (hand.digits[4].is_extended)
            {
                return Gesture::PINKY;
            }
            else
            {
                return Gesture::UNKNOWN;
            }
        case 2:
            if (hand.digits[0].is_extended && hand.digits[1].is_extended)
            {
                return Gesture::PISTOL;
            }
            else if (hand.digits[1].is_extended && hand.digits[2].is_extended)
            {
                return Gesture::VICTORY;
            }
            else if (hand.digits[1].is_extended && hand.digits[4].is_extended)
            {
                return Gesture::HORNS;
            }
            else if (hand.digits[0].is_extended && hand.digits[4].is_extended)
            {
                return Gesture::SHAKA;
            }
            else
            {
                return Gesture::UNKNOWN;
            }
        case 3:
            if (hand.digits[1].is_extended && hand.digits[2].is_extended && (hand.digits[0].is_extended || hand.digits[3].is_extended))
            {
                return Gesture::THREE;
            }
            else if (hand.digits[0].is_extended && hand.digits[1].is_extended && hand.digits[4].is_extended)
            {
                return Gesture::ROCKER;
            }
            else
            {
                return Gesture::UNKNOWN;
            }
        case 4:
            return Gesture::FOUR;
        case 5:
            return Gesture::FIVE;
        default:
            return Gesture::UNKNOWN;
        }
    }

    //function that processes gestures and publishes them to the appropriate topic
    void processGesture()
    {
        LEAP_TRACKING_EVENT *frame = GetFrame();
        if (!frame)
            return;

        for (uint32_t h = 0; h < frame->nHands; h++)
        {
            LEAP_HAND *hand = &frame->pHands[h];
            geometry_msgs::msg::Vector3 raw_rpy;
            geometry_msgs::msg::Vector3 normalized_rpy;
            auto message = std_msgs::msg::String();

            if (frame->nHands == 1)
            {
                current_gestureR = Gesture::UNKNOWN;
                current_gestureL = Gesture::UNKNOWN;
            }

            if (hand->type == eLeapHandType::eLeapHandType_Left) //left hand
            {
                 getHandRPY(hand, handL_roll, handL_pitch, handL_yaw);
                // Publish raw RPY
                raw_rpy.x = rad2Deg(handL_roll);
                raw_rpy.y = rad2Deg(handL_pitch);
                raw_rpy.z = rad2Deg(handL_yaw);
                rpy_pub_L_->publish(raw_rpy);

                // Normalize RPY and Publish
                normalized_rpy.x = normalizeAngle(rad2Deg(handL_roll));
                normalized_rpy.y = normalizeAngle(rad2Deg(handL_pitch));
                normalized_rpy.z = normalizeAngle(rad2Deg(handL_yaw));
                normalized_rpy_pub_L_->publish(normalized_rpy);

                //publish currently recognized gesture
                current_gestureL = recognizeGesture(frame->pHands[h]);
                message.data = gestureToString(current_gestureL);
                gesture_pub_L_->publish(message);
                //RCLCPP_INFO(this->get_logger(), "Counter %i.", gesture_counterL);


                //Condition that filters gestures based on the number of previous gesture frames
                if (gestureFiltered(filter_frames, current_gestureL, previous_gestureL, gesture_counterL))
                {
                    gesture_filtered_pub_L_->publish(message);
                    gesture_filtered_pub_->publish(message);
                }

                // Condition to check if a gesture was performed after an initialization gesture
                if (gestureInitialization(filter_frames, current_gestureL, previous_gestureL, init_gesture, init_gesture_counterL))
                {
                    gesture_initiated_gesture_pub_L_->publish(message);
                    gesture_initiated_gesture_pub_->publish(message);
                }
                previous_gestureL = current_gestureL;
            }
            else    //Right hand
            {
                getHandRPY(hand, handR_roll, handR_pitch, handR_yaw);
                // Publish raw RPY
                raw_rpy.x = handR_roll;
                raw_rpy.y = handR_pitch;
                raw_rpy.z = handR_yaw;
                rpy_pub_R_->publish(raw_rpy);

                // Normalize RPY and Publish
                normalized_rpy.x = normalizeAngle(rad2Deg(handR_roll));
                normalized_rpy.y = normalizeAngle(rad2Deg(handR_pitch));
                normalized_rpy.z = normalizeAngle(rad2Deg(handR_yaw));
                normalized_rpy_pub_R_->publish(normalized_rpy);

                current_gestureR = recognizeGesture(frame->pHands[h]);
                message.data = gestureToString(current_gestureR);
                gesture_pub_R_->publish(message);

                if (gestureFiltered(filter_frames, current_gestureR, previous_gestureR, gesture_counterR))
                {
                    gesture_filtered_pub_R_->publish(message);
                    gesture_filtered_pub_->publish(message);
                }

                if (gestureInitialization(filter_frames, current_gestureR, previous_gestureR, init_gesture, init_gesture_counterR))
                {
                    gesture_initiated_gesture_pub_R_->publish(message);
                    gesture_initiated_gesture_pub_->publish(message);
                }
                previous_gestureR = current_gestureR;
            }

            gesture_pub_->publish(message);

            if ((current_gestureL == current_gestureR) && current_gestureR != Gesture::UNKNOWN)
            {
                RCLCPP_INFO(this->get_logger(), "Counter %i.", gesture_counterL);
                gesture_double_gesture_pub_->publish(message);
                if (gesture_counterL == filter_frames || gesture_counterR == filter_frames)
                {
                    message.data = "DOUBLE" + gestureToString(current_gestureL);
                    gesture_double_filtered_gesture_pub_->publish(message);
                }

                if (init_gesture_counterL == filter_frames || init_gesture_counterR == filter_frames)
                {
                    message.data = "DOUBLE" + gestureToString(current_gestureL);
                    gesture_double_initiated_gesture_pub_->publish(message);
                }
            }
            // RCLCPP_INFO(this->get_logger(), "Current gesture %i.", current_gesture);
            // RCLCPP_INFO(this->get_logger(), "Previous gesture %i.", previous_gesture);
        }
    }

    double normalizeAngle(double angle_deg)
    {
        // Ensure the angle is within the desired range
        if (angle_deg > 90.0)
            angle_deg = 90.0;
        if (angle_deg < -90.0)
            angle_deg = -90.0;

        // Normalize to <-1,1>
        return angle_deg / 90.0;
    }

    //the function returns true only if a specified ammount of frames has passed and the gesture did not change
    bool gestureFiltered(const unsigned int &frames, const Gesture &current_gesture, const Gesture &previous_gesture, unsigned int &gesture_counter)
    {
        if (current_gesture == previous_gesture)
        {
            if (gesture_counter < frames)
                gesture_counter++;

            // RCLCPP_INFO(this->get_logger(), "Counter %i.", gesture_counter);
        }
        else if (current_gesture != previous_gesture)
        {
            gesture_counter = 0;
        }

        if (gesture_counter == frames)
        {
            return true;
        }
        return false;
    }

    //This function returns true only if a gesture was performed after an initialization gesture, which lasted for a specific number of frames
    bool gestureInitialization(const unsigned int &frames, const Gesture &current_gesture, const Gesture &previous_gesture, const Gesture &init_gesture, unsigned int &init_gesture_counter)
    {

        if (current_gesture == init_gesture)
        {
            if (init_gesture_counter < frames)
                init_gesture_counter++;

            // RCLCPP_INFO(this->get_logger(), "Initialization Counter %i.", init_gesture_counter);
        }
        else if (current_gesture != previous_gesture)
        {
            if (init_gesture_counter == frames)
            {
                init_gesture_counter = 0;
                return true;
            }
        }
        return false;
    }


    //convert gesture enum to stirng
    std::string gestureToString(const Gesture &gesture)
    {
        switch (gesture)
        {
        case Gesture::FIST:
            return "FIST";
        case Gesture::THUMB:
            return "THUMB";
        case Gesture::POINT:
            return "POINT";
        case Gesture::PINKY:
            return "PINKY";
        case Gesture::PISTOL:
            return "PISTOL";
        case Gesture::VICTORY:
            return "VICTORY";
        case Gesture::HORNS:
            return "HORNS";
        case Gesture::SHAKA:
            return "SHAKA";
        case Gesture::THREE:
            return "THREE";
        case Gesture::ROCKER:
            return "ROCKER";
        case Gesture::FOUR:
            return "FOUR";
        case Gesture::FIVE:
            return "FIVE";
        default:
            return "UNKNOWN";
        }
    }


    //Get Roll Pitch and Yaw of the palm from the Leap Motion SDK
    void getHandRPY(LEAP_HAND *hand, double &roll, double &pitch, double &yaw)
    {
        // Extract quaternion components from the hand orientation
        double w = hand->palm.orientation.w;
        double x = -(hand->palm.orientation.x);
        double y = hand->palm.orientation.z;
        double z = hand->palm.orientation.y;
        double ysqr = y * y;

        // Roll (x-axis rotation)
        double t0 = +2.0 * (w * x + y * z);
        double t1 = +1.0 - 2.0 * (x * x + ysqr);
        roll = std::atan2(t0, t1);

        // Pitch (y-axis rotation)
        double t2 = +2.0 * (w * y - z * x);
        t2 = ((t2 > 1.0) ? 1.0 : t2);
        t2 = ((t2 < -1.0) ? -1.0 : t2);
        pitch = std::asin(t2);

        // Yaw (z-axis rotation)
        double t3 = +2.0 * (w * z + x * y);
        double t4 = +1.0 - 2.0 * (ysqr + z * z);
        yaw = std::atan2(t3, t4);
    }

    //primitive func for radians conversion
    double rad2Deg(double &radians)
    {
        return radians * (180.0 / M_PI);
    }

    /********************** VARIABLES **********************/

    //Publisher variables
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_pub_; // publisher that publishes gestures as strings
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_pub_L_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_pub_R_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_filtered_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_filtered_pub_L_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_filtered_pub_R_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_initiated_gesture_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_initiated_gesture_pub_L_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_initiated_gesture_pub_R_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_double_gesture_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_double_filtered_gesture_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_double_initiated_gesture_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rpy_pub_L_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rpy_pub_R_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr normalized_rpy_pub_R_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr normalized_rpy_pub_L_;

    //rate timer
    rclcpp::TimerBase::SharedPtr timer_;

    //gesture variables
    Gesture current_gestureL;
    Gesture current_gestureR;
    Gesture previous_gestureL;
    Gesture previous_gestureR;
    Gesture init_gesture;

    //control variables
    double filter_frames;
    unsigned int gesture_counterL;
    unsigned int init_gesture_counterR;
    unsigned int gesture_counterR;
    unsigned int init_gesture_counterL;

    //RPY variables
    double handL_roll;
    double handL_pitch;
    double handL_yaw;
    double handR_roll;
    double handR_pitch;
    double handR_yaw;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GestureInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    CloseConnection();
    DestroyConnection();
    return 0;
}
