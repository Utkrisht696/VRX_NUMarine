// #include "rclcpp/rclcpp.hpp"
// #include "can_msgs/msg/frame.hpp"
// #include <chrono>
// using namespace std::chrono_literals;

// class CanIntPublisher : public rclcpp::Node {
// public:
//     CanIntPublisher() : Node("can_int_publisher") {
//         publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 10);
//         timer_ = this->create_wall_timer(
//             1000ms, std::bind(&CanIntPublisher::publish_int_message, this));
//     }

// private:
//     void publish_int_message() {
//         auto message = can_msgs::msg::Frame();
//         message.header.stamp = this->get_clock()->now();
//         message.id = 0x2b; // CAN ID for THRUST_BOW_STAR
//         message.is_rtr = false;
//         message.is_extended = false;
//         message.is_error = false;
//         message.dlc = 1; // Data length: 1 byte for an int value within 0-255

//         // Example: Encoding a specific integer value
//         int value = 69; // The integer value to send
//         if(value > 255) value = 255; // Ensuring the value fits into 1 byte
//         if(value < 0) value = 0; // Ensuring no negative values
//         message.data[0] = static_cast<uint8_t>(value);

//         publisher_->publish(message);
//         RCLCPP_INFO(this->get_logger(), "Publishing int value over CAN: %d", value);
//     }

//     rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<CanIntPublisher>());
//     rclcpp::shutdown();
//     return 0;
// }


///**************************************THIS CODE WORKS FOR BOAT************************ 

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "can_msgs/msg/frame.hpp"
// #include <algorithm> // For std::clamp

// class BoatMotorController : public rclcpp::Node {
// public:
//     BoatMotorController() : Node("boat_motor_controller") {
//         // Subscribe to cmd_vel
//         subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "cmd_vel", 10, std::bind(&BoatMotorController::twistCallback, this, std::placeholders::_1));
        
//         // Publisher for CAN messages
//         publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 10);
//     }

// private:
//     void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//         // Mapping linear and angular velocities to motor speeds
//         auto forward_speed = mapVelocityToMotorSpeed(msg->linear.x);
//         auto turn_speed = mapVelocityToMotorSpeed(msg->angular.z);

//         // Assuming the same forward speed for both front motors
//         // Adjust if your system is different
//         publishMotorSpeed(0x28, turn_speed); // Front left motor
//         publishMotorSpeed(0x29, turn_speed); // Front right motor
        
//         // For turning, might need to adjust based on your boat's control logic
//         publishMotorSpeed(0x2A, forward_speed); // Back left motor
//         publishMotorSpeed(0x2B, forward_speed); // Back right motor
//     }

//     uint8_t mapVelocityToMotorSpeed(double velocity) {
//         // Assuming velocity is normalized between -1 and 1
//         // Adjust the scaling factor as necessary for your setup
//         int speed = static_cast<int>(128.0 + (velocity * 126.0));
        
//         // Clamping to ensure speed is within the valid range
//         speed = std::clamp(speed, 0, 254);
        
//         return static_cast<uint8_t>(speed);
//     }

//     void publishMotorSpeed(int id, uint8_t speed) {
//         can_msgs::msg::Frame frame;
//         frame.header.stamp = this->get_clock()->now();
//         frame.id = id;
//         frame.is_rtr = false;
//         frame.is_extended = false;
//         frame.is_error = false;
//         frame.dlc = 1;
//         frame.data[0] = speed;

//         publisher_->publish(frame);
//         RCLCPP_INFO(this->get_logger(), "Publishing speed %u on CAN ID: 0x%X", speed, id);
//     }

//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
//     rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<BoatMotorController>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "can_msgs/msg/frame.hpp"
// #include <algorithm> // For std::clamp

// class BoatMotorController : public rclcpp::Node {
// public:
//     BoatMotorController() : Node("boat_motor_controller") {
//         // Subscribe to cmd_vel
//         subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "cmd_vel", 10, std::bind(&BoatMotorController::twistCallback, this, std::placeholders::_1));
        
//         // Publisher for CAN messages
//         publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 10);
//     }

// private:
//    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//         // Mapping velocities to motor speeds
//         auto forward_speed = mapVelocityToMotorSpeed(msg->linear.x);
//         auto turn_speed_left = msg->angular.z < 0 ? mapVelocityToMotorSpeed(std::abs(msg->angular.z)) : 0;
//         auto turn_speed_right = msg->angular.z > 0 ? mapVelocityToMotorSpeed(msg->angular.z) : 0;

//         // Publish motor speeds based on the direction
//         publishMotorSpeed(0x28, forward_speed); // Front left motor for moving forward
//         publishMotorSpeed(0x29, forward_speed); // Front right motor for moving forward
//         publishMotorSpeed(0x2A, turn_speed_left); // Back motor for turning left
//         publishMotorSpeed(0x2B, turn_speed_right); // Back motor for turning right
//     }

//     uint8_t mapVelocityToMotorSpeed(double velocity) {
//         // Clamp the input velocity between 0.0 (stop) and 1.0 (max forward)
//         velocity = std::clamp(velocity, 0.0, 1.0);
        
//         // Map velocity to [0, 254] range for motor speed
//         int speed = static_cast<int>(velocity * 254);

//         return static_cast<uint8_t>(speed);
//     }

//     void publishMotorSpeed(int id, uint8_t speed) {
//         can_msgs::msg::Frame frame;
//         frame.header.stamp = this->get_clock()->now();
//         frame.id = id;
//         frame.is_rtr = false;
//         frame.is_extended = false;
//         frame.is_error = false;
//         frame.dlc = 1;
//         frame.data[0] = speed;

//         publisher_->publish(frame);
//         RCLCPP_INFO(this->get_logger(), "Publishing speed %u on CAN ID: 0x%X", speed, id);
//     }

//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
//     rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<BoatMotorController>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "can_msgs/msg/frame.hpp"
// #include <algorithm> // For std::clamp

// class BoatMotorController : public rclcpp::Node {
// public:
//     BoatMotorController() : Node("boat_motor_controller") {
//         // Subscribe to cmd_vel
//         subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "cmd_vel", 10, std::bind(&BoatMotorController::twistCallback, this, std::placeholders::_1));
        
//         // Publisher for CAN messages
//         publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 10);
//     }

// private:
//     void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//         // Mapping linear and angular velocities to motor speeds
//         auto forward_speed = mapVelocityToMotorSpeed(msg->linear.x);
//         auto turn_speed = mapVelocityToMotorSpeed(msg->angular.z);

//         // Mapping speeds to motor commands
//         publishMotorSpeed(0x28, forward_speed); // Front left motor for turning
//         publishMotorSpeed(0x29, forward_speed); // Front right motor for turning
//         publishMotorSpeed(43, turn_speed); // Back right motor for forward/backward
//         publishMotorSpeed(42, -turn_speed); // Back right motor for forward/backward
//     }

//     uint8_t mapVelocityToMotorSpeed(double velocity) {
//         // Normalize the velocity to the range of 0 to 254
//         // Map -20 to 0 and 20 to 254, linearly scaling between them
//         double scaled_velocity = (velocity + 100) * (254.0 / 200.0);  // Scale and shift velocity range [-20, 20] to [0, 254]
//         int speed = static_cast<int>(scaled_velocity);

//         // Ensure speed is within the valid range
//         speed = std::clamp(speed, 0, 254);
        
//         return static_cast<uint8_t>(speed);
//     }

//     void publishMotorSpeed(int id, uint8_t speed) {
//         can_msgs::msg::Frame frame;
//         frame.header.stamp = this->get_clock()->now();
//         frame.id = id;
//         frame.is_rtr = false;
//         frame.is_extended = false;
//         frame.is_error = false;
//         frame.dlc = 1;
//         frame.data[0] = speed;

//         publisher_->publish(frame);
//         RCLCPP_INFO(this->get_logger(), "Publishing speed %u on CAN ID: 0x%X", speed, id);
//     }

//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
//     rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<BoatMotorController>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "can_msgs/msg/frame.hpp"
// #include <vector>
// #include <algorithm> // For std::clamp

// class BoatMotorController : public rclcpp::Node {
// public:
//     BoatMotorController() : Node("boat_motor_controller"), enabled(false) {
//         // Subscribe to CAN messages to check for enabling condition
//         subscription_can = this->create_subscription<can_msgs::msg::Frame>(
//             "can_tx", 10, std::bind(&BoatMotorController::canMessageCallback, this, std::placeholders::_1));

//         // Subscribe to cmd_vel
//         subscription_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
//             "cmd_vel", 10, std::bind(&BoatMotorController::twistCallback, this, std::placeholders::_1));
        
//         // Publisher for CAN messages
//         publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 10);
//     }

// private:
//     void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg) {
//         if (msg->id == 0x0) {  // Check for ID 0x0
//             // Check if the first data byte is greater than 0
//             if (msg->data[0] >= 1 || msg->data[1] >= 1 || msg->data[2] >=1  || msg->data[3] >= 1 || msg->data[4] >= 1  ) {
//                 RCLCPP_INFO(this->get_logger(), "youre doomed");
//                 enabled = true;
//                 RCLCPP_INFO(this->get_logger(), "Control enabled based on CAN message.");
//             } else {
//                 enabled = false;
//                 RCLCPP_INFO(this->get_logger(), "Control disabled based on CAN message.");
//             }
//         }
//     }

//     void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//         if (enabled) {
//             // Mapping linear and angular velocities to motor speeds
//             auto forward_speed = mapVelocityToMotorSpeed(msg->linear.x);
//             auto turn_speed = mapVelocityToMotorSpeed(msg->angular.z);

//             // Mapping speeds to motor commands
//             publishMotorSpeed(0x2B, turn_speed); // Front RIGHT motor for turning                  BOW STAR
//             publishMotorSpeed(0x2A, -turn_speed); // Front LEFT motor for turning                BOW PORT
//             publishMotorSpeed(0x28, forward_speed); // Back RIGHT motor for forward/backward       STERN STAR
//             publishMotorSpeed(0x29, forward_speed); // Back LEFT motor for forward/backward      STERN PORT
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Received cmd_vel but control is disabled.");
//         }
//     }

//     uint8_t mapVelocityToMotorSpeed(double velocity) {
//         // Normalize the velocity to the range of 0 to 254
//         // Map -20 to 0 and 20 to 254, linearly scaling between them
//         double scaled_velocity = (velocity + 100) * (254.0 / 200.0);  // Scale and shift velocity range [-20, 20] to [0, 254]
//         int speed = static_cast<int>(scaled_velocity);

//         // Ensure speed is within the valid range
//         speed = std::clamp(speed, 0, 254);
        
//         return static_cast<uint8_t>(speed);
//     }

//     void publishMotorSpeed(int id, uint8_t speed) {
//         can_msgs::msg::Frame frame;
//         frame.header.stamp = this->get_clock()->now();
//         frame.id = id;
//         frame.is_rtr = false;
//         frame.is_extended = false;
//         frame.is_error = false;
//         frame.dlc = 1;
//         frame.data[0] = speed;
//         publisher_->publish(frame);
//         RCLCPP_INFO(this->get_logger(), "Publishing speed %u on CAN ID: 0x%X", speed, id);
//     }

//     rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_can;
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel;
//     rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
//     bool enabled;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<BoatMotorController>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "can_msgs/msg/frame.hpp"
#include <vector>
#include <algorithm> // For std::clamp

class BoatMotorController : public rclcpp::Node {
public:
    BoatMotorController() : Node("boat_motor_controller"), enabled(false), Kv(0.03), intercept(-44.94) {
        // Subscribe to CAN messages to check for enabling condition
        subscription_can = this->create_subscription<can_msgs::msg::Frame>(
            "can_tx", 10, std::bind(&BoatMotorController::canMessageCallback, this, std::placeholders::_1));

        // Subscribe to cmd_vel
        subscription_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&BoatMotorController::twistCallback, this, std::placeholders::_1));
        
        // Publisher for CAN messages
        publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 10);
    }

private:
    void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg) {
        if (msg->id == 0x0) {  // Check for ID 0x0
            // Ensure data has at least 5 bytes
            if (msg->data.size() >= 5) {
                // Check if any of the first five data bytes are greater than 0
                if (std::any_of(msg->data.begin(), msg->data.begin() + 5, [](uint8_t b) { return b >= 1; })) {
                    //RCLCPP_INFO(this->get_logger(), "Enabling control based on CAN message.");
                    enabled = true;
                } else {
                    //RCLCPP_INFO(this->get_logger(), "Disabling control based on CAN message.");
                    enabled = false;
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Received CAN message with insufficient data length.");
            }
        }
    }

    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (enabled) {
            // Convert velocities to PWM values
            double pwm_stern_left, pwm_stern_right, pwm_bow_left, pwm_bow_right;
            std::tie(pwm_stern_left, pwm_stern_right, pwm_bow_left, pwm_bow_right) = convertVelocityToPWM(msg->linear.x, msg->angular.z);

            // Remap PWM values from 1100-1900 to 0-254
            pwm_stern_left = remapPWM(pwm_stern_left);
            pwm_stern_right = remapPWM(pwm_stern_right);
            pwm_bow_left = remapPWM(pwm_bow_left);
            pwm_bow_right = remapPWM(pwm_bow_right);

            // Publish the motor commands
            publishMotorSpeed(0x28, pwm_stern_left);  // Back LEFT motor
            publishMotorSpeed(0x29, pwm_stern_right); // Back RIGHT motor
            publishMotorSpeed(0x2A, pwm_bow_left);    // Front LEFT motor
            publishMotorSpeed(0x2B, pwm_bow_right);   // Front RIGHT motor
        } else {
            RCLCPP_INFO(this->get_logger(), "Received cmd_vel but control is disabled.");
        }
    }

    std::tuple<double, double, double, double> convertVelocityToPWM(double linear_vel, double angular_vel) {

        double scaling_lin= 16;
        double scaling_ang= 5;

        // Convert linear velocity to thrust
        double thrust_stern = linear_vel*scaling_lin;
        // Convert angular velocity to thrust for turning
        double thrust_bow = angular_vel*scaling_ang;
        
        // Calculate PWM for stern thrusters
        double pwm_stern_left = (thrust_stern - intercept) / Kv;
        double pwm_stern_right = (thrust_stern - intercept) / Kv;
        
        // Calculate PWM for bow thrusters
        double pwm_bow_left = (-thrust_bow - intercept) / Kv;
        double pwm_bow_right = (thrust_bow - intercept) / Kv;

        RCLCPP_INFO(this->get_logger(), "Publishing speed %f on Stern_Star", pwm_stern_right);
        RCLCPP_INFO(this->get_logger(), "Publishing speed %f on Stern_Port", pwm_stern_left);
        RCLCPP_INFO(this->get_logger(), "Publishing speed %f on Bow_Star", pwm_bow_right);
        RCLCPP_INFO(this->get_logger(), "Publishing speed %f on Bow_Port", pwm_bow_left);
        
        return std::make_tuple(pwm_stern_left, pwm_stern_right, pwm_bow_left, pwm_bow_right);
    }

    uint8_t remapPWM(double pwm) {
        // Clamp the PWM value to 1100-1900 range
        double clamped_pwm = std::clamp(pwm, 1100.0, 1900.0);
        // Remap the clamped PWM value to 0-254 range
        double scaled_pwm = (clamped_pwm - 1100) * (254.0 / 800.0);
        return static_cast<uint8_t>(scaled_pwm);
    }

    void publishMotorSpeed(int id, uint8_t speed) {
        can_msgs::msg::Frame frame;
        frame.header.stamp = this->get_clock()->now();
        frame.id = id;
        frame.is_rtr = false;
        frame.is_extended = false;
        frame.is_error = false;
        frame.dlc = 1;
        frame.data[0] = speed;
        publisher_->publish(frame);
        RCLCPP_INFO(this->get_logger(), "Publishing speed %u on CAN ID: 0x%X", speed, id);

    }

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_can;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
    bool enabled;
    const double Kv;
    const double intercept;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BoatMotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
