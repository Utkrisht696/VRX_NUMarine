#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "can_msgs/msg/frame.hpp"

class BoatMotorController : public rclcpp::Node {
public:
    BoatMotorController() : Node("boat_motor_controller") {
        // Subscribe to CAN messages
        subscription_can_thrusters = this->create_subscription<can_msgs::msg::Frame>(
            "can_tx", 10, std::bind(&BoatMotorController::canThrusterCallback, this, std::placeholders::_1));
        
        // Publishers for thruster topics
        publisher_stern_left = this->create_publisher<std_msgs::msg::Int8>("stern_left_thrust", 10);
        publisher_stern_right = this->create_publisher<std_msgs::msg::Int8>("stern_right_thrust", 10);
        publisher_bow_left = this->create_publisher<std_msgs::msg::Int8>("bow_left_thrust", 10);
        publisher_bow_right = this->create_publisher<std_msgs::msg::Int8>("bow_right_thrust", 10);
    }

private:
    void canThrusterCallback(const can_msgs::msg::Frame::SharedPtr msg) {
        if (msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received CAN message with empty data.");
            return;
        }

        std_msgs::msg::Int8 thrust_msg;
        thrust_msg.data = convertRange(msg->data[0]);
        
        switch (msg->id) {
            case 0x28:
                publisher_stern_left->publish(thrust_msg);
                break;
            case 0x29:
                publisher_stern_right->publish(thrust_msg);
                break;
            case 0x2A:
                publisher_bow_left->publish(thrust_msg);
                break;
            case 0x2B:
                publisher_bow_right->publish(thrust_msg);
                break;
            default:
                // Ignore messages with IDs that are not for thrusters
                break;
        }
    }

    int8_t convertRange(uint8_t value) {
        // Convert the range from 0-255 to -100 to 100
        return static_cast<int8_t>((value / 255.0) * 200 - 100);
    }

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_can_thrusters;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_stern_left;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_stern_right;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_bow_left;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_bow_right;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BoatMotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
