#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "can_msgs/msg/frame.hpp"

class ThrustConverter : public rclcpp::Node {
public:
    ThrustConverter() : Node("thrust_converter") {
        subscription_stern_port_ = this->create_subscription<std_msgs::msg::Float64>(
            "/wamv/thrusters/stern_port/thrust", 10,
            std::bind(&ThrustConverter::stern_port_callback, this, std::placeholders::_1));

        subscription_stern_star_ = this->create_subscription<std_msgs::msg::Float64>(
            "/wamv/thrusters/stern_star/thrust", 10,
            std::bind(&ThrustConverter::stern_star_callback, this, std::placeholders::_1));

        subscription_bow_port_ = this->create_subscription<std_msgs::msg::Float64>(
            "/wamv/thrusters/bow_port/thrust", 10,
            std::bind(&ThrustConverter::bow_port_callback, this, std::placeholders::_1));

        subscription_bow_star_ = this->create_subscription<std_msgs::msg::Float64>(
            "/wamv/thrusters/bow_star/thrust", 10,
            std::bind(&ThrustConverter::bow_star_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 10);
    }

private:
    void stern_port_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        uint8_t converted_value = convert_thrust(msg->data);
        publish_to_can(0x29, converted_value); // Stern Port
    }

    void stern_star_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        uint8_t converted_value = convert_thrust(msg->data);
        publish_to_can(0x28, converted_value); // Stern Starboard
    }

    void bow_port_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        uint8_t converted_value = convert_thrust(msg->data);
        publish_to_can(0x2A, converted_value); // Bow Port
    }

    void bow_star_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        uint8_t converted_value = convert_thrust(msg->data);
        publish_to_can(0x2B, converted_value); // Bow Starboard
    }

    uint8_t convert_thrust(double thrust) {
        // Convert thrust from range -100 to 100 to range 0 to 254
        return static_cast<uint8_t>((thrust + 100) * 254 / 200);
    }

    void publish_to_can(int id, uint8_t speed) {
        auto frame = can_msgs::msg::Frame();
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

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_stern_port_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_stern_star_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_bow_port_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_bow_star_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrustConverter>());
    rclcpp::shutdown();
    return 0;
}
