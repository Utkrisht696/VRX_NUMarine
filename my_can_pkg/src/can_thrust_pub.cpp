#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "can_msgs/msg/frame.hpp"

class ThrustToCANPublisher : public rclcpp::Node {
public:
    ThrustToCANPublisher() : Node("thrust_to_can_publisher") {
        // Subscriptions for each thrust topic
        sub_stern_port_ = this->create_subscription<std_msgs::msg::Float64>(
            "/wamv/thrusters/stern_port/thrust", 10, std::bind(&ThrustToCANPublisher::thrustCallback, this, std::placeholders::_1, 0x28));
        sub_stern_star_ = this->create_subscription<std_msgs::msg::Float64>(
            "/wamv/thrusters/stern_star/thrust", 10, std::bind(&ThrustToCANPublisher::thrustCallback, this, std::placeholders::_1, 0x29));
        sub_bow_port_ = this->create_subscription<std_msgs::msg::Float64>(
            "/wamv/thrusters/bow_port/thrust", 10, std::bind(&ThrustToCANPublisher::thrustCallback, this, std::placeholders::_1, 0x2A));
        sub_bow_star_ = this->create_subscription<std_msgs::msg::Float64>(
            "/wamv/thrusters/bow_star/thrust", 10, std::bind(&ThrustToCANPublisher::thrustCallback, this, std::placeholders::_1, 0x2B));

        // Publisher for CAN messages
        publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 10);
    }

private:
    void thrustCallback(const std_msgs::msg::Float64::SharedPtr msg, int can_id) {
        double thrust = msg->data;
        uint8_t pwm = remapThrustToPWM(thrust);

        can_msgs::msg::Frame frame;
        frame.header.stamp = this->get_clock()->now();
        frame.id = can_id;
        frame.is_rtr = false;
        frame.is_extended = false;
        frame.is_error = false;
        frame.dlc = 1;
        frame.data.resize(1);
        frame.data[0] = pwm;

        publisher_->publish(frame);
        RCLCPP_INFO(this->get_logger(), "Publishing thrust %f as PWM %u on CAN ID: 0x%X", thrust, pwm, can_id);
    }

    uint8_t remapThrustToPWM(double thrust) {
        // Remap the thrust value from -100-100 to 0-254 range
        double clamped_thrust = std::clamp(thrust, -100.0, 100.0);
        double scaled_pwm = (clamped_thrust + 100) * (254.0 / 200.0);
        return static_cast<uint8_t>(scaled_pwm);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_stern_port_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_stern_star_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_bow_port_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_bow_star_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThrustToCANPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
