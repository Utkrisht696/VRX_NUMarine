#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"

class HeartbeatPublisher : public rclcpp::Node
{
public:
    HeartbeatPublisher() : Node("heartbeat_publisher")
    {
        publisher_ = this->create_publisher<can_msgs::msg::Frame>("/can_tx", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), std::bind(&HeartbeatPublisher::publishHeartbeat, this));
    }

private:
    void publishHeartbeat()
    {
        auto message = can_msgs::msg::Frame();
        message.id = 42;  // Set ID for AUTO_MODE
        message.dlc = 1;
        message.data = {1};  // Set AUTO_MODE to 1

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing AUTO_MODE");
    }

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeartbeatPublisher>());
    rclcpp::shutdown();
    return 0;
}

