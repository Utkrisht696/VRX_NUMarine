#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"

class HeartbeatSubscriber : public rclcpp::Node
{
public:
    HeartbeatSubscriber() : Node("heartbeat_subscriber")
    {
        subscription_ = this->create_subscription<can_msgs::msg::Frame>(
            "/can_rx", 10, std::bind(&HeartbeatSubscriber::canMessageCallback, this, std::placeholders::_1));
    }

private:
    void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg)
    {
        const uint32_t AUTO_MODE_ID = 0;   // Replace with actual ID from DBC
        const uint32_t ESTOP_ID = 15;       // Replace with actual ID from DBC
 // Add other E-Stop configurations 	TODO
        if (msg->id == AUTO_MODE_ID)
        {
            if (msg->data[0] == 0)
            {
                RCLCPP_INFO(this->get_logger(), "AUTO_MODE = 0, Printing 1");
            }
            else if (msg->data[0] == 1)
            {
                RCLCPP_INFO(this->get_logger(), "AUTO_MODE = 1, Printing 2");
            }
        }

        if (msg->id == ESTOP_ID && msg->data[0] == 1)
        {
            RCLCPP_INFO(this->get_logger(), "ESTOP = 1, Printing 3");
        }
    }

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeartbeatSubscriber>());
    rclcpp::shutdown();
    return 0;
}

