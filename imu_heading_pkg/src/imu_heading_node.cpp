#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class ImuHeadingNode : public rclcpp::Node
{
public:
    ImuHeadingNode() : Node("imu_heading_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/Imu", 10,
            std::bind(&ImuHeadingNode::imu_callback, this, std::placeholders::_1));
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {
        tf2::Quaternion quaternion(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 m(quaternion);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Convert yaw from radians to degrees
        double yaw_degrees = yaw * (180.0 / M_PI);

        RCLCPP_INFO(this->get_logger(), "Heading (Yaw): %f degrees", yaw_degrees);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuHeadingNode>());
    rclcpp::shutdown();
    return 0;
}
