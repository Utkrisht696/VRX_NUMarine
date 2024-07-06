#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

class VelocityToPWMNode : public rclcpp::Node
{
public:
    VelocityToPWMNode() : Node("velocity_to_pwm_node")
    {
        // Create a subscriber to the /cmd_vel topic
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&VelocityToPWMNode::cmdVelCallback, this, std::placeholders::_1));

        // Create publishers for the thruster topics
        port_thruster_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_port/thrust", 10);
        star_thruster_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_star/thrust", 10);
        bow_port_thruster_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_port/thrust", 10);
        bow_star_thruster_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_star/thrust", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

        // Scale the linear and angular velocities to match the thruster's range
        double port_thrust = (linear_x) * 100.0;
        double star_thrust = (linear_x) * 100.0;
        double bow_port_thrust = (-angular_z) * 50.0;
        double bow_star_thrust = (angular_z) * 50.0;


        // Create and publish thrust messages
        auto port_msg = std_msgs::msg::Float64();
        port_msg.data = port_thrust;
        port_thruster_publisher_->publish(port_msg);

        auto star_msg = std_msgs::msg::Float64();
        star_msg.data = star_thrust;
        star_thruster_publisher_->publish(star_msg);

         // Create and publish thrust messages for bow thrusters
        auto bow_port_msg = std_msgs::msg::Float64();
        bow_port_msg.data = bow_port_thrust;
        bow_port_thruster_publisher_->publish(bow_port_msg);

        auto bow_star_msg = std_msgs::msg::Float64();
        bow_star_msg.data = bow_star_thrust;
        bow_star_thruster_publisher_->publish(bow_star_msg);

        RCLCPP_INFO(this->get_logger(), "Port thrust: %f, Starboard thrust: %f", port_thrust, star_thrust);
        RCLCPP_INFO(this->get_logger(), "Bow Port thrust: %f, Bow Starboard thrust: %f", bow_port_thrust, bow_star_thrust);
   
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr port_thruster_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr star_thruster_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bow_port_thruster_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bow_star_thruster_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityToPWMNode>());
    rclcpp::shutdown();
    return 0;
}
