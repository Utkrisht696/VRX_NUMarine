#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class VelocityToPwmNode : public rclcpp::Node
{
public:
    VelocityToPwmNode()
        : Node("velocity_to_pwm_node")
    {
        // Create a subscriber for the cmd_vel topic
        velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&VelocityToPwmNode::velocityCallback, this, std::placeholders::_1));
        
        // Create publishers for the PWM thruster topics
        pwm_stern_left_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_port/thrust", 10);
        pwm_stern_right_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_star/thrust", 10);
        pwm_bow_left_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_port/thrust", 10);
        pwm_bow_right_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_star/thrust", 10);
    }

private:
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear_vel = msg->linear.x;
        double angular_vel = msg->angular.z;
        
        double pwm_stern_left, pwm_stern_right, pwm_bow_left, pwm_bow_right;
        convertVelocityToPwm(linear_vel, angular_vel, pwm_stern_left, pwm_stern_right, pwm_bow_left, pwm_bow_right);
        
        std_msgs::msg::Float64 pwm_msg;
        
        pwm_msg.data = pwm_stern_left;
        pwm_stern_left_publisher_->publish(pwm_msg);

        pwm_msg.data = pwm_stern_right;
        pwm_stern_right_publisher_->publish(pwm_msg);

        pwm_msg.data = pwm_bow_left;
        pwm_bow_left_publisher_->publish(pwm_msg);

        pwm_msg.data = pwm_bow_right;
        pwm_bow_right_publisher_->publish(pwm_msg);
    }

    void convertVelocityToPwm(double linear_vel, double angular_vel,
                              double &pwm_stern_left, double &pwm_stern_right,
                              double &pwm_bow_left, double &pwm_bow_right)
    {
        // Constants for conversion (these might need to be tuned for your setup)
        const double max_pwm = 200.0;
        const double min_pwm = -200.0;
        const double max_linear_vel = 1.0; // Max linear velocity
        const double max_angular_vel = 1.0; // Max angular velocity

        // Scale the linear velocity to the PWM range
        double scaled_linear = linear_vel / max_linear_vel * max_pwm;
        // Scale the angular velocity to the PWM range
        double scaled_angular = angular_vel / max_angular_vel * max_pwm;

        // Apply the scaled values to the respective thrusters
        pwm_stern_left = scaled_linear;
        pwm_stern_right = scaled_linear;

        pwm_bow_left = scaled_angular;
        pwm_bow_right = -scaled_angular;

        // Clamp the PWM values to the range [-200, 200]
        pwm_stern_left = std::clamp(pwm_stern_left, min_pwm, max_pwm);
        pwm_stern_right = std::clamp(pwm_stern_right, min_pwm, max_pwm);
        pwm_bow_left = std::clamp(pwm_bow_left, min_pwm, max_pwm);
        pwm_bow_right = std::clamp(pwm_bow_right, min_pwm, max_pwm);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pwm_stern_left_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pwm_stern_right_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pwm_bow_left_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pwm_bow_right_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityToPwmNode>());
    rclcpp::shutdown();
    return 0;
}
