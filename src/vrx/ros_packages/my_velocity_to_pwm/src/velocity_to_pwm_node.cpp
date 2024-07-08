#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class ThrusterController : public rclcpp::Node
{
public:
  ThrusterController()
  : Node("thruster_controller")
  {
    this->declare_parameter<double>("x_uu", 72.4);
    this->declare_parameter<double>("x_u", 51.3);
    this->declare_parameter<double>("max_velocity_mps", 7.71667);
    this->declare_parameter<double>("fluid_density", 1000.0);
    this->declare_parameter<double>("propeller_diameter", 0.2);
    this->declare_parameter<double>("linear_scaling_factor", 0.1);
    this->declare_parameter<double>("angular_scaling_factor", 0.05);

    this->get_parameter("x_uu", x_uu_);
    this->get_parameter("x_u", x_u_);
    this->get_parameter("max_velocity_mps", max_velocity_mps_);
    this->get_parameter("fluid_density", fluid_density_);
    this->get_parameter("propeller_diameter", propeller_diameter_);
    this->get_parameter("linear_scaling_factor", linear_scaling_factor_);
    this->get_parameter("angular_scaling_factor", angular_scaling_factor_);

    max_thrust_ = (x_u_ + x_uu_ * max_velocity_mps_) * max_velocity_mps_ / 2;

    // Publishers for thrusters
    stern_port_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_port/thrust", 10);
    stern_star_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_star/thrust", 10);
    bow_port_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_port/thrust", 10);
    bow_star_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_star/thrust", 10);

    // Subscriber to cmd_vel
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&ThrusterController::cmdVelCallback, this, std::placeholders::_1));
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    // Apply scaling factors
    linear_x *= linear_scaling_factor_;
    angular_z *= angular_scaling_factor_;

    // Calculate the thrusts for each thruster
    double stern_thrust = linear_x * max_thrust_;
    double bow_thrust = angular_z * max_thrust_;

    double stern_port_thrust = stern_thrust;
    double stern_star_thrust = stern_thrust;
    double bow_port_thrust = bow_thrust;
    double bow_star_thrust = -bow_thrust;

    // Publish the thrust commands
    publishThrust(stern_port_pub_, stern_port_thrust);
    publishThrust(stern_star_pub_, stern_star_thrust);
    publishThrust(bow_port_pub_, bow_port_thrust);
    publishThrust(bow_star_pub_, bow_star_thrust);
  }

  void publishThrust(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher, double thrust)
  {
    auto thrust_msg = std_msgs::msg::Float64();
    thrust_msg.data = static_cast<double>(thrust);
    publisher->publish(thrust_msg);
  }

  double x_uu_;
  double x_u_;
  double max_velocity_mps_;
  double max_thrust_;
  double fluid_density_;
  double propeller_diameter_;
  double linear_scaling_factor_;
  double angular_scaling_factor_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_port_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_star_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bow_port_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bow_star_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ThrusterController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
