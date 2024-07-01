#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include <Eigen/Dense>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class PIDControllerNode : public rclcpp::Node
{
public:
  PIDControllerNode()
    : Node("pid_controller_node"),
      current_waypoint_index_(0),
      turning_(false),
      turn_start_yaw_(0.0),
      kp_(10.0),
      ki_(0.0),
      kd_(0.0),
      previous_error_(Eigen::Vector3d::Zero()),
      integral_(Eigen::Vector3d::Zero()),
      initial_position_set_(false),
      current_position_(Eigen::Vector3d::Zero())
  {
    ned_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/ned_coordinates", 10, std::bind(&PIDControllerNode::ned_callback, this, std::placeholders::_1));
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/wamv/sensors/imu/imu/data", 10, std::bind(&PIDControllerNode::imu_callback, this, std::placeholders::_1));

    stern_port1_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_port1/thrust", 10);
    stern_port2_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_port2/thrust", 10);
    stern_star1_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_star1/thrust", 10);
    stern_star2_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_star2/thrust", 10);
    bow_port_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_port/thrust", 10);
    bow_star_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_star/thrust", 10);

    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PIDControllerNode::control_loop, this));
  }

private:
  void ned_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    current_position_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    if (!initial_position_set_) {
      initial_position_ = current_position_;
      initialize_waypoints();
      initial_position_set_ = true;
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_orientation_ = msg->orientation;
  }

  void initialize_waypoints()
  {
    waypoints_.clear();
    waypoints_.emplace_back(initial_position_ + Eigen::Vector3d(10.0, 0.0, 0.0)); // First side
    waypoints_.emplace_back(initial_position_ + Eigen::Vector3d(10.0, 10.0, 0.0)); // Second side
    waypoints_.emplace_back(initial_position_ + Eigen::Vector3d(0.0, 10.0, 0.0)); // Third side
    waypoints_.emplace_back(initial_position_); // Back to start
  }

  void control_loop()
  {
    if (!initial_position_set_)
    {
      RCLCPP_WARN(this->get_logger(), "Initial position not set yet");
      return;
    }

    if (turning_)
    {
      perform_turn();
      return;
    }

    if (current_waypoint_index_ >= waypoints_.size())
    {
      RCLCPP_INFO(this->get_logger(), "Completed the square path.");
      return;
    }

    Eigen::Vector3d goal_position = waypoints_[current_waypoint_index_];

    // Calculate the error between the goal position and the current position
    Eigen::Vector3d error = goal_position - current_position_;
    integral_ += error;
    Eigen::Vector3d derivative = error - previous_error_;
    previous_error_ = error;

    // Compute the control input using the PID formula
    Eigen::Vector3d control_input = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // Apply control commands to the stern thrusters
    std_msgs::msg::Float64 stern_thrust_msg;
    stern_thrust_msg.data = control_input.norm(); // Forward/backward thrust based on the magnitude of the control input

    if (error.norm() < 0.5) // Threshold to stop when close to the goal
    {
      stern_thrust_msg.data = 0.0;
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %d. Stopping thrusters and preparing to turn.", current_waypoint_index_);
      turning_ = true;
      turn_start_yaw_ = get_yaw_from_orientation(imu_orientation_);
      return;
    }

    stern_port1_thrust_publisher_->publish(stern_thrust_msg);
    stern_port2_thrust_publisher_->publish(stern_thrust_msg);
    stern_star1_thrust_publisher_->publish(stern_thrust_msg);
    stern_star2_thrust_publisher_->publish(stern_thrust_msg);

    RCLCPP_INFO(this->get_logger(), "Publishing thrust command: [stern: %f]", stern_thrust_msg.data);
  }

  void perform_turn()
  {
    double current_yaw = get_yaw_from_orientation(imu_orientation_);
    double target_yaw = turn_start_yaw_ + M_PI_2; // Target 90 degrees turn
    double yaw_error = shortest_angular_distance(current_yaw, target_yaw);

    std_msgs::msg::Float64 bow_port_thrust_msg;
    std_msgs::msg::Float64 bow_star_thrust_msg;

    if (std::abs(yaw_error) < 0.1) // Close enough to 90 degrees turn
    {
      bow_port_thrust_msg.data = 0.0;
      bow_star_thrust_msg.data = 0.0;
      bow_port_thrust_publisher_->publish(bow_port_thrust_msg);
      bow_star_thrust_publisher_->publish(bow_star_thrust_msg);
      RCLCPP_INFO(this->get_logger(), "Completed turn. Proceeding to next waypoint.");
      current_waypoint_index_++;
      turning_ = false;
      return;
    }

    // Apply thrust for turning
    bow_port_thrust_msg.data = 0.5;
    bow_star_thrust_msg.data = -0.5;
    bow_port_thrust_publisher_->publish(bow_port_thrust_msg);
    bow_star_thrust_publisher_->publish(bow_star_thrust_msg);
    RCLCPP_INFO(this->get_logger(), "Turning: [bow port: %f, bow star: %f]", bow_port_thrust_msg.data, bow_star_thrust_msg.data);
  }

  double get_yaw_from_orientation(const geometry_msgs::msg::Quaternion &q)
  {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  double shortest_angular_distance(double from, double to)
  {
    double delta = to - from;
    while (delta > M_PI) delta -= 2.0 * M_PI;
    while (delta < -M_PI) delta += 2.0 * M_PI;
    return delta;
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ned_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_port1_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_port2_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_star1_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_star2_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bow_port_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bow_star_thrust_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  Eigen::Vector3d current_position_;
  Eigen::Vector3d initial_position_;
  std::vector<Eigen::Vector3d> waypoints_;
  int current_waypoint_index_;
  bool initial_position_set_;
  bool turning_;
  geometry_msgs::msg::Quaternion imu_orientation_;
  double turn_start_yaw_;
  double kp_;
  double ki_;
  double kd_;
  Eigen::Vector3d previous_error_;
  Eigen::Vector3d integral_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControllerNode>());
  rclcpp::shutdown();
  return 0;
}
