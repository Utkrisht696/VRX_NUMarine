#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>

enum class State
{
  INITIAL_YAW_CORRECTION,
  MOVING_FORWARD,
  TURNING,
  STOP
};

class PIDControllerNode : public rclcpp::Node
{
public:
  PIDControllerNode()
    : Node("pid_controller_node"),
      current_state_(State::INITIAL_YAW_CORRECTION),
      current_waypoint_index_(0),
      kp_(10.0),
      ki_(0.0),
      kd_(0.0),
      yaw_kp_(5.0),
      yaw_ki_(0.0),
      yaw_kd_(2.0),
      previous_error_(Eigen::Vector3d::Zero()),
      integral_(Eigen::Vector3d::Zero()),
      yaw_previous_error_(0.0),
      yaw_integral_(0.0),
      initial_position_set_(false),
      current_position_(Eigen::Vector3d::Zero()),
      current_yaw_(0.0)
  {
    ned_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/ned_coordinates", 10, std::bind(&PIDControllerNode::ned_callback, this, std::placeholders::_1));
    yaw_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/imu_orientation_degrees", 10, std::bind(&PIDControllerNode::yaw_callback, this, std::placeholders::_1));

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

  void yaw_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    current_yaw_ = msg->vector.z;
  }

  void initialize_waypoints()
  {
    waypoints_.clear();
    waypoints_.emplace_back(initial_position_ + Eigen::Vector3d(0.0, 10.0, 0.0)); // Move East
    waypoints_.emplace_back(initial_position_ + Eigen::Vector3d(10.0, 10.0, 0.0)); // Move North
    waypoints_.emplace_back(initial_position_ + Eigen::Vector3d(10.0, 0.0, 0.0)); // Move West
    waypoints_.emplace_back(initial_position_); // Move South
  }

  void control_loop()
  {
    if (!initial_position_set_)
    {
      RCLCPP_WARN(this->get_logger(), "Initial position not set yet");
      return;
    }

    switch (current_state_)
    {
    case State::INITIAL_YAW_CORRECTION:
      correct_initial_yaw();
      break;
    case State::MOVING_FORWARD:
      move_forward();
      break;
    case State::TURNING:
      turn();
      break;
    case State::STOP:
      stop();
      break;
    }
  }

  void correct_initial_yaw()
  {
    double target_yaw;
    switch (current_waypoint_index_)
    {
    case 0:
      target_yaw = 90.0; // East
      break;
    case 1:
      target_yaw = 0.0; // North
      break;
    case 2:
      target_yaw = -90.0; // West
      break;
    case 3:
      target_yaw = -180.0; // South
      break;
    default:
      target_yaw = 0.0;
      break;
    }

    double yaw_error = shortest_angular_distance(current_yaw_, target_yaw);

    yaw_integral_ += yaw_error;
    double yaw_derivative = yaw_error - yaw_previous_error_;
    yaw_previous_error_ = yaw_error;

    double yaw_control_input = yaw_kp_ * yaw_error + yaw_ki_ * yaw_integral_ + yaw_kd_ * yaw_derivative;

    if (std::abs(yaw_error) < 5.0) // Close enough to target yaw
    {
      bow_port_thrust_publisher_->publish(create_thrust_msg(0.0));
      bow_star_thrust_publisher_->publish(create_thrust_msg(0.0));
      RCLCPP_INFO(this->get_logger(), "Yaw correction complete. Moving to forward state.");
      current_state_ = State::MOVING_FORWARD;
      return;
    }

    // Apply thrust for turning
    bow_port_thrust_publisher_->publish(create_thrust_msg(yaw_control_input));
    bow_star_thrust_publisher_->publish(create_thrust_msg(-yaw_control_input));
    RCLCPP_INFO(this->get_logger(), "Correcting yaw: [current: %f, target: %f, control input: %f]", current_yaw_, target_yaw, yaw_control_input);
  }

  void move_forward()
  {
    if (current_waypoint_index_ >= waypoints_.size())
    {
      RCLCPP_INFO(this->get_logger(), "Completed the square path.");
      current_state_ = State::STOP;
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
      current_state_ = State::TURNING;
      return;
    }

    stern_port1_thrust_publisher_->publish(stern_thrust_msg);
    stern_port2_thrust_publisher_->publish(stern_thrust_msg);
    stern_star1_thrust_publisher_->publish(stern_thrust_msg);
    stern_star2_thrust_publisher_->publish(stern_thrust_msg);

    RCLCPP_INFO(this->get_logger(), "Moving forward: [stern thrust: %f]", stern_thrust_msg.data);
  }

  void turn()
  {
    // Reset PID integrals for the next phase
    integral_ = Eigen::Vector3d::Zero();
    previous_error_ = Eigen::Vector3d::Zero();

    current_waypoint_index_++;
    if (current_waypoint_index_ >= waypoints_.size())
    {
      current_state_ = State::STOP;
    }
    else
    {
      current_state_ = State::INITIAL_YAW_CORRECTION;
    }
  }

  void stop()
  {
    bow_port_thrust_publisher_->publish(create_thrust_msg(0.0));
    bow_star_thrust_publisher_->publish(create_thrust_msg(0.0));
    stern_port1_thrust_publisher_->publish(create_thrust_msg(0.0));
    stern_port2_thrust_publisher_->publish(create_thrust_msg(0.0));
    stern_star1_thrust_publisher_->publish(create_thrust_msg(0.0));
    stern_star2_thrust_publisher_->publish(create_thrust_msg(0.0));
    RCLCPP_INFO(this->get_logger(), "Stopping all thrusters.");
  }

  double shortest_angular_distance(double from, double to)
  {
    double delta = to - from;
    while (delta > 180.0) delta -= 360.0;
    while (delta < -180.0) delta += 360.0;
    return delta;
  }

  std_msgs::msg::Float64 create_thrust_msg(double thrust)
  {
    std_msgs::msg::Float64 msg;
    msg.data = thrust;
    return msg;
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ned_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr yaw_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_port1_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_port2_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_star1_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_star2_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bow_port_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bow_star_thrust_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  State current_state_;
  Eigen::Vector3d current_position_;
  Eigen::Vector3d initial_position_;
  std::vector<Eigen::Vector3d> waypoints_;
  int current_waypoint_index_;
  bool initial_position_set_;
  double current_yaw_;
  double kp_;
  double ki_;
  double kd_;
  double yaw_kp_;
  double yaw_ki_;
  double yaw_kd_;
  Eigen::Vector3d previous_error_;
  Eigen::Vector3d integral_;
  double yaw_previous_error_;
  double yaw_integral_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControllerNode>());
  rclcpp::shutdown();
  return 0;
}
