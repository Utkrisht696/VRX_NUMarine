#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <Eigen/Dense>
#include <cmath>

class GoalPoseListener : public rclcpp::Node
{
public:
  GoalPoseListener()
    : Node("goal_pose_listener"),
      kp_(8.0),
      ki_(0.0),
      kd_(2.0),
      yaw_kp_(4.0),
      yaw_ki_(0.0),
      yaw_kd_(2.0),
      previous_error_(Eigen::Vector3d::Zero()),
      integral_(Eigen::Vector3d::Zero()),
      yaw_previous_error_(0.0),
      yaw_integral_(0.0),
      current_position_(Eigen::Vector3d::Zero()),
      goal_position_(Eigen::Vector3d::Zero()),
      current_yaw_(0.0),
      goal_yaw_(0.0),
      goal_received_(false)
  {
    goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&GoalPoseListener::goal_pose_callback, this, std::placeholders::_1));
    ned_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/ned_coordinates", 10, std::bind(&GoalPoseListener::ned_callback, this, std::placeholders::_1));
    yaw_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/imu_orientation_degrees", 10, std::bind(&GoalPoseListener::yaw_callback, this, std::placeholders::_1));

    stern_port1_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_port1/thrust", 10);
    stern_port2_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_port2/thrust", 10);
    stern_star1_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_star1/thrust", 10);
    stern_star2_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_star2/thrust", 10);
    bow_port_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_port/thrust", 10);
    bow_star_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_star/thrust", 10);

    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GoalPoseListener::control_loop, this));
  }

private:
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_position_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    tf2::Quaternion quat(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
    tf2::Matrix3x3 mat(quat);
    mat.getRPY(goal_roll_, goal_pitch_, goal_yaw_);
    goal_yaw_ = goal_yaw_ * 180.0 / M_PI; // Convert to degrees
    goal_received_ = true;

    RCLCPP_INFO(this->get_logger(), "Goal Pose Received: [N: %f, E: %f, D: %f], [Yaw: %f]",
                goal_position_.x(), goal_position_.y(), goal_position_.z(), goal_yaw_);
  }

  void ned_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    current_position_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    RCLCPP_INFO(this->get_logger(), "Current NED Coordinates: [N: %f, E: %f, D: %f]", current_position_.x(), current_position_.y(), current_position_.z());
  }

  void yaw_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    current_yaw_ = msg->vector.z;
    RCLCPP_INFO(this->get_logger(), "Current Yaw: %f", current_yaw_);
  }

  void control_loop()
  {
    if (!goal_received_) {
      RCLCPP_WARN(this->get_logger(), "Goal pose not received yet.");
      return;
    }

    // Position control
    Eigen::Vector3d error = goal_position_ - current_position_;
    integral_ += error;
    Eigen::Vector3d derivative = error - previous_error_;
    previous_error_ = error;

    Eigen::Vector3d control_input = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // Apply control commands to the stern thrusters
    std_msgs::msg::Float64 stern_thrust_msg;
    stern_thrust_msg.data = control_input.norm(); // Forward/backward thrust based on the magnitude of the control input

    stern_port1_thrust_publisher_->publish(stern_thrust_msg);
    stern_port2_thrust_publisher_->publish(stern_thrust_msg);
    stern_star1_thrust_publisher_->publish(stern_thrust_msg);
    stern_star2_thrust_publisher_->publish(stern_thrust_msg);

    // Orientation control
    double yaw_error = shortest_angular_distance(current_yaw_, goal_yaw_);
    yaw_integral_ += yaw_error;
    double yaw_derivative = yaw_error - yaw_previous_error_;
    yaw_previous_error_ = yaw_error;

    double yaw_control_input = yaw_kp_ * yaw_error + yaw_ki_ * yaw_integral_ + yaw_kd_ * yaw_derivative;

    // Apply control commands to the bow thrusters for yaw correction
    std_msgs::msg::Float64 bow_port_thrust_msg;
    std_msgs::msg::Float64 bow_star_thrust_msg;
    bow_port_thrust_msg.data = yaw_control_input;
    bow_star_thrust_msg.data = -yaw_control_input;

    bow_port_thrust_publisher_->publish(bow_port_thrust_msg);
    bow_star_thrust_publisher_->publish(bow_star_thrust_msg);

    // Print current state for debugging
    RCLCPP_INFO(this->get_logger(), "Current Position: [N: %f, E: %f, D: %f]", current_position_.x(), current_position_.y(), current_position_.z());
    RCLCPP_INFO(this->get_logger(), "Current Yaw: %f", current_yaw_);
    RCLCPP_INFO(this->get_logger(), "Goal Position: [N: %f, E: %f, D: %f]", goal_position_.x(), goal_position_.y(), goal_position_.z());
    RCLCPP_INFO(this->get_logger(), "Goal Yaw: %f", goal_yaw_);
  }

  double shortest_angular_distance(double from, double to)
  {
    double delta = to - from;
    while (delta > 180.0) delta -= 360.0;
    while (delta < -180.0) delta += 360.0;
    return delta;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ned_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr yaw_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_port1_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_port2_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_star1_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_star2_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bow_port_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bow_star_thrust_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  Eigen::Vector3d current_position_;
  Eigen::Vector3d goal_position_;
  double current_yaw_;
  double goal_yaw_;
  double goal_roll_, goal_pitch_;

  double kp_, ki_, kd_;
  double yaw_kp_, yaw_ki_, yaw_kd_;
  Eigen::Vector3d previous_error_;
  Eigen::Vector3d integral_;
  double yaw_previous_error_;
  double yaw_integral_;
  bool goal_received_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPoseListener>());
  rclcpp::shutdown();
  return 0;
}
