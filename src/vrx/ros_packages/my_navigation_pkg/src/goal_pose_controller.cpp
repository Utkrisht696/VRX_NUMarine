#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // Include this for tf2::getYaw
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

class GoalPoseController : public rclcpp::Node
{
public:
  GoalPoseController()
    : Node("goal_pose_controller"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
  {
    stern_port1_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_port1/thrust", 10);
    stern_port2_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_port2/thrust", 10);
    stern_star1_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_star1/thrust", 10);
    stern_star2_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_star2/thrust", 10);
    bow_port_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_port/thrust", 10);
    bow_star_thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_star/thrust", 10);

    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GoalPoseController::control_loop, this));
  }

private:
  void control_loop()
  {
    geometry_msgs::msg::TransformStamped current_transform;
    geometry_msgs::msg::TransformStamped goal_transform;

    try {
      current_transform = tf_buffer_.lookupTransform("map", "wamv/wamv/base_link", tf2::TimePointZero);
      goal_transform = tf_buffer_.lookupTransform("map", "goal_pose", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    tf2::Transform current_pose;
    tf2::Transform goal_pose;
    tf2::fromMsg(current_transform.transform, current_pose);
    tf2::fromMsg(goal_transform.transform, goal_pose);

    // Calculate relative transform from current pose to goal pose
    tf2::Transform relative_transform = current_pose.inverse() * goal_pose;
    tf2::Vector3 position_error = relative_transform.getOrigin();
    double yaw_error = tf2::getYaw(relative_transform.getRotation());

    // Print current state for debugging
    RCLCPP_INFO(this->get_logger(), "Position Error: [x: %f, y: %f, z: %f]", position_error.x(), position_error.y(), position_error.z());
    RCLCPP_INFO(this->get_logger(), "Yaw Error: %f", yaw_error);

    // Apply control (simplified for this example, replace with PID control logic)
    std_msgs::msg::Float64 stern_thrust_msg;
    stern_thrust_msg.data = position_error.length(); // Use magnitude of position error for thrust
    stern_port1_thrust_publisher_->publish(stern_thrust_msg);
    stern_port2_thrust_publisher_->publish(stern_thrust_msg);
    stern_star1_thrust_publisher_->publish(stern_thrust_msg);
    stern_star2_thrust_publisher_->publish(stern_thrust_msg);

    std_msgs::msg::Float64 bow_port_thrust_msg;
    std_msgs::msg::Float64 bow_star_thrust_msg;
    bow_port_thrust_msg.data = yaw_error; // Use yaw error for yaw control
    bow_star_thrust_msg.data = -yaw_error;
    bow_port_thrust_publisher_->publish(bow_port_thrust_msg);
    bow_star_thrust_publisher_->publish(bow_star_thrust_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_port1_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_port2_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_star1_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stern_star2_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bow_port_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bow_star_thrust_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPoseController>());
  rclcpp::shutdown();
  return 0;
}
