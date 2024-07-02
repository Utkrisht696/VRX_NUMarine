#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class GoalPoseBroadcaster : public rclcpp::Node
{
public:
  GoalPoseBroadcaster() : Node("goal_pose_broadcaster")
  {
    goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&GoalPoseBroadcaster::goal_pose_callback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "goal_pose";
    transform.transform.translation.x = msg->pose.position.x;
    transform.transform.translation.y = msg->pose.position.y;
    transform.transform.translation.z = msg->pose.position.z;
    transform.transform.rotation = msg->pose.orientation;

    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPoseBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
