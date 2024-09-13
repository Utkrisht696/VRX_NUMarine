#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

class LidarFilterNode : public rclcpp::Node
{
public:
    LidarFilterNode(const rclcpp::NodeOptions & options) : Node("lidar_filter_node", options)
    {
        // Explicitly declare parameters
        this->declare_parameter<double>("min_z", -0.5);
        this->declare_parameter<double>("max_z", 5.0);

        // Subscriber to the input point cloud
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/wamv/sensors/lidars/lidar_wamv_sensor/points", 10,
            std::bind(&LidarFilterNode::pointCloudCallback, this, std::placeholders::_1));

        // Publisher for the filtered point cloud
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_point_cloud", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 message to PCL PointCloud with XYZ, intensity, and ring fields
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        // Create the PassThrough filter object
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(cloud);

        // Apply the filter along the z-axis
        double min_z = this->get_parameter("min_z").as_double();
        double max_z = this->get_parameter("max_z").as_double();
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_z, max_z);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pass.filter(*cloud_filtered);

        // Convert the filtered cloud back to ROS message format
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header = msg->header;  // Maintain the original header

        // Publish the filtered point cloud
        point_cloud_pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Load parameters from a YAML file
    auto node_options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);

    // Create and spin the node
    rclcpp::spin(std::make_shared<LidarFilterNode>(node_options));

    rclcpp::shutdown();
    return 0;
}

