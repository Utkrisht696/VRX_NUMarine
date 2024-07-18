#include <memory>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <std_msgs/msg/float64_multi_array.hpp>

using Spline2d = Eigen::Spline<double, 2>;
using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;

class GuidanceService : public rclcpp::Node
{
public:
    GuidanceService() : Node("guidance_service")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/guidance_service/output", 10);
        set_waypoints();
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    void set_waypoints()
    {
        std::vector<double> times = {0.0, 15.0, 30.0}; // Example times
        std::vector<double> north_positions = {0.0, 5.0, 10.0}; // Example north positions in meters
        std::vector<double> east_positions = {0.0, 0.0, 0.0}; // Example east positions in meters
        std::vector<double> north_velocities = {0.0, 0.0, 0.0}; // Example north velocities in meters per second
        std::vector<double> east_velocities = {0.0, 0.0, 0.0}; // Example east velocities in meters per second

        size_t n_points = times.size();

        VectorXd times_vec = Eigen::Map<VectorXd>(times.data(), n_points);
        VectorXd north_positions_vec = Eigen::Map<VectorXd>(north_positions.data(), n_points);
        VectorXd east_positions_vec = Eigen::Map<VectorXd>(east_positions.data(), n_points);
        VectorXd north_velocities_vec = Eigen::Map<VectorXd>(north_velocities.data(), n_points);
        VectorXd east_velocities_vec = Eigen::Map<VectorXd>(east_velocities.data(), n_points);

        MatrixXd waypoints(2, n_points);
        waypoints.row(0) = north_positions_vec.transpose();
        waypoints.row(1) = east_positions_vec.transpose();

        MatrixXd derivatives(2, n_points);
        derivatives.row(0) = north_velocities_vec.transpose();
        derivatives.row(1) = east_velocities_vec.transpose();

        std::vector<int> indices(n_points);
        std::iota(indices.begin(), indices.end(), 0); // Create sequence 0, 1, ..., n_points-1

        // Use Eigen::SplineFitting to fit the spline with derivatives
        auto spline = Eigen::SplineFitting<Spline2d>::InterpolateWithDerivatives(waypoints, derivatives, indices, 2);

        // Publish spline knots and coefficients
        std_msgs::msg::Float64MultiArray spline_msg;
        for (int i = 0; i < spline.knots().size(); ++i) {
            spline_msg.data.push_back(spline.knots()(i));
        }
        for (int i = 0; i < spline.ctrls().rows(); ++i) {
            for (int j = 0; j < spline.ctrls().cols(); ++j) {
                spline_msg.data.push_back(spline.ctrls()(i, j));
            }
        }
        publisher_->publish(spline_msg);

        // For debug: Evaluate splines at desired times
        VectorXd eval_times = VectorXd::LinSpaced(100, times_vec.minCoeff(), times_vec.maxCoeff());

        for (int i = 0; i < eval_times.size(); ++i)
        {
            double t = eval_times[i];
            Eigen::Matrix<double, 2, 1> eval = spline(t);
            Eigen::Matrix<double, 2, 1> eval_der = spline.derivatives(t, 1).col(1);

            RCLCPP_INFO(this->get_logger(), "Time: %.2f, N: %.2f, E: %.2f, N Vel: %.2f, E Vel: %.2f",
                        t, eval(0), eval(1), eval_der(0), eval_der(1));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuidanceService>());
    rclcpp::shutdown();
    return 0;
}
