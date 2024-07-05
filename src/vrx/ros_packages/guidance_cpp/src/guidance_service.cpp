#include <memory>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "guidance_cpp/srv/trajectory.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

using std::placeholders::_1;
using std::placeholders::_2;
using Spline2d = Eigen::Spline<double, 2>;

const double METERS_PER_DEGREE_LATITUDE = 111320;
const double METERS_PER_DEGREE_LONGITUDE = 111320 * std::cos(-33.72276913511836 * M_PI / 180.0);

class GuidanceService : public rclcpp::Node
{
public:
    GuidanceService() : Node("guidance_service")
    {
        service_ = this->create_service<guidance_cpp::srv::Trajectory>("trajectory", std::bind(&GuidanceService::trajectory_callback, this, _1, _2));
    }

private:
    void trajectory_callback(const std::shared_ptr<guidance_cpp::srv::Trajectory::Request> request,
                             std::shared_ptr<guidance_cpp::srv::Trajectory::Response> response)
    {
        auto times = request->times;
        auto north_positions = request->north_positions;
        auto east_positions = request->east_positions;

        size_t n_points = times.size();
        if (north_positions.size() != n_points || east_positions.size() != n_points)
        {
            RCLCPP_ERROR(this->get_logger(), "Number of north/east positions must match number of times");
            return;
        }

        Eigen::VectorXd times_vec = Eigen::Map<Eigen::VectorXd>(times.data(), n_points);
        Eigen::VectorXd north_positions_vec = Eigen::Map<Eigen::VectorXd>(north_positions.data(), n_points);
        Eigen::VectorXd east_positions_vec = Eigen::Map<Eigen::VectorXd>(east_positions.data(), n_points);

        // Convert north and east positions to latitude and longitude
        const double origin_lat = -33.72276913511836;
        const double origin_lon = 150.67398721748103;
        const double origin_alt = 1.1486131474375725;

        Eigen::VectorXd latitudes = origin_lat + north_positions_vec / METERS_PER_DEGREE_LATITUDE;
        Eigen::VectorXd longitudes = origin_lon + east_positions_vec / METERS_PER_DEGREE_LONGITUDE;
        Eigen::VectorXd altitudes = Eigen::VectorXd::Constant(n_points, origin_alt);

        Eigen::Matrix<double, 2, Eigen::Dynamic> waypoints(2, n_points);
        waypoints.row(0) = latitudes.transpose();
        waypoints.row(1) = longitudes.transpose();

        Spline2d spline = Eigen::SplineFitting<Spline2d>::Interpolate(waypoints, 2);

        // Evaluate splines at desired times
        Eigen::VectorXd eval_times = Eigen::VectorXd::LinSpaced(100, times_vec.minCoeff(), times_vec.maxCoeff());

        for (int i = 0; i < eval_times.size(); ++i)
        {
            double t = eval_times[i];
            Eigen::Matrix<double, 2, 1> eval = spline(t);
            Eigen::Matrix<double, 2, 1> eval_der = spline.derivatives(t, 1).col(1);

            response->latitudes.push_back(eval(0));
            response->longitudes.push_back(eval(1));
            response->altitudes.push_back(origin_alt);  // Constant altitude
            response->lat_velocities.push_back(eval_der(0));
            response->lon_velocities.push_back(eval_der(1));
            response->alt_velocities.push_back(0.0);  // Constant altitude
        }
    }

    rclcpp::Service<guidance_cpp::srv::Trajectory>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuidanceService>());
    rclcpp::shutdown();
    return 0;
}
