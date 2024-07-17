#include <memory>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

using Spline2d = Eigen::Spline<double, 2>;

const double METERS_PER_DEGREE_LATITUDE = 111320;
const double METERS_PER_DEGREE_LONGITUDE = 111320 * std::cos(-33.72276913511836 * M_PI / 180.0);

class GuidanceService : public rclcpp::Node
{
public:
    GuidanceService() : Node("guidance_service")
    {
        set_waypoints();
    }

private:
    void set_waypoints()
    {
        std::vector<double> times = {0.0, 10.0, 20.0, 30.0}; // Example times
        std::vector<double> north_positions = {0.0, 100.0, 200.0, 300.0}; // Example north positions
        std::vector<double> east_positions = {0.0, 50.0, 100.0, 150.0}; // Example east positions

        size_t n_points = times.size();

        Eigen::VectorXd times_vec = Eigen::Map<Eigen::VectorXd>(times.data(), n_points);
        Eigen::VectorXd north_positions_vec = Eigen::Map<Eigen::VectorXd>(north_positions.data(), n_points);
        Eigen::VectorXd east_positions_vec = Eigen::Map<Eigen::VectorXd>(east_positions.data(), n_points);

        // Convert north and east positions to latitude and longitude
        const double origin_lat = -33.72276913511836;
        const double origin_lon = 150.67398721748103;
        const double origin_alt = 1.1486131474375725;

        Eigen::VectorXd latitudes = Eigen::VectorXd::Constant(n_points, origin_lat) + (north_positions_vec.array() / METERS_PER_DEGREE_LATITUDE).matrix();
        Eigen::VectorXd longitudes = Eigen::VectorXd::Constant(n_points, origin_lon) + (east_positions_vec.array() / METERS_PER_DEGREE_LONGITUDE).matrix();
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

            RCLCPP_INFO(this->get_logger(), "Time: %.2f, Lat: %.6f, Lon: %.6f, Alt: %.2f, Lat Vel: %.6f, Lon Vel: %.6f, Alt Vel: %.2f",
                        t, eval(0), eval(1), origin_alt, eval_der(0), eval_der(1), 0.0);
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
