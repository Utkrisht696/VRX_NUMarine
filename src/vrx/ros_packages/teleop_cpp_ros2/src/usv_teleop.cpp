// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float64.hpp"
// #include <stdio.h>
// #include <unistd.h>
// #include <termios.h>

// #include <map>
// using namespace std::chrono_literals;

// /* This example creates a subclass of Node and uses std::bind() to register a
// * member function as a callback from the timer. */


// int getch(void)
// {
//   int ch;
//   struct termios oldt;
//   struct termios newt;

//   // Store old settings, and copy to new settings
//   tcgetattr(STDIN_FILENO, &oldt);
//   newt = oldt;

//   // Make required changes and apply the settings
//   newt.c_lflag &= ~(ICANON | ECHO);
//   newt.c_iflag |= IGNBRK;
//   newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
//   newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
//   newt.c_cc[VMIN] = 1;
//   newt.c_cc[VTIME] = 0;
//   tcsetattr(fileno(stdin), TCSANOW, &newt);

//   // Get the current character
//   ch = getchar();

//   // Reapply old settings
//   tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

//   return ch;
// }

// int main(int argc, char * argv[])
// {
// const char* msg = R"(
// Reading from the keyboard!
// ---------------------------
// Thrust Key Left:
//     w    
        
//     s    

// Thrust Key Left:
//     e    
        
//     d    

// w/s : increase/decrease Thrust Left Key by 0.1
// w/s : increase/decrease Thrust Right Key by 0.1
// NOTE : Increasing or Decreasing will take affect live on the moving robot.
//     Consider Stopping the robot before changing it.
// CTRL-C to quit
// )";
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("usv_teleop");
//     std::cout<<msg<<std::endl;

//     auto publisher_thrust_left = node->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
//     auto publisher_thrust_right = node->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10);


//     auto message_thrust_left = std_msgs::msg::Float64();
//     auto message_thrust_right = std_msgs::msg::Float64();

//     message_thrust_left.data = 0;
//     message_thrust_right.data = 0;

//     while (rclcpp::ok())
//     {
//         char key = getch();

//         if(key == 's' || key == 'S'){
//         message_thrust_left.data =  message_thrust_left.data - 2.0;
//         publisher_thrust_left->publish(message_thrust_left);
//         std::cout<<"Thrust Value Left:- "<<message_thrust_left.data<<std::endl;
//         }

//         if(key == 'w' || key == 'W'){
//         message_thrust_left.data =  message_thrust_left.data + 2.0;
//         publisher_thrust_left->publish(message_thrust_left);
//         std::cout<<"Thrust Value Left:- "<<message_thrust_left.data<<std::endl;
//         }

//         if(key == 'd' || key == 'D'){
//         message_thrust_right.data =  message_thrust_right.data - 2.0;
//         publisher_thrust_right->publish(message_thrust_right);
//         std::cout<<"Thrust Value Right:- "<<message_thrust_right.data<<std::endl;
//         }

//         if(key == 'e' || key == 'E'){
//         message_thrust_right.data =  message_thrust_right.data + 2.0;
//         publisher_thrust_right->publish(message_thrust_right);
//         std::cout<<"Thrust Value Right:- "<<message_thrust_right.data<<std::endl;
//         }

//         else if(key == '\x03'){
//             break;
//         }
//     }
    

//     rclcpp::shutdown();
//     return 0;
// }

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

using namespace std::chrono_literals;

class TeleopUSV : public rclcpp::Node
{
public:
    TeleopUSV()
        : Node("usv_teleop"),
          thrust_speed_(5.0),
          thrust_bow_port_(0.0), thrust_bow_star_(0.0),
          thrust_stern_port1_(0.0), thrust_stern_port2_(0.0),
          thrust_stern_star1_(0.0), thrust_stern_star2_(0.0)
    {
        publisher_thrust_bow_port_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_port/thrust", 10);
        publisher_thrust_bow_star_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/bow_star/thrust", 10);
        publisher_thrust_stern_port1_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_port1/thrust", 10);
        publisher_thrust_stern_port2_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_port2/thrust", 10);
        publisher_thrust_stern_star1_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_star1/thrust", 10);
        publisher_thrust_stern_star2_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/stern_star2/thrust", 10);

        print_instructions();
    }

    void run()
    {
        while (rclcpp::ok())
        {
            char key = getch();

            switch (key)
            {
            case 'w':
            case 'W':
                adjust_stern_thrust(thrust_speed_);
                break;
            case 's':
            case 'S':
                adjust_stern_thrust(-thrust_speed_);
                break;
            case 'd':
            case 'D':
                adjust_bow_thrust(thrust_speed_, -thrust_speed_);
                break;
            case 'a':
            case 'A':
                adjust_bow_thrust(-thrust_speed_, thrust_speed_);
                break;
            case 'q':
            case 'Q':
                thrust_speed_ += 5;
                std::cout << "Thrust speed increased to: " << thrust_speed_ << std::endl;
                break;
            case 'z':
            case 'Z':
                thrust_speed_ -= 5;
                std::cout << "Thrust speed decreased to: " << thrust_speed_ << std::endl;
                break;
            case '\x03':
                return;
            default:
                std::cout << "Invalid key pressed!" << std::endl;
                break;
            }
        }
    }

private:
    void print_instructions()
    {
        const char *msg = R"(
Reading from the keyboard!
---------------------------
Movement:
  W : All stern thrusters forward
  S : All stern thrusters backward
  D : Bow star thruster forward, bow port thruster backward
  A : Bow star thruster backward, bow port thruster forward

q : Increase thrust speed
z : Decrease thrust speed

NOTE : Increasing or Decreasing will take effect live on the moving robot.
Consider stopping the robot before changing it.
CTRL-C to quit
)";
        std::cout << msg << std::endl;
    }

    void adjust_stern_thrust(double thrust)
    {
        thrust_stern_port1_ = thrust;
        thrust_stern_port2_ = thrust;
        thrust_stern_star1_ = thrust;
        thrust_stern_star2_ = thrust;
        publish_stern_thrust();
        std::cout << "Stern thrusters set to: " << thrust << std::endl;
    }

    void adjust_bow_thrust(double thrust_bow_star, double thrust_bow_port)
    {
        thrust_bow_star_ = thrust_bow_star;
        thrust_bow_port_ = thrust_bow_port;
        publish_bow_thrust();
        std::cout << "Bow star thruster set to: " << thrust_bow_star << ", Bow port thruster set to: " << thrust_bow_port << std::endl;
    }

    void publish_stern_thrust()
    {
        auto message_stern_port1 = std_msgs::msg::Float64();
        auto message_stern_port2 = std_msgs::msg::Float64();
        auto message_stern_star1 = std_msgs::msg::Float64();
        auto message_stern_star2 = std_msgs::msg::Float64();

        message_stern_port1.data = thrust_stern_port1_;
        message_stern_port2.data = thrust_stern_port2_;
        message_stern_star1.data = thrust_stern_star1_;
        message_stern_star2.data = thrust_stern_star2_;

        publisher_thrust_stern_port1_->publish(message_stern_port1);
        publisher_thrust_stern_port2_->publish(message_stern_port2);
        publisher_thrust_stern_star1_->publish(message_stern_star1);
        publisher_thrust_stern_star2_->publish(message_stern_star2);
    }

    void publish_bow_thrust()
    {
        auto message_bow_port = std_msgs::msg::Float64();
        auto message_bow_star = std_msgs::msg::Float64();

        message_bow_port.data = thrust_bow_port_;
        message_bow_star.data = thrust_bow_star_;

        publisher_thrust_bow_port_->publish(message_bow_port);
        publisher_thrust_bow_star_->publish(message_bow_star);
    }

    int getch()
    {
        int ch;
        struct termios oldt;
        struct termios newt;

        // Store old settings, and copy to new settings
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        // Make required changes and apply the settings
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_iflag |= IGNBRK;
        newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
        newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
        newt.c_cc[VMIN] = 1;
        newt.c_cc[VTIME] = 0;
        tcsetattr(fileno(stdin), TCSANOW, &newt);

        // Get the current character
        ch = getchar();

        // Reapply old settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        return ch;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_thrust_bow_port_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_thrust_bow_star_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_thrust_stern_port1_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_thrust_stern_port2_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_thrust_stern_star1_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_thrust_stern_star2_;
    double thrust_speed_;
    double thrust_bow_port_;
    double thrust_bow_star_;
    double thrust_stern_port1_;
    double thrust_stern_port2_;
    double thrust_stern_star1_;
    double thrust_stern_star2_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopUSV>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
