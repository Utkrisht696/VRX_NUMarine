#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


int getch(void)
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

int main(int argc, char * argv[])
{
const char* msg = R"(
Reading from the keyboard!
---------------------------
Thrust Key Left:
    w    
        
    s    

Thrust Key Left:
    e    
        
    d    

w/s : increase/decrease Thrust Left Key by 0.1
w/s : increase/decrease Thrust Right Key by 0.1
NOTE : Increasing or Decreasing will take affect live on the moving robot.
    Consider Stopping the robot before changing it.
CTRL-C to quit
)";
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("usv_teleop");
    std::cout<<msg<<std::endl;

    auto publisher_thrust_left = node->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
    auto publisher_thrust_right = node->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10);


    auto message_thrust_left = std_msgs::msg::Float64();
    auto message_thrust_right = std_msgs::msg::Float64();

    message_thrust_left.data = 0;
    message_thrust_right.data = 0;

    while (rclcpp::ok())
    {
        char key = getch();

        if(key == 's' || key == 'S'){
        message_thrust_left.data =  message_thrust_left.data - 2.0;
        publisher_thrust_left->publish(message_thrust_left);
        std::cout<<"Thrust Value Left:- "<<message_thrust_left.data<<std::endl;
        }

        if(key == 'w' || key == 'W'){
        message_thrust_left.data =  message_thrust_left.data + 2.0;
        publisher_thrust_left->publish(message_thrust_left);
        std::cout<<"Thrust Value Left:- "<<message_thrust_left.data<<std::endl;
        }

        if(key == 'd' || key == 'D'){
        message_thrust_right.data =  message_thrust_right.data - 2.0;
        publisher_thrust_right->publish(message_thrust_right);
        std::cout<<"Thrust Value Right:- "<<message_thrust_right.data<<std::endl;
        }

        if(key == 'e' || key == 'E'){
        message_thrust_right.data =  message_thrust_right.data + 2.0;
        publisher_thrust_right->publish(message_thrust_right);
        std::cout<<"Thrust Value Right:- "<<message_thrust_right.data<<std::endl;
        }

        else if(key == '\x03'){
            break;
        }
    }
    

    rclcpp::shutdown();
    return 0;
}