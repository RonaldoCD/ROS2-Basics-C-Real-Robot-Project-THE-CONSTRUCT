#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class TopicsQuiz : public rclcpp::Node
{
public:
  TopicsQuiz()
  : Node("topics_project")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&TopicsQuiz::scan_callback, this, _1));
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TopicsQuiz::timer_callback, this));
    this->laser_left = 0.0;
    this->laser_right = 0.0;
    this->laser_forward = 0.0;

  }

private:
  geometry_msgs::msg::Twist twist; 

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {

    this->laser_left = msg->ranges[180];
    this->laser_right = msg->ranges[540];
    this->laser_forward = msg->ranges[360];

    RCLCPP_INFO(this->get_logger(), "[LEFT] = '%f'", this->laser_left);
    RCLCPP_INFO(this->get_logger(), "[RIGHT] = '%f'", this->laser_right);
    RCLCPP_INFO(this->get_logger(), "[FORWARD] = '%f'", this->laser_forward);
    
  }

  void timer_callback()
  {
    if (this->laser_forward <= 1.0 || this->laser_right <= 1.0) 
    {
        this->twist_msg.angular.z = 0.3;
        this->twist_msg.linear.x = 0.0;
    }            
    else if (this->laser_left <= 1.0)
    {
        this->twist_msg.angular.z = -0.3;
        this->twist_msg.linear.x = 0.0;
    }
    else
    {
        this->twist_msg.angular.z = 0.0;
        this->twist_msg.linear.x = 0.3;
    }

    publisher_->publish(this->twist_msg);  
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  geometry_msgs::msg::Twist twist_msg; 
  float laser_left;
  float laser_right;
  float laser_forward;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicsQuiz>());
  rclcpp::shutdown();
  return 0;
}