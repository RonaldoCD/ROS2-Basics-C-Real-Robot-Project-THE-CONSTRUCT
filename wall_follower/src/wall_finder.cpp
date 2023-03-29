#include "rclcpp/utilities.hpp"
#include <iterator>
#define _USE_MATH_DEFINES
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "custom_interfaces/srv/find_wall.hpp"
#include <chrono>
#include <vector>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <memory>


using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using FindWall = custom_interfaces::srv::FindWall;
using Twist = geometry_msgs::msg::Twist;
using Scan = sensor_msgs::msg::LaserScan;
using Odom = nav_msgs::msg::Odometry;


class WallFinderNode : public rclcpp::Node
{
    public:
    WallFinderNode()
    : Node("find_wall_node")
    {
        callback_group =this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group;

        find_wall_srv = create_service<FindWall>("find_wall", std::bind(&WallFinderNode::wall_finder_srv_callback, this, _1, _2), rmw_qos_profile_services_default, callback_group);
        twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        scan_subscriber = this->create_subscription<Scan>("scan", 10, std::bind(&WallFinderNode::scan_callback, this, _1), options);
        
        set_initial_configuration = false;
        turn_sign = +1;
        laser_min_idx = 0;
        laser_min = -1.0;
        angle_to_rotate = 0.0;
        laser_front = 0.0;
        n_ranges = 0.0;
        front_laser_idx = 0;
        right_laser_idx = 0;
        
        dist_tol = 0.03;
        laser_idx_tol = 20;
        
        vel_angular_z = 0.2;
        vel_linear_x = 0.05;

        first_turn_finished = false;
        second_turn_finished = false;

    }

    private:
    rclcpp::Service<FindWall>::SharedPtr find_wall_srv;
    rclcpp::Publisher<Twist>::SharedPtr twist_publisher;
    rclcpp::Subscription<Scan>::SharedPtr scan_subscriber;

    rclcpp::CallbackGroup::SharedPtr callback_group;

    bool set_initial_configuration;
    int turn_sign;

    int laser_min_idx;
    float laser_min;
    float angle_to_rotate;
    float laser_front;
    int n_ranges;
    int front_laser_idx;
    int right_laser_idx;
        
    float dist_tol;
    int laser_idx_tol;
        
    float vel_angular_z;
    float vel_linear_x;

    bool first_turn_finished;
    bool second_turn_finished;


    void wall_finder_srv_callback(const std::shared_ptr<FindWall::Request> request,
                                  const std::shared_ptr<FindWall::Response> response)
    {
        auto message = Twist();
        rclcpp::Rate loop_rate(0.5);
        
        while (this->laser_min < 0){
            RCLCPP_INFO(this->get_logger(), "WAITING FOR IDENTIFYING THE NEAREST WALL");
        }

        while(abs(this->laser_min_idx - this->front_laser_idx) > this->laser_idx_tol){
            message.angular.z = this->turn_sign * this->vel_angular_z;
            twist_publisher->publish(message);
            RCLCPP_INFO(this->get_logger(), "FIRST ROTATION");
            loop_rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "First turn finished, the robot is looking the nearest wall");    
        this->first_turn_finished = true;
        message.angular.z = 0.0;
        twist_publisher->publish(message);
        
        int linear_x_sign = 1;
        if (this->laser_front < 0.3){
            linear_x_sign = -1;
        }
        while(abs(this->laser_front - 0.3) > this->dist_tol){
            message.linear.x = linear_x_sign * this->vel_linear_x;
            message.angular.z = 0.0;
            twist_publisher->publish(message);
            RCLCPP_INFO(this->get_logger(), "LINEAR MOTION");
            loop_rate.sleep();
        }
        
        message.linear.x = 0.0;
        twist_publisher->publish(message);
        RCLCPP_INFO(this->get_logger(), "The robot 30 cm away from the wall");

        while(abs(this->laser_min_idx - this->right_laser_idx) > this->laser_idx_tol){
            message.angular.z = this->vel_angular_z;
            twist_publisher->publish(message);
            RCLCPP_INFO(this->get_logger(), "SECOND ROTATION");
            loop_rate.sleep();
        }

        message.linear.x = 0.0;
        message.angular.z = 0.0;
        twist_publisher->publish(message);
        RCLCPP_INFO(this->get_logger(), "The robot is aligned with the right wall");
        this->second_turn_finished = true;

        response->wallfound = true;
    }

    
    void scan_callback(const Scan::SharedPtr msg)
    {
        if (!this->set_initial_configuration){
            this->set_initial_configuration = true;
            this->n_ranges = round(1.0 + (msg->angle_max - msg->angle_min) / msg->angle_increment);
            this->front_laser_idx = n_ranges / 2;
            this->right_laser_idx = n_ranges / 4;
            RCLCPP_INFO(this->get_logger(), "Front laser idx: '%d'", front_laser_idx);
            RCLCPP_INFO(this->get_logger(), "Right laser idx: '%d'", right_laser_idx);
        }
        
        vector<float> ranges;
        copy(begin(msg->ranges), end(msg->ranges), back_inserter(ranges));
        vector<float>::iterator min_range_idx_iter = min_element(ranges.begin(), ranges.end());
        this->laser_min_idx = distance(ranges.begin(), min_range_idx_iter);
        this->laser_min = ranges[this->laser_min_idx];

        float min_range_angle = (2 * M_PI * (this->laser_min_idx + 1)) / this->n_ranges;

        if (!this->first_turn_finished){
            this->angle_to_rotate = min_range_angle - M_PI;
            if (this->angle_to_rotate < 0){
                this->turn_sign = -1;
            }
            RCLCPP_INFO(this->get_logger(), "Angle to rotate: '%f'", this->angle_to_rotate * 180 / M_PI);
        }
        else if (!this->second_turn_finished){
            this->angle_to_rotate = min_range_angle - M_PI/2;
            RCLCPP_INFO(this->get_logger(), "Angle to rotate 2: '%f'", this->angle_to_rotate * 180 / M_PI);
        }    
        this->laser_front = msg->ranges[this->front_laser_idx];
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<WallFinderNode> wall_finder_node = std::make_shared<WallFinderNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(wall_finder_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}