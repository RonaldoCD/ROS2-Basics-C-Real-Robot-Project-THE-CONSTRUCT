#include "custom_interfaces/srv/detail/find_wall__struct.hpp"
#include "rclcpp/logging.hpp"
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
#include <memory>

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;


vector<float> euler_from_quaternion(float x, float y, float z, float w){
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    float roll = atan2(sinr_cosp, cosr_cosp);
    
    float sinp = 2 * (w * y - z * x);
    float pitch = asin(sinp);

    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    float yaw = atan2(siny_cosp, cosy_cosp);

    vector<float> euler_angles(3, 0.0);
    euler_angles[0] = roll;
    euler_angles[1] = pitch;
    euler_angles[2] = yaw;

    return euler_angles;
}

class WallFollower : public rclcpp::Node
{
public:
    WallFollower()
    : Node("topics_project")
    {
        vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&WallFollower::odom_subs_callback, this, _1));
        scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind(&WallFollower::scan_subs_callback, this, _1));
        timer_ = this->create_wall_timer(
        500ms, std::bind(&WallFollower::robot_motion, this));
        print_timer = this->create_wall_timer(
        2000ms, std::bind(&WallFollower::print_function, this));
        
        this->rotation_z = 0.0;
        this->distance_to_wall = 0.0;
        this->distance_to_front_wall= 0.0;
        this->offset_angle = 0.0;
        this->linear_vel = 0.1;
        this->angular_vel = 0.1;
        this->dist_tol = 0.02;
        this->angle_tol = 3.0*M_PI/180;
        this->timer_period = 0.5;

    }

private:
    
    void print_function(){
        RCLCPP_INFO(this->get_logger(), "--------------");
        RCLCPP_INFO(this->get_logger(), "Rotation z = '%f'", this->rotation_z * 180 / M_PI);
        RCLCPP_INFO(this->get_logger(), "Distance to wall = '%f'", this->distance_to_wall);
        RCLCPP_INFO(this->get_logger(), "Distance to front wall = '%f'", this->distance_to_front_wall);
        RCLCPP_INFO(this->get_logger(), "Offset angle = '%f'", this->offset_angle * 180 / M_PI);
        RCLCPP_INFO(this->get_logger(), "--------------");
    }
    void scan_subs_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float offset_angle = fmod(this->rotation_z, (M_PI/2.0));
        if (offset_angle >= M_PI/4 && offset_angle < M_PI/2){
            offset_angle -= M_PI / 2;
        }
        
        this->offset_angle = offset_angle;
        float range_angle = (3 * M_PI / 2) - offset_angle - M_PI;
        int ranges_length = msg->ranges.size();
        int range_idx = range_angle * ranges_length / (2 * M_PI);
        this->distance_to_wall = msg->ranges[range_idx];
        this->distance_to_front_wall = msg->ranges[ranges_length/2];
    }

    void odom_subs_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {   
        float q_x = msg->pose.pose.orientation.x;
        float q_y = msg->pose.pose.orientation.y;
        float q_z = msg->pose.pose.orientation.z;
        float q_w = msg->pose.pose.orientation.w;

        vector<float> euler_angles = euler_from_quaternion(q_x, q_y, q_z, q_w);
        float yaw = euler_angles[2];
        
        if (yaw < 0){
            float modulo = M_PI + yaw;
            yaw = M_PI + modulo;
        }

        this->rotation_z = yaw;
        
        // RCLCPP_INFO(this->get_logger(), "[LEFT] = '%f'", this->laser_left);
        // RCLCPP_INFO(this->get_logger(), "[RIGHT] = '%f'", this->laser_right);
        // RCLCPP_INFO(this->get_logger(), "[FORWARD] = '%f'", this->laser_forward);
        
    }

    void robot_motion()
    {
        geometry_msgs::msg::Twist msg;
        if (this->distance_to_front_wall < 0.5){
            msg.linear.x = this->linear_vel;
            msg.angular.z = 4.5 * this->angular_vel;
            RCLCPP_WARN(this->get_logger(), "Wall in front");
        }
        else{
            if (this->distance_to_wall > 0.3){
                msg.linear.x = this->linear_vel;
                msg.angular.z = -1 * this->angular_vel * 1.4;
                RCLCPP_WARN(this->get_logger(), "Going to the right wall");
            }
            else if (this->distance_to_wall < 0.2) {
                msg.linear.x = this->linear_vel;
                msg.angular.z = this->angular_vel * 1.4;
                RCLCPP_WARN(this->get_logger(), "Going away from the right wall");
            }
            else {
                RCLCPP_INFO(this->get_logger(), "In the right area");
                vector<float> velocities = this->motion_to_center();
                msg.linear.x = velocities[0];
                msg.angular.z = velocities[1];
                
            }      
        }
        
        // RCLCPP_INFO(this->get_logger(), "(lin_x, ang_z) = ('%f', '%f')", msg.linear.x, msg.angular.z);
        vel_publisher->publish(msg);
    }

    vector<float> motion_to_center(){
        float linear_x, angular_z;
        float delta_dist_to_wall = this->distance_to_wall - 0.25;
        int turn_sign = 1;
        if (delta_dist_to_wall > 0){
            turn_sign = -1;
        }
        
        if (abs(delta_dist_to_wall) >= this->dist_tol){
            if ((delta_dist_to_wall > 0 && this->offset_angle > 0) || (delta_dist_to_wall<0 && this->offset_angle<0)){
                linear_x = this->linear_vel;
                angular_z = turn_sign * this->angular_vel;    
            // RCLCPP_INFO(this->get_logger(), "1. negative turn");
            }
            else if ((delta_dist_to_wall > 0 && this->offset_angle < 0)||(delta_dist_to_wall < 0 && this->offset_angle > 0)) {
                float dist_to_go = abs(delta_dist_to_wall) / sin(abs(this->offset_angle));
                if (dist_to_go / (this->timer_period * this->linear_vel) >= 1){
                    linear_x = this->linear_vel;
                    angular_z = (-0.5) * turn_sign * this->angular_vel; 
                }
                else{
                    linear_x = (dist_to_go / (this->timer_period * this->linear_vel)) * this->linear_vel;
                    angular_z = (-0.5) * turn_sign * this->offset_angle / this->timer_period;
                }
            }
                // RCLCPP_INFO(this->get_logger(), "2. normal velocities");          
                // RCLCPP_INFO(this->get_logger(), "2. calculate velocities"); 
        }
        else{
            linear_x = this->linear_vel;
            RCLCPP_INFO(this->get_logger(), "Inside distance tolerance");
            if (this->offset_angle >= this->angle_tol){
                linear_x = 0.5 * this->linear_vel;
                if (this->offset_angle > 0){
                    angular_z = (-1) * this->offset_angle / this->timer_period;
                }
                else if(this->offset_angle < 0){
                    angular_z = (+1) * this->offset_angle / this->timer_period;
                }
                else {
                    angular_z = 0.0;
                }
                // RCLCPP_INFO(this->get_logger(), "3. calculate angular velocity");
            }
            else {
                angular_z = 0.0;
            }
        }

        if (abs(angular_z) > this->angular_vel){
            if (angular_z < 0){
                angular_z = - this->angular_vel;
            } 
            else {
                angular_z = this->angular_vel;
            }            
        }
        vector<float> velocities(2, 0.0);
        velocities[0] = linear_x;
        velocities[1] = angular_z;
        return velocities;
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr print_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    float rotation_z;
    float distance_to_wall;
    float distance_to_front_wall;
    float offset_angle;
    float linear_vel;
    float angular_vel;
    float dist_tol;
    float angle_tol;
    float timer_period;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("find_wall_client");
    rclcpp::Client<custom_interfaces::srv::FindWall>::SharedPtr client =
        node->create_client<custom_interfaces::srv::FindWall>("find_wall");

    auto request = std::make_shared<custom_interfaces::srv::FindWall::Request>();

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    rclcpp::Rate loop_rate(1);
    loop_rate.sleep();

    auto result_future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = result_future.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot is moving");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /moving");
    }

    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}

