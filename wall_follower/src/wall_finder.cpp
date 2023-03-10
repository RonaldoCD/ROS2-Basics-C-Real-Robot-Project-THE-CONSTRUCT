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

class WallFinderNode : public rclcpp::Node
{
    public:
    WallFinderNode()
    : Node("find_wall_node")
    {
        callback_group =this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        odom_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options1;
        options1.callback_group = odom_callback_group;

        scan_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options2;
        options2.callback_group = scan_callback_group;

        find_wall_srv = create_service<FindWall>("find_wall", std::bind(&WallFinderNode::wall_finder_srv_callback, this, _1, _2), rmw_qos_profile_services_default, callback_group);
        twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_subscriber = this->create_subscription<Odom>("odom", 10, std::bind(&WallFinderNode::odom_callback, this, _1), options1);
        scan_subscriber = this->create_subscription<Scan>("scan", 10, std::bind(&WallFinderNode::scan_callback, this, _1), options2);
        angle_z = 0.0;
        nearest_wall_identified = false;
        angle_to_rotate = 0.0;
        dist_to_wall = 0.0;
        angle_tol = 5 * M_PI / 180;
        dist_tol = 0.03;
        angular_z = 0.1;
        linear_x = 0.08;
    }

    private:
    rclcpp::Service<FindWall>::SharedPtr find_wall_srv;
    rclcpp::Publisher<Twist>::SharedPtr twist_publisher;
    rclcpp::Subscription<Odom>::SharedPtr odom_subscriber;
    rclcpp::Subscription<Scan>::SharedPtr scan_subscriber;

    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::CallbackGroup::SharedPtr odom_callback_group;
    rclcpp::CallbackGroup::SharedPtr scan_callback_group;

    float angle_z;
    bool nearest_wall_identified;
    float angle_to_rotate;
    float dist_to_wall;
    float angle_tol;
    float dist_tol;
    float angular_z;
    float linear_x;

    void wall_finder_srv_callback(const std::shared_ptr<FindWall::Request> request,
                                  const std::shared_ptr<FindWall::Response> response)
    {
        auto message = Twist();
        float robot_initial_angle = this->angle_z;
        float new_angle = fmod(robot_initial_angle + this->angle_to_rotate, 2 * M_PI);
        int quadrant_idx = round(new_angle/(M_PI/2));
        float objective_angle = fmod((quadrant_idx * M_PI / 2), (2 * M_PI));

        RCLCPP_INFO(this->get_logger(), "Initial yaw angle: '%f'", robot_initial_angle * 180 / M_PI);
        RCLCPP_INFO(this->get_logger(), "Objective angle: '%f'", objective_angle * 180 / M_PI);
        
        while (!this->nearest_wall_identified){
            RCLCPP_INFO(this->get_logger(), "WAITING FOR IDENTIFYING THE NEAREST WALL");
            sleep(1);
        }

        while (fmod(abs(this->angle_z - objective_angle), 2*M_PI) > this->angle_tol){
            if (this->angle_to_rotate > 0){
                message.angular.z = this->angular_z;
            }
            else if(this->angle_to_rotate < 0){
                message.angular.z = - this->angular_z;
            }
            twist_publisher->publish(message);
        }

        int linear_x_sign = 1;
        if (this->dist_to_wall < 0.3){
            linear_x_sign = -1;
        }
        while(abs(this->dist_to_wall - 0.3) > this->dist_tol){
            message.linear.x = linear_x_sign * this->linear_x;
            message.angular.z = 0.0;
            twist_publisher->publish(message);
        }

        float final_objective_angle = fmod((objective_angle + (M_PI / 2.0)), (2 * M_PI));
        RCLCPP_INFO(this->get_logger(), "Last objective angle: '%f'", final_objective_angle * 180 / M_PI);
        while (fmod(abs(this->angle_z - final_objective_angle), 2*M_PI) > this->angle_tol){
            message.linear.x = 0.0;
            message.angular.z = this->angular_z;
            twist_publisher->publish(message);
        }
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        twist_publisher->publish(message);

        response->wallfound = true;
    }

    
    void scan_callback(const Scan::SharedPtr msg)
    {
        if (!this->nearest_wall_identified){

            int N_ranges = round(1.0 + (msg->angle_max - msg->angle_min) / msg->angle_increment);
            RCLCPP_INFO(this->get_logger(), "N ranges 2: '%d'", N_ranges);
            
            vector<float> ranges;
            copy(begin(msg->ranges), end(msg->ranges), back_inserter(ranges));
            // ranges.insert(ranges.begin(), msg->ranges, msg->ranges + N_ranges);
            vector<float>::iterator min_range_idx_iter = min_element(ranges.begin(), ranges.end());
            int min_range_idx = distance(ranges.begin(), min_range_idx_iter);
            RCLCPP_INFO(this->get_logger(), "Min range idx: '%d'", min_range_idx);
            float min_range_angle = (2 * M_PI * (min_range_idx + 1)) / N_ranges;

            for (int i=0; i<10; i++){
                cout << ranges[i] << " ";
            }
            cout << "\n";
            for (int i=0; i<10; i++){
                cout << msg->ranges[i] << " ";
            }

            this->angle_to_rotate = min_range_angle - M_PI;
            RCLCPP_INFO(this->get_logger(), "Angle to rotate: '%f'", this->angle_to_rotate * 180 / M_PI);
            this->nearest_wall_identified = true;
        }
        this->dist_to_wall = msg->ranges[360];
    }

    void odom_callback(const Odom::SharedPtr msg)
    {
        float q_x, q_y, q_z, q_w;
        q_x = msg->pose.pose.orientation.x;
        q_y = msg->pose.pose.orientation.y;
        q_z = msg->pose.pose.orientation.z;
        q_w = msg->pose.pose.orientation.w;

        vector<float> euler_angles = euler_from_quaternion(q_x, q_y, q_z, q_w);
        float yaw = euler_angles[2];
        // RCLCPP_INFO(this->get_logger(), "Yaw: '%f'", yaw * 180 / M_PI);
        
        if (yaw < 0){
            float modulo = M_PI + yaw;
            yaw = M_PI + modulo;
        }
        this->angle_z = yaw;

        // RCLCPP_INFO(this->get_logger(), "Angle direction with respect to Z: '%f'", angle_z * 180 / M_PI);
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