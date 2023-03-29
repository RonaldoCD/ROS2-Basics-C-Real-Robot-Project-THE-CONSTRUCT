
#include "custom_interfaces/action/detail/odom_record__struct.hpp"
#include "custom_interfaces/srv/detail/find_wall__struct.hpp"
#include "rclcpp/logging.hpp"
#define _USE_MATH_DEFINES
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "custom_interfaces/srv/find_wall.hpp"
#include "custom_interfaces/action/odom_record.hpp"

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
    using OdomRecord = custom_interfaces::action::OdomRecord;
    using GoalHandleOdomRecord = rclcpp_action::ClientGoalHandle<OdomRecord>;

    explicit WallFollower(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("wall_follower", node_options), goal_done_(false)
    {
        RCLCPP_INFO(this->get_logger(), "WALL FOLLOWER NODE HAS INITIALIZED");
        subscribers_callback_group =this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        publishers_callback_group =this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        action_client_callback_group =this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        
        rclcpp::SubscriptionOptions options1;
        options1.callback_group = subscribers_callback_group;

        rclcpp::SubscriptionOptions options2;
        options2.callback_group = subscribers_callback_group;

        vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&WallFollower::odom_subs_callback, this, _1), options1);
        scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(), std::bind(&WallFollower::scan_subs_callback, this, _1), options2);
        timer_ = this->create_wall_timer(500ms, std::bind(&WallFollower::robot_motion, this), publishers_callback_group);
        print_timer = this->create_wall_timer(2000ms, std::bind(&WallFollower::print_function, this), publishers_callback_group);
        
        this->client_ptr_ = rclcpp_action::create_client<OdomRecord>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "record_odom");

        this->action_client_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&WallFollower::send_goal, this), action_client_callback_group);

        this->rotation_z = 0.0;
        this->distance_to_wall = 0.0;
        this->distance_to_front_wall= 0.0;
        this->offset_angle = 0.0;
        this->linear_vel = 0.08;
        this->angular_vel = 0.1;
        this->dist_tol = 0.02;
        this->angle_tol = 3.0*M_PI/180;
        this->timer_period = 0.5;
    }

    bool is_goal_done() const
    {
        return this->goal_done_;
    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->action_client_timer_->cancel();

        this->goal_done_ = false;

        if (!this->client_ptr_) {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        }

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            this->goal_done_ = true;
            return;
        }

        auto goal_msg = OdomRecord::Goal();

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<OdomRecord>::SendGoalOptions();
                    
        send_goal_options.goal_response_callback = 
            std::bind(&WallFollower::goal_response_callback, this, _1);

        send_goal_options.feedback_callback =
            std::bind(&WallFollower::feedback_callback, this, _1, _2);

        send_goal_options.result_callback =
            std::bind(&WallFollower::result_callback, this, _1);
        
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
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
        if(!this->goal_done_){
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
        }
        else {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;     
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

    // Functions of the action client 

    void goal_response_callback(std::shared_future<GoalHandleOdomRecord::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleOdomRecord::SharedPtr,
        const std::shared_ptr<const OdomRecord::Feedback> feedback)
    {
        RCLCPP_INFO(
        this->get_logger(), "Feedback received: Total current distance = %f", feedback->current_total);
    }

    void result_callback(const GoalHandleOdomRecord::WrappedResult & result)
    {
        this->goal_done_ = true;
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "RESULT RECEIVED");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr print_timer;
    rclcpp::TimerBase::SharedPtr action_client_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp_action::Client<OdomRecord>::SharedPtr client_ptr_;
    
    rclcpp::CallbackGroup::SharedPtr subscribers_callback_group;
    rclcpp::CallbackGroup::SharedPtr publishers_callback_group;
    rclcpp::CallbackGroup::SharedPtr action_client_callback_group;
    
    float rotation_z;
    float distance_to_wall;
    float distance_to_front_wall;
    float offset_angle;
    float linear_vel;
    float angular_vel;
    float dist_tol;
    float angle_tol;
    float timer_period;
    bool goal_done_;
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Find wall service has finished");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /find_wall");
    }

    
    // rclcpp::shutdown();
    // rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SLEEP");
    loop_rate.sleep();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AWAKE");

    auto wall_follower_node = std::make_shared<WallFollower>();        
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(wall_follower_node);

    while (!wall_follower_node->is_goal_done()) {
        executor.spin();
    }

    rclcpp::shutdown();

    return 0;
}
