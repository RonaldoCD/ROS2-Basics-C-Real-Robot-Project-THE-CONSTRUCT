#include <cstddef>
#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include <iostream>
#include <vector>


#include "custom_interfaces/action/detail/odom_record__struct.hpp"
#include "geometry_msgs/msg/detail/point32__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/detail/float64__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "custom_interfaces/action/odom_record.hpp"
#include "geometry_msgs/msg/point32.h"

using namespace std;

class OdomRecorderServer : public rclcpp::Node
{
    public:
    using OdomRecord = custom_interfaces::action::OdomRecord;
    using GoalHandleOdomRecord = rclcpp_action::ServerGoalHandle<OdomRecord>;

    explicit OdomRecorderServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("odom_recorder_server", options)
    {
        using namespace std::placeholders;

        callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions options1;
        options1.callback_group = callback_group;

        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&OdomRecorderServer::odom_callback, this, _1), options1);

        this->action_server_ = rclcpp_action::create_server<OdomRecord>(
            this,
            "record_odom",
            std::bind(&OdomRecorderServer::handle_goal, this, _1, _2),
            std::bind(&OdomRecorderServer::handle_cancel, this, _1),
            std::bind(&OdomRecorderServer::handle_accepted, this, _1),
            rcl_action_server_get_default_options(),
            callback_group);


        start_point = geometry_msgs::msg::Point32();
        current_point = geometry_msgs::msg::Point32();
        
        total_distance = 0.0;
        lap_finished = false;
        start_point_identified = false;
        distance_to_start = 0.0;
        check_completed_lap_flag = false;
    }

    private:
    rclcpp_action::Server<OdomRecord>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    geometry_msgs::msg::Point32 start_point;
    geometry_msgs::msg::Point32 current_point;
    float total_distance;
    float distance_to_start;
    bool lap_finished;
    bool start_point_identified;
    bool check_completed_lap_flag;

    rclcpp::CallbackGroup::SharedPtr callback_group;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const OdomRecord::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request YEAHH");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleOdomRecord> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleOdomRecord> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&OdomRecorderServer::execute, this, _1), goal_handle}.detach();
    }

    void check_completed_lap(){
        this->distance_to_start = sqrt(pow(this->current_point.x - this->start_point.x, 2) + pow(this->current_point.y - this->start_point.y, 2));
        if (this->check_completed_lap_flag && this->distance_to_start <= 0.1){
            this->lap_finished = true;
        }        
    }

    void print_vector_of_points(vector<geometry_msgs::msg::Point32> vector_of_points){
        for (size_t i = 0; i < vector_of_points.size(); i++){
            RCLCPP_INFO(this->get_logger(), "Point '%d' = ('%f', '%f')", i+1, vector_of_points[i].x, vector_of_points[i].y);
        }
    }

    void execute(const std::shared_ptr<GoalHandleOdomRecord> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<OdomRecord::Feedback>();
        auto & current_distance_feedback = feedback->current_total;
        // message = "Starting movement...";
        auto result = std::make_shared<OdomRecord::Result>();
        rclcpp::Rate loop_rate(1);
        float delta_x, delta_y, distance_to_last_point;

        vector<geometry_msgs::msg::Point32> vector_of_odoms = {};
        while (!this->start_point_identified){
            loop_rate.sleep();
            RCLCPP_INFO(this->get_logger(), "Waiting for start point to be identified");
        }
        vector_of_odoms.push_back(this->start_point);
        int N = 1;
        
        while (!this->lap_finished && rclcpp::ok()){
            if (goal_handle->is_canceling()) {
                // geometry_msgs::msg::Point32 list_of_odoms[vector_of_odoms.size()];
                // copy(vector_of_odoms.begin(), vector_of_odoms.end(), list_of_odoms);
                result->list_of_odoms = vector_of_odoms;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            loop_rate.sleep();
            RCLCPP_INFO(this->get_logger(), "Distance to start point: '%f'", this->distance_to_start);
            vector_of_odoms.push_back(this->current_point);
            N += 1;
            delta_x = vector_of_odoms[N-2].x - vector_of_odoms[N-1].x;
            delta_y = vector_of_odoms[N-2].y - vector_of_odoms[N-1].y;
            distance_to_last_point = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
            this->total_distance += distance_to_last_point;
            
            current_distance_feedback = this->total_distance;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback - Current total distance = '%f'", this->total_distance);
            if (N > 5){
                this->check_completed_lap_flag = true;
            }
        }

        // Check if goal is done
        if (rclcpp::ok()) {
            result->list_of_odoms = vector_of_odoms;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded, the points registered during the lap are:");
            print_vector_of_points(vector_of_odoms);
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "ODOM topic");
        if (!this->start_point_identified){
            this->start_point.x = msg->pose.pose.position.x;
            this->start_point.y = msg->pose.pose.position.y;
            this->start_point_identified = true;
            // RCLCPP_INFO(this->get_logger(), "IF");
        }
        else {
            this->current_point = geometry_msgs::msg::Point32();
            this->current_point.x = msg->pose.pose.position.x;
            this->current_point.y = msg->pose.pose.position.y;
            // RCLCPP_INFO(this->get_logger(), "ELSE");
            check_completed_lap();
        }
    }
};  // class MyActionServer

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto action_server = std::make_shared<OdomRecorderServer>();
        
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_server);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}