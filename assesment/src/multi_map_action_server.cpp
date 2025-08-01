#include "assesment/multi_map_action_server.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "assesment/database_manager.hpp"
#include <chrono>
#include <std_msgs/msg/float64.hpp>
#include <thread>

using NavigateMultiMap = wormhole_action::action::NavigateMultiMap;

using namespace std::chrono_literals;

MultiMapActionServer::MultiMapActionServer()
: Node("multi_map_action_server") {
    std::cout << "Entered MultiMapActionServer constructor..." << std::endl;
    RCLCPP_INFO(this->get_logger(), "Initializing MultiMapActionServer...");
    auto connection_str = this->declare_parameter<std::string>("database_connection", "host=localhost dbname=wormhole_database user=headuser password=sqlpass123");
    db_manager_ = std::make_shared<DatabaseManager>(connection_str);
    db_manager_->connect();
    std::cout << "DatabaseManager connected." << std::endl;

    map_loader_client_ = this->create_client<nav_msgs::srv::LoadMap>("/map_server/load_map");


    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");
    
    RCLCPP_INFO(this->get_logger(), "Now Serious, OK?");

    if (!nav2_client_->wait_for_action_server(5s)) {
        RCLCPP_WARN(this->get_logger(), "Nav2 action server not available after 5 seconds.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Nav2 action server ready.");
    }

    action_server_ = rclcpp_action::create_server<NavigateMultiMap>(
        this,
        "navigate_multi_map",
        std::bind(&MultiMapActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MultiMapActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&MultiMapActionServer::handle_accepted, this, std::placeholders::_1)
    );

    // Initialize publisher here once
    init_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    // Optionally initialize current_map_name_ if needed
    current_map_name_ = "";

    RCLCPP_INFO(this->get_logger(), "MultiMapActionServer initialized.");
}

rclcpp_action::GoalResponse MultiMapActionServer::handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateMultiMap::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal to map: %s", goal->target_map_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MultiMapActionServer::handle_cancel(
    [[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateMultiMap>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MultiMapActionServer::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateMultiMap>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Goal accepted, executing.");
    std::thread{std::bind(&MultiMapActionServer::execute, this, goal_handle)}.detach();
}

void MultiMapActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateMultiMap>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal.");
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<wormhole_action::action::NavigateMultiMap>> goal_handle_ptr = goal_handle;
    
    const auto goal = goal_handle_ptr->get_goal();
    auto feedback = std::make_shared<NavigateMultiMap::Feedback>();
    auto result = std::make_shared<NavigateMultiMap::Result>();

    if(goal->target_map_name == current_map_name_) {
        feedback->status = "Already on the target map.";
        goal_handle_ptr->publish_feedback(feedback);
        bool nav_ok = navigateTo(goal->target_pose);
        result->success = nav_ok;
        result->message = nav_ok ? "Navigation successful." : "Navigation failed.";
        nav_ok ? goal_handle_ptr->succeed(result) : goal_handle_ptr->abort(result);
        return;
    }

    auto wormholes = db_manager_->getWormholes(current_map_name_, goal->target_map_name);
    if (wormholes.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No wormholes found from %s to %s", current_map_name_.c_str(), goal->target_map_name.c_str());
        result->success = false;
        result->message = "No wormholes found to target map.";
        goal_handle_ptr->abort(result);
        return;
    }

    WormholeData wh = wormholes[0];
    geometry_msgs::msg::PoseStamped wh_pose;
    wh_pose.header.frame_id = "map";
    wh_pose.pose.position.x = wh.map1_x;
    wh_pose.pose.position.y = wh.map1_y;
    wh_pose.pose.orientation.z = wh.map1_oz;
    wh_pose.pose.orientation.w = wh.map1_ow;

    wh_pose.header.stamp = this->now();
    
    feedback->status = "Navigating to wormhole in " + current_map_name_;
    goal_handle_ptr->publish_feedback(feedback);
    if(!navigateTo(wh_pose)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to navigate to wormhole.");
        result->success = false;
        result->message = "Failed to navigate to wormhole.";
        goal_handle_ptr->abort(result);
        return;
    }
    
    feedback->status = "Wormhole reached, switching maps.";
    goal_handle_ptr->publish_feedback(feedback);
    if(!switchMap(goal->target_map_name, wh)){
        result->success = false;
        result->message = "Map switch failed.";
        goal_handle_ptr->abort(result);
        return;
    }
    
    feedback->status = "Navigating to target pose in " + goal->target_map_name;
    goal_handle_ptr->publish_feedback(feedback);
    bool final_nav = navigateTo(goal->target_pose);
    if (!final_nav) {
        RCLCPP_ERROR(this->get_logger(), "Failed to navigate to target pose.");
        result->success = false;
        result->message = "Failed to navigate to target pose.";
        goal_handle_ptr->abort(result);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Navigation to target pose successful.");
    result->success = true;
    result->message = "Navigation to target pose successful.";
    final_nav ? goal_handle_ptr->succeed(result) : goal_handle_ptr->abort(result);
}

bool MultiMapActionServer::navigateTo(const geometry_msgs::msg::PoseStamped& pose) {
    RCLCPP_INFO(this->get_logger(), "Navigating to pose: (%.2f, %.2f)", pose.pose.position.x, pose.pose.position.y);
    if (!nav2_client_) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 client not initialized.");
        return false;
    }

    if (!nav2_client_->wait_for_action_server(5s)) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 action server unavailable.");
        return false;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = pose;

    auto future_result = nav2_client_->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, 60s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send goal to Nav2.");
        return false;
    }

    auto result = nav2_client_->async_get_result(future_result.get());
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, 120s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Navigation execution failed.");
        return false;
    }

    // Check if the result is returned in this method only or not
    if (result.get().code == rclcpp_action::ResultCode::SUCCEEDED) {
        return true;
    } else {
        return false;
    }
}

bool MultiMapActionServer::switchMap(const std::string& new_map, const WormholeData& wh) {
    RCLCPP_INFO(this->get_logger(), "Switching to map: %s", new_map.c_str());
    if (!map_loader_client_) {
        RCLCPP_ERROR(this->get_logger(), "Map loader client not initialized.");
        return false;
    }
    if (!map_loader_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(this->get_logger(), "Map load service unavailable.");
        return false;
    }

    auto request = std::make_shared<nav_msgs::srv::LoadMap::Request>();
    request->map_url = "/maps/" + new_map + ".yaml";

    auto future = map_loader_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 10s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Map load failed.");
        return false;
    }

    // auto init_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
    init_pose.header.stamp = this->now();
    init_pose.header.frame_id = "map";
    init_pose.pose.pose.position.x = wh.map2_x;
    init_pose.pose.pose.position.y = wh.map2_y;
    init_pose.pose.pose.orientation.z = wh.map2_oz;
    init_pose.pose.pose.orientation.w = wh.map2_ow;

    for (int i = 0; i < 5; ++i) {
        init_pose_pub->publish(init_pose);
        rclcpp::sleep_for(500ms);
    }

    current_map_name_ = new_map;
    return true;
}

int main(int argc, char * argv[])
{
    std::cout << "Starting MultiMapActionServer..." << std::endl;
    rclcpp::init(argc, argv);
    std::cout << "RCLCPP initialized." << std::endl;
    auto node = std::make_shared<MultiMapActionServer>();
    std::cout << "MultiMapActionServer node created." << std::endl;
    // RCLCPP_INFO(rclcpp::get_logger("MultiMapActionServer"), "Initializing MultiMapActionServer...");
    std::cout << "MultiMapActionServer initialized. Will Spin Now" << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}