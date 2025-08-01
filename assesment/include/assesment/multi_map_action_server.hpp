#ifndef MULTI_MAP_ACTION_SERVER_HPP
#define MULTI_MAP_ACTION_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <nav_msgs/srv/load_map.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "assesment/database_manager.hpp"
#include "wormhole_action/action/navigate_multi_map.hpp"  // Auto-generated

class MultiMapActionServer : public rclcpp::Node {
public:
    MultiMapActionServer();

private:
    using NavigateMultiMap = wormhole_action::action::NavigateMultiMap;

    rclcpp_action::Server<NavigateMultiMap>::SharedPtr action_server_;
    std::shared_ptr<DatabaseManager> db_manager_;

    rclcpp::Client<nav_msgs::srv::LoadMap>::SharedPtr map_loader_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;

    // Add publisher as class member
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub;

    std::string current_map_name_;

    // Action handlers
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateMultiMap::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateMultiMap>> goal_handle);

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateMultiMap>> goal_handle);

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateMultiMap>> goal_handle);

    bool navigateTo(const geometry_msgs::msg::PoseStamped& pose);
    bool switchMap(const std::string& new_map, const WormholeData& wh);
};

#endif  // MULTI_MAP_ACTION_SERVER_HPP