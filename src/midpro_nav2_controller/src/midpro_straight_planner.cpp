// midpro_straight_planner.cpp
// Fallback global planner: a straight line from start to goal, ignoring the costmap.
// Restores the ROS1 midpro behaviour where the local planner (straight/side/front/line/detour)
// always received a target and resolved obstacles itself. Used by the BT only when the grid
// planner fails ("no valid path"), so FollowPath (midpro FSM) still runs.
#include <cmath>
#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace midpro_nav2_controller
{

class MidproStraightPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    name_ = name;
    frame_ = costmap_ros->getGlobalFrameID();
    nav2_util::declare_parameter_if_not_declared(node, name + ".interval", rclcpp::ParameterValue(0.05));
    node->get_parameter(name + ".interval", interval_);
    logger_ = node->get_logger();
    RCLCPP_INFO(logger_, "[%s] straight-line fallback planner configured (interval %.2f m)", name_.c_str(), interval_);
  }
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_;
    path.header.stamp = start.header.stamp;
    const double dx = goal.pose.position.x - start.pose.position.x;
    const double dy = goal.pose.position.y - start.pose.position.y;
    const double d = std::hypot(dx, dy);
    const int n = std::max(1, static_cast<int>(d / interval_));
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / n;
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = start.pose.position.x + t * dx;
      p.pose.position.y = start.pose.position.y + t * dy;
      p.pose.orientation = (i == n) ? goal.pose.orientation
        : nav2_util::geometry_utils::orientationAroundZAxis(std::atan2(dy, dx));
      path.poses.push_back(p);
    }
    RCLCPP_WARN(logger_, "[%s] grid planner failed; using a straight line (%.1f m) and letting the midpro FSM resolve obstacles",
      name_.c_str(), d);
    return path;
  }

private:
  std::string name_, frame_;
  double interval_{0.05};
  rclcpp::Logger logger_{rclcpp::get_logger("MidproStraightPlanner")};
};

}  // namespace midpro_nav2_controller

PLUGINLIB_EXPORT_CLASS(midpro_nav2_controller::MidproStraightPlanner, nav2_core::GlobalPlanner)
