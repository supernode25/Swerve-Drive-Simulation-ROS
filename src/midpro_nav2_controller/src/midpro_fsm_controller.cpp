// midpro_fsm_controller.cpp  (part 1: lifecycle, helpers, costmap)
#include "midpro_nav2_controller/midpro_fsm_controller.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace midpro_nav2_controller
{

using nav2_util::declare_parameter_if_not_declared;
using rclcpp::ParameterValue;

// ---------------------------------------------------------------- lifecycle
void MidproFsmController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("MidproFsmController: unable to lock node");
  }
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  costmap_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // ---- planner params (ex loas_local_planner / launch)
  declare_parameter_if_not_declared(node, name + ".near_obstacle_value", ParameterValue(95));
  declare_parameter_if_not_declared(node, name + ".near_obstacle_distance", ParameterValue(0.59));
  declare_parameter_if_not_declared(node, name + ".path_pose_interval", ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name + ".orient_mode", ParameterValue(std::string("robot")));
  declare_parameter_if_not_declared(node, name + ".initial_path_mode", ParameterValue(std::string("straight")));
  declare_parameter_if_not_declared(node, name + ".side_search_max", ParameterValue(5.0));
  declare_parameter_if_not_declared(node, name + ".side_search_step", ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name + ".detour_timeout", ParameterValue(5.0));
  declare_parameter_if_not_declared(node, name + ".detour_path_max_age", ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name + ".detour_goal_lookahead", ParameterValue(4.0));
  declare_parameter_if_not_declared(node, name + ".detour_source", ParameterValue(std::string("global_plan")));
  declare_parameter_if_not_declared(node, name + ".detour_check_dist", ParameterValue(2.0));
  declare_parameter_if_not_declared(node, name + ".speed_profile_enabled", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".lateral_acc_limit", ParameterValue(0.4));
  declare_parameter_if_not_declared(node, name + ".profile_curv_window", ParameterValue(0.3));
  declare_parameter_if_not_declared(node, name + ".profile_min_speed", ParameterValue(0.15));
  declare_parameter_if_not_declared(node, name + ".waypoint_simplify_tol", ParameterValue(0.10));
  declare_parameter_if_not_declared(node, name + ".waypoint_dist_tolerance", ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name + ".waypoint_yaw_tolerance", ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name + ".goal_orient_dist", ParameterValue(0.8));
  declare_parameter_if_not_declared(node, name + ".waypoint_orient_dist", ParameterValue(0.0));
  declare_parameter_if_not_declared(node, name + ".linear_acc_limit", ParameterValue(0.8));
  declare_parameter_if_not_declared(node, name + ".linear_dec_limit", ParameterValue(0.6));
  declare_parameter_if_not_declared(node, name + ".speed_lookahead_dist", ParameterValue(1.5));
  declare_parameter_if_not_declared(node, name + ".near_obstacle_speed", ParameterValue(0.3));
  declare_parameter_if_not_declared(node, name + ".inflation_speed", ParameterValue(0.6));
  declare_parameter_if_not_declared(node, name + ".curve_speed", ParameterValue(0.4));
  declare_parameter_if_not_declared(node, name + ".curve_angle", ParameterValue(0.17));
  declare_parameter_if_not_declared(node, name + ".governor_floor", ParameterValue(0.1));
  declare_parameter_if_not_declared(node, name + ".inflation_cost_threshold", ParameterValue(80));
  declare_parameter_if_not_declared(node, name + ".curve_lookahead", ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name + ".tracking_mode", ParameterValue(std::string("lookahead")));
  declare_parameter_if_not_declared(node, name + ".waypoint_mode", ParameterValue(std::string("final")));
  declare_parameter_if_not_declared(node, name + ".lookahead_min", ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name + ".lookahead_gain", ParameterValue(0.8));
  declare_parameter_if_not_declared(node, name + ".heading_gain", ParameterValue(1.5));
  declare_parameter_if_not_declared(node, name + ".rotate_threshold", ParameterValue(1.2));
  declare_parameter_if_not_declared(node, name + ".align_angle", ParameterValue(0.35));
  declare_parameter_if_not_declared(node, name + ".holonomic_policy", ParameterValue(std::string("goal_zone")));
  declare_parameter_if_not_declared(node, name + ".holonomic_zone", ParameterValue(1.5));
  declare_parameter_if_not_declared(node, name + ".steer_gain", ParameterValue(2.0));
  declare_parameter_if_not_declared(node, name + ".planner_divider", ParameterValue(2));
  declare_parameter_if_not_declared(node, name + ".oblique_yaw_tolerance", ParameterValue(1e-4));
  // ---- controller params (ex loas_controller)
  declare_parameter_if_not_declared(node, name + ".max_linear_vel", ParameterValue(0.2));
  declare_parameter_if_not_declared(node, name + ".min_linear_vel", ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name + ".max_angular_vel", ParameterValue(0.4));
  declare_parameter_if_not_declared(node, name + ".min_angular_vel", ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name + ".acceleration_limit", ParameterValue(0.1));
  declare_parameter_if_not_declared(node, name + ".angular_acc_limit", ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name + ".approach_dist", ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name + ".turn_angular_vel", ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name + ".dist_tolerance", ParameterValue(0.02));
  declare_parameter_if_not_declared(node, name + ".yaw_tolerance", ParameterValue(0.025));
  declare_parameter_if_not_declared(node, name + ".final_approach_threshold", ParameterValue(5));
  declare_parameter_if_not_declared(node, name + ".k_prop", ParameterValue(0.80));
  declare_parameter_if_not_declared(node, name + ".linear_y_lpf_alpha", ParameterValue(0.3));
  declare_parameter_if_not_declared(node, name + ".linear_x_rate_limit", ParameterValue(0.015));
  declare_parameter_if_not_declared(node, name + ".max_linear_y", ParameterValue(0.1));
  declare_parameter_if_not_declared(node, name + ".min_turn_speed_ratio", ParameterValue(0.67));
  declare_parameter_if_not_declared(node, name + ".heading_lookahead", ParameterValue(0.4));
  declare_parameter_if_not_declared(node, name + ".curvature_feedforward", ParameterValue(false));
  declare_parameter_if_not_declared(node, name + ".no_rotate_in_near", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".escape_speed", ParameterValue(0.25));
  declare_parameter_if_not_declared(node, name + ".holonomic_only_when_needed", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".goal_obstacle_check_radius", ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name + ".goal_side_angle", ParameterValue(0.52));
  declare_parameter_if_not_declared(node, name + ".side_parking_holonomic", ParameterValue(false));
  declare_parameter_if_not_declared(node, name + ".detour_clear_ticks", ParameterValue(5));
  declare_parameter_if_not_declared(node, name + ".max_path_deviation", ParameterValue(1.5));
  declare_parameter_if_not_declared(node, name + ".collision_model", ParameterValue(std::string("footprint")));
  declare_parameter_if_not_declared(node, name + ".footprint_padding", ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name + ".tight_passage_speed", ParameterValue(0.3));
  declare_parameter_if_not_declared(node, name + ".hybrid_fallback", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".align_gate_dist", ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name + ".align_tolerance", ParameterValue(0.12));
  declare_parameter_if_not_declared(node, name + ".plan_switch_gain", ParameterValue(0.15));
  declare_parameter_if_not_declared(node, name + ".plan_keep_check_dist", ParameterValue(3.0));
  declare_parameter_if_not_declared(node, name + ".plan_switch_small_gain", ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name + ".plan_switch_confirm", ParameterValue(2));
  declare_parameter_if_not_declared(node, name + ".traj_check_enabled", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".traj_check_time", ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name + ".align_move_speed", ParameterValue(0.2));
  declare_parameter_if_not_declared(node, name + ".track_lat_tol", ParameterValue(0.08));
  declare_parameter_if_not_declared(node, name + ".track_yaw_tol", ParameterValue(0.15));
  declare_parameter_if_not_declared(node, name + ".escape_latch", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".plan_hysteresis", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".rotation_check", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".align_gate_enabled", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".transit_holonomic", ParameterValue(false));
  declare_parameter_if_not_declared(node, name + ".plan_switching", ParameterValue(false));
  declare_parameter_if_not_declared(node, name + ".straight_clearance", ParameterValue(0.15));
  declare_parameter_if_not_declared(node, name + ".min_angular_in_place_only", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".steer_deadband", ParameterValue(0.01));
  declare_parameter_if_not_declared(node, name + ".goal_same_tol", ParameterValue(0.6));
  declare_parameter_if_not_declared(node, name + ".plan_switch_force_ratio", ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name + ".block_persist_time", ParameterValue(3.0));
  declare_parameter_if_not_declared(node, name + ".yield_stop_margin", ParameterValue(1.2));
  declare_parameter_if_not_declared(node, name + ".block_check_dist", ParameterValue(4.5));
  declare_parameter_if_not_declared(node, name + ".block_classify_time", ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name + ".block_move_tol", ParameterValue(0.20));
  declare_parameter_if_not_declared(node, name + ".back_off_speed", ParameterValue(0.12));
  declare_parameter_if_not_declared(node, name + ".shortcut_dist", ParameterValue(3.5));
  declare_parameter_if_not_declared(node, name + ".oblique_lookahead", ParameterValue(0.10));
  declare_parameter_if_not_declared(node, name + ".orthogonal_avoid", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".side_front_enabled", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".side_trigger_dist", ParameterValue(2.5));
  declare_parameter_if_not_declared(node, name + ".goal_staging", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".path_conditioner_enabled", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".conditioner_horizon", ParameterValue(25.0));
  declare_parameter_if_not_declared(node, name + ".tight_clearance", ParameterValue(0.06));
  declare_parameter_if_not_declared(node, name + ".trust_guard", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".tight_track_enabled", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".tight_track_lat_gain", ParameterValue(0.8));
  declare_parameter_if_not_declared(node, name + ".tight_track_lat_max", ParameterValue(0.08));
  declare_parameter_if_not_declared(node, name + ".tight_track_yaw_gain", ParameterValue(0.6));
  declare_parameter_if_not_declared(node, name + ".tight_track_yaw_max", ParameterValue(0.15));
  declare_parameter_if_not_declared(node, name + ".stage_eval_dist", ParameterValue(4.0));
  declare_parameter_if_not_declared(node, name + ".stage_min_dist", ParameterValue(0.6));
  declare_parameter_if_not_declared(node, name + ".stage_max_dist", ParameterValue(3.0));
  declare_parameter_if_not_declared(node, name + ".orth_trigger_dist", ParameterValue(1.2));
  declare_parameter_if_not_declared(node, name + ".orth_max_shift", ParameterValue(1.6));
  declare_parameter_if_not_declared(node, name + ".orth_speed", ParameterValue(0.25));
  declare_parameter_if_not_declared(node, name + ".prefer_comfortable_curve", ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".comfort_check_dist", ParameterValue(3.0));

  node->get_parameter(name + ".near_obstacle_value", near_obstacle_value_);
  node->get_parameter(name + ".near_obstacle_distance", near_obstacle_distance_);
  node->get_parameter(name + ".path_pose_interval", path_pose_interval_);
  node->get_parameter(name + ".orient_mode", orient_mode_);
  node->get_parameter(name + ".initial_path_mode", initial_path_mode_);
  node->get_parameter(name + ".side_search_max", side_search_max_);
  node->get_parameter(name + ".side_search_step", side_search_step_);
  node->get_parameter(name + ".detour_timeout", detour_timeout_);
  node->get_parameter(name + ".detour_path_max_age", detour_path_max_age_);
  node->get_parameter(name + ".detour_goal_lookahead", detour_goal_lookahead_);
  node->get_parameter(name + ".detour_source", detour_source_);
  node->get_parameter(name + ".detour_check_dist", detour_check_dist_);
  node->get_parameter(name + ".speed_profile_enabled", speed_profile_enabled_);
  node->get_parameter(name + ".lateral_acc_limit", lateral_acc_limit_);
  node->get_parameter(name + ".profile_curv_window", profile_curv_window_);
  node->get_parameter(name + ".profile_min_speed", profile_min_speed_);
  node->get_parameter(name + ".waypoint_simplify_tol", waypoint_simplify_tol_);
  node->get_parameter(name + ".waypoint_dist_tolerance", waypoint_dist_tolerance_);
  node->get_parameter(name + ".waypoint_yaw_tolerance", waypoint_yaw_tolerance_);
  node->get_parameter(name + ".goal_orient_dist", goal_orient_dist_);
  node->get_parameter(name + ".waypoint_orient_dist", waypoint_orient_dist_);
  node->get_parameter(name + ".linear_acc_limit", linear_acc_limit_);
  node->get_parameter(name + ".linear_dec_limit", linear_dec_limit_);
  node->get_parameter(name + ".speed_lookahead_dist", speed_lookahead_dist_);
  node->get_parameter(name + ".near_obstacle_speed", near_obstacle_speed_);
  node->get_parameter(name + ".inflation_speed", inflation_speed_);
  node->get_parameter(name + ".curve_speed", curve_speed_);
  node->get_parameter(name + ".curve_angle", curve_angle_);
  node->get_parameter(name + ".governor_floor", governor_floor_);
  node->get_parameter(name + ".inflation_cost_threshold", inflation_cost_threshold_);
  node->get_parameter(name + ".curve_lookahead", curve_lookahead_);
  node->get_parameter(name + ".tracking_mode", tracking_mode_);
  node->get_parameter(name + ".waypoint_mode", waypoint_mode_);
  node->get_parameter(name + ".lookahead_min", lookahead_min_);
  node->get_parameter(name + ".lookahead_gain", lookahead_gain_);
  node->get_parameter(name + ".heading_gain", heading_gain_);
  node->get_parameter(name + ".rotate_threshold", rotate_threshold_);
  node->get_parameter(name + ".align_angle", align_angle_);
  node->get_parameter(name + ".holonomic_policy", holonomic_policy_);
  node->get_parameter(name + ".holonomic_zone", holonomic_zone_);
  node->get_parameter(name + ".steer_gain", steer_gain_);
  node->get_parameter("controller_frequency", controller_frequency_);
  if (controller_frequency_ <= 0.0) {controller_frequency_ = 20.0;}
  node->get_parameter(name + ".planner_divider", planner_divider_);
  node->get_parameter(name + ".oblique_yaw_tolerance", oblique_yaw_tolerance_);
  node->get_parameter(name + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(name + ".min_linear_vel", min_linear_vel_);
  node->get_parameter(name + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(name + ".min_angular_vel", min_angular_vel_);
  node->get_parameter(name + ".acceleration_limit", acceleration_limit_);
  node->get_parameter(name + ".angular_acc_limit", angular_acc_limit_);
  node->get_parameter(name + ".approach_dist", approach_dist_);
  node->get_parameter(name + ".turn_angular_vel", turn_angular_vel_);
  node->get_parameter(name + ".dist_tolerance", dist_tolerance_);
  node->get_parameter(name + ".yaw_tolerance", yaw_tolerance_);
  node->get_parameter(name + ".final_approach_threshold", final_approach_threshold_);
  node->get_parameter(name + ".k_prop", k_prop_);
  node->get_parameter(name + ".linear_y_lpf_alpha", linear_y_lpf_alpha_);
  node->get_parameter(name + ".linear_x_rate_limit", linear_x_rate_limit_);
  node->get_parameter(name + ".max_linear_y", max_linear_y_);
  node->get_parameter(name + ".min_turn_speed_ratio", min_turn_speed_ratio_);
  node->get_parameter(name + ".heading_lookahead", heading_lookahead_);
  node->get_parameter(name + ".curvature_feedforward", curvature_feedforward_);
  node->get_parameter(name + ".no_rotate_in_near", no_rotate_in_near_);
  node->get_parameter(name + ".escape_speed", escape_speed_);
  node->get_parameter(name + ".holonomic_only_when_needed", holonomic_only_when_needed_);
  node->get_parameter(name + ".goal_obstacle_check_radius", goal_obstacle_check_radius_);
  node->get_parameter(name + ".goal_side_angle", goal_side_angle_);
  node->get_parameter(name + ".side_parking_holonomic", side_parking_holonomic_);
  node->get_parameter(name + ".detour_clear_ticks", detour_clear_ticks_);
  node->get_parameter(name + ".max_path_deviation", max_path_deviation_);
  node->get_parameter(name + ".collision_model", collision_model_);
  node->get_parameter(name + ".footprint_padding", footprint_padding_);
  node->get_parameter(name + ".tight_passage_speed", tight_passage_speed_);
  node->get_parameter(name + ".hybrid_fallback", hybrid_fallback_);
  node->get_parameter(name + ".align_gate_dist", align_gate_dist_);
  node->get_parameter(name + ".align_tolerance", align_tolerance_);
  node->get_parameter(name + ".plan_switch_gain", plan_switch_gain_);
  node->get_parameter(name + ".plan_keep_check_dist", plan_keep_check_dist_);
  node->get_parameter(name + ".plan_switch_small_gain", plan_switch_small_gain_);
  node->get_parameter(name + ".plan_switch_confirm", plan_switch_confirm_);
  node->get_parameter(name + ".traj_check_enabled", traj_check_enabled_);
  node->get_parameter(name + ".traj_check_time", traj_check_time_);
  node->get_parameter(name + ".align_move_speed", align_move_speed_);
  node->get_parameter(name + ".track_lat_tol", track_lat_tol_);
  node->get_parameter(name + ".track_yaw_tol", track_yaw_tol_);
  node->get_parameter(name + ".escape_latch", escape_latch_);
  node->get_parameter(name + ".plan_hysteresis", plan_hysteresis_);
  node->get_parameter(name + ".rotation_check", rotation_check_);
  node->get_parameter(name + ".align_gate_enabled", align_gate_enabled_);
  node->get_parameter(name + ".transit_holonomic", transit_holonomic_);
  node->get_parameter(name + ".plan_switching", plan_switching_);
  node->get_parameter(name + ".straight_clearance", straight_clearance_);
  node->get_parameter(name + ".min_angular_in_place_only", min_angular_in_place_only_);
  node->get_parameter(name + ".steer_deadband", steer_deadband_);
  node->get_parameter(name + ".goal_same_tol", goal_same_tol_);
  node->get_parameter(name + ".plan_switch_force_ratio", plan_switch_force_ratio_);
  node->get_parameter(name + ".block_persist_time", block_persist_time_);
  node->get_parameter(name + ".yield_stop_margin", yield_stop_margin_);
  node->get_parameter(name + ".block_check_dist", block_check_dist_);
  node->get_parameter(name + ".block_classify_time", block_classify_time_);
  node->get_parameter(name + ".block_move_tol", block_move_tol_);
  node->get_parameter(name + ".back_off_speed", back_off_speed_);
  node->get_parameter(name + ".shortcut_dist", shortcut_dist_);
  node->get_parameter(name + ".oblique_lookahead", oblique_lookahead_);
  node->get_parameter(name + ".orthogonal_avoid", orthogonal_avoid_);
  node->get_parameter(name + ".side_front_enabled", side_front_enabled_);
  node->get_parameter(name + ".side_trigger_dist", side_trigger_dist_);
  node->get_parameter(name + ".goal_staging", goal_staging_);
  node->get_parameter(name + ".path_conditioner_enabled", path_conditioner_enabled_);
  node->get_parameter(name + ".conditioner_horizon", conditioner_horizon_);
  {
    // the global costmap as published by planner_server (full map, obstacles included)
    auto lnode = std::dynamic_pointer_cast<nav2_util::LifecycleNode>(node);
    if (lnode) {
      global_cm_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(lnode, "/global_costmap/costmap_raw");
    } else {
      RCLCPP_WARN(logger_, "[%s] path conditioner disabled: parent is not a nav2_util::LifecycleNode", name.c_str());
    }
  }
  node->get_parameter(name + ".tight_clearance", tight_clearance_);
  node->get_parameter(name + ".trust_guard", trust_guard_);
  node->get_parameter(name + ".tight_track_enabled", tight_track_enabled_);
  node->get_parameter(name + ".tight_track_lat_gain", tight_track_lat_gain_);
  node->get_parameter(name + ".tight_track_lat_max", tight_track_lat_max_);
  node->get_parameter(name + ".tight_track_yaw_gain", tight_track_yaw_gain_);
  node->get_parameter(name + ".tight_track_yaw_max", tight_track_yaw_max_);
  guard_sub_ = node->create_subscription<std_msgs::msg::String>("/obstacle_guard/state", 10,
    std::bind(&MidproFsmController::onGuardState, this, std::placeholders::_1));
  node->get_parameter(name + ".stage_eval_dist", stage_eval_dist_);
  node->get_parameter(name + ".stage_min_dist", stage_min_dist_);
  node->get_parameter(name + ".stage_max_dist", stage_max_dist_);
  node->get_parameter(name + ".orth_trigger_dist", orth_trigger_dist_);
  node->get_parameter(name + ".orth_max_shift", orth_max_shift_);
  node->get_parameter(name + ".orth_speed", orth_speed_);
  node->get_parameter(name + ".prefer_comfortable_curve", prefer_comfortable_curve_);
  node->get_parameter(name + ".comfort_check_dist", comfort_check_dist_);
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&MidproFsmController::dynamicParamsCallback, this, std::placeholders::_1));
  if (planner_divider_ < 1) {planner_divider_ = 1;}
  computeNearRawCost();
  fp_base_ = costmap_ros_->getUnpaddedRobotFootprint();
  fp_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  RCLCPP_INFO(logger_, "[%s] collision model: %s (footprint %zu points, padding %.2f m), escape_latch %d, plan_hysteresis %d, rotation_check %d, align_gate %d, traj_check %d, hybrid_fallback %d, transit_holonomic %d, plan_switching %d",
    plugin_name_.c_str(), collision_model_.c_str(), fp_base_.size(), footprint_padding_, escape_latch_, plan_hysteresis_,
    rotation_check_, align_gate_enabled_, traj_check_enabled_, hybrid_fallback_, transit_holonomic_, plan_switching_);

  // ---- publishers (names kept close to ROS1 for the existing monitor tooling)
  auto latched = rclcpp::QoS(10).transient_local();
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("/midpro/local_plan", 1);
  ctrl_state_pub_ = node->create_publisher<std_msgs::msg::String>("/midpro/ctrl_state", latched);
  ctrl_transition_pub_ = node->create_publisher<std_msgs::msg::String>("/midpro/ctrl_transition", 10);
  path_mode_pub_ = node->create_publisher<std_msgs::msg::String>("/midpro/path_mode", latched);
  path_transition_pub_ = node->create_publisher<std_msgs::msg::String>("/midpro/path_transition", 10);
  detour_request_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/midpro/detour_request", 1);
  speed_limit_pub_ = node->create_publisher<std_msgs::msg::String>("/midpro/speed_limit", 10);
  conditioned_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("/midpro/conditioned_plan", 1);
  block_event_pub_ = node->create_publisher<std_msgs::msg::String>("/midpro/block_event", 20);
  profile_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("/midpro/speed_profile", rclcpp::QoS(1).transient_local());

  detour_path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/midpro/detour_path", rclcpp::QoS(1),
    [this](const nav_msgs::msg::Path::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mutex_);
      // ROS1 ebandCallback: a path of <=2 poses is accepted only if its end matches the goal
      if (msg->poses.size() <= 2 && have_global_plan_) {
        const auto & g = local_goal_.pose.position;
        Pose end_in_cm;
        if (!msg->poses.empty() && transformToCostmapFrame(msg->poses.back(), end_in_cm)) {
          if (std::hypot(g.x - end_in_cm.pose.position.x, g.y - end_in_cm.pose.position.y) > 0.020) {
            return;
          }
        }
      }
      detour_path_ = msg;
      detour_path_time_ = clock_->now();
      detour_path_time_set_ = true;
    });

  RCLCPP_INFO(logger_, "[%s] configured. costmap frame=%s, tracking=%s, waypoints=%s, detour source=%s",
    plugin_name_.c_str(), costmap_frame_.c_str(), tracking_mode_.c_str(), waypoint_mode_.c_str(), detour_source_.c_str());
}

void MidproFsmController::cleanup()
{
  dyn_params_handler_.reset();
  local_plan_pub_.reset();
  ctrl_state_pub_.reset();
  ctrl_transition_pub_.reset();
  path_mode_pub_.reset();
  path_transition_pub_.reset();
  detour_request_pub_.reset();
  speed_limit_pub_.reset();
  conditioned_plan_pub_.reset();
  global_cm_sub_.reset();
  guard_sub_.reset();
  block_event_pub_.reset();
  profile_pub_.reset();
  detour_path_sub_.reset();
}

void MidproFsmController::activate()
{
  local_plan_pub_->on_activate();
  ctrl_state_pub_->on_activate();
  ctrl_transition_pub_->on_activate();
  path_mode_pub_->on_activate();
  path_transition_pub_->on_activate();
  detour_request_pub_->on_activate();
  speed_limit_pub_->on_activate();
  conditioned_plan_pub_->on_activate();
  block_event_pub_->on_activate();
  profile_pub_->on_activate();
}

void MidproFsmController::deactivate()
{
  local_plan_pub_->on_deactivate();
  ctrl_state_pub_->on_deactivate();
  ctrl_transition_pub_->on_deactivate();
  path_mode_pub_->on_deactivate();
  path_transition_pub_->on_deactivate();
  detour_request_pub_->on_deactivate();
  speed_limit_pub_->on_deactivate();
  conditioned_plan_pub_->on_deactivate();
  block_event_pub_->on_deactivate();
  profile_pub_->on_deactivate();
}

void MidproFsmController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    speed_scale_ = 1.0;
  } else if (percentage) {
    speed_scale_ = std::clamp(speed_limit / 100.0, 0.0, 1.0);
  } else {
    speed_scale_ = std::clamp(speed_limit / max_linear_vel_, 0.0, 1.0);
  }
}

// ---------------------------------------------------------------- helpers
double MidproFsmController::yawOf(const Pose & p)
{
  return tf2::getYaw(p.pose.orientation);
}

double MidproFsmController::angleDiff(double a, double b)
{
  double d = a - b;
  while (d > M_PI) {d -= 2.0 * M_PI;}
  while (d < -M_PI) {d += 2.0 * M_PI;}
  return d;
}

geometry_msgs::msg::Quaternion MidproFsmController::quatFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0; q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

double MidproFsmController::dist(const Pose & a, const Pose & b)
{
  return std::hypot(b.pose.position.x - a.pose.position.x,
           b.pose.position.y - a.pose.position.y);
}

bool MidproFsmController::transformToCostmapFrame(const Pose & in, Pose & out) const
{
  if (in.header.frame_id == costmap_frame_ || in.header.frame_id.empty()) {
    out = in;
    out.header.frame_id = costmap_frame_;
    return true;
  }
  try {
    // use latest available transform (ROS1 code also used Time(0))
    Pose tmp = in;
    tmp.header.stamp = rclcpp::Time(0);
    out = tf_->transform(tmp, costmap_frame_);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "TF %s -> %s failed: %s",
      in.header.frame_id.c_str(), costmap_frame_.c_str(), ex.what());
    return false;
  }
}

// ---------------------------------------------------------------- costmap (ROS1 scale)
// loas_costmap produced three discrete bands: 100 obstacle, 99 inside the inscribed radius,
// 95 ("NEAR") inside circumscribed radius + 0.05 m. The planner blocks the path centre line at >= 95,
// which is what kept the whole footprint (and in-place rotation) clear of obstacles in ROS1.
// Nav2's inflation layer decays continuously, so we rebuild the 95 band from a distance:
// any raw cost >= computeCost(near_obstacle_distance) is reported as 95.
void MidproFsmController::computeNearRawCost()
{
  near_raw_cost_ = 243;   // fallback: 95 on the 0..100 scale if no inflation layer is found
  auto layered = costmap_ros_->getLayeredCostmap();
  if (!layered) {return;}
  for (auto & layer : *layered->getPlugins()) {
    auto infl = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(layer);
    if (!infl) {continue;}
    const double res = costmap_->getResolution();
    const unsigned char c = infl->computeCost(near_obstacle_distance_ / res);
    near_raw_cost_ = std::max<unsigned char>(1, c);
    // footprint prefilter: a centre cell cheaper than cost(circumscribed + 2*pad) is far enough that no
    // footprint orientation can touch an obstacle -> skip the perimeter check (same idea as Hybrid-A*)
    const double circ = layered->getCircumscribedRadius();
    prefilter_raw_cost_ = std::max<unsigned char>(1, infl->computeCost((circ + std::max({2.0 * footprint_padding_, straight_clearance_, tight_clearance_})) / res));
    RCLCPP_INFO(logger_, "[%s] NEAR band: raw cost >= %u at %.2f m (inscribed %.2f m); footprint prefilter raw < %u (circumscribed %.2f m + 2 x %.2f)",
      plugin_name_.c_str(), near_raw_cost_, near_obstacle_distance_, layered->getInscribedRadius(), prefilter_raw_cost_,
      layered->getCircumscribedRadius(), footprint_padding_);
    if (c <= 1) {
      RCLCPP_WARN(logger_, "[%s] inflation_radius is smaller than near_obstacle_distance (%.2f m); "
        "the NEAR band collapses to the inscribed radius. Increase inflation_radius.",
        plugin_name_.c_str(), near_obstacle_distance_);
    }
    return;
  }
  RCLCPP_WARN(logger_, "[%s] no InflationLayer in the costmap; NEAR band uses raw cost >= 243",
    plugin_name_.c_str());
}

// Map nav2 raw cost (0..255) to the loas_costmap convention:
//   NO_INFORMATION(255) -> -1,  LETHAL(254) -> 100,  INSCRIBED(253) -> 99,
//   >= near_raw_cost_   -> 95 (NEAR band),  below -> 1..94 proportional,  0 -> 0,  out of map -> -999
int MidproFsmController::getCost(const Pose & p) const
{
  if (!costmap_) {return -999;}
  unsigned int mx, my;
  if (!costmap_->worldToMap(p.pose.position.x, p.pose.position.y, mx, my)) {
    return -999;
  }
  const unsigned char c = costmap_->getCost(mx, my);
  if (c == nav2_costmap_2d::NO_INFORMATION) {return -1;}
  if (c == nav2_costmap_2d::LETHAL_OBSTACLE) {return 100;}
  if (c == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {return 99;}
  if (c == nav2_costmap_2d::FREE_SPACE) {return 0;}
  if (c >= near_raw_cost_) {return 95;}
  return 1 + static_cast<int>(93.0 * static_cast<double>(c) / static_cast<double>(near_raw_cost_));
}

// ---------------------------------------------------------------- footprint helpers
std::vector<geometry_msgs::msg::Point> MidproFsmController::paddedFootprint(double pad) const
{
  std::vector<geometry_msgs::msg::Point> fp = fp_base_;
  for (auto & p : fp) {
    p.x += (p.x >= 0.0 ? pad : -pad);
    p.y += (p.y >= 0.0 ? pad : -pad);
  }
  return fp;
}

// footprintFree: the robot footprint (+pad on every side) placed at (x, y, yaw) touches no lethal cell.
// Prefilter keeps it cheap: lethal/inscribed centre -> blocked, far from everything -> free.
bool MidproFsmController::footprintFree(double x, double y, double yaw, double pad) const
{
  if (!costmap_ || fp_base_.empty()) {return true;}
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {return false;}
  const unsigned char c = costmap_->getCost(mx, my);
  if (c == nav2_costmap_2d::NO_INFORMATION) {return false;}
  if (c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {return false;}
  if (c < prefilter_raw_cost_) {return true;}
  if (!fp_checker_) {
    fp_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  }
  const double cost = fp_checker_->footprintCostAtPose(x, y, yaw, paddedFootprint(pad));
  return cost < static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
}

// rotationFree: every heading (12 samples) fits at this position -> rotating in place cannot hit anything.
bool MidproFsmController::rotationFree(const Pose & p) const
{
  const double x = p.pose.position.x, y = p.pose.position.y;
  unsigned int mx, my;
  if (costmap_ && costmap_->worldToMap(x, y, mx, my) && costmap_->getCost(mx, my) < prefilter_raw_cost_) {return true;}
  for (int k = 0; k < 12; ++k) {
    if (!footprintFree(x, y, k * M_PI / 6.0, footprint_padding_)) {return false;}
  }
  return true;
}

// getBlockingCell: walk the straight segment from -> to and report the first blocking sample.
//   circle model (v0.28): centre cell cost >= 95 (include_near) or > 95 (hard only)
//   footprint model: footprint at the robot heading (straight/line paths keep it) collides with
//                    2 x padding (include_near = "comfortable") or 1 x padding (hard only = "tight").
//                    Leading samples that already collide are skipped (the robot is squeezing out of a bay).
bool MidproFsmController::getBlockingCell(
  const Pose & from, const Pose & to, bool include_near,
  int & blk_cost, double & bx, double & by) const
{
  if (!costmap_) {return false;}
  const double res = costmap_->getResolution();
  const double x0 = from.pose.position.x, y0 = from.pose.position.y;
  const double x1 = to.pose.position.x, y1 = to.pose.position.y;
  const double len = std::hypot(x1 - x0, y1 - y0);
  if (useFootprint()) {
    const double yaw = yawOf(from);
    // include_near = 'comfortable' (is a straight line a GOOD choice), else 'passable' (can the body get through)
    const double pad = include_near ? std::max({2.0 * footprint_padding_, straight_clearance_, tight_clearance_}) : footprint_padding_;
    const double step = 2.0 * res;
    const int num = std::max(1, static_cast<int>(len / step));
    bool seen_free = false;
    for (int k = 0; k <= num; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(num);
      const double x = x0 + t * (x1 - x0), y = y0 + t * (y1 - y0);
      const bool ok = footprintFree(x, y, yaw, pad);
      if (ok) {seen_free = true; continue;}
      if (!seen_free && t * len < 0.5) {continue;}      // escaping from a tight start
      blk_cost = include_near ? near_obstacle_value_ : 100; bx = x; by = y;
      return true;
    }
    return false;
  }
  const int num = std::max(1, static_cast<int>(len / res));
  for (int k = 0; k < num; ++k) {
    const double t = (num == 1) ? 0.0 : static_cast<double>(k) / static_cast<double>(num - 1);
    Pose p;
    p.pose.position.x = x0 + t * (x1 - x0);
    p.pose.position.y = y0 + t * (y1 - y0);
    const int cost = getCost(p);
    const bool blocked = include_near ?
      (cost == -999 || cost >= near_obstacle_value_) :
      (cost == -999 || cost > near_obstacle_value_);
    if (blocked) {
      blk_cost = cost; bx = p.pose.position.x; by = p.pose.position.y;
      return true;
    }
  }
  return false;
}

bool MidproFsmController::isLineCollisionFree(const Pose & from, const Pose & to, bool include_near) const
{
  int c; double bx, by;
  return !getBlockingCell(from, to, include_near, c, bx, by);
}

// ---------------------------------------------------------------- state bookkeeping
void MidproFsmController::setPathState(const std::string & new_state, const std::string & reason)
{
  if (path_state_ == new_state) {return;}
  std_msgs::msg::String t;
  t.data = path_state_ + " -> " + new_state + " | " + reason;
  path_transition_pub_->publish(t);
  RCLCPP_INFO(logger_, "[path] %s", t.data.c_str());
  path_state_ = new_state;
  std_msgs::msg::String m; m.data = path_state_;
  path_mode_pub_->publish(m);
  if (new_state == "detour") {
    detour_enter_time_ = clock_->now();
    detour_path_.reset();
    detour_path_time_set_ = false;
  }
}

void MidproFsmController::setCtrlState(const std::string & s, const std::string & reason)
{
  if (ctrl_state_ == s) {return;}
  std_msgs::msg::String t;
  t.data = ctrl_state_ + " -> " + s + " | " + reason;
  ctrl_transition_pub_->publish(t);
  RCLCPP_INFO(logger_, "[ctrl] %s", t.data.c_str());
  ctrl_state_ = s;
  std_msgs::msg::String m; m.data = ctrl_state_;
  ctrl_state_pub_->publish(m);
}

void MidproFsmController::resetPlannerState()
{
  detour_path_.reset();
  detour_info_ = DetourInfo{};
  detour_dir_.clear();
  start_set_ = false;
  start_is_escape_ = false;
  detour_path_time_set_ = false;
  try_side_ = false;
  was_blocked_ = false;
  tick_counter_ = 0;
  const std::string old = path_state_;
  path_state_ = initial_path_mode_;
  if (old != path_state_) {
    std_msgs::msg::String m; m.data = path_state_;
    path_mode_pub_->publish(m);
  }
}

void MidproFsmController::resetControllerState()
{
  path_.clear();
  final_approach_counter_ = 0;
  has_init_cmd_time_ = false;
  has_rotate_start_time_ = false;
  prev_linear_x_ = 0.0;
  prev_linear_y_ = 0.0;
  goal_reached_ = false;
}

// dynamicParamsCallback: runtime switches for the dashboard check boxes. controller_server skips plugin
// parameters (names containing '.') in its own callback, so the plugin handles them here.
rcl_interfaces::msg::SetParametersResult MidproFsmController::dynamicParamsCallback(std::vector<rclcpp::Parameter> params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::lock_guard<std::mutex> lk(mutex_);
  for (const auto & p : params) {
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {continue;}
    if (p.get_name() == plugin_name_ + ".plan_switching") {
      plan_switching_ = p.as_bool();
      RCLCPP_INFO(logger_, "[%s] plan_switching -> %d", plugin_name_.c_str(), plan_switching_);
    } else if (p.get_name() == plugin_name_ + ".transit_holonomic") {
      transit_holonomic_ = p.as_bool();
      RCLCPP_INFO(logger_, "[%s] transit_holonomic -> %d", plugin_name_.c_str(), transit_holonomic_);
    } else if (p.get_name() == plugin_name_ + ".path_conditioner_enabled") {
      path_conditioner_enabled_ = p.as_bool();
      RCLCPP_INFO(logger_, "[%s] path_conditioner_enabled -> %d", plugin_name_.c_str(), path_conditioner_enabled_);
    } else if (p.get_name() == plugin_name_ + ".tight_track_enabled") {
      tight_track_enabled_ = p.as_bool();
      RCLCPP_INFO(logger_, "[%s] tight_track_enabled -> %d", plugin_name_.c_str(), tight_track_enabled_);
    } else if (p.get_name() == plugin_name_ + ".goal_staging") {
      goal_staging_ = p.as_bool();
      if (!goal_staging_ && stage_phase_ != 0) {stageEnd("disabled");}
      RCLCPP_INFO(logger_, "[%s] goal_staging -> %d", plugin_name_.c_str(), goal_staging_);
    } else if (p.get_name() == plugin_name_ + ".side_front_enabled") {
      side_front_enabled_ = p.as_bool();
      RCLCPP_INFO(logger_, "[%s] side_front_enabled -> %d", plugin_name_.c_str(), side_front_enabled_);
    } else if (p.get_name() == plugin_name_ + ".orthogonal_avoid") {
      orthogonal_avoid_ = p.as_bool();
      if (!orthogonal_avoid_ && orth_phase_ != 0) {orthEnd("disabled");}
      RCLCPP_INFO(logger_, "[%s] orthogonal_avoid -> %d", plugin_name_.c_str(), orthogonal_avoid_);
    } else if (p.get_name() == plugin_name_ + ".traj_check_enabled") {
      traj_check_enabled_ = p.as_bool();
      RCLCPP_INFO(logger_, "[%s] traj_check_enabled -> %d", plugin_name_.c_str(), traj_check_enabled_);
    }
  }
  return result;
}

// onGuardState: the obstacle_guard measures, from the raw laser points, how far the footprint can travel
// straight ahead ("dist": {"x+": m}); the costmap raster (2.5-5 cm cells + padding) is coarser than that.
void MidproFsmController::onGuardState(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & d = msg->data;
  const size_t k = d.find("\"dist\"");
  if (k == std::string::npos) {return;}
  const size_t p = d.find("\"x+\"", k);
  if (p == std::string::npos) {return;}
  const size_t colon = d.find(':', p);
  if (colon == std::string::npos) {return;}
  std::lock_guard<std::mutex> lk(mutex_);
  if (d.compare(colon + 1, 5, " null") == 0 || d.compare(colon + 1, 4, "null") == 0) {
    guard_fwd_dist_ = 1e9;                      // nothing on the way
  } else {
    try {guard_fwd_dist_ = std::stod(d.substr(colon + 1));} catch (...) {return;}
  }
  guard_stamp_ = clock_->now();
}

// guardSaysForwardClear: the segment robot -> to is (nearly) along the heading and the guard's exact forward
// distance is well beyond the costmap's blocking point -> the "blocked" is raster/padding noise in a tight gap.
bool MidproFsmController::guardSaysForwardClear(const Pose & robot, const Pose & to, double block_dist) const
{
  if (!trust_guard_ || guard_fwd_dist_ < 0.0) {return false;}
  if ((clock_->now() - guard_stamp_).seconds() > 0.5) {return false;}
  const double dir = std::atan2(to.pose.position.y - robot.pose.position.y, to.pose.position.x - robot.pose.position.x);
  if (std::abs(angleDiff(dir, yawOf(robot))) > 0.17) {return false;}   // only straight ahead (~10 deg)
  return guard_fwd_dist_ > block_dist + 0.30;
}

// conditionPlan: reshape the received plan for the long swerve body against the global costmap (see
// path_conditioner.hpp). Any failure (no costmap yet, TF) leaves the plan untouched.
void MidproFsmController::conditionPlan(nav_msgs::msg::Path & path)
{
  if (!global_cm_sub_ || fp_base_.empty() || path.poses.size() < 4) {return;}
  std::shared_ptr<nav2_costmap_2d::Costmap2D> gcm;
  try {
    gcm = global_cm_sub_->getCostmap();
  } catch (const std::exception &) {
    return;                                        // not received yet
  }
  if (!gcm) {return;}
  geometry_msgs::msg::PoseStamped rp, rp_plan;
  if (!costmap_ros_->getRobotPose(rp)) {return;}
  if (!nav2_util::transformPoseInTargetFrame(rp, rp_plan, *costmap_ros_->getTfBuffer(), path.header.frame_id, 0.2)) {return;}
  ConditionerParams cp;
  cp.horizon = conditioner_horizon_;
  cp.interval = std::max(path_pose_interval_, 0.02);
  cp.pull_margins = {std::max(tight_clearance_, 0.03), 0.03, 0.01};
  cp.move_pad = std::max(0.03, footprint_padding_);
  PathConditioner pc(gcm.get(), fp_base_, cp);
  const ConditionerReport rep = pc.condition(path, yawOf(rp_plan));
  if (!rep.note.empty()) {
    RCLCPP_INFO(logger_, "[cond] plan conditioned: %s(%zu poses)", rep.note.c_str(), path.poses.size());
  }
  if (conditioned_plan_pub_ && conditioned_plan_pub_->is_activated()) {conditioned_plan_pub_->publish(path);}
}

}  // namespace midpro_nav2_controller
