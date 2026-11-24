// midpro_fsm_controller.hpp
// ROS2 Humble port of midpro 1.4 loas_planner (loas_local_planner + loas_controller)
// as a single nav2_core::Controller plugin.
//
// Logic preserved from ROS1:
//   - Local path state machine : straight -> side_front / front_side -> line -> detour(eband)
//   - Pose yaw filling rules   : fixed yaw (holonomic) vs tangent yaw (ackermann)
//   - Controller FSM           : FOLLOW_PATH <-> ROTATE_IN_PLACE -> FINAL_APPROACH -> FINAL_ROTATE -> IDLE
//   - Output filtering         : linear.y LPF(0.3), linear.x rate limit(0.015/tick), min angular clamp
//
// Changes vs ROS1 (intentional):
//   - eband path source replaced by either the Nav2 global plan itself (detour_source: global_plan, default)
//     or a Hybrid-A* path from a second planner_server via detour_bridge (detour_source: planner)
//   - costmap read through nav2_costmap_2d API; raw costs rescaled to the ROS1 0..100 convention
//   - holonomic/ackermann decision is made once at path generation (same rule, angular tolerance
//     instead of the 1e-10 compare) and published on /midpro/path_mode
//   - goal_info string checks removed: the goal is the last pose of the Nav2 plan

#ifndef MIDPRO_NAV2_CONTROLLER__MIDPRO_FSM_CONTROLLER_HPP_
#define MIDPRO_NAV2_CONTROLLER__MIDPRO_FSM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "midpro_nav2_controller/path_conditioner.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace midpro_nav2_controller
{

class MidproFsmController : public nav2_core::Controller
{
public:
  MidproFsmController() = default;
  ~MidproFsmController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  using Pose = geometry_msgs::msg::PoseStamped;

  // ------------------------------------------------------------------ helpers
  static double yawOf(const Pose & p);
  static double angleDiff(double a, double b);
  static geometry_msgs::msg::Quaternion quatFromYaw(double yaw);
  static double dist(const Pose & a, const Pose & b);
  bool transformToCostmapFrame(const Pose & in, Pose & out) const;

  // costmap access (ROS1 scale: -999 out of map, -1 unknown, 0..100)
  int getCost(const Pose & p) const;
  bool getBlockingCell(const Pose & from, const Pose & to, bool include_near,
    int & blk_cost, double & bx, double & by) const;
  bool isLineCollisionFree(const Pose & from, const Pose & to, bool include_near = true) const;

  // ------------------------------------------------------------------ local path builder (ex loas_local_planner)
  void plannerTick(const Pose & robot_pose);
  bool makeLocalPath(const Pose & robot, const Pose & goal, nav_msgs::msg::Path & out);
  void finalizeLocalPath(nav_msgs::msg::Path & path, bool modify_orient, bool detour);
  bool findSideGoal(const Pose & robot, const Pose & goal, Pose & out) const;
  bool findFrontGoal(const Pose & robot, const Pose & goal, Pose & out) const;
  nav_msgs::msg::Path makeDetourPath();
  bool pickDetourGoal(const Pose & robot_cm, geometry_msgs::msg::PoseStamped & goal_map) const;
  nav_msgs::msg::Path makeGlobalPlanPath(const Pose & robot_cm) const;
  // waypoint sequencing (ROS1 move_server role): corners of the global plan become successive goals
  void buildWaypoints(const nav_msgs::msg::Path & plan_map);
  bool advanceWaypoint();
  bool isLastWaypoint() const {return wp_index_ + 1 >= waypoints_.size();}
  // speed governor (neo_local_planner style): obstacle clearance, curvature, stopping distance
  double speedGovernor(double v, double dist_to_goal, double next_yaw_diff) const;
  // path-based speed profile (planned once per plan, replaces the reactive curvature/inflation caps)
  void buildSpeedProfile();
  double profileSpeedAt(const Pose & robot_cm) const;
  // lookahead path tracking (tracking_mode: lookahead): replaces the ROS1 point-to-point law in FOLLOW_PATH
  bool lookaheadTick(geometry_msgs::msg::Twist & cmd, double dist_to_goal);
  bool lookaheadPoint(double & dir, double & tangent) const;   // direction of travel and path tangent at the lookahead point
  void setPathState(const std::string & new_state, const std::string & reason);
  void resetPlannerState();

  // ------------------------------------------------------------------ FSM controller (ex loas_controller)
  geometry_msgs::msg::Twist controlTick();
  void setCtrlState(const std::string & s, const std::string & reason);
  void resetControllerState();

  // ------------------------------------------------------------------ members
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("MidproFsmController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  std::string costmap_frame_;   // usually "odom"

  // publishers
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr ctrl_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr ctrl_transition_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr path_mode_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr path_transition_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr detour_request_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr detour_path_sub_;

  // --- parameters: planner
  int near_obstacle_value_{95};
  double near_obstacle_distance_{0.59};   // m from an obstacle that counts as NEAR (ROS1: circumscribed 0.54 + 0.05)
  unsigned char near_raw_cost_{243};      // raw costmap cost at near_obstacle_distance_ (computed from the inflation layer)
  void computeNearRawCost();
  double path_pose_interval_{0.05};
  std::string orient_mode_{"robot"};      // robot | path | goal
  std::string initial_path_mode_{"straight"};
  double side_search_max_{5.0};
  double side_search_step_{0.05};
  double detour_timeout_{5.0};            // s without detour path before failing
  double detour_path_max_age_{1.0};       // s, same as ROS1 eband timeout
  double detour_goal_lookahead_{4.0};     // m along the global plan; must fit in the detour costmap window
  std::string detour_source_{"global_plan"};  // global_plan | planner (Hybrid-A* via detour_bridge)
  double detour_check_dist_{2.0};           // m of the followed global plan checked for hard obstacles
  // --- speed profile
  bool speed_profile_enabled_{true};
  double lateral_acc_limit_{0.4};           // m/s^2: v <= sqrt(a_lat / kappa)
  double profile_curv_window_{0.3};         // m: tangent change measured over +-window
  double profile_min_speed_{0.15};          // m/s: profile never goes below this except at the goal
  std::vector<Pose> profile_poses_;         // costmap frame
  std::vector<double> profile_v_;
  mutable size_t profile_hint_{0};
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>::SharedPtr profile_pub_;
  // --- waypoints
  double waypoint_simplify_tol_{0.10};     // m, Douglas-Peucker tolerance on the global plan
  double waypoint_dist_tolerance_{0.05};   // intermediate waypoint tolerances (final uses dist/yaw_tolerance)
  double waypoint_yaw_tolerance_{0.05};
  double goal_orient_dist_{0.8};           // [ROS1] final goal: poses within this distance take the goal yaw
  double waypoint_orient_dist_{0.0};       // intermediate corners: keep segment heading until the corner
  // --- speed governor
  double linear_acc_limit_{0.8};           // m/s^2 (replaces the fixed per-tick rate limit when > 0)
  double linear_dec_limit_{0.6};           // m/s^2 used for the stopping-distance law
  double speed_lookahead_dist_{1.5};       // m of path checked ahead for obstacle cost
  double near_obstacle_speed_{0.3};        // cap when a NEAR(95) cell lies ahead
  double inflation_speed_{0.6};            // cap when any inflated cell lies ahead
  double curve_speed_{0.4};                // cap when the next heading change exceeds curve_angle
  double curve_angle_{0.17};               // rad
  double governor_floor_{0.1};              // m/s; situational caps never go below this (goal braking still does)
  int inflation_cost_threshold_{80};        // 0..100 scale; inflated cells below this do not cap the speed
  double curve_lookahead_{1.0};             // m; curvature = path direction change over this distance (not the next 5 cm point)
  mutable std::string gov_reason_;          // which cap is active, published on midpro/speed_limit
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr speed_limit_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr block_event_pub_;   // v0.56 tuning feedback
  void blockEvent(const std::string & ev, double at, double age, const std::string & why = "") const;
  double controller_frequency_{20.0};
  // --- tracking mode
  std::string tracking_mode_{"lookahead"};  // lookahead (smooth path following) | midpro (ROS1 point-to-point)
  std::string waypoint_mode_{"final"};      // final (goal only) | corners (Douglas-Peucker corner stops)
  double lookahead_min_{0.5}, lookahead_gain_{0.8};  // L = max(lookahead_min, lookahead_gain * v)
  double heading_gain_{1.5};                // yaw P gain toward the path tangent
  double rotate_threshold_{1.2};            // rad; rotate in place only above this heading error
  double align_angle_{0.35};                // rad; below this the body is considered aligned after a rotation
  std::string holonomic_policy_{"goal_zone"};  // goal_zone (car-like in transit, holonomic near the goal) | always
  double holonomic_zone_{1.5};              // m from the goal where lateral motion is allowed
  double steer_gain_{2.0};                  // car-like mode: yaw rate per rad of pure-pursuit heading error
  int planner_divider_{2};                // run planner every N control ticks (20Hz/2 = 10Hz as ROS1)
  double oblique_yaw_tolerance_{1e-4};   // rad; ROS1 used 1e-10 on raw yaw compare

  // --- parameters: controller
  double max_linear_vel_{0.2}, min_linear_vel_{0.05};
  double max_angular_vel_{0.4}, min_angular_vel_{0.05};
  double acceleration_limit_{0.1}, angular_acc_limit_{1.0};
  double approach_dist_{0.5}, turn_angular_vel_{0.5};
  double dist_tolerance_{0.02}, yaw_tolerance_{0.025};
  int final_approach_threshold_{5};
  double k_prop_{0.80};
  double linear_y_lpf_alpha_{0.3};
  double linear_x_rate_limit_{0.015};
  double max_linear_y_{0.1};
  double min_turn_speed_ratio_{0.67};     // ackermann law: linear.x >= linear_speed * this while steering (ROS1 had no floor)
  double heading_lookahead_{0.4};          // m; steering reference = path point this far ahead (0 = ROS1: next point, 5 cm)
  bool curvature_feedforward_{false};      // ackermann law: add v*kappa(reference) so curves need no steady-state heading error
  // --- bay exit / goal-zone policy
  bool no_rotate_in_near_{true};           // inside the NEAR band never rotate in place: slide out holonomically (midpro escape)
  double escape_speed_{0.25};              // m/s used while escaping the NEAR band (ROS1 crawled at 0.1 when backing)
  bool holonomic_only_when_needed_{true};  // goal-yaw hold on the last goal_orient_dist only near obstacles or for side parking
  double goal_obstacle_check_radius_{1.0}; // m around the goal scanned for NEAR cells
  double goal_side_angle_{0.52};           // rad: goal yaw vs approach direction above this = side parking -> holonomic (only if side_parking_holonomic)
  bool side_parking_holonomic_{false};     // false: in open space always arrive aligned and rotate at the goal
  // --- detour recovery / path deviation
  int detour_clear_ticks_{5};              // planner ticks the straight line must stay clear before leaving detour
  int detour_clear_count_{0};
  double max_path_deviation_{1.5};         // m from the global plan before FollowPath fails (BT replans from here)
  // --- collision model
  std::string collision_model_{"footprint"};   // footprint (pose-yaw footprint checks) | circle (v0.28: centre cell vs NEAR band)
  double footprint_padding_{0.05};             // m added on every side; raise to 0.08 with AMCL-grade localisation
  double tight_passage_speed_{0.3};            // m/s when clearance ahead is below 2 x padding
  bool hybrid_fallback_{true};                 // footprint-infeasible global plan -> ask the Hybrid-A* detour server (if running)
  unsigned char prefilter_raw_cost_{200};      // raw cost below this = farther than circumscribed + 2*pad -> free without a footprint check
  mutable std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> fp_checker_;
  std::vector<geometry_msgs::msg::Point> fp_base_;   // unpadded footprint
  std::vector<geometry_msgs::msg::Point> paddedFootprint(double pad) const;
  bool footprintFree(double x, double y, double yaw, double pad) const;
  bool rotationFree(const Pose & p) const;
  bool useFootprint() const {return collision_model_ == "footprint";}
  // --- consistency / commitment
  double align_gate_dist_{1.0};            // m: look this far ahead for a tight stretch before entering it
  double align_tolerance_{0.12};           // rad: body must be within this of the tight stretch heading
  bool align_active_{false};
  bool following_hybrid_{false};          // a local Hybrid-A* detour is being followed (deviation guard suspended)
  // --- NEAR escape latch (v0.32): escape (hold yaw, slide) only when a plan STARTS inside the NEAR band
  bool escape_active_{false};              // latched at the first tick of a new plan if the robot is inside NEAR
  bool escape_latch_{true};                // false = v0.28: escape whenever the robot is inside NEAR
  bool plan_hysteresis_{true};             // false = v0.28: accept every replan
  bool rotation_check_{true};              // footprint model: forbid in-place rotation where the body does not fit
  bool align_gate_enabled_{true};          // footprint model: align before a tight stretch
  // --- v0.41 policy switches (runtime-settable: ros2 param set /controller_server FollowPath.<name>)
  bool transit_holonomic_{false};          // false: avoidance and transit drive aligned (curved path, ackermann law);
                                           // holonomic only for the NEAR escape, the goal zone and as a last resort
  bool plan_switching_{false};             // false: keep the current plan for the same goal unless it is blocked / left
  double straight_clearance_{0.15};        // footprint model: m of clearance around the body to PREFER a straight line
  bool min_angular_in_place_only_{true};   // v0.46: min_angular_vel floor only for in-place / final rotation
  // --- v0.48 goal identity / plan cache
  double goal_same_tol_{0.6};              // m: plan ends this close = same goal (planner tolerance moves the end)
  double plan_switch_force_ratio_{0.5};    // same goal: a plan at most this fraction of the current length is taken
                                           // even with plan_switching off (goal ambiguity, not a left/right flip-flop)
  mutable std::vector<Pose> plan_cm_;      // global plan in the costmap frame, refreshed at most every 0.5 s
  // --- v0.49 yield: a blockage must persist before the path is abandoned (pedestrians pass, boxes stay)
  double block_persist_time_{3.0};         // s the followed plan must stay blocked before replanning around it
  double yield_stop_margin_{0.6};          // m: stop this far before the blocking point while waiting
  bool block_active_{false};
  rclcpp::Time blocked_since_;
  double yield_cap_{-1.0};                 // m/s cap while waiting (-1 = none), set by the planner tick
  nav_msgs::msg::Path pending_plan_;       // latest alternative plan offered by the BT while yielding
  // --- v0.55 early decision: look far, classify the blockage (static pallet vs moving person)
  double block_check_dist_{4.5};           // m of the followed plan checked for a blockage (inside the local costmap)
  double block_classify_time_{1.0};        // s of observation before a non-moving blockage counts as static
  double block_move_tol_{0.20};            // m: the obstacle cell nearest to the robot moving more = moving obstacle
  bool block_moving_{false};
  double block_ref_x_{0.0}, block_ref_y_{0.0};
  bool blockingObstaclePoint(const Pose & p, double yaw, const Pose & robot, double & ox, double & oy) const;
  bool pendingAvoids(double ox, double oy) const;
  double back_off_speed_{0.12};            // m/s: slide away from the nearest obstacle to make room before aligning
  double shortcut_dist_{3.5};              // m: string-pull the followed plan up to this far (0 = off)
  double oblique_lookahead_{0.10};         // v0.69: holonomic paths aim at the point this far ahead (ROS1: path[1], 5 cm);
                                           // the 0.4 m ackermann reference cut the corners of side/front paths
  // --- v0.68 orthogonal manoeuvre (midpro side_front, made strictly right-angled): obstacle close ahead and the
  // goal beyond it -> keep the heading, shift sideways just enough, stop (wheels re-steer), drive straight
  bool orthogonal_avoid_{true};            // runtime switch (dashboard check box)
  bool side_front_enabled_{true};          // v0.72: midpro side_front / front_side in transit (yaw held, right-angled)
  double side_trigger_dist_{2.5};          // v0.79: side/front only when the blockage is this close (midpro goals were near waypoints)
  // --- v0.73 goal staging: never turn in place where the body does not fit (wall-side parking, L/U pockets).
  // If turning at the goal from the arrival heading would hit something, stop at a staging pose on the plan
  // where it fits, turn to the goal yaw there, then slide into the goal with the yaw held.
  bool goal_staging_{true};
  // --- v0.81 path conditioner: the plan is reshaped once per setPlan against the GLOBAL costmap (start shift,
  // corridor straightening, goal staging through off-plan axis poses) before the controller follows it
  bool path_conditioner_enabled_{true};
  double conditioner_horizon_{25.0};
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> global_cm_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr conditioned_plan_pub_;
  void conditionPlan(nav_msgs::msg::Path & path);
  // --- v0.75 narrow passages: passability is geometry-only (padding 0, the obstacle_guard keeps the physical
  // margin from the raw laser points); "tight" detection uses tight_clearance; the guard's exact forward
  // distance overrides a costmap-raster "blocked straight ahead"
  double tight_clearance_{0.06};           // m: less than this around the body = tight passage (slow, align first)
  // --- v0.76 tight-passage tracking (ROS1 behaviour for a swerve body): keep the heading along the corridor,
  // correct the lateral error by crabbing (no body swing), tiny yaw correction only
  bool tight_track_enabled_{true};
  double tight_track_lat_gain_{0.8};       // vy = gain * lateral error (m/s per m)
  double tight_track_lat_max_{0.08};       // m/s
  double tight_track_yaw_gain_{0.6};       // wz = gain * heading error
  double tight_track_yaw_max_{0.15};       // rad/s
  mutable bool tight_now_{false};          // cache written by the (const) speed governor each tick: clearance ahead < tight_clearance
  bool signedTrackError(double x, double y, double yaw, double & e_lat, double & e_yaw) const;
  bool trust_guard_{true};
  double guard_fwd_dist_{-1.0};            // m along the heading from /obstacle_guard/state (-1 = unknown)
  rclcpp::Time guard_stamp_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr guard_sub_;
  void onGuardState(const std_msgs::msg::String::SharedPtr msg);
  bool guardSaysForwardClear(const Pose & robot, const Pose & to, double block_dist) const;
  double stage_eval_dist_{4.0};            // m from the goal at which the decision is made
  double stage_min_dist_{0.6};             // m: staging pose search range back along the plan
  double stage_max_dist_{3.0};
  int stage_phase_{0};                     // 0 none, 1 drive to staging pose, 2 turn there, 3 slide into the goal
  bool stage_evaluated_{false};
  Pose stage_pose_;
  double stage_goal_yaw_{0.0};
  double stage_best_d_{1e9};
  rclcpp::Time stage_t_;
  bool rotSweepFree(double x, double y, double yaw0, double yaw1, double pad) const;
  bool stageEvaluate(const Pose & robot, const Pose & goal);
  void stageTick(const Pose & robot, const Pose & goal);
  void stageEnd(const std::string & why);
  bool translationFree(const Pose & a, const Pose & b, double yaw) const;
  double orth_trigger_dist_{1.2};          // m: obstacle this close ahead (or robot stopped and < 2.5 m) triggers
  double orth_max_shift_{1.6};             // m: largest sideways shift considered
  double orth_speed_{0.25};                // m/s during the manoeuvre
  int orth_phase_{0};                      // 0 none, 1 side shift, 2 straight
  Pose orth_p1_, orth_p2_;
  double orth_yaw_{0.0};
  rclcpp::Time orth_t0_, orth_phase_t_, orth_end_t_;
  bool orth_ended_once_{false};
  double meas_speed_{0.0};                 // measured planar speed (from computeVelocityCommands)
  rclcpp::Time orth_last_eval_;
  bool orth_eval_once_{false};
  double forwardBlockDist(const Pose & from, double yaw, double max_d, double pad) const;
  bool lateralFree(const Pose & a, const Pose & b, double yaw, double pad) const;
  bool orthTrigger(const Pose & robot, const Pose & goal);
  void orthTick(const Pose & robot, const Pose & goal);
  void orthEnd(const std::string & why);
  void orthPublishPath(const Pose & robot, const Pose & goal);
  bool segmentFree(const Pose & a, const Pose & b, double pad) const;
  void shortcutPath(nav_msgs::msg::Path & ep) const;
  bool nearestObstacleAround(const Pose & robot, double reach, double & ox, double & oy) const;
  bool has_pending_plan_{false};
  void adoptPendingPlan();
  double blockAge() const;
  mutable rclcpp::Time plan_cm_stamp_;
  mutable bool plan_cm_valid_{false};
  mutable size_t plan_hint_{0};
  const std::vector<Pose> & planCm() const;
  size_t nearestPlanIndex(const std::vector<Pose> & cm, const Pose & robot_cm, double & best) const;
  double steer_deadband_{0.01};            // rad/s: while driving, smaller steering corrections are zeroed
  bool prefer_comfortable_curve_{true};    // straight too close but passable: follow the global plan if IT has comfortable
                                           // clearance; if not (narrow corridor) pass straight (line state, midpro intent)
  double comfort_check_dist_{3.0};         // m of the global plan checked for comfortable clearance
  bool globalPlanComfortable(const Pose & robot_cm) const;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  rcl_interfaces::msg::SetParametersResult dynamicParamsCallback(std::vector<rclcpp::Parameter> params);
  bool fresh_plan_{false};                 // set by setPlan on a new target, consumed by the next control tick
  double align_move_speed_{0.2};           // m/s while aligning to the path tangent on the move (no room to rotate in place)
  // --- trust-the-plan thresholds (v0.33): no intervention while the robot is tracking its path
  double track_lat_tol_{0.08};             // m lateral distance to the local path
  double track_yaw_tol_{0.15};             // rad heading error to the local path tangent at the robot
  bool trackingWell(double x, double y, double yaw, double & lat, double & dyaw) const;
  // --- trajectory safety filter (forward simulation of the actual command with the footprint)
  bool traj_check_enabled_{true};
  double traj_check_time_{1.0};            // s simulated ahead (0.1 s steps)
  bool trajectoryFree(double x, double y, double th, double vx, double vy, double wz, double pad, int skip) const;
  bool motionFits(double x, double y, double yaw, double vx, double vy, double wz) const;   // footprint rollout, escape-aware
  geometry_msgs::msg::Twist safetyFilter(const geometry_msgs::msg::Twist & in, double x, double y, double yaw, double ref_dir);
  double align_target_yaw_{0.0};
  double plan_switch_gain_{0.15};          // same goal: switch at once if the new plan is this much shorter
  double plan_switch_small_gain_{0.05};    // ... or if it is at least this much shorter on plan_switch_confirm_ replans in a row
  int plan_switch_confirm_{2};
  int shorter_count_{0};
  double plan_keep_check_dist_{3.0};       // m of the current plan that must stay footprint-feasible to keep it
  double remainingLength(const std::vector<Pose> & plan_cm, const Pose & robot_cm, double * dev = nullptr) const;
  double pathDeviation(const Pose & robot_cm) const;
  bool goalNeedsHolonomic(const Pose & goal_cm, double approach_dir) const;
  double speed_scale_{1.0};

  // --- planner state
  std::mutex mutex_;
  nav_msgs::msg::Path global_plan_;          // in costmap frame
  bool have_global_plan_{false};
  Pose local_goal_;                          // current waypoint (costmap frame)
  std::vector<Pose> waypoints_;              // map frame; yaw = heading of the next segment, last = goal pose
  size_t wp_index_{0};
  double dist_tol_active_{0.02}, yaw_tol_active_{0.025}, orient_dist_active_{0.8};
  std::string path_state_{"straight"};       // straight | side_front | front_side | line | detour
  bool was_blocked_{false};
  bool try_side_{false};
  bool start_set_{false};
  bool start_is_escape_{false};
  Pose start_pose_;
  struct DetourInfo { int len{0}; int start{0}; int end{0}; bool empty{true}; } detour_info_;
  std::string detour_dir_;
  nav_msgs::msg::Path::SharedPtr detour_path_;   // raw (map frame)
  rclcpp::Time detour_path_time_;
  bool detour_path_time_set_{false};
  rclcpp::Time detour_enter_time_;
  int tick_counter_{0};

  // --- controller state
  std::vector<Pose> path_;                   // current local plan (costmap frame)
  bool is_oblique_{true};                    // explicit holonomic flag from planner
  std::string ctrl_state_{"IDLE"};
  int final_approach_counter_{0};
  bool has_init_cmd_time_{false};
  rclcpp::Time init_cmd_time_;
  bool has_rotate_start_time_{false};
  rclcpp::Time rotate_start_time_;
  double prev_linear_x_{0.0}, prev_linear_y_{0.0};
  bool goal_reached_{false};
};

}  // namespace midpro_nav2_controller

#endif  // MIDPRO_NAV2_CONTROLLER__MIDPRO_FSM_CONTROLLER_HPP_
