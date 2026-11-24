// midpro_local_path.cpp  (part 2: local path builder, ported from loas_local_planner.cpp)
#include "midpro_nav2_controller/midpro_fsm_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "nav2_core/exceptions.hpp"
#include "tf2/utils.h"

namespace midpro_nav2_controller
{

// setPlan: Nav2 hands over the global plan (map frame). Equivalent to a new goal_info in ROS1.
void MidproFsmController::setPlan(const nav_msgs::msg::Path & raw_path)
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (raw_path.poses.empty()) {
    throw nav2_core::PlannerException("MidproFsmController: received empty plan");
  }
  // v0.81: reshape the planner's point-robot path for the long swerve body before anything else looks at it
  nav_msgs::msg::Path path = raw_path;
  if (path_conditioner_enabled_ && useFootprint()) {conditionPlan(path);}
  // Plan hysteresis: a replan for the SAME goal is ignored while the current plan is still good
  // (robot on it, next plan_keep_check_dist metres footprint-feasible) and the new plan is not at least
  // plan_switch_gain shorter. Equal-cost alternatives (left/right of an obstacle) otherwise alternate.
  if (plan_hysteresis_ && have_global_plan_ && !global_plan_.poses.empty() && !path_.empty() && ctrl_state_ != "IDLE") {
    Pose g_new, g_old;
    nav_msgs::msg::Path incoming = path;
    for (auto & p : incoming.poses) {if (p.header.frame_id.empty()) {p.header.frame_id = path.header.frame_id;}}
    if (transformToCostmapFrame(incoming.poses.back(), g_new) && transformToCostmapFrame(global_plan_.poses.back(), g_old) &&
      dist(g_new, g_old) < goal_same_tol_)
    {
      std::vector<Pose> old_cm, new_cm;
      bool ok = true;
      for (const auto & p : global_plan_.poses) {Pose q; if (!transformToCostmapFrame(p, q)) {ok = false; break;} old_cm.push_back(q);}
      for (const auto & p : incoming.poses) {Pose q; if (!transformToCostmapFrame(p, q)) {ok = false; break;} new_cm.push_back(q);}
      if (ok) {
        const Pose & robot = path_[0];
        double dev_old = 0.0;
        const double len_old = remainingLength(old_cm, robot, &dev_old);
        const double len_new = remainingLength(new_cm, robot);
        bool old_feasible = dev_old < 0.5;
        if (old_feasible && !useFootprint()) {
          // circle model: the current plan is blocked if a pose ahead sits in a hard obstacle (99/100)
          size_t k0 = 0; double best = 1e18;
          for (size_t i = 0; i < old_cm.size(); ++i) {const double d = dist(robot, old_cm[i]); if (d < best) {best = d; k0 = i;}}
          double acc = 0.0;
          for (size_t i = k0 + 1; i < old_cm.size() && acc < plan_keep_check_dist_; ++i) {
            acc += dist(old_cm[i - 1], old_cm[i]);
            const int c = getCost(old_cm[i]);
            if (c != -999 && c > near_obstacle_value_) {old_feasible = false; break;}
          }
        }
        if (old_feasible && useFootprint()) {
          // walk the old plan from the robot for plan_keep_check_dist metres
          size_t k0 = 0; double best = 1e18;
          for (size_t i = 0; i < old_cm.size(); ++i) {const double d = dist(robot, old_cm[i]); if (d < best) {best = d; k0 = i;}}
          double acc = 0.0;
          for (size_t i = k0 + 1; i + 1 < old_cm.size() && acc < plan_keep_check_dist_; ++i) {
            acc += dist(old_cm[i - 1], old_cm[i]);
            const double tng = std::atan2(old_cm[i + 1].pose.position.y - old_cm[i].pose.position.y,
                old_cm[i + 1].pose.position.x - old_cm[i].pose.position.x);
            if (!footprintFree(old_cm[i].pose.position.x, old_cm[i].pose.position.y, tng, footprint_padding_)) {old_feasible = false; break;}
          }
        }
        if (!old_feasible && dev_old < 0.5) {
          if (!block_active_) {block_active_ = true; blocked_since_ = clock_->now();}
          if (blockAge() < block_persist_time_) {
            // keep waiting on the current plan, but remember the BT's alternative: once the wait expires we
            // switch to it directly (the BT will not resend a plan it already considers valid)
            pending_plan_ = path;
            for (auto & p : pending_plan_.poses) {if (p.header.frame_id.empty()) {p.header.frame_id = path.header.frame_id;}}
            has_pending_plan_ = true;
            RCLCPP_DEBUG(logger_, "[plan] current plan blocked for %.1f s < %.1f s: keep it, store the alternative", blockAge(), block_persist_time_);
            return;
          }
        }
        const bool drastic = len_new <= plan_switch_force_ratio_ * len_old;
        if (old_feasible && !plan_switching_ && !drastic) {
          // keep following the current plan, but remember the BT's latest plan: if a blockage appears ahead
          // (e.g. a pallet the global costmap saw 10 m away) we switch to it without waiting for a failure
          pending_plan_ = path;
          for (auto & p : pending_plan_.poses) {if (p.header.frame_id.empty()) {p.header.frame_id = path.header.frame_id;}}
          has_pending_plan_ = true;
          RCLCPP_DEBUG(logger_, "[plan] switching off: keep the current plan (%.1f m), new %.1f m stored", len_old, len_new);
          return;
        }
        if (old_feasible && len_new > (1.0 - plan_switch_gain_) * len_old) {
          // not a large gain. A small gain is accepted only when it persists over consecutive replans: a
          // left/right flip-flop alternates every replan, a real shortcut (e.g. an obstacle disappeared) stays.
          if (len_new < (1.0 - plan_switch_small_gain_) * len_old) {
            if (++shorter_count_ < plan_switch_confirm_) {
              RCLCPP_DEBUG(logger_, "[plan] shorter plan %.1f < %.1f m, waiting for confirmation (%d/%d)", len_new, len_old, shorter_count_, plan_switch_confirm_);
              return;
            }
          } else {
            shorter_count_ = 0;
            RCLCPP_DEBUG(logger_, "[plan] keep current plan (%.1f m) over new (%.1f m)", len_old, len_new);
            return;
          }
        }
        shorter_count_ = 0;
        RCLCPP_INFO(logger_, "[plan] switch: current %.1f m (%s), new %.1f m", len_old, old_feasible ? "feasible" : "blocked/left", len_new);
      }
    }
  }
  // keep the raw plan (map frame); used for detour following and re-transformed every tick.
  // Nav2 planners fill only the Path header, so propagate the frame to every pose.
  global_plan_ = path;
  plan_cm_valid_ = false; plan_hint_ = 0;
  has_pending_plan_ = false;
  for (auto & p : global_plan_.poses) {
    if (p.header.frame_id.empty()) {p.header.frame_id = path.header.frame_id;}
  }
  have_global_plan_ = true;

  // A new target (different final pose, or nothing in progress) restarts the waypoint sequence.
  // Periodic replans of the same goal (BT RateController) only refresh global_plan_.
  Pose new_goal;
  if (!transformToCostmapFrame(global_plan_.poses.back(), new_goal)) {new_goal = global_plan_.poses.back();}
  Pose cur_final;
  const bool have_wp = !waypoints_.empty() && transformToCostmapFrame(waypoints_.back(), cur_final);
  const bool new_target = !have_wp || ctrl_state_ == "IDLE" ||
    std::hypot(new_goal.pose.position.x - cur_final.pose.position.x,
      new_goal.pose.position.y - cur_final.pose.position.y) > goal_same_tol_;
  if (new_target) {
    buildWaypoints(global_plan_);
    wp_index_ = 0;
    resetPlannerState();
    resetControllerState();
    detour_clear_count_ = 0;
    align_active_ = false;
    following_hybrid_ = false;
    fresh_plan_ = true;
    shorter_count_ = 0;
    block_active_ = false; yield_cap_ = -1.0; has_pending_plan_ = false;
    stage_phase_ = 0; stage_evaluated_ = false; stage_best_d_ = 1e9;
    dist_tol_active_ = isLastWaypoint() ? dist_tolerance_ : waypoint_dist_tolerance_;
    yaw_tol_active_ = isLastWaypoint() ? yaw_tolerance_ : waypoint_yaw_tolerance_;
    orient_dist_active_ = isLastWaypoint() ? goal_orient_dist_ : waypoint_orient_dist_;
    setCtrlState("FOLLOW_PATH", "new plan: " + std::to_string(waypoints_.size()) + " waypoint(s)");
    RCLCPP_INFO(logger_, "[plan] goal yaw = %.1f deg (%s), goal (%.2f, %.2f), waypoints %zu",
      tf2::getYaw(path.poses.back().pose.orientation) * 180.0 / M_PI, path.header.frame_id.c_str(),
      path.poses.back().pose.position.x, path.poses.back().pose.position.y, waypoints_.size());
  } else if (have_wp && std::hypot(new_goal.pose.position.x - cur_final.pose.position.x,
    new_goal.pose.position.y - cur_final.pose.position.y) > 0.05)
  {
    // same goal, but the planner's goal tolerance moved the end (e.g. the exact goal sits just behind a thin
    // obstacle): follow the end of the accepted plan, keep the control state (no stop / reset)
    const size_t keep_index = wp_index_;
    buildWaypoints(global_plan_);
    wp_index_ = std::min(keep_index, waypoints_.empty() ? size_t(0) : waypoints_.size() - 1);
    RCLCPP_INFO(logger_, "[plan] same goal, end moved to (%.2f, %.2f) by the planner tolerance",
      path.poses.back().pose.position.x, path.poses.back().pose.position.y);
  }
  if (!waypoints_.empty()) {
    Pose g; if (transformToCostmapFrame(waypoints_[wp_index_], g)) {local_goal_ = g;}
  }
  if (speed_profile_enabled_) {buildSpeedProfile();}
}

// buildSpeedProfile: per-pose speed cap along the global plan, then forward/backward passes with the
// longitudinal acceleration limits so the result is the fastest profile that respects
//   v^2 * kappa <= a_lat,  obstacle proximity (costmap),  v <= sqrt(2 a_dec d_goal),  |d(v^2)/ds| <= 2a.
void MidproFsmController::buildSpeedProfile()
{
  profile_poses_.clear(); profile_v_.clear(); profile_hint_ = 0;
  const auto & P = global_plan_.poses;
  if (P.size() < 2) {return;}
  std::vector<Pose> cm; cm.reserve(P.size());
  for (const auto & p : P) {
    Pose q; if (!transformToCostmapFrame(p, q)) {return;}
    cm.push_back(q);
  }
  const size_t n = cm.size();
  std::vector<double> s(n, 0.0);
  for (size_t i = 1; i < n; ++i) {s[i] = s[i - 1] + dist(cm[i - 1], cm[i]);}
  const double total = s.back();
  const double vmax = max_linear_vel_ * speed_scale_;
  std::vector<double> v(n, vmax);
  // curvature from tangent change over +-window
  auto tangent = [&](size_t i) {
      const size_t a = (i == 0) ? 0 : i - 1, b = (i + 1 < n) ? i + 1 : i;
      return std::atan2(cm[b].pose.position.y - cm[a].pose.position.y, cm[b].pose.position.x - cm[a].pose.position.x);
    };
  for (size_t i = 0; i < n; ++i) {
    size_t a = i, b = i;
    while (a > 0 && s[i] - s[a] < profile_curv_window_) {--a;}
    while (b + 1 < n && s[b] - s[i] < profile_curv_window_) {++b;}
    const double ds = std::max(s[b] - s[a], 1e-3);
    const double kappa = std::abs(angleDiff(tangent(b), tangent(a))) / ds;
    if (kappa > 1e-3) {v[i] = std::min(v[i], std::sqrt(lateral_acc_limit_ / kappa));}
    // obstacle proximity along the plan (local costmap window only; -999 = outside -> no information)
    const int c = getCost(cm[i]);
    if (c >= 95) {v[i] = std::min(v[i], near_obstacle_speed_);} else if (c >= inflation_cost_threshold_) {v[i] = std::min(v[i], inflation_speed_);}
    v[i] = std::max(v[i], profile_min_speed_);
    // goal braking
    const double d_goal = total - s[i];
    v[i] = std::min(v[i], std::max(min_linear_vel_, std::sqrt(2.0 * linear_dec_limit_ * d_goal)));
  }
  v.back() = std::min(v.back(), min_linear_vel_);
  // forward (acceleration) and backward (deceleration) passes
  for (size_t i = 1; i < n; ++i) {
    const double ds = s[i] - s[i - 1];
    v[i] = std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2.0 * linear_acc_limit_ * ds));
  }
  for (size_t i = n - 1; i-- > 0;) {
    const double ds = s[i + 1] - s[i];
    v[i] = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] + 2.0 * linear_dec_limit_ * ds));
  }
  profile_poses_ = cm; profile_v_ = v;
  if (profile_pub_) {
    std_msgs::msg::Float32MultiArray m;
    m.data.reserve(2 * n);
    for (size_t i = 0; i < n; ++i) {m.data.push_back(static_cast<float>(s[i])); m.data.push_back(static_cast<float>(v[i]));}
    profile_pub_->publish(m);
  }
  double vmin = vmax; for (double x : v) {vmin = std::min(vmin, x);}
  RCLCPP_INFO(logger_, "[profile] %zu poses, %.1f m, min cap %.2f m/s (a_lat %.2f, a %.2f/%.2f)",
    n, total, vmin, lateral_acc_limit_, linear_acc_limit_, linear_dec_limit_);
}

// profileSpeedAt: speed cap at the plan pose nearest to the robot (search starts at the last hit)
double MidproFsmController::profileSpeedAt(const Pose & robot_cm) const
{
  if (profile_poses_.empty()) {return max_linear_vel_ * speed_scale_;}
  const size_t n = profile_poses_.size();
  size_t best = profile_hint_; double bd = dist(robot_cm, profile_poses_[best]);
  const size_t lo = (profile_hint_ > 40) ? profile_hint_ - 40 : 0;
  for (size_t i = lo; i < n; ++i) {
    const double d = dist(robot_cm, profile_poses_[i]);
    if (d < bd) {bd = d; best = i;}
    if (i > profile_hint_ + 200 && d > bd + 1.0) {break;}   // moved well past the best match
  }
  profile_hint_ = best;
  // off-path (side/front detour): widen the cap a little so the profile does not stall the robot
  if (bd > 0.5) {return std::max(profile_v_[best], near_obstacle_speed_);}
  return profile_v_[best];
}

// buildWaypoints: Douglas-Peucker on the global plan -> corner points. Intermediate waypoints take
// the heading of the following segment (so the FSM rotates at the corner), the last one the goal pose.
void MidproFsmController::buildWaypoints(const nav_msgs::msg::Path & plan_map)
{
  waypoints_.clear();
  const auto & P = plan_map.poses;
  const size_t n = P.size();
  if (n == 0) {return;}
  if (n < 3 || waypoint_mode_ != "corners") {
    Pose g = P.back(); g.header = plan_map.header; waypoints_.push_back(g); return;
  }
  std::vector<bool> keep(n, false);
  keep[0] = true; keep[n - 1] = true;
  std::vector<std::pair<size_t, size_t>> stack{{0, n - 1}};
  const double tol2 = waypoint_simplify_tol_ * waypoint_simplify_tol_;
  while (!stack.empty()) {
    auto [a, b] = stack.back(); stack.pop_back();
    if (b <= a + 1) {continue;}
    const double ax = P[a].pose.position.x, ay = P[a].pose.position.y;
    const double bx = P[b].pose.position.x, by = P[b].pose.position.y;
    const double vx = bx - ax, vy = by - ay, L2 = vx * vx + vy * vy;
    double best = 0.0; size_t idx = a;
    for (size_t i = a + 1; i < b; ++i) {
      const double px = P[i].pose.position.x - ax, py = P[i].pose.position.y - ay;
      double d2;
      if (L2 < 1e-12) {d2 = px * px + py * py;} else {
        const double t = std::clamp((px * vx + py * vy) / L2, 0.0, 1.0);
        const double ex = px - t * vx, ey = py - t * vy; d2 = ex * ex + ey * ey;
      }
      if (d2 > best) {best = d2; idx = i;}
    }
    if (best > tol2) {keep[idx] = true; stack.push_back({a, idx}); stack.push_back({idx, b});}
  }
  std::vector<size_t> kept;
  for (size_t i = 0; i < n; ++i) {if (keep[i]) {kept.push_back(i);}}
  // drop the first point (robot position); every remaining corner is a waypoint
  for (size_t k = 1; k < kept.size(); ++k) {
    Pose w = P[kept[k]]; w.header = plan_map.header;
    if (k + 1 < kept.size()) {
      const auto & nx = P[kept[k + 1]].pose.position;
      w.pose.orientation = quatFromYaw(std::atan2(nx.y - w.pose.position.y, nx.x - w.pose.position.x));
    } else {
      w.pose.orientation = P.back().pose.orientation;   // final goal keeps the requested yaw
    }
    waypoints_.push_back(w);
  }
  RCLCPP_INFO(logger_, "[waypoints] %zu corner(s) from %zu plan poses (tol %.2f m)",
    waypoints_.size(), n, waypoint_simplify_tol_);
}

// advanceWaypoint: called when the FSM reached the current waypoint. Returns false on the final goal.
bool MidproFsmController::advanceWaypoint()
{
  if (isLastWaypoint()) {return false;}
  ++wp_index_;
  resetPlannerState();
  path_.clear();   // controller idles until the planner tick builds the path to the next waypoint
  final_approach_counter_ = 0;
  has_init_cmd_time_ = false;
  has_rotate_start_time_ = false;
  dist_tol_active_ = isLastWaypoint() ? dist_tolerance_ : waypoint_dist_tolerance_;
  yaw_tol_active_ = isLastWaypoint() ? yaw_tolerance_ : waypoint_yaw_tolerance_;
  orient_dist_active_ = isLastWaypoint() ? goal_orient_dist_ : waypoint_orient_dist_;
  Pose g; if (transformToCostmapFrame(waypoints_[wp_index_], g)) {local_goal_ = g;}
  setCtrlState("FOLLOW_PATH", "waypoint " + std::to_string(wp_index_ + 1) + "/" + std::to_string(waypoints_.size()));
  return true;
}

// makeLocalPath: straight segment robot -> goal, poses every `interval`, orientation by orient_mode
bool MidproFsmController::makeLocalPath(const Pose & robot, const Pose & goal, nav_msgs::msg::Path & out)
{
  out.header.frame_id = costmap_frame_;
  out.header.stamp = clock_->now();
  const double rx = robot.pose.position.x, ry = robot.pose.position.y;
  const double gx = goal.pose.position.x, gy = goal.pose.position.y;
  const double dx = gx - rx, dy = gy - ry;
  const double distance = std::hypot(dx, dy);
  double ux = 0.0, uy = 0.0;
  if (distance > 1e-6) {ux = dx / distance; uy = dy / distance;}
  const int steps = std::max(1, static_cast<int>(distance / path_pose_interval_));

  geometry_msgs::msg::Quaternion orient;
  if (orient_mode_ == "robot") {
    orient = robot.pose.orientation;
  } else if (orient_mode_ == "goal") {
    orient = local_goal_.pose.orientation;
  } else {
    orient = quatFromYaw(std::atan2(dy, dx));
  }

  std::vector<Pose> rev;
  rev.push_back(goal);
  for (int i = 1; i < steps; ++i) {
    const double t = i * path_pose_interval_;
    Pose ps;
    ps.header.frame_id = costmap_frame_;
    ps.header.stamp = out.header.stamp;
    ps.pose.position.x = gx - ux * t;
    ps.pose.position.y = gy - uy * t;
    ps.pose.position.z = 0.0;
    ps.pose.orientation = orient;
    rev.push_back(ps);
  }
  rev.push_back(robot);
  std::reverse(rev.begin(), rev.end());
  if (!start_set_ && !start_is_escape_) {start_pose_ = rev[0]; start_set_ = true;}
  out.poses = rev;
  return true;
}

// findSideGoal / findFrontGoal: unchanged search geometry
bool MidproFsmController::findSideGoal(const Pose & robot, const Pose & goal, Pose & out) const
{
  const double yaw = yawOf(robot);
  const double gx = goal.pose.position.x, gy = goal.pose.position.y;
  const double lx = -std::sin(yaw), ly = std::cos(yaw);
  const double rx = std::sin(yaw), ry = -std::cos(yaw);
  for (double off = side_search_step_; off <= side_search_max_ + 1e-9; off += side_search_step_) {
    for (int d = 0; d < 2; ++d) {
      Pose sp = robot;
      sp.pose.position.x = robot.pose.position.x + (d == 0 ? lx : rx) * off;
      sp.pose.position.y = robot.pose.position.y + (d == 0 ? ly : ry) * off;
      if (!translationFree(robot, sp, yaw)) {continue;}
      if (!translationFree(sp, goal, yaw)) {continue;}
      const double diff = angleDiff(yaw, std::atan2(gy - sp.pose.position.y, gx - sp.pose.position.x));
      if (std::abs(diff) < M_PI / 36.0) {out = sp; return true;}
    }
  }
  return false;
}

bool MidproFsmController::findFrontGoal(const Pose & robot, const Pose & goal, Pose & out) const
{
  const double yaw = yawOf(robot);
  const double gx = goal.pose.position.x, gy = goal.pose.position.y;
  const double fx = std::cos(yaw), fy = std::sin(yaw);
  for (double off = side_search_step_; off <= side_search_max_ + 1e-9; off += side_search_step_) {
    Pose fp = robot;
    fp.pose.position.x = robot.pose.position.x + fx * off;
    fp.pose.position.y = robot.pose.position.y + fy * off;
    if (!translationFree(robot, fp, yaw)) {continue;}
    const int cost = getCost(fp);
    if (cost == -999 || cost >= near_obstacle_value_) {continue;}
    if (!translationFree(fp, goal, yaw)) {continue;}
    const double diff = angleDiff(yaw, std::atan2(gy - fp.pose.position.y, gx - fp.pose.position.x));
    if (M_PI / 2.0 - M_PI / 180.0 < std::abs(diff) && std::abs(diff) < M_PI / 2.0 + M_PI / 180.0) {
      out = fp; return true;
    }
  }
  return false;
}

// makeDetourPath: ex makeEbandPath. Transforms the latest detour path (map frame) into the costmap frame.
nav_msgs::msg::Path MidproFsmController::makeDetourPath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_frame_;
  path.header.stamp = clock_->now();
  if (!detour_path_time_set_ || !detour_path_) {return path;}
  const double age = (clock_->now() - detour_path_time_).seconds();
  if (age > detour_path_max_age_) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "[detour] path stale (%.1fs)", age);
    return path;
  }
  std::vector<Pose> trans;
  trans.reserve(detour_path_->poses.size());
  for (const auto & p : detour_path_->poses) {
    Pose q;
    Pose in = p; in.header = detour_path_->header;
    if (!transformToCostmapFrame(in, q)) {return path;}
    trans.push_back(q);
  }
  if (!trans.empty()) {
    if (!start_set_ && !start_is_escape_) {start_pose_ = trans[0]; start_set_ = true;}
    path.poses = trans;
  }
  return path;
}

// finalizeLocalPath: ex publishLocalPath. Fills pose orientations (holonomic vs ackermann rules),
// stores the result for the controller and publishes it for visualisation.
void MidproFsmController::finalizeLocalPath(nav_msgs::msg::Path & local_path, bool modify_orient, bool detour)
{
  const int n = static_cast<int>(local_path.poses.size());

  if (modify_orient && n > 0) {
    int start_idx = 0;
    for (int i = 0; i < n; ++i) {
      const int c = getCost(local_path.poses[i]);
      if (c == -999 || c < near_obstacle_value_ || i == n - 1) {start_idx = i; break;}
    }
    int end_idx = n - 1;
    for (int i = n - 1; i >= 0; --i) {
      const int c = getCost(local_path.poses[i]);
      if (c == -999 || c < near_obstacle_value_ || i == 0) {end_idx = i; break;}
    }

    if (detour) {
      if (n < 2) {
        path_ = local_path.poses; is_oblique_ = true;
        local_plan_pub_->publish(local_path);
        return;
      }
      if (start_set_ && !start_is_escape_) {
        const double yaw = yawOf(local_path.poses[0]);
        const double ddx = local_path.poses[1].pose.position.x - local_path.poses[0].pose.position.x;
        const double ddy = local_path.poses[1].pose.position.y - local_path.poses[0].pose.position.y;
        const double diff = angleDiff(std::atan2(ddy, ddx), yaw);
        const double ed = M_PI / 18.0, eD = 0.6;
        const double d0 = dist(start_pose_, local_path.poses[0]);
        if (d0 > eD || std::abs(diff) < ed) {
          start_is_escape_ = true; start_set_ = false; start_idx = 0;
        }
      }
      if (start_idx == 0 && !start_is_escape_) {start_is_escape_ = true; start_set_ = false;}
      const int offset = 2;
      if (start_idx != 0) {start_idx = std::min(start_idx + offset, n - 1);}
      if (end_idx != n - 1 && end_idx - offset >= 0) {end_idx -= offset;}
      if (detour_info_.empty) {
        detour_info_ = {n, start_idx, end_idx, false};
      } else if (detour_info_.len == n) {
        start_idx = std::max(start_idx, detour_info_.start); detour_info_.start = start_idx;
        end_idx = std::min(end_idx, detour_info_.end); detour_info_.end = end_idx;
      } else if (detour_info_.len > n) {
        detour_info_ = {n, start_idx, end_idx, false};
      } else {
        start_idx = detour_info_.start; end_idx = detour_info_.end;
      }
      if (start_is_escape_) {start_idx = 0;}

      const double yaw = yawOf(local_path.poses[0]);
      const double ddx = local_path.poses[1].pose.position.x - local_path.poses[0].pose.position.x;
      const double ddy = local_path.poses[1].pose.position.y - local_path.poses[0].pose.position.y;
      const double yaw_err = angleDiff(std::atan2(ddy, ddx), yaw);
      const int cost0 = getCost(local_path.poses[0]);

      // escape run: consecutive poses from the robot whose cell is NEAR (95) or worse keep the robot yaw,
      // so the FSM slides out holonomically (parallel / diagonal / backwards) instead of rotating in a bay.
      // ROS1 applied this only in the forward case; here it applies to every case below.
      int escape_end = 0;
      if (no_rotate_in_near_ && escape_active_ && cost0 != -999 && cost0 >= near_obstacle_value_) {
        for (int i = 1; i < n - 1; ++i) {
          const int ci = getCost(local_path.poses[i]);
          if (ci == -999 || ci < near_obstacle_value_) {break;}
          escape_end = i;
        }
        escape_end = std::min(escape_end + 4, n - 2);   // + offset so the heading reference stays inside the run
      }
      if (cost0 != -999 && cost0 > near_obstacle_value_) {
        // robot sits next to an obstacle: keep current heading, slide out holonomically
        for (int i = 1; i < n - 1; ++i) {
          local_path.poses[i].pose.orientation = local_path.poses[0].pose.orientation;
        }
      } else {
        if (cost0 != -999 && cost0 < near_obstacle_value_) {detour_dir_ = "forward";}
        if (std::abs(yaw_err) < M_PI / 2.0 || detour_dir_ == "forward") {
          for (int i = 1; i < n - 1; ++i) {
            if ((i <= start_idx && !start_is_escape_) || i <= escape_end) {
              local_path.poses[i].pose.orientation = local_path.poses[0].pose.orientation;
            } else {
              const double dx2 = local_path.poses[i + 1].pose.position.x - local_path.poses[i].pose.position.x;
              const double dy2 = local_path.poses[i + 1].pose.position.y - local_path.poses[i].pose.position.y;
              local_path.poses[i].pose.orientation = quatFromYaw(std::atan2(dy2, dx2));
            }
          }
        } else {
          for (int i = n - 2; i > 0; --i) {
            if (std::abs(yaw_err) > M_PI / 180.0 * 140.0 && i > escape_end) {
              const double dx2 = -(local_path.poses[i + 1].pose.position.x - local_path.poses[i].pose.position.x);
              const double dy2 = -(local_path.poses[i + 1].pose.position.y - local_path.poses[i].pose.position.y);
              local_path.poses[i].pose.orientation = quatFromYaw(std::atan2(dy2, dx2));
            } else {
              local_path.poses[i].pose.orientation = local_path.poses[0].pose.orientation;
            }
          }
        }
      }
    } else {
      // straight / line
      if (start_idx == 0 && !start_is_escape_) {start_is_escape_ = true; start_set_ = false;}
      const int offset = 4;
      if (start_idx != 0) {start_idx = (start_idx + offset < n) ? start_idx + offset : n - 1;}
      if (end_idx != n - 1) {
        if (end_idx - offset > 0) {end_idx -= offset;} else {start_idx = 0; end_idx = 0;}
      }
      if (start_is_escape_) {start_idx = 0;}
      bool near_goal = dist(local_path.poses[0], local_path.poses[n - 1]) < orient_dist_active_;
      if (near_goal && holonomic_only_when_needed_) {
        // open space + goal yaw roughly along the approach: drive in aligned (tangent yaw), let FINAL_ROTATE trim
        const double approach = std::atan2(local_path.poses[n - 1].pose.position.y - local_path.poses[0].pose.position.y,
            local_path.poses[n - 1].pose.position.x - local_path.poses[0].pose.position.x);
        near_goal = goalNeedsHolonomic(local_path.poses[n - 1], approach);
      }
      for (int i = 1; i < n - 1; ++i) {
        if (i <= start_idx && start_idx != n - 1) {
          local_path.poses[i].pose.orientation = local_path.poses[0].pose.orientation;
        } else if (i >= end_idx && (near_goal || !holonomic_only_when_needed_)) {
          local_path.poses[i].pose.orientation = local_path.poses[n - 1].pose.orientation;
        } else if (near_goal) {
          local_path.poses[i].pose.orientation = local_path.poses[n - 1].pose.orientation;
        } else {
          const double dx2 = local_path.poses[i + 1].pose.position.x - local_path.poses[i].pose.position.x;
          const double dy2 = local_path.poses[i + 1].pose.position.y - local_path.poses[i].pose.position.y;
          local_path.poses[i].pose.orientation = quatFromYaw(std::atan2(dy2, dx2));
        }
      }
    }
  } else {
    // side_front / front_side: poses keep the orient_mode orientation set in makeLocalPath
  }

  if (!local_path.poses.empty()) {
    local_path.poses.front().pose.position.z = 0.0;
    local_path.poses.back().pose.position.z = 0.0;
  }
  path_ = local_path.poses;
  // Holonomic/ackermann decision, same rule as ROS1 loas_controller but computed once here,
  // at path generation, with an angular tolerance instead of the 1e-10 compare:
  //   yaw(path[1]) == yaw(path[2])  or  path has <= 3 poses  ->  holonomic (is_oblique)
  //   otherwise (curved tangent yaws)                        ->  ackermann
  if (path_.size() <= 3) {
    is_oblique_ = true;
  } else {
    is_oblique_ = std::abs(angleDiff(yawOf(path_[1]), yawOf(path_[2]))) < oblique_yaw_tolerance_;
  }
  std_msgs::msg::String m;
  m.data = path_state_ + "/" + (is_oblique_ ? "holonomic" : "ackermann");
  path_mode_pub_->publish(m);
  local_plan_pub_->publish(local_path);
}

// pickDetourGoal: walk the global plan from the pose nearest to the robot and return (map frame)
// the last pose within detour_goal_lookahead_ metres of path length.
bool MidproFsmController::pickDetourGoal(const Pose & robot_cm, geometry_msgs::msg::PoseStamped & goal_map) const
{
  const auto & poses = global_plan_.poses;
  if (poses.empty()) {return false;}
  std::vector<Pose> cm; cm.reserve(poses.size());
  for (const auto & p : poses) {
    Pose q; if (!transformToCostmapFrame(p, q)) {return false;}
    cm.push_back(q);
  }
  size_t nearest = 0; double best = 1e18;
  for (size_t i = 0; i < cm.size(); ++i) {
    const double d = dist(robot_cm, cm[i]);
    if (d < best) {best = d; nearest = i;}
  }
  double acc = 0.0; size_t pick = nearest;
  for (size_t i = nearest + 1; i < cm.size(); ++i) {
    acc += dist(cm[i - 1], cm[i]);
    if (acc > detour_goal_lookahead_) {break;}
    pick = i;
  }
  goal_map = poses[pick];
  goal_map.header = global_plan_.header;
  return true;
}

// makeGlobalPlanPath: robot pose + the remainder of the Nav2 global plan (costmap frame).
nav_msgs::msg::Path MidproFsmController::makeGlobalPlanPath(const Pose & robot_cm) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_frame_;
  path.header.stamp = clock_->now();
  const auto & cm = planCm();
  if (cm.empty()) {return path;}
  double best = 1e18;
  const size_t nearest = nearestPlanIndex(cm, robot_cm, best);
  // end at the plan pose closest to the current waypoint (ROS1 eband paths ended at the goal_info)
  size_t stop = cm.size() - 1; double bg = 1e18;
  for (size_t i = nearest; i < cm.size(); ++i) {
    const double dd = dist(local_goal_, cm[i]);
    if (dd < bg) {bg = dd; stop = i;}
  }
  path.poses.push_back(robot_cm);
  for (size_t i = nearest; i <= stop; ++i) {
    if (dist(path.poses.back(), cm[i]) < 0.5 * path_pose_interval_) {continue;}  // drop duplicates
    path.poses.push_back(cm[i]);
  }
  return path;
}

// goalNeedsHolonomic: hold the goal yaw on the final approach (holonomic slide-in) only when the goal
// sits next to obstacles (bay, rack) or the goal yaw differs from the approach direction (side parking).
bool MidproFsmController::goalNeedsHolonomic(const Pose & goal_cm, double approach_dir) const
{
  if (side_parking_holonomic_ && std::abs(angleDiff(yawOf(goal_cm), approach_dir)) > goal_side_angle_) {return true;}
  const double r = goal_obstacle_check_radius_;
  for (double dx = -r; dx <= r; dx += 0.2) {
    for (double dy = -r; dy <= r; dy += 0.2) {
      if (dx * dx + dy * dy > r * r) {continue;}
      Pose p = goal_cm; p.pose.position.x += dx; p.pose.position.y += dy;
      const int c = getCost(p);
      if (c >= near_obstacle_value_) {return true;}
    }
  }
  return false;
}

// pathDeviation: distance from the robot to the nearest pose of the global plan (costmap frame).
double MidproFsmController::pathDeviation(const Pose & robot_cm) const
{
  const auto & cm = planCm();
  if (cm.empty()) {return 0.0;}
  double best = 1e18;
  nearestPlanIndex(cm, robot_cm, best);
  return (best > 1e17) ? 0.0 : best;
}

// planCm: the global plan transformed into the costmap frame. Transforming thousands of poses on every tick
// (several callers) made the 20 Hz loop miss its rate on long plans; refresh at most every 0.5 s instead
// (map->odom moves slowly) and whenever a new plan is accepted.
const std::vector<MidproFsmController::Pose> & MidproFsmController::planCm() const
{
  const rclcpp::Time now = clock_->now();
  if (plan_cm_valid_ && (now - plan_cm_stamp_).seconds() < 0.5) {return plan_cm_;}
  std::vector<Pose> cm; cm.reserve(global_plan_.poses.size());
  for (const auto & p : global_plan_.poses) {
    Pose q;
    if (!transformToCostmapFrame(p, q)) {return plan_cm_;}   // keep the previous cache on a TF hiccup
    cm.push_back(q);
  }
  plan_cm_.swap(cm); plan_cm_stamp_ = now; plan_cm_valid_ = true;
  return plan_cm_;
}

// nearestPlanIndex: search around the last hit first (the robot advances along the plan); full search if lost.
size_t MidproFsmController::nearestPlanIndex(const std::vector<Pose> & cm, const Pose & robot_cm, double & best) const
{
  const size_t n = cm.size();
  size_t bi = 0; best = 1e18;
  if (n == 0) {return 0;}
  const size_t lo = (plan_hint_ > 200) ? plan_hint_ - 200 : 0;
  const size_t hi = std::min(n, plan_hint_ + 600);
  for (size_t i = lo; i < hi; ++i) {const double d = dist(robot_cm, cm[i]); if (d < best) {best = d; bi = i;}}
  if (best > 2.0) {   // lost (new plan, teleport): full search
    for (size_t i = 0; i < n; ++i) {const double d = dist(robot_cm, cm[i]); if (d < best) {best = d; bi = i;}}
  }
  plan_hint_ = bi;
  return bi;
}

// remainingLength: arc length of a plan from the pose nearest to the robot to its end (optionally the robot's distance to it)
double MidproFsmController::remainingLength(const std::vector<Pose> & plan_cm, const Pose & robot_cm, double * dev) const
{
  if (plan_cm.empty()) {return 0.0;}
  size_t k0 = 0; double best = 1e18;
  for (size_t i = 0; i < plan_cm.size(); ++i) {const double d = dist(robot_cm, plan_cm[i]); if (d < best) {best = d; k0 = i;}}
  if (dev) {*dev = best;}
  double len = best;
  for (size_t i = k0 + 1; i < plan_cm.size(); ++i) {len += dist(plan_cm[i - 1], plan_cm[i]);}
  return len;
}

// globalPlanComfortable: the global plan ahead (comfort_check_dist_) keeps the comfortable clearance
// (footprint + straight_clearance at the plan tangent; circle model: centre cost below NEAR).
// Leading poses where the robot is still squeezed are skipped.
bool MidproFsmController::globalPlanComfortable(const Pose & robot_cm) const
{
  nav_msgs::msg::Path gp = makeGlobalPlanPath(robot_cm);
  if (gp.poses.size() < 3) {return false;}
  const double pad = std::max({2.0 * footprint_padding_, straight_clearance_, tight_clearance_});
  double acc = 0.0;
  bool started = false;
  for (size_t i = 1; i + 1 < gp.poses.size(); ++i) {
    acc += dist(gp.poses[i - 1], gp.poses[i]);
    if (acc > comfort_check_dist_) {break;}
    bool ok;
    if (useFootprint()) {
      const double tng = std::atan2(gp.poses[i + 1].pose.position.y - gp.poses[i].pose.position.y,
          gp.poses[i + 1].pose.position.x - gp.poses[i].pose.position.x);
      ok = footprintFree(gp.poses[i].pose.position.x, gp.poses[i].pose.position.y, tng, pad);
    } else {
      const int c = getCost(gp.poses[i]);
      ok = (c != -999 && c < near_obstacle_value_);
    }
    if (!started) {
      if (ok) {started = true; continue;}
      if (acc < 0.5) {continue;}
      return false;
    }
    if (!ok) {return false;}
  }
  return started;
}

// blockAge: seconds the followed plan has been continuously blocked (0 when clear)
double MidproFsmController::blockAge() const
{
  return block_active_ ? (clock_->now() - blocked_since_).seconds() : 0.0;
}

// adoptPendingPlan: the followed plan stayed blocked for block_persist_time -> switch to the alternative the BT
// offered while we were waiting (no FollowPath failure, no recovery behaviours, no extra stop).
void MidproFsmController::adoptPendingPlan()
{
  const double age = blockAge();
  global_plan_ = pending_plan_;
  has_pending_plan_ = false;
  plan_cm_valid_ = false; plan_hint_ = 0;
  block_active_ = false; block_moving_ = false;
  const size_t keep_index = wp_index_;
  buildWaypoints(global_plan_);
  wp_index_ = std::min(keep_index, waypoints_.empty() ? size_t(0) : waypoints_.size() - 1);
  if (!waypoints_.empty()) {
    Pose g; if (transformToCostmapFrame(waypoints_[wp_index_], g)) {local_goal_ = g;}
  }
  if (speed_profile_enabled_) {buildSpeedProfile();}
  RCLCPP_INFO(logger_, "[yield] blocked for %.1f s -> taking the alternative plan (%zu poses)", age, global_plan_.poses.size());
}

// blockingObstaclePoint: the lethal cell nearest to the robot inside the (padded) footprint placed at p/yaw.
bool MidproFsmController::blockingObstaclePoint(const Pose & p, double yaw, const Pose & robot, double & ox, double & oy) const
{
  if (!costmap_) {return false;}
  double fx = 0.42, rx = -0.48, hw = 0.25;
  if (!fp_base_.empty()) {
    fx = -1e9; rx = 1e9; hw = 0.0;
    for (const auto & q : fp_base_) {fx = std::max(fx, q.x); rx = std::min(rx, q.x); hw = std::max(hw, std::abs(q.y));}
  }
  const double pad = useFootprint() ? footprint_padding_ : 0.05;
  const double R = std::hypot(std::max(fx, -rx), hw) + pad;
  const double res = costmap_->getResolution();
  const double cx = p.pose.position.x, cy = p.pose.position.y, c = std::cos(yaw), s = std::sin(yaw);
  double best = 1e18; bool found = false;
  for (double dx = -R; dx <= R; dx += res) {
    for (double dy = -R; dy <= R; dy += res) {
      const double wx = cx + dx, wy = cy + dy;
      unsigned int mx, my;
      if (!costmap_->worldToMap(wx, wy, mx, my)) {continue;}
      if (costmap_->getCost(mx, my) != nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
      const double lx = c * dx + s * dy, ly = -s * dx + c * dy;          // cell in the footprint frame
      if (lx > fx + pad || lx < rx - pad || std::abs(ly) > hw + pad) {continue;}
      const double d = std::hypot(wx - robot.pose.position.x, wy - robot.pose.position.y);
      if (d < best) {best = d; ox = wx; oy = wy; found = true;}
    }
  }
  return found;
}

// pendingAvoids: the stored alternative does not run through the current blockage (it is not just the same
// route re-planned before the obstacle appeared). Checks the first ~20 m of it.
bool MidproFsmController::pendingAvoids(double ox, double oy) const
{
  if (!has_pending_plan_ || pending_plan_.poses.empty()) {return false;}
  double hw = 0.25;
  for (const auto & q : fp_base_) {hw = std::max(hw, std::abs(q.y));}
  const size_t n = std::min<size_t>(pending_plan_.poses.size(), 400);
  for (size_t i = 0; i < n; ++i) {
    Pose q;
    if (!transformToCostmapFrame(pending_plan_.poses[i], q)) {return false;}
    if (std::hypot(q.pose.position.x - ox, q.pose.position.y - oy) < hw + 0.10) {return false;}
  }
  return true;
}

// blockEvent: JSON event for the dashboard / tuning report / regression runner (/midpro/block_event)
void MidproFsmController::blockEvent(const std::string & ev, double at, double age, const std::string & why) const
{
  if (!block_event_pub_) {return;}
  std_msgs::msg::String m;
  char buf[256];
  std::snprintf(buf, sizeof(buf), "{\"ev\":\"%s\",\"at\":%.2f,\"age\":%.2f,\"why\":\"%s\",\"x\":%.2f,\"y\":%.2f}",
    ev.c_str(), at, age, why.c_str(), block_ref_x_, block_ref_y_);
  m.data = buf;
  block_event_pub_->publish(m);
}

// nearestObstacleAround: the lethal cell nearest to the robot body (within reach of the footprint edge).
bool MidproFsmController::nearestObstacleAround(const Pose & robot, double reach, double & ox, double & oy) const
{
  if (!costmap_) {return false;}
  double fx = 0.42, rx = -0.48, hw = 0.25;
  if (!fp_base_.empty()) {
    fx = -1e9; rx = 1e9; hw = 0.0;
    for (const auto & q : fp_base_) {fx = std::max(fx, q.x); rx = std::min(rx, q.x); hw = std::max(hw, std::abs(q.y));}
  }
  const double R = std::hypot(std::max(fx, -rx), hw) + reach;
  const double res = costmap_->getResolution();
  const double cx = robot.pose.position.x, cy = robot.pose.position.y, yw = yawOf(robot);
  const double c = std::cos(yw), s = std::sin(yw);
  double best = 1e18; bool found = false;
  for (double dx = -R; dx <= R; dx += res) {
    for (double dy = -R; dy <= R; dy += res) {
      unsigned int mx, my;
      if (!costmap_->worldToMap(cx + dx, cy + dy, mx, my)) {continue;}
      if (costmap_->getCost(mx, my) != nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
      const double lx = c * dx + s * dy, ly = -s * dx + c * dy;           // body frame
      const double ex = std::max({lx - fx, rx - lx, 0.0}), ey = std::max(std::abs(ly) - hw, 0.0);
      const double d = std::hypot(ex, ey);                                 // distance to the footprint edge
      if (d <= reach && d < best) {best = d; ox = cx + dx; oy = cy + dy; found = true;}
    }
  }
  return found;
}

// segmentFree: the footprint (+pad), aligned with the segment, fits all along the straight line a -> b.
bool MidproFsmController::segmentFree(const Pose & a, const Pose & b, double pad) const
{
  const double x0 = a.pose.position.x, y0 = a.pose.position.y;
  const double x1 = b.pose.position.x, y1 = b.pose.position.y;
  const double len = std::hypot(x1 - x0, y1 - y0);
  if (len < 1e-3) {return true;}
  const double yaw = std::atan2(y1 - y0, x1 - x0);
  const double step = costmap_ ? 2.0 * costmap_->getResolution() : 0.05;
  const int n = std::max(1, static_cast<int>(len / step));
  for (int k = 1; k <= n; ++k) {
    const double t = static_cast<double>(k) / n;
    if (!footprintFree(x0 + t * (x1 - x0), y0 + t * (y1 - y0), yaw, pad)) {return false;}
  }
  return true;
}

// shortcutPath: from the robot, find the farthest pose of ep within shortcut_dist_ that a straight, segment-aligned
// footprint (+2 x padding) reaches, and replace everything before it by that straight line.
void MidproFsmController::shortcutPath(nav_msgs::msg::Path & ep) const
{
  if (!useFootprint() || shortcut_dist_ <= 0.0 || ep.poses.size() < 4) {return;}
  const Pose & r = ep.poses.front();
  // the robot itself must not overlap anything (it may already be inside a tight gap: no padding here)
  if (!footprintFree(r.pose.position.x, r.pose.position.y, yawOf(r), 0.0)) {return;}
  std::vector<size_t> cand;
  double acc = 0.0;
  for (size_t i = 1; i < ep.poses.size(); ++i) {
    acc += dist(ep.poses[i - 1], ep.poses[i]);
    if (acc > shortcut_dist_) {break;}
    cand.push_back(i);
  }
  // margin levels: comfortable first; in a gap of width + 10 cm only the smaller margins fit, and a straight
  // stretch along the gap axis is exactly what a long body needs there (a curve inside it wedges the body)
  size_t best = 0;
  for (const double pad : {tight_clearance_, footprint_padding_, 0.01}) {
    for (size_t k = cand.size(); k > 0; k = (k > 5 ? k - 5 : 0)) {     // farthest first, 25 cm strides
      const size_t j = cand[k - 1];
      if (j <= best) {break;}
      if (segmentFree(r, ep.poses[j], pad)) {best = j; break;}
    }
    if (best >= 3) {break;}
  }
  if (best < 3) {return;}
  nav_msgs::msg::Path out; out.header = ep.header;
  out.poses.push_back(r);
  const double len = dist(r, ep.poses[best]);
  const int n = std::max(1, static_cast<int>(len / std::max(path_pose_interval_, 0.02)));
  for (int k = 1; k < n; ++k) {
    Pose p = r; const double t = static_cast<double>(k) / n;
    p.pose.position.x = r.pose.position.x + t * (ep.poses[best].pose.position.x - r.pose.position.x);
    p.pose.position.y = r.pose.position.y + t * (ep.poses[best].pose.position.y - r.pose.position.y);
    out.poses.push_back(p);
  }
  out.poses.insert(out.poses.end(), ep.poses.begin() + static_cast<long>(best), ep.poses.end());
  ep = out;
}

// ======================================================================= orthogonal manoeuvre (v0.68)
// forwardBlockDist: distance the footprint (heading `yaw`, +pad) can translate straight along `yaw` from `from`.
double MidproFsmController::forwardBlockDist(const Pose & from, double yaw, double max_d, double pad) const
{
  const double step = costmap_ ? 2.0 * costmap_->getResolution() : 0.05;
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (double d = step; d <= max_d + 1e-9; d += step) {
    if (!footprintFree(from.pose.position.x + d * c, from.pose.position.y + d * s, yaw, pad)) {return d;}
  }
  return std::numeric_limits<double>::infinity();
}

// lateralFree: the footprint keeps heading `yaw` and translates from a to b without touching anything.
bool MidproFsmController::lateralFree(const Pose & a, const Pose & b, double yaw, double pad) const
{
  const double step = costmap_ ? 2.0 * costmap_->getResolution() : 0.05;
  const double dx = b.pose.position.x - a.pose.position.x, dy = b.pose.position.y - a.pose.position.y;
  const double len = std::hypot(dx, dy);
  const int n = std::max(1, static_cast<int>(len / step));
  for (int k = 1; k <= n; ++k) {
    const double t = static_cast<double>(k) / n;
    if (!footprintFree(a.pose.position.x + t * dx, a.pose.position.y + t * dy, yaw, pad)) {return false;}
  }
  return true;
}

// orthTrigger: an obstacle is close straight ahead, the goal lies beyond it, and a pure sideways shift
// (heading held) followed by a straight run reaches a point from which the goal is in comfortable line of sight.
bool MidproFsmController::orthTrigger(const Pose & robot, const Pose & goal)
{
  if (!orthogonal_avoid_ || !useFootprint() || following_hybrid_) {return false;}
  if (ctrl_state_ != "FOLLOW_PATH" && ctrl_state_ != "ROTATE_IN_PLACE") {return false;}
  if (block_moving_) {return false;}                                    // a person: yield, do not side-step
  if (orth_ended_once_ && (clock_->now() - orth_end_t_).seconds() < 2.0) {return false;}   // no immediate re-trigger
  if (orth_eval_once_ && (clock_->now() - orth_last_eval_).seconds() < 0.5) {return false;}   // search at 2 Hz
  orth_last_eval_ = clock_->now(); orth_eval_once_ = true;
  const double gd = dist(robot, goal);
  if (gd < holonomic_zone_ + 0.5) {return false;}                       // the goal zone has its own logic
  const double yaw0 = yawOf(robot), c = std::cos(yaw0), s = std::sin(yaw0);
  const double gx = goal.pose.position.x - robot.pose.position.x, gy = goal.pose.position.y - robot.pose.position.y;
  const double gf = c * gx + s * gy, gl = -s * gx + c * gy;              // goal in the robot frame
  if (gf < 1.0 || std::abs(std::atan2(gl, gf)) > 1.2) {return false;}   // goal must be ahead (within ~70 deg)
  const double pad = footprint_padding_;
  const bool stopped = meas_speed_ < 0.05;
  const double look = std::min(gf, stopped ? 2.5 : orth_trigger_dist_);
  const double sb = forwardBlockDist(robot, yaw0, look, pad);
  if (!std::isfinite(sb)) {return false;}                               // nothing close ahead
  // pure sideways shift: smallest d per side that gives a free straight run to a goal-visible point
  double best_d = 1e9, best_L = 0.0; int best_side = 0;
  for (const int side : {gl >= 0.0 ? 1 : -1, gl >= 0.0 ? -1 : 1}) {
    for (double d = 0.10; d <= orth_max_shift_ + 1e-9 && d < best_d; d += 0.05) {
      Pose p1 = robot;
      p1.pose.position.x += side * d * (-s); p1.pose.position.y += side * d * c;
      if (!lateralFree(robot, p1, yaw0, pad)) {break;}                  // blocked sideways: give up this side
      const double run = forwardBlockDist(p1, yaw0, gf, 2.0 * pad);
      // the straight run must pass the obstacle and end where the goal is in comfortable line of sight
      const double Lmax = std::min({run, gf, sb + 4.0});
      for (double L = sb + 0.3; L <= Lmax + 1e-9; L += 0.3) {
        Pose p2 = p1; p2.pose.position.x += L * c; p2.pose.position.y += L * s;
        int blk; double bx2, by2;
        // passable line of sight is enough here: after the corner the normal logic picks straight or curve
        if (!getBlockingCell(p2, goal, false, blk, bx2, by2)) {
          if (d < best_d) {best_d = d; best_L = L; best_side = side;}
          break;
        }
      }
      if (best_side == side) {break;}
    }
  }
  if (best_side == 0) {return false;}
  orth_yaw_ = yaw0;
  orth_p1_ = robot;
  orth_p1_.pose.position.x += best_side * best_d * (-s); orth_p1_.pose.position.y += best_side * best_d * c;
  orth_p2_ = orth_p1_;
  orth_p2_.pose.position.x += best_L * c; orth_p2_.pose.position.y += best_L * s;
  orth_phase_ = 1; orth_t0_ = clock_->now(); orth_phase_t_ = orth_t0_;
  char buf[160];
  std::snprintf(buf, sizeof(buf), "obstacle %.1f m ahead -> shift %s %.2f m, then straight %.1f m",
    sb, best_side > 0 ? "left" : "right", best_d, best_L);
  setPathState("orth_side", buf);
  blockEvent("orth", sb, 0.0, best_side > 0 ? "left" : "right");
  return true;
}

// orthTick: phase handling and the local path shown in RViz (robot -> corner -> straight -> goal).
void MidproFsmController::orthTick(const Pose & robot, const Pose & goal)
{
  const auto now = clock_->now();
  if ((now - orth_t0_).seconds() > 25.0) {orthEnd("timeout"); return;}
  if (orth_phase_ == 1) {
    if (dist(robot, orth_p1_) < 0.04) {
      orth_phase_ = 2; orth_phase_t_ = now;
      setPathState("orth_front", "side shift done -> straight");
    } else if (!lateralFree(robot, orth_p1_, orth_yaw_, 0.0)) {
      orthEnd("side blocked"); return;
    }
  }
  if (orth_phase_ == 2) {
    const double rem = dist(robot, orth_p2_);
    int blk; double bx, by;
    const double progress = (robot.pose.position.x - orth_p1_.pose.position.x) * std::cos(orth_yaw_) +
      (robot.pose.position.y - orth_p1_.pose.position.y) * std::sin(orth_yaw_);
    if (rem < 0.08) {orthEnd("done"); return;}
    if (progress > 0.3 && !getBlockingCell(robot, goal, false, blk, bx, by)) {orthEnd("goal in sight"); return;}
    if (std::isfinite(forwardBlockDist(robot, orth_yaw_, rem, 0.0))) {orthEnd("straight blocked"); return;}
  }
  orthPublishPath(robot, goal);
}

void MidproFsmController::orthEnd(const std::string & why)
{
  orth_phase_ = 0; orth_end_t_ = clock_->now(); orth_ended_once_ = true;
  setPathState("straight", "orthogonal manoeuvre: " + why);
}

void MidproFsmController::orthPublishPath(const Pose & robot, const Pose & goal)
{
  nav_msgs::msg::Path lp;
  lp.header.frame_id = costmap_ros_->getGlobalFrameID();
  lp.header.stamp = clock_->now();
  auto seg = [&](const Pose & a, const Pose & b, double yaw) {
      const double len = dist(a, b);
      const int n = std::max(1, static_cast<int>(len / std::max(path_pose_interval_, 0.02)));
      for (int k = 0; k < n; ++k) {
        Pose p = a; const double t = static_cast<double>(k) / n;
        p.pose.position.x = a.pose.position.x + t * (b.pose.position.x - a.pose.position.x);
        p.pose.position.y = a.pose.position.y + t * (b.pose.position.y - a.pose.position.y);
        p.pose.orientation.z = std::sin(yaw / 2.0); p.pose.orientation.w = std::cos(yaw / 2.0);
        p.pose.orientation.x = p.pose.orientation.y = 0.0;
        p.header = lp.header;
        lp.poses.push_back(p);
      }
    };
  if (orth_phase_ == 1) {seg(robot, orth_p1_, orth_yaw_);}
  seg(orth_phase_ == 1 ? orth_p1_ : robot, orth_p2_, orth_yaw_);
  const double gyaw = std::atan2(goal.pose.position.y - orth_p2_.pose.position.y, goal.pose.position.x - orth_p2_.pose.position.x);
  seg(orth_p2_, goal, gyaw);
  Pose g = goal; g.header = lp.header; lp.poses.push_back(g);              // the real goal stays path.back()
  finalizeLocalPath(lp, false, false);
}

// translationFree: the robot keeps heading `yaw` and translates a -> b (side/front segments are driven holonomically
// with the yaw held). Footprint model: the real body orientation along the whole segment (a crab carries the 0.9 m
// length ACROSS the motion; a check with the footprint turned to the segment direction would see only 0.5 m).
bool MidproFsmController::translationFree(const Pose & a, const Pose & b, double yaw) const
{
  if (!useFootprint()) {return isLineCollisionFree(a, b);}
  return lateralFree(a, b, yaw, footprint_padding_);
}

// ======================================================================= goal staging (v0.73)
// rotSweepFree: turning in place from yaw0 to yaw1 (shortest way) keeps the footprint (+pad) free.
bool MidproFsmController::rotSweepFree(double x, double y, double yaw0, double yaw1, double pad) const
{
  const double d = angleDiff(yaw1, yaw0);
  const int n = std::max(1, static_cast<int>(std::ceil(std::abs(d) / (M_PI / 36.0))));   // 5 deg steps
  for (int k = 1; k <= n; ++k) {
    if (!footprintFree(x, y, yaw0 + d * k / n, pad)) {return false;}
  }
  return true;
}

// stageEvaluate: does the goal need a staging pose, and where (closest to the goal on the plan)?
bool MidproFsmController::stageEvaluate(const Pose & /*robot*/, const Pose & goal)
{
  const auto & cm = planCm();
  if (cm.size() < 4) {return false;}
  const double gyaw = yawOf(goal);
  const double gx = goal.pose.position.x, gy = goal.pose.position.y;
  // arrival heading: plan tangent over its last ~0.5 m
  size_t ia = cm.size() - 1; double acc = 0.0;
  while (ia > 0 && acc < 0.5) {acc += dist(cm[ia - 1], cm[ia]); --ia;}
  const double arrive = std::atan2(gy - cm[ia].pose.position.y, gx - cm[ia].pose.position.x);
  if (std::abs(angleDiff(gyaw, arrive)) < 0.26) {return false;}                 // arrives aligned (straight in)
  if (rotSweepFree(gx, gy, arrive, gyaw, footprint_padding_)) {return false;}  // turning at the goal fits
  Pose g_yaw = goal;
  // search back along the plan: the first pose (closest to the goal) where the turn fits and a
  // yaw-held slide reaches the goal
  acc = 0.0;
  for (size_t i = cm.size() - 1; i > 0; --i) {
    acc += dist(cm[i - 1], cm[i]);
    if (acc < stage_min_dist_) {continue;}
    if (acc > stage_max_dist_) {break;}
    const Pose & c = cm[i - 1];
    const double cx = c.pose.position.x, cyy = c.pose.position.y;
    const double here = std::atan2(cm[i].pose.position.y - cyy, cm[i].pose.position.x - cx);   // heading on arrival
    if (!footprintFree(cx, cyy, gyaw, footprint_padding_)) {continue;}
    if (!rotSweepFree(cx, cyy, here, gyaw, footprint_padding_)) {continue;}
    Pose s = c; s.pose.orientation.z = std::sin(gyaw / 2.0); s.pose.orientation.w = std::cos(gyaw / 2.0);
    s.pose.orientation.x = s.pose.orientation.y = 0.0;
    if (!translationFree(s, g_yaw, gyaw)) {continue;}
    stage_pose_ = s; stage_goal_yaw_ = gyaw; stage_phase_ = 1; stage_t_ = clock_->now(); stage_best_d_ = 1e9;
    char buf[200];
    std::snprintf(buf, sizeof(buf), "turning %.0f deg at the goal does not fit -> stage %.1f m before it, turn there, slide in",
      std::abs(angleDiff(gyaw, arrive)) * 180.0 / M_PI, acc);
    setPathState("stage_drive", buf);
    return true;
  }
  RCLCPP_WARN(logger_, "[stage] turning at the goal does not fit and no staging pose within %.1f m of the plan",
    stage_max_dist_);
  return false;
}

void MidproFsmController::stageTick(const Pose & robot, const Pose & goal)
{
  const auto now = clock_->now();
  if ((now - stage_t_).seconds() > 60.0) {stageEnd("timeout"); return;}
  nav_msgs::msg::Path lp;
  if (stage_phase_ == 1) {
    const double ds = dist(robot, stage_pose_);
    const bool passing = ds < 0.25 && ds > stage_best_d_ + 0.02;      // went by it (path not exactly through it)
    stage_best_d_ = std::min(stage_best_d_, ds);
    const bool arrived = (ds < 0.06 && meas_speed_ < 0.03) || passing || (ds < 0.25 && meas_speed_ < 0.02);
    Pose here_g = robot;
    here_g.pose.orientation.z = std::sin(stage_goal_yaw_ / 2.0); here_g.pose.orientation.w = std::cos(stage_goal_yaw_ / 2.0);
    here_g.pose.orientation.x = here_g.pose.orientation.y = 0.0;
    if (arrived && rotSweepFree(robot.pose.position.x, robot.pose.position.y, yawOf(robot), stage_goal_yaw_, footprint_padding_) &&
      translationFree(here_g, goal, stage_goal_yaw_))
    {
      stage_phase_ = 2; setPathState("stage_turn", "at the staging pose -> stop, turn to the goal yaw");
    } else if (arrived && ds < 0.06) {
      stageEnd("no room to turn at the staging pose"); return;
    } else {
      // follow the plan up to the staging pose (goal stays path.back() for the FSM)
      nav_msgs::msg::Path ep = makeGlobalPlanPath(robot);
      const auto & cm = planCm();
      double best = 1e18; size_t bi = ep.poses.size();
      for (size_t i = 0; i < ep.poses.size(); ++i) {
        const double d = dist(ep.poses[i], stage_pose_);
        if (d < best) {best = d; bi = i;}
      }
      (void)cm;
      if (bi < ep.poses.size()) {ep.poses.resize(bi + 1);}
      // the local path ENDS at the staging pose: the reference point never aims past it towards the goal
      // (that cut the corner before the stop), and the controller's goal braking stops the robot there
      ep.poses.push_back(stage_pose_);
      finalizeLocalPath(ep, true, true);
      return;
    }
  }
  if (stage_phase_ == 2) {
    if (std::abs(angleDiff(stage_goal_yaw_, yawOf(robot))) < 0.03) {        // ~1.7 deg; the slide keeps correcting
      stage_phase_ = 3; setPathState("stage_slide", "aligned -> slide into the goal with the yaw held");
    }
  }
  if (stage_phase_ == 3) {
    if (dist(robot, goal) < std::max(0.02, dist_tol_active_ * 0.8)) {stageEnd("in place"); return;}
    if (!translationFree(robot, goal, stage_goal_yaw_) &&
      !lateralFree(robot, goal, stage_goal_yaw_, 0.0)) {stageEnd("slide blocked"); return;}
  }
  // path shown in RViz: robot -> goal with the goal yaw (holonomic)
  lp.header.frame_id = costmap_ros_->getGlobalFrameID(); lp.header.stamp = now;
  const int n = std::max(2, static_cast<int>(dist(robot, goal) / std::max(path_pose_interval_, 0.02)));
  for (int k = 0; k <= n; ++k) {
    Pose p = robot; const double tt = static_cast<double>(k) / n;
    p.pose.position.x = robot.pose.position.x + tt * (goal.pose.position.x - robot.pose.position.x);
    p.pose.position.y = robot.pose.position.y + tt * (goal.pose.position.y - robot.pose.position.y);
    p.pose.orientation = goal.pose.orientation; p.header = lp.header;
    lp.poses.push_back(p);
  }
  finalizeLocalPath(lp, false, false);
}

void MidproFsmController::stageEnd(const std::string & why)
{
  stage_phase_ = 0;
  setPathState("straight", "goal staging: " + why);
}

// plannerTick: ex LocalPlanner::run() body (one iteration). Robot pose already in costmap frame.
void MidproFsmController::plannerTick(const Pose & robot_pose)
{
  if (!have_global_plan_) {return;}

  // refresh the current waypoint in the costmap frame (map->odom drift)
  if (waypoints_.empty()) {return;}
  Pose goal_cm;
  if (!transformToCostmapFrame(waypoints_[wp_index_], goal_cm)) {return;}
  local_goal_ = goal_cm;
  const Pose & local_goal = local_goal_;

  // Deviation guard: local detours (side/front) can walk away from the global plan. The BT only replans
  // when the plan becomes invalid, and a plan the robot has left is still "valid", so fail here and let
  // the BT recover/replan from the current position.
  const bool orth_recent = orth_phase_ != 0 || (orth_ended_once_ && (clock_->now() - orth_end_t_).seconds() < 6.0);
  if (max_path_deviation_ > 0.0 && !global_plan_.poses.empty() && !following_hybrid_ && !orth_recent) {
    const double dev = pathDeviation(robot_pose);
    if (dev > max_path_deviation_) {
      throw nav2_core::PlannerException(
              "MidproFsmController: " + std::to_string(dev) + " m off the global plan, replanning");
    }
  }

  // lookahead tracking: the local path is always the (obstacle-free, 1 Hz replanned) global plan.
  // straight/side/front/line are not used; obstacle handling = global replanning + speed governor.
  if (tracking_mode_ == "lookahead") {
    setPathState("detour", "lookahead tracking follows the global plan");
    nav_msgs::msg::Path ep = makeGlobalPlanPath(robot_pose);
    if (ep.poses.empty()) {return;}
    if (dist(ep.poses.back(), local_goal) > 0.3) {ep.poses.push_back(local_goal);}
    finalizeLocalPath(ep, true, true);
    return;
  }

  if (stage_phase_ != 0) {stageTick(robot_pose, local_goal); return;}
  if (goal_staging_ && useFootprint() && !stage_evaluated_ && isLastWaypoint() &&
    dist(robot_pose, local_goal) <= stage_eval_dist_)
  {
    stage_evaluated_ = true;
    if (stageEvaluate(robot_pose, local_goal)) {stageTick(robot_pose, local_goal); return;}
  }
  if (orth_phase_ != 0) {orthTick(robot_pose, local_goal); return;}
  if (orthTrigger(robot_pose, local_goal)) {orthTick(robot_pose, local_goal); return;}

  if (path_state_ == "straight") {
    int blk; double bx, by;
    const bool blocked = getBlockingCell(robot_pose, local_goal, true, blk, bx, by);
    if (!blocked) {
      nav_msgs::msg::Path lp;
      if (makeLocalPath(robot_pose, local_goal, lp)) {finalizeLocalPath(lp, true, false);}
    } else {
      was_blocked_ = true;
      int blk2; double bx2, by2;
      bool hard = getBlockingCell(robot_pose, local_goal, false, blk2, bx2, by2);
      if (hard && guardSaysForwardClear(robot_pose, local_goal, std::hypot(bx2 - robot_pose.pose.position.x, by2 - robot_pose.pose.position.y))) {
        hard = false;   // raster / padding artefact in a tight gap: the raw points say the way ahead is open
      }
      // The straight line is passable but not comfortable. Compare with the curve: if the global plan keeps a
      // comfortable clearance, follow it (do not hug an obstacle when a curve passes easily); if the curve is
      // tight as well we are in a narrow corridor -> pass straight (line state, midpro's original purpose).
      if (!hard && prefer_comfortable_curve_ && globalPlanComfortable(robot_pose)) {
        setPathState("detour", "straight too close, comfortable curve exists -> follow the global plan");
      } else if (!hard) {
        setPathState("line", "narrow: no comfortable curve either, pass straight");
      } else if (!transit_holonomic_ && !side_front_enabled_) {
        // side/front disabled: avoid along the global plan (curved, followed aligned)
        setPathState("detour", "hard obstacle -> follow the global plan (aligned avoidance)");
      } else if (std::hypot(bx2 - robot_pose.pose.position.x, by2 - robot_pose.pose.position.y) > side_trigger_dist_) {
        // midpro's side/front manoeuvres are for an obstacle right ahead / beside the robot. Its goals were
        // nearby waypoints, so "line blocked" meant exactly that; here the goal can be 20 m away and the line
        // may only graze a far wall corner -> that is the global plan's business, not a sidestep here.
        char buf[120];
        std::snprintf(buf, sizeof(buf), "line blocked %.1f m ahead (> %.1f m) -> follow the global plan",
          std::hypot(bx2 - robot_pose.pose.position.x, by2 - robot_pose.pose.position.y), side_trigger_dist_);
        setPathState("detour", buf);
      } else {
        Pose side_goal, front_goal;
        const bool found_side = findSideGoal(robot_pose, local_goal, side_goal);
        if (found_side && !try_side_) {
          setPathState("side_front", "side goal found");
        } else {
          const bool found_front = findFrontGoal(robot_pose, local_goal, front_goal);
          if (found_front && !try_side_) {
            setPathState("front_side", "front goal found");
          } else {
            setPathState("detour", "hard obstacle, no detour goal -> detour planner");
          }
        }
      }
    }
  } else if (path_state_ == "front_side") {
    int blk; double bx, by;
    const bool blocked = getBlockingCell(robot_pose, local_goal, true, blk, bx, by);
    if (!blocked) {
      was_blocked_ = false;
      setPathState("straight", "path clear, back to straight");
      try_side_ = true;
    } else {
      was_blocked_ = true;
      Pose front_goal;
      if (findFrontGoal(robot_pose, local_goal, front_goal)) {
        nav_msgs::msg::Path rsp;
        if (makeLocalPath(robot_pose, front_goal, rsp)) {
          rsp.poses.push_back(local_goal);
          finalizeLocalPath(rsp, false, false);
        }
      } else {
        setPathState("line", "front goal lost");
      }
    }
  } else if (path_state_ == "side_front") {
    int blk; double bx, by;
    const bool blocked = getBlockingCell(robot_pose, local_goal, true, blk, bx, by);
    if (!blocked) {
      was_blocked_ = false;
      setPathState("straight", "path clear, back to straight");
      try_side_ = true;
    } else {
      was_blocked_ = true;
      Pose side_goal;
      if (findSideGoal(robot_pose, local_goal, side_goal) && dist(robot_pose, side_goal) > side_search_step_) {
        nav_msgs::msg::Path rsp;
        if (makeLocalPath(robot_pose, side_goal, rsp)) {
          rsp.poses.push_back(local_goal);
          finalizeLocalPath(rsp, false, false);
        }
      } else {
        setPathState("line", "side goal lost");
      }
    }
  } else if (path_state_ == "line") {
    int blk; double bx, by;
    bool blocked = getBlockingCell(robot_pose, local_goal, false, blk, bx, by);
    if (blocked && guardSaysForwardClear(robot_pose, local_goal, std::hypot(bx - robot_pose.pose.position.x, by - robot_pose.pose.position.y))) {
      RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "[line] costmap says blocked %.2f m ahead but the guard sees %.2f m -> pass",
        std::hypot(bx - robot_pose.pose.position.x, by - robot_pose.pose.position.y), guard_fwd_dist_);
      blocked = false;
    }
    if (!blocked) {
      nav_msgs::msg::Path lp;
      if (makeLocalPath(robot_pose, local_goal, lp)) {finalizeLocalPath(lp, true, false);}
    } else {
      was_blocked_ = true;
      setPathState("detour", "hard obstacle on line path");
    }
  } else if (path_state_ == "detour") {
    int blk; double bx, by;
    const bool blocked = getBlockingCell(robot_pose, local_goal, true, blk, bx, by);   // comfortable, not merely passable
    // Leave detour as soon as the direct line to the goal is clear again (a dynamic obstacle moved away).
    // ROS1 also had a "within goal_orient_dist" rule, but ANDing the two kept the robot on the detour for
    // the whole run. A few consecutive clear ticks are required so a flickering cell does not toggle states.
    if (!blocked) {
      if (++detour_clear_count_ >= detour_clear_ticks_) {
        detour_clear_count_ = 0;
        detour_info_ = {0, 0, 0, true};
        detour_dir_.clear();
        start_is_escape_ = false; start_set_ = false;
        following_hybrid_ = false;
        setPathState("straight", "line clear again, back to straight");
        return;
      }
    } else {
      detour_clear_count_ = 0;
    }

    nav_msgs::msg::Path ep;
    // Hybrid-A* request on the local rolling window (fast); used as the primary detour source or as a
    // fallback when the global plan is not footprint-feasible (e.g. a corner too tight for the long body).
    auto request_hybrid = [&](nav_msgs::msg::Path & out) -> bool {
        geometry_msgs::msg::PoseStamped req;
        if (!pickDetourGoal(robot_pose, req)) {req = global_plan_.poses.back(); req.header = global_plan_.header;}
        req.header.stamp = clock_->now();
        detour_request_pub_->publish(req);
        out = makeDetourPath();
        if (out.poses.empty()) {
          const double waited = (clock_->now() - detour_enter_time_).seconds();
          if (waited > detour_timeout_) {
            throw nav2_core::PlannerException(
                    "MidproFsmController: no detour path for " + std::to_string(waited) + " s");
          }
          return false;   // keep the last local plan until the planner answers
        }
        return true;
      };
    if (detour_source_ == "planner") {
      if (!request_hybrid(ep)) {return;}
      following_hybrid_ = true;
    } else {
      following_hybrid_ = false;
      // default: follow the Nav2 global plan exactly the way the ROS1 controller followed the eband path
      ep = makeGlobalPlanPath(robot_pose);
      if (ep.poses.empty()) {return;}
      // Safety: check the next detour_check_dist metres of the plan. Circle model: centre cell > 95.
      // Footprint model: the footprint at the plan tangent collides (corner too tight / gap narrower than
      // the body) -> local Hybrid-A* if that server runs, otherwise fail so the BT replans.
      double acc = 0.0; bool infeasible = false; double at = 0.0;
      size_t blk_i = 0; double blk_yaw = 0.0;
      // only skip the first 0.5 m when the robot is already squeezed (escaping); otherwise check from the robot
      const bool start_free = !useFootprint() ||
        footprintFree(robot_pose.pose.position.x, robot_pose.pose.position.y, yawOf(robot_pose), footprint_padding_);
      // v0.55: look block_check_dist ahead (was detour_check_dist, 2 m): decide while there is room to go around
      const double look = std::max(detour_check_dist_, block_check_dist_);
      for (size_t i = 1; i + 1 < ep.poses.size() && acc < look; ++i) {
        acc += dist(ep.poses[i - 1], ep.poses[i]);
        if (!start_free && acc < 0.5) {continue;}
        const double tng = std::atan2(ep.poses[i + 1].pose.position.y - ep.poses[i].pose.position.y,
            ep.poses[i + 1].pose.position.x - ep.poses[i].pose.position.x);
        if (useFootprint()) {
          if (!footprintFree(ep.poses[i].pose.position.x, ep.poses[i].pose.position.y, tng, footprint_padding_)) {
            infeasible = true; at = acc; blk_i = i; blk_yaw = tng; break;
          }
        } else if (getCost(ep.poses[i]) > near_obstacle_value_) {
          infeasible = true; at = acc; blk_i = i; blk_yaw = tng; break;
        }
      }
      // classify: track the obstacle cell nearest to the robot inside the colliding footprint. A pallet keeps its
      // front face in place; a person crossing moves it sideways (the colliding PATH pose would not move).
      double ox = 0.0, oy = 0.0;
      const bool have_pt = infeasible && blockingObstaclePoint(ep.poses[blk_i], blk_yaw, robot_pose, ox, oy);
      if (infeasible) {
        if (!block_active_) {
          block_active_ = true; blocked_since_ = clock_->now(); block_moving_ = false;
          block_ref_x_ = ox; block_ref_y_ = oy;
          blockEvent("detect", at, 0.0);
        } else if (have_pt && std::hypot(ox - block_ref_x_, oy - block_ref_y_) > block_move_tol_) {
          if (!block_moving_) {
            RCLCPP_INFO(logger_, "[block] obstacle %.1f m ahead is moving -> yield", at);
            blockEvent("moving", at, blockAge());
          }
          block_moving_ = true; block_ref_x_ = ox; block_ref_y_ = oy;
        }
      } else {
        if (block_active_) {blockEvent("clear", 0.0, blockAge(), block_moving_ ? "moving" : "unclassified");}
        block_active_ = false; block_moving_ = false;
      }
      yield_cap_ = -1.0;
      const bool block_static = infeasible && !block_moving_ && blockAge() >= block_classify_time_;
      if (infeasible && has_pending_plan_ && (block_static || blockAge() >= block_persist_time_) &&
        (!have_pt || pendingAvoids(ox, oy)))
      {
        // static (pallet / new structure): go around now, from a distance. Moving and still there after
        // block_persist_time: go around as well.
        RCLCPP_INFO(logger_, "[block] %s obstacle %.1f m ahead -> alternative plan", block_static ? "static" : "persistent", at);
        blockEvent("switch", at, blockAge(), block_static ? "static" : "persistent");
        adoptPendingPlan();
        yield_cap_ = std::sqrt(2.0 * linear_dec_limit_ * std::max(0.0, at - yield_stop_margin_));
        return;
      }
      if (infeasible && blockAge() < block_persist_time_) {
        // Yield: slow down early and stop yield_stop_margin before the blockage (room to manoeuvre if it turns out
        // to be fixed). Moving obstacles usually clear before we get there.
        yield_cap_ = std::sqrt(2.0 * linear_dec_limit_ * std::max(0.0, at - yield_stop_margin_));
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "[yield] plan blocked %.1f m ahead for %.1f s (%s) -> slow/wait",
          at, blockAge(), block_moving_ ? "moving" : "classifying");
        infeasible = false;
      }
      if (infeasible) {
        const bool hybrid_up = hybrid_fallback_ && detour_request_pub_ && detour_request_pub_->get_subscription_count() > 0;
        if (hybrid_up) {
          RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "[detour] global plan not footprint-feasible %.1f m ahead -> local Hybrid-A*", at);
          if (!request_hybrid(ep)) {return;}
          following_hybrid_ = true;
        } else {
          blockEvent("fail", at, blockAge(), has_pending_plan_ ? "alternative_blocked" : "no_alternative");
          throw nav2_core::PlannerException(
                  "MidproFsmController: global plan blocked for the footprint " + std::to_string(at) + " m ahead");
        }
      }
    }
    // string pulling: replace the wiggles of a grid plan (centre line jumping as the aisle narrows) by the
    // longest straight stretch the body can drive with a small margin -> straight driving in corridors
    if (!following_hybrid_ && yield_cap_ < 0.0) {shortcutPath(ep);}
    if (dist(ep.poses.back(), local_goal) > 0.3) {
      ep.poses.push_back(local_goal);   // keep the real goal as path.back() for the FSM
    }
    finalizeLocalPath(ep, true, true);
  }
}

}  // namespace midpro_nav2_controller
