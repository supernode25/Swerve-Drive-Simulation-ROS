// midpro_control_loop.cpp  (part 3: FSM controller, ported from loas_controller.cpp controlLoop())
#include "midpro_nav2_controller/midpro_fsm_controller.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace midpro_nav2_controller
{

geometry_msgs::msg::TwistStamped MidproFsmController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lk(mutex_);
  meas_speed_ = std::hypot(velocity.linear.x, velocity.linear.y);
  geometry_msgs::msg::TwistStamped out;
  out.header.stamp = clock_->now();
  out.header.frame_id = costmap_ros_->getBaseFrameID();

  if (!have_global_plan_) {
    throw nav2_core::PlannerException("MidproFsmController: no plan set");
  }

  // robot pose in costmap frame
  Pose robot;
  if (!transformToCostmapFrame(pose, robot)) {
    throw nav2_core::PlannerException("MidproFsmController: failed to transform robot pose");
  }

  // ---- planner step (10 Hz in ROS1 -> every planner_divider-th tick here)
  if (tick_counter_++ % planner_divider_ == 0) {
    plannerTick(robot);
  }

  // ROS1 controller used path_[0] as the robot pose. The local plan starts at the robot pose
  // captured on the planner tick; refresh it with the current pose for the controller step.
  if (!path_.empty()) {
    path_[0] = robot;
  }

  out.twist = controlTick();
  return out;
}

geometry_msgs::msg::Twist MidproFsmController::controlTick()
{
  geometry_msgs::msg::Twist cmd;

  if (path_.size() < 2) {
    return cmd;   // zero twist, as ROS1 did when path_.size() < 2
  }

  const auto & robot_pose = path_[0].pose;
  const double x = robot_pose.position.x;
  const double y = robot_pose.position.y;
  const double yaw = yawOf(path_[0]);

  const auto & goal_pose = path_.back().pose;
  const double gx = goal_pose.position.x;
  const double gy = goal_pose.position.y;
  const double goal_yaw = yawOf(path_.back());
  const double dxg = gx - x, dyg = gy - y;
  const double dist_to_goal = std::hypot(dxg, dyg);
  const double yaw_diff_to_goal = angleDiff(goal_yaw, yaw);

  const auto & pre_goal_pose = path_[path_.size() - 2].pose;
  const double pgx = pre_goal_pose.position.x, pgy = pre_goal_pose.position.y;
  const double dist_to_pregoal = std::hypot(pgx - x, pgy - y);
  const double dist_pre_to_goal = std::hypot(gx - pgx, gy - pgy);

  // steering reference: ROS1 used path_[1] (5 cm ahead). With heading_lookahead > 0 the reference is the
  // path point heading_lookahead metres ahead, which averages out the point-to-point jitter of grid paths.
  size_t ref = 1;
  if (heading_lookahead_ > 0.0) {
    double acc = 0.0;
    for (size_t i = 1; i < path_.size(); ++i) {
      acc += dist(path_[i - 1], path_[i]);
      ref = i;
      if (acc >= heading_lookahead_) {break;}
    }
    if (ref >= path_.size() - 1 && path_.size() > 2) {ref = path_.size() - 2;}   // keep the goal pose out of the heading reference
    if (ref < 1) {ref = 1;}
  }
  // NEAR escape is latched only when a plan STARTS inside the NEAR band (exiting a bay without rotating).
  // A robot that drives INTO the NEAR band (a narrow aisle) is traversing, not escaping: it keeps steering
  // its body to the path tangent. The latch clears as soon as the robot is out of the NEAR band.
  const int cost_here = getCost(path_[0]);
  const bool inside_near = cost_here != -999 && cost_here >= near_obstacle_value_;
  if (!escape_latch_) {
    escape_active_ = no_rotate_in_near_ && inside_near;      // v0.28: escape whenever inside NEAR
  } else {
    if (fresh_plan_) {escape_active_ = no_rotate_in_near_ && inside_near;}
    if (!inside_near) {escape_active_ = false;}
  }
  fresh_plan_ = false;
  const bool in_near = escape_active_;
  if (in_near) {ref = 1;}
  // travel direction: a point a bit ahead (less noisy than 5 cm) even while escaping
  size_t dir_ref = ref;
  if (in_near) {
    double acc = 0.0;
    for (size_t i = 1; i < path_.size(); ++i) {acc += dist(path_[i - 1], path_[i]); dir_ref = i; if (acc >= 0.3) {break;}}
  } else if (is_oblique_ && oblique_lookahead_ > 0.0) {
    // holonomic paths (side/front, goal zone): aim at the next point like ROS1 did, so a two-segment path is
    // driven segment by segment (sideways, then straight) instead of cutting the corner diagonally
    double acc = 0.0;
    for (size_t i = 1; i < path_.size(); ++i) {acc += dist(path_[i - 1], path_[i]); dir_ref = i; if (acc >= oblique_lookahead_) {break;}}
  }
  const double dxn = path_[dir_ref].pose.position.x - x;
  const double dyn = path_[dir_ref].pose.position.y - y;
  // heading reference: holonomic paths hold their yaw point by point (ROS1: path[1]); taking it from the 0.4 m
  // reference made the two-segment side paths flip to the goal's yaw just before the corner (rotate <-> follow
  // every tick = "looking around"). Ackermann paths keep the lookahead tangent.
  const double next_yaw = yawOf(path_[(is_oblique_ && oblique_lookahead_ > 0.0 && !in_near) ? dir_ref : ref]);
  const double next_yaw_diff = angleDiff(next_yaw, yaw);
  const double yaw_error = angleDiff(std::atan2(dyn, dxn), yaw);
  // path curvature at the reference (for the optional feedforward term)
  double kappa_ref = 0.0;
  if (curvature_feedforward_ && ref + 1 < path_.size()) {
    size_t r2 = ref; double acc = 0.0;
    for (size_t i = ref + 1; i < path_.size(); ++i) {acc += dist(path_[i - 1], path_[i]); r2 = i; if (acc >= 0.3) {break;}}
    if (acc > 1e-3) {kappa_ref = angleDiff(yawOf(path_[r2]), next_yaw) / acc;}
  }

  // footprint model: rotating in place here must not sweep into anything
  // (holonomic) along the path until a spot where a rotation fits.
  // No room to rotate IN PLACE only forbids in-place rotation; steering while moving (ackermann) is still
  // allowed (it sweeps far less, and the trajectory filter checks it). Holding the yaw and sliding is used
  // only for the NEAR escape.
  const bool can_rotate = !useFootprint() || !rotation_check_ || rotationFree(path_[0]);
  // v0.41 holonomic policy: transit and obstacle avoidance drive ALIGNED (ackermann law on the path tangent).
  // The holonomic (oblique) law is used only for the NEAR escape at plan start, inside the goal zone (where the
  // path builder holds the goal yaw only next to obstacles / for side parking), or when transit_holonomic is on.
  const bool in_goal_zone = dist_to_goal <= holonomic_zone_;
  const bool side_state = (path_state_ == "side_front" || path_state_ == "front_side");   // v0.72: midpro side/front
  const bool oblique_allowed = transit_holonomic_ || in_goal_zone || side_state;
  const bool is_oblique = in_near || (is_oblique_ && oblique_allowed);
  const double max_lin = max_linear_vel_ * speed_scale_;
  const double min_lin = std::min(min_linear_vel_, max_lin);
  const auto now = clock_->now();

  // ---------------- high-level transitions (approach / follow)
  // goal staging: the staging pose is not the goal, so no approach / final states; rotate-in-place may still
  // be used by the law while driving to it
  if (stage_phase_ != 0 && ctrl_state_ != "IDLE" && ctrl_state_ != "FOLLOW_PATH" && ctrl_state_ != "ROTATE_IN_PLACE") {
    setCtrlState("FOLLOW_PATH", "goal staging");
  }
  if (ctrl_state_ != "IDLE" && ctrl_state_ != "ROTATE_IN_PLACE" && stage_phase_ == 0) {
    if (dist_to_goal <= approach_dist_) {
      if (ctrl_state_ != "FINAL_ROTATE") {
        if (dist_to_goal <= dist_tol_active_) {
          if (final_approach_counter_ >= final_approach_threshold_) {
            setCtrlState("FINAL_ROTATE", "FA counter >= threshold");
          } else {
            ++final_approach_counter_;
            has_init_cmd_time_ = false;
            setCtrlState("FINAL_APPROACH", "reached goal within tolerance");
          }
        } else {
          if (ctrl_state_ != "FINAL_APPROACH") {has_init_cmd_time_ = false;}
          setCtrlState("FINAL_APPROACH", "entered approach_dist");
        }
      }
    } else {
      final_approach_counter_ = 0;
      setCtrlState("FOLLOW_PATH", "goal distance recovered (> approach_dist)");
    }
  }

  // ---------------- FOLLOW_PATH (lookahead tracking)
  if (ctrl_state_ == "FOLLOW_PATH" && tracking_mode_ == "lookahead") {
    if (!has_init_cmd_time_) {init_cmd_time_ = now; has_init_cmd_time_ = true;}
    lookaheadTick(cmd, dist_to_goal);
  }

  // ---------------- goal staging (v0.73)
  if (stage_phase_ == 2 || stage_phase_ == 3) {
    if (stage_phase_ == 2 && meas_speed_ > 0.03) {             // brake to a full stop before turning
      gov_reason_ = "stage_stop";
    } else if (stage_phase_ == 2) {                            // turn to the goal yaw where the body fits
      const double e = angleDiff(stage_goal_yaw_, yaw);
      // proportional, no minimum-speed floor: a floor here produced a limit cycle around the tolerance
      cmd.angular.z = std::clamp(1.5 * e, -turn_angular_vel_, turn_angular_vel_);
      gov_reason_ = "stage_turn";
    } else {                                                   // slide in with the goal yaw held
      const double gdx = path_.back().pose.position.x - x, gdy = path_.back().pose.position.y - y;
      const double gd = std::hypot(gdx, gdy);
      const double cy = std::cos(yaw), sy = std::sin(yaw);
      const double v = std::min({orth_speed_, max_lin, std::sqrt(2.0 * linear_dec_limit_ * std::max(0.0, gd - 0.005))});
      if (gd > 1e-3) {cmd.linear.x = v * (cy * gdx + sy * gdy) / gd; cmd.linear.y = v * (-sy * gdx + cy * gdy) / gd;}
      cmd.angular.z = std::clamp(1.0 * angleDiff(stage_goal_yaw_, yaw), -0.15, 0.15);
      gov_reason_ = "stage_slide cap=" + std::to_string(v).substr(0, 4);
      if (useFootprint() && v > 0.0 && !motionFits(x, y, yaw, cmd.linear.x, cmd.linear.y, cmd.angular.z)) {
        cmd = geometry_msgs::msg::Twist(); gov_reason_ = "stage_blocked";
      }
    }
    if (speed_limit_pub_) {std_msgs::msg::String m; m.data = ctrl_state_ + " | " + gov_reason_; speed_limit_pub_->publish(m);}
    prev_linear_x_ = cmd.linear.x; prev_linear_y_ = cmd.linear.y;
    return cmd;
  }

  // ---------------- orthogonal manoeuvre (v0.68): heading held, straight to the corner, stop, straight ahead
  if (orth_phase_ != 0 && (ctrl_state_ == "FOLLOW_PATH" || ctrl_state_ == "ROTATE_IN_PLACE")) {
    if (ctrl_state_ != "FOLLOW_PATH") {setCtrlState("FOLLOW_PATH", "orthogonal manoeuvre");}
    const Pose & tgt = (orth_phase_ == 1) ? orth_p1_ : orth_p2_;
    const double tdx = tgt.pose.position.x - x, tdy = tgt.pose.position.y - y;
    const double td = std::hypot(tdx, tdy);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    const double bx = cy * tdx + sy * tdy, by = -sy * tdx + cy * tdy;          // target in the body frame
    double v = std::min({orth_speed_, max_lin, std::sqrt(2.0 * linear_dec_limit_ * std::max(0.0, td - 0.01))});
    // stop first, and pause at the corner, so the wheels can re-steer (crab <-> straight) before moving
    const bool settling = meas_speed_ > 0.05 && (now - orth_phase_t_).seconds() < 1.5 &&
      std::abs(std::atan2(by, bx)) > 0.5;
    if (settling || (now - orth_phase_t_).seconds() < 0.3) {v = 0.0;}
    if (td > 1e-3) {cmd.linear.x = v * bx / td; cmd.linear.y = v * by / td;}
    cmd.angular.z = std::clamp(1.0 * angleDiff(orth_yaw_, yaw), -0.15, 0.15);
    gov_reason_ = std::string(orth_phase_ == 1 ? "orth_side" : "orth_front") + " cap=" + std::to_string(v).substr(0, 4);
    if (useFootprint() && v > 0.0 && !motionFits(x, y, yaw, cmd.linear.x, cmd.linear.y, cmd.angular.z)) {
      cmd = geometry_msgs::msg::Twist(); gov_reason_ = "orth_blocked";
    }
    if (speed_limit_pub_) {std_msgs::msg::String m; m.data = ctrl_state_ + " | " + gov_reason_; speed_limit_pub_->publish(m);}
    prev_linear_x_ = cmd.linear.x; prev_linear_y_ = cmd.linear.y;
    return cmd;
  }

  // ---------------- FOLLOW_PATH (ROS1 point-to-point law)
  if (ctrl_state_ == "FOLLOW_PATH" && tracking_mode_ != "lookahead") {
    if (!has_init_cmd_time_) {init_cmd_time_ = now; has_init_cmd_time_ = true;}
    double linear_speed;
    const double detect_side_dist = 0.1;
    if (dist_pre_to_goal > detect_side_dist) {
      linear_speed = std::max(min_lin, std::min(max_lin, dist_to_pregoal));
    } else {
      const double acc_dur = (now - init_cmd_time_).seconds();
      linear_speed = std::max(min_lin, std::min(max_lin, acceleration_limit_ * acc_dur));
    }
    // situational cap: obstacles ahead, curvature, stopping distance to the waypoint
    linear_speed = std::max(min_lin, speedGovernor(linear_speed, dist_to_goal, next_yaw_diff));
    // planned speed profile (curvature / proximity / goal braking computed once per plan)
    if (speed_profile_enabled_) {
      const double pv = profileSpeedAt(path_[0]);
      if (pv < linear_speed) {
        linear_speed = std::max(min_lin, pv);
        gov_reason_ = "profile cap=" + std::to_string(pv).substr(0, 4);
      }
    }
    // goal staging, phase 1: brake to a stop at the staging pose
    if (stage_phase_ == 1) {
      const double ds = dist(path_[0], stage_pose_);
      const double vs = std::sqrt(2.0 * linear_dec_limit_ * std::max(0.0, ds - 0.01));
      if (vs < linear_speed) {linear_speed = vs < 0.02 ? 0.0 : vs; gov_reason_ = "stage_stop cap=" + std::to_string(linear_speed).substr(0, 4);}
    }
    // yield: waiting for a (probably moving) blockage on the plan to clear -> may stop completely
    if (yield_cap_ >= 0.0 && yield_cap_ < linear_speed) {
      linear_speed = yield_cap_ < 0.03 ? 0.0 : yield_cap_;
      gov_reason_ = "yield cap=" + std::to_string(linear_speed).substr(0, 4);
    }

    // Alignment gate: a tight stretch (footprint clearance < 2 x padding) within align_gate_dist must be
    // entered with the body aligned to it; entering at an angle is what wedges a long body in a gap.
    // v0.33: the gate compares the body with the path tangent AT THE ROBOT (tracking error), not with the
    // tangent at the tight point: on a curve those always differ, and following the path aligns the body
    // by the time it gets there. It intervenes only when the robot is off its path heading.
    double trk_lat = 0.0, trk_yaw = 0.0;
    const bool tracking = trackingWell(x, y, yaw, trk_lat, trk_yaw);
    if (useFootprint() && align_gate_enabled_ && !in_near && ctrl_state_ == "FOLLOW_PATH" && !tracking && std::abs(trk_yaw) > align_tolerance_) {
      double acc = 0.0;
      for (size_t i = 1; i < path_.size() && acc < align_gate_dist_; ++i) {
        acc += dist(path_[i - 1], path_[i]);
        if (!footprintFree(path_[i].pose.position.x, path_[i].pose.position.y, yawOf(path_[i]), tight_clearance_)) {
          const double dyaw = trk_yaw;
          if (std::abs(dyaw) > align_tolerance_) {
            if (can_rotate) {
              align_target_yaw_ = yaw + trk_yaw; align_active_ = true;
              setCtrlState("ROTATE_IN_PLACE", "align before tight passage");
            } else {
              linear_speed = std::min(linear_speed, align_move_speed_);   // let the steering catch up before the gap
              gov_reason_ = "align_before_gap cap=" + std::to_string(align_move_speed_).substr(0, 4);
            }
          }
          break;
        }
      }
    }
    if (ctrl_state_ != "FOLLOW_PATH") {
      // handled by ROTATE_IN_PLACE below
    } else if (is_oblique) {
      if (!in_near && can_rotate && std::abs(next_yaw_diff) > yaw_tolerance_ * 6.0 && path_.size() > 3) {
        setCtrlState("ROTATE_IN_PLACE", "omni heading error > yaw_tolerance x6");
      } else {
        const double v_esc = in_near ? std::min(linear_speed, escape_speed_) : linear_speed;
        cmd.linear.x = v_esc * std::cos(yaw_error);
        cmd.linear.y = v_esc * std::sin(yaw_error);
        if (cmd.linear.x < 0.0 && !in_near) {
          cmd.linear.x = 0.1 * std::cos(yaw_error);   // [ROS1] crawl when the target is behind
          cmd.linear.y = 0.1 * std::sin(yaw_error);
        }
        if (in_near) {gov_reason_ = "escape cap=" + std::to_string(v_esc).substr(0, 4);}
      }
    } else {
      // v0.80: a path that leaves sideways (45-135 deg) is taken by a sideways shift with the heading held when
      // that shift fits, even if there is room to rotate: for a swerve body a parallel shift is shorter and
      // safer than a 120 deg turn in place followed by an arc back.
      const bool leaves_sideways = std::abs(yaw_error) > M_PI / 4.0 && std::abs(yaw_error) < 3.0 * M_PI / 4.0;
      const double v_shift = std::min(linear_speed, escape_speed_);
      const bool shift_fits = leaves_sideways && useFootprint() &&
        motionFits(x, y, yaw, v_shift * std::cos(yaw_error), v_shift * std::sin(yaw_error), 0.0);
      if (std::abs(next_yaw_diff) > M_PI / 3.0 && can_rotate && !shift_fits) {
        setCtrlState("ROTATE_IN_PLACE", "ackermann steering > 60 deg");
      } else if (std::abs(next_yaw_diff) > M_PI / 3.0) {
        // Large heading error and no room to rotate in place. Pick the motion that actually fits (1 s footprint
        // rollout): when the path leaves sideways/backwards, slide out holding the yaw (crab); otherwise align on
        // the move (forward or reverse arc); if nothing fits, stop and let the BT recover.
        const double v_slow = std::min(linear_speed, align_move_speed_);
        const double wz_al = std::clamp(k_prop_ * next_yaw_diff, -0.5 * max_angular_vel_, 0.5 * max_angular_vel_);
        const double crab_vx = std::min(linear_speed, escape_speed_) * std::cos(yaw_error);
        const double crab_vy = std::min(linear_speed, escape_speed_) * std::sin(yaw_error);
        const bool sideways = std::abs(yaw_error) > M_PI / 4.0;
        const double fwd = (std::abs(yaw_error) > M_PI / 2.0) ? -1.0 : 1.0;
        // v0.80: when the path leaves sideways, a swerve body should shift sideways with the heading held FIRST
        // (then the path comes into the front and is followed aligned). Trying the forward / reverse alignment
        // arcs first swung the body into the rack beside it. (The old diagonal-crab wedging came from long
        // side paths at speed, not from this slow, footprint-checked shift.)
        const bool crab_first = sideways;
        // back off: slide straight away from the nearest obstacle (heading held) to make room to turn
        double nox = 0.0, noy = 0.0, bo_vx = 0.0, bo_vy = 0.0;
        bool back_off_ok = false;
        if (useFootprint() && nearestObstacleAround(path_[0], 0.35, nox, noy)) {
          const double away = std::atan2(y - noy, x - nox);          // world direction away from the obstacle
          const double rel = angleDiff(away, yaw);                   // in the body frame
          bo_vx = back_off_speed_ * std::cos(rel); bo_vy = back_off_speed_ * std::sin(rel);
          back_off_ok = motionFits(x, y, yaw, bo_vx, bo_vy, 0.0);
        }
        if (crab_first && motionFits(x, y, yaw, crab_vx, crab_vy, 0.0)) {
          cmd.linear.x = crab_vx; cmd.linear.y = crab_vy; cmd.angular.z = 0.0;
          gov_reason_ = "crab_no_room cap=" + std::to_string(escape_speed_).substr(0, 4);
        } else if (back_off_ok) {
          cmd.linear.x = bo_vx; cmd.linear.y = bo_vy; cmd.angular.z = 0.0;
          gov_reason_ = "back_off cap=" + std::to_string(back_off_speed_).substr(0, 4);
        } else if (motionFits(x, y, yaw, fwd * v_slow, 0.0, wz_al)) {
          cmd.linear.x = fwd * v_slow; cmd.angular.z = wz_al;
          gov_reason_ = "align_on_move cap=" + std::to_string(align_move_speed_).substr(0, 4);
        } else if (motionFits(x, y, yaw, -fwd * v_slow, 0.0, wz_al)) {
          cmd.linear.x = -fwd * v_slow; cmd.angular.z = wz_al;
          gov_reason_ = "align_on_move_rev cap=" + std::to_string(align_move_speed_).substr(0, 4);
        } else if (!crab_first && motionFits(x, y, yaw, crab_vx, crab_vy, 0.0)) {
          cmd.linear.x = crab_vx; cmd.linear.y = crab_vy; cmd.angular.z = 0.0;
          gov_reason_ = "crab_no_room cap=" + std::to_string(escape_speed_).substr(0, 4);
        } else {
          gov_reason_ = "boxed_in cap=0";
          RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "[motion] no rotation room and no fitting move -> stop (BT recovery)");
        }
      } else {
        double e_lat = 0.0, e_yaw = 0.0;
        const bool tight_track = tight_track_enabled_ && useFootprint() && tight_now_ &&
          signedTrackError(x, y, yaw, e_lat, e_yaw) && std::abs(e_yaw) < 0.35 && std::abs(yaw_error) < M_PI / 2.0;
        if (tight_track) {
          // tight passage: steering would swing the 0.9 m body into the walls. Keep the heading along the
          // corridor and take the lateral error out by crabbing (ROS1 straight/line segments did the same).
          cmd.linear.x = linear_speed;
          cmd.linear.y = std::clamp(tight_track_lat_gain_ * e_lat, -tight_track_lat_max_, tight_track_lat_max_);
          cmd.angular.z = std::clamp(tight_track_yaw_gain_ * e_yaw, -tight_track_yaw_max_, tight_track_yaw_max_);
          if (gov_reason_.rfind("tight_passage", 0) == 0) {gov_reason_ = "tight_track " + gov_reason_;}
        } else {
          cmd.angular.z = std::clamp(k_prop_ * next_yaw_diff + (curvature_feedforward_ ? linear_speed * kappa_ref : 0.0),
              -max_angular_vel_, max_angular_vel_);
          const double ratio = cmd.angular.z / max_angular_vel_ * M_PI / 2.3;
          if (std::abs(yaw_error) > M_PI / 2.0) {
            cmd.linear.x = -0.1 * std::cos(ratio);
          } else {
            cmd.linear.x = linear_speed * std::max(std::cos(ratio), min_turn_speed_ratio_);   // [ROS1 had no floor]
          }
        }
      }
    }
  }

  // ---------------- ROTATE_IN_PLACE
  if (ctrl_state_ == "ROTATE_IN_PLACE") {
    if (!has_rotate_start_time_) {rotate_start_time_ = now; has_rotate_start_time_ = true;}
    const double ang_dur = (now - rotate_start_time_).seconds();
    const double max_ang_this_tick = std::min(turn_angular_vel_, angular_acc_limit_ * ang_dur);
    double rot_err = next_yaw_diff;
    double exit_tol = yaw_tolerance_;
    if (align_active_) {rot_err = angleDiff(align_target_yaw_, yaw); exit_tol = 0.5 * align_tolerance_;}
    if (tracking_mode_ == "lookahead") {
      double dir, tangent;
      if (lookaheadPoint(dir, tangent)) {rot_err = angleDiff(dir, yaw);}
      exit_tol = align_angle_;
    }
    const bool sweep_blocked = useFootprint() && rotation_check_ &&
      !footprintFree(x, y, yaw + std::copysign(0.2, rot_err), footprint_padding_);
    if (std::abs(rot_err) < exit_tol) {
      has_rotate_start_time_ = false;
      has_init_cmd_time_ = false;
      align_active_ = false;
      setCtrlState("FOLLOW_PATH", "yaw aligned");
    } else if (sweep_blocked) {
      has_rotate_start_time_ = false;
      has_init_cmd_time_ = false;
      align_active_ = false;
      setCtrlState("FOLLOW_PATH", "rotation sweep blocked by the footprint -> align on the move");
    } else {
      const double raw_z = std::clamp(rot_err, -turn_angular_vel_, turn_angular_vel_);
      cmd.angular.z = std::copysign(std::min(std::abs(raw_z), max_ang_this_tick), raw_z);
    }
  }

  // ---------------- FINAL_APPROACH (lookahead tracking: same law, governor brakes to the goal)
  if (ctrl_state_ == "FINAL_APPROACH" && tracking_mode_ == "lookahead") {
    if (!has_init_cmd_time_) {init_cmd_time_ = now; has_init_cmd_time_ = true;}
    lookaheadTick(cmd, dist_to_goal);
  }

  // ---------------- FINAL_APPROACH (ROS1 law)
  if (ctrl_state_ == "FINAL_APPROACH" && tracking_mode_ != "lookahead") {
    if (!has_init_cmd_time_) {init_cmd_time_ = now; has_init_cmd_time_ = true;}
    const double acc_dur = (now - init_cmd_time_).seconds();
    const double acc_speed = acceleration_limit_ * acc_dur;
    const double dec_speed = dist_to_goal;
    const double linear_speed = std::max(min_lin, std::min({max_lin, acc_speed, dec_speed}));

    if (is_oblique) {
      if (!in_near && can_rotate && std::abs(next_yaw_diff) > yaw_tolerance_ * 4.0 && path_.size() > 3) {
        setCtrlState("ROTATE_IN_PLACE", "omni heading drift in final approach > yaw_tolerance x4");
      } else {
        cmd.linear.x = linear_speed * std::cos(yaw_error);
        cmd.linear.y = linear_speed * std::sin(yaw_error);
        if (cmd.linear.x < 0.0) {
          cmd.linear.x = 0.1 * std::cos(yaw_error);
          cmd.linear.y = 0.1 * std::sin(yaw_error);
        }
      }
    } else {
      if (std::abs(next_yaw_diff) > M_PI / 3.0) {
        setCtrlState("ROTATE_IN_PLACE", "ackermann steering > 60 deg in final approach");
      } else {
        double e_lat = 0.0, e_yaw = 0.0;
        const bool tight_track = tight_track_enabled_ && useFootprint() && tight_now_ &&
          signedTrackError(x, y, yaw, e_lat, e_yaw) && std::abs(e_yaw) < 0.35 && std::abs(yaw_error) < M_PI / 2.0;
        if (tight_track) {
          // tight passage: steering would swing the 0.9 m body into the walls. Keep the heading along the
          // corridor and take the lateral error out by crabbing (ROS1 straight/line segments did the same).
          cmd.linear.x = linear_speed;
          cmd.linear.y = std::clamp(tight_track_lat_gain_ * e_lat, -tight_track_lat_max_, tight_track_lat_max_);
          cmd.angular.z = std::clamp(tight_track_yaw_gain_ * e_yaw, -tight_track_yaw_max_, tight_track_yaw_max_);
          if (gov_reason_.rfind("tight_passage", 0) == 0) {gov_reason_ = "tight_track " + gov_reason_;}
        } else {
          cmd.angular.z = std::clamp(k_prop_ * next_yaw_diff + (curvature_feedforward_ ? linear_speed * kappa_ref : 0.0),
              -max_angular_vel_, max_angular_vel_);
          const double ratio = cmd.angular.z / max_angular_vel_ * M_PI / 2.3;
          if (std::abs(yaw_error) > M_PI / 2.0) {
            cmd.linear.x = -0.1 * std::cos(ratio);
          } else {
            cmd.linear.x = linear_speed * std::max(std::cos(ratio), min_turn_speed_ratio_);   // [ROS1 had no floor]
          }
        }
      }
    }
  }

  // ---------------- FINAL_ROTATE
  if (ctrl_state_ == "FINAL_ROTATE") {
    cmd.angular.z = std::clamp(yaw_diff_to_goal, -turn_angular_vel_, turn_angular_vel_);
    if (useFootprint() && std::abs(yaw_diff_to_goal) > yaw_tol_active_ &&
      !rotSweepFree(x, y, yaw, goal_yaw, 0.0))
    {
      // turning here would hit something: do not wedge the body (goal staging should have aligned it earlier)
      cmd.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "[FINAL_ROTATE] no room to turn %.0f deg at the goal",
        yaw_diff_to_goal * 180.0 / M_PI);
    }
    // (ROS1 goal_info string cross-check dropped: in Nav2 the goal IS the last plan pose)
    if (std::abs(yaw_diff_to_goal) <= yaw_tol_active_) {
      RCLCPP_INFO(logger_, "[FINAL_ROTATE] goal reached: dist=%.3f m, dyaw=%.2f deg, goal yaw %.1f deg, robot yaw %.1f deg",
        dist_to_goal, yaw_diff_to_goal * 180.0 / M_PI, goal_yaw * 180.0 / M_PI, yaw * 180.0 / M_PI);
      cmd = geometry_msgs::msg::Twist();
      if (advanceWaypoint()) {
        // intermediate corner reached and aligned with the next segment: keep going
        prev_linear_x_ = 0.0; prev_linear_y_ = 0.0;
        return cmd;
      }
      setCtrlState("IDLE", "goal reached");
      goal_reached_ = true;
      final_approach_counter_ = 0;
      has_rotate_start_time_ = false;
      has_init_cmd_time_ = false;
    }
  }

  // ---------------- IDLE
  if (ctrl_state_ == "IDLE") {
    cmd = geometry_msgs::msg::Twist();
    prev_linear_x_ = 0.0;
    prev_linear_y_ = 0.0;
    // Nav2's goal checker (xy 0.02 / yaw 0.025, see params) will end the FollowPath action.
    return cmd;
  }

  if (speed_limit_pub_ && !gov_reason_.empty()) {
    std_msgs::msg::String m; m.data = ctrl_state_ + " | " + gov_reason_;
    speed_limit_pub_->publish(m);
  }

  // ---------------- output clamps and filters (unchanged from ROS1)
  cmd.linear.x = std::clamp(cmd.linear.x, -max_lin, max_lin);
  cmd.angular.z = std::clamp(cmd.angular.z, -max_angular_vel_, max_angular_vel_);
  if (std::abs(cmd.linear.y) > max_linear_y_) {
    const double r = max_linear_y_ / std::abs(cmd.linear.y);
    cmd.linear.y *= r;
    cmd.linear.x *= r;
  }
  // Minimum angular speed exists to beat the motor deadband when turning IN PLACE. Applied to every state
  // (ROS1), it inflated tiny driving corrections (0.01 -> 0.05 rad/s), overshot, flipped sign and produced a
  // +-min_angular_vel square-wave steering limit cycle (~1 s period) while driving straight. v0.46: in place only;
  // while driving, corrections below steer_deadband are zeroed and the rest pass through unchanged.
  const bool turning_in_place = (ctrl_state_ == "ROTATE_IN_PLACE" || ctrl_state_ == "FINAL_ROTATE");
  if (!min_angular_in_place_only_ || turning_in_place) {
    if (cmd.angular.z != 0.0 && std::abs(cmd.angular.z) < min_angular_vel_) {
      cmd.angular.z = std::copysign(min_angular_vel_, cmd.angular.z);
    }
  } else if (std::abs(cmd.angular.z) < steer_deadband_) {
    cmd.angular.z = 0.0;
  }
  if (cmd.linear.y != 0.0) {
    cmd.linear.y = linear_y_lpf_alpha_ * cmd.linear.y + (1.0 - linear_y_lpf_alpha_) * prev_linear_y_;
  } else {
    cmd.linear.y = 0.0;
  }
  prev_linear_y_ = cmd.linear.y;
  {
    const double step = (linear_acc_limit_ > 0.0) ? linear_acc_limit_ / controller_frequency_ : linear_x_rate_limit_;
    const double dx = cmd.linear.x - prev_linear_x_;
    if (std::abs(dx) > step) {
      cmd.linear.x = prev_linear_x_ + std::copysign(step, dx);
    }
    prev_linear_x_ = cmd.linear.x;
  }
  // ---------------- trajectory safety filter: simulate the command forward with the real footprint
  cmd = safetyFilter(cmd, x, y, yaw, yaw + yaw_error);
  prev_linear_x_ = cmd.linear.x;
  prev_linear_y_ = cmd.linear.y;
  return cmd;
}

// trackingWell: the robot sits on its local path (lateral and heading error to the nearest pose ahead are small).
// signedTrackError: lateral offset of the robot from the local path (positive = path is to the robot's left,
// i.e. the robot must move left) and heading error to the path tangent, in the body frame.
bool MidproFsmController::signedTrackError(double x, double y, double yaw, double & e_lat, double & e_yaw) const
{
  if (path_.size() < 3) {return false;}
  size_t bi = 1; double best = 1e18; double acc = 0.0;
  for (size_t i = 1; i < path_.size() && acc < 1.0; ++i) {
    acc += dist(path_[i - 1], path_[i]);
    const double d = std::hypot(path_[i].pose.position.x - x, path_[i].pose.position.y - y);
    if (d < best) {best = d; bi = i;}
  }
  const size_t j = std::min(bi + 1, path_.size() - 1);
  const size_t k = (j == bi) ? bi - 1 : bi;
  const double tng = std::atan2(path_[j].pose.position.y - path_[k].pose.position.y, path_[j].pose.position.x - path_[k].pose.position.x);
  // offset of the nearest path point from the robot, expressed in the body frame: +y = left
  const double dx = path_[bi].pose.position.x - x, dy = path_[bi].pose.position.y - y;
  e_lat = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  e_yaw = angleDiff(tng, yaw);
  return true;
}

bool MidproFsmController::trackingWell(double x, double y, double yaw, double & lat, double & dyaw) const
{
  lat = 1e9; dyaw = 0.0;
  if (path_.size() < 3) {return false;}
  size_t best_i = 1; double best = 1e18;
  double acc = 0.0;
  for (size_t i = 1; i < path_.size() && acc < 1.0; ++i) {
    acc += dist(path_[i - 1], path_[i]);
    const double d = std::hypot(path_[i].pose.position.x - x, path_[i].pose.position.y - y);
    if (d < best) {best = d; best_i = i;}
  }
  lat = best;
  // tangent of the local path at that pose (path pose yaw may be held for holonomic segments; use geometry)
  const size_t j = std::min(best_i + 1, path_.size() - 1);
  const size_t k = (j == best_i) ? best_i - 1 : best_i;
  const double tng = std::atan2(path_[j].pose.position.y - path_[k].pose.position.y, path_[j].pose.position.x - path_[k].pose.position.x);
  const double pose_yaw = yawOf(path_[best_i]);
  // holonomic segments hold a fixed yaw: track that; ackermann segments: track the tangent
  const bool held = std::abs(angleDiff(pose_yaw, tng)) > 0.3;
  dyaw = angleDiff(held ? pose_yaw : tng, yaw);
  return lat < track_lat_tol_ && std::abs(dyaw) < track_yaw_tol_;
}

// motionFits: would (vx, vy, wz) held for traj_check_time_ keep the footprint clear? Starting in contact,
// only where the robot ends up counts (escape). Used to choose the motion primitive when rotation does not fit,
// independent of the trajectory safety filter switch.
bool MidproFsmController::motionFits(double x, double y, double yaw, double vx, double vy, double wz) const
{
  if (!useFootprint()) {return true;}
  double pad = footprint_padding_;
  int skip = 0;
  if (!footprintFree(x, y, yaw, pad)) {
    pad = 0.0;
    if (!footprintFree(x, y, yaw, 0.0)) {skip = 3;}
  }
  return trajectoryFree(x, y, yaw, vx, vy, wz, pad, skip);
}

// trajectoryFree: integrate (vx, vy, wz) in the body frame for traj_check_time_ (0.1 s steps) and check the
// footprint (+pad) at every step. The first `skip` steps are not checked (escaping from contact).
bool MidproFsmController::trajectoryFree(
  double x, double y, double th, double vx, double vy, double wz, double pad, int skip) const
{
  const double dt = 0.1;
  const int n = std::max(1, static_cast<int>(std::round(traj_check_time_ / dt)));
  for (int k = 1; k <= n; ++k) {
    th += wz * dt;
    x += (vx * std::cos(th) - vy * std::sin(th)) * dt;
    y += (vx * std::sin(th) + vy * std::cos(th)) * dt;
    if (k <= skip) {continue;}
    if (!footprintFree(x, y, th, pad)) {return false;}
  }
  return true;
}

// safetyFilter: the FSM law decides WHERE to go; this checks that the command it produced does not sweep the
// body into anything within traj_check_time_ (a 0.9 m body turning while moving swings its rear out, and
// the path poses alone do not show that). On a predicted collision it tries, in order: the same command
// scaled down, the same translation without rotation, a heading-held slide toward the path, rotation only
// (if it fits), and finally a stop. Starting in contact, moves that do not dig deeper are allowed (escape).
geometry_msgs::msg::Twist MidproFsmController::safetyFilter(
  const geometry_msgs::msg::Twist & in, double x, double y, double yaw, double ref_dir)
{
  if (!useFootprint() || !traj_check_enabled_) {return in;}
  const bool moving = std::abs(in.linear.x) + std::abs(in.linear.y) > 1e-3 || std::abs(in.angular.z) > 1e-3;
  if (!moving) {return in;}
  double pad = footprint_padding_;
  int skip = 0;
  if (!footprintFree(x, y, yaw, pad)) {
    pad = 0.0;
    if (!footprintFree(x, y, yaw, 0.0)) {skip = 3;}   // already touching in the costmap: judge where it ends up
  }
  auto ok = [&](double vx, double vy, double wz) {return trajectoryFree(x, y, yaw, vx, vy, wz, pad, skip);};
  // Trust the plan: while the robot tracks its local path and the path poses it will reach within the horizon
  // are footprint-free at their own yaw, the constant-command rollout is too pessimistic on curves (the next
  // ticks change the turn rate). Roll out only when the robot is off its path.
  {
    double lat = 0.0, dyw = 0.0;
    if (pad > 0.0 && trackingWell(x, y, yaw, lat, dyw)) {
      const double reach = std::hypot(in.linear.x, in.linear.y) * traj_check_time_ + 0.3;
      double acc = 0.0; bool free_ahead = true;
      for (size_t i = 1; i < path_.size() && acc < reach; ++i) {
        acc += dist(path_[i - 1], path_[i]);
        if (!footprintFree(path_[i].pose.position.x, path_[i].pose.position.y, yawOf(path_[i]), pad)) {free_ahead = false; break;}
      }
      if (free_ahead) {return in;}
    }
  }
  if (ok(in.linear.x, in.linear.y, in.angular.z)) {return in;}

  struct Cand {double vx, vy, wz; const char * why;};
  std::vector<Cand> cands = {
    {0.6 * in.linear.x, 0.6 * in.linear.y, 0.6 * in.angular.z, "scaled 60%"},
    {0.3 * in.linear.x, 0.3 * in.linear.y, 0.3 * in.angular.z, "scaled 30%"},
    {in.linear.x, in.linear.y, 0.0, "no rotation"},
    {0.5 * in.linear.x, 0.5 * in.linear.y, 0.0, "no rotation 50%"},
  };
  const double v = std::hypot(in.linear.x, in.linear.y);
  const double e = angleDiff(ref_dir, yaw);
  const double vs = std::max(std::min(std::max(v, 0.1), escape_speed_), 0.1);
  cands.push_back({vs * std::cos(e), vs * std::sin(e), 0.0, "slide to path"});
  if (std::abs(in.angular.z) > 1e-3 && !path_.empty() && rotationFree(path_[0])) {
    cands.push_back({0.0, 0.0, in.angular.z, "rotate only"});
  }
  // wedged in a tight gap (a few cm each side, body slightly skewed): tiny moves the big candidates miss
  {
    double lat = 0.0, dyw = 0.0;
    trackingWell(x, y, yaw, lat, dyw);                       // dyw: heading error to the local path (gap axis)
    if (std::abs(dyw) > 0.01) {
      cands.push_back({0.0, 0.0, std::clamp(1.0 * dyw, -0.12, 0.12), "micro align"});
    }
    cands.push_back({-0.08, 0.0, 0.0, "reverse straight"});
    cands.push_back({0.08, 0.0, 0.0, "forward straight"});
  }
  geometry_msgs::msg::Twist out;
  std::string why = "stop";
  for (const auto & cnd : cands) {
    if (ok(cnd.vx, cnd.vy, cnd.wz)) {
      out.linear.x = cnd.vx; out.linear.y = cnd.vy; out.angular.z = cnd.wz; why = cnd.why;
      break;
    }
  }
  if (why == std::string("stop")) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "[safety] every candidate collides within %.1f s -> stop", traj_check_time_);
  } else {
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "[safety] predicted footprint collision -> %s", why.c_str());
  }
  if (speed_limit_pub_) {
    std_msgs::msg::String m; m.data = ctrl_state_ + " | traj_safety " + why;
    speed_limit_pub_->publish(m);
  }
  return out;
}

// speedGovernor: situational upper bound on the requested speed (idea borrowed from neo_local_planner2)
//   - stopping-distance law toward the current waypoint:  v <= sqrt(2 * dec * d)
//   - obstacle cost along the path ahead: NEAR(95) -> near_obstacle_speed, inflated -> inflation_speed
//   - heading change ahead larger than curve_angle -> curve_speed
double MidproFsmController::speedGovernor(double v, double dist_to_goal, double /*next_yaw_diff*/) const
{
  const double vmax = max_linear_vel_ * speed_scale_;
  double cap = std::min(v, vmax);
  std::string reason = "none";
  double situ = vmax;

  // curvature: direction change of the path over curve_lookahead metres (grid paths jitter point to point)
  if (!speed_profile_enabled_ && path_.size() >= 3) {
    double acc = 0.0; size_t li = path_.size() - 1;
    for (size_t i = 1; i < path_.size(); ++i) {
      acc += dist(path_[i - 1], path_[i]);
      if (acc >= curve_lookahead_) {li = i; break;}
    }
    const double d0 = std::atan2(path_[1].pose.position.y - path_[0].pose.position.y,
        path_[1].pose.position.x - path_[0].pose.position.x);
    const double d1 = std::atan2(path_[li].pose.position.y - path_[li > 0 ? li - 1 : 0].pose.position.y,
        path_[li].pose.position.x - path_[li > 0 ? li - 1 : 0].pose.position.x);
    if (std::abs(angleDiff(d1, d0)) > curve_angle_) {situ = std::min(situ, curve_speed_); reason = "curve";}
  }
  // obstacle cost along the path ahead
  const double look = std::max(speed_lookahead_dist_, 2.0 * v);
  double acc = 0.0; int worst = 0; bool tight = false;
  for (size_t i = 1; i < path_.size() && acc < look; ++i) {
    acc += dist(path_[i - 1], path_[i]);
    const int c = getCost(path_[i]);
    if (useFootprint()) {
      // footprint clearance below 2 x padding ahead = tight passage
      if (i + 1 < path_.size() && !footprintFree(path_[i].pose.position.x, path_[i].pose.position.y, yawOf(path_[i]), tight_clearance_)) {
        tight = true;
      }
      if (c >= inflation_cost_threshold_ && c < 95) {worst = std::max(worst, c);}
      continue;
    }
    if (c >= 95 || c == -999) {worst = 95; break;}
    if (c >= inflation_cost_threshold_) {worst = std::max(worst, c);}
  }
  tight_now_ = tight;
  if (tight && tight_passage_speed_ < situ) {situ = tight_passage_speed_; reason = "tight_passage";}
  if (worst >= 95) {
    if (near_obstacle_speed_ < situ) {situ = near_obstacle_speed_; reason = "near_obstacle";}
  } else if (worst > 0 && !speed_profile_enabled_) {
    if (inflation_speed_ < situ) {situ = inflation_speed_; reason = "inflation";}
  }
  const double floored = std::max(situ, governor_floor_);
  if (floored < cap) {cap = floored;} else {reason = (cap < vmax) ? "ramp" : "none";}
  // braking toward the waypoint is not floored (the robot has to stop there)
  double brake = std::max(dist_to_goal, 0.0);
  if (linear_dec_limit_ > 0.0) {brake = std::min(brake, std::sqrt(2.0 * linear_dec_limit_ * std::max(dist_to_goal, 0.0)));}
  if (brake < cap) {cap = brake; reason = "goal_brake";}
  gov_reason_ = reason + " cap=" + std::to_string(cap).substr(0, 4);
  return cap;
}

// lookaheadPoint: walk the path from the robot by arc length L = max(lookahead_min, lookahead_gain * v)
// and return the direction of travel to that point and the path tangent there.
bool MidproFsmController::lookaheadPoint(double & dir, double & tangent) const
{
  if (path_.size() < 2) {return false;}
  const double x = path_[0].pose.position.x, y = path_[0].pose.position.y;
  const double v_prev = std::hypot(prev_linear_x_, prev_linear_y_);
  const double L = std::max(lookahead_min_, lookahead_gain_ * v_prev);
  double acc = 0.0; size_t li = path_.size() - 1;
  for (size_t i = 1; i < path_.size(); ++i) {
    acc += dist(path_[i - 1], path_[i]);
    if (acc >= L) {li = i; break;}
  }
  const double lx = path_[li].pose.position.x, ly = path_[li].pose.position.y;
  dir = std::atan2(ly - y, lx - x);
  if (li + 1 < path_.size()) {
    tangent = std::atan2(path_[li + 1].pose.position.y - ly, path_[li + 1].pose.position.x - lx);
  } else if (li > 0) {
    tangent = std::atan2(ly - path_[li - 1].pose.position.y, lx - path_[li - 1].pose.position.x);
  } else {
    tangent = dir;
  }
  return true;
}

// lookaheadTick: smooth path following for a holonomic (swerve) base.
//   velocity vector -> lookahead point;  body heading -> path tangent (P control, rate-limited)
//   rotate in place -> only when the travel direction differs from the heading by > rotate_threshold
// Lateral crabbing (vy) absorbs the heading lag so the body never has to stop to re-align.
bool MidproFsmController::lookaheadTick(geometry_msgs::msg::Twist & cmd, double dist_to_goal)
{
  double dir, tangent;
  if (!lookaheadPoint(dir, tangent)) {return false;}
  const double yaw = yawOf(path_[0]);
  const double yaw_error = angleDiff(dir, yaw);          // travel direction vs body heading
  const bool holonomic = (holonomic_policy_ == "always") || (dist_to_goal <= holonomic_zone_);

  if (std::abs(yaw_error) > rotate_threshold_) {
    setCtrlState("ROTATE_IN_PLACE", "travel direction vs heading > rotate_threshold (lookahead)");
    return true;
  }
  const double v_prev = std::hypot(prev_linear_x_, prev_linear_y_);
  double v = std::min(max_linear_vel_ * speed_scale_, v_prev + linear_acc_limit_ / controller_frequency_);

  if (!holonomic) {
    // car-like transit: no lateral motion, the body heading IS the travel direction (pure pursuit)
    v = std::max(min_linear_vel_, speedGovernor(v, dist_to_goal, yaw_error));
    cmd.linear.x = v * std::max(0.0, std::cos(yaw_error));
    cmd.linear.y = 0.0;
    cmd.angular.z = std::clamp(steer_gain_ * yaw_error, -max_angular_vel_, max_angular_vel_);
    return true;
  }
  // goal zone: slide toward the goal while the body turns to the goal yaw (straight-in or side parking)
  const double goal_yaw = yawOf(path_.back());
  const double heading_error = angleDiff(goal_yaw, yaw);
  v = std::max(min_linear_vel_, speedGovernor(v, dist_to_goal, 0.0));
  cmd.linear.x = v * std::cos(yaw_error);
  cmd.linear.y = v * std::sin(yaw_error);
  cmd.angular.z = std::clamp(heading_gain_ * heading_error, -max_angular_vel_, max_angular_vel_);
  return true;
}

}  // namespace midpro_nav2_controller

PLUGINLIB_EXPORT_CLASS(midpro_nav2_controller::MidproFsmController, nav2_core::Controller)
