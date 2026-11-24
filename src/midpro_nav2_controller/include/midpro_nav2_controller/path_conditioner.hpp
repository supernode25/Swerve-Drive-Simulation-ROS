// path_conditioner.hpp — clean up a grid planner path for a long swerve body (v0.81)
//
// Smac 2D plans for a point: it knows neither the heading nor the 0.9 m length of the body. The conditioner
// reshapes the plan once, against the global costmap and the real footprint, before the controller follows it:
//   1. start shift      first metre leaves sideways (45-135 deg from the heading) -> one straight, yaw-held
//                       segment to the first plan point whose direction is close to the heading
//   2. corridor pull    string pulling: the farthest plan point reachable on a footprint-free straight line,
//                       margins tried large -> small (a gap of width + 10 cm keeps its straight run)
//   3. goal staging     turning at the goal does not fit and no plan pose before it allows the turn ->
//                       reshape the end through a staging pose on the goal's axes (behind / left / right) so the
//                       plan ends with "staging pose -> straight slide with the goal yaw"; the controller's
//                       goal staging then finds that pose on the plan
// Pure geometry on a Costmap2D; no ROS communication (unit-testable).
#ifndef MIDPRO_NAV2_CONTROLLER__PATH_CONDITIONER_HPP_
#define MIDPRO_NAV2_CONTROLLER__PATH_CONDITIONER_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"

namespace midpro_nav2_controller
{

struct ConditionerParams
{
  double horizon = 25.0;            // m of the plan that is conditioned (the rest is redone on the next replan)
  double interval = 0.05;           // m between output poses
  double start_check = 0.8;         // m of the plan that defines its initial direction
  double start_search = 3.0;        // m along the plan searched for the end of the start shift
  double start_align_tol = 0.52;    // rad: plan direction within this of the heading ends the shift (30 deg)
  double pull_max_seg = 6.0;        // m: longest straight segment created by string pulling
  std::vector<double> pull_margins = {0.06, 0.03, 0.01};
  double move_pad = 0.03;           // m around the body for shift / slide / connection segments
  double stage_min = 0.6;           // m: staging pose search range
  double stage_max = 2.5;
  double stage_connect = 5.0;       // m of plan end searched for a point that connects to an axis candidate
  double protect_end = 1.0;         // m before the goal never pulled (final approach geometry stays)
};

struct ConditionerReport
{
  bool start_shift = false;
  bool staged = false;
  int pulled_segments = 0;
  std::string note;
};

class PathConditioner
{
public:
  PathConditioner(const nav2_costmap_2d::Costmap2D * costmap,
    const std::vector<geometry_msgs::msg::Point> & footprint, const ConditionerParams & p);

  // path: map-frame plan (path.poses.front() = start, back() = goal incl. goal yaw); robot_yaw in the same frame
  ConditionerReport condition(nav_msgs::msg::Path & path, double robot_yaw) const;

  // geometry helpers (public for tests)
  bool poseFree(double x, double y, double yaw, double pad) const;
  bool translationFree(double x0, double y0, double x1, double y1, double yaw, double pad) const;
  bool segmentFree(double x0, double y0, double x1, double y1, double pad) const;   // yaw = segment direction
  bool rotSweepFree(double x, double y, double yaw0, double yaw1, double pad) const;
  // as translationFree, but a start pose that already overlaps (robot squeezed against a wall) skips the first 0.3 m
  bool escapeFree(double x0, double y0, double x1, double y1, double yaw, double pad) const;

private:
  const nav2_costmap_2d::Costmap2D * cm_;
  std::vector<geometry_msgs::msg::Point> fp_;
  ConditionerParams p_;

  bool startShift(std::vector<geometry_msgs::msg::PoseStamped> & poses, double robot_yaw, size_t & shift_end) const;
  bool goalStaging(std::vector<geometry_msgs::msg::PoseStamped> & poses, size_t & protect_from) const;
  int stringPull(std::vector<geometry_msgs::msg::PoseStamped> & poses, size_t first, size_t last) const;
  void appendLine(std::vector<geometry_msgs::msg::PoseStamped> & out, const geometry_msgs::msg::PoseStamped & a,
    double x1, double y1, double yaw, bool include_end) const;
};

}  // namespace midpro_nav2_controller

#endif  // MIDPRO_NAV2_CONTROLLER__PATH_CONDITIONER_HPP_
