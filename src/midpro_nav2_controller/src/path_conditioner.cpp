// path_conditioner.cpp — see path_conditioner.hpp
#include "midpro_nav2_controller/path_conditioner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"

namespace midpro_nav2_controller
{

namespace
{
using PoseStamped = geometry_msgs::msg::PoseStamped;

double yawOfQ(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
double angDiff(double a, double b)
{
  double d = a - b;
  while (d > M_PI) {d -= 2.0 * M_PI;}
  while (d < -M_PI) {d += 2.0 * M_PI;}
  return d;
}
double pdist(const PoseStamped & a, const PoseStamped & b)
{
  return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
}
void setYaw(PoseStamped & p, double yaw)
{
  p.pose.orientation.x = 0.0; p.pose.orientation.y = 0.0;
  p.pose.orientation.z = std::sin(yaw / 2.0); p.pose.orientation.w = std::cos(yaw / 2.0);
}
}  // namespace

PathConditioner::PathConditioner(const nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint, const ConditionerParams & p)
: cm_(costmap), fp_(footprint), p_(p)
{
}

// ---------------------------------------------------------------- geometry
bool PathConditioner::poseFree(double x, double y, double yaw, double pad) const
{
  if (!cm_ || fp_.empty()) {return true;}
  unsigned int mx, my;
  auto blocked = [&](double wx, double wy) {
      if (!cm_->worldToMap(wx, wy, mx, my)) {return true;}
      const unsigned char c = cm_->getCost(mx, my);
      return c == nav2_costmap_2d::NO_INFORMATION || c >= nav2_costmap_2d::LETHAL_OBSTACLE;
    };
  if (blocked(x, y)) {return false;}
  double fx = -1e9, rx = 1e9, hw = 0.0;
  for (const auto & q : fp_) {fx = std::max(fx, q.x); rx = std::min(rx, q.x); hw = std::max(hw, std::abs(q.y));}
  fx += pad; rx -= pad; hw += pad;
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double step = std::max(0.02, cm_->getResolution() * 0.5);
  // perimeter + longitudinal lines through the interior (a thin object fully inside the body is caught too)
  for (double ly = -hw; ly <= hw + 1e-9; ly += std::max(step, (2.0 * hw) / 4.0)) {
    for (double lx = rx; lx <= fx + 1e-9; lx += step) {
      if (blocked(x + c * lx - s * ly, y + s * lx + c * ly)) {return false;}
    }
  }
  for (double ly = -hw; ly <= hw + 1e-9; ly += step) {
    if (blocked(x + c * fx - s * ly, y + s * fx + c * ly)) {return false;}
    if (blocked(x + c * rx - s * ly, y + s * rx + c * ly)) {return false;}
  }
  return true;
}

bool PathConditioner::translationFree(double x0, double y0, double x1, double y1, double yaw, double pad) const
{
  const double len = std::hypot(x1 - x0, y1 - y0);
  const double step = cm_ ? std::max(0.025, 2.0 * cm_->getResolution()) : 0.05;
  const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
  for (int k = 1; k <= n; ++k) {
    const double t = static_cast<double>(k) / n;
    if (!poseFree(x0 + t * (x1 - x0), y0 + t * (y1 - y0), yaw, pad)) {return false;}
  }
  return true;
}

bool PathConditioner::escapeFree(double x0, double y0, double x1, double y1, double yaw, double pad) const
{
  const bool start_free = poseFree(x0, y0, yaw, pad);
  const double len = std::hypot(x1 - x0, y1 - y0);
  const double step = cm_ ? std::max(0.025, 2.0 * cm_->getResolution()) : 0.05;
  const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
  for (int k = 1; k <= n; ++k) {
    const double t = static_cast<double>(k) / n;
    if (!start_free && t * len < 0.3) {continue;}
    if (!poseFree(x0 + t * (x1 - x0), y0 + t * (y1 - y0), yaw, pad)) {return false;}
  }
  return true;
}

bool PathConditioner::segmentFree(double x0, double y0, double x1, double y1, double pad) const
{
  return translationFree(x0, y0, x1, y1, std::atan2(y1 - y0, x1 - x0), pad);
}

bool PathConditioner::rotSweepFree(double x, double y, double yaw0, double yaw1, double pad) const
{
  const double d = angDiff(yaw1, yaw0);
  const int n = std::max(1, static_cast<int>(std::ceil(std::abs(d) / (M_PI / 36.0))));
  for (int k = 1; k <= n; ++k) {
    if (!poseFree(x, y, yaw0 + d * k / n, pad)) {return false;}
  }
  return true;
}

void PathConditioner::appendLine(std::vector<PoseStamped> & out, const PoseStamped & a,
  double x1, double y1, double yaw, bool include_end) const
{
  const double x0 = a.pose.position.x, y0 = a.pose.position.y;
  const double len = std::hypot(x1 - x0, y1 - y0);
  const int n = std::max(1, static_cast<int>(std::ceil(len / std::max(p_.interval, 0.01))));
  for (int k = 0; k < n + (include_end ? 1 : 0); ++k) {
    PoseStamped p = a;
    const double t = static_cast<double>(k) / n;
    p.pose.position.x = x0 + t * (x1 - x0);
    p.pose.position.y = y0 + t * (y1 - y0);
    setYaw(p, yaw);
    out.push_back(p);
  }
}

// ---------------------------------------------------------------- 1. start shift
bool PathConditioner::startShift(std::vector<PoseStamped> & P, double robot_yaw, size_t & shift_end) const
{
  shift_end = 0;
  if (P.size() < 4) {return false;}
  double acc = 0.0; size_t i0 = 1;
  for (; i0 < P.size(); ++i0) {acc += pdist(P[i0 - 1], P[i0]); if (acc >= p_.start_check) {break;}}
  if (i0 >= P.size()) {return false;}
  const double d0 = std::atan2(P[i0].pose.position.y - P[0].pose.position.y, P[i0].pose.position.x - P[0].pose.position.x);
  const double e = std::abs(angDiff(d0, robot_yaw));
  if (e < M_PI / 4.0 || e > 3.0 * M_PI / 4.0) {return false;}        // not leaving sideways
  acc = 0.0;
  for (size_t i = 1; i + 1 < P.size(); ++i) {
    acc += pdist(P[i - 1], P[i]);
    if (acc > p_.start_search) {break;}
    if (acc < 0.3) {continue;}
    size_t j = i; double a2 = 0.0;
    while (j + 1 < P.size() && a2 < 0.4) {a2 += pdist(P[j], P[j + 1]); ++j;}
    const double dl = std::atan2(P[j].pose.position.y - P[i].pose.position.y, P[j].pose.position.x - P[i].pose.position.x);
    if (std::abs(angDiff(dl, robot_yaw)) > p_.start_align_tol) {continue;}
    if (!escapeFree(P[0].pose.position.x, P[0].pose.position.y, P[i].pose.position.x, P[i].pose.position.y,
      robot_yaw, p_.move_pad)) {continue;}
    std::vector<PoseStamped> out;
    appendLine(out, P[0], P[i].pose.position.x, P[i].pose.position.y, robot_yaw, false);
    shift_end = out.size();                                     // first pose of the original plan after the shift
    out.insert(out.end(), P.begin() + static_cast<long>(i), P.end());
    P.swap(out);
    return true;
  }
  return false;
}

// ---------------------------------------------------------------- 3. goal staging (axis candidates)
bool PathConditioner::goalStaging(std::vector<PoseStamped> & P, size_t & protect_from) const
{
  const size_t n = P.size();
  if (n < 4) {return false;}
  const PoseStamped G = P.back();
  const double gx = G.pose.position.x, gy = G.pose.position.y, gyaw = yawOfQ(G.pose.orientation);
  size_t ia = n - 1; double acc = 0.0;
  while (ia > 0 && acc < 0.5) {acc += pdist(P[ia - 1], P[ia]); --ia;}
  const double arrive = std::atan2(gy - P[ia].pose.position.y, gx - P[ia].pose.position.x);
  if (std::abs(angDiff(gyaw, arrive)) < 0.26) {return false;}              // arrives aligned
  if (rotSweepFree(gx, gy, arrive, gyaw, p_.move_pad)) {return false;}     // turning at the goal fits
  // a staging pose already on the plan? then the controller's goal staging handles it
  acc = 0.0;
  for (size_t i = n - 1; i > 0; --i) {
    acc += pdist(P[i - 1], P[i]);
    if (acc < p_.stage_min) {continue;}
    if (acc > p_.stage_max) {break;}
    const auto & c = P[i - 1].pose.position;
    const double here = std::atan2(P[i].pose.position.y - c.y, P[i].pose.position.x - c.x);
    if (poseFree(c.x, c.y, gyaw, p_.move_pad) && rotSweepFree(c.x, c.y, here, gyaw, p_.move_pad) &&
      translationFree(c.x, c.y, gx, gy, gyaw, p_.move_pad)) {return false;}
  }
  // candidates on the goal's axes: behind (straight in), left / right (side in)
  std::vector<double> cum(n, 0.0);
  for (size_t i = 1; i < n; ++i) {cum[i] = cum[i - 1] + pdist(P[i - 1], P[i]);}
  const double cg = std::cos(gyaw), sg = std::sin(gyaw);
  const double dirs[3][2] = {{-cg, -sg}, {-sg, cg}, {sg, -cg}};
  double best = std::numeric_limits<double>::infinity();
  size_t best_m = 0; double bx = 0.0, by = 0.0;
  for (double k = p_.stage_min; k <= p_.stage_max + 1e-9; k += 0.1) {
    for (const auto & d : dirs) {
      const double cx = gx + k * d[0], cy = gy + k * d[1];
      if (!poseFree(cx, cy, gyaw, p_.move_pad)) {continue;}
      if (!translationFree(cx, cy, gx, gy, gyaw, p_.move_pad)) {continue;}
      for (size_t m = n - 1; m > 0; --m) {                    // closest plan point that connects
        if (cum[n - 1] - cum[m - 1] > p_.stage_connect) {break;}
        const auto & pm = P[m - 1].pose.position;
        const double th = std::atan2(cy - pm.y, cx - pm.x);
        if (!rotSweepFree(cx, cy, th, gyaw, p_.move_pad)) {continue;}
        if (!segmentFree(pm.x, pm.y, cx, cy, p_.move_pad)) {continue;}
        const double cost = cum[m - 1] + std::hypot(cx - pm.x, cy - pm.y) + k + 0.2 * std::abs(angDiff(gyaw, th));
        if (cost < best) {best = cost; best_m = m - 1; bx = cx; by = cy;}
        break;
      }
    }
  }
  if (!std::isfinite(best)) {return false;}
  std::vector<PoseStamped> out(P.begin(), P.begin() + static_cast<long>(best_m));
  const double th = std::atan2(by - P[best_m].pose.position.y, bx - P[best_m].pose.position.x);
  appendLine(out, P[best_m], bx, by, th, false);
  protect_from = out.size();                                   // index of the staging pose
  PoseStamped st = P[best_m]; st.pose.position.x = bx; st.pose.position.y = by;
  appendLine(out, st, gx, gy, gyaw, false);
  out.push_back(G);                                            // real goal (with its yaw) stays back()
  P.swap(out);
  return true;
}

// ---------------------------------------------------------------- 2. corridor pull
int PathConditioner::stringPull(std::vector<PoseStamped> & P, size_t first, size_t last) const
{
  if (last >= P.size() || last <= first + 3) {return 0;}
  std::vector<PoseStamped> out(P.begin(), P.begin() + static_cast<long>(first));
  size_t i = first; int pulled = 0;
  while (i < last) {
    std::vector<size_t> cand; double acc = 0.0;
    for (size_t j = i + 1; j <= last; ++j) {
      acc += pdist(P[j - 1], P[j]);
      if (acc > p_.pull_max_seg) {break;}
      cand.push_back(j);
    }
    size_t best = 0;
    for (const double pad : p_.pull_margins) {
      for (size_t k = cand.size(); k > 0; k = (k > 5 ? k - 5 : 0)) {
        const size_t j = cand[k - 1];
        if (j <= best) {break;}
        if (segmentFree(P[i].pose.position.x, P[i].pose.position.y, P[j].pose.position.x, P[j].pose.position.y, pad)) {
          best = j; break;
        }
      }
      if (best >= i + 3) {break;}
    }
    if (best >= i + 3) {
      const double yaw = std::atan2(P[best].pose.position.y - P[i].pose.position.y, P[best].pose.position.x - P[i].pose.position.x);
      appendLine(out, P[i], P[best].pose.position.x, P[best].pose.position.y, yaw, false);
      i = best; ++pulled;
    } else {
      out.push_back(P[i]); ++i;
    }
  }
  out.insert(out.end(), P.begin() + static_cast<long>(last), P.end());
  P.swap(out);
  return pulled;
}

// ---------------------------------------------------------------- driver
ConditionerReport PathConditioner::condition(nav_msgs::msg::Path & path, double robot_yaw) const
{
  ConditionerReport rep;
  auto & P = path.poses;
  if (!cm_ || fp_.empty() || P.size() < 4) {return rep;}
  double total = 0.0;
  for (size_t i = 1; i < P.size(); ++i) {total += pdist(P[i - 1], P[i]);}

  // protected tail: final approach geometry (and, if staged, the staging pose -> slide segment)
  size_t protect_from = P.size() - 1;
  {double acc = 0.0; while (protect_from > 0 && acc < p_.protect_end) {acc += pdist(P[protect_from - 1], P[protect_from]); --protect_from;}}
  if (total <= p_.horizon && goalStaging(P, protect_from)) {
    rep.staged = true; rep.note += "staged ";
  }
  auto shifted = [](size_t idx, size_t before, size_t after) {
      return static_cast<size_t>(std::max<long>(0, static_cast<long>(idx) + static_cast<long>(after) - static_cast<long>(before)));
    };
  const size_t before = P.size();
  size_t shift_end = 0;
  if (startShift(P, robot_yaw, shift_end)) {
    rep.start_shift = true; rep.note += "start-shift ";
    protect_from = shifted(protect_from, before, P.size());
  }
  // pull inside the horizon only
  size_t last = protect_from;
  {double acc = 0.0; for (size_t i = shift_end + 1; i < last; ++i) {acc += pdist(P[i - 1], P[i]); if (acc > p_.horizon) {last = i; break;}}}
  const size_t size_before_pull = P.size();
  rep.pulled_segments = stringPull(P, shift_end, last);
  if (rep.pulled_segments) {rep.note += "pulled ";}
  protect_from = shifted(protect_from, size_before_pull, P.size());
  // orientations: tangent on followed parts, held yaw on the start shift and the staging slide, goal yaw last
  const size_t tangent_end = rep.staged ? std::min(protect_from, P.size() - 1) : P.size() - 1;
  for (size_t i = shift_end; i < tangent_end && i + 1 < P.size(); ++i) {
    setYaw(P[i], std::atan2(P[i + 1].pose.position.y - P[i].pose.position.y, P[i + 1].pose.position.x - P[i].pose.position.x));
  }
  return rep;
}

}  // namespace midpro_nav2_controller
