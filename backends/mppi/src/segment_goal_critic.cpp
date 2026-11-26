// SPDX-License-Identifier: Apache-2.0
#include "navigation_sim_mppi_critics/segment_goal_critic.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{
void SegmentGoalCritic::score(CriticData & data)
{
  const auto count = data.path.x.size();
  if (!enabled_ || count == 0 || data.path.y.size() != count ||
    data.path.yaws.size() != count)
  {
    return;
  }
  auto segment_goal = data.goal;
  segment_goal.position.x = data.path.x(count - 1);
  segment_goal.position.y = data.path.y(count - 1);
  CriticData segment_data{
    data.state, data.trajectories, data.path, segment_goal, data.costs,
    data.model_dt, data.fail_flag, data.goal_checker, data.motion_model,
    data.path_pts_valid, data.furthest_reached_path_point};
  GoalCritic::score(segment_data);
}
}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::SegmentGoalCritic, mppi::critics::CriticFunction)
