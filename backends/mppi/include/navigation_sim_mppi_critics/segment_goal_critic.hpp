// SPDX-License-Identifier: Apache-2.0
#ifndef NAVIGATION_SIM_MPPI_CRITICS__SEGMENT_GOAL_CRITIC_HPP_
#define NAVIGATION_SIM_MPPI_CRITICS__SEGMENT_GOAL_CRITIC_HPP_

#include "nav2_mppi_controller/critics/goal_critic.hpp"

namespace mppi::critics
{
class SegmentGoalCritic : public GoalCritic
{
public:
  void score(CriticData & data) override;
};
}  // namespace mppi::critics
#endif
