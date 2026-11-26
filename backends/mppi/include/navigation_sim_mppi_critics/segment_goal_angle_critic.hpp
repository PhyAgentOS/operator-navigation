// SPDX-License-Identifier: Apache-2.0
#ifndef NAVIGATION_SIM_MPPI_CRITICS__SEGMENT_GOAL_ANGLE_CRITIC_HPP_
#define NAVIGATION_SIM_MPPI_CRITICS__SEGMENT_GOAL_ANGLE_CRITIC_HPP_

#include "nav2_mppi_controller/critics/goal_angle_critic.hpp"

namespace mppi::critics
{
// Retains the installed GoalAngle initialization, weighting and yaw scoring.
// Only the distance activation target changes to match that yaw's endpoint.
class SegmentGoalAngleCritic : public GoalAngleCritic
{
public:
  void score(CriticData & data) override;
};
}  // namespace mppi::critics
#endif
