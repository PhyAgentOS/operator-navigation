// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include "navigation_sim_mppi_critics/segment_goal_angle_critic.hpp"
#include "navigation_sim_mppi_critics/segment_goal_critic.hpp"

template<class Base>
class ConfiguredCritic : public Base
{
public:
  void configure(bool enabled = true, unsigned int power = 1, bool symmetric = false)
  {
    this->enabled_ = enabled;
    this->threshold_to_consider_ = 0.5f;
    this->weight_ = 3.0f;
    this->power_ = power;
    this->symmetric_yaw_tolerance_ = symmetric;
  }
};
template<class Base>
class ConfiguredPositionCritic : public Base
{
public:
  void configure(bool enabled = true, unsigned int power = 1)
  {
    this->enabled_ = enabled;
    this->threshold_to_consider_ = 0.8f;
    this->weight_ = 5.0f;
    this->power_ = power;
  }
};
void require(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}
void close(float actual, float expected, const std::string & message)
{
  require(std::abs(actual - expected) < 1e-5f, message);
}

int main()
{
  try {
    ConfiguredCritic<mppi::critics::SegmentGoalAngleCritic> segment;
    ConfiguredCritic<mppi::critics::GoalAngleCritic> original;
    segment.configure(); original.configure();
    mppi::models::State state; state.reset(2, 4);
    mppi::models::Trajectories trajectories; trajectories.reset(2, 4);
    mppi::models::Path path; path.reset(1);
    geometry_msgs::msg::Pose goal;
    float dt = .05f;
    xt::xtensor<float, 1> costs = xt::zeros<float>({2});
    mppi::CriticData data{state, trajectories, path, goal, costs, dt,
      false, nullptr, nullptr, std::nullopt, std::nullopt};

    // Frozen DSH failure: XY is inside inversion tolerance; yaw is outside it;
    // global goal is 6m away, so stock GoalAngle scores neither candidate.
    state.pose.pose.position.x = 3.7637331421;
    state.pose.pose.position.y = -2.6997498734;
    path.x(0) = 3.74425991f; path.y(0) = -2.75562131f;
    path.yaws(0) = .959931135f;
    goal.position.x = 6.74126955; goal.position.y = -8.28332652;
    for (size_t j = 0; j < 4; ++j) {
      trajectories.yaws(0, j) = 1.222589116f;  // stationary failed orientation
      trajectories.yaws(1, j) = path.yaws(0);  // aligned with current segment
    }
    original.score(data);
    close(costs(0), 0, "Stock reproduces missing single-cusp yaw objective");
    segment.score(data);
    require(costs(0) > .75f && costs(1) < 1e-6f,
      "Segment objective must prefer cusp-aligned yaw without relaxing gate");
    close(static_cast<float>(goal.position.x), 6.74126955f, "Global goal input unchanged");
    require(!data.fail_flag && !data.path_pts_valid && !data.furthest_reached_path_point,
      "Critic must not mutate caller bookkeeping");
    std::cout << "PASS recorded remote single-point cusp regression\n";

    costs.fill(0); path.x(0) += 2.0f;
    segment.score(data);
    close(costs(0), 0, "Remote segment endpoint must not activate heading early");
    std::cout << "PASS remote segment inactive\n";

    // At an ordinary terminal goal, all native weighting/power/symmetry remain.
    path.x(0) = static_cast<float>(state.pose.pose.position.x);
    path.y(0) = static_cast<float>(state.pose.pose.position.y);
    goal.position = state.pose.pose.position;
    for (unsigned int power : {1u, 2u}) {
      for (bool symmetric : {false, true}) {
        segment.configure(true, power, symmetric); original.configure(true, power, symmetric);
        for (size_t j = 0; j < 4; ++j) {
          trajectories.yaws(0, j) = path.yaws(0) + 3.0f;
          trajectories.yaws(1, j) = path.yaws(0) - .1f;
        }
        costs.fill(0); original.score(data); auto expected = costs;
        costs.fill(0); segment.score(data);
        close(costs(0), expected(0), "Terminal native score parity candidate 0");
        close(costs(1), expected(1), "Terminal native score parity candidate 1");
      }
    }
    std::cout << "PASS native final-goal weight/power/symmetry parity\n";

    segment.configure(false); costs.fill(1); segment.score(data);
    close(costs(0), 1, "Disabled critic preserves costs");
    segment.configure(); path.reset(0); segment.score(data);
    close(costs(0), 1, "Empty path safely preserves costs");
    path.x = xt::zeros<float>({1}); segment.score(data);
    close(costs(0), 1, "Mismatched path safely preserves costs");
    std::cout << "PASS disabled/empty/malformed path handling\n";

    ConfiguredPositionCritic<mppi::critics::SegmentGoalCritic> segment_position;
    ConfiguredPositionCritic<mppi::critics::GoalCritic> original_position;
    segment_position.configure(); original_position.configure();
    path.reset(1); path.x(0) = .18f;
    state.pose.pose.position.x = 0; state.pose.pose.position.y = 0;
    goal.position.x = 6; goal.position.y = 6;
    for (size_t j = 0; j < 4; ++j) {
      trajectories.x(0, j) = 0; trajectories.y(0, j) = 0;
      trajectories.x(1, j) = .18f; trajectories.y(1, j) = 0;
    }
    costs.fill(0); original_position.score(data);
    close(costs(0), 0, "Native position score inactive at distant global goal");
    segment_position.score(data);
    close(costs(0), .9f, "XY outside inversion gate must retain segment position objective");
    close(costs(1), 0, "Segment endpoint should have zero position error");
    close(static_cast<float>(goal.position.x), 6, "Position critic preserves global goal");
    std::cout << "PASS single-point XY .18m outside .12m gate with remote global goal\n";

    for (unsigned int power : {1u, 2u}) {
      segment_position.configure(true, power); original_position.configure(true, power);
      goal.position.x = path.x(0); goal.position.y = path.y(0);
      costs.fill(0); original_position.score(data); auto expected = costs;
      costs.fill(0); segment_position.score(data);
      close(costs(0), expected(0), "Terminal native position power parity");
      close(costs(1), expected(1), "Terminal native position endpoint parity");
    }
    std::cout << "PASS native position scoring parity at final goal\n";
    segment_position.configure(); path.x(0) = 2; costs.fill(1);
    segment_position.score(data); close(costs(0), 1, "Remote segment position inactive");
    path.reset(0); segment_position.score(data);
    close(costs(0), 1, "Empty segment position safe");
    path.reset(1); segment_position.configure(false); segment_position.score(data);
    close(costs(0), 1, "Disabled segment position preserves costs");
    std::cout << "PASS position remote/empty/disabled guards\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
