// SPDX-License-Identifier: Apache-2.0
#include <iostream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include "pluginlib/class_loader.hpp"
#include "nav2_mppi_controller/critic_function.hpp"

int main()
{
  try {
    pluginlib::ClassLoader<mppi::critics::CriticFunction> loader(
      "nav2_mppi_controller", "mppi::critics::CriticFunction");
    for (const std::string name : {
      "SegmentGoalAngleCritic", "SegmentGoalCritic", "GoalAngleCritic", "GoalCritic"})
    {
      auto plugin = loader.createUniqueInstance("mppi::critics::" + name);
      if (!plugin || std::string(typeid(*plugin).name()).find(name) == std::string::npos) {
        throw std::runtime_error("Wrong dynamic plugin type: " + name);
      }
      std::cout << "PASS independent pluginlib load " << name << '\n';
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
