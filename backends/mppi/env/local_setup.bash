# Source after the isolated ROS environment. No system files are modified.
nav_sim_critic_prefix="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export AMENT_PREFIX_PATH="$nav_sim_critic_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$nav_sim_critic_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$nav_sim_critic_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
unset nav_sim_critic_prefix
