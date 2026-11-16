"""Default managed navigation backend; robot topics and parameters are supplied by configuration."""
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = LaunchConfiguration("params_file")
    command, smoothed = LaunchConfiguration("command_topic"), LaunchConfiguration("smoothed_topic")
    definitions = [('planner_server','nav2_planner','planner_server',[]),('controller_server','nav2_controller','controller_server',[('cmd_vel',command)]),('bt_navigator','nav2_bt_navigator','bt_navigator',[]),('behavior_server','nav2_behaviors','behavior_server',[('cmd_vel',command)]),('velocity_smoother','nav2_velocity_smoother','velocity_smoother',[('cmd_vel',command),('cmd_vel_smoothed',smoothed)]),('navigation_collision_container','rclcpp_components','component_container_isolated',[])]
    names = ['planner_server', 'controller_server', 'velocity_smoother', 'collision_monitor', 'behavior_server', 'bt_navigator']
    actions = []
    for name, package, executable, remappings in definitions:
        # Scope the versioned planner plugin to the planner process. System ROS
        # packages and other backends retain their original search paths.
        environment = {}
        if name == "planner_server":
            plugin = Path(__file__).with_name("smac_overlay")
            if plugin.is_dir():
                for dependency in ("nav2_core", "nav2_costmap_2d"):
                    manifest = Path(get_package_share_directory(dependency)) / "package.xml"
                    if ET.parse(manifest).getroot().findtext("version") != "1.3.13":
                        raise RuntimeError("Bundled Smac plugin requires the validated Nav2 1.3.13 ABI")
                environment = {
                    "AMENT_PREFIX_PATH": str(plugin) + os.pathsep + os.environ.get("AMENT_PREFIX_PATH", ""),
                    "LD_LIBRARY_PATH": str(plugin / "lib") + os.pathsep + os.environ.get("LD_LIBRARY_PATH", ""),
                }
        node = Node(name=name, package=package, executable=executable, output="screen",
                    parameters=[params], remappings=remappings, additional_env=environment)
        actions += [node, RegisterEventHandler(OnProcessExit(target_action=node,
                    on_exit=[EmitEvent(event=Shutdown(reason=name+" exited"))]))]
    actions.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", output="screen", parameters=[{
            "autostart": ParameterValue(LaunchConfiguration("autostart"),value_type=bool),
            "node_names": names, "bond_timeout": 4.0}]))
    return LaunchDescription([DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("mode", default_value="navigation"),
        DeclareLaunchArgument("autostart", default_value="false"),
        DeclareLaunchArgument("command_topic", default_value="/cmd_vel_nav"),
        DeclareLaunchArgument("smoothed_topic", default_value="/cmd_vel_smoothed"), *actions])
