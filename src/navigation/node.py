"""Own Nav2 lifecycle/actions and gate safe ROS velocities into Forge commands."""

import json
import math
import os
import time
from pathlib import Path

import yaml
from dora import Node

from navigation.arrival import check_arrival
from navigation.costmap_reset import CostmapRefresh
from navigation.policy import PolicyEndpoint
from navigation.process import OwnedProcess
from navigation.quality import planar_pose
from navigation.velocity import VelocityGate


def run(cfg: dict) -> None:
    import rclpy
    from action_msgs.msg import GoalStatus
    from composition_interfaces.srv import ListNodes, LoadNode
    from geometry_msgs.msg import Twist
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.srv import ManageLifecycleNodes
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter as ROSParameter
    from tf2_ros import Buffer, TransformListener

    rclpy.init(domain_id=cfg["ros_domain_id"])
    ros = rclpy.create_node("forge_nav2_adapter")
    buffer = Buffer()
    _listener = TransformListener(buffer, ros)
    client = ActionClient(ros, NavigateToPose, "/navigate_to_pose")
    lifecycle = ros.create_client(GetState, "/bt_navigator/get_state")
    lifecycle_future = None
    lifecycle_sent_at = 0.0
    manager_client = ros.create_client(
        ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes"
    )
    # Empty arrays need explicit ROS types; apply them before lifecycle configure
    # so native default plugins/polygons cannot become active even briefly.
    empty_arrays = {}
    if not cfg.get("progress_check_enabled", True):
        empty_arrays["controller_server"] = "progress_checker_plugins"
    parameter_clients = {
        name: ros.create_client(SetParameters, f"/{name}/set_parameters") for name in empty_arrays
    }
    parameter_futures = {}
    parameter_ready_at = {}
    manager_future = None
    collision_client = ros.create_client(
        LoadNode, "/navigation_collision_container/_container/load_node"
    )
    collision_future = None
    collision_ready_at = None
    collision_list = ros.create_client(
        ListNodes, "/navigation_collision_container/_container/list_nodes"
    )
    collision_list_future, collision_list_at = None, 0.0
    collision_loaded = False
    parameter_sent_at = {}
    manager_ready_at = None

    def collision_parameters():
        values = yaml.safe_load(Path(cfg["backend_params"]).read_text())["collision_monitor"][
            "ros__parameters"
        ]
        parameters = []

        def flatten(items, prefix=""):
            for name, value in items.items():
                name = prefix + name
                if isinstance(value, dict):
                    flatten(value, name + ".")
                else:
                    parameters.append(ROSParameter(name, value=value).to_parameter_msg())

        flatten(values)
        if not cfg.get("fixed_stop_zone_enabled", True):
            parameters.append(
                Parameter(
                    name="polygons",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=[]
                    ),
                )
            )
        return parameters

    dora = Node()
    endpoint = PolicyEndpoint(dora, "navigation")
    backend = OwnedProcess()
    gate = VelocityGate(
        max_vx=cfg.get("max_vx_mps", 0.08),
        max_wz=cfg.get("max_wz_radps", 0.4),
        timeout_seconds=cfg.get("velocity_timeout_seconds", 0.25),
    )
    control, health = {}, {}
    state, start_request, start_deadline = "idle", None, 0
    current = handle = None
    goal_at = 0.0
    cancel_reason = None
    cancel_sent = False
    distance = None
    last_output = last_runtime = 0.0
    odom_pose, odom_at, initial_pose, initial_odom, target_pose = None, 0.0, None, None, None
    wheel_motion = False
    costmap_refresh = CostmapRefresh(ros)
    refresh_request = None

    def map_pose():
        tf = buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
        if abs(ros.get_clock().now().nanoseconds / 1e9 - stamp) > 1.0:
            raise ValueError("arrival TF stale")
        p, q = tf.transform.translation, tf.transform.rotation
        return [p.x, p.y, 2 * math.atan2(q.z, q.w)]

    def on_odom(msg):
        nonlocal odom_pose, odom_at
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        odom_pose, odom_at = [p.x, p.y, 2 * math.atan2(q.z, q.w)], time.monotonic()

    ros.create_subscription(Odometry, cfg.get("odometry_topic", "/odom"), on_odom, 20)

    def permitted(now):
        return (
            control.get("armed") is True
            and health.get("healthy") is True
            and 0 <= now - control.get("issued_monotonic", 0) < 0.35
            and 0 <= now - health.get("issued_monotonic", 0) < 0.65
        )

    def on_velocity(msg):
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
            gate.set_enabled(False)
            return
        gate.receive(msg.linear.x, msg.linear.y, msg.angular.z, now=time.monotonic())

    ros.create_subscription(Twist, cfg.get("safe_velocity_topic", "/cmd_vel"), on_velocity, 10)

    def request_cancel(reason):
        nonlocal cancel_reason, cancel_sent
        gate.set_enabled(False)
        if current is not None:
            cancel_reason = reason
            if handle is not None and not cancel_sent:
                handle.cancel_goal_async()
                cancel_sent = True

    def on_feedback(msg):
        nonlocal distance
        distance = float(msg.feedback.distance_remaining)

    def on_result(future, request):
        nonlocal current, handle, cancel_reason, cancel_sent, distance
        if current is not request:
            return
        gate.set_enabled(False)
        result = future.result()
        if current is not None:
            ok = result.status == GoalStatus.STATUS_SUCCEEDED and cancel_reason is None
            canceled = result.status == GoalStatus.STATUS_CANCELED or cancel_reason is not None
            evidence = {}
            if ok:
                try:
                    if time.monotonic() - odom_at > 0.5:
                        raise ValueError("arrival odometry stale")
                    evidence = check_arrival(
                        target_pose,
                        initial_pose,
                        map_pose(),
                        initial_odom,
                        odom_pose,
                        wheel_motion=wheel_motion,
                        require_motion=cfg.get("require_motion_evidence", False),
                    )
                    ok = evidence["verified"]
                except Exception as exc:
                    ok, evidence = (
                        False,
                        {"verified": False, "reason": "无法核验到达位置：" + str(exc)},
                    )
            code = int(getattr(result.result, "error_code", 0))
            failure_message = {
                205: "机器人起点被代价地图判为障碍区域",
                206: "目标位置被代价地图判为障碍区域，请选择有足够空间的位置",
                207: "路径规划超时",
                208: "找不到可通行路径；障碍刷新与重规划后仍未恢复，请检查目标及通道",
            }.get(code, f"Nav2 未完成目标（错误码 {code}）")
            endpoint.emit(
                current,
                "done" if (ok or canceled) else "error",
                "已到达目标位置和朝向"
                if ok
                else cancel_reason or evidence.get("reason", failure_message),
                {
                    "reached": ok,
                    "canceled": canceled,
                    "ros_action_status": result.status,
                    "error_code": code,
                    "error_message": str(getattr(result.result, "error_msg", "")),
                    "arrival": evidence,
                    "result": "succeeded" if ok else "cancelled" if canceled else "failed",
                },
            )
        current = handle = None
        cancel_reason, cancel_sent, distance = None, False, None

    def on_accepted(future, request):
        nonlocal handle, current, cancel_reason, cancel_sent
        received = future.result()
        if current is not request:
            if received.accepted:
                received.cancel_goal_async()
            return
        handle = received
        if not handle.accepted:
            endpoint.emit(current, "rejected", "Nav2 rejected the goal")
            current = handle = None
            cancel_reason, cancel_sent = None, False
            return
        handle.get_result_async().add_done_callback(lambda f: on_result(f, request))
        if cancel_reason is not None or not permitted(time.monotonic()):
            request_cancel(cancel_reason or "行驶授权已失效")
        else:
            endpoint.emit(current, "running", "Nav2 正在规划并执行目标")

    endpoint.runtime(message="Nav2 adapter ready; backend idle")
    try:
        for event in dora:
            if event["type"] == "STOP" or event.get("id") == "finish":
                break
            if event["type"] == "ERROR":
                raise RuntimeError(str(event))
            if event["type"] != "INPUT":
                continue
            if event["id"] in ("control", "localization_health"):
                value = json.loads(event["value"][0].as_py())
                if event["id"] == "control":
                    control = value
                else:
                    health = value
            if event["id"] == "base_feedback" and current is not None:
                value = json.loads(event["value"][0].as_py())
                if 0 <= time.monotonic() - value.get("issued_monotonic", 0) < 0.4:
                    wheel_motion |= (
                        max([abs(v) for v in value.get("wheel_speed", {}).values()] or [0]) > 0.05
                    )
            if event["id"] == "policy_command":
                command = endpoint.receive(event)
                if command:
                    if command.command == "start" and state == "idle":
                        parameter_futures = {}
                        parameter_ready_at = {}
                        manager_future = None
                        collision_future = None
                        collision_ready_at = None
                        collision_loaded = False
                        collision_list_future, collision_list_at = None, 0.0
                        parameter_sent_at = {}
                        manager_ready_at = None
                        backend.start(
                            [
                                "ros2",
                                "launch",
                                cfg["launch_file"],
                                "params_file:=" + cfg["backend_params"],
                                "mode:=navigation",
                                "autostart:=false",
                            ],
                            cwd=Path(cfg["session_dir"]),
                            log_path=Path(cfg["session_dir"]) / "nav2.log",
                            env={**os.environ, "ROS_DOMAIN_ID": str(cfg["ros_domain_id"])},
                        )
                        state, start_request, start_deadline = (
                            "starting",
                            command,
                            time.monotonic() + 60,
                        )
                        endpoint.emit(command, "running", "starting Nav2 servers")
                    elif command.command == "navigate_to_pose" and state == "running":
                        if (
                            current is not None
                            or refresh_request is not None
                            or not permitted(time.monotonic())
                        ):
                            endpoint.emit(
                                command, "rejected", "已有任务，或底盘/定位/页面授权未就绪"
                            )
                            continue
                        try:
                            x, y, yaw = planar_pose(command.inputs().get("pose"))
                            current_map_pose = map_pose()
                        except Exception as exc:
                            endpoint.emit(
                                command, "rejected", "target or TF unavailable: " + str(exc)
                            )
                            continue
                        if odom_pose is None or time.monotonic() - odom_at > 0.5:
                            endpoint.emit(command, "rejected", "odometry unavailable")
                            continue
                        initial_pose, initial_odom, target_pose = (
                            current_map_pose,
                            list(odom_pose),
                            [x, y, yaw],
                        )
                        wheel_motion = False
                        goal = NavigateToPose.Goal()
                        goal.pose.header.frame_id = "map"
                        goal.pose.header.stamp = ros.get_clock().now().to_msg()
                        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
                        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = (
                            math.sin(yaw / 2),
                            math.cos(yaw / 2),
                        )
                        current, goal_at = command, time.monotonic()
                        cancel_reason, cancel_sent = None, False
                        endpoint.emit(command, "accepted", "目标已提交给 Nav2")
                        client.send_goal_async(
                            goal, feedback_callback=on_feedback
                        ).add_done_callback(lambda f, request=command: on_accepted(f, request))
                    elif command.command == "clear_costmaps" and state == "running":
                        if (
                            current is not None
                            or control.get("armed")
                            or refresh_request is not None
                        ):
                            endpoint.emit(
                                command,
                                "rejected",
                                "clear_costmaps requires idle, unarmed navigation",
                            )
                            continue
                        refresh_request = command
                        costmap_refresh.start(time.monotonic())
                        endpoint.emit(command, "running", "刷新定位前遗留代价，等待新扫描")
                    elif command.command == "cancel":
                        target = command.inputs().get("target_request_id")
                        if target and (current is None or target != current.request_id):
                            endpoint.emit(
                                command, "rejected", "target_request_id is not the active task"
                            )
                            continue
                        request_cancel("用户停止了导航")
                        endpoint.emit(
                            command,
                            "done",
                            "行驶授权已撤销，已请求取消 Nav2 任务",
                            {
                                "cancellation_requested": current is not None,
                                "target_request_id": None
                                if current is None
                                else current.request_id,
                            },
                        )
                    elif command.command == "stop":
                        request_cancel("导航后端停止")
                        backend.stop()
                        state = "idle"
                        endpoint.emit(command, "done", "owned Nav2 backend stopped")
                    else:
                        endpoint.emit(command, "rejected", "unsupported command or lifecycle state")
            now = time.monotonic()
            allowed = (
                current is not None
                and handle is not None
                and cancel_reason is None
                and permitted(now)
            )
            if gate.enabled != allowed:
                gate.set_enabled(allowed)
            if current is not None and not permitted(now):
                request_cancel("定位、页面连接或行驶授权失效，已停车")
            if current is not None and now - goal_at > cfg.get("goal_timeout_seconds", 600):
                request_cancel("任务超时，已停车")
            if current is not None and handle is None and now - goal_at > 5:
                gate.set_enabled(False)
                endpoint.emit(
                    current,
                    "error",
                    "Nav2 goal acceptance timed out; late acceptance will be canceled",
                )
                current, cancel_reason, cancel_sent = None, None, False
            for _ in range(12):
                rclpy.spin_once(ros, timeout_sec=0)
            if state in ("starting", "running") and backend.poll() is not None:
                raise RuntimeError("Nav2 launch exited")
            if state == "starting":
                if collision_future is None and collision_client.service_is_ready():
                    if collision_ready_at is None:
                        collision_ready_at = now
                    # Allow DDS response-endpoint discovery to complete too.
                    if now - collision_ready_at > 1.0:
                        request = LoadNode.Request()
                        request.package_name = "nav2_collision_monitor"
                        request.plugin_name = "nav2_collision_monitor::CollisionMonitor"
                        request.node_name = "collision_monitor"
                        request.parameters = collision_parameters()
                        collision_future = collision_client.call_async(request)
                # A LoadNode response can be lost during early DDS discovery.
                # Never blindly retry a mutation: confirm the component exists.
                if (
                    collision_future is not None
                    and collision_list.service_is_ready()
                    and not collision_loaded
                ):
                    if collision_list_future is not None and collision_list_future.done():
                        listed = collision_list_future.result()
                        names = [] if listed is None else list(listed.full_node_names)
                        if names.count("/collision_monitor") > 1:
                            raise RuntimeError("duplicate collision monitor components")
                        collision_loaded = names.count("/collision_monitor") == 1
                        collision_list_future = None
                    elif collision_list_future is not None and now - collision_list_at > 1.5:
                        collision_list_future.cancel()
                        collision_list_future = None
                    if collision_list_future is None and now - collision_list_at > 0.5:
                        collision_list_future = collision_list.call_async(ListNodes.Request())
                        collision_list_at = now
                collision_ready = collision_loaded or (
                    collision_future is not None and collision_future.done()
                )
                if collision_future is not None and collision_future.done():
                    reply = collision_future.result()
                    if reply is None or not reply.success:
                        raise RuntimeError("could not load native collision monitor: " + str(reply))
                if manager_future is None:
                    for name, parameter_name in empty_arrays.items():
                        if (
                            name in parameter_futures
                            and not parameter_futures[name].done()
                            and now - parameter_sent_at[name] > 2.0
                        ):
                            # Setting the same typed empty array is idempotent.
                            parameter_futures[name].cancel()
                            del parameter_futures[name]
                        if (
                            name not in parameter_futures
                            and parameter_clients[name].service_is_ready()
                        ):
                            if name not in parameter_ready_at:
                                parameter_ready_at[name] = now
                            if now - parameter_ready_at[name] < 1.0:
                                continue
                            request = SetParameters.Request()
                            request.parameters = [
                                Parameter(
                                    name=parameter_name,
                                    value=ParameterValue(
                                        type=ParameterType.PARAMETER_STRING_ARRAY,
                                        string_array_value=[],
                                    ),
                                )
                            ]
                            parameter_futures[name] = parameter_clients[name].call_async(request)
                            parameter_sent_at[name] = now
                    parameters_ready = len(parameter_futures) == len(empty_arrays)
                    for name, future in parameter_futures.items():
                        if not future.done():
                            parameters_ready = False
                            continue
                        reply = future.result()
                        if (
                            reply is None
                            or not reply.results
                            or not all(r.successful for r in reply.results)
                        ):
                            raise RuntimeError(
                                f"could not set {name}/{empty_arrays[name]} before startup"
                            )
                    if parameters_ready and collision_ready:
                        if manager_client.service_is_ready():
                            if manager_ready_at is None:
                                manager_ready_at = now
                            if now - manager_ready_at < 1.5:
                                continue
                            request = ManageLifecycleNodes.Request()
                            request.command = ManageLifecycleNodes.Request.STARTUP
                            manager_future = manager_client.call_async(request)
                if manager_future is not None and manager_future.done():
                    reply = manager_future.result()
                    if reply is None or not reply.success:
                        raise RuntimeError("Nav2 lifecycle startup failed")
                if lifecycle_future is None and lifecycle.service_is_ready():
                    lifecycle_future = lifecycle.call_async(GetState.Request())
                    lifecycle_sent_at = now
                active = False
                if lifecycle_future is not None and lifecycle_future.done():
                    response = lifecycle_future.result()
                    active = response is not None and response.current_state.id == 3
                    lifecycle_future = None
                elif lifecycle_future is not None and now - lifecycle_sent_at > 1.5:
                    lifecycle_future.cancel()
                    lifecycle_future = None
                if client.server_is_ready() and active:
                    state = "running"
                    endpoint.emit(start_request, "done", "Nav2 NavigateToPose action ready")
                elif now > start_deadline:
                    raise TimeoutError("Nav2 lifecycle startup timeout")
            if refresh_request is not None:
                try:
                    if costmap_refresh.poll(now):
                        endpoint.emit(refresh_request, "done", "代价地图已刷新并等待新观测")
                        refresh_request = None
                except Exception as exc:
                    endpoint.emit(refresh_request, "error", str(exc))
                    refresh_request = None
            if now - last_output > 0.04:
                command = gate.output(now=now)
                dora.send_output(
                    "locomotion_command",
                    command.to_arrow(),
                    {
                        "issued_monotonic_ns": time.monotonic_ns(),
                        "navigation_authorized": gate.enabled,
                        "request_id": "" if current is None else current.request_id,
                    },
                )
                last_output = now
            if now - last_runtime > 0.4:
                endpoint.runtime(
                    message="Nav2 " + state,
                    state=state,
                    active=current is not None,
                    distance_remaining=distance,
                    goal_request_id=None if current is None else current.request_id,
                    velocity_enabled=gate.enabled,
                )
                last_runtime = now
    except Exception as exc:
        endpoint.runtime("error", str(exc))
        raise
    finally:
        gate.set_enabled(False)
        dora.send_output(
            "locomotion_command",
            gate.output(now=time.monotonic()).to_arrow(),
            {"issued_monotonic_ns": time.monotonic_ns(), "navigation_authorized": False},
        )
        request_cancel("navigation adapter shutdown")
        backend.stop()
        ros.destroy_node()
        rclpy.shutdown()
