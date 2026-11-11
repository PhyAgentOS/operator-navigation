# 接口

输入：tick、finish、policy_command（PolicyCommand，policy_id=navigation）。命令：start / navigate_to_pose / clear_costmaps / cancel / stop。输出：status（PolicyCommandStatus，保留 request_id）。另需 control、localization_health、base_feedback；均为 单元素 Arrow string 数组中的 JSON。输出 locomotion_command（LocomotionCommand）；默认要求底盘运动反馈。

ROS2 的 LaserScan、Odometry、OccupancyGrid、Path、TF 在后端交换，不新增 Forge 消息类型。具体配置键见示例 YAML；topic、ROS domain、程序路径由调用方指定。

## 最小调用

```python
from forge_msgs import PolicyCommand

request = PolicyCommand.from_inputs(
    policy_id="navigation", command="start", request_id="example-1", inputs={}
)
node.send_output("policy_command", request.to_arrow())
```

每次新任务使用唯一 request_id；重复相同请求返回已有状态，重复 ID 配不同参数被拒绝。status 常见值为 accepted/running/done/rejected/error；done 是此命令完成，不表示后端进程退出。

| command | inputs 示例 | 作用 |
|---|---|---|
| `start` | `{}` | 启动 Nav2 生命周期 |
| `navigate_to_pose` | `{"pose": [1, 2, 0.5]}` | map 坐标目标 x(m)、y(m)、yaw(rad) |
| `clear_costmaps` | `{}` | 仅在空闲且未授权行驶时刷新 |
| `cancel` | `{"target_request_id": "goal-1"}` | 取消匹配目标；省略 target_request_id 取消当前目标 |
| `stop` | `{}` | 取消任务并停止 Nav2 后端 |

`examples/dataflow.yaml` 展示完整输入输出连接。dynamic 节点是调用方占位，不会凭空提供数据；实际应用需用自己的节点替换。`tick` 为调度输入，`finish` 任意负载触发退出。所有进程在同一机器人运行，统一 ROS_DOMAIN_ID。

## 低频健康与控制接口

这些是已有 Arrow string 数组中的 JSON，不新增 forge_msgs 类型。发送方式 `pa.array([json.dumps(value)])`。

- `health` / `localization_health`：至少 `healthy: bool`、`issued_monotonic: float`；附带 reason、pose=[x,y,yaw]、agreement、covariance、mode 等诊断。
- `control`：`{"armed": true, "issued_monotonic": time.monotonic()}`。调用方维持授权心跳；navigation 要求其年龄 <0.35s，health 年龄 <0.65s，否则速度输出归零。与网页无强制绑定，其他编排器也可发此接口。
- `base_feedback`：`{"wheel_speed": {"wheel_left": 0.1, "wheel_right": 0.1}, "issued_monotonic": time.monotonic()}`，轮速单位 rad/s，用于验证有实际运动；require_motion_evidence 默认开启。
- 单调时间只适用于同机进程，不可把此 JSON 直接跨主机转发作为行驶授权。
