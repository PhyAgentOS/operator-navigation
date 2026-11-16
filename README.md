# 地图导航

管理 Nav2 生命周期、目标执行、速度门控与到达核验。机器人型号、话题、速度、车身轮廓和行为树由调用方配置。

## 运行要求

- Node 版本：`0.3.8`；兼容 **Forge 1**。
- Linux x86_64、Ubuntu 24.04、Python 3.12 ABI、ROS 2 Jazzy。
- Dora CLI / Python 绑定 `0.4.1`、Forge Msgs `1.3.0`、Forge Common `1.0.1`。
- ROS 2、算法后端、Livox 消息工作空间由部署方提供，不包含在 Node 包内。启动前加载对应 ROS setup，运行 `--check-runtime` 检查环境；该命令不连接硬件。

制品是 `directory_tar_gz`，解压后保留整个 `navigation/` 目录，入口为 `navigation/navigation`。冻结包包含 Dora Python 绑定，不能只升级外部 Dora CLI 来切换到 Forge 2。

## 能力边界与可选配置

- 包内默认 launch 管理 Nav2 后端；可通过 `launch_file` 替换。命令及平滑话题可配置。
- `backend_overrides` 生成参数副本；`snapshot_dir` 可输出路径、代价地图等观测。
- 可选 `map_fusion` 使用连续空闲观测和有限动态记忆生成独立导航障碍图；保留定位先验地图。缺失或被遮挡的回波不是空闲证明；目前要求轴对齐二维地图。
- 包含独立命名的 `navigation_smac_planner`，修复连续坐标、转移采样和最终路径的碰撞判定一致性。只为 planner 子进程设置插件查找路径，不覆盖系统 ROS 库；该插件要求 Nav2 1.3.13 ABI。
- 行驶授权由外部协调器提供，本 Node 验证时效、定位和执行反馈，不负责用户登录、网页按钮或 H1 初始化姿态。
- 源码和可复现 Smac 补丁见 `backends/smac/README.md`；许可证和二进制来源说明见 `THIRD_PARTY_NOTICES.md`。

这些能力通过配置启用，未配置的可选功能保持关闭。Node 仅拥有自己的后端子进程，不接管其他应用进程或写入机器人厂家工程。快照是运行观测，不是控制接口；读取方负责检查时效。

## 开发与构建

```bash
uv sync --group dev
uv run navigation --check-config --config config/policy.example.yaml
uv run pytest
uv run ruff check .
uv pip install --python .venv/bin/python -r requirements-build.txt
source /opt/ros/jazzy/setup.bash
# 按部署环境加载所需消息及算法工作空间。
# 先按 backends/smac/README.md 构建并放置锁定的插件。
BUILD_PYTHON=.venv/bin/python bash scripts/build_pyinstaller.sh
```

配置示例中的路径和话题需由使用者填入；实际执行应由 Dora dataflow 编排。构建时使用与测试相同的依赖环境。`--version`、配置检查和单元测试不替代 ROS 后端及真机验收。

接口说明见 [docs/interfaces.md](docs/interfaces.md)，配置在 `config/`，实现位于 `src/`，测试位于 `tests/`。构建工具、测试及源码不随冻结运行制品提供。
