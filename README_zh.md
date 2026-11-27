[English](README.md) | 简体中文

# 地图导航

## BREAKING CHANGE: Forge 2.x / Dora 1.x migration

升级时须配套部署 Forge 包、Dora CLI/coordinator/daemon 及所有上下游节点，并在匹配环境重新构建可执行制品；不支持在 Forge 1.x / Dora 0.4.x 数据流中仅替换单个节点。

锁定的 Python 基线为 `forge-msgs==2.1.0`、`forge-common==2.1.0`、`Pillow==13.0.0` 和 `dora-rs==1.0.1`。Forge 各包族独立版本化，`forge_msgs.v1` 消息命名空间保持不变。

`2.0.0` 将 Node 主版本号与 Forge 兼容线对齐。此前已发布的 `0.x` Forge 2 制品保持不可变；本次版本对齐不改变其运行算法或消息接口。此处的部署破坏性变更指从 Forge 1 / Dora 0.4 迁移，不指此前的 Forge 2 制品。回退须保留上一套匹配的制品和锁文件。本仓库目前未提供 `stable/1.x` 维护分支或 Forge 1 发布工作流。

源码检查、二进制构建和仿真验证是不同验收阶段。真机、长时间运行及回退验收须在目标部署环境完成，版本声明本身不代表这些验收通过。

管理 Nav2 生命周期、目标执行、速度门控与到达核验。机器人型号、话题、速度、车身轮廓和行为树由调用方配置。

## Node、Endpoint 与 Tool API

| 项目 | 当前实现 |
| --- | --- |
| 生产 Node | 1：`navigation`；源码入口 `navigation.main:main`；制品入口 `navigation/navigation` |
| 辅助程序 | `--version`、`--check-config`、`--check-runtime` 是离线 CLI 模式，不另计 Node |
| Tool 支持 | 仅数据面，不实现 ToolEndpoint provider 或 caller |
| Endpoint / operation | 0 / 0；Dora policy 消息不属于 ToolEndpoint operation |
| 直接 HTTP Tool API | 0；外部策略与 Gateway 负责面向调用方的 Tool |
| Gateway Tools | 本仓库不固定 Tool ID；绑定由消费方 Skill 配置 |

消息输入输出、单位和生命周期见[接口契约](docs/interfaces.md)。源码：[入口](src/navigation/main.py)、[处理器](src/navigation/node.py)。部署示例：[dataflow](examples/dataflow.yaml)、[配置](config/)。运行与发布章节说明依赖锁、制品结构及源码溯源。

## 运行要求

- Node 版本：`2.0.0`；兼容 **Forge 2**。

扫描采集时效上限为 400 ms，允许时间戳量化产生的 1 ns 误差。延迟到达的
坐标变换不能续期旧观测；未来时间戳、墙钟租约过期和定位异常仍会停止运动。
- Linux x86_64、Ubuntu 24.04、Python 3.12 ABI、ROS 2 Jazzy。
- Dora CLI / Python 绑定 `1.0.1`、Forge Msgs / Common `2.1.0`（精确构建依赖见 uv.lock）。
- ROS 2、算法后端、Livox 消息工作空间由部署方提供，不包含在 Node 包内。启动前加载对应 ROS setup，运行 `--check-runtime` 检查环境；该命令不连接硬件。

制品是 `directory_tar_gz`，解压后保留整个 `navigation/` 目录，入口为 `navigation/navigation`。冻结包包含 Dora Python 绑定，须与 Dora 1.x 及 Forge 2 对等节点一起使用；旧 Forge 1 制品保持原版本，不原地替换。

## 能力边界与可选配置

- `use_sim_time: true` 显式使用仿真时间；默认仍使用宿主 ROS 时间。快照以同目录独占临时文件原子更新。

- 包内默认 launch 管理 Nav2 后端；可通过 `launch_file` 替换。命令及平滑话题可配置。
- `backend_overrides` 生成参数副本；`snapshot_dir` 可输出路径、代价地图等观测。
- 可选 `map_fusion` 使用连续空闲观测和有限动态记忆生成独立导航障碍图；保留定位先验地图。缺失或被遮挡的回波不是空闲证明；目前要求轴对齐二维地图。
- 包含独立命名的 `navigation_smac_planner`，修复连续坐标、转移采样和最终路径的碰撞判定一致性。只为 planner 子进程设置插件查找路径，不覆盖系统 ROS 库；该插件使用固定的 Nav2 1.3.13 源码，运行时 ABI 必须匹配其构建元数据。
- 同一个 Node 包含 SegmentGoal/SegmentGoalAngle MPPI 插件，仅为控制器子进程设置加载路径；打包前运行原生回归与插件发现测试。
- 行驶授权由外部协调器提供，本 Node 验证时效、定位和执行反馈，不负责用户登录、网页按钮或 H1 初始化姿态。
- 源码和可复现 Smac 补丁见 `backends/smac/README.md`；许可证和二进制来源说明见 `THIRD_PARTY_NOTICES.md`。

这些能力通过配置启用，未配置的可选功能保持关闭。Node 仅拥有自己的后端子进程，不接管其他应用进程或写入机器人厂家工程。快照是运行观测，不是控制接口；读取方负责检查时效。

## 开发与构建

```bash
uv sync --frozen --group dev --group build
uv run navigation --check-config --config config/policy.example.yaml
uv run --frozen pytest
uv run --frozen ruff check .
source /opt/ros/jazzy/setup.bash
# 按部署环境加载所需消息及算法工作空间。
# 先构建 backends/smac 和 backends/mppi 中锁定的规划/控制插件。
BUILD_PYTHON=.venv/bin/python bash scripts/build_pyinstaller.sh
```

配置示例中的路径和话题需由使用者填入；实际执行应由 Dora dataflow 编排。构建时使用与测试相同的依赖环境。`--version`、配置检查和单元测试不替代 ROS 后端及真机验收。

接口说明见 [docs/interfaces.md](docs/interfaces.md)，配置在 `config/`，实现位于 `src/`，测试位于 `tests/`。构建工具、测试及源码不随冻结运行制品提供。

## 版本化发布

在个人 fork 修改并通过 PR 合入 main。创建与 pyproject.toml 一致的新 annotated `v2.X.Y` tag，推送后手动运行 `Build versioned release`，指定 `release_line=2.x` 和该 tag。该工作流仅接受稳定的 `v2.X.Y` tag，拒绝其他主版本及预发布 tag。工作流检查源码、Forge 2/Dora 版本、依赖锁、测试和 lint，在 Ubuntu 24.04 / ROS 2 Jazzy 构建并检查真实冻结入口，再生成带 SHA256 与源码元数据的 Draft Release。发布者核查后发布，再单独登记资源平台。

制品格式为 `<entrypoint>-v<version>-ubuntu24.04-x86_64-ros2-jazzy.tar.gz`。已存在 Release 或资产不覆盖，改变字节必须使用新版本。外部 ROS 与算法后端不随包提供；构建期 Livox 仅编译经过摘要核对的官方消息定义，不运行驱动。CI 与仿真验证不代表真机验收。

Nav2 构建兼容范围与运行时精确 ABI 校验见 [backends/ABI.md](backends/ABI.md)。

路径、代价地图、融合地图和停止诊断快照共用一个有界后台写入线程。内容与观测时间在入队前固定；写盘延迟或失败不会刷新旧记录的时效，也不会阻塞控制消息处理。速度输出前重新检查授权和速度租约。
