# FR3 + Gemini RGB + Codex

本 fork 在上游 **7e4d1b7aee1c0d3cfc3a05a7492b9d12cda666f9** 上新增 `fr3` embodiment 和 agent 的 `wire=codex`。核心框架不变；原生 agent 的工具、插值、对话历史、图像窗口、operator feedback、评分及日志继续使用。

## 本机直接运行

沿用 DepthUMI 原有两个终端：

```bash
# 终端 1
cd ~/projects/DepthUMI
sudo env ROBOT_IP=172.16.0.2 bash deploy/real/scripts/launch_comm.sh
```

```bash
# 终端 2
conda activate depthumi
cd ~/projects/DepthUMI
bash deploy/real/scripts/launch_ik.sh
```

```bash
# 终端 3：先无运动复检，再开始杯子任务
cd ~/projects/inspect-robots
bash scripts/fr3/run.sh --dry-run
bash scripts/fr3/run.sh
```

启动保持操作者摆好的当前姿态，不回 home，不预先开爪。默认固定腕部朝向。运行沿用上游的终端反馈和结束评分；`Ctrl-C` 退出，清理自身控制会话。两个 baseline 必须先后运行，不能同时占用机械臂和 RGB 相机。

本机部署检查时真实 RGB 已通过；DepthUMI 夹爪读数返回 `gripper SDK worker: RuntimeError: Net Exception`，native comm 日志也有网络异常。因此尚未做真机运动。确认机械臂连接/FCI 可用，在原有终端重启 comm 和 IK 后重跑 `--dry-run`。程序不会吞掉连接故障或用模拟状态代替实机状态。

## 保持原生设置

仓库配置是 `configs/fr3.ini`，格式与 `inspect-robots setup` 写入的 config.ini 相同。`-P` / `-E` 仍覆盖配置。薄脚本只选择配置、默认杯子指令和当前 `.venv`：

```bash
# 等价原生命令
.venv/bin/inspect-robots run --config configs/fr3.ini \
  --instruction "Pick up the cup and place it upright in the plate. Release it and lift clear."

# 同一入口切模型，模型必须在你的 Codex 订阅中可用
bash scripts/fr3/run.sh -P model=gpt-6-astra -P effort=low

# 首轮缩短试验；步数是框架执行的 waypoint 数，不是模型调用数
bash scripts/fr3/run.sh --max-steps 60

# 查看本地结果
.venv/bin/inspect-robots view logs
```

默认模型为已经验证的 `gpt-5.5`，本地明确设置 `effort=low` 降低推理等待。其余沿用上游 agent 默认：`images=always`、`image_horizon=2`、`max_llm_calls=100`、`max_speed_frac=0.1`，默认控制器完整播放 chunk，原生 guardrails 开启。300 步是上游 ad-hoc 默认；operator 评分不会把模型的 `done` 当成客观成功。需要人工判断杯子确实直立落入盘子。

其他模型仍走原来的 API wire，例如 `-P wire=responses -P model=openai/...`，以及相应 API key；Codex 订阅路径不自动回退到 API。

## 无运动检查

```bash
bash scripts/fr3/run.sh --check          # 插件、配置、ChatGPT 登录；不连硬件
bash scripts/fr3/run.sh --preview        # 真实 RGB，保存 logs/preflight/preview.png
bash scripts/fr3/run.sh --vision-check   # 真实 RGB + 一次 Codex；完全不连接机器人
bash scripts/fr3/run.sh --dry-run        # 真实 RGB + 只读 TCP/夹爪 + 原生 agent；不执行 step
bash scripts/fr3/run.sh --dry-run --mock # 显式 mock + 实际 Codex，完全不连接真机
```

`--vision-check` 只验证图像和传输，既不执行模型工具，也不声称完成了真机状态检查。它使用原生 agent 工具 schema、prompt 和图像编码。`--dry-run` 则完整调用原生 `policy.act()`，不启动控制、不执行返回动作。

## 硬件接口与时间

动作是 `[x, y, z, gripper_width]`，单位全部为米，在已配置的 policy/base TCP 坐标系。启动时记录四元数，整个试验保持此朝向；没有假造旋转轴或相机标定。原生 `move_to` 允许命名部分目标，其余维度从 `eef_state` 继承。`eef_state` 的 aperture 维保留上次**命令宽度**，避免抓住杯子后下一次平移把手张回实测宽度；单独 `gripper_width` 字段始终记录实测宽度。

每个 waypoint 默认每轴最多 1 cm、周期 0.3 s。Inspect Robots 生成 waypoint，DepthUMI 在每个 waypoint 内进行实时插值/IK/速度限制；客户端不再产生额外动作序列。确认队列完成、实际 TCP 在 5 mm / 0.05 rad 内、夹爪不忙之后，获取动作后的新帧才返回。`control_hz=3.333...` 是名义速率，物理到位和图像开销可能使实际更慢，日志中的真实时间用于实验比较。模型等待期间 RPC 线程独立维持心跳。

本机没有校准的桌面边界，数值 action box 只是命令范围，不代表无碰撞区域；已有 DepthUMI workspace 配置会优先使用。可通过 `-E z_floor_m=<实测高度>` 收紧下界。默认最多运行 1800 秒（推理超时默认 120 秒），每次将要执行动作时检查超时、TCP 漂移和新相机帧；故障立即退出，网络失联仍受 DepthUMI lease/watchdog 管理。

相机采用已验证的 V4L2/UVC RGB 发现和重绑代码；不安装 Orbbec SDK，不需要深度，不复制仿真外参。只暴露一个真实 `wrist` 视角，缩放并补边到 640×640。RGB 是主机接收时间，不是曝光时间。

## 安装和记录

```bash
PYTHON_BIN=~/miniconda3/envs/depthumi/bin/python bash scripts/fr3/setup.sh ~/projects/DepthUMI
```

独立 `.venv`，只安装 core、agent、FR3；运行时不导入 Show-Harness 或 DepthUMI Python 包。硬件传输代码固定来自 Show-Harness 的 `957b1a707a666226a29d97922c7826d91c504ccb`（Apache-2.0，见 NOTICE），共用的是 DepthUMI 部署协议。`configs/site/fr3.yaml` 是 gitignored 本机标定副本，升级标定后显式运行 `scripts/fr3/import_site.py ... --refresh`。复制不提升原估计变换的物理标定精度。

`logs/` 保存原生 eval JSON、frames、transcripts 和 wire capture。Codex 的精确 prompt/schema/images/命令/输出另存于 `logs/codex/<run>/<trial>/`。硬件回执、相机日志和当次 site 设置在 `logs/fr3/<timestamp-id>/`，通过 observation/step info 关联。`execution.jsonl` 分别记录 started/completed，未完成动作不会伪记为完成。预检在 `logs/preflight/`。

Codex 使用官方 `codex exec` 和 ChatGPT 登录，保留原生会话历史并把工具调用转为结构化输出；禁用 shell/apps/web/multi-agent，机械臂工具只由 Python 原生 agent 执行。CLI 的取消/超时会终止其子进程组。日志可能包含实验图像和任务文本，未纳入 Git。
