# FR3 + Gemini RGB + Codex

本 fork 在上游 **7e4d1b7aee1c0d3cfc3a05a7492b9d12cda666f9** 上新增 `fr3` embodiment 和 agent 的 `wire=codex`。继续使用原生 agent 的工具、插值、对话历史、图像窗口、operator feedback、评分及日志。完整 TCP 控制额外为 core/agent 增加了显式的有界起始姿态旋转向量支持；未声明该语义的四元数、绝对旋转表示仍沿用原拒绝规则。

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

启动保持操作者摆好的当前姿态，不回 home，不预先开爪。模型可以通过 `move_to` 的 `rx,ry,rz` 改变朝向。运行沿用上游的终端反馈和结束评分；`Ctrl-C` 退出，清理自身控制会话。两个 baseline 必须先后运行，不能同时占用机械臂和 RGB 相机。

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

动作升级为 `[x, y, z, rx, ry, rz, gripper_width]`。XYZ 是 policy/base 坐标系中的绝对 TCP 位置，单位米；旋转分量单位弧度。旧 4 维动作会报错，不自动解释。

旋转遵循 `R_target = Exp([rx, ry, rz]) @ R_start`：`R_start` 是本轮 reset 后的实测 TCP 朝向；向量方向是 policy/base 坐标中的旋转轴，模长是角度，正方向为右手定则。这不是 Euler 角，也不是逐步累加的旋转。例如 `rx=0, ry=0.2, rz=0` 表示从起始朝向绕基座 y 正向旋转 0.2 rad；三个零恢复起始朝向，**不代表俯视**。姿态合成和 EE→TCP 变换只各应用一次。

`move_to` 允许只给部分目标，其余从测量 `eef_state` 继承。`eef_state` 的最后一维保留上次**命令宽度**，避免持杯平移时误开爪；单独 `gripper_width` 是实测宽度。`tcp_quat` 是绝对 xyzw 四元数；`joint_pos` / `joint_vel` 暴露关节实测值，供查看而非直接下发关节目标。

每个 waypoint 默认位置每轴最多 1 cm、周期 0.3 s，旋转总角度最多 `rotation_step_rad=0.05`（约 2.86°）；三个旋转分量各限为 0.05/√3 rad，并另外检查实际姿态间角度。默认每个旋转目标分量范围 ±`rotation_range_rad=1.5` rad，整个矩形参数域严格位于旋转向量模长 π 内，避免跨越主值分支。

上游工具在这个显式 `axis_angle + rotation_reference=trial_start` 参数域中生成 waypoint，DepthUMI 对相邻姿态进行 SO(3) 插值/IK/速度限制。确认队列完成、实际 TCP 在 5 mm / **0.01 rad** 内、夹爪不忙之后，获取动作后的新帧才返回。`control_hz=3.333...` 是名义速率；到位和图像开销可能使实际更慢。每段原生工具仍受 10 秒名义播放长度限制，较大的转角应拆成多次调用。原生普通 playout 控制器保持不变；线性 ensembling 不支持此旋转表示。

旋转保持的是 TCP 目标位置，腕部、相机和长手指仍会扫过空间；需要先有间隙再倾转。该实现保留 DepthUMI 的 IK/关节限位和故障停止，没有加入全路径碰撞/IK 可行性规划，也没有消除此前真机出现的关节限位问题。

本机没有校准的桌面边界，数值 action box 只是命令范围，不代表无碰撞区域；已有 DepthUMI workspace 配置会优先使用。可通过 `-E z_floor_m=<实测高度>` 收紧下界。默认最多运行 1800 秒（推理超时默认 120 秒），每次将要执行动作时检查超时、TCP 漂移和新相机帧；故障立即退出，网络失联仍受 DepthUMI lease/watchdog 管理。

相机采用已验证的 V4L2/UVC RGB 发现和重绑代码；不安装 Orbbec SDK，不需要深度，不复制仿真外参。只暴露一个真实 `wrist` 视角，缩放并补边到 640×640。RGB 是主机接收时间，不是曝光时间。

## 安装和记录

```bash
PYTHON_BIN=~/miniconda3/envs/depthumi/bin/python bash scripts/fr3/setup.sh ~/projects/DepthUMI
```

独立 `.venv`，只安装 core、agent、FR3；运行时不导入 Show-Harness 或 DepthUMI Python 包。硬件传输代码固定来自 Show-Harness 的 `957b1a707a666226a29d97922c7826d91c504ccb`（Apache-2.0，见 NOTICE），共用的是 DepthUMI 部署协议。`configs/site/fr3.yaml` 是 gitignored 本机标定副本，升级标定后显式运行 `scripts/fr3/import_site.py ... --refresh`。复制不提升原估计变换的物理标定精度。

`logs/` 保存原生 eval JSON、frames、transcripts 和 wire capture。Codex 的精确 prompt/schema/images/命令/输出另存于 `logs/codex/<run>/<trial>/`。硬件回执、相机日志和当次 site 设置在 `logs/fr3/<timestamp-id>/`，通过 observation/step info 关联。`execution.jsonl` 记录 rotation_reference、每次观察的 TCP/q/dq、started/completed 和 fault，未完成动作不会伪记为完成。预检在 `logs/preflight/`。

Codex 使用官方 `codex exec` 和 ChatGPT 登录，保留原生会话历史并把工具调用转为结构化输出；禁用 shell/apps/web/multi-agent，机械臂工具只由 Python 原生 agent 执行。CLI 的取消/超时会终止其子进程组。日志可能包含实验图像和任务文本，未纳入 Git。

## 杯沿抓取经验

`configs/fr3.ini` 默认通过原生 `prior_learnings` 加载
`configs/learnings/fr3_cup_rim.md`。它结合操作者“竖着夹”的要求和 overfit
示范 episode0 的离线图像检查，说明从上方接近杯沿、一指在杯内一指在杯外夹住
同一段杯壁、提起验证、移到盘中并支撑后释放的流程。

现在还通过 Codex 扩展参数 `prior_demo_image` 加载 18 帧历史示范拼图：
`configs/site/fr3_cup_episode0.jpg`。它在策略构造时读取并冻结，每次 Codex 请求都放在
当前观察历史之前，明确标注 HISTORICAL DEMONSTRATION；不占用 `image_horizon=2`
的实时观察窗口。Codex CLI 每次调用没有持久会话，因此必须重复携带同一参考图。
图像来源路径及 SHA256 进入 eval 配置，实际送入模型的 PNG 和提示词进入 Codex/
wire 日志。原生 transcript 含引用说明；确切图片附件以 wire 和 Codex 日志为准。
这是一张图像示范，不是直接播放 HDF5/视频；当前只有 `wire=codex` 支持此参数。

episode0 对齐数据跨度 8.9 秒。1 Hz 适合约十张概览图，但闭爪（约 2.4–2.8 秒）
和开爪（约 5.0–5.4 秒）需要补看 5 Hz 关键帧。示范存在姿态变化，当前接口已能
转腕，但不直接复现源轨迹或源坐标。示范是适度留出间隙后，持续接近杯沿并逐渐调姿，
然后局部闭爪；“向上向前、向下看”不是先垂直抬高再原地转到完全垂直的独立阶段。
应以手指能跨住杯沿为准，而非追求严格俯视。纯基线对照用
`-P prior_learnings=none -P prior_demo_image=none` 同时禁用文字和图片；只禁用图片
则用 `-P prior_demo_image=none`。

经验现已包含 episode0 的六阶段文字示范：接近并调整姿态、杯沿局部对位、闭爪、
搬运、放置开爪、撤离检查。每阶段区分画面证据、实测运动与当前机器人执行建议。
前 2.2 秒的端点姿态变化约 42.1°，全程相对初始姿态最大变化约 55.2°；因此不能
把这条示范理解为从任意初始姿态平移就能完成。数字是历史统计，不是 FR3 目标。

可重复的离线提取命令如下。使用 DepthUMI 环境中的 h5py、NumPy、SciPy、Pillow，
不增加 Inspect Robots 的在线依赖，也不连接相机或机械臂：

```bash
cd ~/projects/inspect-robots
~/miniconda3/envs/depthumi/bin/python scripts/fr3/analyze_cup_demo.py \
  ~/projects/DepthUMI/outputs/policy/real_experiments/2026_09_08-03_00_00-pick_cup-8runs/processing/overfit/episode_000000/vive_tracker.hdf5 \
  --out logs/demo-assessment/episode0/trajectory-review
cp logs/demo-assessment/episode0/trajectory-review/storyboard.jpg \
  configs/site/fr3_cup_episode0.jpg
```

输出 `trajectory.json`、18 张关键帧和 `storyboard.jpg`。统计使用全部 90 个 10 Hz
状态，图像选择采用 1 Hz 概览加 5 Hz 闭爪/开爪事件帧。脚本核对 HDF5 声明的
`xyz+qwqxqyqz`，并用保存的局部 delta actions 交叉检查变换，避免误读四元数。
每阶段位移向量属于该阶段起点的 TCP 坐标，不是机器人基座；标注的阶段边界来自
人工图像复核，不是力/接触测量。原始 HDF5、导出图像及实验日志仍保留在本机。
示范拼图也保存在 gitignored 的 `configs/site/`，不会发布实验图像。其他机器需要
生成/提供该图，或显式禁用 `prior_demo_image`；文件不存在时初始化报错。
