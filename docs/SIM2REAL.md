# H1 Sim2Real

面向标准 19 关节 H1 的 GO 协议，不适用于 H1-2。Python 和 C++ 路径共用部署核心。当前 `last` 的 C++ 里程计接入命令、参数与验证边界见本文末尾“C++ 里程计接入、高度补偿与动作对齐”；早期外部定位命令仍保留。

## H1 接口

策略顺序为左腿 5、右腿 5、torso yaw 1、左臂 4、右臂 4，硬件通道映射为：

```text
7, 3, 4, 5, 10, 8, 0, 1, 2, 11, 6, 16, 17, 18, 19, 12, 13, 14, 15
```

空槽 9 不启用；强电机 mode 为 0x0A，弱电机 10–19 使用 0x01。GO header 为 FE EF，level_flag 为 FF。收发均应用映射，不使用旧消息的机型模式字段。

H1 torso IMU 通过 torso_joint 的 q/dq 转为 pelvis 姿态和角速度。GO 单个温度字段在兼容日志中重复为两列；motorstate 日志列保存 GO lost 计数，不代表故障位掩码。`--max-motor-lost` 可显式设置计数阈值。

适配参考本机 Unitree 官方工程 H1 配置和 SDK GO IDL，尚未在实际硬件上核验。

## Python 路径

单独安装 SDK2 Python 及其 Cyclone DDS 依赖，例如 `pip install -e /path/to/unitree_sdk2_python`。只读查看：

```bash
python scripts/deploy/check_robot.py --iface enp12s0 --domain 0 --once --check-mode
```

使用当前 anchor 策略，需要先接入[位姿流](ANCHOR_POSE.md)：

```bash
bash scripts/run_sim2real.sh --iface enp12s0 --domain 0 --stage air \
  --clip deploy_artifacts/clips/walk1_subject1.npz \
  --pose-listen 127.0.0.1:15101
```

网卡按实际修改。`air/feet/slack/ground` 沿用原分阶段参数，未重新标定为 H1 实测阈值。保留启动前控制器冲突检查、现场确认、遥控器急停和命令循环看门狗，不主动释放官方运动服务。

## C++ bridge 路径

```bash
bash scripts/deploy/bridge/build.sh /path/to/unitree_sdk2
bash scripts/run_sim2real_bridge.sh --iface enp12s0 --domain 0 --stage air \
  --clip deploy_artifacts/clips/walk1_subject1.npz \
  --pose-listen 127.0.0.1:15101
```

C++ SDK 头文件是 `unitree/idl/go2/LowCmd_.hpp` 和 `LowState_.hpp`，命名空间为 unitree_go。bridge 处理 DDS、CRC、共享内存快照、消息环、定频发送及独立 watchdog。H1 共享内存协议改为 20 槽位及新 magic/version；必须配套使用本仓库编译的 bridge 和 client 库。

保留主循环失联时的阻尼、orphan 超时、温度趋势及热停止参考。热停止是参考轨迹切换，不是已验证的 H1 平衡保护策略。

## 操作

| 操作 | 行为 |
|---|---|
| 启动 | PASSIVE 阻尼 |
| L2+up | 插值到起始姿态，再进入 HOLD |
| R1+A | 通过起始状态检查后进入 STANDBY_PRE |
| R1+X | 通过待机稳定性检查后开始 POLICY |
| 动作结束 | STANDBY_POST，或选择 END_HOLD |
| L2+B / 保护触发 | DAMP |
| 多动作 | `--clip` 接收多个文件，按原状态机切换 |

日志写入 `deploy_logs/`；`--log-obs` 保存观测历史。anchor 策略额外记录 root_pos 和 anchor_quat，非策略状态用 NaN 标记。

没有完整位姿反馈时，当前策略不能直接上机。使用者还需完成实际硬件调试、标定和保护流程。本页命令未执行。

## 三动作选择与 Start 播放（`--motion-select`）

Python 和 bridge 两个部署入口都支持此模式。启动时加载一份策略和三个动作参考，切换的是参考 NPZ，不是网络权重。未加 `--motion-select` 时仍使用上表的原操作方式。

| 按键 | 行为 |
|---|---|
| L1+A | 选择第一个自定义动作文件（旧组合为 `default.npz`），进入准备流程 |
| L1+X | 选择第二个自定义动作文件（旧组合为 `walk.npz`），进入准备流程 |
| L1+Y | 选择第三个自定义动作文件（旧组合为 `dance.npz`），进入准备流程 |
| R1+B | HOLD 中请求起始姿态检查；通过后进入 STANDBY_PRE，开始首帧策略待机 |
| Start（单独按） | 仅在 STANDBY_PRE 且原稳定性检查通过后，从第 0 帧开始播放 |
| L2+B | 沿用原 DAMP 阻尼，进入后不能用选择键恢复 |

启动后保持 PASSIVE，必须先选择一个动作。第一次选择的流程为：

```text
选择动作 → INTERP（固定 PD 插值）→ HOLD（固定 PD 保持）
         → 按 R1+B，起始姿态检查通过 → STANDBY_PRE（策略控制起始静态参考）
         → 稳定性检查通过，打印 READY → 按 Start → POLICY（播放）
         → 动作结束 → STANDBY_POST（策略控制末帧静态参考）
```

已经处于策略待机时选择其他动作，沿用原仓库的固定 PD 过渡：

```text
STANDBY_PRE / STANDBY_POST → END_HOLD（目标和增益渐变）
                         → INTERP（插值到所选动作首帧）→ HOLD
                         → 按 R1+B，接管检查通过 → STANDBY_PRE → READY → 按 Start
```

选择键触发固定 PD 准备，插值结束后停在 HOLD，必须另按 **R1+B** 才进行起始姿态检查。检查失败会打印 REFUSED，继续固定 PD；松开后重新按 R1+B 重试，不会等条件恢复后自动接管。检查通过才进入 STANDBY_PRE；该阶段已经在运行策略，只是参考固定在首帧，随后 READY → Start 的流程不变。HOLD 中等待按键不代表关节误差已达标，常规温度、倾斜和通信保护仍然有效。

R1+B 必须仅由这两个键组成；若与其他按键重叠，需将 R1、B 都松开后再按。L2+B 仍优先进入 DAMP。提前按 R1+B 不排队，进入 HOLD 后需重新按；每次重新选动作，包括播放结束返回 default、重播同一动作，都要再次经过这一步。

- 播放中选择其他动作：保存目标，等当前动作自然结束后自动进入上述 PD 准备；不会中途切断。期间再次选择其他动作会更新待切换目标。
- 返回 L1+A 对应动作：按 L1+A；流程相同。旧文件名组合对应 `default`，自定义文件名组合对应 `--clip` 的第一个文件。
- 准备过程（到首次 READY 为止）中忽略新的选择，避免准备目标被改动。
- 提前按 Start 会被拒绝或忽略，不会排队；出现 READY 后松开并重新按 Start。选择键和 Start 同时触发时只处理选择。
- 播放中重复按 Start 不会重播。已准备好或正在播放时重复选择当前动作不重启；动作结束后可重新选择同一动作，再按 R1+B、等 READY、按 Start 重播。
- 动作结束且没有待切换目标时停留在 STANDBY_POST；Start 本身不触发重新准备。bridge 的热停止流程仍禁止下一动作。

**固定 PD 阶段仍需要吊装/外部支撑承重。** 选择键会触发这段过渡；若播放中预选了下一动作，当前动作结束后就会开始过渡。实机操作必须为这个时刻准备好支撑。此次改动没有加入持续策略平衡的动作衔接。

### 启动命令

在已有 SDK/DDS 和参考坐标系下的位姿流可用、外部支撑已就绪的前提下，Python 部署入口为：

```bash
conda activate supermimic
cd /home/ljp/human_learn/supermimic/H1_SuperMimic_Deploy

bash scripts/run_sim2real.sh \
  --iface enp12s0 \
  --domain 0 \
  --stage air \
  --artifacts deploy_artifacts/ckpt_latest \
  --motion-select \
  --clip deploy_artifacts/use/default.npz \
         deploy_artifacts/use/walk.npz \
         deploy_artifacts/use/dance.npz \
  --pose-listen 127.0.0.1:15101
```

`--iface` 替换为机器人连接网卡；`--stage air` 沿用原吊装调试档位，不代表动作已通过落地验证。`--pose-listen` 必须与外部位姿发送端一致，当前 anchor 策略需要它。使用已编译好的 bridge 时，将脚本名换为 `scripts/run_sim2real_bridge.sh`，其余本段参数相同。

DDS 仿真联调使用相同控制器参数，把 `--iface enp12s0 --domain 0` 换为 `--iface lo --domain 1 --sim`；仿真器也需使用 domain 1、相同的位姿端口，并通过原有 `--buttons-file` 注入 `L1+A`、`L1+X`、`L1+Y`、`R1+B`、`start` 等按键。文件中用 `+` 分隔组合键，清空文件表示松开；按键和松开需满足原去抖时长。若使用仿真弹性吊带，控制器的 `--sim-band-file` 与仿真器的 `--band-file` 必须指向同一文件。

仿真吊带在固定 PD 阶段保持启用，直到 STANDBY_PRE 首次有效推理且控制指令提交成功后才释放；动作结束进入 STANDBY_POST、进入 END_HOLD 或 DAMP 时恢复支撑，退出控制器也保留支撑文件。Python 和 bridge 入口一致，详见 [SIM2SIM.md 第 6.4 节](SIM2SIM.md)。这些操作只改变仿真标志文件，实机吊装仍由人工完成；首帧误差 `--hold-tol 0.1` 等接管门槛没有放宽。

`--clip` 必须提供三个不同文件。自定义文件名按命令行顺序绑定 **L1+A、L1+X、L1+Y**；例如 `default.npz walk2.npz dance2.npz` 分别选择 default、walk2、dance2，无需重命名。为兼容旧命令，若文件名恰好是 `default.npz`、`walk.npz`、`dance.npz` 各一次，仍按这三个名称绑定，排列顺序任意。启动横幅会显示实际按键到文件名的对应关系。本模式不能与 `--auto-sequence`、`--end-hold-legacy` 同时使用，因为它需要人工 Start 和策略末帧待机。

### 修改位置与原参数

| 内容 | 位置/参数 |
|---|---|
| 选择键绑定 | `src/h1_deploy/deploy/motion_selection.py` 的 `MOTION_BUTTONS` |
| 三个动作文件的映射 | 同文件的 `MotionSelection.from_paths()`；自定义文件按 `--clip` 顺序绑定，保留旧名称组合映射 |
| R1+B、Start、DAMP 绑定 | 同文件的 `selection_combos()` |
| HOLD 等待 R1+B、排队、忙状态拒绝、READY 提示 | 同文件的 `MotionSelectionMixin.motion_events()` |
| 接管组合键排他识别、松开后重新使能 | `scripts/deploy/h1_common.py` 的 `ComboDebounce`；仅三动作模式的接管键启用 |
| 选择后的固定 PD 过渡 | 同文件的 `begin_selected_motion()`，及两个 Runner 的 `run()` |
| 动作文件路径、策略路径 | 启动命令中的 `--clip`、`--artifacts` |
| 固定 PD 插值时长 | `--interp-s`，原默认 4 秒 |
| HOLD 接入策略的目标渐变时长 | `--policy-blend-s`，默认 5 秒，在 STANDBY_PRE 内执行；与 `--interp-s` 不同 |
| 退出策略时的 PD 混合时长 | `--end-blend-s`，原默认 1 秒 |
| 固定 PD 的 KD | 默认复制当前 `--artifacts/deploy_config.json` 的 `kd`；仅显式传入 `--stand-arm-kd` / `--stand-waist-kd` 时覆盖对应关节 |
| HOLD 后接管条件 | `--hold-min-s`（1 秒）、`--hold-tol`（0.1 rad）、`--start-max-dq`（0.5 rad/s）、`--start-max-tilt-deg`（10°） |
| Start 前待机条件 | `--standby-min-s`（3 秒）、`--standby-window-s`（3 秒）、`--standby-max-tilt-deg`（8°）、`--standby-max-dq`（1 rad/s） |
| bridge 额外温升预测 | 原有 `--temp-rate-min-s` 等参数；因此满足 3 秒待机不一定立即 READY |

以上门槛和固定 PD 的 KP 沿用原实现。固定 PD 的 KD 现在默认跟随当前部署配置，两个 Runner 各自的 `policy_allowed()` / `standby_allowed()` 仍负责检查。新动作接管时重建 DeployCore 并清空历史；待机转播放时仍使用原来的 `switch_clip()` 保持策略历史连续。渐变结束前不计入播放准备窗口，详见下一节。

### R1+B 后的目标渐变接管（2026-11-14）

**实现了目标渐变，但当前 last 的无吊架支撑仿真尚未通过，不能据此直接重试实机。** 此功能处理首次目标跳变，不保证冻结动作首帧时的策略平衡。

操作仍为：选择动作 → 固定 PD 插值 → HOLD → R1+B 起始检查 → STANDBY_PRE 内渐变 → 完全策略待机、检查 → READY → Start。实机可以先吊起完成固定 PD 姿态，再放下双脚承重，之后按 R1+B；接管检查使用的是放下之后的实际状态，不能沿用悬空时的误差。R1+B 不控制实机吊架。

新增共享实现 [policy_takeover.py](../src/h1_deploy/deploy/policy_takeover.py)，两个 Runner 的 `ev == 'policy'` 分支记录上一条 HOLD 目标。每个策略周期执行：

```text
alpha = clip(进入 STANDBY_PRE 后经过的时间 / policy_blend_s, 0, 1)
发送目标 = (1-alpha) × 上一条 HOLD 目标 + alpha × 当前策略目标
```

策略每周期读取反馈并正常推理，参考保持首帧；历史、last_action、KP/KD、定位对齐和模型初始化顺序均保持原定义。没有加入 RoboJuDo 的全程动作滤波或全程目标速率限制。混合后重新应用原力矩/位置裁剪，防止旧 HOLD 目标在新的实测状态下超出原力矩边界；因此若安全裁剪介入，实际首目标不保证与旧 HOLD 目标完全相同。所有原速度、动作幅值、倾斜、温度、定位和通信保护仍有效。

`alpha=1` 的目标成功提交后打印 `BLEND: complete`，从此重新累计稳定性窗口；bridge 同时从此累计温升历史。默认 5 秒渐变后，Python 入口还至少等待原 3 秒待机，bridge 还需要原至少 10 秒温升历史；均须实际检查通过，时间到了不保证 READY。Start 提前按下仍丢弃。每次重新准备动作都重新渐变；POLICY 和 STANDBY_POST 不重复执行渐变。

启动命令可以显式增加：

```bash
--policy-blend-s 5
```

默认值在两个 Runner 的 `main()` 中对应 `add_argument`，不在 `deploy_config.json`。它只设置本次接管时长；`--interp-s 4` 仍是固定 PD 姿态插值，`--end-blend-s 1` 仍是退出策略的过渡。`--policy-blend-s 0` 保留原直接接管行为，供仿真基线对照；负数、NaN、无穷值拒绝启动。5 秒来自 RoboJuDo 接管方式的对照值，**不是本策略已经验证可用的实机推荐值**。

日志新增 `policy_q_target`（原策略输出经原裁剪后的目标）、`policy_blend_alpha`（渐变比例，非渐变阶段为 NaN）和 `blend_n_clamped`（混合后额外裁剪数）。原 `q_target` 现在记录混合后的候选目标，`cmd_q` 仍是实际提交命令记录；发生 abort 的候选目标不会发送。

离线验证分两类：

- 两个真实 Runner + DeployCore，在虚拟时钟、理想反馈和替代策略下通过：首命令连续、渐变中拒绝 Start、提交失败不宣布完成、渐变后重新采样、walk/dance/default 及重播重新渐变、速度超限和 L2+B 仍进入 DAMP。200 组变化状态检查确认额外力矩裁剪有效。
- 真正的 last ONNX + MuJoCo 闭环、原 air 保护、无吊架支撑：参考首帧落地初始化的三个动作均未通过；不能把软件逻辑通过理解为策略稳定。

另用 `deploy_logs/last_bridge/run_20261114_204443.npz` 接管时的关节角/速度、骨盆位置/姿态和角速度初始化 walk。没有人为升降吊架，也没有修改保护；原记录未包含根线速度和完整接触力，因此此仿真不等同于实机精确重现。结果如下，时间从开始接管计：

| 渐变时长 | 首次最大目标跳变 | 首个保护触发 |
|---|---:|---|
| 0 秒，原直接接管 | 0.398857 rad | 0.02 秒，左膝速度约 6.94 rad/s |
| 0.1 秒 | 0 rad | 0.98 秒，右膝速度约 4.65 rad/s |
| 0.2 秒 | 0 rad | 0.84 秒，右膝速度约 5.52 rad/s |
| 0.5 秒 | 0 rad | 0.66 秒，右膝速度约 4.60 rad/s |
| 1 秒 | 0 rad | 0.68 秒，右膝速度约 6.59 rad/s |
| 2 秒 | 0 rad | 0.72 秒，右膝速度约 4.27 rad/s |
| 5 秒 | 0 rad | 0.94 秒，倾斜误差超过 0.35 rad |

结论：首次指令跳变被消除，但单靠渐变还没有解决待机稳定性，需要继续区分静态参考适用性、策略输入初始化及仿真实机差异。此次未通过放宽门槛掩盖结果；未执行实机控制，临时测试脚本验证后删除。

### 固定 PD 阻尼默认跟随策略配置

Python 和 C++ bridge 两个 Runner 的 `INTERP/HOLD` 均使用 `deploy_config.json` 的 `kd` 副本；`END_HOLD` 仍按原逻辑渐变到该固定 PD 增益。当前 `last` 每条腿 KD 为 `[2, 2, 2, 4, 2]`，腰部为 `3`，两臂均为 `2`。原来的固定 PD 腰部 `5`、手臂 `10` 不再作为默认值。

一般不必再写 `--stand-arm-kd 2 --stand-waist-kd 3`。显式传入这两个参数仍会优先覆盖固定 PD 对应关节的 KD，不修改策略配置；要让参数随配置文件更新，应从命令中省略覆盖项。固定 PD 的 KP、PASSIVE/DAMP 的 `KP=0、KD=3`、插值时间、状态顺序和接管门槛不变。

固定 PD 验证仍保留 `--no-policy`；修改默认 KD 不会使这类命令自动进入策略控制。本次实机反馈“无异响”只对应用户完成的固定 PD 测试，不代表已完成策略播放验证。

**坐标条件：** 默认 `--pose-source reference` 沿用外部已对齐位姿，不自动重设坐标。C++ Runner 现在可显式选择 `--pose-source unitree-odom`，在每次动作准备时重新对齐水平位置和朝向，详见本文末尾。`last` 虽然设置了 `anchor_obs=false`，仍通过 9 维骨盆相对位姿观测依赖定位。文件能加载和状态机能切换，不等于三个动作已经通过实机验证。

### 无硬件回归验证

此次 R1+B 接管改动使用临时离线测试，检查组合键去抖、两个 Runner 的 HOLD 等待、提前/失败请求不排队、手动接管及 READY→Start、重新选择动作，以及 L2+B 优先。测试使用虚拟时钟、合成状态和替代策略输出，不连接 DDS、不发送实机指令；临时脚本验证后删除，不验证实机平衡能力。

## 骨盆定位只读校准与漂移检查

入口：[scripts/deploy/check_pelvis_pose.py](../scripts/deploy/check_pelvis_pose.py)。计算逻辑：[src/h1_deploy/deploy/pose_diagnostics.py](../src/h1_deploy/deploy/pose_diagnostics.py)。

这是独立的 Python DDS 状态检查工具，不启动 C++ bridge、Runner 或策略，不创建电机命令/位姿发布者，不调用模式切换服务。现有 C++ 实机控制链路、按键和状态顺序均不受它影响。它仅读取 `last/deploy_config.json` 中的模型配置，不加载网络权重。

### 实机命令

保持已确认的支撑与姿态，不需要按手柄键。下面的 `0.96` 表示**地面到模型 pelvis 原点（腰部旋转关节原点）**的实测高度，单位米；应按每次测量时的实际姿态填写，不能使用骨盆外壳下缘的高度。

```bash
conda activate supermimic
cd /home/ljp/human_learn/supermimic/H1_SuperMimic_Deploy
export CYCLONEDDS_HOME=/home/ljp/human_learn/Camera2Real/RoboJuDo/.runtime/cyclonedds
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

python -B scripts/deploy/check_pelvis_pose.py \
  --iface enp12s0 \
  --domain 0 \
  --artifacts deploy_artifacts/last \
  --seconds 20 \
  --pelvis-height 0.96 \
  --contact both \
  --stationary
```

`--stationary` 是操作者确认机器人整体静止，包括没有被吊架平移或转向；`--contact both` 是确认双脚接触同一平地。它们不会向机器人发送指令。程序不能仅凭关节角不变或 GO 的零值足力字段确认这些条件。若只是查看数据、无法确认静止，可省略 `--stationary`；此时仅报告观测变化，不提供可用校准候选。

默认只打印终端结果，不写报告、NPZ 或 DDS 跟踪日志。需要保留诊断时，在命令末尾增加 `--report outputs/pose_check/h1_static_001.json`。已存在的文件不会覆盖，请换一个名称。JSON 中始终有 `deployment_ready: false`；**它不是部署配置，两个 Runner 都不会读取它。**

### 参数与修改位置

参数均可在命令行修改；默认值在 `check_pelvis_pose.py` 的 `main()` 中。此工具没有需要写入 `deploy_config.json` 的标定参数。

| 参数 | 默认值/含义 |
|---|---|
| `--source` | `dds`；`mujoco` 用于不联网的几何仿真与已知误差注入验证 |
| `--iface`、`--domain` | DDS 模式必须填写网卡；域默认 `0` |
| `--odom-topic` | `rt/odommodestate`，消息类型必须是 GO `SportModeState` |
| `--artifacts`、`--model` | 默认本仓库 `deploy_artifacts/last` 和 `h1_data/h1_training.xml` |
| `--seconds` | `20` 秒；至少 3 秒，且比校准窗口长 1.1 秒以上 |
| `--pelvis-height` | 可选实测高度，米；填写时优先用它拟合候选偏移 |
| `--contact` | `unknown`；可选 `left`、`right`、`both`，用于接地几何核对 |
| `--stationary` | 默认不声明；确认整体静止后才填写 |
| `--calibration-s` | `1` 秒；仅用最初窗口拟合偏移，后续窗口检查固定偏移的结果 |
| `--connect-timeout` | `5` 秒；等待两路有效状态的时限 |
| `--max-pair-ms` | `5` 毫秒；lowstate 与里程计 DDS 源时间戳的最大配对差 |
| `--max-gap` | `0.2` 秒；用于候选校准的源时间/本机接收最大间隔 |
| `--max-joint-span` | `0.02` rad；超过此关节角变化范围，不生成静止校准候选 |
| `--max-contact-gap` | `0.02` m；两脚接地推算或实测与接地几何之间允许的最大高度差 |
| `--status-s` | `3` 秒；采样进度输出间隔 |
| `--report` | 可选新 JSON 文件路径；未填写则不生成文件 |
| `--sim-z-bias` | 仅几何仿真使用，默认 `-0.08` m，表示模拟里程计比真值低 8 cm |
| `--sim-yaw-drift` | 仅几何仿真使用，默认 `0.025` 度/秒 |
| `--sim-xy-drift VX VY` | 仅几何仿真使用，默认 `0.00007 -0.00002` m/s |

以上质量门槛只用于诊断工具，不是实机控制保护阈值，也不修改 `hold-tol`、定位超时或看门狗。

### 怎样理解输出

- `里程计平均高度`：原始 `position[2]`。
- `脚底接地所需骨盆高度`：当前关节角和骨盆姿态下，由左右脚碰撞网格最低点分别推算的高度。这里假设所声明的脚接触平地，并使用当前模型几何。
- `脚踝参考平面高度`：骨盆原点到左右脚踝原点的垂直距离，不包含脚踝以下的脚底几何。
- `实测高度拟合偏移`：实测高度减去最初窗口的里程计高度中位数。
- `接地几何拟合偏移`：由声明的接地脚推算的高度，减去最初窗口的里程计高度；双脚采用两侧的平均拟合值。
- `候选世界 z 偏移`：假设问题来自世界高度零点时的一次性固定偏移；不会逐帧重算来抹掉误差。若实测高度与接地几何明显矛盾、未声明静止、反馈中断、发布者变化、源时间回退或配对覆盖不足，会标记候选不可用。
- `后续窗口脚底 z 均值`：固定候选偏移后，剩余采样窗口中模型脚底相对地面的高度残差。正值在地面以上，负值在地面以下；未声明接地的脚不应要求为零。这是模型一致性检查，不能单独证明真实定位精度。
- `静止条件下报告的漂移`：水平首尾窗口位移、里程计及骨盆 IMU 航向变化。未声明静止时只称为观测变化。固定 z 偏移不会改变这些数值。

工具检查 lowstate CRC、DDS 发布者和递增的源时间戳，并用源时间戳对齐两路数据。两路发布时钟必须相容；配对失败不会改用接收时间凑数。机器人与电脑的绝对时钟不一致不直接被当作网络延迟；本机接收进度也不等于估计器内部测量一定新鲜。发布者重启或时钟回退后，应重新采样。

### 不联网的仿真验证

```bash
python -B scripts/deploy/check_pelvis_pose.py \
  --source mujoco \
  --seconds 20 \
  --sim-z-bias -0.08 \
  --sim-yaw-drift 0.025 \
  --sim-xy-drift 0.00007 -0.00002
```

该模式使用 H1 MuJoCo 模型正运动学建立已知骨盆高度，再注入指定位置/航向误差，检查工具能否恢复高度偏移与漂移率；无需填写 `--pelvis-height`。它不连接 DDS，也不运行策略或动力学闭环，不能代替 G2/G3 或实机动作验证。

**诊断边界：** 单次静止采样无法最终区分“世界高度零点不同”和“机器人测量原点不同”。本工具没有识别 `robot_center → pelvis` 的完整外参、消除航向/水平漂移或发布位姿。下述新接入使用显式指定的世界 z 偏移假设；这不将静态校准结果变成所有姿态通用的安装外参。

## C++ 里程计接入、高度补偿与动作对齐

### 流程和适用范围

这是 **C++ Runner 的可选输入模式**，通过 `--pose-source unitree-odom` 开启。默认 `reference` 模式、原 Python Runner、DDS 仿真的原始位姿接口保持原行为。`last.pt`、`deploy_config.json`、三个动作 NPZ、策略 KP/KD、按键和保护门槛未改动；固定 PD 默认 KD 的后续调整见上节。

```text
实机 rt/lowstate ── CRC、腰关节角、电机 tick ──┐
                                            ├─ h1_odom_bridge（只读 C++）
实机 rt/odommodestate ── 位置、姿态、DDS元数据 ─┘
       → 两路源时间配对 → 本机 UDP
       → 胸部姿态转换为骨盆姿态（显式选择 torso 时）
       → 加固定世界 z 偏移
       → 每次 STANDBY_PRE 首次读取时，确定一次 XY/yaw 对齐
       → DeployCore → last 策略推理 → 原电机通信 bridge
```

电机控制状态顺序仍是 `PASSIVE → INTERP → HOLD → STANDBY_PRE → POLICY → STANDBY_POST`；再次选择动作仍走原来的 `END_HOLD → INTERP → HOLD` 固定 PD 过渡。位姿仍只在原策略控制分支读取；没有把读取或模型初始化提前到固定 PD 阶段。

### 1. 编译定位接收程序

本机已经编译，可直接使用。修改 C++ 源码或换电脑后重新编译：

```bash
cd /home/ljp/human_learn/supermimic/H1_SuperMimic_Deploy
bash scripts/deploy/bridge/build_pose.sh \
  /home/ljp/human_learn/whole_body/OTHER/RobotBridge/unitree_sdk2
```

仅生成 `scripts/deploy/bridge/h1_odom_bridge`。原 `h1_bridge`、共享内存 ABI 和 `libh1bridge_client.so` 不需要重新编译。

### 2. 终端 A：持续接收里程计

```bash
cd /home/ljp/human_learn/supermimic/H1_SuperMimic_Deploy
./scripts/deploy/bridge/h1_odom_bridge \
  --iface enp12s0 \
  --domain 0 \
  --udp 127.0.0.1:15101
```

终端 A 保持运行。它不发布电机指令，不调用 `ReleaseMode`；周期性显示 `paired`、原始 `z`、腰角、配对时间差、数据年龄和 CRC 丢弃数量。若只想验证输入，可以增加 `--check-only --seconds 20`：不发送 UDP，20 秒后退出。这个只读检查模式不能给终端 B 提供位姿。

### 3. 终端 B：使用定位输入启动原控制器

下面保留你原来的 `--stage air` 和三个动作路径。启动参数中显式采用本次静止检查的候选补偿和胸部姿态约定；这条命令不是已完成实机动态验证的结论。

```bash
conda activate supermimic
cd /home/ljp/human_learn/supermimic/H1_SuperMimic_Deploy
export CYCLONEDDS_HOME=/home/ljp/human_learn/Camera2Real/RoboJuDo/.runtime/cyclonedds
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

bash scripts/run_sim2real_bridge.sh \
  --iface enp12s0 \
  --domain 0 \
  --stage air \
  --artifacts deploy_artifacts/last \
  --motion-select \
  --clip deploy_artifacts/use/default.npz \
         deploy_artifacts/use/walk.npz \
         deploy_artifacts/use/dance.npz \
  --pose-listen 127.0.0.1:15101 \
  --pose-source unitree-odom \
  --odom-z-offset 0.075624 \
  --odom-orientation-body torso \
  --pose-max-age 0.1 \
  --log-dir deploy_logs/last_bridge
```

沿用原启动检查和现场 `yes` 确认。选择动作后自动进行固定 PD 过渡，停在 HOLD；按 R1+B 后才执行接管检查，通过后进入策略待机。首次进入 `STANDBY_PRE` 会额外打印一条 `POSE_ALIGN`，包含本次动作的 yaw 旋转、XY 平移、世界高度补偿及补偿后的实测高度；随后满足原条件才打印 `READY`，再按 Start 播放。

两个程序必须在**同一台部署电脑**运行，UDP 只允许 `127.0.0.1`，因为使用本机单调时钟计算数据年龄。该端口只运行这一种位姿发送源。结束整个控制会话后，再结束终端 A；在策略控制中停止 A 会触发原定位失效 DAMP 逻辑。

### 参数怎么改、代码在哪里

| 参数/逻辑 | 修改位置 | 含义 |
|---|---|---|
| `--iface`、`--domain` | A、B 两条命令均修改 | 实机网卡和 DDS 域，必须一致 |
| `--odom-topic` | A 命令；默认 `rt/odommodestate` | 必须是 GO `SportModeState` 消息 |
| `--udp` / `--pose-listen` | A / B 命令 | 相同的本机 UDP 端口，默认示例为 15101 |
| `--pose-source` | B 命令；默认 `reference` | `unitree-odom` 才启用本次接入和自动对齐 |
| `--odom-z-offset` | B 命令，必须显式填写 | 世界坐标 z 加上这个数，单位米；不是机器人目标高度 |
| `--odom-orientation-body` | B 命令，必须显式填写 | `torso`：里程计四元数右乘腰角逆旋转；`pelvis`：不再去除腰角 |
| `--pose-max-age` | B 命令；默认 0.1 秒 | 原位姿超时门槛；新模式以同机单调时钟计时 |
| 5 ms 配对、100 ms 源进度/断流限制 | `scripts/deploy/bridge/h1_odom_bridge.cpp` | C++ 输入检查；不是 `hold-tol`，不会因放宽 B 的超时而自动放宽 |
| 姿态转换、高度补偿、包验证 | `src/h1_deploy/deploy/odometry.py` 的 `OdomPoseReceiver` | 仅在消费里程计包时执行 |
| XY/yaw 对齐公式 | 同文件的 `ReferenceAlignment` | 每次动作准备确定一次，之后保持固定 |
| 重置对齐的时机 | `scripts/deploy/bridge/h1_runner_bridge.py` 中 `ev == 'policy'` | 原 HOLD 接管通过后设置新参考，首次读取才计算变换 |
| 新参数定义及验证 | 同文件 `main()` | 不写入策略的 `deploy_config.json` |

`0.075624` 来自本次“实测腰部原点 0.96 m、原始里程计约 0.884406 m”的首秒拟合。程序执行 `z = odom.z + 0.075624`，不会强行保持 `z=0.96`，也不会把实际高度设成动作首帧的高度。它采用**固定世界高度零点偏移**假设，不等同于沿机身旋转的安装位移。新的可靠标定结果需要在命令中更新，不能根据机器人被吊起后的目标高度随意改补偿。

`torso` 依据当前静态观测与 H1 胸部 IMU 布置选用，仍需在改变腰角、机身姿态时核验该里程计生产者的姿态语义。代码使用里程计自己的世界姿态配合时间对齐的腰角，不把 lowstate 的 IMU 世界航向直接拼到里程计位置上。腰角来自 H1 电机槽位 6，对应策略关节索引 10。

### 动作切换时怎样对齐

设补偿后的实测位置为 `p`，骨盆姿态为 `R`，所选动作首帧为 `p_ref0/R_ref0`。第一次读取时计算 `yaw_delta = yaw(R_ref0) - yaw(R)`，并建立固定 yaw 旋转 `R_align`；水平平移使 `R_align * p` 的 XY 与 `p_ref0` 的 XY 重合，垂直平移始终为 0。随后输出：

```text
p_in = R_align * p + [tx, ty, 0]
R_in = R_align * R
```

因此从走到约 2 m 处再选择 default，会在新一轮准备结束后把该处作为 default 的水平起点。它不会要求机器人回到上一个动作的世界原点。对齐不消除真实高度、横滚、俯仰误差；也不修改参考动作文件。

**Start 时不重算，播放中不重算，进入末帧待机也不重算。** 这样策略仍能看到本次动作过程中的位置和朝向偏差。只有重新经过固定 PD 准备并接管策略，才确定下一次对齐；同一个动作播放结束后重新选择它也遵守这个规则。

R1+B 后、Start 前若机器人移动，位移仍会形成真实跟踪误差，不会自动抹掉。2026-11-15 起，DDS 仿真的 `--sim` 路径也复用此固定 XY/yaw 对齐时机；仿真高度直接保留，不加实机里程计的高度补偿。

两个 runner 的待机/播放状态行现增加 `pelvis err XYZ=[...] m (actual-ref), XY=... m`，退出后的 `[summary] pelvis` 按动作和阶段输出位置误差统计。NPZ 保存比较时使用的参考位置及帧号，详见 [SIM2SIM.md 第 6.5 节](SIM2SIM.md#65-每次动作准备时的坐标对齐和位置误差报告)。这是诊断输出，未新增 Start 位置门槛，也不改变既有 READY/保护条件。实机报告的位置误差由里程计反馈计算，仍受其估计误差影响，不能直接等同于独立测量的真实空间误差。

### 时效性和故障行为

1. C++ 校验 lowstate CRC、腰角、里程计位置/四元数、里程计错误码；检查两路 DDS 发布者身份、源时间递增和 lowstate tick。
2. 使用 DDS 源时间戳配对，最大差 5 ms。消息内部的零值 `stamp` 不作为新鲜度依据；配对失败不退回接收时间配对。
3. 不直接用电脑 UTC 减机器人源时间。两路源时钟必须相容；C++ 跟踪源时间进度相对本机单调时钟的延迟增量，并把样本年龄传给 Python。首次已有的固定传输延迟无法仅靠未同步时钟绝对量出，估计器内部是否重复旧测量也不能只靠 DDS 发布进度证明。
4. 发布者变化、源时间回退/明显前跳、tick 重置、断流或配对失效超过 100 ms 时，C++ 停止输出并报错。Python 另查接收会话、序号、源时间和年龄，重复包不会刷新有效期；生产者重启会锁定错误，需要重新启动控制器并准备动作。
5. 在策略阶段，位姿失效沿用 `anchor feedback unavailable → DAMP`，没有固定高度/最后位姿/单独 IMU 的隐式回退。恢复反馈不会自动恢复运动。

### 验证结果与后续工作

已编译新 C++ 程序；三个实际参考动作在已知 XY/yaw/高度变换和腰角转换后，与正确坐标下的输入完成 3,978 次策略推理对照，动作输出差为 0。覆盖首帧待机、播放、末帧待机，检查了真实高度/倾斜误差保留与约 2 m 后重新对齐。`last.pt` SHA256 仍为 `5c5c47e8dadd4d350ea5058b142e601b44a043d43bd931cc375326c73e9fb1bf`。

本机隔离 DDS 域的 C++ → UDP → Python 测试通过，覆盖有效数据、CRC 错误、冻结源时间戳、两路时间不匹配、断流、源时间回退/前跳、tick 重置、DDS 发布者变化，以及 Python 会话重启拒绝。测试未创建 `LowCmd` 发布者，临时文件随测试清理。这些是通信与输入/推理验证，不能代替动力学闭环或实机动作稳定性验证。

新 C++ 程序随后在 `enp12s0/domain 0` 执行了 20 秒 `--check-only` 实机只读检查，取得 1,875 组有效配对，CRC 丢弃为 0；周期采样显示的配对差约 0.11～1.81 ms、数据年龄约 0.48～2.92 ms。这些是显示点的范围，不是全部样本的最大值。该检查没有 UDP 输出、日志文件或电机命令。

`last` 输入维度保持 `obs=720（10×72）、goal=530、action=19`。输入链路已接齐，但还需按现场条件逐步核验改变姿态后的高度/原点与胸部姿态约定，以及带保护的策略待机、播放、返回 default 表现。本次没有向实机发送电机命令，也没有改变实机支撑或控制模式。
