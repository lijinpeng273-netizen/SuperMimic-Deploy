# H1 Sim2Sim

## 回放

```bash
bash scripts/run_sim2sim.sh \
  --artifacts deploy_artifacts/ckpt_latest \
  --clip deploy_artifacts/clips/walk1_subject1.npz
```

加 `--headless` 关闭窗口；`--seconds` 限制播放时长；`--latency-ms` 增加指令延迟；`--log` 指定状态/动作输出。发生部署核心 abort 时停止本次回放。

仿真通过策略关节顺序访问模型，使用训练 PD 参数、力矩限幅和 mesh 足底。anchor 使用当前仿真根位置、姿态、关节及相应参考帧计算，6D 姿态按训练的旋转矩阵前两列逐行展开。

## 原部署评估功能

```bash
python scripts/deploy/sim2sim.py --cmd g2,g3,report \
  --ckpt ckpt_latest --clips walk1_subject1 --workers 1
python scripts/deploy/sim2sim.py --cmd standby \
  --ckpt ckpt_latest --clips walk1_subject1 \
  --standby-pre-s 10 --standby-post-s 10
python scripts/deploy/sim2sim.py --cmd timing,compare --clips walk1_subject1
```

`g2` 保留 PD、延迟、Kp、额外质量、摩擦和控制限幅变体；`g3` 保留站姿、插值、HOLD、策略、结束过渡和吊架条件。`standby` 保留动作前后策略待机及稳定性窗口检查。`timing` 统计本机推理时延；`compare` 对照位置控制和力矩控制场景。报告位于 `deploy_artifacts/reports/`。

`parity` 对照训练环境轨迹，输入应来自原训练工程的 `scripts/deploy/record_h1_reference.py`，放到 `deploy_artifacts/reports/sim2sim/ref_<prefix><ckpt>_no-tf32.npz`（另可提供 `_tf32.npz`）。设置 `--ref-prefix` 选择前缀。该参考文件不能由独立部署仓库凭空生成。

`lw`、`lw_robust` 保留锁定/释放 torso yaw 的仿真对照；`--lock-waist` 只代表仿真焊接，不代表 H1 硬件型号或电机缺失。数据分组名称 SAFE 仅沿用报告结构，不表示已批准用于硬件。

`--imu-model` 是可选饱和压力模型，默认关闭，参数不代表 H1 实测标定。仿真 gate 为统计故障可在记录 abort 后继续，和发生 abort 即停止的回放/真机运行器用途不同。

## DDS 仿真

### 1. 验证范围与当前结果

这里运行的是 **MuJoCo 仿真器 + 实际部署控制器 + 模拟手柄**，不是上面的批量评估入口 `scripts/deploy/sim2sim.py`。三个终端的关系是：

```text
终端 C：写入/清空按键文件
          ↓
终端 A：sim_h1_mujoco.py
          ├─ rt/lowstate（关节、IMU、手柄按键）── DDS ──→ 终端 B
          ├─ pelvis 位姿 ────────────────────── UDP ──→ 终端 B
          └─ 接收 rt/lowcmd ←──── DDS ───── h1_runner.py：状态机、策略、控制指令
```

仿真发布 20 个 GO 槽位（19 个关节和空槽 9），IMU 为 torso 观测；控制器使用与实机相同的电机映射和 torso→pelvis 转换。DDS、位姿 UDP、按键文件是三种不同的接口，配置时不能混淆。

截至 **2026-11-12**，本机验证结果如下：

| 项目 | 实际结果 |
|---|---|
| SDK 的 CRC 动态库缺失 | 已在 `supermimic` 环境补齐；实际 LowState/LowCmd CRC 计算通过 |
| 默认 MuJoCo 场景缺少 IMU 传感器 | 已在 `models.py` 的 `scene_xml()` 中启用 `imu_quat`、`imu_gyro`、`imu_acc` |
| 无窗口 DDS / 固定 PD / 按键 | 已通过：PASSIVE→INTERP→HOLD→L2+B→DAMP；约 500.23 Hz，写入失败和回调错误均为 0，两进程正常退出 |
| 原程序顺序下的完整策略播放及三动作切换 | **尚未跑通**；存在固定 PD 接管条件、首次位姿缓存过期和首次 FK 初始化耗时问题 |
| 带窗口退出 | 观察到一次日志保存后退出段错误；未通过改变窗口退出顺序处理 |

随后按用户要求，将 DDS 仿真吊点从骨盆移到颈根：`torso_link` 局部坐标 `[0, 0, 0.50]` m，详见第 7.2 节。上表的 DDS 通过记录来自此次吊点修正之前；颈部吊点已完成离线位置/速度、力矩等价性和固定 PD 检查，尚不能据此认定完整 DDS 策略切换通过。

2026-11-13 新增 `--band-anchor-offset-z`，可以独立抬高上方悬挂点，默认 `0`，保持原支撑条件；终端 A 同时显示颈部高度、上方悬挂点高度和吊带竖向力。固定 PD 的目标是吊带承担主要重量、双脚轻触地面。本次只进行了离线受力及固定 PD 对照，具体数值见第 6.1 节，不代表 DDS 策略播放或释放吊带后的平衡已通过。

按用户要求，控制阶段顺序、内部位姿读取时机、FK 模型初始化时机均保持原样。调试中尝试过的“非策略阶段提前读取位姿”和“启动阶段提前预热 FK”已撤回，不能把临时版本的结果算成当前程序的通过结果。保护阈值也没有放宽。

随后按用户要求调整了 DDS 吊带启停时机：固定 PD 全程保留支撑；STANDBY_PRE 首次有效推理并成功提交控制指令后才释放；动作结束进入 STANDBY_POST、进入固定 PD 过渡或 DAMP 时恢复支撑。退出控制器也保留支撑文件，直到仿真器停止后再清理。Python 和 bridge 两个入口保持一致，详见第 6.4 节。12 项无 DDS 的控制流程回归检查通过，包含发送失败、推理失败和并发急停；测试脚本及临时文件已按用户要求删除。这不代表完整 DDS 物理播放或实机接管验证通过。

下面先给出已通过的基础联调方法，再给出完整播放的操作顺序和调参位置。完整播放部分是复现与诊断步骤，**尚不能保证当前配置可以完成三个动作**。

### 2. 三个终端的公共准备

打开终端 A、B、C，**每个终端各执行一次**：

```bash
conda activate supermimic
cd /home/ljp/human_learn/supermimic/H1_SuperMimic_Deploy

export CYCLONEDDS_HOME=/home/ljp/human_learn/Camera2Real/RoboJuDo/.runtime/cyclonedds
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

这些路径对应当前机器。使用的是本机已有的 Cyclone DDS 动态库；环境变量只影响当前终端，不需要修改 `.bashrc` 或 base 环境。当前 `supermimic` 的 SDK 已修复，不要为了执行下面步骤重复安装。

本节统一使用以下连接配置，避免与其他正在运行的仿真实例混用：

| 接口 | 本节取值 | 要求 |
|---|---|---|
| DDS 网卡 | `lo` | A、B 相同，表示本机回环 |
| DDS 域 | `137` | A、B 相同，且必须非零；同步改为 `1` 也可以 |
| 位姿 UDP | `127.0.0.1:25137` | A 的 `--pose-udp` 对应 B 的 `--pose-listen` |
| 按键文件 | `/tmp/h1_supermimic_dds.buttons` | A 读取，C 写入 |
| 吊带控制文件 | `/tmp/h1_supermimic_dds.band` | B 写入完整支撑/渐进释放指令，A 逐物理步计算支撑比例；完全释放后 B 删除 |

`--domain 0` 是实机域，不用于本节；不要同时在同一域启动两份控制器。若已有上一轮测试，先按本节“停止、日志与清理”结束，再启动新一轮。

在任一终端做一次依赖检查；这不会创建 DDS 通道或发送控制指令：

```bash
python - <<'PY'
import sys
from pathlib import Path
import mujoco
import unitree_sdk2py
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.idl.default import (
    unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_,
)

print('Python:', sys.executable)
print('MuJoCo:', mujoco.__version__)
library = Path(unitree_sdk2py.__file__).parent / 'utils/lib/crc_amd64.so'
print('CRC library:', library, 'exists:', library.is_file())
crc = CRC()
print('LowState CRC:', hex(crc.Crc(unitree_go_msg_dds__LowState_())))
print('LowCmd CRC:', hex(crc.Crc(unitree_go_msg_dds__LowCmd_())))
PY
```

只验证 `import unitree_sdk2py` 不足以发现缺失的 CRC 动态库。若这里仍出现 `crc_amd64.so: cannot open shared object file`，先处理 SDK 安装完整性；修改 `--iface`、KP/KD 或动作文件无法修复这个错误。

### 3. 第一轮：验证 DDS、固定 PD 和按键

这一轮使用 `--no-policy`，目的是先确认通信、插值和阻尼按键；不会进入策略，也不会显示用于播放的 READY。

#### 3.1 终端 A：准备文件，先启动仿真器

```bash
mkdir -p outputs/dds_switch/basic_001
: > /tmp/h1_supermimic_dds.buttons
: > /tmp/h1_supermimic_dds.band

python scripts/deploy/sim_h1_mujoco.py \
  --iface lo \
  --domain 137 \
  --clip deploy_artifacts/use/default.npz \
  --dt 0.001 \
  --band-file /tmp/h1_supermimic_dds.band \
  --buttons-file /tmp/h1_supermimic_dds.buttons \
  --pose-udp 127.0.0.1:25137 \
  --log outputs/dds_switch/basic_001/simulator.npz
```

默认无窗口。需要观察画面时在命令最后追加 `--viewer`；窗口退出问题应与 DDS 收发问题分别记录。

仿真器收到第一条控制指令前不推进物理仿真。此时输出 `t=0.00s`、`commands=0` 是正常等待，不是卡死。

仿真器这里的 `--clip` **仅用于初始化根位置、根姿态和关节角**。它不负责选择、推进或播放动作；后续播放哪个 NPZ 由终端 B 决定。

#### 3.2 终端 B：启动控制器，明确禁止策略接管

```bash
bash scripts/run_sim2real.sh \
  --iface lo \
  --domain 137 \
  --sim \
  --stage ground \
  --artifacts deploy_artifacts/ckpt_latest \
  --motion-select \
  --clip deploy_artifacts/use/default.npz \
         deploy_artifacts/use/walk.npz \
         deploy_artifacts/use/dance.npz \
  --pose-listen 127.0.0.1:25137 \
  --sim-band-file /tmp/h1_supermimic_dds.band \
  --no-policy \
  --log-dir outputs/dds_switch/basic_001/controller
```

虽然脚本名包含 sim2real，本条命令通过 `lo`、非零 DDS 域和 `--sim` 连接仿真器。`--stage ground` 选择原有 ground 保护档位，不自动改变吊带或接触面，也不表示已批准实机落地。

等待终端 B 出现：

```text
STATE: start in PASSIVE
```

此后终端 A 的 `commands` 应持续增加。不要在启动前一直按住选择键；原按键检测要求松开后再按下。

#### 3.3 终端 C：定义模拟按键函数

在 **B 启动完成之后**，在终端 C 执行：

```bash
press() {
  printf '%s\n' "$1" > /tmp/h1_supermimic_dds.buttons
  sleep 0.25
  : > /tmp/h1_supermimic_dds.buttons
  sleep 0.25
}
```

以后在终端 C 输入 `press 'L1+A'` 并按回车，就是模拟一次手柄组合键。过程为：

```text
写入 L1+A → 保持 0.25 秒 → 清空文件 → 松开 0.25 秒
             ↓
仿真器把组合键转成 wireless_remote 按键位 → 发布 LowState → 控制器去抖并处理事件
```

这不是给 MuJoCo 窗口发送键盘事件，也不要求连接真实手柄。组合键使用 `+` 分隔，不能写成 `L1 A`。`press` 是当前终端里的 Bash 函数，换终端或重新打开终端后需要重新定义；若出现 `press: command not found`，先执行上面的函数定义。

| 在终端 C 输入的命令 | 模拟的手柄操作 |
|---|---|
| `press 'L1+A'` | 选择 default |
| `press 'L1+X'` | 选择 walk |
| `press 'L1+Y'` | 选择 dance |
| `press 'R1+B'` | HOLD 中请求起始检查，通过后进入策略首帧待机 |
| `press 'start'` | 单独按 Start，准备完成后播放 |
| `press 'L2+B'` | 阻尼停止 |

按下面顺序逐条执行，每一步先观察终端 B，不要一次性粘贴所有按键命令：

1. 执行 `press 'L1+A'`，观察 `PASSIVE → INTERP → HOLD`。默认插值约 4 秒。
2. HOLD 中先执行 `press 'R1+B'`：本轮有 `--no-policy`，应出现 `REFUSED: R1+B ... --no-policy` 并继续固定 PD。再执行 `press 'start'`，应提示先按 R1+B、不排队播放。
3. 执行 `press 'L2+B'`，应出现 `DAMP: ... remote L2+B`。
4. 按下面的停止顺序结束 A、B，保存日志。

只有在没有通信异常、选择键正常、Start 没有越过门槛、L2+B 正常触发的情况下，才算基础联调通过。这个结果不包含策略播放成功。

### 4. 第二轮：完整播放和三动作切换的操作顺序

**当前原顺序程序仍有第 9 节列出的阻塞。本节描述目标流程；遇到 WAIT、REFUSED 或 DAMP 时先记录原因，不要把“按键发出”当作“切换完成”。**

第一轮进入 DAMP 后不能用 Start 恢复。先结束两个程序；第二轮必须重新启动仿真器和控制器，复位机器人状态、按键去抖和日志。不能让上一轮阻尼后的机器人继续作为新一轮起点。

#### 4.1 终端 A：重新初始化仿真器

下面按 default 首帧初始化，并用新增的 `--band-anchor-offset-z 0.18` 抬高上方悬挂点 18 cm，保持吊带自然长度 1 m。它是当前模型下“双脚轻触地面”的离线固定 PD 对照条件，详见第 6.1 节；不代表后续策略待机、walk 或 dance 已通过。

```bash
mkdir -p outputs/dds_switch/motion_001
: > /tmp/h1_supermimic_dds.buttons
: > /tmp/h1_supermimic_dds.band

python scripts/deploy/sim_h1_mujoco.py \
  --iface lo \
  --domain 137 \
  --clip deploy_artifacts/use/default.npz \
  --dt 0.001 \
  --band-file /tmp/h1_supermimic_dds.band \
  --band-length 1.0 \
  --band-anchor-offset-z 0.18 \
  --buttons-file /tmp/h1_supermimic_dds.buttons \
  --pose-udp 127.0.0.1:25137 \
  --log outputs/dds_switch/motion_001/simulator.npz
```

这里的 `--clip` 决定机器人的初始根位姿和关节角；`--band-anchor-offset-z` 只改变上方悬挂点，不移动机器人，也不改变参考 NPZ。`--init`、`--pelvis-z` 在传入 `--clip` 后不生效。

若要复现新增参数前的支撑条件，将 `0.18` 改为 `0`。若改用 `--init knees_bent --pelvis-z 1.10` 替换 `--clip`，应从偏移 `0` 重新调起，不能直接叠加这 18 cm；初始高度、姿态或模型改变后，同一偏移可能把双脚吊离地面。该参数不提供动作参考坐标的自动对齐。

#### 4.2 终端 B：允许原有检查通过后进入策略

```bash
bash scripts/run_sim2real.sh \
  --iface lo \
  --domain 137 \
  --sim \
  --stage ground \
  --artifacts deploy_artifacts/ckpt_latest \
  --motion-select \
  --clip deploy_artifacts/use/default.npz \
         deploy_artifacts/use/walk.npz \
         deploy_artifacts/use/dance.npz \
  --pose-listen 127.0.0.1:25137 \
  --sim-band-file /tmp/h1_supermimic_dds.band \
  --log-dir outputs/dds_switch/motion_001/controller
```

与第一轮相比，去掉 `--no-policy`，更换日志目录；不增加 `--auto-sequence`、`--end-hold-legacy`。后二者与 `--motion-select` 不兼容。

控制器的 `--clip` 必须包含三个不同文件。自定义文件名按参数顺序对应 L1+A、L1+X、L1+Y，例如 `default.npz walk2.npz dance2.npz`。若仍使用旧组合 `default.npz`、`walk.npz`、`dance.npz` 各一次，则保留按文件名绑定、排列顺序任意的行为。启动横幅显示实际映射。切换的是同一策略的参考动作；本例网络模型仍来自 `--artifacts deploy_artifacts/ckpt_latest`。

#### 4.3 终端 C：先 default，再 walk/dance，最后返回 default

保持第 3.3 节的 `press` 函数，等待 B 启动完成。下面各步都必须根据日志逐步执行。

| 步骤 | 终端 C 操作 | 终端 B 应出现的结果 |
|---|---|---|
| 1 | `press 'L1+A'` | 选择 default，INTERP→HOLD，保持固定 PD 等待 R1+B |
| 2 | HOLD 中 `press 'R1+B'`，通过起始检查后等待 | 进入 STANDBY_PRE；稳定性检查通过后出现 `READY: ...准备完成，可以按 Start 播放` |
| 3 | `press 'start'` | STANDBY_PRE→POLICY，从 default 第 0 帧播放 |
| 4 | 等 default 结束 | POLICY→STANDBY_POST，立即恢复仿真吊带支撑，策略保持末帧参考 |
| 5 | `press 'L1+X'` | 选择 walk，END_HOLD→INTERP→HOLD，等待 R1+B |
| 6 | HOLD 中 `press 'R1+B'`，等新的 READY，再 `press 'start'` | 播放 walk |
| 7 | walk 播放期间 `press 'L1+A'` | 记录待切换目标 default，等待 walk 自然结束，不立即打断 |
| 8 | 等 walk 结束、PD 过渡完成到 HOLD，再 `press 'R1+B'`，等新的 READY | 准备 default；仍需手动接管，不会自动播放 |
| 9 | `press 'start'` | 播放 default |
| 10 | default 结束后 `press 'L1+Y'` | 选择 dance，按同样流程准备 |
| 11 | HOLD 中 `press 'R1+B'`，等新的 READY，再 `press 'start'` | 播放 dance |
| 12 | dance 结束后 `press 'L1+A'`，等 HOLD、`press 'R1+B'`，等 READY、`press 'start'` | 验证 dance 返回 default |

对应状态顺序为：

```text
第一次选择：
PASSIVE → INTERP（固定 PD）→ HOLD（固定 PD）
        → 按 R1+B，原接管检查通过 → STANDBY_PRE（策略首帧待机）
        → 原待机检查通过 / READY → Start → POLICY
        → 动作结束 → STANDBY_POST（策略末帧待机）

待机时选择其他动作：
STANDBY_PRE / STANDBY_POST → END_HOLD（目标/增益渐变）
                         → INTERP → HOLD → 按 R1+B，检查通过 → STANDBY_PRE
                         → READY → Start → POLICY
```

STANDBY_PRE 已经在运行策略，只是参考固定在首帧；Start 控制的是动态参考开始推进。固定 PD 阶段仍依赖外部支撑，仿真通过吊带标志文件模拟，不能将其理解为始终由策略保持平衡。

两个 DDS Runner 支持 `--policy-blend-s`（默认 5 秒）：R1+B 检查通过后，在 STANDBY_PRE 内从 HOLD 目标渐变到实时策略目标。配置仿真吊带时，还需等待 `--sim-band-ramp-s`（默认 5 秒）的渐进释放完成；**策略完全接管且吊带支撑为零后**，才累计 READY 所需的稳定性/温升窗口。KP/KD 和保护门槛不变。参数可追加到终端 B 命令，**它们不属于批量评估入口 `scripts/deploy/sim2sim.py`，该批量入口的历史报告不代表此接管逻辑已经通过**。早期 last 无吊架闭环测试的限制见 [SIM2REAL.md 的渐变接管说明](SIM2REAL.md#r1b-后的目标渐变接管2026-11-14)；2026-11-15 的 ckpt_latest + default DDS 验证见第 6.4 节。

吊带释放以首次有效策略指令提交成功为前提，不会在首次推理或位姿读取前提前释放。动作结束进入 STANDBY_POST 时恢复吊带，虽然该阶段继续推理静态末帧，也不会再次放绳；选下一动作并再次进入 STANDBY_PRE 后，才开始下一轮释放流程。

其余按键边界：

- HOLD 不自动接管；R1+B 检查失败会打印原因并继续固定 PD，需要松开后重新按。提前按 R1+B 不排队；每次准备动作都必须重新按。
- R1+B 与其他按键重叠时不接管，需松开 R1、B 后再按；L2+B 保持最高优先级。
- 准备过程到首次 READY 前忽略新的动作选择。
- 提前按 Start 会被拒绝/忽略，不会排队；准备完成后需松开并重新按。
- 播放中重复按 Start 不重播；同时按选择键和 Start 时只处理选择。
- 已准备好或正在播放时重复选当前动作不重启。播放结束后，可重新选择同一动作，再等 HOLD、按 R1+B、等 READY、按 Start 重播。
- 播放中选择其他动作只更新待切换目标；当前动作结束后才进入 PD 过渡。不要把它理解成任意帧立即切换。
- DAMP 是锁定状态，进入后结束并重启本轮，不能靠选择键或 Start 恢复。

### 5. 停止、日志与清理

正常结束顺序：

1. 终端 C 执行 `press 'L2+B'`，确认终端 B 报告阻尼原因 `remote L2+B`。
2. 终端 B 按 `Ctrl+C`，等待约 1 秒阻尼发送和 `[log]` / `[summary]` 输出，直到进程结束。
3. 终端 A 按 `Ctrl+C`，等待保存仿真日志并退出。默认不使用窗口；若启用了窗口，记录退出是否出现段错误。
4. 两个进程都结束后，才清理按键和吊带标志。

进入 DAMP 和退出终端 B 时，吊带支撑文件会保留，避免终端 A 仍在运行时失去支撑。正常清理应在终端 A 结束之后进行，不能将这个运行中的标志文件当作测试垃圾提前删除。

```bash
rm -f /tmp/h1_supermimic_dds.buttons /tmp/h1_supermimic_dds.band
```

程序运行时不要通过手工删除吊带文件来代替控制器的吊带释放流程。文件被手动删除后，控制器内部的释放时间记录不一定同步。

日志位置示例：

```text
outputs/dds_switch/basic_001/       # 第一轮
├── simulator.npz
└── controller/
    ├── run_时间戳.console.txt
    └── run_时间戳.npz

outputs/dds_switch/motion_001/      # 第二轮，结构相同
```

| 文件 | 保存的内容 | 用途 |
|---|---|---|
| 输入 `deploy_artifacts/use/*.npz` | 参考动作、目标特征、参考根位姿等 | 告诉策略要做什么 |
| 输出 `simulator.npz` | `t`、实际 `qpos`、`pd`、吊带状态 `band`、支撑比例 `band_scale`、上方吊点坐标 `band_anchor`、收到的命令数量 `n_cmd` | `band_scale` 从 1 渐减到 0；`pd` 是本步限幅前需求力矩，实际提交给模型时另有限幅 |
| 输出 controller `run_*.npz` | 关节状态、动作、目标、状态机、命令时序及 `meta` 中的事件/停止原因等 | 定位通信、策略和保护问题 |
| 输出 `run_*.console.txt` | 控制器控制台文本 | 优先阅读状态切换和拒绝原因 |

输入和输出都用 NPZ 容器，但含义、字段不同，不能互相替代。

查看第一轮关键日志：

```bash
rg 'STATE|SELECT|READY|WAIT|REFUSED|ABORT|DAMP|summary' \
  outputs/dds_switch/basic_001/controller/*.console.txt
```

第二轮把路径里的 `basic_001` 换成 `motion_001`。每次调参换一个新的目录名，避免覆盖 `simulator.npz`。确认不需要保留某一轮记录后，才删除该轮目录，例如：

```bash
rm -r -- outputs/dds_switch/basic_001
```

本次助手调试使用的临时脚本、构建目录、日志和 NPZ 已删除；上述目录是供用户之后运行时创建的示例，不代表仍保存着那次测试的原始日志。

### 6. 调参原则与命令行修改位置

先在启动命令中传参数，每次只改一类因素并重新启动该轮。命令行参数只对该进程生效；修改 JSON、XML 或 Python 常量则会影响之后的运行，需要记录原值。

建议顺序：**先检查通信与模型 → 用 `--no-policy` 检查固定 PD → 检查接管原因 → 检查策略待机 → 单动作播放 → 动作切换**。本节不更改控制阶段顺序，也不提前读取位姿或预热 FK。

#### 6.1 终端 A：仿真器参数

定义位置：[sim_h1_mujoco.py](../scripts/deploy/sim_h1_mujoco.py) 的 `main()` / `add_argument()`。

| 参数 | 默认值 | 怎么修改、影响什么 |
|---|---|---|
| `--domain` / `--iface` | `1` / `lo` | 本节显式使用 `137` / `lo`；改域时同步修改 B，避免其他控制器干扰 |
| `--clip` | 未指定 | 提供后从该参考首帧初始化；不是播放动作选择开关 |
| `--init` | `knees_bent` | 仅在未给 `--clip` 时生效；可选 `zero` / `knees_bent`，通常先保留后者 |
| `--pelvis-z` | `1.05` m | 仅在未给 `--clip` 时生效；调整初始高度也会移动初始吊带悬挂点 |
| `--dt` | `0.001` s | 物理步长；先保持 1 ms。增大它会改变物理积分和状态发布节奏，不等于加快策略 |
| `--band-file` | 未指定 | 读取 B 的支撑控制文件；空文件为完整支撑，释放指令为渐减支撑，文件不存在为无支撑；必须与 B 的 `--sim-band-file` 一致 |
| `--band-length` | `1.0` m | 同时影响吊带自然长度和初始悬挂点位置；不是“支撑力倍数” |
| `--band-anchor-offset-z` | `0.0` m | 独立调整上方固定悬挂点的竖向位置；正值抬高，负值降低，不改变自然长度或机器人初始位置；与 `--clip` 同时生效 |
| `--buttons-file` | 未指定 | 必须与 C 的 `press` 写入路径一致 |
| `--pose-udp` | 未指定 | 当前策略需要；与 B 的监听地址一致，独立于 DDS 域号 |
| `--duration` | `0` | 运行墙钟秒数；0 表示一直运行，按 Ctrl+C 结束 |
| `--viewer` | 关闭 | 打开 MuJoCo 窗口；排查收发或退出问题时先保持关闭 |
| `--status-every` | `1` s | 仿真器状态打印周期；只改变打印频率 |
| `--log` / `--log-tau` | 不记录 / 关闭 | 指定日志路径；`--log-tau` 使仿真日志每物理步采样，数据量更大 |
| `--motor-temp` / `--temp-file` | `30` °C / 未指定 | 模拟温度。温度文件每行是硬件电机槽位、起始温度、升温速率；不是策略关节序号 |

初始高度调参的正确写法是用：

```text
--init knees_bent --pelvis-z 1.10
```

**替换** A 命令中的 `--clip deploy_artifacts/use/default.npz`，不要把两组参数同时保留后误以为高度已变化。B 的三个动作 `--clip` 保持不变。

原骨盆吊点的历史测试：default 的参考首帧高度约 1.044 m；按参考初始化时曾在 HOLD 出现约 0.127 rad 关节误差。使用已有高度参数 1.10 m 时，曾获得约 0.083 rad 误差、约 1.5° 倾斜，能够通过原 HOLD 门槛；但后续出现位姿缓存阻塞，不能称为全流程成功。1.30 m 的试验反而在 INTERP 中倾斜约 47° 并触发 DAMP，因此不要认为“抬得越高越稳定”。

颈部吊点修正后的离线固定 PD 检查（未连接 DDS、未播放策略）：运行 6.5 秒后，首帧高度约 1.044 m 时关节误差约 0.126 rad、倾斜约 1.68°；初始高度 1.10 m 时误差约 0.086 rad、倾斜约 0.53°。这些结果只描述所测固定 PD 条件，不是 READY 或完整动作切换的验收结果。原 `--hold-tol 0.1` 未修改。

**独立调高上方悬挂点（2026-11-13）：**

只在终端 A 的启动命令中添加或修改，例如：

```text
--clip deploy_artifacts/use/default.npz --band-length 1.0 --band-anchor-offset-z 0.18
```

终端 B、C 的命令和按键不需要因为这个新参数而修改。命令行值每次启动生效；如需修改程序默认值，位置是 `sim_h1_mujoco.py` 中该参数的 `add_argument(..., default=0.0)`。上方悬挂点计算位于 `ElasticBand.__init__()`：

```text
上方悬挂点 = 初始颈部世界位置 + [0, 0, band_length + band_anchor_offset_z]
吊带自然长度 = band_length
```

此参数没有自动找地面或自动保持接触的功能。调参目标是在 HOLD 时让双脚轻触地面、由吊带承担主要重量，避免下沉造成较大关节误差，也避免双脚悬空。零速度、吊带竖直且颈部位置相同的条件下，抬高 5 cm 会增加约 `2000 × 0.05 = 100 N` 的弹簧拉力；机器人随后移动、倾斜时，实际竖向力会变化。

本机离线对照使用默认模型、default 首帧原位初始化、1 ms 物理步长、20 ms 更新固定 PD 目标、原固定 PD 增益和力矩限幅。PASSIVE 0.06 s 后做 4 s 插值，再保持到总时长 6.5 s；下表是结束时的测量值，没有运行 DDS 或策略网络：

| 上方悬挂点偏移 | 最大关节误差 rad | 吊带竖向力 N | 左脚地面法向力 N | 右脚地面法向力 N |
|---|---:|---:|---:|---:|
| `0` m | 0.1263 | 177.8 | 165.2 | 163.7 |
| `0.03` m | 0.1017 | 225.1 | 141.5 | 140.1 |
| `0.05` m | 0.0881 | 257.2 | 125.4 | 124.1 |
| `0.10` m | 0.0613 | 339.3 | 84.4 | 83.1 |
| `0.15` m | 0.0410 | 422.4 | 43.8 | 40.4 |
| `0.18` m | 0.0514 | 472.6 | 18.8 | 15.3 |
| `0.20` m | 0.0344 | 499.7 | 3.9 | 3.1 |

模型重量约 506.7 N。因此，5 cm 的结果只是“仍然接地”，双脚还承担约一半重量；18 cm 更接近轻触地面，20 cm 已接近离地边界。18 cm 时两脚碰撞几何最低点约为 -0.22 mm、-0.13 mm；MuJoCo 的柔性接触允许微小穿透，不要求数学上严格等于 0。上述数值证明测量时两脚有接触力，并不证明整个脚掌完全贴平，也不能推广到其他姿态。

另做了 PASSIVE 等待 26 s 才选择动作的离线对照，以接近此前操作日志中的等待时间，之后仍插值 4 s、固定 PD 保持 2.44 s。偏移 `0`、`0.05`、`0.10` m 时，结束关节误差分别约为 0.1922、0.1513、0.1202 rad，均未低于 0.1 rad；偏移 `0.18` m 时约为 0.0508 rad，关节最大速度约 0.0012 rad/s，倾斜约 2.44°。这时左右脚法向力约为 2.1 N、34.2 N，双脚虽都接触，但受力已不均匀。说明 PASSIVE 等待造成的姿态变化会影响结果，不能将短等待测试的轻触状态视作所有操作时机都能复现。

每轮应结束 B、A，复位按键文件和支撑文件，再重新启动 A、B，使用新的日志文件名。保持相同初始姿态和选择动作的等待时间，只修改该偏移，从 `0`、`0.03`、`0.05`、`0.10` 等值逐步对照；接近轻触状态时可改用 1 cm 的步长。不能因为关节误差低于 0.1 rad 就认定高度合适，还要观察双脚是否离地、躯干是否倾斜。不要同时修改 KP/KD 或放宽 HOLD 门槛。

终端 A 的状态行新增：

```text
neck_z=1.487m anchor_z=2.724m band_fz=472.6N
```

这是 18 cm 对照条件下的示意读数：`neck_z` 是当前颈部吊点世界高度，`anchor_z` 是固定上方悬挂点世界高度，`band_fz` 是当前物理步实际施加的吊带竖向力，正值向上，包含弹簧与阻尼。`band=False` 时该力为 0，上方悬挂点坐标仍保留。渐进释放版本另显示 `support=100.0% → 0.0%`，并在 NPZ 增加 `band_scale`；该比例乘在原弹簧、阻尼力上，不是机器人重量的支撑百分比。上表的左右脚接触力来自离线测量，并不是状态行新增字段。

当前在 STANDBY_PRE 有效推理并成功提交控制指令后释放吊带，时机详见第 6.4 节。轻触地面时吊带承担的重量更多，释放时转移到双脚的载荷也更大；固定 PD 数据不能证明这一接管过程稳定。这里没有增加渐进卸载、自动调高或跟随机器人移动的悬挂点。

#### 6.2 终端 B：过渡、稳定性与按键参数

定义位置：[h1_runner.py](../scripts/deploy/h1_runner.py) 的 `main()`。实际判断位于 `policy_allowed()`、`standby_allowed()` 和 `run()`。

| 参数 | 原默认值 | 用途与调参说明 |
|---|---|---|
| `--artifacts` | `gc.DEFAULT_ARTIFACTS` | 显式指向 checkpoint 目录，本节为 `deploy_artifacts/ckpt_latest`；与批量评估的产物根目录参数不同 |
| `--motion-select` | 关闭 | 启用本节三个动作选择键及 Start；不改变模型权重 |
| `--no-policy` | 关闭 | 基础联调时启用；HOLD 后策略接管必然被拒绝，不要等待 READY |
| `--interp-s` | `4.0` s | 固定 PD 插值时间；如需比较更慢的过渡，可在 B 中追加 `--interp-s 6.0`，它不会解决固定姿态本身不能平衡的问题 |
| `--end-blend-s` | `1.0` s | 离开策略待机时目标和 KP/KD 渐变时间；如比较 `2.0` s，应记录过渡误差和姿态，而不只看是否进入下一状态 |
| `--hold-min-s` | `1.0` s | HOLD 至少等待多久再允许策略接管；它不替代误差/速度/倾斜检查 |
| `--hold-tol` | `0.1` rad | 接管时最大关节位置误差必须小于它；约 5.73°。先定位哪个关节偏差大，增大门槛不改善姿态 |
| `--start-max-dq` | `0.5` rad/s | HOLD 接管时最大关节速度门槛 |
| `--start-max-tilt-deg` | `10.0`° | HOLD 接管时相对参考重力方向的倾斜门槛 |
| `--standby-min-s` | `3.0` s | Start 前最短策略待机时间；增加等待只能提供观察时间，不能修复持续失稳 |
| `--standby-window-s` | `3.0` s | 检查最近多少秒的待机稳定性；加长窗口会要求更长的稳定记录 |
| `--standby-max-tilt-deg` | `8.0`° | 待机窗口内最大参考倾斜误差门槛 |
| `--standby-max-dq` | `1.0` rad/s | 待机窗口内最大关节速度门槛 |
| `--button-stable-n` | `3` 条状态消息 | 同一按键位连续稳定多少条才处理；不稳定时先检查按键保持时间和状态接收 |
| `--button-release-s` | `0.1` s | 重新触发前的松开时间；C 函数当前松开 0.25 s |
| `--button-max-age-s` | `0.1` s | 主循环丢弃过期按键事件的门槛；不是按键保持时长 |
| `--status-s` | `3.0` s | 控制器状态/等待提示打印周期；观察拒绝原因可先改为 `1.0` |
| `--log-dir` / `--log-obs` | `deploy_logs/` / 关闭 | 输出目录；`--log-obs` 另存策略观测历史，体积更大 |

例如只比较更慢的插值，可在第 3.2 节完整命令中把日志目录改为 `outputs/dds_switch/interp6_001/controller`，并在最后追加：

```text
--interp-s 6.0 --status-s 1.0
```

A 的 `--log` 也放入同名实验目录。保持其他参数不变，重复相同按键步骤，比较最差关节误差、倾斜和停止原因。这里给的是现有参数的使用方法，并未实际替你修改参数。

#### 6.3 执行频率、保护档位与时效参数

| 参数/配置 | 默认值 | 含义 |
|---|---|---|
| `--cmd-hz` | `500` Hz | 低层命令发送频率；不是策略推理频率 |
| JSON 的 `control_dt` | `0.02` s | 策略 50 Hz；当前配置加载要求此值为 0.02，不能当作普通提速开关随意改 |
| `--ort-threads` | `1` | ONNX Runtime 线程数，也由当前策略加载器使用；加线程可能争抢控制循环 CPU，需根据实测时序判断 |
| `--cmd-cpu` | `auto` | 命令线程 CPU 绑定；可传 CPU 编号或 `none`，需保证编号在允许 CPU 集合内 |
| `--rt-prio` | `50` | 尝试 SCHED_FIFO；无权限时记录失败。本机该提示下仍测得约 500 Hz，不等于已经获得实时调度保证 |
| `--wait-state-s` | `5.0` s | 启动等待第一条 lowstate 的时长；先确认 A 已启动，再考虑等待时间 |
| `--stale-ms` | `20.0` ms | 已在运行时的 lowstate 超时保护；与启动等待不是一回事 |
| `--pose-max-age` | `0.1` s | 外部位姿时效上限；增大它会接受更旧位姿，不能解决 UDP 积压 |
| `--max-overrun` | `2.0` 个策略周期 | 控制循环超时保护；命令心跳上限为 `(2.0 + 0.5) × 0.02 = 0.05` s，即 50 ms |
| `--max-temp` / `--warn-temp` | `80` / `70` °C | 超温阻尼与接管/待机检查阈值，沿用原配置，非 H1 实测标定结论 |

`--stage` 只选择以下保护档位，不会切换 KP/KD，也不会自动挂上或释放吊带：

| 档位 | 最大速度占关节速度上限比例 | 最大原始动作幅值 | 策略目标力矩限幅因子 | 固定 PD 阶段最大倾斜 |
|---|---:|---:|---:|---:|
| `air` / `feet` | 0.3 | 10 | 0.5 | 0.35 rad，约 20° |
| `slack` | 0.5 | 15 | 1.0 | 0.5 rad，约 29° |
| `ground` | 0.8 | 50 | 1.5 | 0.8 rad，约 46° |

对应命令行覆盖项为 `--max-vel-frac`、`--max-abs-action`、`--torque-clamp-factor`、`--max-tilt-hold`。最后一个单位是 **rad**，与带 `-deg` 的检查参数不同。策略阶段另有 `SafetyCfg.max_tilt_err = 0.35 rad` 的参考倾斜误差检查，不能用上表固定 PD 倾斜阈值替代。

仿真器实际提交的关节力矩仍受机器人执行器上限裁剪；`--torque-clamp-factor 1.5` 不代表仿真电机能输出 1.5 倍额定上限。END_HOLD 的目标限幅还使用 `min(1.0, torque_clamp_factor)`。

本节保持 `--pose-max-age`、`--max-overrun` 等保护默认值。当前已定位的旧位姿和首次 FK 超时问题，不以放宽这些阈值作为“通过”的依据。

#### 6.4 吊带时机：有效策略指令后渐进释放，动作结束恢复

自动吊带仅用于 `--sim` 且设置了 `--sim-band-file` 的 DDS 仿真，不能控制实机吊装设备。实际启停顺序如下；模型初始化、位姿读取、推理及控制状态的先后顺序没有调整：

| 阶段/事件 | 吊带行为 |
|---|---|
| PASSIVE / INTERP / HOLD | 保持支撑；即使等待接管条件，也不提前释放 |
| 刚进入 STANDBY_PRE | 保持支撑，按原顺序初始化策略、读取位姿、进行首次推理 |
| 推理通过检查且控制指令提交成功 | 开始默认 5 秒的渐进释放；发送失败不启动释放，后续成功后再启动 |
| STANDBY_PRE 释放期间 | 弹簧和阻尼强度一起按平滑曲线从 100% 降到 0%；不累计 READY 稳定性/温升窗口 |
| STANDBY_PRE 完成释放 / POLICY | 策略目标渐变也完成后，再收集无支撑的待机数据；Start 仍需满足原检查 |
| 动作结束进入 STANDBY_POST | 在切换状态的同一控制循环恢复支撑；末帧策略推理继续，但不会重新释放 |
| 进入 END_HOLD 固定 PD 过渡 | 恢复支撑，直到下一次 STANDBY_PRE 的有效策略指令提交成功 |
| 推理异常、保护触发、急停进入 DAMP | 恢复或保持支撑；与释放操作串行化，避免并发急停后又被放绳 |
| 正常退出控制器 | 保留支撑文件，仿真器停止后再人工清理 |

这里的“提交成功”：Python runner 是 DDS 写入接口返回成功；bridge runner 是向 bridge 提交成功，不是等待机器人或仿真器确认执行。B 原子写入释放开始的单调时钟时间和时长，A 在每个物理步计算支撑比例，避免按控制周期阶跃撤掉支撑。DAMP/动作结束写回完整支撑，同时取消未完成的释放。“立即恢复”指立即启用弹力绳，不能保证机器人瞬间升到某个高度。

具体采用 `scale = 1 - (3u² - 2u³)`，`u = clip(已释放时间 / 释放时长, 0, 1)`，将原弹簧力、阻尼力及其力矩同时乘以 scale。这是**逐渐撤掉仿真支撑**，没有移动上方吊点，也没有模拟真实卷扬机的收放绳机构。

重新启用完整支撑时（包括动作结束、DAMP、取消尚未完成的释放），A 会将上方吊点的 **XY 一次性移到当前颈部上方**，Z 高度和自然长度保持原值；持续支撑期间不跟随机器人移动。这样消除旧吊点位置导致的初始横向弹簧拉力，但原阻尼力和后续偏移产生的拉力仍存在，不能理解为完全没有水平力或垂直拉力。A 输出 `band ON: anchor XY moved above current neck...`，日志增加 `band_anchor`。

现有 A、B、C 命令不需要增加新开关，只要 A 的 `--band-file` 与 B 的 `--sim-band-file` 一致。需要结束当前 A、B 并重新启动才会使用修改后的逻辑。

| 参数 | 默认值 | 当前行为 |
|---|---|---|
| `--sim-band-ramp-s` | `5.0` s | 终端 B：从完整支撑降到零支撑的时长；可设 `8` 或 `10` 更慢；`0` 恢复原立即释放行为 |
| `--sim-band-release-s` | `-1.0` | 保留兼容参数名；非负值会在启动时拒绝，不能再要求 HOLD 中提前放绳 |
| `--sim-band-release-standby-s` | `-1.0` | 默认首次有效策略指令提交成功后开始渐进释放；非负值可额外延迟启动释放，例如 `5` 表示进入 STANDBY_PRE 至少 5 秒后才启动 |
| `--sim-after-band-s` | `1.0` | Start 播放前，吊带必须已完全释放至少这么久；稳定性窗口仍需单独满足，默认 3 秒 |
| `--sim-no-band-end-hold` | 关闭 | 启用会在启动时拒绝，不能再要求固定 PD 过渡或动作结束后不挂绳 |

如原命令含 `--sim-band-release-s 0` 等非负配置或 `--sim-no-band-end-hold`，应移除。这两种旧设置与当前指定的吊带时序冲突，程序不会静默忽略。

终端 B 的现有命令可直接追加 `--sim-band-ramp-s 5`；终端 A 不需要额外参数，但 **A、B 都必须重启，才能使用新的文件协议**。设为 8 秒时，即使策略目标在第 5 秒已经完全接管，也要等第 8 秒支撑降为零，再累计待机窗口。

正常输出顺序：`SIM: elastic band RELEASING over 5 s` → `BLEND: complete` / `SIM: elastic band OFF (smooth release complete...)` → 无支撑稳定性检查 → `READY`。两个完成事件的先后由各自时长决定。终端 A 的 `support` 显示当前强度比例。

修改位置：两个 runner 的 `release_band()`、`sim_band_on()`、`standby_feedback_ready()`、`standby_allowed()`；公共文件协议和曲线在 `src/h1_deploy/deploy/sim_support.py`；仿真施力在 `scripts/deploy/sim_h1_mujoco.py` 的 `ElasticBand.apply()` 和主循环。

2026-11-15 验证：独立 lo / DDS 域 146，ckpt_latest、default、ground，仿真按 default 初始化、吊点额外抬高 0.18 m。5 秒释放结束后约 3 秒出现 READY，随后完整播放 default 并恢复支撑，未触发倾角中止。两个 runner 的渐减、释放期间禁止 READY、DAMP 取消释放、0 秒兼容行为已做离线检查；C++ bridge 未重复整段 DDS 播放。测试文件和日志已清理。该结果不代表所有动作或所有初始吊点高度都已验证。

HOLD 的关节误差、速度和倾斜检查保持不变；持续输出关节误差 WAIT 时仍停留在固定 PD，吊带保持启用。本次改变释放过程，不能单独解决固定 PD 的姿态误差或此前的位姿缓存、首次 FK 超时问题。实机依然需要人工协调吊装与接管，不能把这些文件开关理解为实机吊绳控制。

也不启用 `--auto-sequence`、`--end-hold-legacy`、NaN/主循环停顿故障注入来代替本节手动选择与 Start 流程。

#### 6.5 每次动作准备时的坐标对齐和位置误差报告

两个 runner 在 `--sim` 下接收 MuJoCo 位姿时，使用 `anchor.py::SimPoseReceiver`，复用实机 `unitree-odom` 的 `ReferenceAlignment`。R1+B 检查通过后只重置所选参考；仍在原 STANDBY_PRE 首次位姿读取时建立固定 XY/yaw 变换。Start、播放和 STANDBY_POST 均沿用该变换，只有下一次准备动作才重建。真实的高度、横滚、俯仰以及对齐后的位移均保留；原 UDP 时效检查保持生效。模型、参考 NPZ 和物理场景坐标不修改。

**R1+B 后的小位移会形成跟踪误差。** 策略继续接收该误差；Start 不会把它归零。READY 检查本次没有新增位置门槛，READY 不等于骨盆位置误差为零。

原命令可直接使用，重启 A、B 后生效。B 每轮准备打印一次 `POSE_ALIGN`；待机和播放按 `--status-s`（默认 3 秒）输出：

```text
pelvis err XYZ=[+0.030, -0.040, -0.020] m (actual-ref), XY=0.050 m
```

符号为 **实际位置减参考位置**，坐标位于送入策略的参考世界系，XY 是水平误差长度；上例表示水平误差 5 cm、比参考低 2 cm。比较的是本周期推进前的当前参考帧，不是 ckpt_latest 的未来躯干锚点特征。PRE/POST 比较各自的首帧/末帧静态参考。

控制器 NPZ 新增 `root_ref_pos`、`root_pos_error`、`root_ref_frame`、`root_ref_time_s`、`root_tracking_state`。最后一项保存实际执行推理的阶段，避免动作完成当周期已经转成 POST 后，把最后一个播放样本归错组。非策略状态或没有有效根位置时，误差为 NaN。报告不依赖重新读取可能已被更换的动作文件。

退出后的 `[summary] pelvis` 按动作和连续控制阶段列出 XYZ 平均误差、XY 平均/最大误差、Z 最小/最大误差，以及最大水平误差对应的参考帧与运行时刻。历史日志缺少这些新字段时跳过此新增统计；不会回写历史日志。公共统计代码位于 `src/h1_deploy/deploy/tracking_report.py`。

本轮验证：独立 lo / 域 147，Python runner、ckpt_latest、ground、5 秒放绳、初始吊点偏移 0.18 m，完成 default → dance_smooth 的接管、播放和恢复支撑；仅出现两次 POSE_ALIGN，各对应一次准备，Start 未重新对齐。误差记录逐项满足 `root_pos_error = root_pos - root_ref_pos`，吊点 Z 全程保持不变。dance_smooth 仍在参考 9.06 秒出现最大水平误差约 0.607 m，因此**接口与时序验证完成，不代表该动作已经稳定跟踪**。对齐、小位移保留和报告统计另做了离线检查；bridge runner 共用这些逻辑，未再执行完整 C++ DDS 播放。本轮测试文件、日志及进程均已清理。

### 7. KP/KD、摩擦和吊带参数在哪里修改

#### 7.1 先区分固定 PD 与策略控制的增益来源

| 阶段 | 增益来源 | 修改位置 |
|---|---|---|
| INTERP / HOLD | `self.stand_kp`、`self.stand_kd` | [h1_runner.py](../scripts/deploy/h1_runner.py) 顶部 `STAND_KP`；KD 默认复制当前 `deploy_config.json` 的 `kd`，手臂和腰部可显式命令行覆盖 |
| STANDBY_PRE / POLICY / STANDBY_POST | `cfg.kp`、`cfg.kd` | 实际 `--artifacts` 目录中的 [deploy_config.json](../deploy_artifacts/ckpt_latest/deploy_config.json) 的 `kp`、`kd` |
| END_HOLD | 从策略增益渐变到固定 PD 增益 | runner 的 `run()` / END_HOLD 分支，时长由 `--end-blend-s` 控制 |
| DAMP | KP 为 0，KD 为 `DAMP_KD` | runner 顶部常量，当前为 3.0；不是本次平衡调参对象 |

当前固定 PD 每条腿按 **hip_yaw、hip_roll、hip_pitch、knee、ankle** 的顺序为：

```text
KP = [150, 150, 150, 200, 40]
KD = [  2,   2,   2,   4,  2]
```

腰部固定 PD 的 KP=300，每条手臂 KP=[100,100,100,50]；KD 默认读取当前部署配置，`last` 的腰部为 3、手臂为 2。可在 B 的启动命令中显式用 `--stand-arm-kd`、`--stand-waist-kd` 覆盖这两组固定 PD 阻尼，它们不改变策略阶段 JSON 的 KD。

当前 checkpoint 的 KP 数值恰好与上述固定 PD KP 相同，但 KP 的配置来源仍不同。KD 则默认来自同一份 JSON；有显式固定 PD 覆盖参数时，对应关节才不同。上方腿部 KD 数值也来自当前 JSON，不再是 runner 中的固定常量。

KP/KD 调整建议：

1. 先根据日志确定偏差发生在固定 PD 还是策略阶段，以及具体关节。修改 JSON 的 KD 会影响两阶段默认值；只测试固定 PD 手臂/腰部时，可使用专用覆盖参数。
2. 固定 PD 持续姿态误差先检查初始高度、足底接触、支撑和目标姿态。增大 KP 可能减小静差，也可能增大冲击、震荡和力矩饱和。
3. 明显震荡时再小步比较 KD；更大的 KD 也可能带来迟滞和更大制动力矩，并非越大越好。
4. 策略按训练时的增益学习，改 JSON 后属于新的执行条件，需重新验证待机和动作。`action_scale`、关节顺序、默认姿态不能当作普通 PD 增益一起调整。
5. 如要比较 JSON 增益，复制一份 checkpoint 产物目录再修改，`--artifacts` 指向副本。当前 JSON 的 `policy_checkpoint` 是相对路径，副本目录深度变化时必须同步核对模型路径。

固定 PD 的腿部 KP 没有通用的命令行倍率开关，需要修改源码常量；腿部 KD 默认跟随当前部署 JSON。Python 与 [bridge Runner](../scripts/deploy/bridge/h1_runner_bridge.py) 均已采用上述 KD 读取方式。

本节说明的是 DDS 联调的两个 Runner。批量评估脚本 `scripts/deploy/sim2sim.py::stand_gains()` 仍有独立的固定 PD 配方，本次未修改；此前 G3/standby 的结果不能直接视为已评估本次新的默认 KD。

#### 7.2 模型、地面与吊带

| 内容 | 位置 | 说明 |
|---|---|---|
| 默认 H1 机器人资产 | [h1.py](../src/h1_deploy/robots/h1.py) 的 `H1_XML`，指向 [h1_training.xml](../h1_data/h1_training.xml) | 模型质量、惯性、碰撞几何等来自资产 |
| 默认地面摩擦 | [models.py](../src/h1_deploy/deploy/models.py) 的 `scene_xml()`，`floor` 的 `friction` | 当前为 `1 0.005 0.0001`，依次是滑动、扭转、滚动摩擦系数 |
| 自定义场景 | A 的 `--scene /path/to/scene.xml` | 使用完整场景；DDS 仿真需要匹配的 19 个执行器和三个 IMU 传感器，不能随意用位置执行器场景替换 |
| 机器人侧颈部吊点 | [sim_h1_mujoco.py](../scripts/deploy/sim_h1_mujoco.py) 的 `BAND_BODY`、`BAND_POINT_LOCAL` | 刚体为 `torso_link`，局部坐标默认 `[0, 0, 0.50]` m |
| 吊带弹簧与阻尼 | 同文件的 `ElasticBand.apply()` | 弹簧系数 2000、速度阻尼 200；阻尼使用颈部吊点线速度，目前没有对应命令行参数 |
| 上方悬挂点 | 同文件的 `ElasticBand.anchor / reanchor_xy()` | 初始颈部吊点世界位置加 `[0, 0, band_length + band_anchor_offset_z]`；每次重新启用完整支撑时，XY 移到当前颈部上方，Z 保持初始值；支撑期间不持续跟随 |
| 仿真力矩上限 | [h1.py](../src/h1_deploy/robots/h1.py) 的 `effort_limit` 及模型执行器 | 与控制器 JSON 中目标限幅的 `effort_limit` 需区分，两边都存在限幅 |

`--band-length` 同时影响初始悬挂点和自然长度，仅修改它不能直接等价为“增加预拉力”。独立抬高上方悬挂点应使用 `--band-anchor-offset-z`，保持自然长度不变；参数、状态输出及离线对照见第 6.1 节。修改高度、摩擦、质量或支撑都会改变仿真条件，应保留一轮未改参数的对照结果。

**颈部吊点如何定义和微调：** 此 H1 模型没有独立的 neck 刚体，肩关节位于 `torso_link` 局部高度约 0.43 m，因此用局部 `[0, 0, 0.50]` m 作为颈根附近的吊点。该坐标随躯干的平移和转动变换到世界坐标，不是世界坐标中固定的 0.50 m 高度。

如需微调位置，在 `sim_h1_mujoco.py` 中修改：

```python
BAND_BODY = 'torso_link'
BAND_POINT_LOCAL = np.array([0., 0., 0.50])
```

三个数依次为躯干局部坐标的 X、Y、Z，单位米；可微调 Z 选择颈根高度，X/Y 用来对齐模型中的实际吊点。修改后结束当前 A、B，重新启动 A，再启动 B，避免继续使用旧进程或旧物理状态。

`ElasticBand.apply()` 根据该点的世界位置和线速度计算弹簧/阻尼力。MuJoCo 的 `xfrc_applied` 作用在刚体质心，因此还施加 `(吊点位置 - 躯干质心位置) × 吊带力` 的力矩，使其等价于在颈部施力。只把骨盆刚体名换成躯干、却不补这个力矩，并不能正确表示颈部吊点。

吊点、高度参数和状态输出的修改保持原弹簧/阻尼系数。随后按用户要求调整的支撑启停时机见第 6.4 节；控制阶段、位姿读取和 FK 初始化顺序保持原样。这些修改不会改变 `sim2sim.py` 批量评估中各实验自己的支撑模型。

DDS 仿真器没有 `--latency-ms`、`--g2`、`--standby-kps` 这些评估参数。需要系统扫描延迟、Kp、质量、摩擦时，使用本文前面的批量评估或 [评估详解](SIM2SIM_EVALUATION_GUIDE_CN.md)，不要把那些参数直接传给 DDS 脚本。

### 8. 不启用三动作模式时的原操作，以及 bridge

本节前面的 L1+A/X/Y、Start 仅在 `--motion-select` 下生效。若要核对仓库原来的手动操作，在 B 的命令中去掉 `--motion-select`，再使用下表；需要策略时也去掉 `--no-policy`。

| 原按键命令 | 原行为 |
|---|---|
| `press 'L2+up'` | PASSIVE→INTERP→HOLD |
| `press 'R1+A'` | HOLD 检查通过后进入 STANDBY_PRE |
| `press 'R1+X'` | 待机检查通过后开始 POLICY；不是单独 Start |
| 动作结束 | 默认进入 STANDBY_POST |
| STANDBY_POST 中两次 `press 'L2+up'`，间隔不超过 `--confirm-s`（默认 3 s） | 在有下一动作时，经确认退出策略并按 `--clip` 列表顺序准备下一动作 |
| `press 'L2+B'` | DAMP |

原模式在 STANDBY_PRE 中两次 L2+up 是返回当前动作的固定 PD 准备，不是直接选任意动作。不要混用两种模式的按键。

已编译好 C++ bridge 时，可将终端 B 的脚本名换为 `scripts/run_sim2real_bridge.sh`，A、C 不变。bridge 还保留温升预测：`--temp-rate-min-s` 默认 10 秒，因此 STANDBY_PRE 达到 3 秒也不一定 READY。本节约 500.23 Hz 的实测来自 Python runner，不是对 bridge 的实测结论。构建与接口见 [SIM2REAL.md](SIM2REAL.md)。

### 9. 遇到错误时怎样判断

| 现象/日志 | 含义与先检查的项目 |
|---|---|
| `crc_amd64.so: cannot open shared object file` | SDK 普通打包可能漏掉 native library；先运行第 2 节的实际 CRC 检查，当前本机已修复 |
| `Invalid name 'imu_quat'` | 场景缺少 IMU 传感器；默认场景已修复，若自带 `--scene` 还需检查该 XML |
| `selected interface "lo" is not multicast-capable: disabling multicast` | 本机回环的提示，单独出现不代表失败；本次该提示下 DDS 已正常收发 |
| A 一直 `commands=0`，B 尚未启动 | 正常等待；先启动 B |
| `no rt/lowstate within ...` | 检查 A 是否真正运行、两端 domain/iface 是否相同、DDS 是否初始化成功 |
| `another publisher` / `foreign rt/lowcmd publisher` | 同域已有其他控制器；结束自己上一轮进程或两端一起换独立域，不关闭检测来掩盖冲突 |
| `press: command not found` | 当前终端 C 没有定义函数，重新执行第 3.3 节 |
| 按键没有反应 | 检查文件路径、`+` 分隔符、模式是否为 `--motion-select`、程序是否启动完成，以及是否松开后重按 |
| HOLD 中 `--no-policy` | 第一轮有意禁止策略，属于预期；完整播放需结束该轮、重启后去掉该参数 |
| HOLD 中 `max\|q - f0\| ... >= 0.1` | 固定 PD 尚未满足位置门槛，不是程序卡死；看报错关节、初始条件、支撑和固定 PD 参数 |
| STANDBY_PRE 中拒绝 Start | 看倾斜、关节速度、待机窗口、温度和吊带释放条件；bridge 还检查温升预测 |
| `anchor pose stale or clock mismatch` | 检查 A 的位姿输出、端口和时间戳；当前原流程首次读取还存在缓存积压问题，单纯调大 `--pose-max-age` 不是修复 |
| `main loop stalled ... limit 50 ms` | 计算或调度超过命令心跳时限；本机已观察到首次 FK 初始化约 100 ms，不通过调大 `--max-overrun` 算作通过 |
| `cmd_sched_err: PermissionError` | 未获得 SCHED_FIFO 权限；结合实际低层发送间隔判断，不能仅凭该提示判定 DDS 不通 |
| 窗口关闭后段错误 | 本机曾出现；先用不带 `--viewer` 的方式验证收发/正常退出，保留窗口问题单独排查 |

早期版本中，`walk` 参考末位置比起点前移约 **2.05 m**，切回原点附近的 `default` 时缺少自动平移/航向对齐；固定吊点也会回拉机器人。2026-11-15 已加入第 6.5 节的每轮固定 XY/yaw 对齐，以及第 6.4 节的重新挂绳时吊点 XY 重定位。它们解决坐标起点和旧吊点拉拽问题；具体动作的自由站立、位移跟踪仍需看本轮误差报告验证。

在“内部读取和初始化时机不改变”的约束下，本节保留上述未解决项，不把改阈值、跳过检查或临时改变时机后的结果算作原程序通过。默认独立场景来自 H1 资产重建，DDS 联调通过也不能代替训练场景数值一致性或实机稳定性验证。
