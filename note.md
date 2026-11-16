

# 策略推理 ：
cd /home/ljp/human_learn/supermimic/H1_SuperMimic_Deploy

bash scripts/run_sim2sim.sh \
  --artifacts deploy_artifacts/ckpt_latest \
  --clip data/use/walk.npz \
  --seconds 0 \
  --latency-ms 0 \
  --log outputs/sim2sim/walk1_playback.npz

实际执行的是 scripts/deploy/play_sim.py。

| 参数 | 含义 | 怎么改 |
|---|---|---|
| --artifacts | 策略目录，里面包含 deploy_config.json，以及可选的 policy.onnx | 切换策略时换成对应目录 |
| --clip | 要跟踪的部署格式动作文件 | 换成 deploy_artifacts/clips/ 中其他 .npz |
| --seconds 0 | 播放一个完整动作；达到动作结尾或触发停止条件就结束 | 改成 10 表示最多运行 10 秒仿真时间，不会因此循环动作 |
| --latency-ms 0 | 注入的目标指令延迟，单位毫秒 | 例如改成 20 |
| --log | 保存仿真状态、策略动作和关节目标的文件路径 | 改文件名可以区分实验 |
| --headless，可选 | 关闭窗口 | 在命令末尾追加即可，不需要跟数值 |
| --scene，可选 | 指定 MuJoCo 场景 XML；省略时使用内置场景 | 例如追加 --scene deploy_artifacts/models/h1_torque_scene.xml |

默认值在 play_sim.py:18 (human_learn/supermimic/H1_SuperMimic_Deploy/scripts/deploy/play_sim.py:18) 的 ap.add_argument(...) 中修改。



# DDS 仿真运行实机控制
终端 A：启动仿真器。

cd /home/ljp/human_learn/supermimic/H1_SuperMimic_Deploy

touch /tmp/h1_band

python scripts/deploy/sim_h1_mujoco.py \
  --domain 1 \
  --iface lo \
  --clip deploy_artifacts/clips/walk1_subject1.npz \
  --dt 0.001 \
  --band-file /tmp/h1_band \
  --pose-udp 127.0.0.1:15101 \
  --viewer \
  --log outputs/dds_sim/walk1.npz

| 参数 | 含义 |
|---|---|
| --domain 1 | DDS 通信域。两端必须一致；此仿真器要求非零值 |
| --iface lo | 使用本机回环网卡，仿真器与控制器在同一台电脑通信 |
| --clip | 用动作第一帧初始化机器人位置、姿态和关节角；这里不负责播放动作 |
| --dt 0.001 | 物理仿真步长，单位秒；0.001 对应 1000 Hz |
| --band-file | 支撑标志文件：文件存在时启用弹性支撑，删除后解除 |
| --pose-udp | 向该 UDP 地址发送仿真的 pelvis 位置和姿态，供当前 anchor 策略使用 |
| --viewer | 打开 MuJoCo 窗口；省略则没有窗口 |
| --log | 保存仿真器日志 |

默认值在 sim_h1_mujoco.py:28 (human_learn/supermimic/H1_SuperMimic_Deploy/scripts/deploy/sim_h1_mujoco.py:28) 修改。另可追加 --duration 60，使仿真器在启动后约 60 秒墙钟时间退出；默认 0 表示持续运行。

终端 B：启动控制器。

cd /home/ljp/human_learn/supermimic/H1_SuperMimic_Deploy

bash scripts/run_sim2real.sh \
  --domain 1 \
  --iface lo \
  --sim \
  --stage feet \
  --artifacts deploy_artifacts/ckpt_latest \
  --clip deploy_artifacts/clips/walk1_subject1.npz \
  --sim-band-file /tmp/h1_band \
  --pose-listen 127.0.0.1:15101 \
  --auto-sequence \
  --log-dir outputs/dds_runner

虽然脚本叫 run_sim2real.sh，这条命令通过 --sim 和 --domain 1 连接的是仿真器。

| 参数 | 含义 |
|---|---|
| --domain 1 / --iface lo | 与终端 A 保持一致 |
| --sim | 启用运行器的仿真模式，允许自动流程和仿真辅助功能 |
| --stage feet | 选择 feet 保护参数组，影响速度、动作幅度和力矩目标限幅等；可选 air/feet/slack/ground |
| --artifacts | 策略目录，与第一种模式含义相同 |
| --clip | 控制器实际跟踪的动作；此处应与终端 A 的初始化动作一致 |
| --sim-band-file | 控制器通过创建或删除该文件控制仿真支撑，路径必须与终端 A 一致 |
| --pose-listen | 接收 pelvis 位姿的 UDP 地址，与终端 A 的 --pose-udp 对应；当前策略需要它 |
| --auto-sequence | 自动依次尝试插值、保持、策略待机、动作执行和结束流程；各阶段仍需满足启动条件 |
| --log-dir | 保存控制器运行日志和控制台日志的目录 |

**--stage feet 只选择保护参数，不会自动把机器人吊起来；支撑由 --band-file 配合控制器管理。**默认在进入策略待机时解除支撑。自动流程如果未通过稳定性检查，会停止并记录原因。

这部分参数默认值在 h1_runner.py:1204 (human_learn/supermimic/H1_SuperMimic_Deploy/scripts/deploy/h1_runner.py:1204) 修改。几个常用的附加参数是：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| --interp-s | 4.0 | 插值到动作初始姿态的时间，单位秒 |
| --cmd-hz | 500 | DDS 控制命令发送频率，不是策略推理频率 |
| --auto-standby-s | 3.5 | 进入动作前待机多久后开始尝试启动动作 |
| --pose-max-age | 0.1 | 位姿反馈允许的最大数据年龄，单位秒 |
| --log-obs | 默认关闭 | 追加后保存观测历史，便于检查推理输入 |

———


# 实机部署

cd /home/ljp/human_learn/supermimic/H1_SuperMimic_Deploy

python scripts/deploy/sim2sim.py \
  --artifacts deploy_artifacts \
  --ckpt ckpt_latest \
  --clips walk1_subject1 \
  --cmd g2,g3,report \
  --g2 g2 \
  --workers 1 \
  --tag walk1

| 参数 | 含义 | 怎么改 |
|---|---|---|
| --artifacts | 部署资产总目录 | 注意这里是 deploy_artifacts，不是 deploy_artifacts/ckpt_latest |
| --ckpt | 策略子目录名 | 例如 ckpt_latest；支持空格分隔多个名称 |
| --clips | 动作名称，不带路径和 .npz | 例如 walk1_subject1 dance1_subject1 |
| --cmd g2,g3,report | 运行 G2 扰动评估、G3 过渡评估，并生成报告 | 多项任务用逗号分隔 |
| --g2 g2 | G2 部分只选择名为 g2 的基础变体，仍会扫描延迟、增益、质量和摩擦组合 | 省略此参数会运行全部 G2 变体 |
| --workers 1 | 并行评估进程数 | 可根据 CPU 和内存改成 2、4 等 |
| --tag walk1 | 输出文件名中的标记 | 用来区分不同实验 |




