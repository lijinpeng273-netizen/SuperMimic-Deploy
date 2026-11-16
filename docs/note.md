新的csv：

conda activate supermimic
cd /home/ljp/human_learn/supermimic/H1_SuperMimic_Deploy

python -B - <<'PY'
from pathlib import Path
from h1_deploy.data.motion_convert import convert_csv_to_npz
from h1_deploy.deploy.export import export_clip
from h1_deploy.robots.h1 import load_h1_model

model = load_h1_model()

for name in ("dance1_2x_flatfeet",):
    source = Path("data/use") / f"{name}.csv"
    motion = Path("data/use") / f"{name}.npz"
    deploy = Path("deploy_artifacts/use") / f"{name}.npz"

    info = convert_csv_to_npz(
        source, motion,
        input_fps=30,
        output_fps=50,
        model=model,
    )
    export_clip(motion, deploy)
    print(info)
    print("部署文件：", deploy)
PY

新改变的npz文件适配方式：

例如新文件叫 `stand.npz`、`walk3.npz`、`dance3.npz`，就写：

```
--clip deploy_artifacts/use/stand.npz \
       deploy_artifacts/use/walk3.npz \
       deploy_artifacts/use/dance3.npz
```

对应关系：

- **L1+A**：第一个文件 `stand.npz`
- **L1+X**：第二个文件 `walk3.npz`
- **L1+Y**：第三个文件 `dance3.npz`



现在使用 两个终端：终端 A 提供里程计位姿，终端 B 运行 last 策略和电机通信。两个终端必须在同一台部署电脑上运行。

以下保留你之前的 --stage air 参数。软件接入已验证，但实机动态动作还未验证；首次先检查 default 的策略待机。

1．终端 A：启动里程计接收，保持运行

conda activate supermimic
cd /home/ljp/human_learn/supermimic/H1_SuperMimic_Deploy

./scripts/deploy/bridge/h1_odom_bridge \
  --iface enp12s0 \
  --domain 0 \
  --udp 127.0.0.1:15101

正常会周期性输出：

[odom] READ ONLY ...
[odom] paired=... z=... waist=... pair=... ms age=... ms CRC_bad=0

paired 应持续增加。这里的 z 是补偿前的里程计高度，约 0.884 m 属于你此前的测量结果。

实际部署时，不要添加 --check-only 或 --seconds 20，否则不会持续向控制器提供位姿。

2．终端 B：启动策略控制器

机器人需要处于此前确认的调试状态，没有其他控制器发布 rt/lowcmd；固定 PD 准备期间由吊架承担支撑。

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
         deploy_artifacts/use/walk2.npz \
         deploy_artifacts/use/dance_smooth_flatfeet_bolder.npz \
  --pose-listen 127.0.0.1:15101 \
  --pose-source unitree-odom \
  --odom-z-offset 0.075624 \
  --odom-orientation-body torso \
  --pose-max-age 0.1 \
  --log-dir deploy_logs/last_bridge





程序会执行原有的状态、控制器冲突和模式检查。到现场确认提示时，确认吊装及急停条件后输入：

yes

此后开始发送控制命令，初始状态为 PASSIVE 阻尼。

几个关键参数：

| 参数 | 含义 |
|---|---|
| --artifacts deploy_artifacts/last | 使用已适配的 last 策略配置 |
| --motion-select | 开启三个动作选择和 Start 播放 |
| --pose-source unitree-odom | 使用本次接入的里程计，并按动作准备流程进行坐标对齐 |
| --odom-z-offset 0.075624 | 原始高度加上约 7.56 cm；不是把机器人高度固定为 96 cm |
| --odom-orientation-body torso | 按胸部姿态约定，结合腰角转换为骨盆姿态 |
| --pose-max-age 0.1 | 位姿允许的最大年龄为 100 ms |
| --stage air | 沿用原来的空中调试保护参数，不会自动操作实机吊架 |
| --log-dir | 保存本次控制日志的目录 |

高度补偿和 torso 约定仍需在改变机器人姿态时核验，不能只凭静态结果认定所有动作中的定位都正确。

3．用实机手柄选择和播放

不需要仿真中的终端 C，也不用输入 press 命令。

首次按 L1+A 选择 default，正常流程为：

PASSIVE
  ↓ L1+A
INTERP：固定 PD 插值到 default 首帧
  ↓
HOLD：固定 PD 保持，检查关节误差等条件
  ↓ 条件通过
STANDBY_PRE：已经开始策略推理，参考固定在首帧
  ├─ POSE_ALIGN：显示本次位置、朝向对齐结果
  └─ READY：满足播放条件
  ↓ 按 Start
POLICY：播放动作
  ↓ 播放结束
STANDBY_POST：继续策略推理，参考固定在末帧

不用等到 Start 才开始推理：进入 STANDBY_PRE 就已经是策略控制。 实机吊架仍由人工操作。

如果出现 WAIT，根据提示处理未满足的条件；不要通过放宽阈值跳过问题。若进入 DAMP，本次运行不能通过选择键恢复。

播放过程中选择其他动作会排队，当前动作结束后再进入原有固定 PD 准备流程。新动作准备完成后，需要再次按 Start。

4．结束运行

先让吊架可靠承担机器人重量，再按 L2+B 进入阻尼。随后在终端 B 按 Ctrl+C，等待控制器完成退出；最后在终端 A 按 Ctrl+C。

不要先关闭终端 A，否则策略阶段会因为定位失效进入 DAMP。

完整说明和参数修改位置见 SIM2REAL.md (human_learn/supermimic/H1_SuperMimic_Deploy/docs/SIM2REAL.md:247)。

当前按键功能如下：

| 手柄按键 | 功能 |
|---|---|
| L1+A | 选择 default，开始准备 |
| L1+X | 选择 walk，开始准备或排队 |
| L1+Y | 选择 dance，开始准备或排队 |
| Start | READY 后播放当前动作；提前按下不会排队 |
| L2+B | 进入 DAMP 阻尼 |

机器人退出AI的模式：

```
conda activate supermimic

python -B - <<'PY'
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

ChannelFactoryInitialize(0, "enp12s0")

client = MotionSwitcherClient()
client.SetTimeout(5.0)
client.Init()

code, info = client.CheckMode()
print("退出前：", code, info)

if code != 0:
    raise SystemExit("模式查询失败，未执行退出。")

if info.get("name"):
    code, _ = client.ReleaseMode()
    print("ReleaseMode 返回码：", code)
    if code != 0:
        raise SystemExit("退出请求失败，请检查返回码。")
    time.sleep(1.0)
else:
    print("当前已经没有活动的官方运动模式。")

code, info = client.CheckMode()
print("退出后：", code, info)
PY
```

退出成功后，通常显示：

```
退出后： 0 {'form': '0', 'name': ''}
```



