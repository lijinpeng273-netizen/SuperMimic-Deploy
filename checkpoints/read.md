我读取了两个检查点及部署代码，确认：ckpt_latest.pt 来自 SuperMimic，last.pt 来自 BlinkMimic 的 H1 版本。它们采用不同的策略输入和动作生成方式。

| 对比项 | ckpt_latest.pt | last.pt |
|---|---|---|
| 策略来源 | SuperMimic | BlinkMimic |
| 历史观测输入 | 630 维：10 × 63 | 720 维：10 × 72 |
| 参考动作输入 | 489 维 | 530 维 |
| 历史排列 | 按观测项排列 term_major | 按时间帧排列 frame_major |
| 相对参考位姿反馈 | 9 维，放在目标输入中 | 每帧增加 9 维，进入历史观测 |
| 当前部署使用的锚点 | 躯干 torso_link，见下方说明 | 骨盆 pelvis，检查点明确记录 |
| 输入归一化 | 固定比例缩放 | 使用训练中统计的均值、方差 |
| 隐藏层激活函数 | SiLU | ELU |
| 参考关节角直接参与动作输出 | 没有显式直连 | 启用了参考动作直连 pi_ref_skip |
| Actor 参数量 | 约 788 万 | 约 815 万 |
| 保存的推理权重 | policy | pi，使用 EMA 平滑后的权重 |

其中最值得你关注的是下面几个区别。

1. BlinkMimic 明确以参考关节角为基准，网络学习修正量

   两者最终都通过位置目标控制关节，但计算方式不同。这里动作缩放都是 0.25：

   SuperMimic：
目标关节角 = 默认关节角 + 0.25 × 网络输出

当前 BlinkMimic：
目标关节角 = 下一参考帧的关节角 + 0.25 × 网络输出的修正量

   所以，BlinkMimic 的解码网络修正量为零时，目标就落在参考关节角上；SuperMimic 的解码网络需要直接给出相对于默认姿态的动作偏移。

   SuperMimic 同样会利用参考动作，只是通过编码器输入网络，没有这条显式直连。实现分别位于 SuperMimic 策略 (human_learn/supermimic/H1_SuperMimic_Deploy/src/h1_deploy/policy/nets.py:62) 和 BlinkMimic 策略 (human_learn/supermimic/H1_SuperMimic_Deploy/src/h1_deploy/policy/blinkmimic.py:56)。
2. 两者反馈身体偏差的位置不同，不能共用同一套输入拼接

   SuperMimic 的本体观测每帧为 63 维：关节位置、关节速度、角速度、重力方向、上次动作。参考躯干相对实际躯干的 9 维位姿误差另加在目标输入中，因此目标是 480＋9＝489 维。

   BlinkMimic 把参考骨盆相对实际骨盆的 9 维位姿误差加入每帧观测，因此历史输入是：

   (63＋9) × 10 = 720 维

   它的未来参考特征也重新组织成每帧 53 维，共 530 维，包含参考位移、姿态、速度和关节信息。

   因此，last.pt 配置里的 anchor_obs=False 不表示它不需要位置反馈。 两个策略目前都需要与参考轨迹坐标系对齐的骨盆位置。拼接逻辑见 core.py (human_learn/supermimic/H1_SuperMimic_Deploy/src/h1_deploy/deploy/core.py:275)。

   关于锚点还有一个记录上的区别：last.pt 明确保存了 anchor_body="pelvis"；ckpt_latest.pt 没有保存这个字段，当前部署代码按 torso_link 计算。因此，后者的锚点结论来自当前部署适配约定，无法仅凭检查点元数据确认其历史训练设置。
3. 训练方式有所变化，但世界模型都不参与当前部署推理

   检查点记录显示：

   | 训练项 | SuperMimic | BlinkMimic |
   |---|---|---|
   | 世界模型隐藏层 | 2048、2048、2048 | 1024、1024、1024 |
   | 世界模型训练窗口 | 8 步 | 8 步 |
   | 策略想象长度 | 32 步 | 32 步 |
   | 优化器配置 | Adam | Actor、世界模型启用 Muon |
   | 策略 EMA | 未保存此机制 | 开启，系数 0.995 |
   | 世界模型镜像增强 | 当前配置未体现 | 开启，比例 0.5 |

   last.pt 同时保存了 pi 和 pi_live。当前部署加载的是 EMA 平滑后的 pi，pi_live 用于恢复训练。

   两个 PT 都包含世界模型、Critic、优化器等训练状态，当前部署只提取 Actor。所以 last.pt 文件更小，主要不能用来说明推理更快；它的 Actor 参数量实际上还略多。
4. 机器人控制参数相同，但部署配置必须分别选择

   我核对到两者的 H1 资产哈希、19 个关节顺序、默认姿态、PD 增益、力矩上限和动作缩放一致。记录的训练动作名称集合也相同，都是 40 个。

   当前仓库已经为两者提供独立适配，选择时对应使用：

   SuperMimic：--artifacts deploy_artifacts/ckpt_latest
BlinkMimic：--artifacts deploy_artifacts/last

   不要只替换 PT 路径而沿用另一份部署配置，因为输入维度、历史排列、归一化和参考特征都会不匹配。加载分支见 policy.py (human_learn/supermimic/H1_SuperMimic_Deploy/src/h1_deploy/deploy/policy.py:10)。

另外，文件中的 iteration 分别为 58,300 和 20,000，但计数口径不同：SuperMimic 按环境采集步计数，BlinkMimic 按训练外循环计数，不能直接据此判断训练充分程度。谁在实机更稳定，需要在相同动作、起始状态和绳索受力条件下比较完成率与跟踪误差；这次核对确认的是结构和配置差别。
