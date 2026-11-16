# Third-party notices

本部署代码提取自 SuperMimic 复现工程，保留原有 MIT LICENSE 中 Yuxi Liu、Hongyu Ding 的版权声明。修改部分沿用同一代码许可证。

- 上游复现：[CuboiLeo/SuperMimic-Reproduced](https://github.com/CuboiLeo/SuperMimic-Reproduced)。部署扩展来源：[SuperMimic-Repro-Sim2Real](https://github.com/NJU-R-L-Group-Embodied-Lab/SuperMimic-Repro-Sim2Real)。本仓库保留原作者声明，不表示部署功能已获得 H1 验证。
- C++ bridge 的 Crc32Core 保留自 [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2) 示例，Copyright (c) 2016-2023 HangZhou YuShu TECHNOLOGY CO.,LTD.，BSD-3-Clause。完整原文见 [unitree_sdk2-BSD-3-Clause.txt](unitree_sdk2-BSD-3-Clause.txt)。SDK 本身由用户另行安装。
- Python DDS 接口使用 [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)，其依赖及许可由上游提供。
- H1 XML、URDF、网格与资料原样复制自源仓库 h1_data。该目录没有独立资产来源/许可证说明，代码的 MIT 许可不能替代资产许可；分发时应向资产提供方确认并补充相关声明。
- MuJoCo、NumPy、SciPy、PyTorch、ONNX Runtime、tyro、pandas、tqdm 作为外部依赖安装，各自保留原许可证。可选视频工具依赖 FFmpeg；外部视频和动作数据的权利归其所有者。
- 用户 checkpoint 原样复制。训练数据许可及参考轨迹不包含在 checkpoint 中，不能由本仓库代码许可推断模型/数据的分发权利。

本仓库未复制 mjlab/MuJoCo Warp 训练环境，运行器不依赖它们。
