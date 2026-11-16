此目录包含由用户提供的 47 个 H1 动作生成的部署 NPZ：50 Hz、19 关节、48 维逐帧目标，以及 anchor 需要的参考根位姿。

原始训练格式在 `data/h1/`，对应关系与校验值在根目录 `data_manifest.json`。Sim2Sim/Sim2Real 的 `--clip` 使用本目录文件。可在仓库根目录执行 `python -m h1_deploy.deploy.export --cmd clips` 重新生成。
