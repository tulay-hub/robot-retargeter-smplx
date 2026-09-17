# 数据范围 / Data Scope

## 中文

本 Public 仓库包含完整的 robot retargeting 框架、代码、配置、机器人 URDF/MJCF、网格、body-model 资产和小型示例数据。

`dataset/bones_g1_origin/` 是本机约 49G 的 G1 原始动作组，包含约 142,222 个文件。该动作组暂不上传到 GitHub，以避免仓库过大和重复占用动作数据存储；它不影响框架代码、配置和小型示例的部署。需要复现实验时，请把该目录恢复到 `dataset/bones_g1_origin/`，然后按照 `config/` 与 `scripts/` 中的示例传入显式数据路径。

其余 `dataset/ACCAD`、`dataset/bones_g1` 和 `dataset/lafan1_g1` 内容保留在仓库中。所有动作转换仍需检查输入来源、FPS、坐标系、四元数顺序、joint order、关节限位、接触和输出 schema。

## English

This Public repository includes the complete robot-retargeting framework, source code, configuration, robot URDF/MJCF assets, meshes, body-model assets, and small example datasets.

`dataset/bones_g1_origin/` is a local G1 raw-motion collection of about 49 GB and roughly 142,222 files. It is intentionally held out of GitHub for now because of repository size and duplicate motion-data storage. The framework and its deployment do not depend on that collection being present. To reproduce experiments that use it, restore the directory at `dataset/bones_g1_origin/` and pass explicit data paths following the examples under `config/` and `scripts/`.

The smaller `dataset/ACCAD`, `dataset/bones_g1`, and `dataset/lafan1_g1` groups remain included. Every converted motion must still be checked for source provenance, FPS, coordinate frame, quaternion order, joint order, joint limits, contacts, and output schema.
