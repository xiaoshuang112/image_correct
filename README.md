# calibri_nano

## 标定审计部署边界

- 掉电持久性保证当前仅适用于 Linux/Jetson。Windows 构建尚未实现等价的
  `FlushFileBuffers`、目录项持久化和覆盖已有目标文件的原子发布语义，不能按
  Phase 1.1 的掉电安全标准验收。
- 标定结果发布采用单写者模型。同一生产 YAML 路径同一时间只允许一个 GUI、
  CLI 或离线 Replay 进程写入；当前版本不提供跨进程文件锁。
- 已封存 Session 是不可变审计数据。Replay 输出必须位于源 Session 目录之外，
  工具会在复算前解析相对路径、`..` 和符号链接并拒绝越界输出。

## Phase 3 LOO 稳定性诊断

LOO 是显式离线深诊断，不在在线标定或普通 Replay 中默认执行，避免重复求解
影响产线节拍。对封存 Session 执行：

```text
offline_eye_in_hand --replay-session <session_dir> <output.yaml> --loo
```

也可对普通离线数据集执行：

```text
offline_eye_in_hand <calibration_dir> <camera_params.yaml> <output.yaml> --loo
```

报告中的 `hand_eye_loo_diagnostics` 同时包含：

- `fixed_final_inliers`：只从最终内点集删除一帧，锁定完整结果选中的 solver，
  不重新运行 MAD，用于隔离单帧影响；
- `full_pipeline`：删除一帧后重新运行 MAD、五算法共识、SE(3) medoid 和精化，
  同时记录最终内点集合相对完整结果的新增和移除项。

两种模式均输出平移和旋转影响的 mean、P95、max、最高影响帧及失败次数。
深诊断最多使用 4 个并行 worker，结果按原始帧顺序持久化；每 25 次试验保存一次
进度。计算量随帧数明显增长，建议使用产线标准的约 90 帧数据集执行，超大历史
数据集应预留更长的离线分析时间。
