# calibri_nano

## 全自动眼在手外标定

- `T_base_camera` 将相机坐标转换到机器人基座坐标，平移单位为米。
  求解使用 `inv(T_base_flange_i) * T_base_camera * T_camera_board_i`
  在法兰坐标系下保持不变的约束，输出不再额外求逆。
- 默认仍生成 `3^6 = 729` 个候选，保留可达且路径连通的采集目标；
  按关节路径排序、检查相邻关节跳变，必要的过渡点不采图。
  全自动流程不使用半自动的 90 个选点上限。
- 采集到位后等待 1200 ms，丢弃两帧，每个位置最多尝试三次采图。
  每帧至少 12 个角点；求解至少 12 帧，MAD 过滤后还须保留至少 60%。
- 求解复用五算法共识与代表解精化。最终平移 RMS/P95 门限为 5/8 mm，
  旋转 RMS/P95 门限为 1/2 度；至少三个算法成功，算法间最大差异不超过
  10 mm 和 1 度。这些是程序验收门限，不代表实际定位精度保证。
- GUI 必须加载有效的 `param/eye_to_hand_config.yaml`。
  生产结果保存失败会报告失败，不会显示标定成功。
- 每次采集保存到 `eye_to_hand/run_<时间戳>_<编号>/`，GUI 显示本次目录，
  历史采集不再被清空。中断采集保留 `.collection_incomplete` 标记，拒绝自动复算；
  完整数据严格检查 CSV 行与编号图像的一一对应关系。

无硬件回归测试：构建 `test_eye_to_hand_solver` 后运行
`ctest --test-dir build_mvs --output-on-failure`。测试使用合成位姿，不连接设备。
旧版本输出的眼在手外矩阵方向可能与键名相反，应重新标定或复算原始数据，
不要将旧结果与修复后的结果混用。

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
