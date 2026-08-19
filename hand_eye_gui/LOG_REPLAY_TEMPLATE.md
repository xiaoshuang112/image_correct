# 可复盘日志字段模板（统一规范）

日志系统统一后，所有事件日志都应采用 `APP_LOG_EVENT_*` 宏：

`APP_LOG_EVENT_<LEVEL>("category", "event", "result", { {key, value}, ... })`

最终落盘文本采用如下格式：

`YYYY-MM-DDThh:mm:ss... [LEVEL] [pid=... tid=... ] [run=<session_id> seq=<seq>] [category] file:line function | event=<event> result=<result> key=value ...`

说明：

- `run`：同一启动周期会话号（`DiagnosticLogger::sessionId()`），用于跨文件串联一次执行。
- `seq`：全局自增序号，便于按顺序恢复现场。
- `category`：一级域（如 `camera.calibration`、`calibration.pose_search`）。
- `event`：动作名（如 `camera_pipeline`、`candidate_search`、`collection`）。
- `result`：状态，建议统一使用：
  - `start` / `success` / `fail` / `skip` / `warn` / `cancel` / `progress`
- 键值对字段：`key=value`，由事件上下文补充。
- 约定字段值需避免空格和引号（日志器会做清洗）。

建议的字段规范（建议优先使用）：

| key | 含义 |
| --- | --- |
| `phase` | 阶段名（例如 `initial_optimizer`、`selection`、`restore_position`） |
| `reason` | 失败/跳过/警告原因（例如 `insufficient_frames`） |
| `stage` | 业务子阶段（例如 `z_values`、`image_discovery`） |
| `elapsed_ms` | 耗时，单位毫秒 |
| `selected_frames` / `input_frames` / `valid_frames` | 帧数统计 |
| `path` / `file` / `directory` | 文件路径 |
| `error` | 错误码或错误码字符串 |
| `index` | 当前序号（0-based 或 1-based，按调用方约定写明） |
| `corners` | 检测到的角点数 |
| `result` 字段外不再重复放“成功/失败/跳过”标签，以免语义重复 |

核心分类与常用事件（可用于离线复盘脚本聚类）：

- `camera.calibration`
  - `camera_input`：图像扫描/检测过程
  - `camera_intrinsics_initial`：初次内参优化
  - `camera_intrinsics_refine`：代表帧精化优化
  - `camera_intrinsics_validation`：全量复核结果
  - `camera_pipeline`：整条内参流水线
- `calibration.collection`
  - `collection`：采集主流程（start/captured/success/skip/fail）
- `calibration.pose_search`
  - `pose_evaluation` / `best_pose` / `candidate_search` / `xy_search`
- `calibration.height`
  - `height_estimate`
- `calibration.output`
  - `yaml_output`
- `preview`、`robot`、`controller`、`config`、`filesystem` 等按模块记录界面和外设状态。

排障建议（最小字段集）：

- 首先按 `seq` 排序；再按 `run` 分组。
- 聚焦同一 `category+event`：
  1. 看 `result="start"` 到 `result` 变为 `success/fail/skip` 的时间区间；
  2. 查看同组 `elapsed_ms`、`selected_frames`、`valid_frames`、`reason`；
  3. 对应 `sessionId` 下若出现 `ERROR/WARN`，追溯前后 `progress` 日志。

本文件对应的日志行为与当前代码中的模板保持一致，避免离线脚本解析字段错位。
