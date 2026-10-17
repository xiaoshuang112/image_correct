> 历史实施方案：文中的双后端交付要求已废止。当前工程仅支持 MVS，构建和启动方式以 README.md 为准。

# 海康 MVS 版本支持实施方案

## 1. 基线与目标

- 开发分支：`codex/mvs-support`
- 基线提交：`be5b1e2a9ea6e647b68899ffc7e5bd514293534e`
- 参考工程：`/home/nvidia/work/caitu-nano-new`
- 交付产物：独立的 `hand-eye-mvs`，现有 `hand-eye-orb` 行为保持不变。

本轮的目标是把当前已有的 MVS 条件编译代码，从“能够调用 SDK”补齐为可交付、可诊断、不会污染现有 Orbbec 标定链的生产版本。

明确不做：

- 不修改 HandEye solver、MAD、共识门禁和 LOO 算法；
- 不修改 360 个候选姿态、1200 ms 稳定等待和机器人运动策略；
- 不把 Orbbec 与 MVS 做成运行时热切换的同一个二进制；
- 不从 `caitu-nano-new` 整文件覆盖当前 Controller、CMake 或标定代码。

## 2. 当前状态判断

当前仓库已经有：

- `cameras/MVSCamera.*`；
- `USE_ORB=OFF` 条件编译；
- `build_hand_eye.sh mvs`；
- `hand-eye-mvs` 输出命名；
- MVS 的枚举、打开、BGR取帧、曝光和增益调用；
- 相机序列号、型号、固件的基础审计字段。

仍需关闭的生产缺口：

1. 选机失败时日志称“使用第一台”，实际却直接失败；多相机场景也缺少安全策略。
2. SDK返回值大量被忽略，打开失败只能看到笼统错误。
3. 使用请求分辨率而非实际流分辨率写入 Session provenance。
4. MVS不提供镜头内参，但当前链路仍可能在内参缺失时回退到现场全量重算。
5. GUI只读取平铺配置，不能直接兼容参考工程的 `camera_mvs_130/500` 配置节。
6. 远端缺少 `libMvCameraControl.so` 时，二进制部署后无法启动；当前没有完整打包和部署校验。
7. MVS实现主要使用 `stderr`，缺少可复盘的阶段、SDK错误码、设备身份和实际参数日志。

## 3. 架构决策

采用“同一代码库、两个独立构建产物”：

```text
hand-eye-orb  -> Orbbec SDK，保持现有产线版本
hand-eye-mvs  -> 海康 MVS SDK，只在 MVS 环境部署
```

每个产物只链接自己的相机 SDK。MVS运行库缺失不能影响 `hand-eye-orb` 启动。

相机创建集中到一个小型 factory，Controller 不再自行拼接多个 `#ifdef` 分支，但本轮不实现运行时切换。

## 4. 分阶段实现

### Phase A：构建和交付闭环

涉及文件：

- `CMakeLists.txt`
- `hand_eye_gui/CMakeLists.txt`
- `build_hand_eye.sh`
- 新增部署/检查脚本（名称由实现者确定）

要求：

1. 保留 `USE_ORB` 兼容入口，或者等价替换为显式 `CAMERA_BACKEND=ORB|MVS`，但不得改变默认 ORB 产物。
2. MVS配置阶段必须同时检查：
   - `/opt/MVS/include/MvCameraControl.h`；
   - 当前架构对应的 `libMvCameraControl.so`。
3. SDK不完整时配置直接失败，不允许生成名为 MVS、实际无相机支持的二进制。
4. `./build_hand_eye.sh all` 必须以 Release 构建两个 GUI，并输出：
   - `build_orb/hand-eye-orb`；
   - `build_mvs/hand-eye-mvs`。
5. MVS发布包必须解决运行库交付：
   - 优先在发布目录放置 `lib/mvs/libMvCameraControl.so*`；
   - MVS二进制设置 `$ORIGIN/lib/mvs` 的 install RPATH；
   - 是否允许分发SDK库须遵循海康SDK许可；若不能随包分发，部署脚本必须验证目标机 `/opt/MVS` 并明确失败。
6. 构建身份和启动日志增加 `camera_backend=orbbec|mvs`、MVS SDK版本及 Release/commit 信息。

验收：两个版本均是 ARM64 Release；`ldd hand-eye-mvs` 和 `ldd hand-eye-orb` 均无 `not found`，且 ORB产物不依赖 MVS库。

### Phase B：加固 MVSCamera

涉及文件：

- `cameras/AICamera.hpp`
- `cameras/MVSCamera.hpp`
- `cameras/MVSCamera.cpp`
- 可新增纯C++的设备选择/错误转换辅助文件

设备选择顺序固定为：

```text
配置了 serial -> 必须精确匹配 serial
否则配置了 IP -> 必须精确匹配 GigE IP
否则只有一台设备 -> 使用唯一设备
否则 -> 拒绝打开并列出所有候选设备
```

禁止“找不到指定设备后静默使用第一台”。这会破坏内参与相机身份的绑定。

生命周期要求：

1. `open()`、部分打开失败、`close()`、析构均可重复调用且不会重复释放。
2. 显式记录 SDK initialize、create handle、open、configure、start grabbing 各阶段状态。
3. 每个关键SDK调用保存十六进制错误码和阶段；Controller可通过 `lastError()` 或等价的Qt无关接口读取。
4. 仅对 GigE 设置最佳包大小和心跳；USB设备不得写GigE节点。
5. Width/Height/PixelFormat/Exposure/Gain 设置后必须回读实际值。
6. 分辨率不满足设备步进或范围时，应明确失败或记录回退后的实际值，不能继续假装使用请求分辨率。
7. 取帧缓冲区使用实际尺寸安全分配，检查乘法溢出；实际帧尺寸变化时安全扩容。
8. 输出统一为拥有独立内存的 `CV_8UC3 BGR`，不得把SDK缓冲区生命周期暴露给上层。
9. 空帧、超时和连续失败日志限频，记录SDK帧号、设备时间戳、主机时间戳、实际尺寸和像素格式。
10. 曝光/增益切换后清空旧缓冲并丢弃至少一帧，避免把旧曝光帧用于标定。
11. 为 open/close/capture/参数设置增加互斥保护；现有“标定期间停止预览”策略继续保留。

可从 `caitu-nano-new` 复用的仅是以下调用顺序和字段解析：

- `MV_CC_Initialize` / `MV_CC_EnumDevices`；
- USB/GigE 序列号和IP读取；
- `MV_CC_CreateHandle` / `MV_CC_OpenDevice`；
- `MV_CC_StartGrabbing` / `MV_CC_GetImageForBGR`；
- 曝光、增益及设备固件节点名称。

当前仓库新增的结构化日志、异常保护和审计字段必须保留。

### Phase C：Controller、配置与审计

涉及文件：

- `hand_eye_gui/CalibrationController.*`
- `hand_eye_gui/qml/HandEyeCalibration.qml`
- `my_base.h`
- `calibration/auto_eye_in_hand_charuco_calibration.cpp`
- `calibration/calibration_session.*`（仅补相机审计字段，不改变Session事务语义）

要求：

1. MVS默认IP保持 `192.168.3.10`；机器人默认IP保持 `192.168.3.6`。
2. 增加可选 `cam_sn`。配置了序列号时序列号优先，IP作为审计信息而不是替代身份。
3. 配置兼容顺序：
   - 当前平铺键 `cam_ip/cam_port/cam_width/cam_height/cam_sn`；
   - 参考工程的 `camera_mvs_130`、`camera_mvs_500` 节；
   - 编译期默认值。
4. 对嵌套配置增加显式 `camera_profile` 选择，禁止根据枚举顺序猜型号。
5. UI显示只读 backend、实际设备型号、序列号、固件、transport和实际分辨率。MVS下标记 `cam_port` 为未使用或隐藏。
6. Session必须记录实际值而非请求值：
   - backend和transport；
   - serial/model/firmware/current IP；
   - 实际 width/height/pixel format；
   - exposure/gain/trigger mode；
   - MVS SDK版本。
7. 打开失败日志至少包含：选择条件、枚举设备数、候选设备、失败阶段和SDK错误码。

### Phase D：内参安全门禁

MVS SDK不等于镜头标定数据库。当前 `MVSCamera::getCameraIntrinsics()` 返回空是合理的，不得伪造设备内参。

在线 Eye-in-Hand 对 MVS 采用以下规则：

1. 默认必须加载外部 `param/camera_params.yaml`。
2. 校验 K、D 的形状、有限性、焦距合理性以及内参绑定的 serial、分辨率、ROI/offset/binning。
3. 内参与当前相机或实际流配置不匹配时阻断标定和生产YAML发布。
4. 不得在“打开MVS相机”时覆盖已有 `camera_params.yaml`。
5. 不得因内参缺失而静默进入360帧全量内参非线性重算。
6. 若产品仍需自动重算，必须作为用户显式选择的“相机内参标定模式”，与手眼标定分开，并在Session中记录来源和hash。

兼容旧的仅含 K/D 的 YAML 时可以给出“身份未绑定”的警告，但生产模式是否放行必须使用一个明确的兼容开关，默认关闭。

### Phase E：验证与回归

无硬件测试：

- serial/IP/唯一设备/多设备的选择策略；
- IPv4转换和GigE设备信息解析；
- 配置优先级及旧配置兼容；
- 分辨率、缓冲区和溢出边界；
- SDK错误码到诊断信息的转换；
- MVS内参缺失或身份不匹配时禁止发布。

构建测试：

```bash
./build_hand_eye.sh all
file build_orb/hand-eye-orb build_mvs/hand-eye-mvs
ldd build_orb/hand-eye-orb
ldd build_mvs/hand-eye-mvs
```

Jetson真机冒烟测试：

1. 精确连接 `192.168.3.10`，日志中的 serial/model/IP 与设备一致。
2. 连续取流1000帧，无崩溃、无尺寸变化、空帧率低于1%；超阈值必须报警。
3. 曝光、自动曝光、增益设置和回读一致。
4. 连续执行10次 open/preview/close，不泄漏handle且可再次打开。
5. 运行中断网或拔线，程序不闪退，能够超时退出并给出SDK错误码。
6. 使用匹配内参完成一次360候选、1200 ms等待的 Eye-in-Hand 标定，Session provenance中的相机身份和实际分辨率正确。
7. 使用错误序列号、错误IP、错误分辨率内参分别验证必须失败且不覆盖生产 `hand_eye_result.yaml`。

ORB回归：

- `hand-eye-orb` 编译、打开、预览和一次离线Replay结果与基线一致；
- 不链接 `libMvCameraControl.so`；
- 现有Session、Phase 2、Phase 3、日志和发布事务测试继续通过。

## 5. 推荐提交拆分

1. `build(mvs): add reproducible mvs release packaging`
2. `feat(camera): harden mvs discovery and frame acquisition`
3. `feat(calibration-ui): add mvs configuration and diagnostics`
4. `fix(calibration): enforce mvs intrinsic identity checks`
5. `test(camera): add mvs selection and configuration coverage`

每个提交都必须能独立编译；不要把 `caitu-nano-new` 的旧版大文件整体覆盖进来。

## 6. 完成定义

只有同时满足以下条件才算MVS支持完成：

- 产出可部署的 ARM64 Release `hand-eye-mvs`；
- 目标机 `ldd` 无缺失库；
- 能按IP或序列号确定性连接海康相机；
- 图像格式、实际尺寸、曝光和设备身份可审计；
- 内参错误或身份不匹配时拒绝标定发布；
- MVS断线、超时、关闭和重开不会导致闪退；
- Orbbec版本功能和标定结果不回退。
