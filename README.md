# calibri_nano

## 标定审计部署边界

- 掉电持久性保证当前仅适用于 Linux/Jetson。Windows 构建尚未实现等价的
  `FlushFileBuffers`、目录项持久化和覆盖已有目标文件的原子发布语义，不能按
  Phase 1.1 的掉电安全标准验收。
- 标定结果发布采用单写者模型。同一生产 YAML 路径同一时间只允许一个 GUI、
  CLI 或离线 Replay 进程写入；当前版本不提供跨进程文件锁。
- 已封存 Session 是不可变审计数据。Replay 输出必须位于源 Session 目录之外，
  工具会在复算前解析相对路径、`..` 和符号链接并拒绝越界输出。
