#ifndef CALIBRATION_CONTROLLER_HPP
#define CALIBRATION_CONTROLLER_HPP

#include <QObject>
#include <QString>
#include <QImage>
#include <QTimer>
#include <thread>
#include <atomic>
#include <memory>

#include "my_base.h"
#include "cameras/AICamera.hpp"
#include "calibration/charuco_detector.hpp"
#include "calibration/calibration_runtime_status.hpp"
#include "robotapi.h"
#include "basestruct.h"

class CameraImageProvider;

/**
 * @brief 手眼标定 QML 控制器
 *
 * 单例注入 QML，管理机器人连接、相机预览、和后台标定线程。
 * 直接调用 autoEyeInHandCharucoCalibration / autoEyeToHandCharucoCalibration。
 */
class CalibrationController : public QObject
{
    Q_OBJECT

    // ---- QML 可绑定属性 ----
    Q_PROPERTY(QString robotIp        READ robotIp        WRITE setRobotIp        NOTIFY robotIpChanged)
    Q_PROPERTY(int     robotPort      READ robotPort      WRITE setRobotPort      NOTIFY robotPortChanged)
    Q_PROPERTY(QString robotPasswd    READ robotPasswd    WRITE setRobotPasswd    NOTIFY robotPasswdChanged)
    Q_PROPERTY(int     cameraToTcpEnd READ cameraToTcpEnd WRITE setCameraToTcpEnd NOTIFY cameraToTcpEndChanged)
    Q_PROPERTY(QString calibrationType READ calibrationType WRITE setCalibrationType NOTIFY calibrationTypeChanged)
    Q_PROPERTY(QString cameraIp       READ cameraIp       WRITE setCameraIp       NOTIFY cameraIpChanged)
    Q_PROPERTY(int     cameraPort     READ cameraPort     WRITE setCameraPort     NOTIFY cameraPortChanged)
    Q_PROPERTY(int     previewWidth   READ previewWidth   WRITE setPreviewWidth   NOTIFY previewWidthChanged)
    Q_PROPERTY(int     previewHeight  READ previewHeight  WRITE setPreviewHeight  NOTIFY previewHeightChanged)
    Q_PROPERTY(bool    robotConnected READ robotConnected NOTIFY robotConnectedChanged)
    Q_PROPERTY(bool    cameraOpened   READ cameraOpened   NOTIFY cameraOpenedChanged)
    Q_PROPERTY(bool    previewActive  READ previewActive  NOTIFY previewActiveChanged)
    Q_PROPERTY(bool    running        READ running        NOTIFY runningChanged)
    Q_PROPERTY(int     progress       READ progress       NOTIFY progressChanged)
    Q_PROPERTY(QString statusMessage  READ statusMessage  NOTIFY statusMessageChanged)
    Q_PROPERTY(QString resultText     READ resultText     NOTIFY resultTextChanged)
    Q_PROPERTY(QString logText        READ logText        NOTIFY logTextChanged)
    Q_PROPERTY(QString diagnosticLogPath READ diagnosticLogPath CONSTANT)

    // 结构化运行时状态（与 logText 解耦）
    Q_PROPERTY(int     calibrationStage READ calibrationStage NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(QString calibrationStageText READ calibrationStageText NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(int     candidatePlanned READ candidatePlanned NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(int     candidateReachable READ candidateReachable NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(int     candidateProcessed READ candidateProcessed NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(int     capturedFrames READ capturedFrames NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(int     skippedFrames READ skippedFrames NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(int     pnpValidFrames READ pnpValidFrames NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(int     madRejectedFrames READ madRejectedFrames NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(int     finalInliers READ finalInliers NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(double  translationRmsMm READ translationRmsMm NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(double  translationP95Mm READ translationP95Mm NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(double  translationMaxMm READ translationMaxMm NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(double  rotationRmsDeg READ rotationRmsDeg NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(double  rotationP95Deg READ rotationP95Deg NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(double  rotationMaxDeg READ rotationMaxDeg NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(QString selectedSolver READ selectedSolver NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(bool    consensusAvailable READ consensusAvailable NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(bool    consensusPassed READ consensusPassed NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(double  consensusMaxTranslationMm READ consensusMaxTranslationMm NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(double  consensusMaxRotationDeg READ consensusMaxRotationDeg NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(QString calibrationSessionId READ calibrationSessionId NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(QString calibrationSessionDir READ calibrationSessionDir NOTIFY calibrationRuntimeChanged)
    Q_PROPERTY(QString publicationState READ publicationState NOTIFY calibrationRuntimeChanged)

public:
    explicit CalibrationController(QObject* parent = nullptr);
    ~CalibrationController() override;

    /// 必须在 QML 引擎初始化后调用：设置 imageProvider 并完成路径初始化
    void init(CameraImageProvider* provider);

    // ---- 属性访问器 ----
    QString robotIp() const         { return m_robotIp; }
    int     robotPort() const       { return m_robotPort; }
    QString robotPasswd() const     { return m_robotPasswd; }
    int     cameraToTcpEnd() const  { return m_cameraToTcpEnd; }
    QString calibrationType() const { return m_calibrationType; }
    QString cameraIp() const        { return m_cameraIp; }
    int     cameraPort() const      { return m_cameraPort; }
    int     previewWidth() const    { return m_previewWidth; }
    int     previewHeight() const   { return m_previewHeight; }
    bool    robotConnected() const  { return m_robotConnected; }
    bool    cameraOpened() const    { return m_cameraOpened; }
    bool    previewActive() const   { return m_previewActive; }
    bool    running() const         { return m_running; }
    int     progress() const        { return m_progress; }
    QString statusMessage() const   { return m_statusMessage; }
    QString resultText() const      { return m_resultText; }
    QString logText() const         { return m_logText; }
    QString diagnosticLogPath() const;

    int     calibrationStage() const { return static_cast<int>(m_runtimeStatus.stage); }
    QString calibrationStageText() const;
    int     candidatePlanned() const { return m_runtimeStatus.candidate_planned; }
    int     candidateReachable() const { return m_runtimeStatus.candidate_reachable; }
    int     candidateProcessed() const { return m_runtimeStatus.candidate_processed; }
    int     capturedFrames() const { return m_runtimeStatus.captured_frames; }
    int     skippedFrames() const { return m_runtimeStatus.skipped_frames; }
    int     pnpValidFrames() const { return m_runtimeStatus.pnp_valid_frames; }
    int     madRejectedFrames() const { return m_runtimeStatus.mad_rejected_frames; }
    int     finalInliers() const { return m_runtimeStatus.final_inliers; }
    double  translationRmsMm() const { return m_runtimeStatus.translation_rms_mm; }
    double  translationP95Mm() const { return m_runtimeStatus.translation_p95_mm; }
    double  translationMaxMm() const { return m_runtimeStatus.translation_max_mm; }
    double  rotationRmsDeg() const { return m_runtimeStatus.rotation_rms_deg; }
    double  rotationP95Deg() const { return m_runtimeStatus.rotation_p95_deg; }
    double  rotationMaxDeg() const { return m_runtimeStatus.rotation_max_deg; }
    QString selectedSolver() const { return QString::fromStdString(m_runtimeStatus.selected_solver); }
    bool    consensusAvailable() const { return m_runtimeStatus.consensus_available; }
    bool    consensusPassed() const { return m_runtimeStatus.consensus_passed; }
    double  consensusMaxTranslationMm() const { return m_runtimeStatus.consensus_max_translation_mm; }
    double  consensusMaxRotationDeg() const { return m_runtimeStatus.consensus_max_rotation_deg; }
    QString calibrationSessionId() const { return QString::fromStdString(m_runtimeStatus.calibration_session_id); }
    QString calibrationSessionDir() const { return QString::fromStdString(m_runtimeStatus.calibration_session_dir); }
    QString publicationState() const { return QString::fromStdString(m_runtimeStatus.publication_state); }

    void setRobotIp(const QString& v);
    void setRobotPort(int v);
    void setRobotPasswd(const QString& v);
    void setCameraToTcpEnd(int v);
    void setCalibrationType(const QString& v);
    void setCameraIp(const QString& v);
    void setCameraPort(int v);
    void setPreviewWidth(int v);
    void setPreviewHeight(int v);

public slots:
    /// 连接 / 断开机器人
    void connectRobot();
    void disconnectRobot();

    /// 打开 / 关闭相机
    void openCamera();
    void closeCamera();

    /// 启动 / 停止预览
    void startPreview();
    void stopPreview();

    /// 启动 / 取消标定（后台线程）
    void startCalibration();
    void stopCalibration();

signals:
    void robotIpChanged();
    void robotPortChanged();
    void robotPasswdChanged();
    void cameraToTcpEndChanged();
    void calibrationTypeChanged();
    void cameraIpChanged();
    void cameraPortChanged();
    void previewWidthChanged();
    void previewHeightChanged();
    void robotConnectedChanged();
    void cameraOpenedChanged();
    void previewActiveChanged();
    void runningChanged();
    void progressChanged();
    void statusMessageChanged();
    void resultTextChanged();
    void logTextChanged();
    void calibrationRuntimeChanged();
    void frameUpdated();

    /// 标定结束信号
    void calibrationFinished(bool success, const QString& message);

private:
    // ---- 初始化 ----
    void setupPaths();
    void loadConfigYaml();

    // ---- 预览循环 ----
    void onPreviewTick();

    // ---- 标定线程 ----
    void calibrationThreadFunc(QString calibrationType,
                               QString robotIp,
                               int robotPort,
                               int cameraToTcpEnd,
                               std::shared_ptr<AICamera> camera);
    void waitForCalibrationThread();

    // ---- 内部辅助 ----
    void setStatus(const QString& msg);
    void setProgress(int p);
    void setResult(const QString& text);
    bool checkAndRestoreRobotHand();

    /// 将 cv::Mat (BGR) 转为 QImage (RGB)，线程安全发送到 ImageProvider
    void publishFrame(const cv::Mat& bgr);

    /// 追加日志（线程安全）
    void appendLog(const QString& line);

    /// 接收核心标定线程发布的结构化运行状态（切回 Controller 线程后更新）
    void applyRuntimeStatus(const CalibrationRuntimeStatus& status);
    void resetRuntimeStatus();

    // ---- 静态回调（供 C 风格 AI_det_callback） ----
    static int calibrationCallback(char* msg, int msg_len, int flag);
    static bool robotMotionCancellationRequested();
    static CalibrationController* s_instance;

    // ==================== 数据成员 ====================

    // 配置
    QString  m_robotIp      = "192.168.3.6";
    int      m_robotPort    = 2323;
    QString  m_robotPasswd  = "123";
    int      m_cameraToTcpEnd = 150;
    QString  m_calibrationType = "eye_to_hand";   // 或 "eye_in_hand"
    QString  m_cameraIp     = "192.168.3.10";
    int      m_cameraPort   = 8090;
    int      m_previewWidth  = DEFAULT_CAM_WIDTH;
    int      m_previewHeight = DEFAULT_CAM_HEIGHT;

    // 状态
    bool     m_robotConnected = false;
    bool     m_cameraOpened   = false;
    bool     m_previewActive  = false;
    bool     m_running        = false;
    int      m_progress       = 0;
    QString  m_statusMessage  = "就绪";
    QString  m_resultText;
    QString  m_logText;
    CalibrationRuntimeStatus m_runtimeStatus = makeEmptyCalibrationRuntimeStatus();

    // 硬件句柄
    RobotHandle m_robotHandle = 0;
    std::shared_ptr<AICamera> m_camera;

    // 预览
    CameraImageProvider* m_imageProvider = nullptr;
    QTimer*              m_previewTimer  = nullptr;
    std::unique_ptr<CharucoDetector> m_previewDetector;
    quint64 m_previewFrameCount = 0;
    quint64 m_previewEmptyCount = 0;
    bool m_haveBoardDetectionState = false;
    bool m_lastBoardDetected = false;

    // 标定线程
    std::thread        m_calibThread;
    std::atomic<bool>  m_cancelRequested{false};
    std::atomic<int>   m_lastLoggedProgress{-1};

    // 路径
    std::string m_appPath;
    std::string m_paramPath;
    std::string m_calibrationPath;

};

#endif // CALIBRATION_CONTROLLER_HPP
