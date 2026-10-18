#ifndef COLLECTION_CONTROLLER_HPP
#define COLLECTION_CONTROLLER_HPP

#include <QObject>
#include <QString>
#include <QImage>
#include <QTimer>
#include <thread>
#include <atomic>
#include <memory>

#include "cameras/AICamera.hpp"
#include "robotapi.h"
#include "basestruct.h"

class CameraImageProvider;
namespace collection { struct UndistortionCalibration; class UndistortionMaps; }

/** Collection devices, preview and background capture operations. */
class CollectionController : public QObject
{
    Q_OBJECT

    // ---- QML 可绑定属性 ----
    Q_PROPERTY(QString robotIp        READ robotIp        WRITE setRobotIp        NOTIFY robotIpChanged)
    Q_PROPERTY(QString cameraIp       READ cameraIp       WRITE setCameraIp       NOTIFY cameraIpChanged)
    Q_PROPERTY(int     previewWidth   READ previewWidth   WRITE setPreviewWidth   NOTIFY previewWidthChanged)
    Q_PROPERTY(int     previewHeight  READ previewHeight  WRITE setPreviewHeight  NOTIFY previewHeightChanged)
    Q_PROPERTY(int     cameraExposureUs READ cameraExposureUs NOTIFY cameraExposureUsChanged)
    Q_PROPERTY(int     cameraActualExposureUs READ cameraActualExposureUs NOTIFY cameraActualExposureUsChanged)
    // 相机后端与实际设备信息（只读审计，打开相机后刷新）
    Q_PROPERTY(QString cameraBackend     READ cameraBackend     CONSTANT)
    Q_PROPERTY(QString cameraModel       READ cameraModel       NOTIFY cameraDeviceInfoChanged)
    Q_PROPERTY(QString cameraSerial      READ cameraSerial      NOTIFY cameraDeviceInfoChanged)
    Q_PROPERTY(QString cameraFirmware    READ cameraFirmware    NOTIFY cameraDeviceInfoChanged)
    Q_PROPERTY(QString cameraTransport   READ cameraTransport   NOTIFY cameraDeviceInfoChanged)
    Q_PROPERTY(int     cameraActualWidth  READ cameraActualWidth  NOTIFY cameraDeviceInfoChanged)
    Q_PROPERTY(int     cameraActualHeight READ cameraActualHeight NOTIFY cameraDeviceInfoChanged)
    Q_PROPERTY(bool    robotConnected READ robotConnected NOTIFY robotConnectedChanged)
    Q_PROPERTY(bool    cameraOpened   READ cameraOpened   NOTIFY cameraOpenedChanged)
    Q_PROPERTY(bool    previewActive  READ previewActive  NOTIFY previewActiveChanged)
    Q_PROPERTY(QString statusMessage  READ statusMessage  NOTIFY statusMessageChanged)
    Q_PROPERTY(QString logText        READ logText        NOTIFY logTextChanged)
    Q_PROPERTY(QString diagnosticLogPath READ diagnosticLogPath CONSTANT)
    Q_PROPERTY(bool collectionBusy READ collectionBusy NOTIFY collectionChanged)
    Q_PROPERTY(QString outputDirectory READ outputDirectory CONSTANT)
    Q_PROPERTY(QString exposureList READ exposureList WRITE setExposureList NOTIFY collectionChanged)
    Q_PROPERTY(QString lastCapturePath READ lastCapturePath NOTIFY collectionChanged)
    Q_PROPERTY(int savedImageCount READ savedImageCount NOTIFY collectionChanged)
    Q_PROPERTY(int savedPoseCount READ savedPoseCount NOTIFY collectionChanged)
    Q_PROPERTY(bool undistortionReady READ undistortionReady NOTIFY collectionChanged)
    Q_PROPERTY(bool undistortionEnabled READ undistortionEnabled NOTIFY collectionChanged)
    Q_PROPERTY(QString undistortionStatus READ undistortionStatus NOTIFY collectionChanged)

public:
    explicit CollectionController(QObject* parent = nullptr);
    ~CollectionController() override;

    /// 必须在 QML 引擎初始化后调用：设置 imageProvider 并完成路径初始化
    void init(CameraImageProvider* provider);
    bool collectionBusy() const { return m_collectionBusy; }
    QString outputDirectory() const { return m_outputDirectory; }
    QString exposureList() const { return m_exposureList; }
    QString lastCapturePath() const { return m_lastCapturePath; }
    int savedImageCount() const { return m_savedImageCount; }
    int savedPoseCount() const { return m_savedPoseCount; }
    bool undistortionReady() const { return bool(m_undistortionCalibration); }
    bool undistortionEnabled() const { return bool(m_undistortionMaps); }
    QString undistortionStatus() const { return m_undistortionStatus; }
    Q_INVOKABLE void toggleUndistortion();
    Q_INVOKABLE void saveFlangePose(const QString& name);
    void setExposureList(const QString& value);
    Q_INVOKABLE void openCollectionFolder();

    // ---- 属性访问器 ----
    QString robotIp() const         { return m_robotIp; }
    QString cameraIp() const        { return m_cameraIp; }
    int     previewWidth() const    { return m_previewWidth; }
    int     previewHeight() const   { return m_previewHeight; }
    int     cameraExposureUs() const { return m_cameraExposureUs; }
    int     cameraActualExposureUs() const { return m_cameraActualExposureUs; }
    QString cameraBackend() const;
    QString cameraModel() const     { return m_camDeviceModel; }
    QString cameraSerial() const    { return m_camDeviceSerial; }
    QString cameraFirmware() const  { return m_camDeviceFirmware; }
    QString cameraTransport() const { return m_camDeviceTransport; }
    int     cameraActualWidth() const  { return m_camActualWidth; }
    int     cameraActualHeight() const { return m_camActualHeight; }
    bool    robotConnected() const  { return m_robotConnected; }
    bool    cameraOpened() const    { return m_cameraOpened; }
    bool    previewActive() const   { return m_previewActive; }
    QString statusMessage() const   { return m_statusMessage; }
    QString logText() const         { return m_logText; }
    QString diagnosticLogPath() const;

    void setRobotIp(const QString& v);
    void setCameraIp(const QString& v);
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

    /// 热更新相机曝光，并将设备回读值持久化到 config.yaml。
    void adjustCameraExposure(int exposureUs);
    void capturePhoto();
    void levelCamera();
    void cancelCollectionOperation();

signals:
    void robotIpChanged();
    void cameraIpChanged();
    void cameraDeviceInfoChanged();
    void previewWidthChanged();
    void previewHeightChanged();
    void cameraExposureUsChanged();
    void cameraActualExposureUsChanged();
    void robotConnectedChanged();
    void cameraOpenedChanged();
    void previewActiveChanged();
    void statusMessageChanged();
    void logTextChanged();
    void frameUpdated();
    void collectionChanged();

private:
    void initializeCollection();
    void loadCollectionUndistortion();
    void persistCollectionString(const std::string& key, const QString& value);
    void persistCollectionInteger(const std::string& key, int value);
    void beginCollectionOperation();
    void finishCollectionOperation(const QString& message, const QString& savedPath = {});
    // ---- 初始化 ----
    void setupPaths();
    void loadConfigYaml();

    // ---- 预览循环 ----
    void onPreviewTick();

    // ---- 内部辅助 ----
    void setStatus(const QString& msg);
    bool checkAndRestoreRobotHand();

    /// 将 cv::Mat (BGR) 转为 QImage (RGB)，线程安全发送到 ImageProvider
    void publishFrame(const cv::Mat& bgr);

    /// 追加日志（线程安全）；UI 侧批量刷新，避免每条触发完整重排
    void appendLog(const QString& line);
    void flushPendingLogs();

    static bool robotMotionCancellationRequested();
    static CollectionController* s_instance;

    // ==================== 数据成员 ====================

    // 配置
    QString  m_robotIp      = "192.168.66.14";
    int      m_robotPort    = 2323;
    QString  m_robotPasswd  = "123";
    QString  m_cameraIp     = QString::fromStdString(CameraConfig{}.cam_ip);
    QString  m_cameraProfile;                   // config.yaml 嵌套配置节名
    int      m_previewWidth  = CameraConfig{}.cam_width;
    int      m_previewHeight = CameraConfig{}.cam_height;
    int      m_cameraExposureUs = CameraConfig{}.exposure_us;
    int      m_cameraGain = CameraConfig{}.gain;

    // 打开相机后回读的实际设备信息（只读审计）
    QString  m_camDeviceModel;
    QString  m_camDeviceSerial;
    QString  m_camDeviceFirmware;
    QString  m_camDeviceTransport;
    int      m_camActualWidth  = 0;
    int      m_camActualHeight = 0;
    int      m_cameraActualExposureUs = -1;

    // 状态
    bool     m_robotConnected = false;
    bool     m_cameraOpened   = false;
    bool     m_previewActive  = false;
    QString  m_statusMessage  = "就绪";
    QString  m_logText;
    QStringList m_pendingLogLines;
    QTimer*  m_logFlushTimer = nullptr;

    // 硬件句柄
    RobotHandle m_robotHandle = 0;
    std::shared_ptr<AICamera> m_camera;

    // 预览
    CameraImageProvider* m_imageProvider = nullptr;
    QTimer*              m_previewTimer  = nullptr;
    quint64 m_previewFrameCount = 0;
    quint64 m_previewEmptyCount = 0;

    std::atomic<bool>  m_cancelRequested{false};

    // 路径
    std::string m_appPath;
    std::string m_paramPath;
    bool m_collectionBusy = false;
    bool m_collectionWasPreviewing = false;
    QString m_outputDirectory;
    QString m_exposureList = "5000,8500,14500,25000";
    QString m_handEyePath;
    QString m_undistortionStatus;
    std::shared_ptr<const collection::UndistortionCalibration> m_undistortionCalibration;
    std::shared_ptr<const collection::UndistortionMaps> m_undistortionMaps;
    QString m_lastCapturePath;
    int m_savedImageCount = 0;
    int m_savedPoseCount = 0;
    std::thread m_collectionThread;

};

#endif // COLLECTION_CONTROLLER_HPP
