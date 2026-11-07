#ifndef COLLECTION_CONTROLLER_HPP
#define COLLECTION_CONTROLLER_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QImage>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>
#include <QElapsedTimer>
#include <QPointer>
#include <QUrl>
#include <functional>
#include <mutex>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <future>

#include "cameras/AICamera.hpp"
#include "RobotMotionLimits.hpp"
#include "TeachMotion.hpp"
#include "robotapi.h"
#include "basestruct.h"

class CameraImageProvider;
namespace collection { struct UndistortionCalibration; class UndistortionMaps; }

/** Collection devices, preview and background capture operations. */
class CollectionController : public QObject
{
    Q_OBJECT

    // ---- QML 可绑定属性 ----
    Q_PROPERTY(QString robotIp        READ robotIp         NOTIFY robotIpChanged)
    Q_PROPERTY(int robotSpeedMmS READ robotSpeedMmS NOTIFY collectionChanged)
    Q_PROPERTY(QString cameraIp       READ cameraIp        NOTIFY cameraIpChanged)
    Q_PROPERTY(int     previewWidth   READ previewWidth    NOTIFY previewWidthChanged)
    Q_PROPERTY(int     previewHeight  READ previewHeight   NOTIFY previewHeightChanged)
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
    Q_PROPERTY(bool previewVisible READ previewVisible WRITE setPreviewVisible NOTIFY previewVisibleChanged)
    Q_PROPERTY(QSize previewDisplaySize READ previewDisplaySize WRITE setPreviewDisplaySize NOTIFY previewDisplaySizeChanged)
    Q_PROPERTY(QString statusMessage  READ statusMessage  NOTIFY statusMessageChanged)
    Q_PROPERTY(QString diagnosticLogPath READ diagnosticLogPath CONSTANT)
    Q_PROPERTY(bool collectionBusy READ collectionBusy NOTIFY collectionChanged)
    Q_PROPERTY(bool settingsSaving READ settingsSaving NOTIFY collectionChanged)
    Q_PROPERTY(bool coordinatesImporting READ coordinatesImporting NOTIFY collectionChanged)
    Q_PROPERTY(bool motionFault READ motionFault NOTIFY collectionChanged)
    Q_PROPERTY(QString outputDirectory READ outputDirectory CONSTANT)
    Q_PROPERTY(QString handEyePath READ handEyePath NOTIFY collectionChanged)
    Q_PROPERTY(QString exposureList READ exposureList NOTIFY collectionChanged)
    Q_PROPERTY(QString lastCapturePath READ lastCapturePath NOTIFY collectionChanged)
    Q_PROPERTY(int savedImageCount READ savedImageCount NOTIFY collectionChanged)
    Q_PROPERTY(int savedPoseCount READ savedPoseCount NOTIFY collectionChanged)
    Q_PROPERTY(bool undistortionReady READ undistortionReady NOTIFY collectionChanged)
    Q_PROPERTY(bool undistortionEnabled READ undistortionEnabled NOTIFY collectionChanged)
    Q_PROPERTY(QString undistortionStatus READ undistortionStatus NOTIFY collectionChanged)
    Q_PROPERTY(QStringList tcpNames READ tcpNames NOTIFY tcpToolsChanged)
    Q_PROPERTY(QString selectedTcp READ selectedTcp WRITE setSelectedTcp NOTIFY tcpToolsChanged)
    Q_PROPERTY(QString ninePointProgress READ ninePointProgress NOTIFY collectionChanged)
    Q_PROPERTY(QVariantList ninePointDefaultYawOffsets READ ninePointDefaultYawOffsets CONSTANT)
    Q_PROPERTY(bool tcpPoseValid READ tcpPoseValid NOTIFY tcpPoseChanged)
    Q_PROPERTY(QString tcpPoseStatus READ tcpPoseStatus NOTIFY tcpPoseChanged)
    Q_PROPERTY(QString tcpPoseTimestamp READ tcpPoseTimestamp NOTIFY tcpPoseChanged)
    Q_PROPERTY(QString tcpPoseTool READ tcpPoseTool NOTIFY tcpPoseChanged)
    Q_PROPERTY(bool manualMoveActive READ manualMoveActive NOTIFY collectionChanged)
    Q_PROPERTY(QString manualMoveProgress READ manualMoveProgress NOTIFY collectionChanged)
    Q_PROPERTY(QString orientationMode READ orientationMode WRITE setOrientationMode NOTIFY teachSettingsChanged)
    Q_PROPERTY(QVariantList tcpDisplayPoseValues READ tcpDisplayPoseValues NOTIFY tcpPoseChanged)
    Q_PROPERTY(bool tcpPoseFresh READ tcpPoseFresh NOTIFY tcpPoseChanged)
    Q_PROPERTY(double manualLinearSpeedMmS READ manualLinearSpeedMmS NOTIFY teachSettingsChanged)
    Q_PROPERTY(double manualAngularSpeedDegS READ manualAngularSpeedDegS NOTIFY teachSettingsChanged)
    Q_PROPERTY(bool jogActive READ jogActive NOTIFY collectionChanged)
    Q_PROPERTY(QString previewPositiveX READ previewPositiveX NOTIFY previewDirectionsChanged)
    Q_PROPERTY(QString previewPositiveY READ previewPositiveY NOTIFY previewDirectionsChanged)
    Q_PROPERTY(QString lastCaptureTime READ lastCaptureTime NOTIFY collectionChanged)

public:
    explicit CollectionController(QObject* parent = nullptr);
    ~CollectionController() override;

    /// 必须在 QML 引擎初始化后调用：设置 imageProvider 并完成路径初始化
    void init(CameraImageProvider* provider);
    bool collectionBusy() const { return m_collectionBusy; }
    bool settingsSaving() const { return m_settingsSaving; }
    bool coordinatesImporting() const { return m_coordinatesImporting; }
    bool motionFault() const { return m_motionFault; }
    QString outputDirectory() const { return m_outputDirectory; }
    QString handEyePath() const { return m_handEyePath; }
    QString exposureList() const { return m_exposureList; }
    QString lastCapturePath() const { return m_lastCapturePath; }
    int savedImageCount() const { return m_savedImageCount; }
    int savedPoseCount() const { return m_savedPoseCount; }
    bool undistortionReady() const { return bool(m_undistortionCalibration); }
    bool undistortionEnabled() const { return bool(m_undistortionMaps); }
    QString undistortionStatus() const { return m_undistortionStatus; }
    Q_INVOKABLE void toggleUndistortion();
    Q_INVOKABLE void saveTcpPose(const QString& name);
    Q_INVOKABLE void refreshTcpTools();
    QStringList tcpNames() const { return m_tcpNames; }
    QString selectedTcp() const { return m_selectedTcp; }
    QString ninePointProgress() const { return m_ninePointProgress; }
    QVariantList ninePointDefaultYawOffsets() const;
    void setSelectedTcp(const QString& name);
    Q_INVOKABLE void openCollectionFolder();
    Q_INVOKABLE void openDiagnosticFolder();
    Q_INVOKABLE void applySettings(const QVariantMap& values);
    Q_INVOKABLE void chooseTcpCoordinateFile();
    Q_INVOKABLE void importTcpCoordinates(const QUrl& url);
    Q_INVOKABLE QVariantMap parseTcpTarget(const QString& text) const;
    QVariantList tcpPoseValues() const { return m_tcpPoseValues; }
    bool tcpPoseValid() const { return m_tcpPoseValid; }
    QString tcpPoseStatus() const;
    QString tcpPoseTimestamp() const { return m_tcpPoseTimestamp; }
    QString tcpPoseTool() const { return m_tcpPoseTool; }
    bool manualMoveActive() const { return m_manualMoveActive; }
    QString manualMoveProgress() const { return m_manualMoveProgress; }
    QString orientationMode() const { return m_orientationMode; }
    void setOrientationMode(const QString& mode);
    QVariantList tcpDisplayPoseValues() const;
    bool tcpPoseFresh() const;
    double manualLinearSpeedMmS() const { return m_teachSpeeds.linearMmS; }
    double manualAngularSpeedDegS() const { return m_teachSpeeds.angularDegS; }
    bool jogActive() const { return bool(m_jogLease); }
    QString previewPositiveX() const { return m_previewPositiveX; }
    QString previewPositiveY() const { return m_previewPositiveY; }
    QString lastCaptureTime() const { return m_lastCaptureTime; }
    Q_INVOKABLE QVariantMap convertTcpPose(const QString& text, const QString& from, const QString& to) const;
    Q_INVOKABLE void copyCurrentTcpCoordinates(const QString& mode);
    Q_INVOKABLE void moveTcpToDisplay(const QString& text, const QString& mode);
    Q_INVOKABLE qulonglong beginTcpJog(int component, int direction, const QString& mode);
    Q_INVOKABLE void renewTcpJog(qulonglong token);
    Q_INVOKABLE void stopTcpJog(qulonglong token);
    Q_INVOKABLE void setManualSpeeds(double linear, double angular);
    Q_INVOKABLE void setPreviewDirections(const QString& x, const QString& y);

    // ---- 属性访问器 ----
    QString robotIp() const         { return m_robotIp; }
    int robotSpeedMmS() const { return m_robotSpeedMmS; }
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
    bool previewVisible() const { return m_previewVisible; }
    void setPreviewVisible(bool visible);
    QSize previewDisplaySize() const { return m_previewDisplaySize; }
    void setPreviewDisplaySize(const QSize& size);
    QString statusMessage() const   { return m_statusMessage; }
    QString diagnosticLogPath() const;


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

    void capturePhoto();
    void captureNinePoints(double xMm, double yMm, const QVariantList& yawValues);
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
    void previewVisibleChanged();
    void previewDisplaySizeChanged();
    void statusMessageChanged();
    void frameUpdated();
    void collectionChanged();
    void settingsFinished(const QString& error);
    void coordinatesImported(const QString& coordinates, const QString& tcpName, const QString& error);
    void tcpToolsChanged();
    void tcpPoseChanged();
    void teachSettingsChanged();
    void previewDirectionsChanged();

private:
    friend struct CollectionControllerTestAccess;
    friend struct CollectionSettingsTestAccess;
    void initializeCollection();
    void loadCollectionUndistortion();
    void beginCollectionOperation();
    void startCapture(bool ninePoints, double xMm = 0, double yMm = 0, const QVariantList& yawValues = {});
    void recordCapture(const QString& path);
    void confirmMotionStopped();
    void retryRobotStop();
    void finishCollectionOperation(const QString& message, const QString& savedPath = {});
    // ---- 初始化 ----
    void setupPaths();
    void loadConfigYaml();
    void loadTeachPreferences();
    void scheduleTeachPreferences();
    void flushTeachPreferences();
    void finishTeachPreferences();
    void startManualMotion(std::function<void(RobotHandle, const QString&, collection::MotionTick)> motion,
        const QString& description);
    CameraConfig currentCameraConfig() const;

    // ---- 预览循环 ----
    void onPreviewTick();
    void updatePreviewTimer();

    // ---- 内部辅助 ----
    void setStatus(const QString& msg);
    struct RobotSession {
        explicit RobotSession(RobotHandle value) : handle(value) {}
        std::unique_lock<std::mutex> acquire(const std::atomic<bool>& cancelled) {
            std::unique_lock<std::mutex> lock(mutex, std::defer_lock);
            while (!cancelled && !closing) {
                if (lock.try_lock()) {
                    if (cancelled || closing) lock.unlock();
                    return lock;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            return lock;
        }
        std::mutex mutex;
        const RobotHandle handle;
        std::atomic<bool> closing{false};
        std::atomic<bool> telemetryInFlight{false};
    };
    std::shared_ptr<RobotSession> robotSession();
    void pollTcpPose();
    void invalidateTcpPose(const QString& status);
    void applyTcpSample(const std::shared_ptr<RobotSession>& session, quint64 generation,
        const QString& tool, const QVariantList& values, const QString& error, qint64 timestamp);
    std::function<void()> tcpSamplingCallback(const std::shared_ptr<RobotSession>& session);

    /// 后台生成适配显示尺寸、拥有像素内存的预览图像；不修改采集原图。
    static QImage makePreviewImage(const cv::Mat& bgr, const QSize& displaySize);

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
    bool m_robotDisconnecting = false;
    QStringList m_tcpNames;
    QString m_selectedTcp;
    std::shared_ptr<RobotSession> m_robotSession;
    std::shared_ptr<QObject> m_robotDelivery;
    QTimer* m_tcpPoseTimer = nullptr;
    quint64 m_tcpPoseGeneration = 0;
    QVariantList m_tcpPoseValues;
    bool m_tcpPoseValid = false;
    QString m_tcpPoseStatus = "机器人未连接";
    QString m_tcpPoseTimestamp;
    QString m_tcpPoseTool;
    QString m_tcpPoseLastFailure;
    QElapsedTimer m_tcpPoseAge;
    qint64 m_tcpPoseDeliveryLagMs = 0;
    bool m_manualMoveActive = false;
    QString m_manualMoveProgress;
    QString m_orientationMode = "axis";
    collection::TeachSpeeds m_teachSpeeds;
    QString m_previewPositiveX = "none", m_previewPositiveY = "none";
    std::shared_ptr<collection::TeachJogLease> m_jogLease;
    qulonglong m_jogToken = 0;
    QString m_lastCaptureTime;
    QTimer* m_teachPreferencesTimer = nullptr;
    bool m_teachPreferencesDirty = false;
    std::future<QString> m_teachPreferencesWrite;
    bool     m_cameraOpened   = false;
    int m_robotSpeedMmS = collection::defaultRobotSpeedMmS;
    bool     m_previewActive  = false;
    bool     m_previewVisible = true;
    QString  m_statusMessage  = "就绪";

    // 硬件句柄
    RobotHandle m_robotHandle = 0;
    std::atomic<RobotHandle> m_pendingRobotHandle{0};
    std::shared_ptr<AICamera> m_camera;

    // 预览
    CameraImageProvider* m_imageProvider = nullptr;
    struct PreviewFrame {
        QImage image;
        QSize originalSize;
        int channels = 0;
    };
    std::shared_future<PreviewFrame> m_previewFuture;
    quint64 m_previewGeneration = 0, m_pendingPreviewGeneration = 0;
    QSize m_previewDisplaySize{1280, 960};
    QTimer*              m_previewTimer  = nullptr;
    quint64 m_previewFrameCount = 0;
    quint64 m_previewEmptyCount = 0;

    std::atomic<bool>  m_cancelRequested{false};

    // 路径
    std::string m_appPath;
    std::string m_paramPath;
    bool m_collectionBusy = false;
    bool m_settingsSaving = false;
    bool m_coordinatesImporting = false;
    QPointer<QObject> m_coordinateDialog;
    bool m_motionFault = false;
    bool m_stopRetrying = false;
    std::atomic<bool> m_robotStopUnconfirmed{false};
    bool m_collectionWasPreviewing = false;
    QString m_outputDirectory;
    QString m_exposureList = "5000,8500,14500,25000";
    QString m_handEyePath;
    QString m_undistortionStatus;
    std::shared_ptr<const collection::UndistortionCalibration> m_undistortionCalibration;
    std::shared_ptr<const collection::UndistortionMaps> m_undistortionMaps;
    QString m_lastCapturePath;
    QString m_ninePointProgress;
    int m_savedImageCount = 0;
    int m_savedPoseCount = 0;
    std::thread m_collectionThread;

};

#endif // COLLECTION_CONTROLLER_HPP
