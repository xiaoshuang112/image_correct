#include "CollectionController.hpp"
#include "DiagnosticLogger.hpp"
#include <QClipboard>
#include <QGuiApplication>
#include <QSettings>
#include <cmath>
#include <stdexcept>

namespace {
collection::TeachOrientation orientation(const QString& mode) {
    if (mode == "axis") return collection::TeachOrientation::RotationVector;
    if (mode == "euler") return collection::TeachOrientation::EulerZYX;
    throw std::runtime_error("未知的姿态格式");
}
QVariantList values(const collection::FlangePose& pose) {
    QVariantList result;
    for (double value : pose) result << value;
    return result;
}
collection::FlangePose pose(const QVariantList& list) {
    if (list.size() != 6) throw std::runtime_error("没有完整的 TCP 位姿");
    collection::FlangePose result;
    for (int i = 0; i < 6; ++i) {
        bool ok = false;
        result[i] = list[i].toDouble(&ok);
        if (!ok || !std::isfinite(result[i])) throw std::runtime_error("TCP 位姿包含无效数值");
    }
    return result;
}
QString perpendicular(const QString& direction) {
    if (direction == "right") return "down";
    if (direction == "down") return "left";
    if (direction == "left") return "up";
    if (direction == "up") return "right";
    return "none";
}
bool horizontal(const QString& direction) { return direction == "left" || direction == "right"; }
QString writePreferences(const QVariantMap& preferences) {
    QSettings settings(QSettings::IniFormat, QSettings::UserScope, "Caitu", "Teaching");
    for (auto it = preferences.begin(); it != preferences.end(); ++it) settings.setValue(it.key(), it.value());
    settings.sync();
    return settings.status() == QSettings::NoError ? QString() : QString("示教偏好保存失败，请检查用户配置目录权限");
}
}

bool CollectionController::tcpPoseFresh() const {
    return m_tcpPoseValid && m_robotConnected && m_tcpPoseTool == m_selectedTcp &&
        m_tcpPoseAge.isValid() && m_tcpPoseAge.elapsed() + m_tcpPoseDeliveryLagMs <= 1000;
}

QString CollectionController::tcpPoseStatus() const {
    if (!m_tcpPoseValid) return m_tcpPoseStatus;
    if (!tcpPoseFresh()) return "TCP 位姿数据已过期，等待新数据";
    try { collection::tcpPoseForDisplay(pose(m_tcpPoseValues), orientation(m_orientationMode)); }
    catch (const std::exception& error) { return QString::fromUtf8(error.what()); }
    return m_orientationMode == "euler" ? "BASE 坐标 · mm / 欧拉角 ZYX °" : "BASE 坐标 · mm / 轴角 °";
}

QVariantList CollectionController::tcpDisplayPoseValues() const {
    if (!tcpPoseFresh()) return {};
    try { return values(collection::tcpPoseForDisplay(pose(m_tcpPoseValues), orientation(m_orientationMode))); }
    catch (const std::exception&) { return {}; }
}

QVariantMap CollectionController::convertTcpPose(const QString& text, const QString& from, const QString& to) const {
    try {
        const auto source = collection::parseTcpTarget(text);
        const auto axisAngle = collection::tcpPoseFromDisplay(source, orientation(from));
        return {{"valid", true}, {"error", QString()},
            {"values", values(collection::tcpPoseForDisplay(axisAngle, orientation(to)))}};
    } catch (const std::exception& error) {
        return {{"valid", false}, {"error", QString::fromUtf8(error.what())}, {"values", QVariantList()}};
    }
}

void CollectionController::setOrientationMode(const QString& mode) {
    if (m_collectionBusy || m_orientationMode == mode) return;
    try { orientation(mode); }
    catch (const std::exception& error) { setStatus(QString::fromUtf8(error.what())); return; }
    m_orientationMode = mode;
    emit teachSettingsChanged();
    emit tcpPoseChanged();
    scheduleTeachPreferences();
}

void CollectionController::setManualSpeeds(double linear, double angular) {
    if (m_collectionBusy) return;
    const collection::TeachSpeeds next{linear, angular};
    if (!collection::validTeachSpeeds(next)) {
        setStatus("平移速度须为 1～1000 mm/s，角度步进须为 0.1～180 °/s"); return;
    }
    if (m_teachSpeeds.linearMmS == linear && m_teachSpeeds.angularDegS == angular) return;
    m_teachSpeeds = next;
    emit teachSettingsChanged();
    scheduleTeachPreferences();
}

void CollectionController::setPreviewDirections(const QString& x, const QString& y) {
    const auto defaultY = perpendicular(x);
    if (x != "none" && defaultY == "none") { setStatus("无效的预览方向"); return; }
    const auto nextY = x == "none" ? QString("none") :
        perpendicular(y) != "none" && horizontal(x) != horizontal(y) ? y : defaultY;
    if (m_previewPositiveX == x && m_previewPositiveY == nextY) return;
    m_previewPositiveX = x;
    m_previewPositiveY = nextY;
    emit previewDirectionsChanged();
    scheduleTeachPreferences();
}

void CollectionController::copyCurrentTcpCoordinates(const QString& mode) {
    if (!tcpPoseFresh()) { setStatus("当前 TCP 位姿不可用或已过期，未复制"); return; }
    try {
        const auto current = collection::tcpPoseForDisplay(pose(m_tcpPoseValues), orientation(mode));
        QStringList parts;
        for (double value : current) parts << QString::number(value, 'g', 17);
        QGuiApplication::clipboard()->setText(parts.join(','));
        setStatus("已复制当前 TCP 坐标：X,Y,Z,Rx,Ry,Rz");
    } catch (const std::exception& error) { setStatus(QString::fromUtf8(error.what())); }
}

void CollectionController::moveTcpTo(const QString& text) { moveTcpToDisplay(text, "axis"); }

void CollectionController::moveTcpToDisplay(const QString& text, const QString& mode) {
    if (m_collectionBusy) return;
    try {
        const auto representation = orientation(mode);
        const auto target = collection::tcpPoseFromDisplay(collection::parseTcpTarget(text), representation);
        const auto speeds = m_teachSpeeds;
        startManualMotion([this, target, speeds, representation](RobotHandle handle, const QString& tool, collection::MotionTick tick) {
            collection::moveSelectedTcpTarget(handle, tool, target, speeds, m_cancelRequested,
                {1200, 180000}, tick, representation);
        }, "正在检查目标并移动所选 TCP");
    } catch (const std::exception& error) { setStatus(QString::fromUtf8(error.what())); }
}

qulonglong CollectionController::beginTcpJog(int component, int direction, const QString& mode) {
    if (m_collectionBusy) return 0;
    if (!tcpPoseFresh()) { setStatus("等待当前所选 TCP 的新位姿后再微调"); return 0; }
    try {
        if (component < 0 || component >= 6 || (direction != -1 && direction != 1))
            throw std::runtime_error("无效的微调轴或方向");
        const collection::TeachJogRequest request{component, direction, orientation(mode), m_teachSpeeds};
        const auto lease = std::make_shared<collection::TeachJogLease>();
        m_jogLease = lease;
        ++m_jogToken;
        const QString axis = QStringList{"X", "Y", "Z", "Rx", "Ry", "Rz"}[component];
        startManualMotion([this, lease, request](RobotHandle handle, const QString& tool, collection::MotionTick tick) {
            collection::jogSelectedTcp(handle, tool, request, *lease, m_cancelRequested, tick);
            // A released lease between segments still requires a stop request
            // and independently observed stationary state before unlocking.
            const auto result = cr_stop(handle);
            if (result != sucess)
                throw std::runtime_error(QString("停止指令失败（错误码 %1），正在独立确认停止状态").arg(result).toStdString());
        }, QString("长按微调 %1%2").arg(axis, direction > 0 ? "+" : "−"));
        if (!m_manualMoveActive) { lease->release(); m_jogLease.reset(); return 0; }
        return m_jogToken;
    } catch (const std::exception& error) {
        if (m_jogLease) m_jogLease->release();
        m_jogLease.reset();
        setStatus(QString::fromUtf8(error.what()));
        return 0;
    }
}

void CollectionController::renewTcpJog(qulonglong token) {
    if (token && token == m_jogToken && m_jogLease) m_jogLease->heartbeat();
}

void CollectionController::stopTcpJog(qulonglong token) {
    if (!token || token != m_jogToken || !m_jogLease) return;
    m_jogLease->release();
    cancelCollectionOperation();
}

void CollectionController::startManualMotion(
    std::function<void(RobotHandle, const QString&, collection::MotionTick)> motion, const QString& description) {
    if (m_collectionBusy) return;
    if (!m_robotConnected) { setStatus("请先连接机器人再移动 TCP"); return; }
    if (m_selectedTcp.isEmpty() || !m_tcpNames.contains(m_selectedTcp)) {
        setStatus("请选择有效的 TCP 工具"); return;
    }
    const auto session = robotSession();
    const auto tool = m_selectedTcp;
    beginCollectionOperation();
    const auto sample = tcpSamplingCallback(session);
    m_manualMoveActive = true;
    m_manualMoveProgress = description + "：" + tool;
    updatePreviewTimer(); // Robot-only teaching does not use the camera.
    emit collectionChanged();
    setStatus(m_manualMoveProgress);
    try {
        m_collectionThread = std::thread([this, session, tool, sample, motion] {
            auto lock = session->acquire(m_cancelRequested);
            QString message = "TCP 移动已取消";
            if (lock.owns_lock() && !m_cancelRequested) {
                m_robotStopUnconfirmed = true;
                bool failed = false;
                try {
                    motion(session->handle, tool, sample);
                    message = "TCP 操作完成";
                } catch (const std::exception& error) { message = QString::fromUtf8(error.what()); failed = true; }
                catch (...) { message = "TCP 移动发生未知异常"; failed = true; }
                if (failed) {
                    // Cancellation may arrive during pacing, between completed
                    // segments, outside the linear move's own stop handler.
                    const auto result = cr_stop(session->handle);
                    if (result != sucess) message += QString("；停止指令失败（错误码 %1）").arg(result);
                }
                try { confirmMotionStopped(); }
                catch (const std::exception& error) { message += "；" + QString::fromUtf8(error.what()); }
            }
            QMetaObject::invokeMethod(this, [this, message] {
                m_manualMoveProgress = message;
                finishCollectionOperation(message);
            }, Qt::QueuedConnection);
        });
    } catch (const std::exception& error) { finishCollectionOperation(QString::fromUtf8(error.what())); }
}

void CollectionController::loadTeachPreferences() {
    QSettings settings(QSettings::IniFormat, QSettings::UserScope, "Caitu", "Teaching");
    const collection::TeachSpeeds speeds{settings.value("linear_mm_s", 10).toDouble(),
        settings.value("angular_step_deg_s", 5).toDouble()};
    if (collection::validTeachSpeeds(speeds)) m_teachSpeeds = speeds;
    m_orientationMode = settings.value("orientation", "axis") == "euler" ? "euler" : "axis";
    const auto x = settings.value("preview_x", "none").toString();
    const auto y = settings.value("preview_y", "none").toString();
    m_previewPositiveX = perpendicular(x) == "none" ? "none" : x;
    m_previewPositiveY = m_previewPositiveX == "none" ? QString("none") :
        perpendicular(y) != "none" && horizontal(x) != horizontal(y) ? y : perpendicular(x);
    emit teachSettingsChanged();
    emit previewDirectionsChanged();
    emit tcpPoseChanged();
}

void CollectionController::scheduleTeachPreferences() {
    m_teachPreferencesDirty = true;
    if (!m_teachPreferencesTimer) {
        m_teachPreferencesTimer = new QTimer(this);
        m_teachPreferencesTimer->setSingleShot(true);
        m_teachPreferencesTimer->setInterval(250);
        connect(m_teachPreferencesTimer, &QTimer::timeout, this, &CollectionController::flushTeachPreferences);
    }
    m_teachPreferencesTimer->start();
}

void CollectionController::flushTeachPreferences() {
    if (m_teachPreferencesWrite.valid()) {
        if (m_teachPreferencesWrite.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            m_teachPreferencesTimer->start(); return;
        }
        const auto error = m_teachPreferencesWrite.get();
        if (!error.isEmpty()) setStatus(error);
    }
    if (!m_teachPreferencesDirty) return;
    const QVariantMap preferences{{"linear_mm_s", m_teachSpeeds.linearMmS},
        {"angular_step_deg_s", m_teachSpeeds.angularDegS}, {"orientation", m_orientationMode},
        {"preview_x", m_previewPositiveX}, {"preview_y", m_previewPositiveY}};
    try {
        m_teachPreferencesWrite = std::async(std::launch::async, [preferences] { return writePreferences(preferences); });
        m_teachPreferencesDirty = false;
        m_teachPreferencesTimer->start();
    } catch (const std::exception& error) { setStatus("无法后台保存示教偏好：" + QString::fromUtf8(error.what())); }
}

void CollectionController::finishTeachPreferences() {
    if (!m_teachPreferencesTimer) return;
    m_teachPreferencesTimer->stop();
    // Shutdown only: drain the single writer, then flush the latest coalesced
    // snapshot. During interaction all settings I/O stays off the GUI thread.
    if (m_teachPreferencesWrite.valid()) m_teachPreferencesWrite.wait();
    flushTeachPreferences();
    if (m_teachPreferencesWrite.valid()) {
        const auto error = m_teachPreferencesWrite.get();
        if (!error.isEmpty()) APP_LOG_EVENT_WARNING("teach", "preferences", "save_failed", {{"error", error}});
    }
    m_teachPreferencesTimer->stop();
}
