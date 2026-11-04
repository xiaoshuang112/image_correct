#include "CollectionController.hpp"
#include "RobotTcp.hpp"
#include "ManualTcpMotion.hpp"
#include "DiagnosticLogger.hpp"

#include <QDateTime>
#include <QPointer>
#include <chrono>
#include <stdexcept>

namespace {
constexpr qint64 poseFreshnessMs = 1000;

void prepareRobot(RobotHandle handle, const std::atomic<bool>& cancelled) {
    RobotModes mode;
    if (cr_get_robotMode(handle, &mode) != sucess)
        throw std::runtime_error("获取机械臂模式失败");
    // Preserve connection's existing power/enable behavior.
    if (mode != JointPowerOff) return;
    auto check = [](CRresult result, const char* error) {
        if (result != sucess) throw std::runtime_error(error);
    };
    auto waitFor = [&](RobotModes target) {
        for (int poll = 0; poll < 300; ++poll) {
            if (cancelled) throw std::runtime_error("连接准备已取消");
            check(cr_get_robotMode(handle, &mode), "等待机器人就绪时通信失败");
            if (mode == target) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        throw std::runtime_error("等待机器人上电或使能就绪超时");
    };
    check(cr_FaultReset(handle), "故障复位失败");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    check(cr_poweron(handle), "上电失败");
    waitFor(JointIdle);
    check(cr_enable(handle), "使能失败");
    waitFor(ProgramStop);
}
}

std::shared_ptr<CollectionController::RobotSession> CollectionController::robotSession() {
    if (!m_robotConnected || !m_robotHandle) return {};
    if (!m_robotSession) m_robotSession = std::make_shared<RobotSession>(m_robotHandle);
    return m_robotSession;
}

void CollectionController::connectRobot() {
    if (m_collectionBusy) { setStatus("操作中，不能更改机器人连接"); return; }
    if (m_robotConnected) { setStatus("机器人已连接"); return; }
    const auto ip = m_robotIp.toLocal8Bit();
    const auto password = m_robotPasswd.toLocal8Bit();
    const auto port = m_robotPort;
    beginCollectionOperation();
    invalidateTcpPose("正在连接机器人…");
    setStatus("正在连接机器人并检查上电使能状态…");
    try {
        m_collectionThread = std::thread([this, ip, password, port] {
            RobotHandle handle = 0;
            QString message;
            bool connected = false;
            bool prepared = false;
            try {
                const auto result = cr_create_robot(&handle, ip.constData(), port, password.constData());
                if (result != sucess)
                    throw std::runtime_error(QString("连接机器人失败（错误码 %1）").arg(result).toStdString());
                connected = true;
                m_pendingRobotHandle = handle;
                prepareRobot(handle, m_cancelRequested);
                prepared = true;
                message = "机器人已连接";
            } catch (const std::exception& error) { message = QString::fromUtf8(error.what()); }
            catch (...) { message = "机器人连接发生未知异常"; }
            if (!prepared) {
                APP_LOG_EVENT_ERROR("robot", "robot_connect", "failed",
                    {{"phase", connected ? "power_enable" : "connect"}, {"message", message}});
                if (connected) message = "机器人已连接；上电使能检查失败：" + message;
            } else {
                APP_LOG_EVENT_INFO("robot", "robot_connect", "ready",
                    {{"message", message}, {"handle", QString::number(static_cast<qulonglong>(handle))}});
            }
            QMetaObject::invokeMethod(this, [this, handle, connected, prepared, message] {
                m_pendingRobotHandle = 0;
                m_robotHandle = connected ? handle : 0;
                m_robotConnected = connected;
                if (connected) m_robotSession = std::make_shared<RobotSession>(handle);
                emit robotConnectedChanged();
                finishCollectionOperation(message);
                if (connected && prepared) refreshTcpTools();
                else invalidateTcpPose(connected ? "请在示教器确认上电使能状态后刷新 TCP 列表" : "机器人未连接");
            }, Qt::QueuedConnection);
        });
    } catch (const std::exception& error) { finishCollectionOperation(QString::fromUtf8(error.what())); }
}

void CollectionController::disconnectRobot() {
    if (m_robotDisconnecting) return;
    if (m_collectionBusy || m_robotStopUnconfirmed) {
        setStatus(m_motionFault ? "机器人停止未确认，请先点击“重试停止”"
                               : "请先停止当前操作并等待结束，再断开机器人");
        return;
    }
    const auto session = robotSession();
    if (!session) {
        invalidateTcpPose("机器人未连接");
        return;
    }
    beginCollectionOperation();
    m_robotDisconnecting = true;
    session->closing = true;
    invalidateTcpPose("正在断开机器人…");
    setStatus("正在等待机器人读取结束并断开连接…");
    const QPointer<CollectionController> guard(this);
    const auto delivery = m_robotDelivery;
    std::thread shutdown;
    try {
        shutdown = std::thread([session, guard, delivery] {
            CRresult result;
            {
                // A reader owns the handle until its SDK call returns. This wait
                // and destruction both run outside the GUI thread.
                std::lock_guard<std::mutex> lock(session->mutex);
                result = cr_destroy_robot(session->handle);
            }
            if (result != sucess)
                APP_LOG_EVENT_ERROR("robot", "robot_destroy", "failed_handle_retained", {{"error", QString::number(result)}});
            QMetaObject::invokeMethod(delivery.get(), [guard, session, result] {
                if (!guard || guard->m_robotSession != session) return;
                auto* controller = guard.data();
                controller->m_robotDisconnecting = false;
                if (result == sucess) {
                    controller->m_robotSession.reset();
                    controller->m_robotHandle = 0;
                    controller->m_robotConnected = false;
                    controller->m_tcpNames.clear();
                    controller->m_selectedTcp.clear();
                    controller->invalidateTcpPose("机器人未连接");
                    emit controller->robotConnectedChanged();
                    emit controller->tcpToolsChanged();
                    controller->finishCollectionOperation("机器人已断开");
                } else {
                    session->closing = false;
                    const auto message = QString("机器人断开失败（错误码 %1），连接已保留，可重试断开").arg(result);
                    controller->invalidateTcpPose(message);
                    controller->finishCollectionOperation(message);
                }
            }, Qt::QueuedConnection);
        });
    } catch (const std::exception& error) {
        session->closing = false;
        m_robotDisconnecting = false;
        const auto message = "无法启动后台断开，机器人连接已保留：" + QString::fromUtf8(error.what());
        invalidateTcpPose(message);
        finishCollectionOperation(message);
        APP_LOG_EVENT_ERROR("robot", "robot_disconnect", "launch_failed", {{"message", message}});
        return;
    }
    shutdown.detach();
}

void CollectionController::invalidateTcpPose(const QString& status) {
    const bool changed = m_tcpPoseValid || !m_tcpPoseValues.isEmpty() ||
        !m_tcpPoseTimestamp.isEmpty() || !m_tcpPoseTool.isEmpty() || m_tcpPoseStatus != status;
    m_tcpPoseValid = false;
    m_tcpPoseValues.clear();
    m_tcpPoseTimestamp.clear();
    m_tcpPoseTool.clear();
    m_tcpPoseStatus = status;
    if (changed) emit tcpPoseChanged();
}

void CollectionController::applyTcpSample(const std::shared_ptr<RobotSession>& session,
    quint64 generation, const QString& tool, const QVariantList& values,
    const QString& error, qint64 timestamp) {
    if (!m_robotConnected || m_robotSession != session || session->closing ||
        generation != m_tcpPoseGeneration || tool != m_selectedTcp) return;
    QString failure = error;
    if (failure.isEmpty() && (values.size() != ROB_AXIS_NUM ||
        QDateTime::currentMSecsSinceEpoch() - timestamp > poseFreshnessMs))
        failure = "TCP 位姿数据已过期，等待新数据";
    if (!failure.isEmpty()) {
        invalidateTcpPose(failure);
        if (m_tcpPoseLastFailure != failure) {
            m_tcpPoseLastFailure = failure;
            APP_LOG_EVENT_WARNING("robot", "tcp_telemetry", "unavailable",
                {{"tool", tool}, {"message", failure}});
        }
        return;
    }
    m_tcpPoseLastFailure.clear();
    m_tcpPoseValues = values;
    m_tcpPoseValid = true;
    m_tcpPoseStatus = "BASE 坐标 · mm / 轴角 °";
    m_tcpPoseTimestamp = QDateTime::fromMSecsSinceEpoch(timestamp).toString("yyyy-MM-dd HH:mm:ss.zzz");
    m_tcpPoseTool = tool;
    m_tcpPoseAge.start();
    if (m_manualMoveActive && !m_cancelRequested && m_manualMoveProgress != "TCP 移动 · 正在到位及稳定等待") {
        m_manualMoveProgress = "TCP 移动 · 正在到位及稳定等待";
        emit collectionChanged();
    }
    emit tcpPoseChanged();
}

std::function<void()> CollectionController::tcpSamplingCallback(const std::shared_ptr<RobotSession>& session) {
    if (!session || m_selectedTcp.isEmpty()) return {};
    const auto generation = m_tcpPoseGeneration;
    const auto tool = m_selectedTcp;
    const QPointer<CollectionController> guard(this);
    const auto delivery = m_robotDelivery;
    const auto lastRead = std::make_shared<std::chrono::steady_clock::time_point>();
    return [session, generation, tool, guard, delivery, lastRead] {
        const auto start = std::chrono::steady_clock::now();
        if (session->closing || start - *lastRead < std::chrono::milliseconds(200)) return;
        *lastRead = start;
        QVariantList values;
        QString error;
        qint64 timestamp = 0;
        try {
            const auto result = collection::readTcpPose(session->handle, tool);
            if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(poseFreshnessMs))
                throw std::runtime_error("TCP 位姿读取超时，等待新数据");
            for (double value : result.pose) values << value;
            timestamp = QDateTime::currentMSecsSinceEpoch();
        } catch (const std::exception& failure) { error = QString::fromUtf8(failure.what()); }
        catch (...) { error = "TCP 位姿读取发生未知异常"; }
        QMetaObject::invokeMethod(delivery.get(), [guard, session, generation, tool, values, error, timestamp] {
            if (guard) guard->applyTcpSample(session, generation, tool, values, error, timestamp);
        }, Qt::QueuedConnection);
    };
}

void CollectionController::pollTcpPose() {
    if (m_tcpPoseValid && m_tcpPoseAge.elapsed() > poseFreshnessMs)
        invalidateTcpPose("TCP 位姿数据已过期，等待新数据");
    if (!m_robotConnected || m_selectedTcp.isEmpty() || m_collectionBusy) return;
    const auto session = robotSession();
    if (!session || session->closing || session->telemetryInFlight.exchange(true)) return;
    const auto sample = tcpSamplingCallback(session);
    try {
        std::thread([session, sample] {
            {
                std::lock_guard<std::mutex> lock(session->mutex);
                if (!session->closing) sample();
            }
            session->telemetryInFlight = false;
        }).detach();
    } catch (const std::exception& error) {
        session->telemetryInFlight = false;
        invalidateTcpPose(QString::fromUtf8(error.what()));
    }
}

QVariantMap CollectionController::parseTcpTarget(const QString& text) const {
    QVariantList values;
    try {
        for (double value : collection::parseTcpTarget(text)) values << value;
        return {{"valid", true}, {"error", QString()}, {"values", values}};
    } catch (const std::exception& error) {
        return {{"valid", false}, {"error", QString::fromUtf8(error.what())}, {"values", values}};
    }
}

void CollectionController::moveTcpTo(const QString& text) {
    if (m_collectionBusy) return;
    collection::FlangePose target;
    try { target = collection::parseTcpTarget(text); }
    catch (const std::exception& error) { setStatus(QString::fromUtf8(error.what())); return; }
    if (!m_robotConnected) { setStatus("请先连接机器人再移动 TCP"); return; }
    if (m_selectedTcp.isEmpty() || !m_tcpNames.contains(m_selectedTcp)) {
        setStatus("请选择有效的 TCP 工具"); return;
    }
    const auto session = robotSession();
    const auto tool = m_selectedTcp;
    beginCollectionOperation();
    const auto sample = tcpSamplingCallback(session);
    m_manualMoveActive = true;
    m_manualMoveProgress = "正在检查目标并移动所选 TCP：" + tool;
    emit collectionChanged();
    setStatus(m_manualMoveProgress);
    try {
        m_collectionThread = std::thread([this, session, tool, target, sample] {
            auto lock = session->acquire(m_cancelRequested);
            if (!lock.owns_lock()) {
                QMetaObject::invokeMethod(this, [this] { finishCollectionOperation("TCP 移动已取消"); }, Qt::QueuedConnection);
                return;
            }
            QString message;
            if (m_cancelRequested) message = "TCP 移动已取消";
            else {
                // Conservative lock: any failed command/read must independently
                // prove stationary before editing or teardown becomes available.
                m_robotStopUnconfirmed = true;
                try {
                    collection::moveSelectedTcpLinear(session->handle, tool, target, m_cancelRequested, {1200, 180000}, sample);
                    message = "TCP 已到达目标，稳定等待完成";
                } catch (const std::exception& error) { message = QString::fromUtf8(error.what()); }
                catch (...) { message = "TCP 移动发生未知异常"; }
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
