#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QMessageBox>
#include <QDir>
#include <QFile>
#include <QDebug>
#include <QSysInfo>

#include <opencv2/core/version.hpp>

#include <memory>

#include "auth_check.hpp"
#include "CalibrationController.hpp"
#include "CameraImageProvider.hpp"
#include "DiagnosticLogger.hpp"
#include "AppBuildInfo.hpp"

#ifdef useMVS
#include "MvCameraControl.h"
#endif

int main(int argc, char* argv[])
{
    qputenv("QT_LOGGING_RULES", "qt.qml.connections=false");

    // Qt Quick 在无显示服务或 offscreen 平台上仍可能尝试创建 OpenGL
    // 上下文，并在失败时直接 qFatal/SIGABRT。仅在这类环境中自动使用
    // 软件后端；现场仍可通过显式 QT_QUICK_BACKEND 覆盖。
    const QByteArray platform = qgetenv("QT_QPA_PLATFORM");
    const bool noDisplayServer = qEnvironmentVariableIsEmpty("DISPLAY") &&
                                 qEnvironmentVariableIsEmpty("WAYLAND_DISPLAY");
    const bool forceSoftware = qEnvironmentVariableIntValue(
                                   "CAITU_FORCE_SOFTWARE_RENDERING") != 0;
    const bool automaticSoftwareFallback = platform == "offscreen" || noDisplayServer;
    if ((forceSoftware || automaticSoftwareFallback) &&
        qEnvironmentVariableIsEmpty("QT_QUICK_BACKEND")) {
        qputenv("QT_QUICK_BACKEND", "software");
    }

    QGuiApplication app(argc, argv);
    app.setApplicationName("collect-app");
    app.setOrganizationName("caitu");
    app.setApplicationVersion(QStringLiteral(APP_VERSION));

    DiagnosticLogger& logger = DiagnosticLogger::instance();
    logger.initialize();
    APP_LOG_EVENT_INFO("lifecycle",
                       "application",
                       "start",
                       {{"version", QStringLiteral(APP_VERSION)},
                        {"build_type", QStringLiteral(APP_BUILD_TYPE)},
                        {"build_time", QStringLiteral(APP_BUILD_TIMESTAMP)},
                        {"git", QStringLiteral(APP_GIT_COMMIT)},
                        {"git_state", QStringLiteral(APP_GIT_WORKTREE_STATE)},
                        {"camera_backend", QStringLiteral(APP_CAMERA_BACKEND)},
#ifdef useMVS
                        {"mvs_sdk_version", QString::number(MV_CC_GetSDKVersion(), 16)},
#endif
                        {"qt", QString::fromLatin1(qVersion())},
                        {"opencv", QString::fromLatin1(CV_VERSION)},
                        {"kernel", QSysInfo::kernelVersion()},
                        {"arch", QSysInfo::currentCpuArchitecture()},
                        {"executable", QCoreApplication::applicationFilePath()},
                        {"cwd", QDir::currentPath()}});
    APP_LOG_EVENT_INFO("lifecycle", "diagnostic", "ready",
                       {{"log", logger.logFilePath()}});
    APP_LOG_EVENT_INFO("graphics",
                       "render",
                       "startup",
                       {{"platform", QString::fromLocal8Bit(qgetenv("QT_QPA_PLATFORM"))},
                        {"quick_backend", QString::fromLocal8Bit(qgetenv("QT_QUICK_BACKEND"))},
                        {"display", QString::fromLocal8Bit(qgetenv("DISPLAY"))},
                        {"wayland_display", QString::fromLocal8Bit(qgetenv("WAYLAND_DISPLAY"))},
                        {"software_requested", QString::number(forceSoftware)},
                        {"automatic_fallback", QString::number(automaticSoftwareFallback)}});
    QObject::connect(&app, &QCoreApplication::aboutToQuit, []() {
        APP_LOG_EVENT_INFO("lifecycle", "application", "about_to_quit", {});
        DiagnosticLogger::instance().flush();
    });


#ifndef X86_64    
    // ================================================================
    //  授权检查
    // ================================================================
    // std::string appPath = QCoreApplication::applicationDirPath()
    //                           .toStdString() + "/";
    // std::string err = check_authorization(appPath);
    // if (!err.empty()) {
    //     qCritical() << "[auth] 授权检查失败:" << err.c_str();
    //     QMessageBox box(QMessageBox::Critical,
    //                     QString::fromUtf8("授权检查失败"),
    //                     QString::fromUtf8(err.c_str()),
    //                     QMessageBox::Ok);
    //     box.exec();
    //     return -1;
    // }
#endif    

    int exitCode = -1;
    {
        // ---- QML 引擎、控制器和图像提供器 ----
        // engine 必须先构造：局部变量按逆序析构，使 controller 先停止并
        // join 标定线程，之后 engine 才销毁其拥有的 imageProvider。
        APP_LOG_EVENT_INFO("lifecycle", "component", "initializing",
                           {{"scope", "controller_and_image_provider"}});
        QQmlApplicationEngine engine;
        auto imageProvider = new CameraImageProvider();
        auto controller = std::make_unique<CalibrationController>();
        const bool collectionMode = !app.arguments().contains("--calibration");
        controller->setCollectionMode(collectionMode);
        controller->init(imageProvider);

        engine.rootContext()->setContextProperty("calibrationController", controller.get());
        engine.rootContext()->setContextProperty("appVersion", QStringLiteral(APP_VERSION));
        engine.rootContext()->setContextProperty("appGitCommit", QStringLiteral(APP_GIT_COMMIT).left(7));
        engine.addImageProvider("camera", imageProvider);
        const QString qmlSource = collectionMode ? QStringLiteral("qrc:///qml/Collection.qml")
                                                 : QStringLiteral("qrc:///qml/HandEyeCalibration.qml");
        APP_LOG_EVENT_INFO("qml", "qml", "loading", {{"source", qmlSource}});
        engine.load(QUrl(qmlSource));

        if (engine.rootObjects().isEmpty()) {
            qCritical() << "QML 加载失败!";
            APP_LOG_EVENT_ERROR("qml", "qml", "fail",
                                {{"source", qmlSource},
                                 {"stage", "load"}});
        } else {
            APP_LOG_EVENT_INFO("qml", "qml", "loaded",
                               {{"root_objects", QString::number(engine.rootObjects().size())}});
            exitCode = app.exec();
            APP_LOG_EVENT_INFO("lifecycle", "event_loop", "exit",
                               {{"code", QString::number(exitCode)}});
        }

        APP_LOG_EVENT_INFO("lifecycle", "controller", "shutdown_begin",
                           {{"running", QString::number(controller->running())}});
        controller.reset();
        APP_LOG_EVENT_INFO("lifecycle", "controller", "shutdown_complete",
                           {{"image_provider_alive", "true"}});
    }
    APP_LOG_EVENT_INFO("qml", "qml", "engine_destroyed", {});
    APP_LOG_EVENT_INFO("lifecycle", "application", "shutdown_complete",
                       {{"code", QString::number(exitCode)}});
    logger.flush();
    return exitCode;
}
