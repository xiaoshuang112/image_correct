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
    app.setApplicationName("hand-eye-app");
    app.setOrganizationName("caitu");

    DiagnosticLogger& logger = DiagnosticLogger::instance();
    logger.initialize();
    APP_LOG_INFO("lifecycle",
        QString("application_start build_type=%1 build_time=%2 git=%3 git_state=%4 "
                "qt=%5 opencv=%6 kernel=%7 arch=%8 executable=%9 cwd=%10")
            .arg(QStringLiteral(APP_BUILD_TYPE))
            .arg(QStringLiteral(APP_BUILD_TIMESTAMP))
            .arg(QStringLiteral(APP_GIT_COMMIT))
            .arg(QStringLiteral(APP_GIT_WORKTREE_STATE))
            .arg(QString::fromLatin1(qVersion()))
            .arg(QString::fromLatin1(CV_VERSION))
            .arg(QSysInfo::kernelVersion())
            .arg(QSysInfo::currentCpuArchitecture())
            .arg(QCoreApplication::applicationFilePath())
            .arg(QDir::currentPath()));
    APP_LOG_INFO("lifecycle", QString("diagnostic_log=%1").arg(logger.logFilePath()));
    APP_LOG_INFO("graphics", QString("startup platform=%1 quick_backend=%2 display=%3 wayland_display=%4 "
                                      "software_requested=%5 automatic_fallback=%6")
        .arg(QString::fromLocal8Bit(qgetenv("QT_QPA_PLATFORM")))
        .arg(QString::fromLocal8Bit(qgetenv("QT_QUICK_BACKEND")))
        .arg(QString::fromLocal8Bit(qgetenv("DISPLAY")))
        .arg(QString::fromLocal8Bit(qgetenv("WAYLAND_DISPLAY")))
        .arg(forceSoftware)
        .arg(automaticSoftwareFallback));
    QObject::connect(&app, &QCoreApplication::aboutToQuit, []() {
        APP_LOG_INFO("lifecycle", "application_about_to_quit");
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
        APP_LOG_INFO("lifecycle", "creating_controller_and_image_provider");
        QQmlApplicationEngine engine;
        auto imageProvider = new CameraImageProvider();
        auto controller = std::make_unique<CalibrationController>();

        controller->init(imageProvider);

        engine.rootContext()->setContextProperty("calibrationController", controller.get());
        engine.addImageProvider("camera", imageProvider);
        APP_LOG_INFO("qml", "loading qrc:///qml/HandEyeCalibration.qml");
        engine.load(QUrl(QStringLiteral("qrc:///qml/HandEyeCalibration.qml")));

        if (engine.rootObjects().isEmpty()) {
            qCritical() << "QML 加载失败!";
        } else {
            APP_LOG_INFO("qml", QString("qml_loaded root_objects=%1").arg(engine.rootObjects().size()));
            exitCode = app.exec();
            APP_LOG_INFO("lifecycle", QString("event_loop_exited code=%1").arg(exitCode));
        }

        APP_LOG_INFO("lifecycle", QString("controller_shutdown_begin running=%1")
            .arg(controller->running()));
        controller.reset();
        APP_LOG_INFO("lifecycle", "controller_shutdown_complete image_provider_still_alive=1");
    }
    APP_LOG_INFO("qml", "qml_engine_destroyed");
    APP_LOG_INFO("lifecycle", QString("application_shutdown_complete code=%1").arg(exitCode));
    logger.flush();
    return exitCode;
}
