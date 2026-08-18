#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QMessageBox>
#include <QDir>
#include <QFile>
#include <QDebug>
#include <QSysInfo>

#include <opencv2/core/version.hpp>

#include "auth_check.hpp"
#include "CalibrationController.hpp"
#include "CameraImageProvider.hpp"
#include "DiagnosticLogger.hpp"

int main(int argc, char* argv[])
{
    qputenv("QT_LOGGING_RULES", "qt.qml.connections=false");

    QGuiApplication app(argc, argv);
    app.setApplicationName("hand-eye-app");
    app.setOrganizationName("caitu");

    DiagnosticLogger& logger = DiagnosticLogger::instance();
    logger.initialize();
    APP_LOG_INFO("lifecycle",
        QString("application_start build_type=%1 build_time=%2 git=%3 "
                "qt=%4 opencv=%5 kernel=%6 arch=%7 executable=%8 cwd=%9")
            .arg(QStringLiteral(APP_BUILD_TYPE))
            .arg(QStringLiteral(APP_BUILD_TIMESTAMP))
            .arg(QStringLiteral(APP_GIT_COMMIT))
            .arg(QString::fromLatin1(qVersion()))
            .arg(QString::fromLatin1(CV_VERSION))
            .arg(QSysInfo::kernelVersion())
            .arg(QSysInfo::currentCpuArchitecture())
            .arg(QCoreApplication::applicationFilePath())
            .arg(QDir::currentPath()));
    APP_LOG_INFO("lifecycle", QString("diagnostic_log=%1").arg(logger.logFilePath()));
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

    // ---- 创建控制器和图像提供器 ----
    APP_LOG_INFO("lifecycle", "creating_controller_and_image_provider");
    auto controller = new CalibrationController(&app);
    auto imageProvider = new CameraImageProvider();

    controller->init(imageProvider);

    // ---- QML 引擎 ----
    QQmlApplicationEngine engine;

    engine.rootContext()->setContextProperty("calibrationController", controller);
    engine.addImageProvider("camera", imageProvider);
    APP_LOG_INFO("qml", "loading qrc:///qml/HandEyeCalibration.qml");
    engine.load(QUrl(QStringLiteral("qrc:///qml/HandEyeCalibration.qml")));

    if (engine.rootObjects().isEmpty()) {
        qCritical() << "QML 加载失败!";
        return -1;
    }

    APP_LOG_INFO("qml", QString("qml_loaded root_objects=%1").arg(engine.rootObjects().size()));

    const int exitCode = app.exec();
    APP_LOG_INFO("lifecycle", QString("event_loop_exited code=%1").arg(exitCode));
    logger.flush();
    return exitCode;
}
