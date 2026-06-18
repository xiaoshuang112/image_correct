#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QMessageBox>
#include <QDir>
#include <QFile>
#include <QDebug>

#include "auth_check.hpp"
#include "CalibrationController.hpp"
#include "CameraImageProvider.hpp"

int main(int argc, char* argv[])
{
    qputenv("QT_LOGGING_RULES", "qt.qml.connections=false");

    QGuiApplication app(argc, argv);
    app.setApplicationName("hand-eye-app");
    app.setOrganizationName("caitu");


#ifndef X86_64    
    // ================================================================
    //  授权检查
    // ================================================================
    std::string appPath = QCoreApplication::applicationDirPath()
                              .toStdString() + "/";
    std::string err = check_authorization(appPath);
    if (!err.empty()) {
        qCritical() << "[auth] 授权检查失败:" << err.c_str();
        QMessageBox box(QMessageBox::Critical,
                        QString::fromUtf8("授权检查失败"),
                        QString::fromUtf8(err.c_str()),
                        QMessageBox::Ok);
        box.exec();
        return -1;
    }
#endif    

    // ---- 创建控制器和图像提供器 ----
    auto controller = new CalibrationController(&app);
    auto imageProvider = new CameraImageProvider();

    controller->init(imageProvider);

    // ---- QML 引擎 ----
    QQmlApplicationEngine engine;

    engine.rootContext()->setContextProperty("calibrationController", controller);
    engine.addImageProvider("camera", imageProvider);
    engine.load(QUrl(QStringLiteral("qrc:///qml/HandEyeCalibration.qml")));

    if (engine.rootObjects().isEmpty()) {
        qCritical() << "QML 加载失败!";
        return -1;
    }

    return app.exec();
}
