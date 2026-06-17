#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QDir>
#include <QFile>
#include <QDebug>

#include "CalibrationController.hpp"
#include "CameraImageProvider.hpp"

int main(int argc, char* argv[])
{
    // Qt5.15 默认 QML 运行时会发出一些无害警告，可按需关闭
    qputenv("QT_LOGGING_RULES", "qt.qml.connections=false");

    QGuiApplication app(argc, argv);
    app.setApplicationName("hand-eye-app");
    app.setOrganizationName("caitu");

    // ---- 创建控制器和图像提供器 ----
    auto controller = new CalibrationController(&app);
    auto imageProvider = new CameraImageProvider();

    controller->init(imageProvider);

    // ---- QML 引擎 ----
    QQmlApplicationEngine engine;

    // 注册单例
    engine.rootContext()->setContextProperty("calibrationController", controller);

    // 注册图像提供器
    engine.addImageProvider("camera", imageProvider);

    // 从 Qt 资源系统加载 QML（编译进二进制，不依赖外部文件）
    engine.load(QUrl(QStringLiteral("qrc:///qml/HandEyeCalibration.qml")));

    if (engine.rootObjects().isEmpty()) {
        qCritical() << "QML 加载失败!";
        return -1;
    }

    return app.exec();
}
