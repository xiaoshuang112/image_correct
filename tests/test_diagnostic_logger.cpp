// 诊断日志的跨平台回归：Unicode（中文）路径、轮转、崩溃诊断。
// 覆盖 Windows 移植里的平台抽象：CreateFileW UTF-16 路径、_open_osfhandle 写入句柄、
// _commit 刷盘，以及未处理异常过滤器 / POSIX 信号处理器写出的崩溃诊断。
#include "hand_eye_gui/DiagnosticLogger.hpp"
#include "test_check.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QProcess>
#include <QTemporaryDir>
#include <QString>
#include <QStringList>
#include <cstdlib>

namespace {

QString logPathIn(const QString& directory)
{
    return QDir(directory).filePath(QStringLiteral("hand-eye.log"));
}

QString readText(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return QString();
    return QString::fromUtf8(file.readAll());
}

int runCrashChild()
{
    DiagnosticLogger& logger = DiagnosticLogger::instance();
    if (!logger.initialize())
        return 2;
    logger.log(DiagnosticLogger::Level::Info, "crash_child", QStringLiteral("child_started"));
    logger.flush();
    // Windows 走未处理异常过滤器，Linux 走 SIGSEGV；崩溃路径不得分配内存或加锁。
    volatile char* unmapped = reinterpret_cast<volatile char*>(1);
    *unmapped = 1;
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);

    if (argc > 1 && QString::fromLocal8Bit(argv[1]) == QStringLiteral("crash-child"))
        return runCrashChild();

    QTemporaryDir temporary(QDir::tempPath() + QStringLiteral("/caitu-diag-XXXXXX"));
    if (!temporary.isValid())
        return 1;
    const QString root = temporary.path();
    const QString unicodeDirectory = QDir(root).filePath(QStringLiteral("日志测试"));
#ifdef Q_OS_WIN
    _wputenv_s(L"CAITU_LOG_DIR", unicodeDirectory.toStdWString().c_str());
#else
    qputenv("CAITU_LOG_DIR", unicodeDirectory.toLocal8Bit());
#endif

    DiagnosticLogger& logger = DiagnosticLogger::instance();
    if (!logger.initialize()) {
        testutil::report(false, "open unicode log directory");
        return testutil::takeFailures();
    }

    const QString logPath = logger.logFilePath();
    testutil::report(logPath == logPathIn(unicodeDirectory), "unicode log directory kept");
    logger.log(DiagnosticLogger::Level::Info, "unicode", QStringLiteral("中文路径写入正常"));
    logger.flush();
    testutil::report(readText(logPath).contains(QStringLiteral("中文路径写入正常")),
                     "unicode content written back as UTF-8");

    // 轮转：越过上限后当前文件重新开始，之前的内容留在归档里。
    const QString marker = QStringLiteral("rotation_marker");
    logger.log(DiagnosticLogger::Level::Info, "rotation", marker);
    const QString filler(8000, QLatin1Char('x'));
    for (int index = 0; index < 1300; ++index)
        logger.log(DiagnosticLogger::Level::Info, "rotation", filler);
    logger.log(DiagnosticLogger::Level::Info, "rotation", QStringLiteral("after_rotation"));
    logger.flush();

    const QString archived = readText(logPath + QStringLiteral(".1"));
    const QString current = readText(logPath);
    testutil::report(archived.contains(marker) && current.contains(QStringLiteral("after_rotation")),
                     "rotation archives previous content");
    // 轮转在“下一次写入会越界”时触发，所以归档恰好在限额内、且离限额不超过一条日志行；
    // 轮转后的当前文件必须重新从 0 计。
    testutil::report(archived.size() <= 10 * 1024 * 1024
                     && archived.size() > 10 * 1024 * 1024 - 16384
                     && current.size() < 10 * 1024 * 1024,
                     "rotated archive reaches the size limit and the new file restarts below it");

    // 崩溃诊断必须在处理器里直接写句柄，所以由子进程崩溃，测试进程本身不能崩。
    const QString crashDirectory = QDir(root).filePath(QStringLiteral("崩溃子进程"));
    QDir().mkpath(crashDirectory);
    QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
    environment.insert(QStringLiteral("CAITU_LOG_DIR"), crashDirectory);
    QProcess process;
    process.setProcessEnvironment(environment);
    process.start(QCoreApplication::applicationFilePath(), QStringList{QStringLiteral("crash-child")});
    if (!process.waitForStarted()) {
        testutil::report(false, "crash child did not start");
        return testutil::takeFailures();
    }
    if (!process.waitForFinished(20000)) {
        process.kill();
        process.waitForFinished();
        testutil::report(false, "crash child timed out");
        return 1;
    }
    const int exitCode = process.exitCode();
    const QString crashLog = readText(logPathIn(crashDirectory));
    testutil::report(exitCode >= 128, "crash child exited with the signal code");
    testutil::report(crashLog.contains(QStringLiteral("process_crash signal=")),
                     "crash report written through the platform fd");
    testutil::report(crashLog.contains(QStringLiteral("backtrace")),
                     "crash backtrace written through the platform fd");

    return testutil::takeFailures() == 0 ? 0 : 1;
}
