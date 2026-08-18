#ifndef DIAGNOSTIC_LOGGER_HPP
#define DIAGNOSTIC_LOGGER_HPP

#ifdef QT_GUI

#include <QString>

#include <functional>
#include <memory>
#include <mutex>

class CapturingStreambuf;

class DiagnosticLogger final
{
public:
    enum class Level {
        Debug,
        Info,
        Warning,
        Error,
        Fatal
    };

    static DiagnosticLogger& instance();

    bool initialize();
    void log(Level level,
             const char* category,
             const QString& message,
             const char* file = nullptr,
             int line = 0,
             const char* function = nullptr);
    void flush();

    QString logFilePath() const;
    QString logDirectory() const;

    // The sink is called outside the file mutex and may be invoked by any thread.
    void setUiSink(std::function<void(const QString&)> sink);

private:
    DiagnosticLogger();
    ~DiagnosticLogger() = default;
    DiagnosticLogger(const DiagnosticLogger&) = delete;
    DiagnosticLogger& operator=(const DiagnosticLogger&) = delete;

    bool openLogFileLocked(const QString& directory);
    void rotateLogLocked();
    void installConsoleCaptureLocked();
    void installFailureHandlers();

    static constexpr qint64 kMaxLogBytes = 10 * 1024 * 1024;
    static constexpr int kArchiveCount = 5;

    mutable std::mutex m_mutex;
    std::mutex m_sinkMutex;
    bool m_initialized = false;
    bool m_previousSessionUnclean = false;
    int m_fd = -1;
    qint64 m_fileSize = 0;
    qint64 m_startMonotonicMs = 0;
    QString m_logDirectory;
    QString m_logFilePath;
    std::function<void(const QString&)> m_uiSink;
    std::unique_ptr<CapturingStreambuf> m_stdoutCapture;
    std::unique_ptr<CapturingStreambuf> m_stderrCapture;
};

#define APP_LOG_DEBUG(category, message) \
    DiagnosticLogger::instance().log(DiagnosticLogger::Level::Debug, category, message, __FILE__, __LINE__, __func__)
#define APP_LOG_INFO(category, message) \
    DiagnosticLogger::instance().log(DiagnosticLogger::Level::Info, category, message, __FILE__, __LINE__, __func__)
#define APP_LOG_WARNING(category, message) \
    DiagnosticLogger::instance().log(DiagnosticLogger::Level::Warning, category, message, __FILE__, __LINE__, __func__)
#define APP_LOG_ERROR(category, message) \
    DiagnosticLogger::instance().log(DiagnosticLogger::Level::Error, category, message, __FILE__, __LINE__, __func__)
#define APP_LOG_FATAL(category, message) \
    DiagnosticLogger::instance().log(DiagnosticLogger::Level::Fatal, category, message, __FILE__, __LINE__, __func__)

#else

// 非 Qt 目标继续使用各模块原有的 stdout/stderr 输出，不引入 Qt 依赖。
// 参数不展开，因此调用处的 QString 格式化表达式也不会参与编译。
#define APP_LOG_DEBUG(category, message) do { } while (false)
#define APP_LOG_INFO(category, message) do { } while (false)
#define APP_LOG_WARNING(category, message) do { } while (false)
#define APP_LOG_ERROR(category, message) do { } while (false)
#define APP_LOG_FATAL(category, message) do { } while (false)

#endif // QT_GUI

#endif // DIAGNOSTIC_LOGGER_HPP
