#include "DiagnosticLogger.hpp"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>
#include <QStringList>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <new>
#include <streambuf>
#include <thread>

#include <execinfo.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace {

std::atomic<int> g_crashLogFd{-1};
std::atomic_flag g_handlingFatalSignal = ATOMIC_FLAG_INIT;

qint64 monotonicMilliseconds()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

const char* levelName(DiagnosticLogger::Level level)
{
    switch (level) {
    case DiagnosticLogger::Level::Debug:   return "DEBUG";
    case DiagnosticLogger::Level::Info:    return "INFO";
    case DiagnosticLogger::Level::Warning: return "WARN";
    case DiagnosticLogger::Level::Error:   return "ERROR";
    case DiagnosticLogger::Level::Fatal:   return "FATAL";
    }
    return "UNKNOWN";
}

DiagnosticLogger::Level qtLevel(QtMsgType type)
{
    switch (type) {
    case QtDebugMsg:    return DiagnosticLogger::Level::Debug;
    case QtInfoMsg:     return DiagnosticLogger::Level::Info;
    case QtWarningMsg:  return DiagnosticLogger::Level::Warning;
    case QtCriticalMsg: return DiagnosticLogger::Level::Error;
    case QtFatalMsg:    return DiagnosticLogger::Level::Fatal;
    }
    return DiagnosticLogger::Level::Info;
}

QString sanitizeMessage(QString message)
{
    message.replace('\r', "\\r");
    message.replace('\n', "\\n");
    return message;
}

QString sanitizeFieldValue(QString value)
{
    value.replace('"', "'");
    if (value.contains(' ') || value.contains('='))
        value.replace(' ', "_");
    return value;
}

void writeAll(int fd, const char* data, size_t length)
{
    while (length > 0) {
        const ssize_t written = ::write(fd, data, length);
        if (written <= 0)
            return;
        data += written;
        length -= static_cast<size_t>(written);
    }
}

void qtMessageHandler(QtMsgType type,
                      const QMessageLogContext& context,
                      const QString& message)
{
    const char* category = context.category ? context.category : "qt";
    DiagnosticLogger::instance().log(qtLevel(type), category, message,
                                     context.file, context.line, context.function);

    const QByteArray consoleLine =
        QString("[%1] %2\n")
            .arg(QString::fromLatin1(levelName(qtLevel(type))), message)
            .toLocal8Bit();
    writeAll(STDERR_FILENO, consoleLine.constData(),
             static_cast<size_t>(consoleLine.size()));
}

size_t appendLiteral(char* buffer, size_t offset, size_t capacity, const char* text)
{
    while (*text && offset < capacity)
        buffer[offset++] = *text++;
    return offset;
}

size_t appendUnsigned(char* buffer, size_t offset, size_t capacity,
                      unsigned long long value, unsigned base)
{
    char digits[32];
    size_t count = 0;
    do {
        const unsigned digit = static_cast<unsigned>(value % base);
        digits[count++] = static_cast<char>(digit < 10 ? '0' + digit : 'a' + digit - 10);
        value /= base;
    } while (value && count < sizeof(digits));

    while (count > 0 && offset < capacity)
        buffer[offset++] = digits[--count];
    return offset;
}

size_t appendSigned(char* buffer, size_t offset, size_t capacity, long long value)
{
    if (value < 0) {
        if (offset < capacity)
            buffer[offset++] = '-';
        const auto magnitude = static_cast<unsigned long long>(-(value + 1)) + 1;
        return appendUnsigned(buffer, offset, capacity, magnitude, 10);
    }
    return appendUnsigned(buffer, offset, capacity,
                          static_cast<unsigned long long>(value), 10);
}

const char* signalName(int signalNumber)
{
    switch (signalNumber) {
    case SIGSEGV: return "SIGSEGV";
    case SIGABRT: return "SIGABRT";
    case SIGFPE:  return "SIGFPE";
    case SIGILL:  return "SIGILL";
#ifdef SIGBUS
    case SIGBUS:  return "SIGBUS";
#endif
    default:      return "UNKNOWN";
    }
}

void fatalSignalHandler(int signalNumber, siginfo_t* info, void*)
{
    if (g_handlingFatalSignal.test_and_set())
        _exit(128 + signalNumber);

    char line[512];
    size_t length = 0;
    length = appendLiteral(line, length, sizeof(line),
                           "\n[FATAL] process_crash signal=");
    length = appendLiteral(line, length, sizeof(line), signalName(signalNumber));
    length = appendLiteral(line, length, sizeof(line), " pid=");
    length = appendUnsigned(line, length, sizeof(line),
                            static_cast<unsigned long long>(::getpid()), 10);
#ifdef SYS_gettid
    length = appendLiteral(line, length, sizeof(line), " tid=");
    length = appendUnsigned(line, length, sizeof(line),
                            static_cast<unsigned long long>(::syscall(SYS_gettid)), 10);
#endif
    if (info) {
        length = appendLiteral(line, length, sizeof(line), " code=");
        length = appendSigned(line, length, sizeof(line), info->si_code);
    }
    if (info && info->si_code > 0) {
        length = appendLiteral(line, length, sizeof(line), " address=0x");
        length = appendUnsigned(line, length, sizeof(line),
                                reinterpret_cast<uintptr_t>(info->si_addr), 16);
    }
    length = appendLiteral(line, length, sizeof(line), "\nbacktrace:\n");

    const int logFd = g_crashLogFd.load();
    if (logFd >= 0)
        writeAll(logFd, line, length);
    writeAll(STDERR_FILENO, line, length);

    void* frames[64];
    const int frameCount = ::backtrace(frames, 64);
    if (logFd >= 0)
        ::backtrace_symbols_fd(frames, frameCount, logFd);
    ::backtrace_symbols_fd(frames, frameCount, STDERR_FILENO);

    ::kill(::getpid(), signalNumber);
    _exit(128 + signalNumber);
}

void terminateHandler()
{
    QString reason = "std::terminate invoked";
    if (const auto exception = std::current_exception()) {
        try {
            std::rethrow_exception(exception);
        } catch (const std::exception& e) {
            reason += QString(": uncaught %1").arg(QString::fromUtf8(e.what()));
        } catch (...) {
            reason += ": uncaught non-standard exception";
        }
    }
    DiagnosticLogger::instance().log(DiagnosticLogger::Level::Fatal,
                                     "process", reason);
    DiagnosticLogger::instance().flush();
    std::abort();
}

void outOfMemoryHandler()
{
    static constexpr char message[] = "\n[FATAL] memory_allocation_failed\n";
    const int logFd = g_crashLogFd.load();
    if (logFd >= 0)
        writeAll(logFd, message, sizeof(message) - 1);
    writeAll(STDERR_FILENO, message, sizeof(message) - 1);
    std::abort();
}

} // namespace

class CapturingStreambuf final : public std::streambuf
{
public:
    CapturingStreambuf(std::streambuf* destination,
                       DiagnosticLogger::Level level,
                       const char* category)
        : m_destination(destination), m_level(level), m_category(category)
    {
    }

protected:
    int overflow(int character) override
    {
        if (character == traits_type::eof())
            return traits_type::not_eof(character);

        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_destination)
            m_destination->sputc(static_cast<char>(character));
        consumeCharacter(static_cast<char>(character));
        return character;
    }

    std::streamsize xsputn(const char* data, std::streamsize count) override
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_destination)
            m_destination->sputn(data, count);
        for (std::streamsize i = 0; i < count; ++i)
            consumeCharacter(data[i]);
        return count;
    }

    int sync() override
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_destination)
            m_destination->pubsync();
        return 0;
    }

private:
    void consumeCharacter(char character)
    {
        if (character == '\n' || character == '\r') {
            flushLine();
        } else {
            m_line.push_back(character);
            if (m_line.size() >= 8192)
                flushLine();
        }
    }

    void flushLine()
    {
        if (m_line.empty())
            return;
        DiagnosticLogger::instance().log(
            m_level, m_category, QString::fromLocal8Bit(m_line.data(),
                                                        static_cast<int>(m_line.size())));
        m_line.clear();
    }

    std::streambuf* m_destination = nullptr;
    DiagnosticLogger::Level m_level;
    const char* m_category = nullptr;
    std::mutex m_mutex;
    std::string m_line;
};

DiagnosticLogger& DiagnosticLogger::instance()
{
    // Deliberately retained until process exit so crash handlers and late static
    // destructors never reference an already-destroyed logger.
    static DiagnosticLogger* logger = new DiagnosticLogger();
    return *logger;
}

DiagnosticLogger::DiagnosticLogger()
    : m_startMonotonicMs(monotonicMilliseconds())
    , m_sessionId(QString("s_") + QDateTime::currentDateTimeUtc().toString("yyyyMMddHHmmsszzz")
                  + "_" + QString::number(::getpid()))
{
}

    bool DiagnosticLogger::initialize()
    {
    QString selectedDirectory;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_initialized)
            return m_fd >= 0;

        QStringList candidates;
        const QString environmentDirectory = qEnvironmentVariable("CAITU_LOG_DIR");
        if (!environmentDirectory.isEmpty())
            candidates << environmentDirectory;
        candidates << QDir(QCoreApplication::applicationDirPath()).filePath("logs");
        candidates << QDir(QStandardPaths::writableLocation(
                               QStandardPaths::AppLocalDataLocation)).filePath("logs");
        candidates << QDir::temp().filePath("caitu-hand-eye/logs");

        for (const QString& directory : candidates) {
            if (openLogFileLocked(directory)) {
                selectedDirectory = directory;
                break;
            }
        }

        m_initialized = true;
        installConsoleCaptureLocked();
    }

    qInstallMessageHandler(qtMessageHandler);
    installFailureHandlers();

    log(Level::Info, "lifecycle",
        QString("logging_initialized file=%1 max_file_mb=%2 archives=%3")
            .arg(logFilePath())
            .arg(kMaxLogBytes / (1024 * 1024))
            .arg(kArchiveCount));
    if (m_previousSessionUnclean) {
        // openLogFileLocked() 已将上一会话的 hand-eye.log 轮转为 .1。
        const QString previousLogPath = logFilePath() + QStringLiteral(".1");
        log(Level::Warning, "lifecycle",
            QString("previous_session_did_not_exit_cleanly previous_log=%1")
                .arg(previousLogPath));
    }
    return !selectedDirectory.isEmpty();
}

QString DiagnosticLogger::sessionId() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_sessionId;
}

bool DiagnosticLogger::openLogFileLocked(const QString& directory)
{
    if (directory.isEmpty() || !QDir().mkpath(directory))
        return false;

    m_logDirectory = QDir(directory).absolutePath();
    m_logFilePath = QDir(m_logDirectory).filePath("hand-eye.log");

    m_previousSessionUnclean = false;
    QFile previousLog(m_logFilePath);
    if (previousLog.exists() && previousLog.open(QIODevice::ReadOnly)) {
        const qint64 tailSize = std::min<qint64>(previousLog.size(), 8192);
        if (previousLog.seek(previousLog.size() - tailSize)) {
            const QByteArray tail = previousLog.readAll();
            m_previousSessionUnclean =
                !tail.contains("application_shutdown_complete");
        }
    }
    rotateLogLocked();

    const QByteArray nativePath = QFile::encodeName(m_logFilePath);
    m_fd = ::open(nativePath.constData(), O_CREAT | O_WRONLY | O_APPEND | O_CLOEXEC,
                  S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if (m_fd < 0) {
        m_logDirectory.clear();
        m_logFilePath.clear();
        return false;
    }

    const off_t offset = ::lseek(m_fd, 0, SEEK_END);
    m_fileSize = offset >= 0 ? static_cast<qint64>(offset) : 0;
    g_crashLogFd.store(m_fd);
    return true;
}

void DiagnosticLogger::rotateLogLocked()
{
    const QString base = m_logFilePath;
    QFile::remove(base + QString(".%1").arg(kArchiveCount));
    for (int index = kArchiveCount - 1; index >= 1; --index) {
        const QString from = base + QString(".%1").arg(index);
        const QString to = base + QString(".%1").arg(index + 1);
        if (QFileInfo::exists(from))
            QFile::rename(from, to);
    }
    if (QFileInfo::exists(base))
        QFile::rename(base, base + ".1");
}

void DiagnosticLogger::installConsoleCaptureLocked()
{
    if (!m_stdoutCapture) {
        m_stdoutCapture = std::make_unique<CapturingStreambuf>(
            std::cout.rdbuf(), Level::Info, "stdout");
        std::cout.rdbuf(m_stdoutCapture.get());
    }
    if (!m_stderrCapture) {
        m_stderrCapture = std::make_unique<CapturingStreambuf>(
            std::cerr.rdbuf(), Level::Error, "stderr");
        std::cerr.rdbuf(m_stderrCapture.get());
    }
}

void DiagnosticLogger::installFailureHandlers()
{
    // Warm up glibc's backtrace machinery before it is needed in a signal handler.
    void* warmup[1];
    ::backtrace(warmup, 1);

    static void* alternateStack = std::malloc(static_cast<size_t>(SIGSTKSZ) * 4);
    if (alternateStack) {
        stack_t stack{};
        stack.ss_sp = alternateStack;
        stack.ss_size = static_cast<size_t>(SIGSTKSZ) * 4;
        ::sigaltstack(&stack, nullptr);
    }

    struct sigaction action {};
    action.sa_sigaction = fatalSignalHandler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_SIGINFO | SA_RESETHAND | SA_ONSTACK;
    sigaction(SIGSEGV, &action, nullptr);
    sigaction(SIGABRT, &action, nullptr);
    sigaction(SIGFPE, &action, nullptr);
    sigaction(SIGILL, &action, nullptr);
#ifdef SIGBUS
    sigaction(SIGBUS, &action, nullptr);
#endif

    std::set_terminate(terminateHandler);
    std::set_new_handler(outOfMemoryHandler);
}

void DiagnosticLogger::log(Level level,
                           const char* category,
                           const QString& message,
                           const char* file,
                           int line,
                           const char* function)
{
    const qint64 elapsed = monotonicMilliseconds() - m_startMonotonicMs;
    const qint64 threadId = static_cast<qint64>(::syscall(SYS_gettid));
    const QString sourceFile = file
        ? QFileInfo(QString::fromLocal8Bit(file)).fileName()
        : QStringLiteral("-");
    const QString sourceFunction = function
        ? QString::fromLocal8Bit(function)
        : QStringLiteral("-");
    const qint64 sequence = m_sequence.fetch_add(1, std::memory_order_relaxed) + 1;
    const QString safeMessage = sanitizeMessage(message);
    const QString sessionId = m_sessionId.isEmpty() ? QStringLiteral("-") : m_sessionId;

    const QString formatted =
        QString("%1 +%2ms [%3] [pid=%4 tid=%5] [run=%6 seq=%7] [%8] %9:%10 %11 | %12")
            .arg(QDateTime::currentDateTime().toString(Qt::ISODateWithMs))
            .arg(elapsed, 8, 10, QLatin1Char('0'))
            .arg(QString::fromLatin1(levelName(level)))
            .arg(QCoreApplication::applicationPid())
            .arg(threadId)
            .arg(sessionId)
            .arg(sequence)
            .arg(QString::fromLatin1(category ? category : "app"))
            .arg(sourceFile)
            .arg(line)
            .arg(sourceFunction)
            .arg(safeMessage);
    const QByteArray bytes = (formatted + '\n').toUtf8();

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_fd >= 0) {
            if (m_fileSize + bytes.size() > kMaxLogBytes) {
                g_crashLogFd.store(-1);
                ::close(m_fd);
                m_fd = -1;
                rotateLogLocked();
                const QByteArray nativePath = QFile::encodeName(m_logFilePath);
                m_fd = ::open(nativePath.constData(),
                              O_CREAT | O_WRONLY | O_APPEND | O_CLOEXEC,
                              S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
                m_fileSize = 0;
                g_crashLogFd.store(m_fd);
            }
            if (m_fd >= 0) {
                writeAll(m_fd, bytes.constData(), static_cast<size_t>(bytes.size()));
                m_fileSize += bytes.size();
            }
        }
    }

    std::function<void(const QString&)> sink;
    {
        std::lock_guard<std::mutex> lock(m_sinkMutex);
        sink = m_uiSink;
    }
    if (sink && level != Level::Debug) {
        const char* cat = category ? category : "";
        if (std::strcmp(cat, "stdout") != 0)
            sink(formatted + '\n');
    }
}

void DiagnosticLogger::logEvent(Level level,
                               const char* category,
                               const QString& event,
                               const QString& result,
                               std::initializer_list<LogField> fields,
                               const char* file,
                               int line,
                               const char* function)
{
    QString payload = QString("event=%1 result=%2")
                          .arg(sanitizeFieldValue(event.isEmpty() ? QStringLiteral("-") : event))
                          .arg(sanitizeFieldValue(result.isEmpty() ? QStringLiteral("unknown") : result));
    for (const auto& field : fields) {
        if (field.first.isEmpty())
            continue;
        payload += QString(" %1=%2")
            .arg(sanitizeFieldValue(field.first))
            .arg(sanitizeFieldValue(field.second));
    }
    log(level, category, payload, file, line, function);
}

void DiagnosticLogger::logEvent(Level level,
                               const char* category,
                               const QString& event,
                               const QString& result,
                               const std::vector<LogField>& fields,
                               const char* file,
                               int line,
                               const char* function)
{
    QString payload = QString("event=%1 result=%2")
                          .arg(sanitizeFieldValue(event.isEmpty() ? QStringLiteral("-") : event))
                          .arg(sanitizeFieldValue(result.isEmpty() ? QStringLiteral("unknown") : result));
    for (const auto& field : fields) {
        if (field.first.isEmpty())
            continue;
        payload += QString(" %1=%2")
                       .arg(sanitizeFieldValue(field.first))
                       .arg(sanitizeFieldValue(field.second));
    }
    log(level, category, payload, file, line, function);
}

void DiagnosticLogger::flush()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_fd >= 0)
        ::fsync(m_fd);
}

QString DiagnosticLogger::logFilePath() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_logFilePath;
}

void DiagnosticLogger::setUiSink(std::function<void(const QString&)> sink)
{
    std::lock_guard<std::mutex> lock(m_sinkMutex);
    m_uiSink = std::move(sink);
}
