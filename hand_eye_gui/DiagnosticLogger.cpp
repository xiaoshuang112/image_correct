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
#include <cstdint>
#include <iostream>
#include <new>
#include <streambuf>
#include <thread>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#include <process.h>
#include <stdio.h>   // stderr for _fileno
#ifndef NOMINMAX
#define NOMINMAX     // 本文件使用 std::max/std::min，不能被子宏覆盖
#endif
#include <windows.h>
#else
#include <execinfo.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace {

std::atomic<int> g_crashLogFd{-1};
std::atomic_flag g_handlingFatalSignal = ATOMIC_FLAG_INIT;

// 日志写入与崩溃路径只依赖这几个最小平台抽象，POSIX 与 Windows 各一份实现。
#ifdef _WIN32
using ssize_t = long long;

int stderrFd() { return _fileno(stderr); }

int openLogFd(const QString& path)
{
    // CreateFileW 直接接收 UTF-16，中文路径不会经 ANSI 代码页转码而丢失。
    // 共享必须含 FILE_SHARE_DELETE：轮转时会重命名仍被其他进程打开的日志，
    // CRT _open 的默认共享模式没有该标志，重命名后重新打开会失败。
    const HANDLE handle = CreateFileW(path.toStdWString().c_str(), GENERIC_WRITE,
                                      FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                                      nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (handle == INVALID_HANDLE_VALUE)
        return -1;
    const int fd = _open_osfhandle(reinterpret_cast<intptr_t>(handle), _O_APPEND | _O_BINARY);
    if (fd < 0)
        CloseHandle(handle);
    return fd;
}

void closeLog(int fd) { _close(fd); }
int flushLog(int fd) { return _commit(fd); }
qint64 seekToEnd(int fd) { return static_cast<qint64>(_lseeki64(fd, 0, SEEK_END)); }
ssize_t writeFd(int fd, const char* data, size_t length)
{
    return static_cast<ssize_t>(_write(fd, data, static_cast<unsigned int>(length)));
}
long long processId() { return static_cast<long long>(GetCurrentProcessId()); }
long long threadIdValue() { return static_cast<long long>(GetCurrentThreadId()); }
#else
int stderrFd() { return STDERR_FILENO; }
int openLogFd(const QString& path)
{
    return ::open(QFile::encodeName(path).constData(),
                  O_CREAT | O_WRONLY | O_APPEND | O_CLOEXEC,
                  S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
}
void closeLog(int fd) { ::close(fd); }
int flushLog(int fd) { return ::fsync(fd); }
qint64 seekToEnd(int fd)
{
    const off_t offset = ::lseek(fd, 0, SEEK_END);
    return offset >= 0 ? static_cast<qint64>(offset) : -1;
}
ssize_t writeFd(int fd, const char* data, size_t length)
{
    return ::write(fd, data, length);
}
long long processId() { return static_cast<long long>(::getpid()); }
long long threadIdValue()
{
#ifdef SYS_gettid
    return static_cast<long long>(::syscall(SYS_gettid));
#else
    return 0;
#endif
}
#endif

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
        const ssize_t written = writeFd(fd, data, length);
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
    writeAll(stderrFd(), consoleLine.constData(),
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

// 崩溃路径不分配内存、不加锁：固定缓冲区 + 平台 fd 抽象。
void writeCrashReport(int signalNumber, long long code, const void* address)
{
    char line[512];
    size_t length = 0;
    length = appendLiteral(line, length, sizeof(line),
                           "\n[FATAL] process_crash signal=");
    length = appendLiteral(line, length, sizeof(line), signalName(signalNumber));
    length = appendLiteral(line, length, sizeof(line), " pid=");
    length = appendUnsigned(line, length, sizeof(line),
                            static_cast<unsigned long long>(processId()), 10);
    length = appendLiteral(line, length, sizeof(line), " tid=");
    length = appendUnsigned(line, length, sizeof(line),
                            static_cast<unsigned long long>(threadIdValue()), 10);
    if (code != 0) {
        length = appendLiteral(line, length, sizeof(line), " code=");
        length = appendSigned(line, length, sizeof(line), code);
    }
    if (code > 0 && address) {
        length = appendLiteral(line, length, sizeof(line), " address=0x");
        length = appendUnsigned(line, length, sizeof(line),
                                reinterpret_cast<uintptr_t>(address), 16);
    }
    length = appendLiteral(line, length, sizeof(line), "\nbacktrace:\n");

    const int logFd = g_crashLogFd.load();
    if (logFd >= 0)
        writeAll(logFd, line, length);
    writeAll(stderrFd(), line, length);
}

#ifdef _WIN32

void writeBacktrace(int logFd)
{
    void* frames[64];
    // DbgHelp 会在崩溃路径里分配内存并加载符号，这里只记录返回地址。
    const int frameCount = static_cast<int>(
        CaptureStackBackTrace(1, 64, frames, nullptr));
    char line[48];
    for (int index = 0; index < frameCount; ++index) {
        size_t length = appendLiteral(line, 0, sizeof(line), "  0x");
        length = appendUnsigned(line, length, sizeof(line),
                                static_cast<unsigned long long>(
                                    reinterpret_cast<uintptr_t>(frames[index])), 16);
        length = appendLiteral(line, length, sizeof(line), "\n");
        if (logFd >= 0)
            writeAll(logFd, line, length);
        writeAll(stderrFd(), line, length);
    }
}

int signalNumberForExceptionCode(DWORD exceptionCode)
{
    switch (exceptionCode) {
    case EXCEPTION_ACCESS_VIOLATION:
    case EXCEPTION_STACK_OVERFLOW:     return SIGSEGV;
    case EXCEPTION_INT_DIVIDE_BY_ZERO: return SIGFPE;
    case EXCEPTION_ILLEGAL_INSTRUCTION: return SIGILL;
    default:                            return SIGABRT;
    }
}

void writeCrashAndExit(int signalNumber, long long code, const void* address)
{
    if (g_handlingFatalSignal.test_and_set())
        std::_Exit(128 + signalNumber);
    // 栈溢出时再压栈取 backtrace 容易二次崩溃，只留一行摘要。
    if (signalNumber == SIGSEGV && code == static_cast<long long>(EXCEPTION_STACK_OVERFLOW)) {
        char line[128];
        size_t length = appendLiteral(line, 0, sizeof(line),
                                      "\n[FATAL] process_crash signal=SIGSEGV "
                                      "reason=stack_overflow\n");
        const int logFd = g_crashLogFd.load();
        if (logFd >= 0)
            writeAll(logFd, line, length);
        writeAll(stderrFd(), line, length);
    } else {
        writeCrashReport(signalNumber, code, address);
        writeBacktrace(g_crashLogFd.load());
    }
    std::_Exit(128 + signalNumber);
}

LONG WINAPI unhandledExceptionHandler(EXCEPTION_POINTERS* exceptionPointers)
{
    // 只用顶层过滤器：避免一阶 VEH 把 SDK/驱动里可恢复的 SEH 提前杀掉。
    const DWORD code = exceptionPointers->ExceptionRecord->ExceptionCode;
    const void* address = (code == EXCEPTION_ACCESS_VIOLATION)
        ? reinterpret_cast<const void*>(
              exceptionPointers->ExceptionRecord->ExceptionInformation[1])
        : nullptr;
    writeCrashAndExit(signalNumberForExceptionCode(code),
                      static_cast<long long>(code), address);
    return EXCEPTION_CONTINUE_SEARCH;
}

void crashSignalHandler(int signalNumber)
{
    writeCrashAndExit(signalNumber, 0, nullptr);
}

#else

void writeBacktrace(int logFd)
{
    void* frames[64];
    const int frameCount = ::backtrace(frames, 64);
    if (logFd >= 0)
        ::backtrace_symbols_fd(frames, frameCount, logFd);
    ::backtrace_symbols_fd(frames, frameCount, stderrFd());
}

void fatalSignalHandler(int signalNumber, siginfo_t* info, void*)
{
    if (g_handlingFatalSignal.test_and_set())
        std::_Exit(128 + signalNumber);

    const long long code = info ? static_cast<long long>(info->si_code) : 0;
    const void* address = info ? info->si_addr : nullptr;
    writeCrashReport(signalNumber, code, address);
    writeBacktrace(g_crashLogFd.load());

    ::kill(::getpid(), signalNumber);
    std::_Exit(128 + signalNumber);
}

#endif

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
    writeAll(stderrFd(), message, sizeof(message) - 1);
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
                  + "_" + QString::number(processId()))
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

    m_fd = openLogFd(m_logFilePath);
    if (m_fd < 0) {
        m_logDirectory.clear();
        m_logFilePath.clear();
        return false;
    }

    m_fileSize = seekToEnd(m_fd);
    if (m_fileSize < 0)
        m_fileSize = 0;
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
#ifdef _WIN32
    // Windows 没有 sigaltstack/sigaction：只挂未处理异常过滤器，不使用一阶 VEH，
    // 以免打断相机或 SDK 库内部的 SEH 恢复路径。CRT 信号仍保留以覆盖 abort()。
    SetUnhandledExceptionFilter(unhandledExceptionHandler);
    std::signal(SIGABRT, crashSignalHandler);
    std::signal(SIGFPE, crashSignalHandler);
    std::signal(SIGILL, crashSignalHandler);
#else
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
    const qint64 threadId = static_cast<qint64>(threadIdValue());
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
                closeLog(m_fd);
                m_fd = -1;
                rotateLogLocked();
                m_fd = openLogFd(m_logFilePath);
                m_fileSize = 0;
                g_crashLogFd.store(m_fd);
            }
            if (m_fd >= 0) {
                writeAll(m_fd, bytes.constData(), static_cast<size_t>(bytes.size()));
                m_fileSize += bytes.size();
            }
        }
    }
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
        flushLog(m_fd);
}

QString DiagnosticLogger::logFilePath() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_logFilePath;
}
