#include "calibration_session.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <system_error>

#if defined(__linux__)
#include <fcntl.h>
#include <unistd.h>
#endif

#if __has_include("AppBuildInfo.hpp")
#include "AppBuildInfo.hpp"
#endif

#ifndef APP_BUILD_TYPE
#define APP_BUILD_TYPE "unknown"
#endif
#ifndef APP_VERSION
#define APP_VERSION "unknown"
#endif
#ifndef APP_BUILD_TIMESTAMP
#define APP_BUILD_TIMESTAMP "unknown"
#endif
#ifndef APP_GIT_COMMIT
#define APP_GIT_COMMIT "unknown"
#endif
#ifndef APP_GIT_WORKTREE_STATE
#define APP_GIT_WORKTREE_STATE "unknown"
#endif

namespace {

namespace fs = std::filesystem;

std::string utcNow(const char* format = "%Y-%m-%dT%H:%M:%SZ") {
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
#if defined(_WIN32)
    gmtime_s(&utc, &now);
#else
    gmtime_r(&now, &utc);
#endif
    std::ostringstream out;
    out << std::put_time(&utc, format);
    return out.str();
}

std::string executablePath() {
#if defined(__linux__)
    std::array<char, 4096> buffer{};
    const ssize_t count = ::readlink("/proc/self/exe", buffer.data(), buffer.size() - 1);
    if (count > 0)
        return std::string(buffer.data(), static_cast<size_t>(count));
#endif
    return "unknown";
}

bool syncFile(const fs::path& path, std::string& error) {
#if defined(__linux__)
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        error = "cannot open file for fsync: " + path.string() + ": " +
                std::strerror(errno);
        return false;
    }
    int result;
    do {
        result = ::fsync(fd);
    } while (result != 0 && errno == EINTR);
    const int savedErrno = errno;
    ::close(fd);
    if (result != 0) {
        error = "cannot fsync file: " + path.string() + ": " +
                std::strerror(savedErrno);
        return false;
    }
#else
    (void)path;
    (void)error;
#endif
    return true;
}

bool syncDirectory(const fs::path& path, std::string& error) {
#if defined(__linux__)
    const int fd = ::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (fd < 0) {
        error = "cannot open directory for fsync: " + path.string() + ": " +
                std::strerror(errno);
        return false;
    }
    int result;
    do {
        result = ::fsync(fd);
    } while (result != 0 && errno == EINTR);
    const int savedErrno = errno;
    ::close(fd);
    if (result != 0) {
        error = "cannot fsync directory: " + path.string() + ": " +
                std::strerror(savedErrno);
        return false;
    }
#else
    (void)path;
    (void)error;
#endif
    return true;
}

bool atomicRename(const fs::path& temporary,
                  const fs::path& destination,
                  std::string& error) {
    if (!syncFile(temporary, error)) {
        std::error_code ignored;
        fs::remove(temporary, ignored);
        return false;
    }
    std::error_code ec;
    fs::rename(temporary, destination, ec);
    if (ec) {
        fs::remove(temporary);
        error = "atomic rename failed: " + ec.message();
        return false;
    }
    return syncDirectory(destination.parent_path(), error);
}

template <typename Writer>
bool atomicYamlWrite(const fs::path& destination, Writer writer, std::string& error) {
    const fs::path temporary = destination.string() + ".tmp.yaml";
    try {
        cv::FileStorage storage(temporary.string(), cv::FileStorage::WRITE);
        if (!storage.isOpened()) {
            error = "cannot open temporary YAML: " + temporary.string();
            return false;
        }
        writer(storage);
        storage.release();
    } catch (const cv::Exception& exception) {
        error = exception.what();
        std::error_code ignored;
        fs::remove(temporary, ignored);
        return false;
    }
    return atomicRename(temporary, destination, error);
}

bool atomicTextWrite(const fs::path& destination,
                     const std::string& contents,
                     std::string& error) {
    const fs::path temporary = destination.string() + ".tmp";
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output) {
            error = "cannot open temporary file: " + temporary.string();
            return false;
        }
        output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
        output.flush();
        if (!output) {
            error = "cannot flush temporary file: " + temporary.string();
            return false;
        }
    }
    return atomicRename(temporary, destination, error);
}

bool atomicFileCopy(const fs::path& source,
                    const fs::path& destination,
                    std::string& error) {
    std::ifstream input(source, std::ios::binary);
    if (!input) {
        error = "cannot open publication candidate: " + source.string();
        return false;
    }
    std::ostringstream contents;
    contents << input.rdbuf();
    if (!input.eof() && input.fail()) {
        error = "cannot read publication candidate: " + source.string();
        return false;
    }
    return atomicTextWrite(destination, contents.str(), error);
}

bool durableRemove(const fs::path& path, std::string& error) {
    std::error_code ec;
    const bool removed = fs::remove(path, ec);
    if (ec) {
        error = "cannot remove durable marker: " + path.string() + ": " + ec.message();
        return false;
    }
    return !removed || syncDirectory(path.parent_path(), error);
}

bool durableAppend(const fs::path& path,
                   const std::string& contents,
                   std::string& error) {
#if defined(__linux__)
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
    if (fd < 0) {
        error = "cannot open append log: " + path.string() + ": " + std::strerror(errno);
        return false;
    }
    size_t offset = 0;
    while (offset < contents.size()) {
        const ssize_t count = ::write(fd, contents.data() + offset, contents.size() - offset);
        if (count < 0 && errno == EINTR)
            continue;
        if (count <= 0) {
            const int savedErrno = errno;
            ::close(fd);
            error = "cannot append log: " + path.string() + ": " +
                    std::strerror(savedErrno);
            return false;
        }
        offset += static_cast<size_t>(count);
    }
    int result;
    do {
        result = ::fsync(fd);
    } while (result != 0 && errno == EINTR);
    const int savedErrno = errno;
    ::close(fd);
    if (result != 0) {
        error = "cannot fsync append log: " + path.string() + ": " +
                std::strerror(savedErrno);
        return false;
    }
    return syncDirectory(path.parent_path(), error);
#else
    std::ofstream output(path, std::ios::binary | std::ios::app);
    output << contents;
    output.flush();
    if (!output) {
        error = "cannot append log: " + path.string();
        return false;
    }
    return true;
#endif
}

void maybeInjectTestCrash(const char* point) {
    const char* requested = std::getenv("CAITU_CALIBRATION_TEST_CRASH_POINT");
    if (!requested || std::string(requested) != point)
        return;
#if defined(__linux__)
    ::kill(::getpid(), SIGKILL);
#else
    std::_Exit(86);
#endif
}

std::string relativeOrFilename(const fs::path& path, const fs::path& base) {
    std::error_code ec;
    fs::path relative = fs::relative(path, base, ec);
    return ec ? path.filename().generic_string() : relative.generic_string();
}

const std::string& orUnknown(const std::string& value) {
    static const std::string unknown = "unknown";
    return value.empty() ? unknown : value;
}

void writeStringSequence(cv::FileStorage& storage,
                         const char* key,
                         const std::vector<std::string>& values) {
    storage << key << "[";
    for (const std::string& value : values)
        storage << value;
    storage << "]";
}

void writeIntSequence(cv::FileStorage& storage,
                      const char* key,
                      const std::vector<int>& values) {
    storage << key << "[";
    for (const int value : values)
        storage << value;
    storage << "]";
}

bool excludedFromManifest(const fs::path& path) {
    const std::string name = path.filename().string();
    return name == "manifest.sha256" || name == ".running" ||
           name.find(".tmp.") != std::string::npos ||
           (name.size() >= 4 && name.substr(name.size() - 4) == ".tmp");
}

bool writeManifestForDirectory(const fs::path& directory, std::string& error) {
    std::vector<fs::path> files;
    std::vector<fs::path> directories{directory};
    std::error_code ec;
    for (const auto& entry : fs::recursive_directory_iterator(directory, ec)) {
        if (ec) {
            error = "cannot scan session for manifest: " + ec.message();
            return false;
        }
        if (entry.is_directory()) {
            directories.push_back(entry.path());
        } else if (entry.is_regular_file() && !excludedFromManifest(entry.path())) {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());
    for (const fs::path& file : files) {
        if (!syncFile(file, error))
            return false;
    }
    std::sort(directories.begin(), directories.end(), [](const fs::path& left,
                                                          const fs::path& right) {
        return left.native().size() > right.native().size();
    });
    for (const fs::path& child : directories) {
        if (!syncDirectory(child, error))
            return false;
    }
    std::ostringstream manifest;
    for (const fs::path& file : files)
        manifest << CalibrationSession::sha256File(file.string()) << "  "
                 << relativeOrFilename(file, directory) << '\n';
    return atomicTextWrite(directory / "manifest.sha256", manifest.str(), error);
}

struct PublicationIntent {
    std::string targetFile;
    std::string candidateFile;
    std::string candidateSha256;
    bool previousExists = false;
    std::string previousSha256;
};

bool readPublicationIntent(const fs::path& path,
                           PublicationIntent& intent,
                           std::string& error) {
    try {
        cv::FileStorage storage(path.string(), cv::FileStorage::READ);
        if (!storage.isOpened()) {
            error = "cannot open publication intent: " + path.string();
            return false;
        }
        storage["target_file"] >> intent.targetFile;
        storage["candidate_file"] >> intent.candidateFile;
        storage["candidate_sha256"] >> intent.candidateSha256;
        int previousExists = 0;
        storage["previous_exists"] >> previousExists;
        intent.previousExists = previousExists != 0;
        storage["previous_sha256"] >> intent.previousSha256;
    } catch (const cv::Exception& exception) {
        error = exception.what();
        return false;
    }
    if (intent.targetFile.empty() || intent.candidateSha256.size() != 64) {
        error = "publication intent is incomplete: " + path.string();
        return false;
    }
    return true;
}

std::string readString(const cv::FileNode& node, const char* key) {
    std::string value;
    if (!node.empty())
        node[key] >> value;
    return value;
}

bool writeRecoveredSessionStatus(const fs::path& sessionFile,
                                 const std::string& sessionId,
                                 const std::string& lifecycleStatus,
                                 const std::string& resultStatus,
                                 const std::string& stage,
                                 const std::string& publicationState,
                                 std::string& error) {
    std::string timestamp;
    std::string mode;
    std::string sourceDirectory;
    std::string effectiveConfigFile = "effective_config.yaml";
    std::string effectiveConfigHash;
    std::string sourceSessionId;
    std::string sourceManifestHash;
    std::array<std::string, 7> software{};
    try {
        cv::FileStorage input(sessionFile.string(), cv::FileStorage::READ);
        if (input.isOpened()) {
            timestamp = readString(input.root(), "timestamp_utc");
            mode = readString(input.root(), "mode");
            sourceDirectory = readString(input.root(), "source_data_directory");
            const std::string configFile = readString(input.root(), "effective_config_file");
            if (!configFile.empty())
                effectiveConfigFile = configFile;
            effectiveConfigHash = readString(input.root(), "effective_config_sha256");
            const cv::FileNode lineage = input["lineage"];
            sourceSessionId = readString(lineage, "source_session_id");
            sourceManifestHash = readString(lineage, "source_manifest_sha256");
            const cv::FileNode node = input["software"];
            const char* keys[] = {"build_type", "build_timestamp", "git_commit",
                                  "git_worktree_state", "opencv_version", "executable",
                                  "executable_sha256"};
            for (size_t index = 0; index < software.size(); ++index)
                software[index] = readString(node, keys[index]);
        }
    } catch (const cv::Exception&) {
        // A damaged partial session is still recoverable into a minimal audit record.
    }
    return atomicYamlWrite(sessionFile, [&](cv::FileStorage& out) {
        out << "schema_version" << 1;
        out << "session_id" << sessionId;
        out << "timestamp_utc" << timestamp;
        out << "lifecycle_status" << lifecycleStatus;
        out << "result_status" << resultStatus;
        out << "current_stage" << stage;
        out << "publication_state" << publicationState;
        out << "recovered_at_utc" << utcNow();
        out << "mode" << mode;
        out << "source_data_directory" << sourceDirectory;
        out << "software" << "{";
        const char* keys[] = {"build_type", "build_timestamp", "git_commit",
                              "git_worktree_state", "opencv_version", "executable",
                              "executable_sha256"};
        for (size_t index = 0; index < software.size(); ++index)
            out << keys[index] << software[index];
        out << "}";
        out << "effective_config_file" << effectiveConfigFile;
        out << "effective_config_sha256" << effectiveConfigHash;
        out << "lineage" << "{";
        out << "source_session_id" << sourceSessionId;
        out << "source_manifest_sha256" << sourceManifestHash;
        out << "}";
    }, error);
}

bool recoverTerminalFailure(const fs::path& directory,
                            const std::string& resultStatus,
                            const std::string& reason,
                            const std::string& publicationState,
                            std::string& error) {
    const fs::path report = directory / "calibration_report.yaml";
    const fs::path sessionFile = directory / "session.yaml";
    if (fs::exists(report) &&
        !fs::exists(directory / "calibration_report.partial.yaml") &&
        !atomicFileCopy(report, directory / "calibration_report.partial.yaml", error))
        return false;
    if (fs::exists(sessionFile) &&
        !fs::exists(directory / "session.partial.yaml") &&
        !atomicFileCopy(sessionFile, directory / "session.partial.yaml", error))
        return false;
    if (!atomicYamlWrite(report, [&](cv::FileStorage& out) {
        out << "schema_version" << 1;
        out << "session_id" << directory.filename().string();
        out << "result" << "{";
        out << "status" << resultStatus;
        out << "failure_stage" << "process_lifecycle";
        out << "failure_reason" << reason;
        out << "publication_state" << publicationState;
        out << "recovered_at_utc" << utcNow();
        out << "partial_report" << "calibration_report.partial.yaml";
        out << "}";
    }, error))
        return false;
    if (!writeRecoveredSessionStatus(sessionFile, directory.filename().string(),
                                     "sealed", resultStatus, "process_lifecycle",
                                     publicationState, error))
        return false;
    if (!durableAppend(directory / "calibration.log",
                       utcNow() + " event=session_recovery result=" + resultStatus +
                       " detail=\"" + reason + "\"\n", error))
        return false;
    if (!writeManifestForDirectory(directory, error))
        return false;
    return durableRemove(directory / ".running", error);
}

bool recoverPublishedSession(const fs::path& directory,
                             const PublicationIntent& intent,
                             std::string& error) {
    const fs::path commit = directory / "publication.commit.yaml";
    if (!fs::exists(commit) &&
        !atomicYamlWrite(commit, [&](cv::FileStorage& out) {
            out << "schema_version" << 1;
            out << "session_id" << directory.filename().string();
            out << "target_file" << intent.targetFile;
            out << "candidate_sha256" << intent.candidateSha256;
            out << "state" << "committed";
            out << "committed_at_utc" << utcNow();
            out << "recovered_after_interruption" << 1;
        }, error))
        return false;
    if (!writeRecoveredSessionStatus(directory / "session.yaml",
                                     directory.filename().string(), "sealed", "pass",
                                     "complete", "published", error))
        return false;
    if (!durableAppend(directory / "calibration.log",
                       utcNow() + " event=session_recovery result=pass"
                       " detail=\"production_candidate_already_published\"\n", error))
        return false;
    if (!writeManifestForDirectory(directory, error))
        return false;
    return durableRemove(directory / ".running", error);
}

bool recoverAlreadySealedSession(const fs::path& directory,
                                 bool& wasSealed,
                                 std::string& error) {
    wasSealed = false;
    std::string lifecycle;
    std::string result;
    try {
        cv::FileStorage input((directory / "session.yaml").string(),
                              cv::FileStorage::READ);
        if (!input.isOpened())
            return true;
        input["lifecycle_status"] >> lifecycle;
        input["result_status"] >> result;
    } catch (const cv::Exception&) {
        return true;
    }
    if (lifecycle != "sealed" ||
        (result != "pass" && result != "fail" && result != "interrupted"))
        return true;
    wasSealed = true;
    if (!durableAppend(directory / "calibration.log",
                       utcNow() + " event=session_recovery result=" + result +
                       " detail=\"terminal_files_completed_before_marker_cleanup\"\n",
                       error))
        return false;
    if (!writeManifestForDirectory(directory, error))
        return false;
    return durableRemove(directory / ".running", error);
}

bool recoverInterruptedSessions(const fs::path& sessionsDirectory,
                                std::vector<std::string>& recoveredSessions,
                                std::string& error) {
    std::error_code ec;
    if (!fs::exists(sessionsDirectory, ec))
        return !ec;
    for (const auto& entry : fs::directory_iterator(sessionsDirectory, ec)) {
        if (ec) {
            error = "cannot scan pending sessions: " + ec.message();
            return false;
        }
        if (!entry.is_directory() || !fs::exists(entry.path() / ".running"))
            continue;

        const fs::path intentPath = entry.path() / "publication.intent.yaml";
        bool recovered = false;
        if (fs::exists(intentPath)) {
            PublicationIntent intent;
            if (!readPublicationIntent(intentPath, intent, error))
                return false;
            const fs::path target(intent.targetFile);
            const bool targetExists = fs::exists(target, ec);
            if (ec) {
                error = "cannot inspect production target: " + ec.message();
                return false;
            }
            const std::string targetHash = targetExists
                ? CalibrationSession::sha256File(target.string()) : std::string();
            const bool candidatePublished = targetExists &&
                targetHash == intent.candidateSha256;
            const bool previousStillPresent = targetExists == intent.previousExists &&
                (!targetExists || targetHash == intent.previousSha256);
            if (candidatePublished) {
                recovered = recoverPublishedSession(entry.path(), intent, error);
            } else if (previousStillPresent) {
                recovered = recoverTerminalFailure(
                    entry.path(), "interrupted",
                    "publication_prepared_but_not_committed", "not_published", error);
            } else {
                recovered = recoverTerminalFailure(
                    entry.path(), "fail", "production_state_ambiguous",
                    "ambiguous_no_automatic_overwrite", error);
            }
        } else {
            bool wasSealed = false;
            if (!recoverAlreadySealedSession(entry.path(), wasSealed, error))
                return false;
            recovered = wasSealed || recoverTerminalFailure(
                entry.path(), "interrupted",
                "previous_process_did_not_finalize_session", "not_started", error);
        }
        if (!recovered)
            return false;
        recoveredSessions.push_back(entry.path().string());
    }
    return true;
}

// Small self-contained SHA-256 implementation. It avoids adding a crypto
// runtime dependency to the calibration executable deployed on Jetson.
class Sha256 final {
public:
    Sha256() { reset(); }

    void update(const unsigned char* data, size_t length) {
        for (size_t index = 0; index < length; ++index) {
            buffer_[bufferLength_++] = data[index];
            if (bufferLength_ == 64) {
                transform();
                bitLength_ += 512;
                bufferLength_ = 0;
            }
        }
    }

    std::string finish() {
        size_t index = bufferLength_;
        buffer_[index++] = 0x80;
        if (index > 56) {
            while (index < 64)
                buffer_[index++] = 0;
            transform();
            index = 0;
        }
        while (index < 56)
            buffer_[index++] = 0;
        bitLength_ += static_cast<std::uint64_t>(bufferLength_) * 8;
        for (int byte = 7; byte >= 0; --byte)
            buffer_[56 + (7 - byte)] = static_cast<unsigned char>(bitLength_ >> (byte * 8));
        transform();

        std::ostringstream out;
        out << std::hex << std::setfill('0');
        for (const std::uint32_t value : state_)
            out << std::setw(8) << value;
        return out.str();
    }

private:
    static std::uint32_t rotateRight(std::uint32_t value, std::uint32_t count) {
        return (value >> count) | (value << (32 - count));
    }

    void reset() {
        state_ = {0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
                  0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
        buffer_.fill(0);
        bufferLength_ = 0;
        bitLength_ = 0;
    }

    void transform() {
        static constexpr std::array<std::uint32_t, 64> constants{{
            0x428a2f98U,0x71374491U,0xb5c0fbcfU,0xe9b5dba5U,0x3956c25bU,0x59f111f1U,0x923f82a4U,0xab1c5ed5U,
            0xd807aa98U,0x12835b01U,0x243185beU,0x550c7dc3U,0x72be5d74U,0x80deb1feU,0x9bdc06a7U,0xc19bf174U,
            0xe49b69c1U,0xefbe4786U,0x0fc19dc6U,0x240ca1ccU,0x2de92c6fU,0x4a7484aaU,0x5cb0a9dcU,0x76f988daU,
            0x983e5152U,0xa831c66dU,0xb00327c8U,0xbf597fc7U,0xc6e00bf3U,0xd5a79147U,0x06ca6351U,0x14292967U,
            0x27b70a85U,0x2e1b2138U,0x4d2c6dfcU,0x53380d13U,0x650a7354U,0x766a0abbU,0x81c2c92eU,0x92722c85U,
            0xa2bfe8a1U,0xa81a664bU,0xc24b8b70U,0xc76c51a3U,0xd192e819U,0xd6990624U,0xf40e3585U,0x106aa070U,
            0x19a4c116U,0x1e376c08U,0x2748774cU,0x34b0bcb5U,0x391c0cb3U,0x4ed8aa4aU,0x5b9cca4fU,0x682e6ff3U,
            0x748f82eeU,0x78a5636fU,0x84c87814U,0x8cc70208U,0x90befffaU,0xa4506cebU,0xbef9a3f7U,0xc67178f2U
        }};
        std::array<std::uint32_t, 64> words{};
        for (int index = 0; index < 16; ++index) {
            words[index] = (static_cast<std::uint32_t>(buffer_[index * 4]) << 24) |
                           (static_cast<std::uint32_t>(buffer_[index * 4 + 1]) << 16) |
                           (static_cast<std::uint32_t>(buffer_[index * 4 + 2]) << 8) |
                           static_cast<std::uint32_t>(buffer_[index * 4 + 3]);
        }
        for (int index = 16; index < 64; ++index) {
            const std::uint32_t s0 = rotateRight(words[index - 15], 7) ^
                                     rotateRight(words[index - 15], 18) ^
                                     (words[index - 15] >> 3);
            const std::uint32_t s1 = rotateRight(words[index - 2], 17) ^
                                     rotateRight(words[index - 2], 19) ^
                                     (words[index - 2] >> 10);
            words[index] = words[index - 16] + s0 + words[index - 7] + s1;
        }
        std::uint32_t a=state_[0], b=state_[1], c=state_[2], d=state_[3];
        std::uint32_t e=state_[4], f=state_[5], g=state_[6], h=state_[7];
        for (int index = 0; index < 64; ++index) {
            const std::uint32_t s1 = rotateRight(e,6)^rotateRight(e,11)^rotateRight(e,25);
            const std::uint32_t choice = (e&f)^((~e)&g);
            const std::uint32_t temp1 = h+s1+choice+constants[index]+words[index];
            const std::uint32_t s0 = rotateRight(a,2)^rotateRight(a,13)^rotateRight(a,22);
            const std::uint32_t majority = (a&b)^(a&c)^(b&c);
            const std::uint32_t temp2 = s0+majority;
            h=g; g=f; f=e; e=d+temp1; d=c; c=b; b=a; a=temp1+temp2;
        }
        state_[0]+=a; state_[1]+=b; state_[2]+=c; state_[3]+=d;
        state_[4]+=e; state_[5]+=f; state_[6]+=g; state_[7]+=h;
    }

    std::array<std::uint32_t, 8> state_{};
    std::array<unsigned char, 64> buffer_{};
    size_t bufferLength_ = 0;
    std::uint64_t bitLength_ = 0;
};

std::string sha256Text(const std::string& text) {
    Sha256 hash;
    hash.update(reinterpret_cast<const unsigned char*>(text.data()), text.size());
    return hash.finish();
}

std::string intrinsicFingerprint(const cv::Mat& cameraMatrix,
                                 const cv::Mat& distortion) {
    if (cameraMatrix.empty() || distortion.empty())
        return "";
    std::ostringstream canonical;
    canonical << std::setprecision(17) << cameraMatrix.rows << 'x' << cameraMatrix.cols << ':';
    cv::Mat camera64;
    cv::Mat distortion64;
    cameraMatrix.convertTo(camera64, CV_64F);
    distortion.reshape(1, 1).convertTo(distortion64, CV_64F);
    for (size_t index = 0; index < camera64.total(); ++index)
        canonical << camera64.ptr<double>()[index] << ',';
    canonical << '|';
    for (size_t index = 0; index < distortion64.total(); ++index)
        canonical << distortion64.ptr<double>()[index] << ',';
    return sha256Text(canonical.str());
}

std::string intrinsicBindingFingerprint(const CalibrationSessionOptions& options) {
    const std::string intrinsics = intrinsicFingerprint(options.camera_matrix,
                                                         options.dist_coeffs);
    if (intrinsics.empty())
        return "";
    std::ostringstream canonical;
    canonical << "intrinsics=" << intrinsics
              << "|serial=" << options.camera_serial
              << "|model=" << options.camera_model
              << "|firmware=" << options.camera_firmware
              << "|endpoint=" << options.camera_endpoint
              << "|stream=" << options.camera_stream
              << "|resolution=" << options.image_width << 'x' << options.image_height
              << "|roi=0,0," << options.image_width << ',' << options.image_height
              << "|binning=unknown|decimation=unknown|scaling=unknown|rotation=unknown"
              << "|distortion_model=opencv_plumb_bob";
    return sha256Text(canonical.str());
}

bool copyDatasetFile(const fs::path& source,
                     const fs::path& destination,
                     std::string& error) {
    std::error_code ec;
    fs::copy_file(source, destination, fs::copy_options::overwrite_existing, ec);
    if (!ec)
        return true;
    error = "cannot import " + source.string() + ": " + ec.message();
    return false;
}

}  // namespace

CalibrationSession::CalibrationSession(CalibrationSessionOptions options)
    : options_(std::move(options)) {}

std::unique_ptr<CalibrationSession> CalibrationSession::create(
    const CalibrationSessionOptions& options,
    std::string& error) {
    auto session = std::unique_ptr<CalibrationSession>(new CalibrationSession(options));
    if (!session->initialize(error))
        return nullptr;
    return session;
}

CalibrationSession::~CalibrationSession() {
    if (!finalized_ && !terminalization_attempted_ && !directory_.empty())
        markInterruptedNoThrow();
}

bool CalibrationSession::initialize(std::string& error) {
    if (options_.base_directory.empty()) {
        error = "session base directory is empty";
        return false;
    }
    const fs::path sessionsDirectory = fs::path(options_.base_directory) / "sessions";
    std::error_code ec;
    fs::create_directories(sessionsDirectory, ec);
    if (ec) {
        error = "cannot create sessions directory: " + ec.message();
        return false;
    }
    std::vector<std::string> recoveredSessions;
    if (!recoverInterruptedSessions(sessionsDirectory, recoveredSessions, error))
        return false;

    timestamp_utc_ = utcNow();
    std::random_device random;
    std::ostringstream identifier;
    identifier << utcNow("%Y%m%dT%H%M%SZ") << "-" << std::hex
               << std::setw(8) << std::setfill('0') << random();
    id_ = identifier.str();
    directory_ = (sessionsDirectory / id_).string();
    frames_directory_ = (fs::path(directory_) / "frames").string();
    robot_poses_directory_ = (fs::path(directory_) / "raw_robot_poses").string();
    preview_directory_ = (fs::path(directory_) / "preview").string();
    fs::create_directories(frames_directory_, ec);
    fs::create_directories(robot_poses_directory_, ec);
    fs::create_directories(preview_directory_, ec);
    if (ec) {
        error = "cannot create session directories: " + ec.message();
        return false;
    }
    if (!syncDirectory(sessionsDirectory, error) ||
        !syncDirectory(directory_, error))
        return false;
    if (!atomicTextWrite(fs::path(directory_) / ".running", timestamp_utc_ + "\n", error))
        return false;
    if (!appendDurableEvent("session", "start", options_.mode, error))
        return false;
    if (!writeEffectiveConfig(error) ||
        !writeSessionYaml("running", "pending", error))
        return false;
    cv::Mat empty;
    if (!writeReport("running", "", "", empty, 0.0, 0, "", error))
        return false;
    return true;
}

void CalibrationSession::setStage(const std::string& stage) {
    current_stage_ = stage;
    appendEvent("stage", "start", stage);
    std::string ignored;
    writeSessionYaml("running", "pending", ignored);
}

void CalibrationSession::setIntrinsics(const cv::Mat& camera_matrix,
                                       const cv::Mat& dist_coeffs,
                                       const std::string& source) {
    options_.camera_matrix = camera_matrix.clone();
    options_.dist_coeffs = dist_coeffs.clone();
    options_.intrinsic_source = source;
    std::string ignored;
    writeEffectiveConfig(ignored);
}

void CalibrationSession::setImageSize(int width, int height) {
    options_.image_width = width;
    options_.image_height = height;
    std::string ignored;
    writeEffectiveConfig(ignored);
}

void CalibrationSession::setFrames(const std::vector<CalibrationFrameAudit>& frames) {
    frames_ = frames;
    appendEvent("frame_audit", "updated", "frames=" + std::to_string(frames_.size()));
    std::string ignored;
    cv::Mat empty;
    writeReport("running", "", "", empty, 0.0, 0, "", ignored);
}

bool CalibrationSession::setHandEyePhase2Audit(const HandEyePhase2Audit& audit,
                                               std::string& error) {
    hand_eye_phase2_audit_ = audit;
    appendEvent("solver_diagnostics", "updated",
                "solvers=" + std::to_string(audit.solvers.size()) +
                " pairs=" + std::to_string(audit.pairs.size()));
    cv::Mat empty;
    if (!writeReport("running", "", "", empty, 0.0, 0, "", error)) {
        appendEvent("solver_diagnostics", "persist_failed", error);
        return false;
    }
    maybeInjectTestCrash("phase2_audit_after_persist");
    return true;
}

bool CalibrationSession::setHandEyePhase3Audit(const HandEyePhase3Audit& audit,
                                               std::string& error) {
    hand_eye_phase3_audit_ = audit;
    appendEvent("loo_stability", audit.status,
                "trials=" + std::to_string(audit.trials.size()) +
                " summaries=" + std::to_string(audit.summaries.size()));
    cv::Mat empty;
    if (!writeReport("running", "", "", empty, 0.0, 0, "", error)) {
        appendEvent("loo_stability", "persist_failed", error);
        return false;
    }
    maybeInjectTestCrash("phase3_audit_after_persist");
    return true;
}

void CalibrationSession::appendEvent(const std::string& event,
                                     const std::string& result,
                                     const std::string& detail) {
    std::string ignored;
    appendDurableEvent(event, result, detail, ignored);
}

bool CalibrationSession::appendDurableEvent(const std::string& event,
                                            const std::string& result,
                                            const std::string& detail,
                                            std::string& error) {
    if (directory_.empty()) {
        error = "session directory is empty";
        return false;
    }
    std::ostringstream log;
    log << utcNow() << " event=" << event << " result=" << result;
    if (!detail.empty())
        log << " detail=\"" << detail << "\"";
    log << '\n';
    return durableAppend(fs::path(directory_) / "calibration.log", log.str(), error);
}

bool CalibrationSession::importLegacyDataset(const std::string& source_directory,
                                             std::string& error) {
    if (source_directory.empty())
        return true;
    std::error_code equivalent_error;
    if (fs::equivalent(fs::path(source_directory), fs::path(frames_directory_),
                       equivalent_error) && !equivalent_error)
        return true;
    const fs::path source_root(source_directory);
    const fs::path image_source = fs::is_directory(source_root / "frames")
        ? source_root / "frames" : source_root;
    const fs::path pose_source = fs::is_directory(source_root / "raw_robot_poses")
        ? source_root / "raw_robot_poses" : source_root;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(image_source, ec)) {
        if (ec) {
            error = "cannot scan source dataset: " + ec.message();
            return false;
        }
        if (!entry.is_regular_file())
            continue;
        std::string extension = entry.path().extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
        if (extension == ".jpg" || extension == ".jpeg" || extension == ".png") {
            if (!copyDatasetFile(entry.path(), fs::path(frames_directory_) / entry.path().filename(), error))
                return false;
        }
    }
    ec.clear();
    for (const auto& entry : fs::directory_iterator(pose_source, ec)) {
        if (ec) {
            error = "cannot scan robot pose dataset: " + ec.message();
            return false;
        }
        if (!entry.is_regular_file())
            continue;
        const std::string extension = entry.path().extension().string();
        if (extension != ".csv" && extension != ".json" && extension != ".yaml")
            continue;
        if (!copyDatasetFile(entry.path(),
                             fs::path(robot_poses_directory_) / entry.path().filename(), error))
            return false;
    }
    if (!fs::exists(fs::path(robot_poses_directory_) / "rob_pos.csv")) {
        error = "source dataset has no rob_pos.csv";
        return false;
    }
    appendEvent("dataset_import", "success", source_directory);
    return true;
}

bool CalibrationSession::writeRawRobotPose(int frame_id,
                                           int candidate_index,
                                           const double actual_joint[6],
                                           const double flange_pose[6],
                                           std::uint64_t motion_completed_timestamp_ns,
                                           int settling_requested_ms,
                                           int discarded_camera_frames,
                                           std::uint64_t image_timestamp_ns,
                                           std::uint64_t robot_timestamp_ns,
                                           std::string& error) {
    std::ostringstream name;
    name << "frame_" << std::setw(4) << std::setfill('0') << frame_id << ".json";
    const fs::path destination = fs::path(robot_poses_directory_) / name.str();
    const fs::path temporary = destination.string() + ".tmp.json";
    try {
        cv::FileStorage out(temporary.string(), cv::FileStorage::WRITE |
                                               cv::FileStorage::FORMAT_JSON);
        if (!out.isOpened()) {
            error = "cannot write raw robot pose";
            return false;
        }
        out << "schema_version" << 1;
        out << "frame_id" << frame_id;
        out << "candidate_index" << candidate_index;
        out << "raw_feedback" << "{";
        out << "source" << "cr_get_jointActualPos";
        out << "joint_unit" << "degree";
        out << "joint" << "[";
        for (int axis = 0; axis < 6; ++axis) out << actual_joint[axis];
        out << "]" << "}";
        out << "normalized_pose" << "{";
        out << "source" << "cr_kineForward(actual_joint)";
        out << "definition" << "base_to_flange_axis_angle";
        out << "translation_unit" << "millimeter";
        out << "rotation_unit" << "degree";
        out << "value" << "[";
        for (int axis = 0; axis < 6; ++axis) out << flange_pose[axis];
        out << "]" << "}";
        out << "timing" << "{";
        out << "motion_completed_host_realtime_ns"
            << std::to_string(motion_completed_timestamp_ns);
        out << "settling_requested_ms" << settling_requested_ms;
        out << "discarded_camera_frames" << discarded_camera_frames;
        out << "image_host_realtime_ns" << std::to_string(image_timestamp_ns);
        out << "robot_host_realtime_ns" << std::to_string(robot_timestamp_ns);
        out << "clock_domain" << "host_system_clock";
        out << "sync_mode" << "software_sequential";
        out << "measurement_order" << "image_then_robot_feedback";
        const long long offset = static_cast<long long>(robot_timestamp_ns) -
                                 static_cast<long long>(image_timestamp_ns);
        out << "offset_estimate_ms" << static_cast<double>(offset) / 1.0e6;
        const long long motion_to_image =
            static_cast<long long>(image_timestamp_ns) -
            static_cast<long long>(motion_completed_timestamp_ns);
        out << "motion_to_image_ms" << static_cast<double>(motion_to_image) / 1.0e6;
        out << "}";
        out.release();
    } catch (const cv::Exception& exception) {
        error = exception.what();
        return false;
    }
    return atomicRename(temporary, destination, error);
}

bool CalibrationSession::writeEffectiveConfig(std::string& error) {
    const std::string intrinsicFileHash = options_.intrinsic_source_file.empty()
        ? "" : sha256File(options_.intrinsic_source_file);
    const std::string poseConfigHash = options_.pose_config_source_file.empty()
        ? "" : sha256File(options_.pose_config_source_file);
    return atomicYamlWrite(fs::path(directory_) / "effective_config.yaml",
                           [&](cv::FileStorage& out) {
        out << "schema_version" << 1;
        out << "camera" << "{";
        out << "serial" << orUnknown(options_.camera_serial);
        out << "model" << orUnknown(options_.camera_model);
        out << "firmware" << orUnknown(options_.camera_firmware);
        out << "endpoint" << orUnknown(options_.camera_endpoint);
        out << "stream" << options_.camera_stream;
        out << "backend" << orUnknown(options_.camera_backend);
        out << "transport" << orUnknown(options_.camera_transport);
        out << "pixel_format" << orUnknown(options_.camera_pixel_format);
        out << "exposure_us" << options_.camera_exposure_us;
        out << "gain" << options_.camera_gain;
        out << "trigger_mode" << orUnknown(options_.camera_trigger_mode);
        out << "sdk_version" << orUnknown(options_.camera_sdk_version);
        out << "resolution" << "{" << "width" << options_.image_width
            << "height" << options_.image_height << "}";
        out << "roi" << "{" << "x" << 0 << "y" << 0
            << "width" << options_.image_width << "height" << options_.image_height
            << "provenance" << "requested_output_frame_camera_api_does_not_expose_sensor_roi"
            << "}";
        out << "binning" << "{" << "known" << 0 << "value" << 0 << "}";
        out << "decimation" << "{" << "known" << 0 << "value" << 0 << "}";
        out << "scaling" << "{" << "known" << 0 << "value" << 0.0 << "}";
        out << "rotation" << "{" << "known" << 0 << "degrees" << 0 << "}";
        out << "}";
        out << "intrinsic" << "{";
        out << "source" << options_.intrinsic_source;
        out << "source_file" << options_.intrinsic_source_file;
        out << "source_file_sha256" << intrinsicFileHash;
        out << "source_calibration_timestamp" << "unknown";
        out << "loaded_at_utc" << timestamp_utc_;
        out << "intrinsics_sha256"
            << intrinsicFingerprint(options_.camera_matrix, options_.dist_coeffs);
        out << "intrinsics_binding_sha256"
            << intrinsicBindingFingerprint(options_);
        out << "distortion_model" << "opencv_plumb_bob";
        out << "camera_matrix" << options_.camera_matrix;
        out << "distortion_coefficients" << options_.dist_coeffs;
        out << "}";
        out << "robot" << "{";
        out << "endpoint" << orUnknown(options_.robot_endpoint);
        out << "model" << orUnknown(options_.robot_model);
        out << "controller" << orUnknown(options_.robot_controller);
        out << "raw_joint_unit" << "degree";
        out << "raw_rotation_representation" << "axis_angle";
        out << "normalized_translation_unit" << "meter";
        out << "normalized_rotation_representation" << "rotation_matrix";
        out << "coordinate_system" << "right_handed";
        out << "handeye_matrix_definition" << "T_flange_camera";
        out << "}";
        if (!options_.pose_offset_x.empty()) {
            out << "candidate_planning" << "{";
            out << "source_file" << options_.pose_config_source_file;
            out << "source_file_sha256" << poseConfigHash;
            out << "offset_x" << options_.pose_offset_x;
            out << "offset_y" << options_.pose_offset_y;
            out << "offset_z" << options_.pose_offset_z;
            out << "offset_rx" << options_.pose_offset_rx;
            out << "offset_ry" << options_.pose_offset_ry;
            out << "offset_rz" << options_.pose_offset_rz;
            out << "min_corners" << options_.pose_min_corners;
            out << "min_valid_frames" << options_.pose_min_valid_frames;
            out << "maximum_candidates" << options_.pose_maximum_candidates;
            out << "maximum_joint_step_degrees"
                << options_.pose_maximum_joint_step_degrees;
            out << "}";
        }
        out << "board" << "{";
        out << "dictionary_id" << options_.board.dictionary_id;
        out << "squares_x" << options_.board.squares_x;
        out << "squares_y" << options_.board.squares_y;
        out << "square_length_m" << options_.board.square_length;
        out << "marker_length_m" << options_.board.marker_length;
        out << "board_id" << "unknown";
        out << "print_scale" << "unknown";
        out << "}";
        out << "timing" << "{";
        out << "timestamp_unit" << "nanosecond";
        out << "clock_domain" << "host_system_clock";
        out << "sync_mode" << "software_sequential";
        out << "}";
    }, error);
}

bool CalibrationSession::writeSessionYaml(const std::string& lifecycle_status,
                                          const std::string& result_status,
                                          std::string& error) {
    const std::string executable = executablePath();
    const std::string configHash = sha256File(
        (fs::path(directory_) / "effective_config.yaml").string());
    return atomicYamlWrite(fs::path(directory_) / "session.yaml",
                           [&](cv::FileStorage& out) {
        out << "schema_version" << 1;
        out << "session_id" << id_;
        out << "timestamp_utc" << timestamp_utc_;
        out << "lifecycle_status" << lifecycle_status;
        out << "result_status" << result_status;
        out << "current_stage" << current_stage_;
        out << "publication_state"
            << (fs::exists(fs::path(directory_) / "publication.commit.yaml")
                    ? "published"
                    : (fs::exists(fs::path(directory_) / "publication.intent.yaml")
                           ? "prepared" : "not_started"));
        out << "mode" << options_.mode;
        out << "source_data_directory" << options_.source_data_directory;
        out << "software" << "{";
        out << "version" << APP_VERSION;
        out << "build_type" << APP_BUILD_TYPE;
        out << "build_timestamp" << APP_BUILD_TIMESTAMP;
        out << "git_commit" << APP_GIT_COMMIT;
        out << "git_worktree_state" << APP_GIT_WORKTREE_STATE;
        out << "camera_backend" << APP_CAMERA_BACKEND;
        out << "opencv_version" << CV_VERSION;
        out << "executable" << executable;
        out << "executable_sha256" << sha256File(executable);
        out << "}";
        out << "effective_config_file" << "effective_config.yaml";
        out << "effective_config_sha256" << configHash;
        out << "lineage" << "{";
        out << "source_session_id" << options_.source_session_id;
        out << "source_manifest_sha256" << options_.source_manifest_sha256;
        out << "}";
    }, error);
}

bool CalibrationSession::writeReport(const std::string& status,
                                     const std::string& failure_stage,
                                     const std::string& failure_reason,
                                     const cv::Mat& transform,
                                     double translation_rms_mm,
                                     int valid_frames,
                                     const std::string& message,
                                     std::string& error) {
    int readableFrames = 0;
    int charucoFrames = 0;
    int pnpFrames = 0;
    int madInliers = 0;
    int acceptedFrames = 0;
    for (const CalibrationFrameAudit& frame : frames_) {
        readableFrames += frame.image_read_pass ? 1 : 0;
        charucoFrames += frame.charuco_pass ? 1 : 0;
        pnpFrames += frame.pnp_pass ? 1 : 0;
        madInliers += frame.mad_filter_pass ? 1 : 0;
        acceptedFrames += frame.final_accepted ? 1 : 0;
    }
    return atomicYamlWrite(fs::path(directory_) / "calibration_report.yaml",
                           [&](cv::FileStorage& out) {
        out << "schema_version" << 1;
        out << "session_id" << id_;
        out << "provenance" << "{";
        out << "session_file" << "session.yaml";
        out << "effective_config_file" << "effective_config.yaml";
        out << "manifest_file" << "manifest.sha256";
        out << "source_session_id" << options_.source_session_id;
        out << "source_manifest_sha256" << options_.source_manifest_sha256;
        out << "}";
        out << "result" << "{";
        out << "status" << status;
        out << "failure_stage" << failure_stage;
        out << "failure_reason" << failure_reason;
        out << "message" << message;
        out << "valid_frames" << valid_frames;
        out << "translation_consistency_rms_mm" << translation_rms_mm;
        out << "T_flange_camera" << transform;
        out << "}";
        out << "frame_summary" << "{";
        out << "total" << static_cast<int>(frames_.size());
        out << "image_read_pass" << readableFrames;
        out << "charuco_pass" << charucoFrames;
        out << "pnp_pass" << pnpFrames;
        out << "mad_inliers" << madInliers;
        out << "final_accepted" << acceptedFrames;
        out << "}";
        out << "hand_eye_solver_diagnostics" << "{";
        out << "raw_results_are_unoptimized" << 1;
        out << "relative_transform_definition" << "inverse(X_left)*X_right";
        out << "medoid_metric"
            << "sum(delta_translation/translation_limit+delta_rotation/rotation_limit)";
        out << "refinement_policy" << "once_per_phase_selected_medoid_only";
        out << "solvers" << "[";
        for (const HandEyeSolverAuditRecord& solver : hand_eye_phase2_audit_.solvers) {
            out << "{";
            out << "phase" << solver.phase;
            out << "method" << solver.method;
            out << "success" << static_cast<int>(solver.success);
            out << "failure_reason" << solver.failure_reason;
            out << "raw_transform" << solver.raw_transform;
            out << "raw_consistency" << "{";
            out << "translation_rms_mm" << solver.raw_translation_rms_mm;
            out << "translation_p95_mm" << solver.raw_translation_p95_mm;
            out << "rotation_rms_deg" << solver.raw_rotation_rms_deg;
            out << "rotation_p95_deg" << solver.raw_rotation_p95_deg;
            out << "}";
            out << "selected_medoid" << static_cast<int>(solver.selected_medoid);
            out << "medoid_normalized_distance_sum"
                << solver.medoid_normalized_distance_sum;
            out << "}";
        }
        out << "]";
        out << "pairwise_se3" << "[";
        for (const HandEyeSolverPairAuditRecord& pair : hand_eye_phase2_audit_.pairs) {
            out << "{";
            out << "phase" << pair.phase;
            out << "left_method" << pair.left_method;
            out << "right_method" << pair.right_method;
            out << "delta_translation_mm" << pair.delta_translation_mm;
            out << "delta_rotation_deg" << pair.delta_rotation_deg;
            out << "}";
        }
        out << "]";
        out << "consensus" << "[";
        for (const HandEyeConsensusAuditRecord& consensus :
             hand_eye_phase2_audit_.consensus) {
            out << "{";
            out << "phase" << consensus.phase;
            out << "successful_solvers" << consensus.successful_solvers;
            out << "medoid_method" << consensus.medoid_method;
            out << "max_pair_translation_mm" << consensus.max_pair_translation_mm;
            out << "max_pair_rotation_deg" << consensus.max_pair_rotation_deg;
            out << "translation_limit_mm" << consensus.translation_limit_mm;
            out << "rotation_limit_deg" << consensus.rotation_limit_deg;
            out << "passed" << static_cast<int>(consensus.passed);
            out << "failure_reason" << consensus.failure_reason;
            out << "}";
        }
        out << "]";
        out << "optimizations" << "[";
        for (const HandEyeOptimizationAuditRecord& optimization :
             hand_eye_phase2_audit_.optimizations) {
            out << "{";
            out << "performed" << static_cast<int>(optimization.performed);
            out << "phase" << optimization.phase;
            out << "method" << optimization.method;
            out << "initial_transform" << optimization.initial_transform;
            out << "optimized_transform" << optimization.optimized_transform;
            out << "delta_translation_mm" << optimization.delta_translation_mm;
            out << "delta_rotation_deg" << optimization.delta_rotation_deg;
            out << "before_translation_rms_mm"
                << optimization.before_translation_rms_mm;
            out << "after_translation_rms_mm"
                << optimization.after_translation_rms_mm;
            out << "before_rotation_rms_deg"
                << optimization.before_rotation_rms_deg;
            out << "after_rotation_rms_deg"
                << optimization.after_rotation_rms_deg;
            out << "}";
        }
        out << "]";
        out << "}";
        out << "hand_eye_loo_diagnostics" << "{";
        out << "enabled" << static_cast<int>(hand_eye_phase3_audit_.enabled);
        out << "completed" << static_cast<int>(hand_eye_phase3_audit_.completed);
        out << "status" << hand_eye_phase3_audit_.status;
        out << "failure_reason" << hand_eye_phase3_audit_.failure_reason;
        out << "full_selected_method"
            << hand_eye_phase3_audit_.full_selected_method;
        out << "worker_count" << hand_eye_phase3_audit_.worker_count;
        out << "comparison_definition" << "inverse(X_full)*X_without_frame";
        out << "fixed_inlier_policy"
            << "remove_one_from_final_inliers_lock_full_solver_without_rerunning_mad";
        out << "full_pipeline_policy"
            << "remove_one_then_rerun_mad_consensus_medoid_refinement";
        out << "full_transform" << hand_eye_phase3_audit_.full_transform;
        writeIntSequence(out, "full_inlier_source_indices",
                         hand_eye_phase3_audit_.full_inlier_source_indices);
        out << "summaries" << "[";
        for (const HandEyeLooSummaryAuditRecord& summary :
             hand_eye_phase3_audit_.summaries) {
            out << "{";
            out << "mode" << summary.mode;
            out << "attempted" << summary.attempted;
            out << "successful" << summary.successful;
            out << "failed" << summary.failed;
            out << "translation" << "{";
            out << "mean_mm" << summary.translation_mean_mm;
            out << "p95_mm" << summary.translation_p95_mm;
            out << "max_mm" << summary.translation_max_mm;
            out << "max_frame_id" << summary.translation_max_frame_id;
            out << "max_source_index" << summary.translation_max_source_index;
            out << "}";
            out << "rotation" << "{";
            out << "mean_deg" << summary.rotation_mean_deg;
            out << "p95_deg" << summary.rotation_p95_deg;
            out << "max_deg" << summary.rotation_max_deg;
            out << "max_frame_id" << summary.rotation_max_frame_id;
            out << "max_source_index" << summary.rotation_max_source_index;
            out << "}";
            out << "trials_with_filter_set_changes"
                << summary.trials_with_filter_set_changes;
            out << "}";
        }
        out << "]";
        out << "trials" << "[";
        for (const HandEyeLooTrialAuditRecord& trial :
             hand_eye_phase3_audit_.trials) {
            out << "{";
            out << "mode" << trial.mode;
            out << "removed_frame_id" << trial.removed_frame_id;
            out << "removed_source_index" << trial.removed_source_index;
            out << "success" << static_cast<int>(trial.success);
            out << "failure_reason" << trial.failure_reason;
            out << "selected_method" << trial.selected_method;
            out << "final_inliers" << trial.final_inliers;
            out << "delta_translation_mm" << trial.delta_translation_mm;
            out << "delta_rotation_deg" << trial.delta_rotation_deg;
            out << "filter_set_evaluated"
                << static_cast<int>(trial.filter_set_evaluated);
            writeIntSequence(out, "filter_set_added_source_indices",
                             trial.filter_set_added_source_indices);
            writeIntSequence(out, "filter_set_removed_source_indices",
                             trial.filter_set_removed_source_indices);
            out << "}";
        }
        out << "]";
        out << "}";
        out << "frames" << "[";
        for (const CalibrationFrameAudit& frame : frames_) {
            out << "{";
            out << "frame_id" << frame.frame_id;
            out << "source_index" << frame.source_index;
            out << "image" << "{";
            out << "file" << frame.image_file;
            out << "sha256" << frame.image_sha256;
            out << "timestamp" << "{";
            out << "value" << std::to_string(frame.image_timestamp_ns);
            out << "unit" << "ns" << "clock_domain"
                << (frame.image_timestamp_ns == 0 ? "unavailable" : "host_system_clock");
            out << "}";
            out << "width" << frame.image_width << "height" << frame.image_height;
            out << "}";
            out << "robot" << "{";
            out << "timestamp" << "{";
            out << "value" << std::to_string(frame.robot_timestamp_ns);
            out << "unit" << "ns" << "clock_domain"
                << (frame.robot_timestamp_ns == 0 ? "unavailable" : "host_system_clock");
            out << "}";
            out << "T_base_flange" << frame.T_base_flange;
            out << "}";
            out << "charuco" << "{" << "pass" << static_cast<int>(frame.charuco_pass)
                << "corners" << frame.corner_count << "coverage" << frame.coverage << "}";
            out << "pnp" << "{" << "pass" << static_cast<int>(frame.pnp_pass)
                << "reprojection_rms_px" << frame.reprojection_rms_px
                << "reprojection_p95_px" << frame.reprojection_p95_px
                << "T_camera_board" << frame.T_camera_board << "}";
            out << "filtering" << "{";
            out << "image_read_pass" << static_cast<int>(frame.image_read_pass);
            out << "quality_gate_pass" << static_cast<int>(frame.quality_gate_pass);
            out << "mad_filter_pass" << static_cast<int>(frame.mad_filter_pass);
            out << "consistency_error_mm" << frame.consistency_error_mm;
            out << "final_accepted" << static_cast<int>(frame.final_accepted);
            out << "reject_stage" << frame.reject_stage;
            writeStringSequence(out, "reject_reasons", frame.reject_reasons);
            out << "}";
            out << "}";
        }
        out << "]";
    }, error);
}

bool CalibrationSession::finalizeSuccess(const cv::Mat& transform,
                                         const cv::Mat& camera_matrix,
                                         const cv::Mat& dist_coeffs,
                                         double translation_rms_mm,
                                         int valid_frames,
                                         const std::string& message,
                                         const std::string& production_yaml,
                                         std::string& error) {
    if (finalized_)
        return true;
    options_.camera_matrix = camera_matrix.clone();
    options_.dist_coeffs = dist_coeffs.clone();
    current_stage_ = "complete";
    if (!writeEffectiveConfig(error) ||
        !writeReport("pass", "", "", transform, translation_rms_mm,
                     valid_frames, message, error))
        return false;
    if (!atomicYamlWrite(fs::path(directory_) / "handeye.yaml",
                         [&](cv::FileStorage& out) {
        out << "T_flange_camera" << transform;
        out << "camera_matrix" << camera_matrix;
        out << "distortion_coefficients" << dist_coeffs;
        out << "reprojection_error_mm" << translation_rms_mm;
        out << "valid_frames" << valid_frames;
        out << "source_production_yaml" << production_yaml;
    }, error))
        return false;

    if (!production_yaml.empty()) {
        const fs::path candidate = fs::path(directory_) / "publication.candidate.yaml";
        if (!atomicYamlWrite(candidate, [&](cv::FileStorage& out) {
            out << "T_flange_camera" << transform;
            out << "camera_matrix" << camera_matrix;
            out << "distortion_coefficients" << dist_coeffs;
            out << "reprojection_error_mm" << translation_rms_mm;
            out << "valid_frames" << valid_frames;
        }, error))
            return false;

        const fs::path target = fs::absolute(fs::path(production_yaml)).lexically_normal();
        std::error_code ec;
        const bool previousExists = fs::exists(target, ec);
        if (ec) {
            error = "cannot inspect previous production YAML: " + ec.message();
            return false;
        }
        const std::string candidateHash = sha256File(candidate.string());
        const std::string previousHash = previousExists ? sha256File(target.string()) : "";
        if (candidateHash.empty() || (previousExists && previousHash.empty())) {
            error = "cannot hash publication candidate or previous production YAML";
            return false;
        }
        if (!atomicYamlWrite(fs::path(directory_) / "publication.intent.yaml",
                             [&](cv::FileStorage& out) {
            out << "schema_version" << 1;
            out << "session_id" << id_;
            out << "target_file" << target.string();
            out << "candidate_file" << candidate.filename().string();
            out << "candidate_sha256" << candidateHash;
            out << "previous_exists" << static_cast<int>(previousExists);
            out << "previous_sha256" << previousHash;
            out << "state" << "prepared";
            out << "prepared_at_utc" << utcNow();
        }, error))
            return false;

        maybeInjectTestCrash("publication_before_rename");
        if (!atomicFileCopy(candidate, target, error))
            return false;
        maybeInjectTestCrash("publication_after_rename");

        if (!atomicYamlWrite(fs::path(directory_) / "publication.commit.yaml",
                             [&](cv::FileStorage& out) {
            out << "schema_version" << 1;
            out << "session_id" << id_;
            out << "target_file" << target.string();
            out << "candidate_sha256" << candidateHash;
            out << "state" << "committed";
            out << "committed_at_utc" << utcNow();
            out << "recovered_after_interruption" << 0;
        }, error))
            return false;
    }

    if (!writeSessionYaml("sealed", "pass", error))
        return false;
    if (!appendDurableEvent("session", "sealed", "pass", error))
        return false;
    maybeInjectTestCrash("manifest_before_write");
    if (!writeManifest(error))
        return false;
    if (!durableRemove(fs::path(directory_) / ".running", error))
        return false;
    finalized_ = true;
    return true;
}

bool CalibrationSession::finalizeFailure(const std::string& stage,
                                         const std::string& reason,
                                         const std::string& message) {
    if (finalized_)
        return true;
    terminalization_attempted_ = true;
    if (fs::exists(fs::path(directory_) / "publication.intent.yaml")) {
        std::string ignored;
        appendDurableEvent("session", "pending_recovery",
                           "failure_after_publication_intent stage=" + stage +
                           " reason=" + reason, ignored);
        return false;
    }
    current_stage_ = stage;
    std::string error;
    cv::Mat empty;
    bool ok = writeReport("fail", stage, reason, empty, 0.0, 0, message, error) &&
              writeSessionYaml("sealed", "fail", error) &&
              appendDurableEvent("session", "sealed",
                                 "fail stage=" + stage + " reason=" + reason, error) &&
              writeManifest(error);
    if (ok)
        ok = durableRemove(fs::path(directory_) / ".running", error);
    finalized_ = ok;
    return ok;
}

void CalibrationSession::markInterruptedNoThrow() {
    if (fs::exists(fs::path(directory_) / "publication.intent.yaml"))
        return;
    std::string error;
    cv::Mat empty;
    current_stage_ = current_stage_.empty() ? "unknown" : current_stage_;
    const bool ok = writeReport("interrupted", current_stage_,
                                "session_destructor_without_finalize",
                                empty, 0.0, 0, "标定会话未正常结束", error) &&
                    writeSessionYaml("sealed", "interrupted", error) &&
                    appendDurableEvent("session", "sealed", "interrupted", error) &&
                    writeManifest(error) &&
                    durableRemove(fs::path(directory_) / ".running", error);
    finalized_ = ok;
}

bool CalibrationSession::writeManifest(std::string& error) {
    return writeManifestForDirectory(directory_, error);
}

bool CalibrationSession::recoverPendingSessions(
    const std::string& base_directory,
    std::vector<std::string>& recovered_sessions,
    std::string& error) {
    recovered_sessions.clear();
    error.clear();
    if (base_directory.empty()) {
        error = "session base directory is empty";
        return false;
    }
    return recoverInterruptedSessions(fs::path(base_directory) / "sessions",
                                      recovered_sessions, error);
}

std::string CalibrationSession::sha256File(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input)
        return "";
    Sha256 hash;
    std::array<unsigned char, 64 * 1024> buffer{};
    while (input) {
        input.read(reinterpret_cast<char*>(buffer.data()), buffer.size());
        const std::streamsize count = input.gcount();
        if (count > 0)
            hash.update(buffer.data(), static_cast<size_t>(count));
    }
    return hash.finish();
}

bool CalibrationSession::verifyManifest(const std::string& session_directory,
                                        std::vector<std::string>& problems) {
    problems.clear();
    std::ifstream manifest(fs::path(session_directory) / "manifest.sha256");
    if (!manifest) {
        problems.push_back("manifest_missing");
        return false;
    }
    std::vector<std::string> expectedFiles;
    std::string line;
    while (std::getline(manifest, line)) {
        if (line.size() < 67 || line.substr(64, 2) != "  ") {
            problems.push_back("manifest_malformed:" + line);
            continue;
        }
        const std::string expected = line.substr(0, 64);
        const std::string relative = line.substr(66);
        if (!std::all_of(expected.begin(), expected.end(), [](unsigned char value) {
                return std::isxdigit(value) != 0;
            })) {
            problems.push_back("manifest_hash_malformed:" + relative);
            continue;
        }
        const fs::path relativePath(relative);
        bool unsafePath = relative.empty() || relativePath.is_absolute();
        for (const fs::path& component : relativePath) {
            if (component == "..")
                unsafePath = true;
        }
        if (unsafePath) {
            problems.push_back("manifest_unsafe_path:" + relative);
            continue;
        }
        if (std::find(expectedFiles.begin(), expectedFiles.end(), relative) !=
            expectedFiles.end()) {
            problems.push_back("manifest_duplicate:" + relative);
            continue;
        }
        expectedFiles.push_back(relative);
        const fs::path file = fs::path(session_directory) / relativePath;
        if (!fs::exists(file)) {
            problems.push_back("missing:" + relative);
            continue;
        }
        if (fs::is_symlink(file)) {
            problems.push_back("symlink_not_allowed:" + relative);
            continue;
        }
        if (sha256File(file.string()) != expected)
            problems.push_back("hash_mismatch:" + relative);
    }
    std::sort(expectedFiles.begin(), expectedFiles.end());
    std::error_code ec;
    for (const auto& entry : fs::recursive_directory_iterator(session_directory, ec)) {
        if (ec || !entry.is_regular_file())
            continue;
        if (excludedFromManifest(entry.path()))
            continue;
        const std::string relative = relativeOrFilename(entry.path(), session_directory);
        if (!std::binary_search(expectedFiles.begin(), expectedFiles.end(), relative))
            problems.push_back("unexpected:" + relative);
    }
    return problems.empty();
}
