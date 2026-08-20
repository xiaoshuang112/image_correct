#include "calibration_session.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <system_error>

#if defined(__linux__)
#include <unistd.h>
#endif

#if __has_include("AppBuildInfo.hpp")
#include "AppBuildInfo.hpp"
#endif

#ifndef APP_BUILD_TYPE
#define APP_BUILD_TYPE "unknown"
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

bool atomicRename(const fs::path& temporary,
                  const fs::path& destination,
                  std::string& error) {
    std::error_code ec;
    fs::rename(temporary, destination, ec);
    if (!ec)
        return true;
    fs::remove(temporary);
    error = "atomic rename failed: " + ec.message();
    return false;
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

void recoverInterruptedSessions(const fs::path& sessionsDirectory) {
    std::error_code ec;
    if (!fs::exists(sessionsDirectory, ec))
        return;
    for (const auto& entry : fs::directory_iterator(sessionsDirectory, ec)) {
        if (ec || !entry.is_directory())
            continue;
        const fs::path running = entry.path() / ".running";
        if (!fs::exists(running, ec))
            continue;
        const fs::path report = entry.path() / "calibration_report.yaml";
        if (fs::exists(report, ec)) {
            fs::copy_file(report, entry.path() / "calibration_report.partial.yaml",
                          fs::copy_options::overwrite_existing, ec);
            ec.clear();
        }
        const fs::path sessionFile = entry.path() / "session.yaml";
        if (fs::exists(sessionFile, ec)) {
            fs::copy_file(sessionFile, entry.path() / "session.partial.yaml",
                          fs::copy_options::overwrite_existing, ec);
            ec.clear();
        }
        std::string error;
        atomicYamlWrite(report,
                        [&](cv::FileStorage& out) {
            out << "schema_version" << 1;
            out << "session_id" << entry.path().filename().string();
            out << "result" << "{";
            out << "status" << "interrupted";
            out << "failure_stage" << "process_lifecycle";
            out << "failure_reason" << "previous_process_did_not_finalize_session";
            out << "recovered_at_utc" << utcNow();
            out << "partial_report" << "calibration_report.partial.yaml";
            out << "}";
        }, error);
        atomicYamlWrite(sessionFile, [&](cv::FileStorage& out) {
            out << "schema_version" << 1;
            out << "session_id" << entry.path().filename().string();
            out << "status" << "interrupted";
            out << "current_stage" << "process_lifecycle";
            out << "recovered_at_utc" << utcNow();
            out << "partial_session" << "session.partial.yaml";
            out << "effective_config_file" << "effective_config.yaml";
        }, error);
        std::ofstream log(entry.path() / "calibration.log", std::ios::app);
        log << utcNow() << " event=session_recovery result=interrupted"
            << " detail=previous_process_did_not_finalize_session\n";
        log.flush();

        std::vector<fs::path> files;
        ec.clear();
        for (const auto& recovered : fs::recursive_directory_iterator(entry.path(), ec)) {
            if (ec || !recovered.is_regular_file())
                continue;
            const std::string name = recovered.path().filename().string();
            if (name == "manifest.sha256" || name == ".running" ||
                name.find(".tmp.") != std::string::npos ||
                (name.size() >= 4 && name.substr(name.size() - 4) == ".tmp"))
                continue;
            files.push_back(recovered.path());
        }
        std::sort(files.begin(), files.end());
        std::ostringstream manifest;
        for (const fs::path& file : files)
            manifest << CalibrationSession::sha256File(file.string()) << "  "
                     << relativeOrFilename(file, entry.path()) << '\n';
        if (atomicTextWrite(entry.path() / "manifest.sha256", manifest.str(), error))
            fs::remove(running, ec);
    }
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
    if (!finalized_)
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
    recoverInterruptedSessions(sessionsDirectory);

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
    if (!atomicTextWrite(fs::path(directory_) / ".running", timestamp_utc_ + "\n", error))
        return false;
    appendEvent("session", "start", options_.mode);
    if (!writeEffectiveConfig(error) || !writeSessionYaml("running", error))
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
    writeSessionYaml("running", ignored);
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

void CalibrationSession::appendEvent(const std::string& event,
                                     const std::string& result,
                                     const std::string& detail) {
    if (directory_.empty())
        return;
    std::ofstream log(fs::path(directory_) / "calibration.log", std::ios::app);
    log << utcNow() << " event=" << event << " result=" << result;
    if (!detail.empty())
        log << " detail=\"" << detail << "\"";
    log << '\n';
    log.flush();
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
        out << "image_host_realtime_ns" << std::to_string(image_timestamp_ns);
        out << "robot_host_realtime_ns" << std::to_string(robot_timestamp_ns);
        out << "clock_domain" << "host_system_clock";
        out << "sync_mode" << "software_sequential";
        out << "measurement_order" << "image_then_robot_feedback";
        const long long offset = static_cast<long long>(robot_timestamp_ns) -
                                 static_cast<long long>(image_timestamp_ns);
        out << "offset_estimate_ms" << static_cast<double>(offset) / 1.0e6;
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
    return atomicYamlWrite(fs::path(directory_) / "effective_config.yaml",
                           [&](cv::FileStorage& out) {
        out << "schema_version" << 1;
        out << "camera" << "{";
        out << "serial" << orUnknown(options_.camera_serial);
        out << "model" << orUnknown(options_.camera_model);
        out << "firmware" << orUnknown(options_.camera_firmware);
        out << "endpoint" << orUnknown(options_.camera_endpoint);
        out << "stream" << options_.camera_stream;
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
        out << "effective_parameters_sha256"
            << intrinsicFingerprint(options_.camera_matrix, options_.dist_coeffs);
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

bool CalibrationSession::writeSessionYaml(const std::string& status,
                                          std::string& error) {
    const std::string executable = executablePath();
    const std::string configHash = sha256File(
        (fs::path(directory_) / "effective_config.yaml").string());
    return atomicYamlWrite(fs::path(directory_) / "session.yaml",
                           [&](cv::FileStorage& out) {
        out << "schema_version" << 1;
        out << "session_id" << id_;
        out << "timestamp_utc" << timestamp_utc_;
        out << "status" << status;
        out << "current_stage" << current_stage_;
        out << "mode" << options_.mode;
        out << "source_data_directory" << options_.source_data_directory;
        out << "software" << "{";
        out << "build_type" << APP_BUILD_TYPE;
        out << "build_timestamp" << APP_BUILD_TIMESTAMP;
        out << "git_commit" << APP_GIT_COMMIT;
        out << "git_worktree_state" << APP_GIT_WORKTREE_STATE;
        out << "opencv_version" << CV_VERSION;
        out << "executable" << executable;
        out << "executable_sha256" << sha256File(executable);
        out << "}";
        out << "effective_config_file" << "effective_config.yaml";
        out << "effective_config_sha256" << configHash;
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
    if (!writeSessionYaml("sealed", error))
        return false;
    appendEvent("session", "sealed", "pass");
    if (!writeManifest(error))
        return false;
    if (!production_yaml.empty()) {
        if (!atomicYamlWrite(production_yaml, [&](cv::FileStorage& out) {
            out << "T_flange_camera" << transform;
            out << "camera_matrix" << camera_matrix;
            out << "distortion_coefficients" << dist_coeffs;
            out << "reprojection_error_mm" << translation_rms_mm;
            out << "valid_frames" << valid_frames;
        }, error))
            return false;
    }
    std::error_code ignored;
    fs::remove(fs::path(directory_) / ".running", ignored);
    finalized_ = true;
    return true;
}

bool CalibrationSession::finalizeFailure(const std::string& stage,
                                         const std::string& reason,
                                         const std::string& message) {
    if (finalized_)
        return true;
    current_stage_ = stage;
    std::string error;
    cv::Mat empty;
    bool ok = writeReport("fail", stage, reason, empty, 0.0, 0, message, error) &&
              writeSessionYaml("sealed", error);
    appendEvent("session", "sealed", "fail stage=" + stage + " reason=" + reason);
    ok = writeManifest(error) && ok;
    std::error_code ignored;
    fs::remove(fs::path(directory_) / ".running", ignored);
    finalized_ = true;
    return ok;
}

void CalibrationSession::markInterruptedNoThrow() {
    std::string error;
    cv::Mat empty;
    current_stage_ = current_stage_.empty() ? "unknown" : current_stage_;
    writeReport("interrupted", current_stage_, "session_destructor_without_finalize",
                empty, 0.0, 0, "标定会话未正常结束", error);
    writeSessionYaml("sealed", error);
    appendEvent("session", "sealed", "interrupted");
    writeManifest(error);
    std::error_code ignored;
    fs::remove(fs::path(directory_) / ".running", ignored);
    finalized_ = true;
}

bool CalibrationSession::writeManifest(std::string& error) {
    std::vector<fs::path> files;
    std::error_code ec;
    for (const auto& entry : fs::recursive_directory_iterator(directory_, ec)) {
        if (ec) {
            error = "cannot scan session for manifest: " + ec.message();
            return false;
        }
        if (!entry.is_regular_file())
            continue;
        const std::string name = entry.path().filename().string();
        if (name == "manifest.sha256" || name == ".running" ||
            name.find(".tmp.") != std::string::npos ||
            (name.size() >= 4 && name.substr(name.size() - 4) == ".tmp"))
            continue;
        files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    std::ostringstream manifest;
    for (const fs::path& file : files)
        manifest << sha256File(file.string()) << "  "
                 << relativeOrFilename(file, directory_) << '\n';
    return atomicTextWrite(fs::path(directory_) / "manifest.sha256",
                           manifest.str(), error);
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
        expectedFiles.push_back(relative);
        const fs::path file = fs::path(session_directory) / relative;
        if (!fs::exists(file)) {
            problems.push_back("missing:" + relative);
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
        const std::string name = entry.path().filename().string();
        if (name == "manifest.sha256" || name == ".running" ||
            name.find(".tmp.") != std::string::npos ||
            (name.size() >= 4 && name.substr(name.size() - 4) == ".tmp"))
            continue;
        const std::string relative = relativeOrFilename(entry.path(), session_directory);
        if (!std::binary_search(expectedFiles.begin(), expectedFiles.end(), relative))
            problems.push_back("unexpected:" + relative);
    }
    return problems.empty();
}
