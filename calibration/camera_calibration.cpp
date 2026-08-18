#include "camera_calibration.hpp"

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <random>
#include <filesystem>

namespace {

// ——— 分批标定 + 逐帧过滤 + 最终标定 ———
// 每批最多 batch_size 帧，标定后只取误差最小的前 30% 进入候选池，
// 最后从候选池取误差最小的 max_good 帧做最终标定。
static bool calibrateCameraFiltered(
    const std::vector<std::vector<cv::Point3f>>& object_points,
    const std::vector<std::vector<cv::Point2f>>& image_points,
    const cv::Size& image_size,
    cv::Mat& camera_matrix,
    cv::Mat& dist_coeffs) {
    int n_final = 65;
    float fRatio = sqrt((float)n_final / image_points.size());
    int  wantedNum =std::max<int>( (int)(image_points.size() * fRatio), std::min<int>(image_points.size(), n_final * 2));
    std::vector<std::vector<cv::Point3f>> object_points_l = object_points;
    std::vector<std::vector<cv::Point2f>> image_points_l = image_points;
    if(filterDataForCameraCalibration(object_points_l, image_points_l, image_size, camera_matrix, dist_coeffs, wantedNum))
       filterDataForCameraCalibration(object_points_l, image_points_l, image_size, camera_matrix, dist_coeffs, n_final);


    camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    double final_err;
    try {
        final_err = cv::calibrateCamera(object_points_l, image_points_l, image_size,
                                        camera_matrix, dist_coeffs,
                                        rvecs, tvecs);
    } catch (const cv::Exception& e) {
        std::cerr << "[autoCalib] 最终相机标定异常: " << e.what() << std::endl;
        return false;
    }
    std::cout << "[autoCalib] 最终标定, " << object_points_l.size()
              << " 帧, 重投影误差: "
              << final_err << " px" << std::endl;
    std::cout << "[autoCalib] 相机矩阵:\n" << camera_matrix << std::endl;
    return true;
}

// 相机内参精化: 用全部有效帧数据做离群过滤后重标定
// K, D 入/出: 初始值来自 filtered 管线, 返回精化结果
static void refineCameraIntrinsics(
    cv::Mat& K, cv::Mat& D,
    const std::vector<std::vector<cv::Point3f>>& all_obj,
    const std::vector<std::vector<cv::Point2f>>& all_img,
    const cv::Size& image_size) {

    int n = (int)all_obj.size();
    if (n < 5) return;

    // 逐帧 solvePnP + 重投影误差 (快: ~0.2ms/帧)
    struct Rec { int idx; double err; };
    std::vector<Rec> recs;
    recs.reserve(n);
    for (int i = 0; i < n; i++) {
        cv::Mat rvec, tvec;
        if (!cv::solvePnP(all_obj[i], all_img[i], K, D, rvec, tvec))
            continue;
        std::vector<cv::Point2f> proj;
        cv::projectPoints(all_obj[i], rvec, tvec, K, D, proj);
        double sum = 0.0;
        for (size_t j = 0; j < proj.size(); j++)
            sum += cv::norm(proj[j] - all_img[i][j]);
        if (proj.empty())
            continue;
        recs.push_back({i, sum / proj.size()});
    }
    if ((int)recs.size() < 5) return;

    std::sort(recs.begin(), recs.end(),
              [](const Rec& a, const Rec& b) { return a.err < b.err; });

    // solvePnP 可能会跳过退化帧，因此中位数索引必须基于 recs，
    // 不能继续使用输入帧数量，否则会发生越界访问。
    double median = recs[recs.size() / 2].err;
    double thresh = median * 1.2;

    std::vector<std::vector<cv::Point3f>> good_obj;
    std::vector<std::vector<cv::Point2f>> good_img;
    for (auto& r : recs) {
        if (r.err <= thresh) {
            good_obj.push_back(all_obj[r.idx]);
            good_img.push_back(all_img[r.idx]);
        }
    }
    std::cout << "[refineCam] 重投影过滤: " << good_obj.size() << "/" << n
              << " 帧保留 (median=" << median << " px, thresh=" << thresh << " px)" << std::endl;
    if ((int)good_obj.size() < 5) return;

    std::vector<cv::Mat> rvecs, tvecs;
    double final_err;
    try {
        final_err = cv::calibrateCamera(good_obj, good_img, image_size,
                                        K, D, rvecs, tvecs,
                                        cv::CALIB_USE_INTRINSIC_GUESS);
    } catch (const cv::Exception& e) {
        std::cerr << "[refineCam] 相机内参精化异常: " << e.what() << std::endl;
        return;
    }
    std::cout << "[refineCam] 精化完成: " << good_obj.size() << " 帧, err=" << final_err << " px" << std::endl;
    std::cout << "[autoCalib] 相机矩阵:\n" << K << std::endl;
}

} // anonymous namespace

// ============================================================
// 公开 API
// ============================================================

bool loadImagesFromDisk(const std::string& save_dir,
                         std::vector<std::string>& img_files,
                         cv::Size& image_size) {
    namespace fs = std::filesystem;
    for (auto& entry : fs::directory_iterator(save_dir)) {
        std::string name = entry.path().filename().string();
        if (name.size() >= 4 && name.substr(name.size() - 4) == ".jpg")
            img_files.push_back(entry.path().string());
    }
    std::sort(img_files.begin(), img_files.end());
    if (img_files.empty()) return false;
    cv::Mat first = cv::imread(img_files[0]);
    if (first.empty()) return false;
    image_size = first.size();
    return true;
}

bool filterDataForCameraCalibration(std::vector<std::vector<cv::Point3f>>& object_points,
    std::vector<std::vector<cv::Point2f>>& image_points,
    const cv::Size& image_size,
    cv::Mat& camera_matrix,
    cv::Mat& dist_coeffs, int wantedNum){
    const int min_frames = 5;
    constexpr size_t target_batch_size = 50;

    if (object_points.size() != image_points.size()) {
        std::cerr << "[autoCalib] 物点/像点帧数不一致: "
                  << object_points.size() << "/" << image_points.size() << std::endl;
        return false;
    }

    const size_t n_total = object_points.size();
    if (n_total < static_cast<size_t>(min_frames)) {
        std::cerr << "[autoCalib] 有效帧数不足: " << n_total
                  << " (need >=" << min_frames << ")" << std::endl;
        return false;
    }

    const float keepRatio = std::max<float>(
        0.3f,
        std::min<float>((static_cast<float>(wantedNum) * 1.5f) /
                            static_cast<float>(n_total) + 0.1f,
                        1.0f));

    // 保留原有的分批方式，只把批次数下限限制为 1。旧代码在
    // n_total < 50 时 nTimes 为 0，下一行会触发整数除零（SIGFPE）。
    const size_t nTimes = std::max<size_t>(
        1, n_total / target_batch_size);
    const size_t batch_size = n_total / nTimes + 1;


    struct FrameRecord {
        std::vector<cv::Point3f> obj_pts;
        std::vector<cv::Point2f> img_pts;
        double error;
    };
    std::vector<FrameRecord> good_frames;

    // ——— 分批处理 ———
    for (size_t start = 0; start < n_total; start += batch_size) {
        size_t end = std::min(start + batch_size, n_total);
        size_t n_batch = end - start;

        std::vector<std::vector<cv::Point3f>> batch_obj(
            object_points.begin() + start, object_points.begin() + end);
        std::vector<std::vector<cv::Point2f>> batch_img(
            image_points.begin() + start, image_points.begin() + end);

        cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
        std::vector<cv::Mat> rvecs, tvecs;
        double err;
        try {
            err = cv::calibrateCamera(batch_obj, batch_img, image_size,
                                      K, D, rvecs, tvecs);
        } catch (const cv::Exception& e) {
            std::cerr << "[autoCalib] 批次相机标定异常: " << e.what() << std::endl;
            return false;
        }
        if (rvecs.size() != n_batch || tvecs.size() != n_batch) {
            std::cerr << "[autoCalib] 批次标定输出数量异常" << std::endl;
            return false;
        }
        std::cout << "[autoCalib] 批次 " << (start / batch_size) << "/" <<nTimes
                  << ": , 重投影误差 " << err << " px" << std::endl;

        // 计算每帧投影误差
        std::vector<FrameRecord> batch_records;
        batch_records.reserve(n_batch);
        for (size_t i = 0; i < n_batch; i++) {
            std::vector<cv::Point2f> proj;
            cv::projectPoints(batch_obj[i], rvecs[i], tvecs[i], K, D, proj);
            double sum_err = 0.0;
            for (size_t j = 0; j < proj.size(); j++)
                sum_err += cv::norm(proj[j] - batch_img[i][j]);
            if (proj.empty())
                continue;
            batch_records.push_back({batch_obj[i], batch_img[i], sum_err / proj.size()});
        }

        if (batch_records.empty()) {
            std::cerr << "[autoCalib] 批次内没有可用的投影结果" << std::endl;
            return false;
        }

        // 取前 30%（误差最小）
        std::sort(batch_records.begin(), batch_records.end(),
                  [](const FrameRecord& a, const FrameRecord& b) {
                      return a.error < b.error;
                  });
        size_t n_keep = std::max<size_t>(
            1, static_cast<size_t>(batch_records.size() * keepRatio));
        n_keep = std::min(n_keep, batch_records.size());
        for (size_t i = 0; i < n_keep; i++)
            good_frames.push_back(batch_records[i]);
        std::cout << "[autoCalib]  保留 " << n_keep << "/" << n_batch
                  << " 帧 (前"<< 100 * keepRatio <<"%, 最大误差 " << batch_records[n_keep - 1].error << " px)"
                  << std::endl;
    }

    // ——— 取误差最小的 max_good 帧做最终标定 ———
    std::sort(good_frames.begin(), good_frames.end(),
              [](const FrameRecord& a, const FrameRecord& b) {
                  return a.error < b.error;
              });

    size_t n_final = std::min(good_frames.size(), size_t(wantedNum));
    if (n_final < (size_t)min_frames) {
        return false;
    }

    std::cout << "[autoCalib] 保留 " << n_final << "/" << n_total
              << " 帧做最终标定" << std::endl;

    std::vector<std::vector<cv::Point3f>> final_obj;
    std::vector<std::vector<cv::Point2f>> final_img;
    final_obj.reserve(n_final);
    final_img.reserve(n_final);
    good_frames.erase(good_frames.begin() + n_final, good_frames.end());
    std::random_device rd;
    std::shuffle(good_frames.begin(), good_frames.end(), std::default_random_engine(rd()));
    for (size_t i = 0; i < n_final; i++) {
        final_obj.push_back(good_frames[i].obj_pts);
        final_img.push_back(good_frames[i].img_pts);
    }
    object_points = final_obj;
    image_points = final_img;
    return true;
}

bool calibrateCameraFromDisk(const std::string& save_dir,
                              CharucoDetector& detector,
                              cv::Mat& camera_matrix,
                              cv::Mat& dist_coeffs) {

    const auto& board_points = detector.getChessboardCorners3D();

    std::vector<std::string> img_files;
    cv::Size image_size;
    if (!loadImagesFromDisk(save_dir, img_files, image_size))
        return false;

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    std::random_device rd;
    std::shuffle(img_files.begin(), img_files.end(), std::default_random_engine(rd()));

    for (const auto& path : img_files) {
        cv::Mat img = cv::imread(path);
        if (img.empty()) continue;

        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        if (!detector.detectCorners(img, corners, ids))
            continue;

        std::vector<cv::Point3f> pts3;
        std::vector<cv::Point2f> pts2;
        for (size_t j = 0; j < ids.size(); j++) {
            int id = ids[j];
            if (id >= 0 && (size_t)id < board_points.size()) {
                pts3.push_back(board_points[id]);
                pts2.push_back(corners[j]);
            }
        }
        if (pts3.size() >= 16) {
            object_points.push_back(pts3);
            image_points.push_back(pts2);
        }
    }

    if (object_points.size() < 5) {
        std::cerr << "[autoCalib] 帧数不足，无法标定相机内参 (need >=5, got "
                  << object_points.size() << ")" << std::endl;
        return false;
    }

    // 保存全部有效帧，过滤 + 精化用
    auto all_obj = object_points;
    auto all_img = image_points;

    camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    if (!calibrateCameraFiltered(object_points, image_points,
                                 image_size, camera_matrix, dist_coeffs))
        return false;

    // 以过滤后的 K,D 为初值，用全部有效帧做离群过滤 + 重标定
    refineCameraIntrinsics(camera_matrix, dist_coeffs,
                           all_obj, all_img, image_size);
    return true;
}
