#include "DepthTester.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>
#include <filesystem>
#include <cmath>
#include <ctime>
#include "utils/robot_util.hpp"

namespace fs = std::filesystem;

DepthTester::DepthTester(std::shared_ptr<AICamera>& camera, int robot_handle,
                         const DepthTestConfig& config)
    : m_camera(camera), m_robot_handle(robot_handle), m_config(config) {}

bool DepthTester::moveToPoint(const DepthTestPoint& pt) {
    double pose[6];
    for (int i = 0; i < 6; i++)
        pose[i] = pt.flange_pose[i];
    return moveToFlangeAxisPos(m_robot_handle, pose);
}

bool DepthTester::captureDepth(double& out_depth) {
    for (int retry = 0; retry < 2; retry++) {
        cv::Mat bgr, depth;
        if (m_camera->captureOneFrameWithDepth(bgr, depth, m_config.capture_timeout_ms) != 0)
            continue;
        if (depth.empty()) continue;

        out_depth = extractCenterDepth(depth, m_config.center_ratio);
        if (out_depth > 0) return true;
    }
    return false;
}

void DepthTester::writeCsvHeader(std::ofstream& csv) {
    csv << "frame,point,raw_depth_mm,default_depth_mm,is_stale\n";
}

void DepthTester::writeCsvLine(std::ofstream& csv, int frame_id, const std::string& point_name,
                                double raw_depth, double default_depth, bool is_stale) {
    csv << frame_id << ","
        << point_name << ","
        << std::fixed << std::setprecision(2) << raw_depth << ","
        << std::fixed << std::setprecision(2) << default_depth << ","
        << (is_stale ? "true" : "false") << "\n";
}

int DepthTester::run() {
    const auto& cfg = m_config;
    fs::create_directories(cfg.save_dir);

    std::string csv_path = cfg.save_dir + "/depth_log.csv";
    std::ofstream csv(csv_path);
    if (!csv.is_open()) {
        std::cerr << "[DepthTester] 无法创建 CSV: " << csv_path << std::endl;
        return 1;
    }
    writeCsvHeader(csv);

    DepthHistory histA, histB;
    const DepthTestPoint& ptA = cfg.pointA;
    const DepthTestPoint& ptB = cfg.pointB;
    int frame_id = 0;

    std::cout << "\n========== 深度测试开始 ==========" << std::endl;
    std::cout << "  Point A: " << ptA.name << " [" << ptA.depth_min << "-" << ptA.depth_max << " mm]" << std::endl;
    std::cout << "  Point B: " << ptB.name << " [" << ptB.depth_min << "-" << ptB.depth_max << " mm]" << std::endl;
    std::cout << "  每点采集: " << cfg.captures_per_point << " 次, 共 "
              << (cfg.captures_per_point * 2) << " 次" << std::endl;
    std::cout << "  输出目录: " << cfg.save_dir << std::endl;
    std::cout << "==================================\n" << std::endl;

    auto t_start = std::chrono::steady_clock::now();

    for (int i = 0; i < cfg.captures_per_point; i++) {
        // Point A
        {
            if (!moveToPoint(ptA)) {
                std::cerr << "[DepthTester] 运动到 " << ptA.name << " 失败" << std::endl;
                continue;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(cfg.settle_time_ms));

            double depth = 0;
            if (!captureDepth(depth)) {
                std::cerr << "[DepthTester] " << ptA.name << " 采集失败" << std::endl;
                continue;
            }

            bool stale = histA.isStaleFrame(depth, ptA.depth_min, ptA.depth_max,
                                            ptB.depth_min, ptB.depth_max);
            if (stale) histA.stale_count++;
            histA.addValidDepth(depth, cfg.outlier_sigma);

            writeCsvLine(csv, frame_id++, ptA.name, depth, histA.getDefaultDepth(), stale);
        }

        // Point B
        {
            if (!moveToPoint(ptB)) {
                std::cerr << "[DepthTester] 运动到 " << ptB.name << " 失败" << std::endl;
                continue;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(cfg.settle_time_ms));

            double depth = 0;
            if (!captureDepth(depth)) {
                std::cerr << "[DepthTester] " << ptB.name << " 采集失败" << std::endl;
                continue;
            }

            bool stale = histB.isStaleFrame(depth, ptB.depth_min, ptB.depth_max,
                                            ptA.depth_min, ptA.depth_max);
            if (stale) histB.stale_count++;
            histB.addValidDepth(depth, cfg.outlier_sigma);

            writeCsvLine(csv, frame_id++, ptB.name, depth, histB.getDefaultDepth(), stale);
        }

        // 进度
        if ((i + 1) % 1000 == 0 || i == 0) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - t_start).count();
            int total = (i + 1) * 2;
            double rate = total / elapsed;
            int remaining = (cfg.captures_per_point - i - 1) * 2;
            double eta = remaining / rate;

            std::cout << "\r  进度: " << (i + 1) << "/" << cfg.captures_per_point
                      << " (" << (int)((i+1)*100.0/cfg.captures_per_point) << "%)"
                      << " | 采集 " << total << " 帧"
                      << " | 速率 " << (int)rate << " 帧/s"
                      << " | ETA " << (int)eta << "s  " << std::flush;
        }
    }

    csv.close();
    std::cout << "\n\n采集完成" << std::endl;

    // 报告
    auto t_end = std::chrono::steady_clock::now();
    double total_sec = std::chrono::duration<double>(t_end - t_start).count();
    std::cout << "\n总耗时: " << (int)(total_sec/60) << " 分 " << ((int)total_sec % 60) << " 秒" << std::endl;

    printReport(histA, histB);
    validateRemedy(csv_path);

    return 0;
}

void DepthTester::printReport(const DepthHistory& histA, const DepthHistory& histB) {
    const auto& ptA = m_config.pointA;
    const auto& ptB = m_config.pointB;

    int total = histA.total_count + histB.total_count;
    int stale_total = histA.stale_count + histB.stale_count;

    std::cout << "\n========== 深度测试报告 ==========" << std::endl;
    std::cout << "  每点目标采集数: " << m_config.captures_per_point << std::endl;
    std::cout << std::endl;

    auto print_point = [](const DepthTestPoint& pt, const DepthHistory& hist) {
        double pct = hist.total_count > 0 ? hist.stale_count * 100.0 / hist.total_count : 0;
        std::cout << "  " << pt.name << ":" << std::endl;
        std::cout << "    采集帧数: " << hist.total_count << std::endl;
        std::cout << "    陈旧帧:   " << hist.stale_count << " (" << std::fixed << std::setprecision(2) << pct << "%)" << std::endl;
        std::cout << "    默认深度: " << std::fixed << std::setprecision(2) << hist.default_depth << " mm" << std::endl;
    };

    print_point(ptA, histA);
    std::cout << std::endl;
    print_point(ptB, histB);

    double total_pct = total > 0 ? stale_total * 100.0 / total : 0;
    std::cout << "\n  综合陈旧概率: " << stale_total << "/" << total
              << " (" << std::fixed << std::setprecision(3) << total_pct << "%)" << std::endl;
    std::cout << "==================================\n" << std::endl;
}

void DepthTester::validateRemedy(const std::string& csv_path) {
    std::cout << "========== 补救验证 ==========" << std::endl;

    std::ifstream csv(csv_path);
    if (!csv.is_open()) {
        std::cerr << "  无法打开 CSV: " << csv_path << std::endl;
        return;
    }

    // 跳过表头
    std::string header;
    std::getline(csv, header);

    DepthHistory histA, histB;
    const auto& ptA = m_config.pointA;
    const auto& ptB = m_config.pointB;

    int remedy_total = 0, remedy_ok = 0;

    std::string line;
    while (std::getline(csv, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string field;
        std::vector<std::string> fields;
        while (std::getline(ss, field, ','))
            fields.push_back(field);

        if (fields.size() < 5) continue;

        std::string& point_name = fields[1];
        double raw_depth = std::stod(fields[2]);
        bool is_stale = (fields[4] == "true");

        DepthHistory* hist = nullptr;
        const DepthTestPoint* pt = nullptr;
        if (point_name == ptA.name) {
            hist = &histA; pt = &ptA;
        } else {
            hist = &histB; pt = &ptB;
        }

        if (is_stale) {
            remedy_total++;
            // 离线验证：使用历史默认深度补救陈旧帧。
            double remedy_val = hist->getDefaultDepth();
            // 如果默认深度在合理范围内 → 补救成功
            if (remedy_val >= pt->depth_min && remedy_val <= pt->depth_max)
                remedy_ok++;
        }

        hist->addValidDepth(raw_depth, m_config.outlier_sigma);
    }

    csv.close();

    double remedy_pct = remedy_total > 0 ? remedy_ok * 100.0 / remedy_total : 0;
    std::cout << "  陈旧帧总数: " << remedy_total << std::endl;
    std::cout << "  默认深度补救成功: " << remedy_ok << "/" << remedy_total
              << " (" << std::fixed << std::setprecision(2) << remedy_pct << "%)" << std::endl;

    // 保存补救报告
    std::string report_path = m_config.save_dir + "/remedy_report.yaml";
    cv::FileStorage rpt(report_path, cv::FileStorage::WRITE);
    if (rpt.isOpened()) {
        rpt << "remedy_report" << "{";
        rpt << "stale_total" << remedy_total;
        rpt << "remedy_success" << remedy_ok;
        rpt << "remedy_rate" << remedy_pct;
        rpt << "}";
        rpt.release();
    }
    std::cout << "  补救报告已保存: " << report_path << std::endl;
    std::cout << "================================\n" << std::endl;
}
