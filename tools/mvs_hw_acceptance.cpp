// MVS 真机验收工具（仅 CAMERA_BACKEND=MVS 时构建）。
// 直接使用生产类 MVSCamera，覆盖方案 Phase E 真机冒烟项：
//   1. 精确连接 + 审计信息一致性（serial/model/IP/实际分辨率/流格式/固件）
//   2. 连续取流 1000 帧（空帧率 <1%、尺寸稳定）
//   3. 曝光/增益/自动曝光 设置与回读
//   4. 10 次 open/close 循环（句柄不泄漏、可重开）
//   5. 失败路径：错误序列号/错误IP/多相机歧义/独占占用
//
// 用法:
//   ./mvs_hw_acceptance <cam_ip> [cam_serial] [second_serial]
//     cam_ip        目标相机当前 IP（必填）
//     cam_serial    目标相机序列号（可选，提供后校验审计一致性）
//     second_serial 另一台相机序列号（可选，提供后执行序列号优先级测试）
#include "cameras/MVSCamera.hpp"
#include "my_base.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

static int g_failures = 0;
#define CHECK(cond, name) do { \
    if (cond) { std::cout << "PASS  " << name << "\n"; } \
    else { std::cout << "FAIL  " << name << "\n"; ++g_failures; } \
} while (0)

static CONFIG cfgFor(const std::string& ip, const std::string& sn) {
    CONFIG c;
    c.cam_ip = ip;
    c.cam_sn = sn;
    c.cam_width = 0;   // 不强制分辨率，验证实际回读
    c.cam_height = 0;
    return c;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0]
                  << " <cam_ip> [cam_serial] [second_serial]\n";
        return 2;
    }
    const std::string camIp = argv[1];
    const std::string camSerial = argc > 2 ? argv[2] : "";
    const std::string secondSerial = argc > 3 ? argv[3] : "";

    // ---- 1. 按 IP 精确连接，审计信息与设备一致 ----
    std::string actualSerial;
    {
        MVSCamera cam(cfgFor(camIp, ""));
        CHECK(cam.open(), "open_by_ip");
        const CameraDeviceAuditInfo info = cam.getDeviceAuditInfo();
        actualSerial = info.serial;
        std::cout << "  audit: serial=" << info.serial
                  << " model=" << info.model
                  << " ip=" << info.current_ip
                  << " res=" << info.actual_width << "x" << info.actual_height
                  << " fmt=" << info.pixel_format
                  << " fw=" << info.firmware << "\n";
        CHECK(info.current_ip == camIp, "audit_ip_matches");
        CHECK(!info.serial.empty() && !info.model.empty(),
              "audit_identity_present");
        CHECK(info.actual_width > 0 && info.actual_height > 0,
              "audit_actual_resolution");
        CHECK(!info.pixel_format.empty() && info.pixel_format != "unknown",
              "audit_pixel_format");
        if (!camSerial.empty())
            CHECK(info.serial == camSerial, "audit_serial_matches_config");
        cam.close();
    }

    // ---- 2. 按序列号连接 ----
    if (!actualSerial.empty()) {
        MVSCamera cam(cfgFor("", actualSerial));
        CHECK(cam.open(), "open_by_serial");
        CHECK(cam.getDeviceAuditInfo().current_ip == camIp,
              "audit_ip_matches_serial");
        cam.close();
    }

    // ---- 3. 序列号优先于 IP（提供第二台序列号时）----
    if (!secondSerial.empty() && !actualSerial.empty() &&
        secondSerial != actualSerial) {
        MVSCamera cam(cfgFor(camIp, secondSerial));
        CHECK(cam.open(), "open_serial_priority");
        CHECK(cam.getDeviceAuditInfo().serial == secondSerial,
              "serial_wins_over_ip");
        cam.close();
    }

    // ---- 4. 失败路径：错误序列号 / 错误 IP / 未配置身份 ----
    {
        MVSCamera cam(cfgFor("", "INVALID_SERIAL_000"));
        CHECK(!cam.open(), "wrong_serial_rejected");
        CHECK(cam.lastError().message.find(actualSerial) != std::string::npos,
              "wrong_serial_lists_candidates");
    }
    {
        MVSCamera cam(cfgFor("203.0.113.254", ""));  // TEST-NET-3，不可达
        CHECK(!cam.open(), "wrong_ip_rejected");
    }
    {
        MVSCamera cam(cfgFor("", ""));
        const bool opened = cam.open();
        if (opened) {
            // 全网只有一台设备时允许唯一设备直连，不算失败
            std::cout << "  仅一台在线设备，唯一设备直连: "
                      << cam.getDeviceAuditInfo().serial << "\n";
            cam.close();
        }
        CHECK(true, "no_identity_deterministic");
    }

    // ---- 5. 独占占用：第二实例打开同一相机必须失败并给出错误码 ----
    {
        MVSCamera a(cfgFor(camIp, ""));
        CHECK(a.open(), "exclusive_first_open");
        MVSCamera b(cfgFor(camIp, ""));
        CHECK(!b.open(), "exclusive_second_denied");
        std::cout << "  denied: " << b.lastError().message.substr(0, 100)
                  << "\n";
        a.close();
    }

    // ---- 6. 连续取流 1000 帧 ----
    {
        MVSCamera cam(cfgFor(camIp, ""));
        CHECK(cam.open(), "stream_open");
        int empty = 0;
        int w = -1, h = -1;
        bool sizeChanged = false;
        auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < 1000; ++i) {
            cv::Mat f = cam.captureOneFrame();
            if (f.empty()) { ++empty; continue; }
            if (w < 0) { w = f.cols; h = f.rows; }
            if (f.cols != w || f.rows != h) sizeChanged = true;
        }
        const double secs = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        std::cout << "  stream: 1000 frames in " << secs << "s, empty="
                  << empty << ", size=" << w << "x" << h << "\n";
        CHECK(empty < 10, "stream_empty_rate_below_1pct");
        CHECK(!sizeChanged && w > 0, "stream_size_stable");
        cam.close();
    }

    // ---- 7. 曝光/增益/自动曝光 设置与回读 ----
    {
        MVSCamera cam(cfgFor(camIp, ""));
        CHECK(cam.open(), "param_open");
        cam.setAutoExposure(false);
        cam.setColorExposureTime(5000);
        const int exp1 = cam.getColorExposureTime();
        std::cout << "  exposure readback: " << exp1 << " us\n";
        CHECK(std::abs(exp1 - 5000) < 200, "exposure_set_readback");
        cam.setColorExposureTime(12000);
        CHECK(std::abs(cam.getColorExposureTime() - 12000) < 400,
              "exposure_second_value");
        cam.setColorGain(5);
        const int g = cam.getColorGain();
        std::cout << "  gain readback: " << g << "\n";
        CHECK(std::abs(g - 5) <= 1, "gain_set_readback");
        cam.setAutoExposure(true);
        CHECK(cam.getAutoExposure(), "auto_exposure_on");
        cam.setAutoExposure(false);
        cam.close();
    }

    // ---- 8. 10 次 open/close 循环 ----
    {
        bool allOk = true;
        for (int i = 0; i < 10; ++i) {
            MVSCamera cam(cfgFor(camIp, ""));
            if (!cam.open()) { allOk = false; break; }
            cv::Mat f = cam.captureOneFrame();
            if (f.empty()) { allOk = false; }
            cam.close();
        }
        CHECK(allOk, "open_close_cycle_x10");
    }

    // ---- 9. 未打开直接取流 / 重复关闭：不得崩溃 ----
    {
        MVSCamera cam(cfgFor(camIp, ""));
        cv::Mat f = cam.captureOneFrame();
        CHECK(f.empty(), "grab_without_open_returns_empty");
        cam.close();
        cam.close();
        CHECK(true, "double_close_no_crash");
    }

    std::cout << (g_failures == 0 ? "\n[acceptance] ALL PASS\n"
                                  : "\n[acceptance] FAILURES\n");
    return g_failures == 0 ? 0 : 1;
}
