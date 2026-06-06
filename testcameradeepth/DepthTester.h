#ifndef DEPTH_TESTER_HPP
#define DEPTH_TESTER_HPP

#include <memory>
#include <fstream>
#include "cameras/AICamera.hpp"
#include "DepthTestConfig.h"
#include "DepthRemedy.h"

class DepthTester {
public:
    DepthTester(std::shared_ptr<AICamera>& camera, int robot_handle,
                const DepthTestConfig& config);

    // 执行测试采集，返回 0=成功
    int run();

private:
    bool moveToPoint(const DepthTestPoint& pt);
    bool captureDepth(double& out_depth);
    void writeCsvHeader(std::ofstream& csv);
    void writeCsvLine(std::ofstream& csv, int frame_id, const std::string& point_name,
                      double raw_depth, double default_depth, bool is_stale);
    void printReport(const DepthHistory& histA, const DepthHistory& histB);
    void validateRemedy(const std::string& csv_path);

    std::shared_ptr<AICamera>& m_camera;
    int m_robot_handle;
    DepthTestConfig m_config;
};

#endif // DEPTH_TESTER_HPP
