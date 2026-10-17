#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>

#include "robotapi.h"
#include "basestruct.h"

static int readCSV(const std::string& path, std::vector<std::vector<double>>& data) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "无法打开: " << path << std::endl;
        return -1;
    }
    std::string line;
    int line_no = 0;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::vector<double> row;
        std::string val;
        for (int i = 0; i < 6; i++) {
            if (!std::getline(ss, val, ',')) break;
            row.push_back(std::stod(val));
        }
        if (row.size() == 6) {
            data.push_back(row);
            line_no++;
        }
    }
    return (int)data.size();
}

int main(int argc, char** argv) {
    std::string dir = "/home/irobotic/work/caitu/build/param/eye_to_hand/";
    if (argc >= 2) 
         dir = argv[1];

    std::string joint_file = dir + "/rob_joint.csv";
    std::string pos_file   = dir + "/rob_pos.csv";

    std::vector<std::vector<double>> joints, poses;
    int nj = readCSV(joint_file, joints);
    int np = readCSV(pos_file, poses);
    if (nj <= 0 || np <= 0) return 1;
    if (nj != np) {
        std::cerr << "行数不匹配: joints=" << nj << " pos=" << np << std::endl;
        return 1;
    }
    std::cout << "读取 " << nj << " 组数据" << std::endl;

    // 轴角 → 欧拉，保存 rob_eule.csv
    {
        std::string eule_file = dir + "/rob_eule.csv";
        std::ofstream f(eule_file);
        if (!f.is_open()) {
            std::cerr << "无法创建: " << eule_file << std::endl;
        } else {
            f << std::fixed << std::setprecision(3);
            int converted = 0;
            for (auto& p : poses) {
                double eule[ROB_AXIS_NUM];
                if (cr_AxisAngle2Eule(p.data(), eule) == sucess) {
                    for (int j = 0; j < 6; j++)
                        f << eule[j] << (j < 5 ? "," : "\n");
                    converted++;
                }
            }
            std::cout << "轴角→欧拉转换完成: " << converted << "/" << poses.size()
                      << " 帧 -> " << eule_file << std::endl;
        }
    }

    // 连接机器人
    RobotHandle robotHandle;
    const char* ip = "192.168.1.6";
    CRresult ret = cr_create_robot(&robotHandle, ip, 2323, "123");
    if (ret != sucess) {
        std::cerr << "连接机器人失败 " << ip << std::endl;
        return 1;
    }
    std::cout << "机器人连接成功" << std::endl;

    // 逐帧 FK 验证
    const double POS_TOL_MM = 0.2;   // 平移容许偏差 mm
    const double ANG_TOL_DEG = 0.05;   // 旋转容许偏差 deg
    int bad_count = 0;
    double max_pos_err = 0, max_ang_err = 0;
    double sum_pos_err = 0, sum_ang_err = 0;

    for (int i = 0; i < nj; i++) {
        double fk_out[6];

        ret = cr_kineForward(robotHandle, (double*)joints[i].data(), fk_out);
        if (ret != sucess) {
            std::cerr << "FK 失败 at line " << i << std::endl;
            bad_count++;
            continue;
        }

        // 比较 FK 输出与 CSV 中的法兰轴角
        double pos_err = std::sqrt(
            std::pow(fk_out[0] - poses[i][0], 2) +
            std::pow(fk_out[1] - poses[i][1], 2) +
            std::pow(fk_out[2] - poses[i][2], 2));

        // 轴角旋转误差：用 Rodrigues 转成矩阵后计算角度差
        // 简化：直接比较轴角向量模长差异（不严谨但够用）
        double ang_err = std::sqrt(
            std::pow(fk_out[3] - poses[i][3], 2) +
            std::pow(fk_out[4] - poses[i][4], 2) +
            std::pow(fk_out[5] - poses[i][5], 2));

        sum_pos_err += pos_err;
        sum_ang_err += ang_err;
        max_pos_err = std::max(max_pos_err, pos_err);
        max_ang_err = std::max(max_ang_err, ang_err);

        if (pos_err > POS_TOL_MM || ang_err > ANG_TOL_DEG) {
            if (bad_count < 10) {
                std::cout << "Line " << i << ": pos_err=" << pos_err
                          << "mm  ang_err=" << ang_err << "deg" << std::endl;
                std::cout << "  FK out:  " << fk_out[0] << "," << fk_out[1] << "," << fk_out[2]
                          << "," << fk_out[3] << "," << fk_out[4] << "," << fk_out[5] << std::endl;
                std::cout << "  CSV pos: " << poses[i][0] << "," << poses[i][1] << "," << poses[i][2]
                          << "," << poses[i][3] << "," << poses[i][4] << "," << poses[i][5] << std::endl;
            }
            bad_count++;
        }
    }

    cr_destroy_robot(robotHandle);

    double avg_pos = sum_pos_err / nj;
    double avg_ang = sum_ang_err / nj;

    std::cout << "\n========== FK 验证结果 ==========" << std::endl;
    std::cout << "总帧数: " << nj << std::endl;
    std::cout << "超出阈值 (pos>" << POS_TOL_MM << "mm || ang>" << ANG_TOL_DEG << "deg): " << bad_count << std::endl;
    std::cout << "平移误差: avg=" << avg_pos << "mm  max=" << max_pos_err << "mm" << std::endl;
    std::cout << "旋转误差: avg=" << avg_ang << "deg  max=" << max_ang_err << "deg" << std::endl;

    if (bad_count == 0 && avg_pos < 1.0 && avg_ang < 0.5) {
        std::cout << "\n结论: rob_joint 与 rob_pos 一一对应 ✓" << std::endl;
    } else if (bad_count < nj * 0.1) {
        std::cout << "\n结论: 大部分对应正确，少量偏差（可能因关节回差或保存时序）" << std::endl;
    } else {
        std::cout << "\n结论: 大量不匹配，CSV 数据可能有错！" << std::endl;
    }

    return 0;
}
