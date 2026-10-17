#include "sampling_utils.hpp"
#include "robot_util.hpp"

#include <iostream>

namespace sampling {

CRresult moveToFlangeAxisPos(RobotHandle robotHandle, double axisAnglePos[6]) {
    double ref_j[ROB_AXIS_NUM], joint[ROB_AXIS_NUM];
    if (cr_get_jointActualPos(robotHandle, ref_j) != sucess)
        return error;
    if (cr_kineInverse(robotHandle, axisAnglePos, ref_j, joint) != sucess)
        return error;
    return moveToJointPos(robotHandle, joint);
}

void printProgress(int current, int total) {
    int bar_width = 50;
    int filled = current * bar_width / total;
    int pct = current * 100 / total;
    std::cout << "\r[";
    for (int i = 0; i < bar_width; i++)
        std::cout << (i < filled ? '#' : (i == filled ? '>' : ' '));
    std::cout << "] " << pct << "% (" << current << "/" << total << ")" << std::flush;
}

} // namespace sampling
