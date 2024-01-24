#ifndef ROS2_CONTROL_EDGEBOTIC_AMR__WHEEL_H_
#define ROS2_CONTROL_EDGEBOTIC_AMR__WHEEL_H_

#include <string>

class Wheel
{
public:
    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;
    double rads_per_count = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int counts_per_rev);
    
    void setup(const std::string &wheel_name, int counts_per_rev);

    double calcEncAngle();
};

#endif // ROS2_CONTROL_EDGEBOTIC_AMR__WHEEL_H_