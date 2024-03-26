#ifndef __ARM_H__
#define __ARM_H__

#include <vector>
#include <map>
#include <iostream>

#include "umi_rtx_controller/arm_parts/joint.h"
#include "umi_rtx_controller/arm_parts/forearm.h"
#include "umi_rtx_controller/arm_parts/wrist.h"
#include "umi_rtx_controller/umi-drivers/rtx.h"

using namespace std;


class Arm {
public:
    Arm();

    void addJoint(Joint *joint);
    Joint* getJoint(int index) const;

    void initArm();

    vector<int> getMotorState(int ID);
    vector<Joint*> mJoints;

private:
    string mName;

    ForeArm FA1,FA2,FA3;
    Wrist W;
};

#endif