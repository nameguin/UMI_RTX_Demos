#ifndef __JOINT_H__
#define __JOINT_H__

#include <string>
#include <vector>
#include <map>
#include <iostream>

#include "umi_rtx_controller/arm_parts/forearm.h"

#include "umi_rtx_controller/umi-drivers/armraw.h"
#include "umi_rtx_controller/umi-drivers/armlib.h"
#include "umi_rtx_controller/umi-drivers/rtx.h"


using namespace std;


// To avoid cyclic dependency
class ForeArm;
class Joint {
public:
    Joint(int ID);
    Joint();

    void setChild(ForeArm* child);
    void setParent(ForeArm* parent);

    ForeArm* getChild();
    ForeArm* getParent();

    vector<float> getOrientation() const;
    vector<float> getPosition() const;

    void setOrientation(const float increment_angle);
    void setZed(const float zed);
    void setGrip(const float grip);

    const string getName() const;

    int m_ID;

    int get_parameter(int PID, int *value);

private:
    string m_Name;
    vector<int> m_parameters;
    vector<float> m_orientation, m_position;

    ForeArm *Child=nullptr, *Parent=nullptr;

};


#endif