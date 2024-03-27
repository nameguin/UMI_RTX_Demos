#ifndef __FOREARM_H__
#define __FOREARM_H__

#include <vector>
#include "umi_rtx_controller/arm_parts/joint.h"

// To avoid cyclic dependency
class Joint;

class ForeArm {
public:
    ForeArm(int id, Joint *parent_joint, Joint *child_joint);
    ForeArm();
    
    const float m_length = 0.252;
    int m_ID;

    Joint* get_parentJoint() const;
    Joint* get_childJoint() const;

    void set_parentJoint(Joint *parent);
    void set_childJoint(Joint *child);


private:

    Joint *m_parent_joint=nullptr, *m_child_joint=nullptr;
    
};

#endif