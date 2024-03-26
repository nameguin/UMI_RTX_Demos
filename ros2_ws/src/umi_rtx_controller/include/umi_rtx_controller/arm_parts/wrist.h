#ifndef __WRIST_H__
#define __WRIST_H__

#include "umi_rtx_controller/arm_parts/joint.h"
#include "umi_rtx_controller/umi-drivers/rtx.h"

class Wrist {
public:
    Wrist(Joint *W1, Joint *W2);
    Wrist();

    void rotate(const float increment_angle); // Rotation on itself
    void turn(const float increment_angle); // Rotation on the arm

private:
    Joint *m_W1, *m_W2;
};

#endif