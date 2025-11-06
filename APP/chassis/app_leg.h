//
// Created by 15082 on 2025/11/6.
//

#ifndef APP_LEG_H
#define APP_LEG_H

#include "app_chassis_motor.h"
#include "app_datasheet.h"
#include "SJTU_Matrix/matrix.h"
struct leg_status {
    float32_t l_AE, l1, l2,l3,l4;
    float32_t phi0,phi1,phi2,phi3,phi4;
    float32_t L0;
    float32_t leg_x, leg_y;
    float32_t xd, yd, xb, yb;
};
struct out_put {
    float32_t tor_JE, tor_JA;
    float32_t tor_dynamic;
    float32_t Tbl, force;
};

namespace chassis_Leg {
class App_Leg {
public:
    App_Leg();
    App_Leg(wheel_leg_motor::joint *joint_A, wheel_leg_motor::joint *joint_E, wheel_leg_motor::dynamic *dynamic_motor):joint_A_(joint_A), joint_E_(joint_E), dynamic_motor_(dynamic_motor) {
        my_leg_status_.l_AE = LEG_AE;
        my_leg_status_.l1 = my_leg_status_.l4 = LEG_L1;
        my_leg_status_.l2 = my_leg_status_.l3 = LEG_L2;
    }
    void leg_ctrl(float32_t Tlw, float32_t Tbl, float32_t force); // Tlw为腿对轮输出扭矩， Tbl对躯干对腿输出扭矩
    void leg_init();
    leg_status my_leg_status_;
    wheel_leg_motor::joint *joint_A_, *joint_E_;
    wheel_leg_motor::dynamic * dynamic_motor_;
    out_put my_out_put_;
private:
    void leg_vmc_ctrl(); //计算VMC
    void leg_status_clc();

};
}

#endif //APP_LEG_H
