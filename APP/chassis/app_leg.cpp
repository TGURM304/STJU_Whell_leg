//
// Created by 15082 on 2025/11/6.
//

#define LEN_KP 10
#define LEN_KD 2

#include "app_leg.h"
void chassis_Leg::App_Leg::leg_init() {
    joint_A_->init();
    joint_E_->init();
    dynamic_motor_->init();
}

void chassis_Leg::App_Leg::leg_ctrl(float32_t Tlw, float32_t Tbl, float32_t force) {
    my_out_put_.Tbl = Tbl;
    my_out_put_.force = force;
    my_out_put_.tor_dynamic = Tlw;
    leg_status_clc();
    leg_vmc_ctrl();

    joint_A_->joint_ctrl(my_out_put_.tor_JA);
    joint_E_->joint_ctrl(my_out_put_.tor_JE);
    dynamic_motor_->tor_ctrl(my_out_put_.tor_dynamic);
}

void chassis_Leg::App_Leg::leg_vmc_ctrl() {
    float32_t data[4];
    data[0] = my_leg_status_.l1*sin(my_leg_status_.phi1-my_leg_status_.phi2)*sin(my_leg_status_.phi3)/sin(my_leg_status_.phi2 - my_leg_status_.phi3);
    data[1] = my_leg_status_.l4*sin(my_leg_status_.phi3 - my_leg_status_.phi4)*sin(my_leg_status_.phi2)/sin(my_leg_status_.phi2-my_leg_status_.phi3);
    data[2] = -my_leg_status_.l1*sin(my_leg_status_.phi1-my_leg_status_.phi2)*cos(my_leg_status_.phi3)/sin(my_leg_status_.phi2-my_leg_status_.phi3);
    data[3] = -my_leg_status_.l4*sin(my_leg_status_.phi3 - my_leg_status_.phi4)*cos(my_leg_status_.phi2)/sin(my_leg_status_.phi2-my_leg_status_.phi3);
    Matrixf<2,2> Jacobi(data);
    Matrixf<2,2> Jacobi_T = Jacobi.trans();
    float32_t vector[2] = {my_out_put_.force,my_out_put_.Tbl};
    Matrixf<2,1> target(vector);
    data[0] = 0;
    data[1] = -1/my_leg_status_.L0;
    data[2] = 1;
    data[3] = 0;
    Matrixf<2,2> M(data);
    data[0] = cos(-my_leg_status_.phi0+PI/2);
    data[1] = -sin(-my_leg_status_.phi0+PI/2);
    data[2] = sin(-my_leg_status_.phi0+PI/2);
    data[3] = cos(-my_leg_status_.phi0+PI/2);
    Matrixf<2,2> R(data);
    Matrixf<2,1> answer;
    answer = Jacobi_T*R*M*target;
    my_out_put_.tor_JA = answer[0][0];
    my_out_put_.tor_JE = answer[1][0];
}
void chassis_Leg::App_Leg::leg_status_clc() {
    my_leg_status_.phi1 = joint_A_->joint_deg_;
    my_leg_status_.phi4 = joint_E_->joint_deg_;
    my_leg_status_.xb = my_leg_status_.l1*cos(my_leg_status_.phi1), my_leg_status_.yb = my_leg_status_.l1*sin(my_leg_status_.phi1);
    my_leg_status_.xd = my_leg_status_.l_AE+my_leg_status_.l4 *cos(my_leg_status_.phi4), my_leg_status_.yd = my_leg_status_.l4*sin(my_leg_status_.phi4);
    const float A0 = 2*my_leg_status_.l2 *(my_leg_status_.xb-my_leg_status_.xd), B0 = 2*LEG_L2*(my_leg_status_.yb - my_leg_status_.yd),
        C0 = powf(my_leg_status_.xb-my_leg_status_.xd, 2) + powf(my_leg_status_.yb - my_leg_status_.yd, 2) - powf(LEG_L3,2)+powf(LEG_L2,2);
    const float32_t phi3_temp_x = (my_leg_status_.xb - my_leg_status_.xd) + my_leg_status_.l2*cos(my_leg_status_.phi2),
        phi3_temp_y = (my_leg_status_.yb - my_leg_status_.yd) + my_leg_status_.l2 * sin(my_leg_status_.phi2);
    my_leg_status_.phi3 = atan2(phi3_temp_y, phi3_temp_x);
    const float tempy =-2*B0 + sqrt(4*B0*B0-4*(C0*C0-A0*A0)), temp_x = 2*(C0-A0);
    my_leg_status_.phi2 = 2*atan2(tempy, temp_x);
    //计算正解算获取腿长与phi0
    my_leg_status_.leg_x = -my_leg_status_.l_AE/2+my_leg_status_.l1*cos(my_leg_status_.phi1)+my_leg_status_.l2*cos(my_leg_status_.phi2);
    my_leg_status_.leg_y = my_leg_status_.l1*sin(my_leg_status_.phi1) + my_leg_status_.l2*sin(my_leg_status_.phi2);
    my_leg_status_.L0 = sqrt(my_leg_status_.leg_x*my_leg_status_.leg_x + my_leg_status_.leg_y*my_leg_status_.leg_y);
    my_leg_status_.phi0 =atan2(my_leg_status_.leg_y, my_leg_status_.leg_x);
}
