//
// Created by 15082 on 2025/11/7.
//

#include "app_chassis_wheel_leg.h"

#include "alg_filter.h"
#include "bsp_uart.h"
#include "app_debug.h"

#define FORWARD (-12)

void wheel_leg::SJTU_wheel_leg::state_update() {
    auto p = &my_state_;
    auto q = ary_state;
    p->leg_phi_l = left_leg_->my_leg_status_.phi0;
    p->leg_phi_r = right_leg_->my_leg_status_.phi0;
    p->phi = ins_->yaw/180.f*PI_F32;
    p->dot_phi = ins_->raw.gyro[2];
    p->theta_b = ins_->roll/180.f*PI_F32;
    p->dot_theta_b = ins_->raw.gyro[0];


    p->S = (left_leg_->my_leg_status_.distance+right_leg_->my_leg_status_.distance)/2.f;
    p->dot_S = (left_leg_->my_leg_status_.ver+right_leg_->my_leg_status_.ver)/2;

    p->old_theta_ll = p->theta_ll, p->old_theta_lr = p->theta_lr;
    p->theta_ll = left_leg_->my_leg_status_.phi0 + p->theta_b - PI_F32/2;
    p->theta_lr = right_leg_->my_leg_status_.phi0 + p->theta_b - PI_F32/2;
    p->dot_theta_ll = left_filter_.update((p->theta_ll - p->old_theta_ll)*1000);
    p->dot_theta_lr = right_filter_.update((p->theta_lr - p->old_theta_lr)*1000);

    p->left_len = left_leg_->my_leg_status_.L0;
    p->right_len = right_leg_->my_leg_status_.L0;
    //同时更新到状态数组中
    q[0] = p->S;
    q[1] = p->dot_S;
    q[2] = p->phi;
    q[3] = p->dot_phi;
    q[4] = p->theta_ll;
    q[5] = p->dot_theta_ll;
    q[6] = p->theta_lr;
    q[7] = p->dot_theta_lr;
    q[8] = p->theta_b;
    q[9] = p->dot_theta_b;
    if(AppDebug::DEBUG_TYPE == AppDebug::CHASSIS_STATE)
        bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7],q[8],q[9]);
}

void wheel_leg::SJTU_wheel_leg::wheel_leg_init() {
    left_leg_->leg_init();
    right_leg_->leg_init();
}

void wheel_leg::SJTU_wheel_leg::wheel_leg_update(float32_t height, float32_t yaw, float32_t speed, chassis_flag c_flag_, LQR_flag l_flag) {
    my_target_.left_len = height;
    my_target_.right_len = height;
    my_target_.my_chassis_flag = c_flag_;
    my_target_.my_LQR_flag = l_flag;
}

void wheel_leg::SJTU_wheel_leg::wheel_leg_ctrl() {
    state_update();
    LQR_clc();
    my_output_.Force_stand_l = FORWARD + left_pid_.update(my_state_.left_len,my_target_.left_len);
    my_output_.Force_stand_r = FORWARD + right_pid_.update(my_state_.right_len,my_target_.right_len);

    if(my_target_.my_chassis_flag == E_stand) {
        left_leg_->leg_ctrl(my_output_.Tlw_l,my_output_.Tbl_l,my_output_.Force_stand_l);
        right_leg_->leg_ctrl(my_output_.Tlw_r,my_output_.Tbl_r,my_output_.Force_stand_r);
        // left_leg_->leg_ctrl(0,0,0);
        // right_leg_->leg_ctrl(0,0,0);
    }
    else if(my_target_.my_chassis_flag == E_waiting) {
        left_leg_->leg_ctrl(0,0,0);
        right_leg_->leg_ctrl(0,0,0);
    }
    //DEBUG,通过修改定义来确定输出
    if(AppDebug::DEBUG_TYPE == AppDebug::CHASSIS_OUTPUT) {
        auto p = my_output_;
        bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f\n",p.Tlw_l,p.Tlw_r,p.Tbl_l,p.Tbl_r);
    }

    // left_leg_->leg_ctrl(my_output_.Tlw_l,my_output_.Tbl_l,my_output_.Force_stand_l+my_output_.Force_other_l);
    // right_leg_->leg_ctrl(my_output_.Tlw_r,my_output_.Tbl_r,my_output_.Force_stand_r+my_output_.Force_other_r);
}

void wheel_leg::SJTU_wheel_leg::LQR_clc() {
    auto state_flag = my_target_.my_LQR_flag;
    Matrixf<10,1> Matrix_state(ary_state);
    Matrixf<10,1> Matrix_target(ary_target);
    Matrixf<4,1> Matrix_answer;
    if(state_flag == E_LQR_static) {
        Matrixf<4,10> Matrix_K(static_K_);
        Matrix_answer = (-1)*Matrix_K*Matrix_state;
        my_output_.Tlw_l = Matrix_answer[0][0];
        my_output_.Tlw_r = Matrix_answer[1][0];
        my_output_.Tbl_l = Matrix_answer[2][0];
        my_output_.Tbl_r = Matrix_answer[3][0];
    }
    else if(state_flag == E_LQR_dynamic) {
        //todo:完成动态矩阵
        return;
    }
}

