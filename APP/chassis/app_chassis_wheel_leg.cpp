//
// Created by 15082 on 2025/11/7.
//

#include "app_chassis_wheel_leg.h"

#include "alg_filter.h"
#include "bsp_uart.h"
#include "app_debug.h"

#define FORWARD (12)

void wheel_leg::SJTU_wheel_leg::state_update() {
    auto p = &my_state_;
    auto q = ary_state;
    p->leg_phi_l = left_leg_->my_leg_status_.phi0;
    p->leg_phi_r = right_leg_->my_leg_status_.phi0;
    p->real_phi = ins_->yaw/180.f*PI_F32;//由于phi存在突变点，我们使用delta的形式来实现
    p->phi = 0;
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

//所有状态的更新都在这里实现
void wheel_leg::SJTU_wheel_leg::wheel_leg_update(float32_t height, float32_t yaw_gro, float32_t speed, chassis_flag c_flag_, LQR_flag l_flag) {
    state_update();
    //置标志位
    my_target_.my_chassis_flag = c_flag_;
    my_target_.my_LQR_flag = l_flag;
    //控制高度
    my_target_.left_len = height;
    my_target_.right_len = height;
    //设定目标target
    my_target_.body_S += speed/1000;
    my_target_.body_ver = speed;
    //设置目标的yaw，这个比较复杂
    my_target_.real_target_yaw += yaw_gro/1000;
    if(my_target_.real_target_yaw > PI_F32) my_target_.real_target_yaw -= 2*PI_F32;
    else if(my_target_.real_target_yaw < -PI_F32) my_target_.real_target_yaw += 2*PI_F32;

    my_target_.delta_yaw = my_target_.real_target_yaw - my_state_.real_phi;
    if(my_target_.delta_yaw > PI_F32)
        my_target_.delta_yaw -= 2*PI_F32;
    else if(my_target_.delta_yaw < -PI_F32)
        my_target_.delta_yaw += 2*PI_F32;
    my_target_.target_yaw_gro = yaw_gro;
    //更新到数组
    ary_target[0] = my_target_.body_S;
    ary_target[1] = my_target_.body_ver;
    ary_target[2] = my_target_.delta_yaw;
    ary_target[3] = my_target_.target_yaw_gro;
    if(AppDebug::DEBUG_TYPE == AppDebug::CHASSIS_TARGET) {
        bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f\n",ary_target[0],ary_target[1],ary_target[2],ary_target[3]);
    }
}

void wheel_leg::SJTU_wheel_leg::wheel_leg_ctrl() {
    LQR_clc();
    my_output_.Force_stand_l = FORWARD + left_pid_.update(my_state_.left_len,my_target_.left_len);
    my_output_.Force_stand_r = FORWARD + right_pid_.update(my_state_.right_len,my_target_.right_len);
    if(my_target_.my_chassis_flag == E_stand) {
        left_leg_->leg_ctrl(my_output_.Tlw_l,my_output_.Tbl_l,my_output_.Force_stand_l);
        right_leg_->leg_ctrl(my_output_.Tlw_r,my_output_.Tbl_r,my_output_.Force_stand_r);
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
}

void wheel_leg::SJTU_wheel_leg::LQR_clc() {
    auto state_flag = my_target_.my_LQR_flag;
    Matrixf<10,1> Matrix_state(ary_state);
    Matrixf<10,1> Matrix_target(ary_target);
    Matrixf<4,1> Matrix_answer;
    if(state_flag == E_LQR_static) {
        Matrixf<4,10> Matrix_K(static_K_);
        Matrix_answer = Matrix_K*(Matrix_target - Matrix_state);
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

