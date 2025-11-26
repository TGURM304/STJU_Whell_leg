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
    p->leg_phi_l = left_leg_->my_leg_status_.phi0;
    p->leg_phi_r = right_leg_->my_leg_status_.phi0;
    p->real_phi = ins_->yaw/180.f*PI_F32;//由于phi存在突变点，我们使用delta的形式来实现
    p->phi = 0;
    p->dot_phi = ins_->raw.gyro[2];
    p->theta_b = ins_->roll/180.f*PI_F32;
    p->dot_theta_b = ins_->raw.gyro[0];

    p->old_S = p->S;
    p->S = (left_leg_->my_leg_status_.distance+right_leg_->my_leg_status_.distance)/2.f;
    p->dot_S = (left_leg_->my_leg_status_.ver+right_leg_->my_leg_status_.ver)/2;
    //todo:卡尔曼滤波器
    p->kalman_dot_S = p->dot_S;

    p->old_theta_ll = p->theta_ll, p->old_theta_lr = p->theta_lr;
    p->theta_ll = left_leg_->my_leg_status_.phi0 + p->theta_b - PI_F32/2;
    p->theta_lr = right_leg_->my_leg_status_.phi0 + p->theta_b - PI_F32/2;
    p->dot_theta_ll = left_filter_.update((p->theta_ll - p->old_theta_ll)*1000);
    p->dot_theta_lr = right_filter_.update((p->theta_lr - p->old_theta_lr)*1000);

    p->left_len = left_leg_->my_leg_status_.L0;
    p->right_len = right_leg_->my_leg_status_.L0;
}

void wheel_leg::SJTU_wheel_leg::wheel_leg_init() {
    left_leg_->leg_init();
    right_leg_->leg_init();
}

void wheel_leg::SJTU_wheel_leg::target_update(update_pkg *pkg) {
    //控制高度
    my_target_.height = pkg->height;
    //设定目标target
    if(abs(pkg->speed) > 0.2f) {
        my_target_.body_S = my_state_.S + pkg->speed/1000.f;
        my_target_.body_ver = pkg->speed;
    }
    else {
        my_target_.body_S += pkg->speed/1000.f;
        my_target_.body_ver = pkg->speed;
    }
    //设置目标的yaw，这个比较复杂
    my_target_.target_yaw += pkg->yaw_gro/1000.f;
    if(my_target_.target_yaw > PI_F32) my_target_.target_yaw -= 2*PI_F32;
    else if(my_target_.target_yaw < -PI_F32) my_target_.target_yaw += 2*PI_F32;
    my_target_.target_yaw_gro = pkg->yaw_gro/1000.f;
    //todo:roll PID
    my_target_.left_len = pkg->height;
    my_target_.right_len = pkg->height;
}

void wheel_leg::SJTU_wheel_leg::flag_update(update_pkg *pkg){
    //置标志位
    my_flags_.mf = pkg->c_flag_;
    my_flags_.lqr_flag = pkg->lqr_flag;
}

void wheel_leg::SJTU_wheel_leg::delta_clc(update_pkg *pkg){
    float32_t delta_yaw;
    delta_yaw = my_target_.target_yaw - my_state_.real_phi;
    if(delta_yaw > PI_F32)
        delta_yaw -= 2*PI_F32;
    else if(delta_yaw < -PI_F32)
        delta_yaw += 2*PI_F32;
    ary_delta[0] = my_target_.body_S - my_state_.S;
    ary_delta[1] = my_target_.body_ver - my_state_.kalman_dot_S ;
    ary_delta[2] = delta_yaw;
    ary_delta[3] = my_target_.target_yaw_gro - my_state_.dot_phi;
    ary_delta[4] = -my_state_.theta_ll;
    ary_delta[5] = -my_state_.dot_theta_ll;
    ary_delta[6] = -my_state_.theta_lr;
    ary_delta[7] = -my_state_.dot_theta_lr;
    ary_delta[8] = -my_state_.theta_b;
    ary_delta[9] = -my_state_.dot_theta_b;
}

//所有状态的更新都在这里实现
void wheel_leg::SJTU_wheel_leg::wheel_leg_update(update_pkg *pkg) {
    state_update();
    flag_update(pkg);
    target_update(pkg);
    delta_clc(pkg);
}

void wheel_leg::SJTU_wheel_leg::wheel_leg_ctrl() {
    LQR_clc();
    my_output_.Force_stand_l = FORWARD + left_pid_.update(my_state_.left_len,my_target_.left_len);
    my_output_.Force_stand_r = FORWARD + right_pid_.update(my_state_.right_len,my_target_.right_len);
    if(my_flags_.mf == E_stand) {
        left_leg_->leg_ctrl(my_output_.Tlw_l,my_output_.Tbl_l,my_output_.Force_stand_l);
        right_leg_->leg_ctrl(my_output_.Tlw_r,my_output_.Tbl_r,my_output_.Force_stand_r);
    }
    else if(my_flags_.mf == E_waiting) {
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
    auto state_flag = my_flags_.lqr_flag;
    Matrixf<10,1> Matrix_delta(ary_delta);
    Matrixf<4,1> Matrix_answer;
    if(state_flag == E_LQR_static) {
        Matrixf<4,10> Matrix_K(static_K_);
        Matrix_answer = Matrix_K*Matrix_delta;
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

