//
// Created by 15082 on 2025/11/7.
//

#ifndef APP_CHASSIS_WHELL_LEG_H
#define APP_CHASSIS_WHELL_LEG_H
#include "alg_filter.h"
#include "app_leg.h"
#include "app_ins.h"
#include "PID/ctrl_pid.h"

#define MID_AVG_SIZE 10

#define LEN_KP 500
#define LEN_KD 10
#define LEN_OUT_LIMIT 30
#define LEN_I_LIMIT 0
namespace wheel_leg {
struct chassis_state {
    float S, dot_S, phi, dot_phi;//轮两点连线中点的位移，yaw轴
    float theta_ll, dot_theta_ll, theta_lr, dot_theta_lr;//腿的左边倾角，腿的右边倾角
    float theta_b ,dot_theta_b;//车体的倾角
    float leg_phi_l, leg_phi_r;
    float old_theta_ll ,old_theta_lr;
    float32_t left_len, right_len;
};
struct chassis_output {
    float Tlw_l, Tlw_r;//轮子的扭矩
    float Tbl_l, Tbl_r;//车对腿的扭矩
    float Force_stand_l, Force_stand_r;
    float32_t Force_other_l, Force_other_r;
};
typedef enum {
    E_waiting,
    E_sit,
    E_stand
}chassis_flag ;
typedef enum {
    E_LQR_static,
    E_LQR_dynamic,
    E_disable
}LQR_flag;
struct chassis_target {
    float32_t left_len, right_len;
    float32_t left_dis, right_dis;
    float32_t left_ver, right_ver;
    float32_t body_S, body_ver;
    float32_t target_yaw;
    chassis_flag my_chassis_flag;
    LQR_flag my_LQR_flag;
};
struct mid_filter {
    float32_t dataBuf[MID_AVG_SIZE] = {0};
    uint8_t index = 0;
};
typedef enum {
    E_left,
    E_right
}side_select;
    class SJTU_wheel_leg {
    public:
        SJTU_wheel_leg();
        SJTU_wheel_leg(chassis_Leg::App_Leg *left_leg, chassis_Leg::App_Leg *right_leg,const app_ins_data_t *ins, float32_t *static_K)
            : left_leg_(left_leg), right_leg_(right_leg), ins_(ins), left_filter_(10),right_filter_(10)
            ,left_pid_(LEN_KP,0,LEN_KD,LEN_OUT_LIMIT,LEN_I_LIMIT), right_pid_(LEN_KP,0,LEN_KD,LEN_OUT_LIMIT,LEN_I_LIMIT)
        {
            memcpy(static_K_,static_K,sizeof(float32_t)*40);
        }
        void wheel_leg_init() ;
        void wheel_leg_update(float32_t height, float32_t yaw, float32_t speed, chassis_flag c_flag_, LQR_flag l_flag);
        void wheel_leg_ctrl();
    private:
        void LQR_clc();
        void state_update();
        //承接底层框架
        chassis_Leg::App_Leg *left_leg_;
        chassis_Leg::App_Leg *right_leg_;
        const app_ins_data_t *ins_;
        //底盘状态
        //todo:底盘状态观测器
        chassis_state my_state_;
        chassis_output my_output_;
        chassis_target my_target_;
        //用于计算Matrix
        float32_t ary_state[10] = {0};
        float32_t ary_target[10] = {0};
        //控制器与滤波器
        Controller::PID left_pid_;
        Controller::PID right_pid_;
        Algorithm::AverageFilter left_filter_;
        Algorithm::AverageFilter right_filter_;
        float32_t static_K_[40];

    };
}




#endif //APP_CHASSIS_WHELL_LEG_H
