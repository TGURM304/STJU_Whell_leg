//
// Created by 15082 on 2025/10/17.
//

#include "app_chassis_motor.h"
#include "app_datasheet.h"

void wheel_leg_motor::joint::init() {
    joint_->init();
    joint_->enable();
}
void wheel_leg_motor::joint::joint_deg_clc() {
    float temp_deg = this->joint_->status.pos;
    temp_deg -= zero_;
    temp_deg *= dir_;
    if(temp_deg < -PI_F32/2) temp_deg = 2 * PI_F32 - temp_deg;
    this->joint_deg_ = temp_deg;
}
void wheel_leg_motor::joint::joint_ctrl(float tor) {
    joint_deg_clc();
    joint_->control(0,0,0,0,tor*dir_);
}
void wheel_leg_motor::joint::joint_ctrl(float P, float I, float D, float out_limit, float I_limit, float target_pos) {
    joint_deg_clc();
    float32_t temp_deg;
    temp_deg = this->joint_deg_;
    float32_t err = target_pos - temp_deg;
    out_p_ = err*P;
    out_i_+= err*I;
    out_d_ = (err - old_err_)*D;
    old_err_ = err;
    old_pos_ = target_pos;
    out_i_ = out_i_ > I_limit? I_limit: out_i_< -I_limit? -I_limit:out_i_;
    out_sum_ = out_p_ + out_i_ + out_d_;
    out_sum_ = out_sum_ > out_limit? out_limit : out_sum_<-out_limit? -out_limit:out_sum_;
    joint_->control(0,0,0,0,out_sum_*dir_);
}
/*
 * 铁损阻力：72rpm/Nm
 * 减速比：3591/187
 * 扭矩常数：0.3Nm/A
 * 机械转角：0~8192
 * 转子转速单位：RPM
 * 电流： -20A~20A  -16384~16384
 */
#define ENCODER_RANGE 8192
#define CURRENT_RANGE 16384
#define CURRENT_CAST(current) (current/20*CURRENT_RANGE)
#define TOR_CAST(tor) CURRENT_CAST(tor*MOTOR_GEAR/((3591.0/187.0)* 0.3))

wheel_leg_motor::dynamic::dynamic(Motor::DJIMotor *dynamic_motor, float32_t dir)
:dynamic_motor_(dynamic_motor)
, dir_(dir) { }
void wheel_leg_motor::dynamic::init() {
    this->dynamic_motor_->init();
}
void wheel_leg_motor::dynamic::motor_deg_clc() {
    this->single_deg = this->dynamic_motor_->status.angle*PI_F32*2*dir_/ENCODER_RANGE;
    (single_deg -old_deg)>PI_F32?round_cnt-=1:(single_deg - old_deg) < -PI_F32?round_cnt+=1:0;
    total_deg = round_cnt*PI_F32*2 + single_deg;
    old_deg = single_deg;
}
void wheel_leg_motor::dynamic::tor_ctrl(float32_t tor) {
    motor_deg_clc();
    int16_t ctrl_current = (int16_t)TOR_CAST(tor*dir_);
    this->dynamic_motor_->update(ctrl_current);
}
float32_t wheel_leg_motor::dynamic::get_deg() {
    return this->total_deg;
}
float32_t wheel_leg_motor::dynamic::get_v(){
    return dir_*dynamic_motor_->status.speed*2*PI_F32/60.0f;
}
