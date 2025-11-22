//
// Created by fish on 2024/12/18.
//

#include "app_chassis.h"

#include "app_sys.h"
#include "sys_task.h"
#include "app_chassis_motor.h"
#include "app_chassis_wheel_leg.h"
#include "app_datasheet.h"
#include "app_ins.h"
#include "app_leg.h"
#include "bsp_rc.h"
#include "bsp_uart.h"
#include "dev_motor_dm.h"
#include "dev_motor_dji.h"
#ifdef COMPILE_CHASSIS

/*
 *         ⬆ X正（向上）
 *         |
 *         |
 *   J4|-------|J1
 *     |   DM  |
 *  M2 |       |  M1
 *     |  分 分 |
 *   J3|_______|J2
 *
 */

// 静态任务，在 CubeMX 中配置

Motor::DMMotor motor1("joint1",Motor::DMMotor::J4310,{
        .slave_id = 0x21,
        .master_id = 0x11,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    });
Motor::DMMotor motor2("joint2",Motor::DMMotor::J4310,{
        .slave_id = 0x22,
        .master_id = 0x12,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    });
Motor::DMMotor motor3("joint3",Motor::DMMotor::J4310,{
        .slave_id = 0x23,
        .master_id = 0x13,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    });
Motor::DMMotor motor4("joint4",Motor::DMMotor::J4310,{
        .slave_id = 0x24,
        .master_id = 0x14,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    });



Motor::DJIMotor left_dji("left",Motor::DJIMotor::M3508,{.id = 2, .port = E_CAN1, .mode = Motor::DJIMotor::CURRENT});
Motor::DJIMotor right_dji("right",Motor::DJIMotor::M3508,{.id = 1, .port = E_CAN1, .mode = Motor::DJIMotor::CURRENT});


wheel_leg_motor::joint right_E(&motor1, PI_F32/2, -1);
wheel_leg_motor::joint right_A(&motor2, PI_F32/2, -1);
wheel_leg_motor::joint left_E(&motor4, -PI_F32/2, 1);
wheel_leg_motor::joint left_A(&motor3, -PI_F32/2, 1);
wheel_leg_motor::dynamic right_dynamic(&right_dji, -1);
wheel_leg_motor::dynamic left_dynamic(&left_dji, 1);

chassis_Leg::App_Leg right_leg(&right_A, &right_E, &right_dynamic);
chassis_Leg::App_Leg left_leg(&left_A, &left_E, &left_dynamic);
const app_ins_data_t *ins = app_ins_data();
float static_K[40] = {-3.16021f, -2.9304f, -0.164658f, -0.222398f, -4.51869f, -1.12821f, -2.22907f, -0.129923f, -0.967312f, -0.15795f, -3.16021f, -2.9304f, 0.164658f, 0.222398f, -2.22907f, -0.129923f, -4.51869f, -1.12821f, -0.967312f, -0.15795f, -0.161891f, -0.130852f, -0.97251f, -1.18862f, 1.48938f, 0.231405f, -1.04259f, -0.0070588f, -2.49032f, -0.995234f, -0.161891f, -0.130852f, 0.97251f, 1.18862f, -1.04259f, -0.0070588f, 1.48938f, 0.231405f, -2.49032f, -0.995234f};
;
wheel_leg::SJTU_wheel_leg my_chassis(&left_leg,&right_leg,ins,static_K);

auto rc = bsp_rc_data();
void app_chassis_task(void *args) {
	// Wait for system init.
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
    my_chassis.wheel_leg_init();
    while(true) {
        if(rc->s_l == 1)my_chassis.wheel_leg_update(0.12,0,0,wheel_leg::E_stand,wheel_leg::E_LQR_static);
        else my_chassis.wheel_leg_update(0.12,0,0,wheel_leg::E_waiting,wheel_leg::E_LQR_static);

        my_chassis.wheel_leg_ctrl();
	    // bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f\n",right_leg.my_leg_status_.L0,right_leg.my_leg_status_.phi0,right_leg.joint_E_->joint_deg_,right_leg.joint_A_->joint_deg_);
        // bsp_uart_printf(E_UART_DEBUG,"%f,%f\n",right_leg.my_out_put_.tor_JA,right_leg.my_out_put_.tor_JE);
        // bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f,%f,%f,%f\n",right_leg.my_leg_status_.phi0,right_leg.my_leg_status_.phi1,right_leg.my_leg_status_.phi2,right_leg.my_leg_status_.phi3,right_leg.my_leg_status_.phi4,right_leg.joint_A_->joint_deg_,right_leg.joint_E_->joint_deg_);
        // bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f\n",ins->roll,ins->pitch,ins->raw.gyro[0]);
	    OS::Task::SleepMilliseconds(1);
	}
}

void app_chassis_init() {

}

#endif