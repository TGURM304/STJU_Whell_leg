//
// Created by fish on 2024/12/18.
//

#include "app_chassis.h"

#include "app_sys.h"
#include "sys_task.h"
#include "app_chassis_motor.h"
#include "app_chassis_wheel_leg.h"
#include "app_datasheet.h"
#include "app_debug.h"
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
float static_K[40] = {-3.14537f, -2.8707f, 0.623247f, 0.233007f, -4.36555f, -1.10713f, -2.16955f, -0.155681f, -0.82658f, -0.10262f, -3.14537f, -2.8707f, -0.623247f, -0.233007f, -2.16955f, -0.155681f, -4.36555f, -1.10713f, -0.82658f, -0.10262f, -0.326573f, -0.270589f, -2.66675f, -1.57233f, 0.565233f, -0.135067f, -0.678189f, 0.198023f, -1.89352f, -0.71604f, -0.326573f, -0.270589f, 2.66675f, 1.57233f, -0.678189f, 0.198023f, 0.565233f, -0.135067f, -1.89352f, -0.71604f};

wheel_leg::SJTU_wheel_leg my_chassis(&left_leg,&right_leg,ins,static_K);

auto rc = bsp_rc_data();
void app_chassis_task(void *args) {
	// Wait for system init.
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
    my_chassis.wheel_leg_init();
    while(true) {
        if(rc->s_l == 1)my_chassis.wheel_leg_update(0.12,-(float32_t)(rc->reserved)/660.f/1000,(float32_t)(rc->rc_l[1])/660.f/1000,0,wheel_leg::E_stand,wheel_leg::E_LQR_static);
        else my_chassis.wheel_leg_update(0.12,0,0,0,wheel_leg::E_waiting,wheel_leg::E_LQR_static);
        my_chassis.wheel_leg_ctrl();
        if(AppDebug::DEBUG_TYPE == AppDebug::FREE_DEBUG)
            bsp_uart_printf(E_UART_DEBUG,"%d,%d,%d\n",rc->rc_l[0],rc->rc_l[1],rc->reserved);
	    OS::Task::SleepMilliseconds(1);
	}
}

void app_chassis_init() {

}

#endif