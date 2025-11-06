//
// Created by fish on 2024/12/18.
//

#include "app_chassis.h"

#include "app_sys.h"
#include "sys_task.h"
#include "app_chassis_motor.h"
#include "app_datasheet.h"
#include "app_leg.h"
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
wheel_leg_motor::dynamic right_dynamic(&right_dji, -1);

chassis_Leg::App_Leg right_leg(&right_A, &right_E, &right_dynamic);

void app_chassis_task(void *args) {
	// Wait for system init.
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
    right_leg.leg_init();
	while(true) {
        right_leg.leg_ctrl(0,-2,0);
	    // bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f\n",right_leg.my_leg_status_.L0,right_leg.my_leg_status_.phi0,right_leg.joint_E_->joint_deg_,right_leg.joint_A_->joint_deg_);
        // bsp_uart_printf(E_UART_DEBUG,"%f,%f\n",right_leg.my_out_put_.tor_JA,right_leg.my_out_put_.tor_JE);
        bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f,%f,%f,%f\n",right_leg.my_leg_status_.phi0,right_leg.my_leg_status_.phi1,right_leg.my_leg_status_.phi2,right_leg.my_leg_status_.phi3,right_leg.my_leg_status_.phi4,right_leg.joint_A_->joint_deg_,right_leg.joint_E_->joint_deg_);

	    OS::Task::SleepMilliseconds(1);
	}
}

void app_chassis_init() {

}

#endif