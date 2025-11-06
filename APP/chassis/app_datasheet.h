//
// Created by 15082 on 2025/10/18.
//

#ifndef APP_WHEEL_LEG_DATASHEET_H
#define APP_WHEEL_LEG_DATASHEET_H

#define LEG_L2 0.16f
#define LEG_L1 0.08f
#define LEG_L4 LEG_L1
#define LEG_L3 LEG_L2
#define LEG_AE 0.0814f
#define LEG_M 0.168f
#define LEG_L (0.184f-LEG_M)
#define LEG_Ip (LEG_L*LEG_L*LEG_M)
#define G 9.82f

#define WHEEL_M 0.8f
#define WHEEL_R 0.06f
#define WHEEL_I (566.352f*1.3/1000/1000)

#define BODY_M 4.56f
#define BODY_L 0.005f
#define BODY_Im (BODY_L*BODY_L*BODY_M)

#define MOTOR_GEAR (3591.0/187.0)

#endif //APP_WHEEL_LEG_DATASHEET_H
