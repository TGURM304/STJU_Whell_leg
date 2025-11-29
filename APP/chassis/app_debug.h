//
// Created by 15082 on 2025/11/23.
//

#ifndef APP_DEBUG_H
#define APP_DEBUG_H

namespace  AppDebug {
    typedef enum { FREE_DEBUG, CHASSIS_OUTPUT, INS_RAW_DEG, INS_RAW_ACC, CHASSIS_STATE, CHASSIS_DELTA, DEBUG_TEMP } Debug_Output;
inline Debug_Output DEBUG_TYPE = CHASSIS_STATE;
};


#endif //APP_DEBUG_H
