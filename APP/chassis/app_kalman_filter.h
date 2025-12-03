//
// Created by 15082 on 2025/12/3.
//

#ifndef APP_KALMAN_FILTER_H
#define APP_KALMAN_FILTER_H

#include "app_chassis_wheel_leg.h"

#include <pstl/execution_defs.h>
using namespace wheel_leg;
namespace wheel_leg {
typedef enum {
    E_U_ENABLE,
    E_U_DISABLE
}kalman_u;
typedef enum {
    E_WAITING,
    E_CLC
}kalman_current_state;
typedef struct {
    float32_t *H; //测量矩阵
    float32_t *F; //状态转移矩阵
    float32_t *B; //控制矩阵
    float32_t *Q; //过程噪声协方差矩阵
    float32_t *R; //测量噪声协方差矩阵
    uint8_t size_x, size_z, size_u;
}kalman_para;
typedef struct {
    kalman_u u_state;
    kalman_current_state state;
}kalman_mode;
template<kalman_mode mode, kalman_para para>
class KalmanFilter {
public:
    KalmanFilter():mode_(mode),para_(para) {
        mode_.state = E_WAITING;
        Kalman_reset();
    }
    void kalman_start() {
        //只要在开始的时候调用一次，不用反复调用
        if(mode_.state == E_CLC)
            return;
        else
            Kalman_reset(),mode_.state = E_CLC;
    }
    void Kalman_update(const float32_t *Z_in, const float32_t *U_in) {
        if(mode_.state != E_CLC) {
            return;
        }
        Z_get = Matrixf<para.size_z,1>(Z_in);
        if(mode_.u_state == E_U_ENABLE) {
            U_out = Matrixf<para.size_u,1>(U_in);
            X_pre = F*X_best + B*U_out;
            P_pre = F*P*F.trans() + Q;
            K = P_pre*H.trans()*(H*P_pre*H.trans() + R).inv();
            P = (I - K*H)*P_pre;
        }
        else if(mode.u_state == E_U_DISABLE) {
            X_pre = F*X_best;
            P_pre = F*P*F.trans() + Q;
            K = P_pre*H.trans()*(H*P_pre*H.trans() + R).inv();
            P = (I - K*H)*P_pre;
        }
    }
private:
    void Kalman_reset() {
        //归零内容，初始化
        if(mode.state == E_WAITING)
            return;
        X_best = matrixf::zeros<para.size_x,1>();
        Z_get = matrixf::zeros<para.size_z,1>();
        U_out = matrixf::zeros<para.size_u,1>();
        Q = Matrixf<para.size_x,para.size_x>(para_.Q);
        R = Matrixf<para.size_z,para.size_z>(para_.R);
        B = Matrixf<para.size_x,para.size_u>(para_.B);
        H = Matrixf<para.size_z,para.size_x>(para_.H);
        F = Matrixf<para.size_x,para.size_x>(para_.F);
        P = matrixf::eye<para.size_x,para.size_x>()*500;//不要太小否则效果会很差，迭代几次就能收敛了
        I = matrixf::eye<para.size_x,para.size_x>();
        mode_.state = E_WAITING;
    }
    kalman_para para_;
    kalman_mode mode_;
    Matrixf<para.size_x,1> X_best;
    Matrixf<para.size_z,1> Z_get;
    Matrixf<para.size_u,1> U_out;
    Matrixf<para.size_x,para.size_x> Q; //过程噪声协方差矩阵
    Matrixf<para.size_z,para.size_z> R; //测量噪声协方差矩阵
    Matrixf<para.size_x,para.size_u> B; //控制矩阵
    Matrixf<para.size_x,para.size_x> F; //状态转移矩阵
    Matrixf<para.size_z,para.size_x> H; //测量矩阵
    //以下为中间变量
    Matrixf<para.size_x,1> X_pre;
    Matrixf<para.size_x,para.size_x> P_pre;
    //以下为最后迭代结果，每次rest记得清空，会参与到后续的迭代中
    Matrixf<para.size_x,para.size_x> P; //估计误差协方差矩阵
    Matrixf<para.size_x,para.size_z> K; //卡尔曼增益
    Matrixf<para.size_x,para.size_x> I;
};
}

#endif //APP_KALMAN_FILTER_H
