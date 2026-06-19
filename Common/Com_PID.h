//
// Created by 23029 on 2026/6/8.
//

#ifndef NEW_DRONE_COM_PID_H
#define NEW_DRONE_COM_PID_H

#include "stm32f1xx_hal.h"

#define PID_PERIOD 0.006

//PID结构体
typedef  struct {

    float Kp;               //比例部分系数，直接作用在误差上
    float Ki;               //积分部分系数
    float Kd;               //微分部分系数
    float Error;            //当前误差值
    float Error_Last;       //上一次误差值
    float Error_Int;        //累计误差
    float Error_Der;        //微分误差
    float Target;           //目标值
    float Actual;          //测量值
    float Output;
}PID_Struct;


void Com_PID_Caculate(PID_Struct * pid);
void Com_PID_Calculate_Chain(PID_Struct * outer_pid, PID_Struct * inner_pid);
int16_t Com_Limit(int16_t input, int16_t min, int16_t max);
#endif //NEW_DRONE_COM_PID_H