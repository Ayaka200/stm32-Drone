//
// Created by 23029 on 2026/6/8.
//
#include "main.h"
#include "Com_PID.h"

/**
 * @brief 计算单环PID
 * @param pid 包含PID参数的结构体
 */
void Com_PID_Caculate(PID_Struct * pid) {
    /*1.通过目标值和测量值计算误差*/
    pid->Error=pid->Target-pid->Actual;
    /*2.计算累计误差和微分误差*/
    pid->Error_Int+=pid->Error;
    pid->Error_Der=pid->Error-pid->Error_Last;
    /*3.计算输出值*/
    pid->Output=pid->Kp*pid->Error+pid->Ki*(pid->Error_Int*PID_PERIOD)+pid->Kd*(pid->Error_Der/PID_PERIOD);
    pid->Error_Last=pid->Error;

}

/**
 * @brief 计算串级PID
 * @param outer_pid 外环控制的PID结构体
 * @param inner_pid 内环控制的PID结构体
 */
void Com_PID_Calculate_Chain(PID_Struct * outer_pid, PID_Struct * inner_pid) {
    /*1.先计算外环PID的Output*/
    Com_PID_Caculate(outer_pid);
    /*2.将外环的输出值作为内环目标值进行计算*/
    outer_pid->Output=inner_pid->Target;
    Com_PID_Caculate(inner_pid);
}

/**
 * @brief 用于限制数值在正常范围内
 * @param input 限制的变量
 * @param min   最小值
 * @param max   最大值
 * @return      限制后的变量
 */
int16_t Com_Limit(int16_t input, int16_t min, int16_t max) {
    if (input < min) {
        return min;
    }
    else if (input > max) {
        return max;
    }
    return input;
}