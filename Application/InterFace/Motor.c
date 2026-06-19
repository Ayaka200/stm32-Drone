//
// Created by 23029 on 2026/5/25.
//
#include "stm32f1xx_hal.h"
#include "Motor.h"

#include <stdio.h>
//include "Com_debug.h"
/**
 * @brief 通过传递的结构体指针修改变量控制电机转速
 * @param motor :电机结构体
 */
void Motor_Set_Speed(Motor_Struct *motor) {
    if (motor->tim_compare>=1000) {
        printf("motor_speed is error");
        return;
    }

    __HAL_TIM_SET_COMPARE(motor->tim,motor->tim_channel,motor->tim_compare);
}

/**
 * @brief 传入具体电机的结构体，启动电机
 * @param motor
 */
void Motor_Start(Motor_Struct *motor) {

    HAL_TIM_PWM_Start(motor->tim,motor->tim_channel);


}
