//
// Created by 23029 on 2026/5/25.
//

#ifndef DRONE_MOTOR_H
#define DRONE_MOTOR_H
#include "stm32f1xx_hal_tim.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/**
 * @brief 定义一个结构体用来设置各个定时器通道的比较值来控制电机的转速，
 * 由__HAL_TIM_SET_COMPARE()可知，改变比较值需传递定时器句柄，通道，以及比较值
 * @param TIM_HandleTypeDef *tim:定时器句柄
 * @param uint16_t tim_channel:定时器通道
 * @param uint16_t tim_compare:比较值
 */
typedef struct {

        TIM_HandleTypeDef *tim;
        uint16_t tim_channel;
        int16_t tim_compare;

}Motor_Struct;
void Motor_Set_Speed(Motor_Struct *motor);

void Motor_Start(Motor_Struct *motor);

#endif //DRONE_MOTOR_H