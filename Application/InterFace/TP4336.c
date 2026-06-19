//
// Created by 23029 on 2026/5/24.
//
#include "stm32f1xx_hal.h"
#include "TP4336.h"

/**
 * @brief 用于启动TP4336电源避免自动关机
 *  TP4336芯片判断按键在按下50ms视为单按，用软件模拟开关按下避免自动关机
 */
void TP4336_Start(void) {

HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
    vTaskDelay(100);
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
}

/**
 * @brief 软件关闭TP4336
 */
void TP4336_Stop(void) {

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
}