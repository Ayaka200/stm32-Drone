//
// Created by 23029 on 2026/5/26.
//
#include "stm32f1xx_hal.h"
#include "LED.h"

/**
 *@brief 打开指定的LED灯
 * @param LED 控制LED的结构体
 */
void LED_Turn_On(LED_Struct *LED) {

    HAL_GPIO_WritePin(LED->GPIO_Port,LED->GPIO_Pin,GPIO_PIN_RESET);

}

/**
 * @brief 关闭指定的LED灯
 * @param LED 控制LED的结构体
 */
void LED_Turn_Off(LED_Struct *LED) {
    HAL_GPIO_WritePin(LED->GPIO_Port,LED->GPIO_Pin,GPIO_PIN_SET);
}

/**
 * @brief 翻转指定LED灯的状态
 * @param LED 控制LED的结构体
 */
void LED_Toggle(LED_Struct *LED) {
    HAL_GPIO_TogglePin(LED->GPIO_Port,LED->GPIO_Pin);
}