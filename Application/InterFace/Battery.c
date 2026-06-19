//
// Created by 23029 on 2026/6/18.
//

#include "Battery.h"

/**
 * @brief ADC检测电池电压初始化
 */
void Bat_ADC_Init(void) {
    /*1.使能ADC检测引脚*/
    HAL_GPIO_WritePin(BAT_ADC_EN_GPIO_Port, BAT_ADC_EN_Pin, GPIO_PIN_RESET);
    /*启动ADC转换*/
    HAL_ADC_Start(&hadc1);
}

/**
 * @brief 读取ADC检测值
 * @return 返回电池电压
 */
float Bat_ADC_Read(void) {
    uint32_t adc_val=0;
    adc_val=HAL_ADC_GetValue(&hadc1);
    float voltage=adc_val*3.3/4095*2;
    return voltage;
}