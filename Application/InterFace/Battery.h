//
// Created by 23029 on 2026/6/18.
//

#include "stm32f1xx_hal.h"
#include "adc.h"

#ifndef NEW_DRONE_BATTERY_H
#define NEW_DRONE_BATTERY_H

void Bat_ADC_Init(void);
float Bat_ADC_Read(void);

#endif //NEW_DRONE_BATTERY_H