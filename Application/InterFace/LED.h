//
// Created by 23029 on 2026/5/26.
//

#ifndef NEW_DRONE_LED_H
#define NEW_DRONE_LED_H
#include "main.h"
typedef struct {

    GPIO_TypeDef * GPIO_Port;
    uint16_t GPIO_Pin;

}LED_Struct;

void LED_Turn_On(LED_Struct *LED);
void LED_Turn_Off(LED_Struct *LED);
void LED_Toggle(LED_Struct *LED);

#endif //NEW_DRONE_LED_H