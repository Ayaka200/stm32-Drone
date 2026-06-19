//
// Created by 23029 on 2026/6/3.
//

#ifndef NEW_DRONE_APP_RECEIVE_DATA_H
#define NEW_DRONE_APP_RECEIVE_DATA_H

#include "stm32f1xx_hal.h"
#include "nRF24L01P.h"
#include "Com_config.h"
#include "FreeRTOS.h"
#include "task.h"
//最大连接状态检测次数
#define RX_TEST_MAX_TIME 50

uint8_t App_Receive_Data(void);
void App_Process_Connect_State(uint8_t result);
void App_Process_Flight_State(void);

#endif //NEW_DRONE_APP_RECEIVE_DATA_H