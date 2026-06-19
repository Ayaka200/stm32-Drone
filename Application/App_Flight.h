//
// Created by 23029 on 2026/6/6.
//

#ifndef NEW_DRONE_APP_FLIGHT_H
#define NEW_DRONE_APP_FLIGHT_H

#include "main.h"
#include "math.h"
#include "stm32f1xx_hal.h"
#include "Com_config.h"
#include "Com_imu.h"
#include "Com_filter.h"
#include "MPU6050.h"
#include "Com_PID.h"
#include  "Motor.h"

void App_Flight_Init(void);
void App_Flight_Get_Euler_Angle(void);
void App_Flight_PID_Process(void);
void App_Flight_Control_Motor(void);

#endif //NEW_DRONE_APP_FLIGHT_H