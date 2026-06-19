//
// Created by 23029 on 2026/5/26.
//

#ifndef NEW_DRONE_COM_CONFIG_H
#define NEW_DRONE_COM_CONFIG_H
#include "main.h"

//连接状态
typedef enum {
    REMOTE_CONNECT=1,
    REMOTE_DISCONNECT,
}Remote_State;
//飞行状态
typedef enum {
    IDLE=0,
    NORMAL,
    FIX_HEIGHT,
    FAIL,
}   Flight_State;
//遥控数据
typedef struct {
    int16_t thr;
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    uint8_t shutdown;
    uint8_t fix_height;

}Remote_Data;

//定义帧头校验
#define FRAME_HEAD_CHECK_1 'w'
#define FRAME_HEAD_CHECK_2 'x'
#define FRAME_HEAD_CHECK_3 'z'

//油门解锁结构体
typedef enum {
    FREE=0,
    MAX,
    MIN,
    LEAVE_MAX,
    UNLOCK,
}Thr_State;

//陀螺仪数据
/*1.加速度*/
typedef struct {
    int16_t accel_x;    //朝前的加速度为正
    int16_t accel_y;    //朝左的加速度为正
    int16_t accel_z;    //朝上的加速度为正
}Accel_Struct;
/*2.角速度*/
typedef struct {
    int16_t gyro_x;     //向右转为正，表示横滚角
    int16_t gyro_y;     //向前转数值为正，向后转为负，表示俯仰角
    int16_t gyro_z;     //逆时针转为正，表示偏航角
}Gyro_Struct;

typedef struct {
    Accel_Struct accel;
    Gyro_Struct gyro;
}Gyro_Accel_Struct;
//欧拉角
typedef struct {
    float yaw;
    float pitch;
    float roll;
}Euler_Struct;

#endif //NEW_DRONE_COM_CONFIG_H