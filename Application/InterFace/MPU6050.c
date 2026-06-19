//
// Created by 23029 on 2026/6/5.
//

#include "MPU6050.h"
#include <stdlib.h>
//加速度的偏移量
int32_t acc_x_offset=0;
int32_t acc_y_offset=0;
int32_t acc_z_offset=0;
//角速度的偏移量
int32_t gyro_x_offset=0;
int32_t gyro_y_offset=0;
int32_t gyro_z_offset=0;

/**
 * @brief 写寄存器
 * @param reg 寄存器地址
 * @param data 传输的数据
 */
void MPU6050_Write_Reg(uint8_t reg, uint8_t data) {

    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

/**
 * @brief 读寄存器
 * @param reg 寄存器地址
 * @param data 存数据的地址
 */
void MPU6050_Read_Reg(uint8_t reg, uint8_t *data) {
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS_READ, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}

/**
 * @brief 对MPU6050的数据进行零篇校准
 */
void MPU6050_Caculate_Offset(void) {
    /*1.判断无人机是否停稳*/
    //判断标准：前后两次加速的的差值小于200
    Accel_Struct current_accel={0};
    Accel_Struct last_accel={0};
    uint8_t count=0;
    MPU6050_Get_Accel(&last_accel);
    while (count<100) {
        MPU6050_Get_Accel(&current_accel);
        if (abs(current_accel.accel_x-last_accel.accel_x)<400&&
            abs(current_accel.accel_y-last_accel.accel_y)<400&&
            abs(current_accel.accel_z-last_accel.accel_z)<400) {
            count++;
        }
        else {
            count=0;
        }
        last_accel=current_accel;
        vTaskDelay(6);
    }
    int32_t acc_x_sum=0;
    int32_t acc_y_sum=0;
    int32_t acc_z_sum=0;
   
    int32_t gyro_x_sum=0;
    int32_t gyro_y_sum=0;
    int32_t gyro_z_sum=0;
    /*2.无人机平稳后，开始进行零篇校准*/
    Gyro_Accel_Struct gyro_accel_data={0};
    for (uint8_t i=0;i<100;i++) {
        //每次循环都获取一次角速度和加速度
        MPU6050_Get_Data(&gyro_accel_data);
        //对偏移量进行累加
        acc_x_sum+=(gyro_accel_data.accel.accel_x-0);
        acc_y_sum+=(gyro_accel_data.accel.accel_y-0);
        acc_z_sum+=(gyro_accel_data.accel.accel_z-16384);

        gyro_x_sum+=(gyro_accel_data.gyro.gyro_x-0);
        gyro_y_sum+=(gyro_accel_data.gyro.gyro_y-0);
        gyro_z_sum+=(gyro_accel_data.gyro.gyro_z-0);
        vTaskDelay(6);
    }
    acc_x_offset=acc_x_sum/100;
    acc_y_offset=acc_y_sum/100;
    acc_z_offset=acc_z_sum/100;
    gyro_x_offset=gyro_x_sum/100;
    gyro_y_offset=gyro_y_sum/100;
    gyro_z_offset=gyro_z_sum/100;
}

/**
 * @brief MPU6050的初始化
 */
void MPU6050_Init(void) {
    /*1.重启MPU6050，重置所有寄存器=>给电源管理器1PWR_MGMT_1(0x6B)的第七位写1即可*/
    MPU6050_Write_Reg(0x6B,0x80);
    //电源管理寄存器1重置的值为0x40,重置后默认进入低功耗模式，需将第6位（sleep）置0唤醒
    uint8_t Data;
    while (Data!=0x40) {
        MPU6050_Read_Reg(0x6B, &Data);
    }
    /*2.唤醒MPU6050，进入正常工作状态*/
    MPU6050_Write_Reg(0x6B,0x00);
    /*3.设置陀螺仪的量程(0x1B)*/
    /*3.1设置角速度GYRO_CONFIG的量程位±2000°*/
    MPU6050_Write_Reg(0x1B,0x18);
    /*3.2设置陀螺仪的加速度ACCEL_CONFIG量程位±2g*/
    MPU6050_Write_Reg(0x1C,0x00);
    /*4.关闭中断使能(0x38)*/
    MPU6050_Write_Reg(0x38,0x00);
    /*5.配置用户控制USER_CTRL寄存器(0x6A)，失能FIFO队列和扩展的I2C*/
    MPU6050_Write_Reg(0x6A,0x00);
    /*6.设置陀螺仪的采样率SMPLRT_DIV寄存器(0x19),默认位1KHz*/
    //根据奈奎斯特采样定理，采样频率f1>=信号频率f2(使用频率),
    //该项目设置为500Hz=1000/1+SMPLRT_DIV,所以给寄存器写1
    MPU6050_Write_Reg(0x19,0x01);
    /*7.设置低通滤波(0x1A)的值为184Hz~188Hz，写1即可*/
    MPU6050_Write_Reg(0x1A,0x01);
    /*8.在点与管理寄存器1中配置时钟源的倍频器PLL*/
    MPU6050_Write_Reg(0x6B,0x01);
    /*9.使能加速度传感器和角速度传感器PWR_MGMT_2(0x6C)*/
    MPU6050_Write_Reg(0x6C,0x00);
    /*10.进行零偏校准*/
    MPU6050_Caculate_Offset();
}

/**
 * @brief 获取MPU6050三轴的的角速度
 * @param gyro 角速度的结构体
 */
void MPU6050_Get_Gyro(Gyro_Struct *gyro) {
    //存储角速度的寄存器有6个，从0x43开始，高8位在前，X开始

    uint8_t Hight_Bit=0;
    uint8_t Low_Bit=0;
    /*1.获取X轴的角速度*/
    MPU6050_Read_Reg(MPU_GYRO_XOUTH_REG, &Hight_Bit);
    MPU6050_Read_Reg(MPU_GYRO_XOUTL_REG, &Low_Bit);
    gyro->gyro_x = (Hight_Bit<<8 | Low_Bit)-gyro_x_offset;
    /*2.获取Y轴的角速度*/
    MPU6050_Read_Reg(MPU_GYRO_YOUTH_REG, &Hight_Bit);
    MPU6050_Read_Reg(MPU_GYRO_YOUTL_REG, &Low_Bit);
    gyro->gyro_y = (Hight_Bit<<8 | Low_Bit)-gyro_y_offset;
    /*3.获取Z轴的角速度*/
    MPU6050_Read_Reg(MPU_GYRO_ZOUTH_REG, &Hight_Bit);
    MPU6050_Read_Reg(MPU_GYRO_ZOUTL_REG, &Low_Bit);
    gyro->gyro_z = (Hight_Bit<<8 | Low_Bit)-gyro_z_offset;

}

/**
 * @brief 获取MPU6050三轴的的加速度
 * @param accel 加速度的结构体
 */
void MPU6050_Get_Accel(Accel_Struct *accel) {
    uint8_t Hight_Bit=0;
    uint8_t Low_Bit=0;
    /*1.获取X轴的加速度*/
    MPU6050_Read_Reg(MPU_ACCEL_XOUTH_REG, &Hight_Bit);
    MPU6050_Read_Reg(MPU_ACCEL_XOUTL_REG, &Low_Bit);
    accel->accel_x = (Hight_Bit<<8 | Low_Bit)-acc_x_offset;
    /*2.获取Y轴的加速度*/
    MPU6050_Read_Reg(MPU_ACCEL_YOUTH_REG, &Hight_Bit);
    MPU6050_Read_Reg(MPU_ACCEL_YOUTL_REG, &Low_Bit);
    accel->accel_y = (Hight_Bit<<8 | Low_Bit)-acc_y_offset;
    /*3.获取Z轴的加速度*/
    MPU6050_Read_Reg(MPU_ACCEL_ZOUTH_REG, &Hight_Bit);
    MPU6050_Read_Reg(MPU_ACCEL_ZOUTL_REG, &Low_Bit);
    accel->accel_z = (Hight_Bit<<8 | Low_Bit)-acc_z_offset;
    
}

/**
 * @brief 获取MPU6050六轴所有的数据
 * @param data 加速度和角速度的结构体
 */
void MPU6050_Get_Data(Gyro_Accel_Struct *data) {
    MPU6050_Get_Gyro(&data->gyro);
    MPU6050_Get_Accel(&data->accel);
}

/**
 * @brief 连续获取MPU6050六轴所有的数据
 * @param data 加速度和角速度的结构体
 */
void MPU6050_Get_Data_Con(Gyro_Accel_Struct *data)
{
    uint8_t buf[14];

    HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDRESS_READ,      // 你当前工程能正常工作的话先保持不变
        MPU_ACCEL_XOUTH_REG,       // 0x3B
        I2C_MEMADD_SIZE_8BIT,
        buf,
        14,
        10);

    /* 加速度 */
    data->accel.accel_x =
        ((int16_t)((buf[0] << 8) | buf[1])) - acc_x_offset;

    data->accel.accel_y =
        ((int16_t)((buf[2] << 8) | buf[3])) - acc_y_offset;

    data->accel.accel_z =
        ((int16_t)((buf[4] << 8) | buf[5])) - acc_z_offset;

    /* 温度寄存器 buf[6]、buf[7] */
    // int16_t temp_raw = (int16_t)((buf[6] << 8) | buf[7]);

    /* 角速度 */
    data->gyro.gyro_x =
        ((int16_t)((buf[8] << 8) | buf[9])) - gyro_x_offset;

    data->gyro.gyro_y =
        ((int16_t)((buf[10] << 8) | buf[11])) - gyro_y_offset;

    data->gyro.gyro_z =
        ((int16_t)((buf[12] << 8) | buf[13])) - gyro_z_offset;
}