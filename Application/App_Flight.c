//
// Created by 23029 on 2026/6/6.
//

#include "App_Flight.h"

Gyro_Accel_Struct gyro_accel_data={0};
Euler_Struct euler_angle={0};
Gyro_Struct gyro_last_data={0};
float gyro_z_sum;

//俯仰角的PID结构体
PID_Struct Pitch_PID={.Kp=5.00,.Ki=0.00,.Kd=0.00};
//Y轴角速度的结构体，用作俯仰角PID的内环
PID_Struct Gyro_y_PID={.Kp=-3.00,.Ki=0.00,.Kd=-0.40};
//横滚角的PID结构体
PID_Struct Roll_PID={.Kp=-5.00,.Ki=0.00,.Kd=0.00};
//X轴角速度的结构体，用作横滚角PID的内环
PID_Struct Gyro_x_PID={.Kp=3.00,.Ki=0.00,.Kd=0.40};
//偏航角的PID结构体
PID_Struct Yaw_PID={.Kp=2.00,.Ki=0.00,.Kd=0.00};
//Z轴角速度的结构体，用作偏航角PID的内环
PID_Struct Gyro_z_PID={.Kp=5.00,.Ki=0.00,.Kd=0.00};

/*电机结构体*/
//定义电机控制结构体0代表上，1代表下，left_0代表左上
Motor_Struct Left_0_Motor={.tim =&htim3,.tim_channel = TIM_CHANNEL_1,.tim_compare = 0};
Motor_Struct Left_1_Motor={.tim = &htim4,.tim_channel = TIM_CHANNEL_4,.tim_compare = 0};
Motor_Struct Right_0_Motor={.tim =&htim2,.tim_channel = TIM_CHANNEL_2,.tim_compare = 0};
Motor_Struct Right_1_Motor={.tim = &htim1,.tim_channel = TIM_CHANNEL_3,.tim_compare = 0};

extern Remote_Data receive_data;
extern Flight_State flight_state;
extern TaskHandle_t com_task_handle;

/**
 * @brief 飞控任务初始化，MPU6050初始化 ，电机初始化
 */
void App_Flight_Init(void) {
    /*1.MPU6050初始化*/
    MPU6050_Init();
    /*2.启动电机*/
    Motor_Start(&Left_0_Motor);
    Motor_Start(&Left_1_Motor);
    Motor_Start(&Right_0_Motor);
    Motor_Start(&Right_1_Motor);
}


/**
 * @brief 根据陀螺仪测量的数据，计算欧拉角
 *
 */
void App_Flight_Get_Euler_Angle(void) {
    /*1.获取六轴数据*/
    //uint32_t t1 = HAL_GetTick();
    MPU6050_Get_Data_Con(&gyro_accel_data);
    //uint32_t t2 = HAL_GetTick();
    //printf("MPU Cost=%lu\r\n", t2-t1);
    /*1.对角速度进行低通滤波*/
    gyro_accel_data.gyro.gyro_x=Common_Filter_LowPass(gyro_accel_data.gyro.gyro_x, gyro_last_data.gyro_x);
    gyro_accel_data.gyro.gyro_y=Common_Filter_LowPass(gyro_accel_data.gyro.gyro_y, gyro_last_data.gyro_y);
    gyro_accel_data.gyro.gyro_z=Common_Filter_LowPass(gyro_accel_data.gyro.gyro_z, gyro_last_data.gyro_z);
    //对上一次的数据进行保存
    gyro_last_data.gyro_x=gyro_accel_data.gyro.gyro_x;
    gyro_last_data.gyro_y=gyro_accel_data.gyro.gyro_y;
    gyro_last_data.gyro_z=gyro_accel_data.gyro.gyro_z;

    /*2.对加速度进行卡尔曼滤波*/
    gyro_accel_data.accel.accel_x=Common_Filter_KalmanFilter(&kfs[0],gyro_accel_data.accel.accel_x);
    gyro_accel_data.accel.accel_y=Common_Filter_KalmanFilter(&kfs[1],gyro_accel_data.accel.accel_y);
    gyro_accel_data.accel.accel_z=Common_Filter_KalmanFilter(&kfs[2],gyro_accel_data.accel.accel_z);

    //printf("%d,%d,%d\r\n",gyro_accel_data.gyro.gyro_x,gyro_accel_data.gyro.gyro_y,gyro_accel_data.gyro.gyro_z);
    //printf("%d,%d,%d\r\n",gyro_accel_data.accel.accel_x,gyro_accel_data.accel.accel_y,gyro_accel_data.accel.accel_z);
                                                // /*3.通过加速度和角速度进行姿态结算=>计算当前倾斜的角度*/
                                                // //使用加速度求俯仰角和横滚角，对角速度积分实现偏航角
                                                // euler_angle.pitch=atan2(gyro_accel_data.accel.accel_x*1.0,
                                                //     gyro_accel_data.accel.accel_z)/3.14159*180;
                                                // euler_angle.roll=atan2(gyro_accel_data.accel.accel_y*1.0,
                                                //     gyro_accel_data.accel.accel_z)/3.14159*180;
                                                // gyro_z_sum+=(gyro_accel_data.gyro.gyro_z*2000.0/32768.0)*0.006;
                                                // euler_angle.yaw=gyro_z_sum;
    // //任务6ms执行一次，每次执行都偶会读取一次角速度，角速度×持续的时间=该任务时段转过的角度，进行累加得到累计转的角度
    /*3.使用移植的四元姿态解算计算欧拉角*/
    Common_IMU_GetEulerAngle(&gyro_accel_data,&euler_angle,0.006);
   // printf("%.2f,%.2f,%.2f\n",euler_angle.pitch,euler_angle.roll,euler_angle.yaw);
}

/**
 * @brief 根据欧拉角计算PID的目标值
 */
void App_Flight_PID_Process(void) {
    /*1.计算俯仰角的PID的值*/
    //串级PID控制需填入外环的目标值和测量值，以及内环的测量值
    //数值转换，俯仰角需控制在±10°，而receive_data的范围在0~1000.
    Pitch_PID.Target=(receive_data.pitch-500)/50.0;
    //外环的真实值是当前的俯仰角
    Pitch_PID.Actual=euler_angle.pitch;
    //内环的真实值是当前Y轴的角速度
    Gyro_y_PID.Actual=(gyro_accel_data.gyro.gyro_y* 2000.0 / 32768.0);
    /*2.进行PID计算*/
    Com_PID_Calculate_Chain(&Pitch_PID,&Gyro_y_PID);
    //printf("%.2f,%.2f\n",Gyro_y_PID.Error,Gyro_y_PID.Output);
    /*3.计算横滚角的PID的值*/
    //数值转换，横滚角需控制在±10°，而receive_data的范围在0~1000.
    Roll_PID.Target=(receive_data.roll-500)/50.0;
    //外环的实际值为当前的横滚角
    Roll_PID.Actual=euler_angle.roll;
    //内环的实际值为当前X轴的角速度
    Gyro_x_PID.Actual=(gyro_accel_data.gyro.gyro_x*2000.0/32768.0);
    /*4.进行PID计算*/
    Com_PID_Calculate_Chain(&Roll_PID,&Gyro_x_PID);
    /*5.计算俯仰角的PID的值*/
    //串级PID控制需填入外环的目标值和测量值，以及内环的测量值
    //数值转换，偏航角需控制在±10°，而receive_data的范围在0~1000.
    Yaw_PID.Target=(receive_data.yaw-500)/50.0;
    //外环的真实值是当前的俯仰角
    Yaw_PID.Actual=euler_angle.yaw;
    //内环的真实值是当前Y轴的角速度
    Gyro_z_PID.Actual=(gyro_accel_data.gyro.gyro_z* 2000.0 / 32768.0);
    /*6.进行PID计算*/
    Com_PID_Calculate_Chain(&Yaw_PID,&Gyro_z_PID);

}

/**
 * @brief 通过PID的输出控制电机
 */
void App_Flight_Control_Motor(void) {

    switch (flight_state) {
        case IDLE:
        /*当飞机处于空闲状态（包括锁死状态）时，电机速度为0*/
            Left_0_Motor.tim_compare=0;
            Left_1_Motor.tim_compare=0;
            Right_0_Motor.tim_compare=0;
            Right_1_Motor.tim_compare=0;

            break;
        case NORMAL:
        /*当飞机处于正常状态时*/
        //控制俯仰角->向前飞时，俯仰角的误差为正->需反馈一个向后飞的效果->前两个电机转速加快，后两个转的慢。
            Left_0_Motor.tim_compare=receive_data.thr+Gyro_y_PID.Output+Gyro_x_PID.Output-Gyro_z_PID.Output;
            Left_1_Motor.tim_compare=receive_data.thr-Gyro_y_PID.Output+Gyro_x_PID.Output+Gyro_z_PID.Output;
            Right_0_Motor.tim_compare=receive_data.thr+Gyro_y_PID.Output-Gyro_x_PID.Output-Gyro_z_PID.Output;
            Right_1_Motor.tim_compare=receive_data.thr-Gyro_y_PID.Output-Gyro_x_PID.Output+Gyro_z_PID.Output;
            break;
        case FIX_HEIGHT:

            break;
        case FAIL:
            //当飞行状态出现故障时，对电机进行减速
            Left_0_Motor.tim_compare-=2;
            Left_1_Motor.tim_compare-=2;
            Right_0_Motor.tim_compare-=2;
            Right_1_Motor.tim_compare-=2;
            //当点击减速为0时，通知任务故障处理完成
            if (Left_0_Motor.tim_compare==0&&Left_1_Motor.tim_compare==0
                &&Right_0_Motor.tim_compare==0&&Right_1_Motor.tim_compare==0) {

                xTaskNotifyGive(com_task_handle);
            }
            break;
        default:

            break;
    }
    /*电机速度限幅*/
    Left_0_Motor.tim_compare=Com_Limit(Left_0_Motor.tim_compare,0,800);
    Left_1_Motor.tim_compare=Com_Limit(Left_1_Motor.tim_compare,0,800);
    Right_0_Motor.tim_compare=Com_Limit(Right_0_Motor.tim_compare,0,800);
    Right_1_Motor.tim_compare=Com_Limit(Right_1_Motor.tim_compare,0,800);
    /*安全限制*/
    if (receive_data.thr<50) {
        Left_0_Motor.tim_compare=0;
        Left_1_Motor.tim_compare=0;
        Right_0_Motor.tim_compare=0;
        Right_1_Motor.tim_compare=0;
    }
    /*设置电机速度*/
    Motor_Set_Speed(&Left_0_Motor);
    Motor_Set_Speed(&Left_1_Motor);
    Motor_Set_Speed(&Right_0_Motor);
    Motor_Set_Speed(&Right_1_Motor);
}
