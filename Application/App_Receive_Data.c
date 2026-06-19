//
// Created by 23029 on 2026/6/3.
//

#include "App_Receive_Data.h"
#include <string.h>

#include "task.h"

uint8_t rx_buff[TX_PLOAD_WIDTH]={0};
extern Remote_State remote_state;
extern Remote_Data receive_data;
extern Flight_State flight_state;
extern uint8_t Back_Buff[TX_PLOAD_WIDTH];
Thr_State thr_state=FREE;
uint8_t count=0;        //用于记录数据接收失败的次数
uint32_t enter_time=0;

/**
 *
 * @return 0:解锁成功，1:解锁失败
 */
static uint8_t App_Process_Unlock(void) {

    switch (thr_state) {
        case FREE:
            if (receive_data.thr>=900) {
                /*油门值大于900进入MAX状态*/
                thr_state=MAX;
                /*开始计算进入MAX状态的时间*/
                enter_time=xTaskGetTickCount();
            }
            break;
        case MAX:
            if (receive_data.thr<=900) {
                /*油门值小于900时判断油门持续时间是否超过1s*/
                if (xTaskGetTickCount()-enter_time>=1000) {
                    thr_state=LEAVE_MAX;
                }
                else {
                    thr_state=FREE;
                }
            }
            break;
        case LEAVE_MAX:
            if (receive_data.thr<=100) {
                thr_state=MIN;
                enter_time=xTaskGetTickCount();
            }

            break;
        case MIN:
            if (xTaskGetTickCount()-enter_time<=1000) {
                if (receive_data.thr>100) {
                    thr_state=FREE;
                }
                thr_state=UNLOCK;
            }
            else {
                thr_state=FREE;
            }
            break;
        case UNLOCK:

            break;
        default:
            break;
    }
    if (thr_state==UNLOCK) {
        return 0;
    }

       return 1;
}

/**
 *  @brief 用于接收遥控器发来的数据
 * @return 0：校验通过 1：校验未通过
 */
uint8_t App_Receive_Data(void) {

    memset(rx_buff,0,TX_PLOAD_WIDTH);
    /*接收遥控发来的数据*/
    uint8_t res= nRF24L01P_RxPacket(rx_buff);
    /*判断是否接收收到数据*/
    if (res==0) {
        /*准备回传电压数据 */
        nRF24L01P_TX_Mode();
        /*等待数据回传成功*/
        while (nRF24L01P_TxPacket(Back_Buff)==1);
        nRF24L01P_RX_Mode();
    }
    /*1.进行帧头校验*/
    if (rx_buff[0]!=FRAME_HEAD_CHECK_1||rx_buff[1]!=FRAME_HEAD_CHECK_2
        ||rx_buff[2]!=FRAME_HEAD_CHECK_3 )
    {
        return 1;
    }
    /*2.进行校验和校验*/
    uint8_t sum_receive=0;
    uint8_t sum=0;
    for (uint8_t i=0;i<TX_PLOAD_WIDTH-4;i++) {
        sum+=rx_buff[i];
    }
    sum_receive=rx_buff[13]<<24|rx_buff[14]<<16|rx_buff[15]<<8|rx_buff[16];
    if (sum_receive!=sum) {
        return 1;
    }
    /*3.进行数据解析并保存*/
    receive_data.thr=(rx_buff[3]<<8)|rx_buff[4];
    receive_data.yaw=(rx_buff[5]<<8)|rx_buff[6];
    receive_data.pitch=(rx_buff[7]<<8)|rx_buff[8];
    receive_data.roll=(rx_buff[9]<<8)|rx_buff[10];
    receive_data.shutdown=rx_buff[11];
    receive_data.fix_height=rx_buff[12];

    //printf(":%d,%d,%d,%d,%d,%d\n",receive_data.thr,receive_data.yaw,
        //receive_data.pitch,receive_data.roll,receive_data.shutdown,receive_data.fix_height);
    return 0;
}

/**
 * @brief 通过判断数据接收的结果确定当前的连接状态
 * @param result 上一次数据接收的结果
 */
void App_Process_Connect_State(uint8_t result) {
     /*接受数据成功,记为连接成功*/

    if (result==0) {
        remote_state=REMOTE_CONNECT;
        count=0;
    }
    /*接受数据失败,记录失败次数，超过最大检测次数则记为连接失败*/
    else if (result==1) {
        count++;
        if (count>=RX_TEST_MAX_TIME) {
            remote_state=REMOTE_DISCONNECT;
            count=0;
        }

    }
}

/**
 * @brief 处理飞机的飞行状态
 */
void App_Process_Flight_State(void) {

    /*使用状态机来判断无人机所处的状态*/
    switch (flight_state) {
        case IDLE:
            if (App_Process_Unlock()==0) {
                flight_state=NORMAL;
                thr_state=FREE;
            }
            break;
        case NORMAL:
            /*1.判断定高状态*/
            if (receive_data.fix_height==1) {
                flight_state=FIX_HEIGHT;
                receive_data.fix_height=0;
            }
            /*2.判断故障失联状态*/
            if (remote_state==REMOTE_DISCONNECT) {
                flight_state=FAIL;
            }
            break;
        case FIX_HEIGHT:
            /*1.判断定高（取消）状态*/
            if (receive_data.fix_height==1) {
                flight_state=NORMAL;
                receive_data.fix_height=0;
            }
            /*2.判断故障失联状态*/
            if (remote_state==REMOTE_DISCONNECT) {
                flight_state=FAIL;
            }
            break;
        case FAIL:
            /*1.处理失联故障，缓慢停止电机*/
            /*code*/
            /*2.退回到正常状态*/
            vTaskDelay(1);
            flight_state=IDLE;
            break;
        default:
            break;
    }

}