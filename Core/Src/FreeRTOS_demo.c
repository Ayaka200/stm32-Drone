#include "stm32f1xx_hal.h"
#include "FreeRTOS_demo.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "main.h"


/*LED结构体*/
LED_Struct Left_0_LED={.GPIO_Port =LED1_GPIO_Port,.GPIO_Pin =LED1_Pin};
LED_Struct Right_0_LED={.GPIO_Port =LED2_GPIO_Port,.GPIO_Pin =LED2_Pin};
LED_Struct Right_1_LED={.GPIO_Port =LED3_GPIO_Port,.GPIO_Pin =LED3_Pin};
LED_Struct Left_1_LED={.GPIO_Port =LED4_GPIO_Port,.GPIO_Pin =LED4_Pin};
//表示当前状态
Remote_State remote_state=REMOTE_DISCONNECT;
//表示当前的飞行状态
Flight_State flight_state=IDLE;
//表示接收到的数据
Remote_Data receive_data={.thr = 0,.yaw = 500,.pitch = 500,.roll = 500,
    .fix_height = 0,.shutdown = 0};
//电压值(以数组的形式保存)
uint8_t Back_Buff[TX_PLOAD_WIDTH]={0};
/*启动任务配置*/
#define TASK_START_STACK 128
#define TASK_START_PRIORITY 1
TaskHandle_t task_start_handle;
void task_start(void * pvParameters);
/*电源管理任务*/
#define POWER_TASK_STACK 128
#define POWER_TASK_PRIORITY 4
TaskHandle_t power_task_handle;
void POWER_TASK(void * pvParameters);
#define POWER_TASK_PERIOD 10000
/*飞行控制任务*/
#define FLIGHT_TASK_STACK 256
#define FLIGHT_TASK_PRIORITY 3
TaskHandle_t flight_task_handle;
void FLIGHT_TASK(void * pvParameters);
#define FLIGHT_TASK_PERIOD 6
/*LED控制任务*/
#define LED_TASK_STACK 256
#define LED_TASK_PRIORITY 1
TaskHandle_t led_task_handle;
void LED_TASK(void * pvParameters);
#define LED_TASK_PERIOD 100
/*通讯任务*/
#define COM_TASK_STACK 256
#define COM_TASK_PRIORITY 3
TaskHandle_t com_task_handle;
void COM_TASK(void * pvParameters);
#define COM_TASK_PERIOD 20
/**
 * 启动FreeRTOS
 */
void freertos_start() {

    /*1.创建一个启动任务*/
    xTaskCreate((TaskFunction_t) task_start,                    //任务函数的地址
                 "task_start",                            // 任务名字符串
                (configSTACK_DEPTH_TYPE) TASK_START_STACK,      //任务栈大小，默认最小128，单位4字节
                (void *) NULL,                                  //传递给任务的参数
                (UBaseType_t) TASK_START_PRIORITY,              //任务的优先级
                (TaskHandle_t *) &task_start_handle);           //任务句柄的地址
    /*2.启动调度器，会自动生成空闲任务*/
    vTaskStartScheduler();
}

/**@description:启动任务:用来创建其他Task
 *@param {void} *pvParameters
 * @return {*}
 */

void task_start(void * pvParameters) {
//进入临界区:临界区的代码不会被打断
    taskENTER_CRITICAL();
    /*1.创建电源管理任务*/
    xTaskCreate((TaskFunction_t) POWER_TASK,                    //任务函数的地址
                     "POWER_TASK",                            // 任务名字符串
                    (configSTACK_DEPTH_TYPE) POWER_TASK_STACK,      //任务栈大小，默认最小128，单位4字节
                    (void *) NULL,                                  //传递给任务的参数
                    (UBaseType_t) POWER_TASK_PRIORITY,              //任务的优先级
                    (TaskHandle_t *) &power_task_handle);           //任务句柄的地址

    /*2.创建飞行控制任务*/
    xTaskCreate((TaskFunction_t) FLIGHT_TASK,
                    "FLIGHT_TASK",
                    (configSTACK_DEPTH_TYPE) FLIGHT_TASK_STACK,
                    (void *) NULL,
                    (UBaseType_t) FLIGHT_TASK_PRIORITY,
                    (TaskHandle_t *) &flight_task_handle);
    /*3.创建LED灯控制任务*/
    xTaskCreate((TaskFunction_t) LED_TASK,
                    "LED_TASK",
                    (configSTACK_DEPTH_TYPE) LED_TASK_STACK,
                    (void *) NULL,
                    (UBaseType_t) LED_TASK_PRIORITY,
                    (TaskHandle_t *) &led_task_handle);
    /*4.创建nRF24L01的通讯任务*/
    xTaskCreate((TaskFunction_t) COM_TASK,
                    "COM_TASK",
                    (configSTACK_DEPTH_TYPE) COM_TASK_STACK,
                    (void *) NULL,
                    (UBaseType_t) COM_TASK_PRIORITY,
                    (TaskHandle_t *) &com_task_handle);
    /*删除任务*/
    vTaskDelete(NULL);
//退出临界区
    taskEXIT_CRITICAL();
}

/**@description:POWER_TASK:实现电源管理芯片每10s自启动一次
 *@param {void} *pvParameters
 * @return {*}
 */
void POWER_TASK(void * pvParameters) {

    TickType_t xLastWakeTime=xTaskGetTickCount();       //获取一次当前基准时间，用于绝对延时
    while (1) {
        //vTaskDelayUntil(&xLastWakeTime,POWER_TASK_PERIOD);          //每10s启动一次电源管理芯片，绝对时间
        /*等待任务通知，res=0时，等待超时执行启动电源操作*/
        uint32_t res=ulTaskNotifyTake(pdTRUE,POWER_TASK_PERIOD);
        if (res!=0) {
            /*收到通知执行关机操作*/
            TP4336_Stop();
        }
        else {
            /*等待超时，执行正常开机操作*/
            TP4336_Start();
        }

    }
}

/**
 * @brief MOTOR_TASK:在固定时间内启动一次控制电机速度
 * @param pvParameters
 */
void FLIGHT_TASK(void * pvParameters) {
    TickType_t xLastWakeTime=xTaskGetTickCount();
    //飞控任务初始化
    App_Flight_Init();
    while (1) {
        //uint32_t t1 = HAL_GetTick();
        /*1.获取三轴加速度和三轴角速度*/
        App_Flight_Get_Euler_Angle();
        /*2.根据当前欧拉角进行PID计算*/
        App_Flight_PID_Process();
        /*3.根据PID计算结果，控制电机*/
        App_Flight_Control_Motor();
        //uint32_t t2 = HAL_GetTick();

            //printf("Task Cost=%lu ms\r\n", t2 - t1);

        vTaskDelayUntil(&xLastWakeTime,FLIGHT_TASK_PERIOD);

    }
}

/**
 * @brief 在固定时间内启动控制LED灯的任务
 * @param pvParameters
 */
void LED_TASK(void * pvParameters) {
    TickType_t xLastWakeTime=xTaskGetTickCount();
    uint8_t count=0;    //用于判断时间
    //该任务执行的绝对时间为100ms一次，每进入一次循环count++,通过判断count的值来判断时间
    while (1) {
        //printf("LED Task Running\r\n");
        count++;
        /*1.前两个灯判断当前的连接状态*/
        if (remote_state==REMOTE_CONNECT) {
            //点亮前两个灯
            LED_Turn_On(&Left_0_LED);
            LED_Turn_On(&Right_0_LED);
        }
        else if (remote_state==REMOTE_DISCONNECT) {
            //关闭前两个灯
            LED_Turn_Off(&Left_0_LED);
            LED_Turn_Off(&Right_0_LED);
        }
        /*2.后两个灯用于判断该当前飞行状态*/
        if (flight_state==IDLE) {
            //后两个灯处于慢闪状态    500ms翻转一次
            if (count%5==0) {
                LED_Toggle(&Left_1_LED);
                LED_Toggle(&Right_1_LED);
            }
        }
        else if (flight_state==NORMAL) {
            //后两个灯处于快闪状态    200ms翻转一次
            if (count%2==0) {
                LED_Toggle(&Left_1_LED);
                LED_Toggle(&Right_1_LED);
            }
        }
        else if (flight_state==FIX_HEIGHT) {
            //后两个灯常亮
            LED_Turn_On(&Left_1_LED);
            LED_Turn_On(&Right_1_LED);
        }
        else if (flight_state==FAIL) {
            //后两个灯灭
            LED_Turn_Off(&Left_1_LED);
            LED_Turn_Off(&Right_1_LED);
        }
        /*10为2和5的最小公倍数，在最后对count进行判断，
                防止count计数溢出对时间判断造成干扰*/
        if (count==10) {

            count=0;

        }
        vTaskDelayUntil(&xLastWakeTime,LED_TASK_PERIOD);
    }
}
uint8_t Com_Data[TX_PLOAD_WIDTH+1]={0};
void COM_TASK(void * pvParameters) {
    TickType_t xLastWakeTime=xTaskGetTickCount();
    Bat_ADC_Init();
    while (1) {

        /*1.接收数据*/
        uint8_t res=App_Receive_Data();
        /*2.判断当前的连接状态*/
        App_Process_Connect_State(res);
        /*3.进行关机操作*/
        if (receive_data.shutdown==1) {
            //通知电源管理任务
            xTaskNotifyGive(power_task_handle);
        }
        /*4.处理飞机的飞行状态*/
        App_Process_Flight_State();
        /*5.回传电压*/
        float voltage=Bat_ADC_Read();
        //printf("Voltage: %.2f\r\n",voltage);
        sprintf(Back_Buff,"%.2f",voltage);
        vTaskDelayUntil(&xLastWakeTime,COM_TASK_PERIOD);
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   char *pcTaskName)
{
    printf("Stack Overflow: %s\r\n", pcTaskName);

    while(1);
}