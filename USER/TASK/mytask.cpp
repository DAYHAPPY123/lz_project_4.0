#include "mytask.h"
#include "cmsis_os2.h"
#include "tim.h"
#include "motor.h"
#include "agv.h"
#include "servos.h"
#include "trace.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "usart.h"
#include "pid.h"
#include "mainpp.h"
#include "button.h"

void init_task(void *argument){
    TickType_t PrTime0 = xTaskGetTickCount();
    light_init();
    for(;;)
    {
        mode_choose();
        state_control();
        vTaskDelayUntil(&PrTime0, pdMS_TO_TICKS(5));  // 延迟5豪秒
    }
};

void control_task(void *argument){
    TickType_t PrTime1 = xTaskGetTickCount();
    motor_reset();
    servos_reset();
    for(;;)
    {
        angle_cal();
        backwheel_speed_cal();
        Speed_Send();
        servos_control();
        vTaskDelayUntil(&PrTime1, pdMS_TO_TICKS(5));  // 延迟5豪秒
    }
};

void agv_task(void *argument){
    TickType_t PrTime2 = xTaskGetTickCount();
    agv_init();
    for (;;)
    {
        read_agv_data();
        vTaskDelayUntil(&PrTime2, pdMS_TO_TICKS(10));  // 延迟50豪秒
    }
};

void light_task(void *argument){
    for (;;)
    {
//        if(mode==MOTOR_MANUAL)
//        {
//            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
//        }
//
//        if(mode==MOTOR_AUTO)
//        {
//            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
//            osDelay(500);
//            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
//            osDelay(500);
//        }
//
//        if(mode==MOTOR_STOP)
//        {
//            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
//        }
        osDelay(50);
    }
};