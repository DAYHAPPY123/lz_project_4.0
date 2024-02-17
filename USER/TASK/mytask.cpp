#include "mytask.h"
#include "cmsis_os2.h"
#include "tim.h"
#include "motor.h"
#include "agv.h"
#include "servos.h"
#include "agv_trace.h"
#include "trace.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "usart.h"

void init_task(void *argument){
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    for(;;)
    {
        ADC_detect();
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,100);//激光
        osDelay(50);
    }
};

void control_task(void *argument){
    TickType_t PrTime1 = xTaskGetTickCount();
    motor_reset();
    servos_reset();
    for(;;)
    {
        state_control();
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
//        agv_calculate();
//        SPL(8,x1_value,y1_value,10,x2_value,y2_value);
        vTaskDelayUntil(&PrTime2, pdMS_TO_TICKS(100));  // 延迟50豪秒
    }
};

