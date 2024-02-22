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
    for(;;)
    {
        ADC_detect();
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
        osDelay(5);
    }
};

void agv_task(void *argument){
    TickType_t PrTime2 = xTaskGetTickCount();
    agv_init();
    for (;;)
    {
        read_agv_data();
        SPL(8,x1_value,y1_value,1,x2_value,y2_value);
        vTaskDelayUntil(&PrTime2, pdMS_TO_TICKS(50));  // 延迟50豪秒
//        usart_printf("%f %f %f %f %f %f %f %f\r\n",y2_value[0],y2_value[1], y2_value[2],
//                     y2_value[3],y2_value[4], y2_value[5], y2_value[6], y2_value[7]);
    }
};

