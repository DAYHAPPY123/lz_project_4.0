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

void init_task(void *argument){
    for(;;)
    {

//            usart_printf("%d,%.2f,%d,%.2f  \r\n",motor2_1.set_pos,motor2_1.calculate_continuous,
//                 motor2_2.set_pos,motor2_2.calculate_continuous);
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

