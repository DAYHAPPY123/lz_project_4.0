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
#include "remote.h"
#include "mycan.h"

void init_task(void *argument){
//    light_init();
    for(;;)
    {
//        mode_choose();
//        light_mode();
        osDelay(5);
    }
}
void control_task(void *argument){
    TickType_t PrTime1 = xTaskGetTickCount();
//    motor_reset();
//    servos_reset();
    for(;;)
    {
//        state_control();
//        angle_cal();
//        backwheel_speed_cal();
//        Speed_Send();
//        servos_control();
        vTaskDelayUntil(&PrTime1, pdMS_TO_TICKS(5));  // 延迟5豪秒
    }
}
void agv_task(void *argument){
    TickType_t PrTime2 = xTaskGetTickCount();
//    agv_init();
    for (;;)
    {
//        read_agv_data();
        vTaskDelayUntil(&PrTime2, pdMS_TO_TICKS(10));  // 延迟10豪秒
    }
}
void error_task(void *argument){
    TickType_t PrTime3 = xTaskGetTickCount();
    for (;;)
    {
//        check_rc_connection();
        usart_printf("%d %d %d %d\r\n",rc_ctrl.rc.ch[0],rc_ctrl.rc.ch[1],rc_ctrl.rc.ch[2],rc_ctrl.rc.ch[3]);
        vTaskDelayUntil(&PrTime3, pdMS_TO_TICKS(100));
    }
}

