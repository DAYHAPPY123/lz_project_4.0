#include "task.h"
#include "cmsis_os2.h"
#include "tim.h"
#include "motor.h"
#include "agv.h"
#include "usart.h"
#include "adc.h"
#include "servos.h"
uint32_t adc_value[2];

void Init_Task(void *argument){
    for(;;)
    {
        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7);//LED
        osDelay(1000);
    }
};

void control_task(void *argument){
    for(;;)
    {
        state_control();
        angle_cal();
        backwheel_speed_cal();
        Speed_Send();
        servos_control();
        osDelay(5);
    }
};

void printf_task(void *argument){
    for(;;)
    {
        osDelay(20);
    }
};

void agv_task(void *argument){
//    TickType_t current_time = xTaskGetTickCoun t();
    for (;;)
    {
        read_agv_data();
        osDelay(100);
//        vTaskDelayUntil(&current_time, 100);
    }
};

void adc_task(void *argument){
    for(;;)
    {
        HAL_ADC_Start_DMA(&hadc4, &adc_value[0], 1);
        HAL_ADC_Start_DMA(&hadc1, &adc_value[1], 1);
        if( (adc_value[0]>4000)||(adc_value[1]>4000) )
        {
            mode=MOTOR_STOP;
            servos_init();
        }
        usart_printf("%d %d \n",adc_value[0],adc_value[1]);
        osDelay(20);
    }
}
