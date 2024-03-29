#include "button.h"
#include "motor.h"
#include "servos.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "remote.h"
#include "cmsis_os2.h"
#include "tim.h"
//auto_speed=7.265时后轮速度在35mm/s,auto_speed=10.378时后轮速度在50mm/s
//angle_limit=50时角度为小范围，即下压力小,angle_limit=60时角度为大范围，即下压力大
void mode_choose()
{
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)==1)//舵机范围选择50/60
    {
        angle_limit=50;
    }
    else
    {
        angle_limit=60;
    }
//    usart_printf("%d \r\n",HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8));
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)==1)//循迹速度范围选择35/50
    {
        auto_speed=0;
    }
    else
    {
        auto_speed=10.378;
    }
}

