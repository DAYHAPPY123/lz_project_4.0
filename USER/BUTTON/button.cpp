#include "button.h"
#include "motor.h"
#include "servos.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"

//auto_speed=7.265时后轮速度在35mm/s
//auto_speed=10.378时后轮速度在50mm/s
//angle_limit=50时角度为小范围，即下压力小
//angle_limit=60时角度为大范围，即下压力大

void mode_choose()
{
    taskENTER_CRITICAL();
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)==0)//舵机范围选择50/60
    {
        angle_limit=50;
    }
    else
    {
        angle_limit=60;
    }

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)==0)//循迹速度范围选择
    {
        auto_speed=7.265;
    }
    else
    {
        auto_speed=10.378;
    }
    taskEXIT_CRITICAL();
}