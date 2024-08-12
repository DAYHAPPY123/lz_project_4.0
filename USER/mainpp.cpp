#include "mainpp.h"
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "fdcan.h"
#include "tim.h"
#include "remote.h"
#include "mycan.h"
#include "motor.h"
#include "gpio.h"
#include "string"
#include "servos.h"
#include "agv.h"
#include "debugc.h"

int main()
{
    BSP_Init();
    User_Init();
    osKernelInitialize();
    MX_FREERTOS_Init();
    osKernelStart();
    while (1)
    {
        HAL_Delay(5);
    }
}

void User_Init()
{
    CAN_Init(&hfdcan2);
    REMOTEC_Init();
    motor_enable();
    servos_init();
    DEBUGC_UartInit();}

void BSP_Init()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_LPUART1_UART_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_FDCAN2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
}

void light_init()//counter_max=2000
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);//指示灯
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
}