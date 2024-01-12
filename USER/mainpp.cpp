#include "mainpp.h"
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "adc.h"
#include "fdcan.h"
#include "tim.h"
#include "remote.h"
#include "mycan.h"
#include "motor.h"
#include "gpio.h"
#include "adc.h"
#include "string"
#include "servos.h"
#include "agv.h"

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
//    DEBUGC_UartInit();
    CAN_Init(&hfdcan2);
    REMOTEC_Init();
    motor_io_init();
    servos_init();
    agv_init();
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,800);//激光
}

void BSP_Init()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_LPUART1_UART_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_ADC4_Init();
    MX_FDCAN2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
}