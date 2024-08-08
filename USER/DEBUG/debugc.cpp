//
// Created by 14298 on 2024/8/8.
//

#include <cstring>
#include <cstdlib>
#include "debugc.h"
#include "usart.h"
#include "motor.h"
#include "pid.h"

char debugRvBuff[DEBUG_RVSIZE]={0};
char debugBuff[DEBUG_RVSIZE]={0};
temp_pid temp;

void DEBUGC_UartInit()
{
    __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&hlpuart1, (uint8_t*)debugRvBuff, DEBUG_RVSIZE);
}

void DEBUGC_UartIrqHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == hlpuart1.Instance)
    {
        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);
            DEBUGC_UartIdleCallback(huart);
        }
    }
}

void DEBUGC_UartIdleCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_DMAStop(huart);
    memcpy(debugBuff ,&debugRvBuff[5], 10);
    uint8_t data_length  = DEBUG_RVSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if( (debugRvBuff[0] == 0x6F) && (debugRvBuff[1] == 0x6E) && (debugRvBuff[2]== maohao) )
    {
        if(debugRvBuff[3] == 0x30)//0
        mode = MOTOR_STOP;
        else if(debugRvBuff[3] == 0x31)//1
        mode = MOTOR_MANUAL;
    }

    switch (debugRvBuff[0])
    {
        case 0x70://p
        {
            switch (debugRvBuff[3])
            {
                case 0x70:temp.p_kp=strtof(debugBuff,NULL);break;//p
                case 0x69:temp.p_ki=strtof(debugBuff,NULL);break;//i
                case 0x64:temp.p_kd=strtof(debugBuff,NULL);break;//d
            }
            break;
        }
        case 0x73://s
        {
            switch (debugRvBuff[3])
            {
                case 0x70:temp.s_kp=strtof(debugBuff,NULL);break;//p
                case 0x69:temp.s_ki=strtof(debugBuff,NULL);break;//i
                case 0x64:temp.s_kd=strtof(debugBuff,NULL);break;//d
            }
            break;
        }
    }
    temp.PID->Pos_Param_set(temp.p_kp,temp.p_ki,temp.p_kd);
    temp.PID->Spd_Param_set(temp.s_kp,temp.s_ki,temp.s_kd);
    memset(debugRvBuff,0,data_length);
    data_length = 0;
    HAL_UART_Receive_DMA(huart, (uint8_t*)debugRvBuff, DEBUG_RVSIZE);
}