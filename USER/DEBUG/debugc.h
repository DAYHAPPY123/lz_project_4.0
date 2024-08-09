//
// Created by 14298 on 2024/8/8.
//

#ifndef LZ_PROJECT_4_0_DEBUGC_H
#define LZ_PROJECT_4_0_DEBUGC_H
#ifdef __cplusplus
extern "C" {
#endif

//#include "pid.h"
#include "usart.h"

#define DEBUG_RVSIZE 255

//#define p 0x70
//#define s 0x73
//#define i 0x69
//#define d 0x64
//#define o 0x6F
//#define n 0x6E
#define maohao 0x3A

    struct temp_pid
    {
        float p_kp;
        float p_ki;
        float p_kd;
        float s_kp;
        float s_ki;
        float s_kd;
    };

void DEBUGC_UartInit();
void DEBUGC_UartIrqHandler(UART_HandleTypeDef *huart);
void DEBUGC_UartIdleCallback(UART_HandleTypeDef *huart);

extern struct temp_pid temp;
#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_4_0_DEBUGC_H
