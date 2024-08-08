//
// Created by 14298 on 2024/8/8.
//

#ifndef LZ_PROJECT_4_0_DEBUGC_H
#define LZ_PROJECT_4_0_DEBUGC_H
#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"
#include "usart.h"

#define DEBUG_RVSIZE 255

//#define p 0x70
//#define s 0x73
//#define i 0x69
//#define d 0x64
//#define o 0x6F
//#define n 0x6E
#define maohao 0x3A

    class temp_pid
    {
    public:
        cPID* PID=&PID3_1;
        float p_kp=0;
        float p_ki=0;
        float p_kd=0;
        float s_kp=0;
        float s_ki=0;
        float s_kd=0;
    };

void DEBUGC_UartInit();
void DEBUGC_UartIrqHandler(UART_HandleTypeDef *huart);
void DEBUGC_UartIdleCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_4_0_DEBUGC_H
