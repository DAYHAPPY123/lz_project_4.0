#include "agv.h"
#include "usart.h"
#include "string.h"
#include "main.h"
#include "motor.h"
#include "remote.h"

uint8_t agv_buffer[8];
uint8_t agvRvBuff[AGV_RVSIZE]={0};
uint8_t agvBuff[AGV_RVSIZE]={0};

uint8_t agv_right[4];
uint8_t agv_left[4];
float turn_angle;

uint8_t send_cmd[] = {0x01, 0x03, 0x00, 0x20, 0x00, 0x08, 0x45, 0xC6};

void agv_init()
{
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)agvRvBuff, AGV_RVSIZE);
}

void read_agv_data()
{
    HAL_UART_Transmit(&huart2, send_cmd, 8, 10);
    memcpy(agvBuff,&agvRvBuff[3],8);
    if( (agvRvBuff[0] == 0x01) && (agvRvBuff[1] == 0x03) && (agvRvBuff[2]== 0x10) )
    {
        agv_buffer[0] = agvBuff[1];
        agv_buffer[1] = agvBuff[0];
        agv_buffer[2] = agvBuff[3];
        agv_buffer[3] = agvBuff[2];
        agv_buffer[4] = agvBuff[5];
        agv_buffer[5] = agvBuff[4];
        agv_buffer[6] = agvBuff[7];
        agv_buffer[7] = agvBuff[6];
    }
//    for(int i=0;i<4;i++)
//    {
//        agv_left[i]=agv_buffer[3-i];
//        agv_right[i]=agv_buffer[i+4];
//    }
//    usart_printf("%d %d %d %d %d %d %d %d\r\n",agv_buffer[0],agv_buffer[1], agv_buffer[2],
//                 agv_buffer[3],agv_buffer[4], agv_buffer[5], agv_buffer[6], agv_buffer[7]);
}

void state_control()//由通道值计算出转向幅度
{
    switch (rc_ctrl.rc.s[1])
    {
        case 1:mode=MOTOR_AUTO;break;//上
        case 3:mode=MOTOR_MANUAL;break;//中
        case 2:mode=MOTOR_STOP;break;//下
    }
//    if (mode==MOTOR_AUTO)
//    {
//        if ( (agv_right[3]+agv_left[3]) >=50)
//        {
//            mode=MOTOR_STOP;
//        }
//    }
}

