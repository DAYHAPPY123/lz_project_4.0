#include <cstdlib>
#include "remote.h"
#include "usart.h"
#include "stm32g4xx_hal.h"
#include "motor.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"

uint32_t last_rc_receive_time = 0; // 上一次接收到遥控器信号的时间戳
uint32_t rc_timeout = 1000; // 超时阈值，单位为毫秒
RC_ctrl_t rc_ctrl;
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
 * 遥控器有bug 如果机器人上电后未启动遥控器，实测读取的通道值混乱，电机要疯转，DJI遥控器未出现该问题，目前原因未知
 */
void REMOTE_detect()
{
    while( (rc_ctrl.ch[4] > -600)||(rc_ctrl.ch[5] > -600) )
    {
        HAL_Delay(5);
    }
    while( (rc_ctrl.ch[4] < 600)||(rc_ctrl.ch[5] < 600) )
    {
        HAL_Delay(5);
    }
    while( (rc_ctrl.ch[4] > -600)||(rc_ctrl.ch[5] > -600) )
    {
        HAL_Delay(5);
    }
}

void REMOTEC_Init(void){
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, sbus_rx_buf[0], SBUS_RX_BUF_NUM);
}

void REMOTEC_UartIrqHandler(void)
{
    static uint16_t this_time_rx_len = 0;
if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_DMAStop(&hlpuart1);
        this_time_rx_len = SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        if(this_time_rx_len == RC_FRAME_LENGTH)
        {
            sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
        }
        __HAL_DMA_ENABLE(&hdma_usart1_rx);
    }
    HAL_UART_Receive_DMA(&huart1, sbus_rx_buf[0], SBUS_RX_BUF_NUM);
    last_rc_receive_time=osKernelGetTickCount();
}

void check_rc_connection()
{
    uint32_t current_time = osKernelGetTickCount();
    uint32_t time_gap = current_time - last_rc_receive_time;
    if (time_gap >= rc_timeout)
    {
        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
    }

    usart_printf("%d  %d \r\n",current_time,last_rc_receive_time);
}

void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)////转换后每通道理论量程为（-720，+720）
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->ch[0] = ((int16_t)sbus_buf[1] >> 0 | ((int16_t)sbus_buf[2] << 8 )) & 0x07FF;
    rc_ctrl->ch[1] = ((int16_t)sbus_buf[2] >> 3 | ((int16_t)sbus_buf[3] << 5 )) & 0x07FF;
    rc_ctrl->ch[2] = ((int16_t)sbus_buf[3] >> 6 | ((int16_t)sbus_buf[4] << 2 )
            |(int16_t)sbus_buf[5] << 10 ) & 0x07FF;//右拨杆竖直
    rc_ctrl->ch[3] = ((int16_t)sbus_buf[5] >> 1 | ((int16_t)sbus_buf[6] << 7 )) & 0x07FF;
    rc_ctrl->ch[4] = ((int16_t)sbus_buf[6] >> 4 | ((int16_t)sbus_buf[7] << 4 )) & 0x07FF;
    rc_ctrl->ch[5] = ((int16_t)sbus_buf[7] >> 7 | ((int16_t)sbus_buf[8] << 1 )
            |(int16_t)sbus_buf[9] <<  9 ) & 0x07FF;//SB(别骂了
    rc_ctrl->ch[6] = ((int16_t)sbus_buf[9] >> 2 | ((int16_t)sbus_buf[10] << 6 )) & 0x07FF;
    rc_ctrl->ch[7] = ((int16_t)sbus_buf[10] >> 5 | ((int16_t)sbus_buf[11] << 3 )) & 0x07FF;
    rc_ctrl->ch[8] = ((int16_t)sbus_buf[12] << 0 | ((int16_t)sbus_buf[13] << 8 )) & 0x07FF;
    rc_ctrl->ch[9] = ((int16_t)sbus_buf[13] >> 3 | ((int16_t)sbus_buf[14] << 5 )) & 0x07FF;
    rc_ctrl->ch[10] = ((int16_t)sbus_buf[14] >> 6 | ((int16_t)sbus_buf[15] << 2 )  | (int16_t)sbus_buf[16] << 10 ) & 0x07FF;
    rc_ctrl->ch[11] = ((int16_t)sbus_buf[16] >> 1 | ((int16_t)sbus_buf[17] << 7 )) & 0x07FF;
    for(int i=0;i<=11;i++)
    {
        rc_ctrl->ch[i]-=1002;
    }
    rc_ctrl->ch[11] = -rc_ctrl->ch[11];
    rc_ctrl->ch[5] = -rc_ctrl->ch[5];

    rc_ctrl->ch[1] = -rc_ctrl->ch[1];
    rc_ctrl->ch[2] = -rc_ctrl->ch[2];
//    usart_printf("%d %d %d %d %d %d %d %d %d %d %d %d\r\n",
//                 rc_ctrl->ch[0],rc_ctrl->ch[1],rc_ctrl->ch[2],rc_ctrl->ch[3],
//                 rc_ctrl->ch[4],rc_ctrl->ch[5],rc_ctrl->ch[6],rc_ctrl->ch[7],
//                 rc_ctrl->ch[8],rc_ctrl->ch[9],rc_ctrl->ch[10],rc_ctrl->ch[11]);
// 以下为WBUS协议剩余内容，遥控器未使用
//    rc_ctrl->ch[12] = ((int16_t)sbus_buf[17] >> 4 | ((int16_t)sbus_buf[18] << 4 )) & 0x07FF;
//    rc_ctrl->ch[13] = ((int16_t)sbus_buf[18] >> 7 | ((int16_t)sbus_buf[19] << 1 )  | (int16_t)sbus_buf[20] <<  9 ) & 0x07FF;
//    rc_ctrl->ch[14] = ((int16_t)sbus_buf[20] >> 2 | ((int16_t)sbus_buf[21] << 6 )) & 0x07FF;
//    rc_ctrl->ch[15] = ((int16_t)sbus_buf[21] >> 5 | ((int16_t)sbus_buf[22] << 3 )) & 0x07FF;

}
