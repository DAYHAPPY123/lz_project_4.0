#include "remote.h"
#include "usart.h"
#include "stm32g4xx_hal.h"
#include "cmsis_os2.h"

RC_ctrl_t rc_ctrl;
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

void REMOTEC_Init(void){
    HAL_UART_Receive_DMA(&huart1, sbus_rx_buf[0], SBUS_RX_BUF_NUM);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

void REMOTEC_UartIrqHandler(void)
{
    static uint16_t this_time_rx_len = 0;
if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
        this_time_rx_len = SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        ((&hdma_usart1_rx)->Instance->CNDTR = (uint16_t)(SBUS_RX_BUF_NUM));
        if(this_time_rx_len == RC_FRAME_LENGTH)
        {
            sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
        }
        __HAL_DMA_ENABLE(&hdma_usart1_rx);
    }
}

void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)//遥控器数据处理
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3

    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);               //!< Switch right
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;              //!< Switch left

    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] = -rc_ctrl->rc.ch[4];
}
