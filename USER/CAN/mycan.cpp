#include "mycan.h"
#include "fdcan.h"
#include "motor.h"

void CAN_Init(FDCAN_HandleTypeDef *hfdcan)
{
    assert_param(hfdcan != NULL);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
    CAN_FilterInit();
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    if (hfdcan == &hfdcan2)
    {
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header,rx_data );
            int16_t ID = rx_header.Identifier;
            if (ID == CAN_3508_M1_ID) {
                motor3_1.pos = (int16_t) (rx_data[0] << 8 | rx_data[1]);
                motor3_1.vel = (int16_t) (rx_data[2] << 8 | rx_data[3]);
                motor3_1.current = (int16_t) ((rx_data)[4] << 8 | rx_data[5]);
                motor3_1.tmp = (int16_t) rx_data[6];
                motor3_1.rpm=motor3_1.vel/100.0;
            }

            if (ID == CAN_3508_M2_ID) {
                motor3_2.pos = (int16_t) (rx_data[0] << 8 | rx_data[1]);
                motor3_2.vel = (int16_t) (rx_data[2] << 8 | rx_data[3]);
                motor3_2.current = (int16_t) ((rx_data)[4] << 8 | rx_data[5]);
                motor3_2.tmp = (int16_t) rx_data[6];
                motor3_2.rpm=motor3_2.vel/100.0;
            }

            if (ID == CAN_6020_M1_ID) {
                static int16_t last_mechanical_angle_1 = 0;
                static int16_t mechanical_angle_change_1 = 0;
                motor6_1.pos = (int16_t) (rx_data[0] << 8 | rx_data[1]);
                motor6_1.vel = (int16_t) (rx_data[2] << 8 | rx_data[3]);
                motor6_1.volt = (int16_t) ((rx_data)[4] << 8 | rx_data[5]);
                motor6_1.tmp = (int16_t) rx_data[6];
                motor6_1.rpm=motor6_1.vel;

                mechanical_angle_change_1 = motor6_1.pos - last_mechanical_angle_1 ;
                mechanical_angle_change_1 = mechanical_angle_change_1 >= 8000 ? mechanical_angle_change_1 - 8192
                                                                              : mechanical_angle_change_1 <= -8000 ? mechanical_angle_change_1 + 8192
                                                                                                                   : mechanical_angle_change_1;
                motor6_1.continuous += mechanical_angle_change_1;
                last_mechanical_angle_1 = motor6_1.pos;
            }

            if (ID == CAN_6020_M2_ID) {
                static int16_t last_mechanical_angle_2 = 0;
                static int16_t mechanical_angle_change_2 = 0;
                motor6_2.pos = (int16_t) (rx_data[0] << 8 | rx_data[1]);
                motor6_2.vel = (int16_t) (rx_data[2] << 8 | rx_data[3]);
                motor6_2.volt = (int16_t) ((rx_data)[4] << 8 | rx_data[5]);
                motor6_2.tmp = (int16_t) rx_data[6];
                motor6_2.rpm=motor6_2.vel;

                mechanical_angle_change_2 = motor6_2.pos - last_mechanical_angle_2 ;
                mechanical_angle_change_2 = mechanical_angle_change_2 >= 8000 ? mechanical_angle_change_2 - 8192
                                                                              : mechanical_angle_change_2 <= -8000 ? mechanical_angle_change_2 + 8192
                                                                                                                   : mechanical_angle_change_2;
                motor6_2.continuous += last_mechanical_angle_2 ;
                last_mechanical_angle_2 = motor6_2.pos;
            }
        }
    }
}

void CAN_FilterInit()
{
    FDCAN_FilterTypeDef fdcan_filter_st;
    fdcan_filter_st.IdType = FDCAN_STANDARD_ID;
    fdcan_filter_st.FilterIndex = 0; // 选择要配置的过滤器编号
    fdcan_filter_st.FilterType = FDCAN_FILTER_MASK; // 设置过滤类型为掩码模式
    fdcan_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 将过滤器结果传输到 RX FIFO 0
    fdcan_filter_st.FilterID1 = 0x0; // 设置标识符1，低16位
    fdcan_filter_st.FilterID2 = 0x0; // 设置标识符2，高16位
    HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter_st); // 将过滤器应用到 FDCAN2
    HAL_FDCAN_Start(&hfdcan2); // 启动 FDCAN2
}


void PortSendMotorsCur_3508(int16_t cur0, int16_t cur1, int16_t cur2, int16_t cur3)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    tx_header.Identifier  = CAN_CHASSIS_Drive_ID;
    tx_header.IdType  = FDCAN_STANDARD_ID;
    tx_header.TxFrameType  = FDCAN_DATA_FRAME;
    tx_header.DataLength  = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;           // 关闭速率切换
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;            // 传统的CAN模式
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 无发送事件
    tx_header.MessageMarker = 0;
    tx_data[0] = (cur0 >> 8);
    tx_data[1] = cur0;
    tx_data[2] = (cur1 >> 8);
    tx_data[3] = cur1;
    tx_data[4] = (cur2 >> 8);
    tx_data[5] = cur2;
    tx_data[6] = (cur3 >> 8);
    tx_data[7] = cur3;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, tx_data);
}

void PortSendMotorsCur_6020(int16_t volt0, int16_t volt1, int16_t volt2, int16_t volt3)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    tx_header.Identifier  = CAN_CHASSIS_Turn_ID;
    tx_header.IdType  = FDCAN_STANDARD_ID;
    tx_header.TxFrameType  = FDCAN_DATA_FRAME;
    tx_header.DataLength  = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;           // 关闭速率切换
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;            // 传统的CAN模式
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 无发送事件
    tx_header.MessageMarker = 0;
    tx_data[0] = (volt0 >> 8);
    tx_data[1] = volt0;
    tx_data[2] = (volt1 >> 8);
    tx_data[3] = volt1;
    tx_data[4] = (volt2 >> 8);
    tx_data[5] = volt2;
    tx_data[6] = (volt3 >> 8);
    tx_data[7] = volt3;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, tx_data);
}