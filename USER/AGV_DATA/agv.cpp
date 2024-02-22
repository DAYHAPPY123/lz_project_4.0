#include "agv.h"
#include "usart.h"
#include "string.h"
#include "main.h"
#include "motor.h"
#include "remote.h"
#include "trace.h"

float agv_buffer[8];
uint8_t agvRvBuff[AGV_RVSIZE]={0};
uint8_t agvBuff[AGV_RVSIZE]={0};

uint8_t agv_right[4];
uint8_t agv_left[4];
float turn_angle;

uint8_t send_cmd[] = {0x01, 0x03, 0x00, 0x20, 0x00, 0x08, 0x45, 0xC6};

uint8_t find_max(float arry[], uint8_t length)
{
    uint8_t max_index = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        if (arry[i] > arry[max_index])
            max_index = i;
    }
    return max_index;
}

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
        agv_buffer[0] = (float)agvBuff[1];
        agv_buffer[1] = (float)agvBuff[0];
        agv_buffer[2] = (float)agvBuff[3];
        agv_buffer[3] = (float)agvBuff[2];
        agv_buffer[4] = (float)agvBuff[5];
        agv_buffer[5] = (float)agvBuff[4];
        agv_buffer[6] = (float)agvBuff[7];
        agv_buffer[7] = (float)agvBuff[6];
    }
for (int i=0;i<=7;i++)
    {
        y1_value[i]=agv_buffer[i];
    }

    uint8_t origin_max_index = 0;
    uint8_t fit_max_index = 0;
    origin_max_index = find_max(agv_buffer, 8);

    if (agv_buffer[origin_max_index] > 10.0f)// 磁导航传感器读值必须大于某个值
    {
        if (origin_max_index != 0 && origin_max_index != 7)// 如果最大值索引不是0或者7 以最大值和最大值的左右值共三个值拟合曲线 寻找极值
        {
            float fit_origin_data_y[3] = {agv_buffer[origin_max_index - 1], agv_buffer[origin_max_index], agv_buffer[origin_max_index + 1]};
//            float fit_origin_data_y[3] ={30,35,33};
            float fit_origin_data_x[3] = {0, 1, 2};
            static float fit_origin_data_u[21]={0};
            static float fit_origin_data_s[21]={0};

            for (int i = 0; i < 21; i++)
            {
                fit_origin_data_u[i] = (float)i * 0.1f;
            }

            SPL(3, fit_origin_data_x, fit_origin_data_y, 21, fit_origin_data_u, fit_origin_data_s);

//             for (int i = 0; i < 21; i++)
//             {
//                 usart_printf("%.2f\r\n", fit_origin_data_s[i]);
//             }

            fit_max_index = find_max(fit_origin_data_s, 21);
            usart_printf("%d\r\n",fit_max_index);
//            turn_angle = -35 + 1 * (origin_max_index * 10 - 10 + fit_max_index);

            turn_angle = (float )(fit_max_index-10)*100;
        }
        else
        {
            fit_max_index = 0;
            turn_angle = -35 + 10 * origin_max_index;
            turn_angle=0;
        }
    }
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

