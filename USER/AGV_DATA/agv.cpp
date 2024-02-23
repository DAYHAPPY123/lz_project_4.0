#include "agv.h"
#include "usart.h"
#include "string.h"
#include "main.h"
#include "motor.h"
#include "remote.h"
#include "trace.h"
#include "pid.h"

float agv_buffer[8];
uint8_t agvRvBuff[AGV_RVSIZE]={0};
uint8_t agvBuff[AGV_RVSIZE]={0};
static uint8_t origin_max_index = 0;

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
//    usart_printf("%f %f %f %f %f %f %f %f\r\n",agv_buffer[0],agv_buffer[1], agv_buffer[2],
//                 agv_buffer[3],agv_buffer[4], agv_buffer[5], agv_buffer[6], agv_buffer[7]);


    static uint8_t fit_max_index = 0;
    origin_max_index = find_max(agv_buffer, 8);

    usart_printf("%f\r\n",agv_buffer[origin_max_index]);
    if (mode == MOTOR_AUTO )
{
    if (agv_buffer[origin_max_index] > 50.0f)// 磁导航传感器读值必须大于某个值
    {
        if (origin_max_index != 0 && origin_max_index != 7)// 如果最大值索引不是0或者7 以最大值和最大值的左右值共三个值拟合曲线，寻找极值
        {
            float fit_origin_data_y[3] = {agv_buffer[origin_max_index - 1], agv_buffer[origin_max_index], agv_buffer[origin_max_index + 1]};
            float fit_origin_data_x[3] = {0, 1, 2};
            static float fit_origin_data_u[21]={0};            //20等分，所以数组有21个成员
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
//            usart_printf("%d\r\n",fit_max_index);
//假设超过实际最大值横坐标与3.5插值超过1就打满转向，而手动转向最大值8191.0*3.0/4.0/2*0.7=2150.1375，故设置阈值为2000
            turn_angle=(float)((3.5-(origin_max_index+(fit_max_index-10)*0.1))*2000.0*2.5);
            limit(&turn_angle,2000);
        }
        else
        {
            turn_angle=(float)((4.5-origin_max_index)*2000.0);
            limit(&turn_angle,2000);
        }
    }

}

}

void state_control()//由通道值计算出转向幅度
{
    mode=MOTOR_MANUAL;
    switch (rc_ctrl.rc.s[1])
    {
        case 1:mode=MOTOR_AUTO;break;//上
        case 3:mode=MOTOR_MANUAL;break;//中
        case 2:mode=MOTOR_STOP;break;//下
    }
    if ( (agv_buffer[origin_max_index] < 50.0f) && (mode==MOTOR_AUTO) )
    {
        mode = MOTOR_STOP;
    }
}

