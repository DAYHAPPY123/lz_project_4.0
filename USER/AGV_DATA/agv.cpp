#include "agv.h"
#include "usart.h"
#include "string.h"
#include "main.h"
#include "motor.h"
#include "remote.h"
#include "trace.h"
#include "pid.h"
#include "tim.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

int noise = 0;
float min_agv_start=20.0f;
double offset_distance=0;
float agv_buffer[8];
uint8_t agvRvBuff[AGV_RVSIZE]={0};
uint8_t agvBuff[AGV_RVSIZE]={0};
static uint8_t origin_max_index = 0;
int A_click = 0;//控制set_rpm;
int B_click = 0;//控制set_rpm;
float V_AB=0;

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
        agv_buffer[0] = (float)agvBuff[1]-noise;
        agv_buffer[1] = (float)agvBuff[0]-noise;
        agv_buffer[2] = (float)agvBuff[3]-noise;
        agv_buffer[3] = (float)agvBuff[2]-noise;
        agv_buffer[4] = (float)agvBuff[5]-noise;
        agv_buffer[5] = (float)agvBuff[4]-noise;
        agv_buffer[6] = (float)agvBuff[7]-noise;
        agv_buffer[7] = (float)agvBuff[6]-noise;
    }
    usart_printf("%.0f %.0f %.0f %.0f %.0f %.0f %.0f %.0f\r\n",agv_buffer[0],agv_buffer[1], agv_buffer[2],
                 agv_buffer[3],agv_buffer[4], agv_buffer[5], agv_buffer[6], agv_buffer[7]);

    static uint8_t fit_max_index = 0;
    origin_max_index = find_max(agv_buffer, 8);

//    usart_printf("%f\r\n",agv_buffer[origin_max_index]);
    if (mode == MOTOR_AUTO )
{
    if (agv_buffer[origin_max_index] >= min_agv_start)// 磁导航传感器读值必须大于某个值
    {
        if (origin_max_index != 0 && origin_max_index != 7)// 如果最大值索引不是0或者7 以最大值和最大值的左右值共三个值拟合曲线，寻找极值
        {
            float fit_origin_data_y[3] = {agv_buffer[origin_max_index - 1], agv_buffer[origin_max_index], agv_buffer[origin_max_index + 1]};
            float fit_origin_data_x[3] = {0, 1, 2};
            static float fit_origin_data_u[41]={0};            //40等分，所以数组有41个成员
            static float fit_origin_data_s[41]={0};
            for (int i = 0; i < 41; i++)
            {
                fit_origin_data_u[i] = (float)i * 0.05f;
            }
            SPL(3,  fit_origin_data_x, fit_origin_data_y, 41, fit_origin_data_u, fit_origin_data_s);

            fit_max_index = find_max(fit_origin_data_s, 41);
//            usart_printf("%d\r\n",fit_max_index);
//假设超过实际最大值横坐标与3.5插值超过0.5就打满转向，而手动转向最大值8191.0*3.0/4.0/2*0.7=2150.1375，故设置阈值为2000
            turn_angle=(float)((3.5-(origin_max_index+(fit_max_index-20)*0.05))*2000.0*1.5);//分辨率300
            offset_distance=(3.5-(origin_max_index+(fit_max_index-20.0)*0.05))*10.0;
//            usart_printf("%.3f\r\n",offset_distance);
            limit(&turn_angle,2150);
        }
        else
        {
            turn_angle=(float)((3.5-origin_max_index)*2000.0*1.5);
            limit(&turn_angle,2000);
        }
    }
}
}

void state_control()//模式控制
{
    taskENTER_CRITICAL();
    mode=MOTOR_STOP;
    if(rc_ctrl.ch[4]>600) mode=MOTOR_AUTO;
    else if(rc_ctrl.ch[4]< -100) mode=MOTOR_STOP;//实测遥控器断连时rc_ctrl.ch[4]突变为-320，正好也用作断连异常处理代码
    else mode=MOTOR_MANUAL;
    if ( (agv_buffer[origin_max_index] < min_agv_start) && (mode==MOTOR_AUTO) )    //若传感器最大读取值小于20.0，则机器人恢复至停止模式
    {
        mode = MOTOR_STOP;
    }
    /**
     * 控制set.rpm的系数
     */

    if (rc_ctrl.ch[6] > 600)
    {
        A_click = 1;
    }else
    {
        A_click = 0;
    }
    if (rc_ctrl.ch[7] > 600)
    {
        B_click = 1;
    }else
    {
        B_click = 0;
    }

    if ( (A_click+B_click) == 2 )
    {
        V_AB = 1.5;
    } else if( (A_click+B_click) == 1 )
    {
        V_AB = 1;
    } else if( (A_click+B_click) == 0 )
    {
        V_AB = 0.5;
    }
    taskEXIT_CRITICAL();
}


