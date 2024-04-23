#include "servos.h"
#include "tim.h"
#include "motor.h"
#include "remote.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

//右边是servo0,左边是servo1

float servos_pos[2];//0到180度
int actual_pos_input[2];
uint32_t adc_value[2];
int detect_counter=0;
float angle_limit;


void servos_init()
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    servos_start();
}

void servos_stop()
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
}

void servos_start()
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
}

void servo0_limit(float* input)
{
if (*input > angle_limit) *input = angle_limit;
if (*input < 0) *input = 0;
}

void servo1_limit(float* input)
{
    if (*input > 0) *input = 0;
    if (*input < -angle_limit) *input = -angle_limit;
}

void servos_reset()
{
    actual_pos_input[0]=servo0_start;
    actual_pos_input[1]=servo1_start;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, actual_pos_input[0]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, actual_pos_input[1]);
}

void servos_control()
{
//    usart_printf("%d \r\n",rc_ctrl.rc.ch[4]);
    if (mode == MOTOR_MANUAL)
    {
        servos_start();
        if (rc_ctrl.rc.s[0] == 3)//中
        {
            if (rc_ctrl.rc.ch[4] > 0)//滚轮上滚
            {
                servos_pos[0]-=servos_speed;
                servo0_limit(&servos_pos[0]);
                actual_pos_input[0]=(int)(servos_pos[0]/180.0*200+servo0_start);
                servos_pos[1]+=servos_speed;
                servo1_limit(&servos_pos[1]);
                actual_pos_input[1]=(int)(servos_pos[1]/180.0*200+servo1_start);
            }
            else if (rc_ctrl.rc.ch[4] < 0)//滚轮下滚
            {
                servos_pos[0]+=servos_speed;
                servo0_limit(&servos_pos[0]);
                actual_pos_input[0]=(int)(servos_pos[0]/180.0*200+servo0_start);
                servos_pos[1]-=servos_speed;
                servo1_limit(&servos_pos[1]);
                actual_pos_input[1]=(int)(servos_pos[1]/180.0*200+servo1_start);
            }
        }
        else if (rc_ctrl.rc.s[0] == 1)//上，右,越大越下
        {
            if (rc_ctrl.rc.ch[4] > 0)//滚轮上滚
            {
                servos_pos[0]-=servos_speed;
                servo0_limit(&servos_pos[0]);
                actual_pos_input[0]=(int)(servos_pos[0]/180.0*200+servo0_start);
            }
            else if (rc_ctrl.rc.ch[4] < -0)//滚轮下滚
            {
                servos_pos[0]+=servos_speed;
                servo0_limit(&servos_pos[0]);
                actual_pos_input[0]=(int)(servos_pos[0]/180.0*200+servo0_start);
            }
        }
        else if (rc_ctrl.rc.s[0] == 2)//下，左，越小越下
        {
            if (rc_ctrl.rc.ch[4] > 0)//滚轮上滚
            {
                servos_pos[1]+=servos_speed;
                servo1_limit(&servos_pos[1]);
                actual_pos_input[1]=(int)(servos_pos[1]/180.0*200+servo1_start);
            }
            else if (rc_ctrl.rc.ch[4] < -0)//滚轮下滚
            {
                servos_pos[1]-=servos_speed;
                servo1_limit(&servos_pos[1]);
                actual_pos_input[1]=(int)(servos_pos[1]/180.0*200+servo1_start);
            }
        }
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, actual_pos_input[0]);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, actual_pos_input[1]);
    }
    else if (mode == MOTOR_STOP)
    {
//        servos_stop();
        actual_pos_input[0]=servo0_start;
        actual_pos_input[1]=servo1_start;
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, actual_pos_input[0]);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, actual_pos_input[1]);
    }
//    usart_printf("%d,%d\r\n",actual_pos_input[0],actual_pos_input[1]);
}

//void ADC_detect()
//{
//    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7);//LED
//    HAL_ADC_Start_DMA(&hadc4, &adc_value[0], 1);
//    HAL_ADC_Start_DMA(&hadc1, &adc_value[1], 1);
//    if( (adc_value[0]>4000)||(adc_value[1]>4000) )
//    {
//        if(detect_counter>=1)
//        {
//            mode=MOTOR_STOP;
//            servos_init();
//            servos_pos[0]=0;servos_pos[1]=0;
//            actual_pos_input[0]=servo0_start;
//            actual_pos_input[1]=servo1_start;
//            detect_counter=0;
//            while(mode==MOTOR_AUTO);
//        }
//        else
//        {
//            detect_counter++;
//        }
//    } else if( (adc_value[0]==0)&&(adc_value[1]==0) )
//    {
//        detect_counter=0;
//    }
//    usart_printf("%d,%d,%d\n",adc_value[0],adc_value[1],detect_counter);
//}