#include "servos.h"
#include "tim.h"
#include "motor.h"
#include "remote.h"
#include "adc.h"

int servos_pos[2];//0到180度
int actual_pos_input[2];

void servos_init()
{
    servos_pos[0]=0;
    actual_pos_input[0]=servos_pos[0]/180*200+50;//实际输入为50-250
    actual_pos_input[1]=250-actual_pos_input[0];
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    servos_start();
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,actual_pos_input[0]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,actual_pos_input[1]);
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

void servos_limit(int* input)
{
if (*input>180) *input=180;
if (*input<0) *input=0;
}

void servos_control()
{
    if ((mode == MOTOR_MANUAL) || (mode == MOTOR_AUTO))
    {
        servos_start();
        if (rc_ctrl.rc.s[0] == 3)
        {//中
            if (rc_ctrl.rc.ch[4] >= 200)//滚轮上滚
            {
                servos_pos[0]+=10;
                servos_pos[1]+=10;
                servos_limit(&servos_pos[0]);servos_limit(&servos_pos[1]);
                actual_pos_input[0]=(int)((float)servos_pos[0]/180.0*200+50);
                actual_pos_input[1]=250-(int)((float)servos_pos[1]/180.0*200+50);
            }
            else if (rc_ctrl.rc.ch[4] <= -200)//滚轮下滚
            {
                servos_pos[0]-=10;
                servos_pos[1]-=10;
                servos_limit(&servos_pos[0]);servos_limit(&servos_pos[1]);
                actual_pos_input[0]=(int)((float)servos_pos[0]/180.0*200+50);
                actual_pos_input[1]=250-(int)((float)servos_pos[1]/180.0*200+50);
            }
        }
        if (rc_ctrl.rc.s[0] == 1)
        {//上
            if (rc_ctrl.rc.ch[4] >= 200)//滚轮上滚
            {
                servos_pos[0]+=10;
                servos_limit(&servos_pos[0]);
                actual_pos_input[0]=(int)((float)servos_pos[0]/180.0*200+50);
            }
            else if (rc_ctrl.rc.ch[4] <= -200)//滚轮下滚
            {
                servos_pos[0]-=10;
                servos_limit(&servos_pos[0]);
                actual_pos_input[0]=(int)((float)servos_pos[0]/180.0*200+50);
            }
        }
        if (rc_ctrl.rc.s[0] == 2)
        {//下
            if (rc_ctrl.rc.ch[4] >= 200)//滚轮上滚
            {
                servos_pos[1]+=10;
                servos_limit(&servos_pos[1]);
                actual_pos_input[1]=(int)((float)servos_pos[1]/180.0*200+50);
            }
            else if (rc_ctrl.rc.ch[4] <= -200)//滚轮下滚
            {
                servos_pos[1]-=10;
                servos_limit(&servos_pos[1]);
                actual_pos_input[1]=(int)((float)servos_pos[1]/180.0*200+50);
            }
        }
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, actual_pos_input[0]);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, actual_pos_input[1]);
    }
    else if (mode == MOTOR_STOP)
    {
        servos_stop();
    }
}