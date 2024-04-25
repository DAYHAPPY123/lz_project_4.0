#include "motor.h"
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include "fdcan.h"
#include "math.h"
#include "mycan.h"
#include "usart.h"
#include "remote.h"
#include "pid.h"
#include "agv.h"
#include "math.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

uint8_t mode;
float left_angle=0;
float right_angle=0;
int16_t left_counter=0;
int16_t right_counter=0;
float back_setrpm=0;
int pos_start;

struct motor_init motor3_1={0};
struct motor_init motor3_2={0};
struct motor_init motor2_1={0};
struct motor_init motor2_2={0};

void motor_reset()
{
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)!=0)
    {
        PIDControl_3508(&pid3_1,0,motor3_1.rpm);
        PIDControl_3508(&pid3_2,0,motor3_2.rpm);
        PIDControl_2006_v(&pid_reset1,-50,motor2_1.rpm);
        PortSendMotorsCur(0,0,pid_reset1.output,0);
        osDelay(5);
    }
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)!=0)
    {
        PIDControl_3508(&pid3_1,0,motor3_1.rpm);
        PIDControl_3508(&pid3_2,0,motor3_2.rpm);
        PIDControl_2006_v(&pid_reset2,50,motor2_2.rpm);
        PortSendMotorsCur(0,0,0,pid_reset2.output);
        osDelay(5);
    }

    taskENTER_CRITICAL();
    {
        counter_change2006_1=0;
        counter_change2006_2=0;
        motor2_1.calculate_continuous=0;motor2_1.continuous=0;
        motor2_2.calculate_continuous=0;motor2_2.continuous=0;
    }
    taskEXIT_CRITICAL();

    while (motor2_1.calculate_continuous<=3200)
    {
        PIDControl_3508(&pid3_1,0,motor3_1.rpm);
        PIDControl_3508(&pid3_2,0,motor3_2.rpm);
        PIDControl_2006_v(&pid_reset1,50,motor2_1.rpm);
        PortSendMotorsCur(0,0,pid_reset1.output,0);
        osDelay(5);
    }

    while (motor2_2.calculate_continuous>=-3000)
    {
        PIDControl_3508(&pid3_1,0,motor3_1.rpm);
        PIDControl_3508(&pid3_2,0,motor3_2.rpm);
        PIDControl_2006_v(&pid_reset2,-50,motor2_2.rpm);
        PortSendMotorsCur(0,0,0,pid_reset2.output);
        osDelay(5);
    }

    pid3_1.error[0]=0;
    pid3_1.integral=0;
    pid3_1.derivative=0;
    pid3_1.output=0;
    pid3_1.error[1]=0;
    pid3_2.error[0]=0;
    pid3_2.integral=0;
    pid3_2.derivative=0;
    pid3_2.output=0;
    pid3_2.error[1]=0;
}

void motor_io_init()
{
    mode=MOTOR_STOP;
    motor2_1.set_pos=mid_counter_3;
    motor2_2.set_pos=mid_counter_4;
}

void Speed_Send(void){
    if ( (mode==MOTOR_AUTO)&&(rc_ctrl.rc.s[0] == 2)&&
    (abs(motor3_1.calculate_continuous-motor3_1.set_pos)<=500) )
    {
        PIDControl_3508(&pid3_1,0,motor3_1.rpm);
        PIDControl_3508(&pid3_2,0,motor3_2.rpm);
    } else
    {
        PIDControl_3508(&pid3_1,motor3_1.set_rpm,motor3_1.rpm);
        PIDControl_3508(&pid3_2,-motor3_2.set_rpm,motor3_2.rpm);
    }
    taskENTER_CRITICAL();
    PIDControl_2006_pos(&pid2_1,motor2_1.set_pos,motor2_1.calculate_continuous);
    PIDControl_2006_pos(&pid2_2,motor2_2.set_pos,motor2_2.calculate_continuous);
    taskEXIT_CRITICAL();
//    usart_printf("%.2f,%d,%.2f\r\n",pid2_1.output,
//                 motor2_1.set_pos,motor2_1.calculate_continuous);
//    usart_printf("%.2f  %.2f \r\n",pid2_1.integral,pid2_2.integral);
    PortSendMotorsCur(pid3_1.output,pid3_2.output,pid2_1.output,pid2_2.output);
}

void angle_cal()
{
    if (mode == MOTOR_MANUAL)
    {
        if(rc_ctrl.rc.ch[2]==0){
            motor2_1.set_pos=mid_counter_3;
            motor2_2.set_pos=mid_counter_4;
        }
        if(rc_ctrl.rc.ch[2]<0){//向左转
            left_counter = int16_t(double(rc_ctrl.rc.ch[2])/660.0f*8191.0*3.0/4.0/2*0.7);
            left_angle = left_counter / 8191.0 /3.0 * 360 * PI / 180.0;
            right_angle = atan(1.0/  (1.0/tan(left_angle)-(float)car_width/(float)car_length));
            right_counter = right_angle*180/PI/360*8191*3;
            motor2_1.set_pos=mid_counter_3-left_counter;
            motor2_2.set_pos=mid_counter_4-right_counter;
        }
        if(rc_ctrl.rc.ch[2]>0){//向右转
            right_counter = int16_t(double(rc_ctrl.rc.ch[2])/660.0f*8191.0*3.0/4.0/2*0.7);
            right_angle = right_counter / 8191.0 /3.0 * 360 * PI / 180.0;
            left_angle = atan(1.0/  (1.0/tan(right_angle)+(float)car_width/(float)car_length));
            left_counter = left_angle*180/PI/360*8191*3;
            motor2_1.set_pos=mid_counter_3-left_counter;
            motor2_2.set_pos=mid_counter_4-right_counter;
        }
    }

    else if (mode == MOTOR_AUTO )
    {
        if (rc_ctrl.rc.s[0] == 1)//右拨杆打上,开启前进循迹
        {
            if(turn_angle==0){
                motor2_1.set_pos=mid_counter_3;
                motor2_2.set_pos=mid_counter_4;
            }
            if(turn_angle<0){//向左转
                left_counter = turn_angle;
                left_angle = left_counter / 8191.0 /3.0 * 360 * PI / 180.0;
                right_angle = atan(1.0/  (1.0/tan(left_angle)-(float)car_width/(float)car_length));
                right_counter = right_angle*180/PI/360*8191*3;
                motor2_1.set_pos=mid_counter_3+left_counter;
                motor2_2.set_pos=mid_counter_4+right_counter;
            }
            if(turn_angle>0){//向右转
                right_counter = turn_angle;
                right_angle = right_counter / 8191.0 /3.0 * 360 * PI / 180.0;
                left_angle = atan(1.0/  (1.0/tan(right_angle)+(float)car_width/(float)car_length));
                left_counter = left_angle*180/PI/360*8191*3;
                motor2_1.set_pos=mid_counter_3+left_counter;
                motor2_2.set_pos=mid_counter_4+right_counter;
            }
        }
        else if (rc_ctrl.rc.s[0] == 2)//右拨杆打下,开启后退循迹
        {
            motor2_1.set_pos=mid_counter_3;
            motor2_2.set_pos=mid_counter_4;
        }
    }
    else if (mode == MOTOR_STOP)
    {
        motor2_1.set_pos=mid_counter_3;
        motor2_2.set_pos=mid_counter_4;
    }
}

void backwheel_speed_cal(void)
{
    if (mode == MOTOR_MANUAL)//0-70mm/s,对应set_rpm=0-14.53
    {
        if(rc_ctrl.rc.ch[2]==0){
            motor3_1.set_rpm=float ((float)(rc_ctrl.rc.ch[1])/660.0f*14.53f);
            motor3_2.set_rpm=float((float)(rc_ctrl.rc.ch[1])/660.0f*14.53f);
        }
        if(rc_ctrl.rc.ch[2]>0){
            motor3_1.set_rpm=float((float)(rc_ctrl.rc.ch[1])/660.0f*14.53f);
            motor3_2.set_rpm=motor3_1.set_rpm* tan(left_angle)/tan(right_angle);}
        if(rc_ctrl.rc.ch[2]<0){
            motor3_2.set_rpm=float((float)(rc_ctrl.rc.ch[1])/660.0f*14.53f);
            motor3_1.set_rpm=motor3_2.set_rpm* tan(right_angle)/tan(left_angle);}
    }

    else if (mode == MOTOR_AUTO )//0 -70mm/s,对应set_rpm=0-14.53
    {
        if (rc_ctrl.rc.s[0] == 1)//右拨杆打上,开启前进循迹
        {
            if(turn_angle==0){
                motor3_1.set_rpm=back_setrpm+float ((float)(rc_ctrl.rc.ch[1])/660.0f*14.53/2.0);
                motor3_2.set_rpm=back_setrpm+float ((float)(rc_ctrl.rc.ch[1])/660.0f*14.53/2.0);
                if (motor3_1.set_rpm<=0)
                {
                    motor3_1.set_rpm=motor3_2.set_rpm=0;
                }
                limit(&motor3_1.set_rpm,14.53);
                limit(&motor3_2.set_rpm,14.53);
            }
            if(turn_angle<0){//向左转
                motor3_2.set_rpm=back_setrpm+float ((float)(rc_ctrl.rc.ch[1])/660.0f*14.53/2.0)+float(-turn_angle/1000.0);
                if (motor3_2.set_rpm<=0)
                {
                    motor3_2.set_rpm=0;
                }
                limit(&motor3_2.set_rpm,14.53);
                motor3_1.set_rpm=motor3_2.set_rpm* tan(left_angle)/tan(right_angle);
            }
            if(turn_angle>0){
                motor3_1.set_rpm=back_setrpm+float ((float)(rc_ctrl.rc.ch[1])/660.0f*14.53/2.0)+float(turn_angle/1000.0);
                if (motor3_1.set_rpm<=0)
                {
                    motor3_1.set_rpm=0;
                }
                limit(&motor3_1.set_rpm,14.53);
                motor3_2.set_rpm=motor3_1.set_rpm* tan(right_angle)/tan(left_angle);
            }
        }

        else if (rc_ctrl.rc.s[0] == 2)//右拨杆打下,开启后退
        {
        if(rc_ctrl.rc.ch[1]<=-500)
        {
            motor3_1.set_rpm=-5;
            motor3_2.set_rpm=-5;
            motor3_1.calculate_continuous=0;
            motor3_2.calculate_continuous=0;
            motor3_1.set_pos=-2834;            //  8191X100/92/PI=2834
            motor3_2.set_pos=-2834;
        }
        }
//        usart_printf("%.2f \r\n",back_setrpm);
    }

    else if (mode == MOTOR_STOP)
    {
        motor3_1.set_rpm=0;
        motor3_2.set_rpm=0;
    }
}


