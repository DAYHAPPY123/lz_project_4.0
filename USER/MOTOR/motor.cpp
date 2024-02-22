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

uint8_t mode;
float left_angle=0;
float right_angle=0;
int16_t left_counter=0;
int16_t right_counter=0;

struct motor_init motor3_1={0};
struct motor_init motor3_2={0};
struct motor_init motor6_3={0};
struct motor_init motor6_4={0};

void motor_reset()
{
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)!=0)
    {
        PIDControl_3508(&pid1,0,motor3_1.rpm);
        PIDControl_3508(&pid2,0,motor3_2.rpm);
        PIDControl_2006_v(&pid3_2,-50,motor6_3.rpm);
        PortSendMotorsCur(0,0,pid3_2.output,0);
    }
    motor6_3.calculate_continuous=0;

    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)!=0)
    {
        PIDControl_3508(&pid1,0,motor3_1.rpm);
        PIDControl_3508(&pid2,0,motor3_2.rpm);
        PIDControl_2006_v(&pid4_2,50,motor6_4.rpm);
        PortSendMotorsCur(0,0,0,pid4_2.output);
    }
    motor6_4.calculate_continuous=0;
}

void motor_io_init()
{
    mode=MOTOR_STOP;
    motor6_3.set_pos=mid_counter_3;
    motor6_4.set_pos=mid_counter_4;
}

void Speed_Send(void){
    PIDControl_3508(&pid1,motor3_1.set_rpm,motor3_1.rpm);
    PIDControl_3508(&pid2,-motor3_2.set_rpm,motor3_2.rpm);
    PIDControl_2006_pos(&pid3,motor6_3.set_pos,motor6_3.calculate_continuous);
    PIDControl_2006_pos(&pid4,motor6_4.set_pos,motor6_4.calculate_continuous);
    PortSendMotorsCur(pid1.output,pid2.output,pid3.output,pid4.output);
}

void angle_cal()
{
    if (mode == MOTOR_MANUAL)
    {
        if(rc_ctrl.rc.ch[2]==0){
            motor6_3.set_pos=mid_counter_3;
            motor6_4.set_pos=mid_counter_4;
        }
        if(rc_ctrl.rc.ch[2]<0){
            left_counter = int16_t(double(rc_ctrl.rc.ch[2])/660.0f*8191.0*3.0/4.0/2*0.7);
            left_angle = left_counter / 8191.0 /3.0 * 360 * PI / 180.0;
            right_angle = atan(1.0/  (1.0/tan(left_angle)-(float)car_width/(float)car_length));
            right_counter = right_angle*180/PI/360*8191*3;
            motor6_3.set_pos=mid_counter_3-left_counter;
            motor6_4.set_pos=mid_counter_4-right_counter;
        }
        if(rc_ctrl.rc.ch[2]>0){
            right_counter = int16_t(double(rc_ctrl.rc.ch[2])/660.0f*8191.0*3.0/4.0/2*0.7);
            right_angle = right_counter / 8191.0 /3.0 * 360 * PI / 180.0;
            left_angle = atan(1.0/  (1.0/tan(right_angle)+(float)car_width/(float)car_length));
            left_counter = left_angle*180/PI/360*8191*3;
            motor6_3.set_pos=mid_counter_3-left_counter;
            motor6_4.set_pos=mid_counter_4-right_counter;
        }
    }

    else if (mode == MOTOR_AUTO )
    {
        if(turn_angle==0){
            motor6_3.set_pos=mid_counter_3;
            motor6_4.set_pos=mid_counter_4;
        }
        if(turn_angle<0){//向左转
            left_counter = turn_angle;
            left_angle = left_counter / 8191 * 360 * PI / 180;
            right_angle = atan(1.0/  (1.0/tan(left_angle)-(float)car_width/(float)car_length));
            right_counter = right_angle*180/PI/360*8191;
            motor6_3.set_pos=mid_counter_3+left_counter;
            motor6_4.set_pos=mid_counter_4+right_counter;
        }
        if(turn_angle>0){//向右转
            right_counter = turn_angle;
            right_angle = right_counter / 8191 * 360 * PI / 180;
            left_angle = atan(1.0/  (1.0/tan(right_angle)+(float)car_width/(float)car_length));
            left_counter = left_angle*180/PI/360*8191;
            motor6_3.set_pos=mid_counter_3+left_counter;
            motor6_4.set_pos=mid_counter_4+right_counter;
        }
    }

    else if (mode == MOTOR_STOP)
    {
        motor6_3.set_pos=mid_counter_3;
        motor6_4.set_pos=mid_counter_4;
    }
}

void backwheel_speed_cal(void)
{
    if (mode == MOTOR_MANUAL)
    {
        if(rc_ctrl.rc.ch[2]==0){
            motor3_1.set_rpm=float ((float)(rc_ctrl.rc.ch[1])/660.0f*20.0f);
            motor3_2.set_rpm=float((float)(rc_ctrl.rc.ch[1])/660.0f*20.0f);}
        if(rc_ctrl.rc.ch[2]>0){
            motor3_1.set_rpm=float((float)(rc_ctrl.rc.ch[1])/660.0f*20.0f);
            motor3_2.set_rpm=motor3_1.set_rpm* tan(left_angle)/tan(right_angle);}
        if(rc_ctrl.rc.ch[2]<0){
            motor3_2.set_rpm=int16_t((float)(rc_ctrl.rc.ch[1])/660.0f*20.0f);
            motor3_1.set_rpm=motor3_2.set_rpm* tan(right_angle)/tan(left_angle);}
    }

    else if (mode == MOTOR_AUTO )
    {
        if(turn_angle==0){
            motor3_1.set_rpm=5;
            motor3_2.set_rpm=5;}
        if(turn_angle<0){
            motor3_1.set_rpm=5+float(-turn_angle/1000.0f*1.5f);
            motor3_2.set_rpm=motor3_1.set_rpm* tan(left_angle)/tan(right_angle);}
        if(turn_angle>0){
            motor3_2.set_rpm=5+float(turn_angle/1000.0f*1.5f);;
            motor3_1.set_rpm=motor3_2.set_rpm* tan(right_angle)/tan(left_angle);}
    }

    else if (mode == MOTOR_STOP)
    {
        motor3_1.set_rpm=0;
        motor3_2.set_rpm=0;
    }
}

