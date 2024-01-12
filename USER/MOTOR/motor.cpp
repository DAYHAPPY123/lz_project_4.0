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

uint8_t mode;
float left_angle=0;
float right_angle=0;
float left_counter=0;
float right_counter=0;

struct motor_init motor3_1={0};
struct motor_init motor3_2={0};
struct motor_init motor6_1={0};
struct motor_init motor6_2={0};

void motor_io_init()
{
    mode=MOTOR_STOP;
    motor6_1.set_pos=3400;
    motor6_2.set_pos=2100;
}

void Speed_Send(void){
    PIDControl_3508(&pid1,motor3_1.set_rpm,motor3_1.rpm);
    PIDControl_3508(&pid2,-motor3_2.set_rpm,motor3_2.rpm);

    PIDControl_6020(&pid3,motor6_1.set_pos,motor6_1.continuous);
    PIDControl_6020(&pid4,motor6_2.set_pos,motor6_2.continuous);

//    usart_printf(" %f,%d,%f,%f \r\n",
//                 wlwg[0].gap_counter,wlwg[0].encoder_counter,pid_front_1.integral,pid_front_1.output);

//    usart_printf(" %f,%d,%f,%f \r\n",
//                 wlwg[1].gap_counter,wlwg[1].encoder_counter,pid_front_2.integral,pid_front_2.output);

    PortSendMotorsCur_3508(pid1.output,pid2.output,0,0);
    PortSendMotorsCur_6020(pid3.output,pid4.output,0,0);

//    usart_printf("%.2f,%.2f,%d,%.2f \r\n",
//                 motor3_1.set_rpm,motor3_1.rpm,pid1.output,pid1.integral);

//    usart_printf("%d,%d,%.2f,%.2f \r\n",
//                 motor6_2.set_pos,motor6_2.continuous,pid4.output,pid4.integral);

}

void angle_cal()
{
    if (mode == MOTOR_MANUAL)
    {
        if(rc_ctrl.rc.ch[2]==0){
            motor6_1.set_pos=mid_counter_1;
            motor6_2.set_pos=mid_counter_2;
        }
        if(rc_ctrl.rc.ch[2]<0){
            left_counter = (float )rc_ctrl.rc.ch[2]/660.0f*1000.0f;
            left_angle = left_counter / 8191 * 360 * PI / 180;
            right_angle = atan(1.0/  (1.0/tan(left_angle)-(float)car_width/(float)car_length));
            right_counter = right_angle*180/PI/360*8191;
            motor6_1.set_pos=mid_counter_1+left_counter;
            motor6_2.set_pos=mid_counter_2+right_counter;
        }
        if(rc_ctrl.rc.ch[2]>0){
            right_counter = (float )rc_ctrl.rc.ch[2]/660.0f*1000.0f;
            right_angle = right_counter / 8191 * 360 * PI / 180;
            left_angle = atan(1.0/  (1.0/tan(right_angle)+(float)car_width/(float)car_length));
            left_counter = left_angle*180/PI/360*8191;
            motor6_1.set_pos=mid_counter_1+left_counter;
            motor6_2.set_pos=mid_counter_2+right_counter;
        }
    }

    else if (mode == MOTOR_AUTO )
    {
        if (  (agv_left[1]-agv_right[1])<10 && (agv_right[1]-agv_left[1])<10 )
        {
            turn_angle=0;
        }
        else
        {
            turn_angle=limit((float)(agv_right[1]-agv_left[1])*10,1000);
        }

        if(turn_angle==0){
            motor6_1.set_pos=mid_counter_1;
            motor6_2.set_pos=mid_counter_2;
        }
        if(turn_angle<0){//向左转
            left_counter = turn_angle;
            left_angle = left_counter / 8191 * 360 * PI / 180;
            right_angle = atan(1.0/  (1.0/tan(left_angle)-(float)car_width/(float)car_length));
            right_counter = right_angle*180/PI/360*8191;
            motor6_1.set_pos=mid_counter_1+left_counter;
            motor6_2.set_pos=mid_counter_2+right_counter;
        }
        if(turn_angle>0){//向右转
            right_counter = turn_angle;
            right_angle = right_counter / 8191 * 360 * PI / 180;
            left_angle = atan(1.0/  (1.0/tan(right_angle)+(float)car_width/(float)car_length));
            left_counter = left_angle*180/PI/360*8191;
            motor6_1.set_pos=mid_counter_1+left_counter;
            motor6_2.set_pos=mid_counter_2+right_counter;
        }
    }

    else if (mode == MOTOR_STOP)
    {
        motor6_1.set_pos=mid_counter_1;
        motor6_2.set_pos=mid_counter_2;
    }
}

void backwheel_speed_cal(void)
{
    if (mode == MOTOR_MANUAL)
    {
        if(rc_ctrl.rc.ch[2]==0){
            motor3_1.set_rpm=float ((float)(rc_ctrl.rc.ch[1])/660.0f*20.0f);
            motor3_2.set_rpm=float((float)(rc_ctrl.rc.ch[1])/660.0f*20.0f);}
        if(rc_ctrl.rc.ch[2]<0){
            motor3_1.set_rpm=float((float)(rc_ctrl.rc.ch[1])/660.0f*20.0f);
            motor3_2.set_rpm=motor3_1.set_rpm* tan(left_angle)/tan(right_angle);}
        if(rc_ctrl.rc.ch[2]>0){
            motor3_2.set_rpm=int16_t((float)(rc_ctrl.rc.ch[1])/660.0f*20.0f);
            motor3_1.set_rpm=motor3_2.set_rpm* tan(right_angle)/tan(left_angle);}
    }

    else if (mode == MOTOR_AUTO )
    {
        if(turn_angle==0){
            motor3_1.set_rpm=5;
            motor3_2.set_rpm=5;}
        if(turn_angle<0){
            motor3_1.set_rpm=5+float(-turn_angle/1000.0f*5.0f);
            motor3_2.set_rpm=motor3_1.set_rpm* tan(left_angle)/tan(right_angle);}
        if(turn_angle>0){
            motor3_2.set_rpm=5+float(turn_angle/1000.0f*5.0f);;
            motor3_1.set_rpm=motor3_2.set_rpm* tan(right_angle)/tan(left_angle);}
    }

    else if (mode == MOTOR_STOP)
    {
        motor3_1.set_rpm=0;
        motor3_2.set_rpm=0;
    }
}

