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

void motor_enable()
{
    mode=MOTOR_STOP;
    PID2_1.motor.set_pos=mid_counter_3;
    PID2_2.motor.set_pos=mid_counter_4;

    PID3_1.Spd_Param_set(0,0,0);
    PID3_1.Pos_Param_set(0,0,0);
    PID3_1.ramp_Spd_set(0);
    PID3_1.ramp_Pos_set(0);
    PID3_1.limit_Spd_set(0,0,0,0);
    PID3_1.limit_Pos_set(0,0,0,0);

    PID3_2.Spd_Param_set(0,0,0);
    PID3_2.Pos_Param_set(0,0,0);
    PID3_2.ramp_Spd_set(0);
    PID3_2.ramp_Pos_set(0);
    PID3_2.limit_Spd_set(0,0,0,0);
    PID3_2.limit_Pos_set(0,0,0,0);

    PID2_1.Spd_Param_set(0,0,0);
    PID2_1.Pos_Param_set(0,0,0);
    PID2_1.ramp_Spd_set(0);
    PID2_1.ramp_Pos_set(0);
    PID2_1.limit_Spd_set(0,0,0,0);
    PID2_1.limit_Pos_set(0,0,0,0);

    PID2_2.Spd_Param_set(0,0,0);
    PID2_2.Pos_Param_set(0,0,0);
    PID2_2.ramp_Spd_set(0);
    PID2_2.ramp_Pos_set(0);
    PID2_2.limit_Spd_set(0,0,0,0);
    PID2_2.limit_Pos_set(0,0,0,0);

}

void motor_reset()
{
    int i1=0;int i2=0;
    while (i1<=10)
    {
        PID3_1.Spd_calculate(0,PID3_1.motor.rpm);
        PID3_2.Spd_calculate(0,PID3_2.motor.rpm);
        PID2_1.Spd_calculate(-50,PID2_1.motor.rpm);
        PID2_2.Spd_calculate(0,PID2_2.motor.rpm);
        PortSendMotorsCur(PID3_1.Spd_output_get(),PID3_2.Spd_output_get()
                          ,PID2_1.Spd_output_get(),PID2_2.Spd_output_get());
//        usart_printf("%f,%d\r\n",pid2_1_v.output,PID2_1.motor.current);
        if(abs(PID2_1.motor.current)>9000) i1++;
        osDelay(5);
    }
    while (i2<=10)
    {
        PID3_1.Spd_calculate(0,PID3_1.motor.rpm);
        PID3_2.Spd_calculate(0,PID3_2.motor.rpm);
        PID2_1.Spd_calculate(0,PID2_1.motor.rpm);
        PID2_2.Spd_calculate(50,PID2_2.motor.rpm);
        PortSendMotorsCur(PID3_1.Spd_output_get(),PID3_2.Spd_output_get()
                ,PID2_1.Spd_output_get(),PID2_2.Spd_output_get());
        if(abs(PID2_2.motor.current)>9000) i2++;
        osDelay(5);
    }

    taskENTER_CRITICAL();
    {
        counter_change2006_1=0;
        counter_change2006_2=0;
        PID2_1.motor.calculate_continuous=0;PID2_1.motor.continuous=0;
        PID2_2.motor.calculate_continuous=0;PID2_2.motor.continuous=0;
    }
    taskEXIT_CRITICAL();

    while (PID2_1.motor.calculate_continuous<=3200)
    {
        PID3_1.Spd_calculate(0,PID3_1.motor.rpm);
        PID3_2.Spd_calculate(0,PID3_2.motor.rpm);
        PID2_1.Spd_calculate(50,PID2_1.motor.rpm);
        PID2_2.Spd_calculate(0,PID2_2.motor.rpm);
        PortSendMotorsCur(PID3_1.Spd_output_get(),PID3_2.Spd_output_get()
                ,PID2_1.Spd_output_get(),PID2_2.Spd_output_get());
        osDelay(5);
    }

    while (PID2_2.motor.calculate_continuous>=-3000)
    {
        PID3_1.Spd_calculate(0,PID3_1.motor.rpm);
        PID3_2.Spd_calculate(0,PID3_2.motor.rpm);
        PID2_1.Spd_calculate(0,PID2_1.motor.rpm);
        PID2_2.Spd_calculate(-50,PID2_2.motor.rpm);
        PortSendMotorsCur(PID3_1.Spd_output_get(),PID3_2.Spd_output_get()
                ,PID2_1.Spd_output_get(),PID2_2.Spd_output_get());
        osDelay(5);
    }
    PID3_1.PID_clear();
    PID3_2.PID_clear();
    PID2_1.PID_clear();
    PID2_2.PID_clear();
}

void Speed_Send(void){

    //****给研究院使用，未面向工业需求，先取消了自动前进后退固定距离的功能****//
//    if ( (mode==MOTOR_AUTO)&&(rc_ctrl.rc.s[0] == 2)&&
//    (abs(PID3_1.motor.calculate_continuous-PID3_1.motor.set_pos)<=100) )
//    {
//        PIDControl_3508(&pid3_1,0,PID3_1.motor.rpm);
//        PIDControl_3508(&pid3_2,0,PID3_2.motor.rpm);
//    } else

    PID3_1.Spd_calculate(PID3_1.motor.set_rpm,PID3_1.motor.rpm);
    PID3_2.Spd_calculate(-PID3_2.motor.set_rpm,PID3_2.motor.rpm);

    taskENTER_CRITICAL();
    PID2_1.Pos_calculate(PID2_1.motor.set_pos,PID2_1.motor.calculate_continuous);
    PID2_2.Pos_calculate(PID2_2.motor.set_pos,PID2_2.motor.calculate_continuous);

    PID2_1.Spd_calculate(PID2_1.Pos_output_get(),PID2_1.motor.rpm);
    PID2_2.Spd_calculate(PID2_2.Pos_output_get(),PID2_2.motor.rpm);

    taskEXIT_CRITICAL();
//    usart_printf("%.2f,%d,%.2f\r\n",pid2_1.output,
//                 PID2_1.motor.set_pos,PID2_1.motor.calculate_continuous);
//    usart_printf("%.2f  %.2f \r\n",pid2_1.integral,pid2_2.integral);
    PortSendMotorsCur(PID3_1.Spd_output_get(),PID3_2.Spd_output_get()
            ,PID2_1.Spd_output_get(),PID2_2.Spd_output_get());
    //    usart_printf("%.2f %.2f %d %d\r\n",PID3_1.motor.set_rpm,PID3_2.motor.set_rpm,rc_ctrl.ch[2],rc_ctrl.ch[3]);

//        usart_printf("%d %d %d %d\r\n",rc_ctrl.ch[0],rc_ctrl.ch[1],rc_ctrl.ch[2],rc_ctrl.ch[3]);

}

void angle_cal()
{
//    usart_printf("%.3f  %d  %d\r\n",turn_angle,PID2_1.motor.set_pos,PID2_2.motor.set_pos);
    if (mode == MOTOR_MANUAL)
    {
        if(rc_ctrl.ch[3]==0){
            PID2_1.motor.set_pos=mid_counter_3;
            PID2_2.motor.set_pos=mid_counter_4;
        }
        if(rc_ctrl.ch[3]<0){//向左转
            left_counter = int16_t(double(rc_ctrl.ch[3])/671.0f*8191.0*3.0/4.0/2*0.7);
            left_angle = left_counter / 8191.0 /3.0 * 360 * PI / 180.0;
            right_angle = atan(1.0/  (1.0/tan(left_angle)-(float)car_width/(float)car_length));
            right_counter = right_angle*180/PI/360*8191*3;
            PID2_1.motor.set_pos=mid_counter_3-left_counter;
            PID2_2.motor.set_pos=mid_counter_4-right_counter;
        }
        if(rc_ctrl.ch[3]>0){//向右转
            right_counter = int16_t(double(rc_ctrl.ch[3])/671.0f*8191.0*3.0/4.0/2*0.7);
            right_angle = right_counter / 8191.0 /3.0 * 360 * PI / 180.0;
            left_angle = atan(1.0/  (1.0/tan(right_angle)+(float)car_width/(float)car_length));
            left_counter = left_angle*180/PI/360*8191*3;
            PID2_1.motor.set_pos=mid_counter_3-left_counter;
            PID2_2.motor.set_pos=mid_counter_4-right_counter;
        }
    }

    else if (mode == MOTOR_AUTO )
    {
            if(turn_angle==0){
                PID2_1.motor.set_pos=mid_counter_3;
                PID2_2.motor.set_pos=mid_counter_4;
            }
            if(turn_angle<0){//向左转
                left_counter = turn_angle;
                left_angle = left_counter / 8191.0 /3.0 * 360 * PI / 180.0;
                right_angle = atan(1.0/  (1.0/tan(left_angle)-(float)car_width/(float)car_length));
                right_counter = right_angle*180/PI/360*8191*3;
                PID2_1.motor.set_pos=mid_counter_3+left_counter;
                PID2_2.motor.set_pos=mid_counter_4+right_counter;
            }
            if(turn_angle>0){//向右转
                right_counter = turn_angle;
                right_angle = right_counter / 8191.0 /3.0 * 360 * PI / 180.0;
                left_angle = atan(1.0/  (1.0/tan(right_angle)+(float)car_width/(float)car_length));
                left_counter = left_angle*180/PI/360*8191*3;
                PID2_1.motor.set_pos=mid_counter_3+left_counter;
                PID2_2.motor.set_pos=mid_counter_4+right_counter;
            }
    }
    else if (mode == MOTOR_STOP)
    {
        PID2_1.motor.set_pos=mid_counter_3;
        PID2_2.motor.set_pos=mid_counter_4;
    }
}

void backwheel_speed_cal(void)
{
    usart_printf("%d %d %f %f   123\r\n",mode,rc_ctrl.ch[3],PID3_1.motor.set_rpm,PID3_2.motor.set_rpm);

    if (mode == MOTOR_MANUAL)//0-70mm/s,对应set_rpm=0-14.53
    {
        if(rc_ctrl.ch[3]==0){
            PID3_1.motor.set_rpm=float ((float)(rc_ctrl.ch[2])/671.0f*14.53f);
            PID3_2.motor.set_rpm=float((float)(rc_ctrl.ch[2])/671.0f*14.53f);
        }
        if(rc_ctrl.ch[3]>0){
            PID3_1.motor.set_rpm=float((float)(rc_ctrl.ch[2])/671.0f*14.53f);
            PID3_2.motor.set_rpm=PID3_1.motor.set_rpm* tan(left_angle)/tan(right_angle);}
        if(rc_ctrl.ch[3]<0){
            PID3_2.motor.set_rpm=float((float)(rc_ctrl.ch[2])/671.0f*14.53f);
            PID3_1.motor.set_rpm=PID3_2.motor.set_rpm* tan(right_angle)/tan(left_angle);}
    }

    else if (mode == MOTOR_AUTO )//0 -70mm/s,对应set_rpm=0-14.53
    {
            if(turn_angle==0){
                PID3_1.motor.set_rpm=back_setrpm+float ((float)(rc_ctrl.ch[2])/671.0f*14.53/2.0);
                PID3_2.motor.set_rpm=back_setrpm+float ((float)(rc_ctrl.ch[2])/671.0f*14.53/2.0);
                if (PID3_1.motor.set_rpm<=0)
                {
                    PID3_1.motor.set_rpm=0;
                    PID3_2.motor.set_rpm=0;
                }
                limit(&PID3_1.motor.set_rpm,14.53);
                limit(&PID3_2.motor.set_rpm,14.53);
            }
            if(turn_angle<0){//向左转
                PID3_2.motor.set_rpm=back_setrpm+float ((float)(rc_ctrl.ch[2])/671.0f*14.53/2.0)+float(-turn_angle/1000.0);
                if (PID3_2.motor.set_rpm<=0)
                {
                    PID3_2.motor.set_rpm=0;
                }
                limit(&PID3_2.motor.set_rpm,14.53);
                PID3_1.motor.set_rpm=PID3_2.motor.set_rpm* tan(left_angle)/tan(right_angle);
            }
            if(turn_angle>0){
                PID3_1.motor.set_rpm=back_setrpm+float ((float)(rc_ctrl.ch[2])/671.0f*14.53/2.0)+float(turn_angle/1000.0);
                if (PID3_1.motor.set_rpm<=0)
                {
                    PID3_1.motor.set_rpm=0;
                }
                limit(&PID3_1.motor.set_rpm,14.53);
                PID3_2.motor.set_rpm=PID3_1.motor.set_rpm* tan(right_angle)/tan(left_angle);
            }

        //****给研究院使用，未面向工业需求，先取消了自动前进后退固定距离的功能****//
//        else if (rc_ctrl.rc.s[0] == 2)//右拨杆打下,开启后退
//        {
//        if(rc_ctrl.rc.ch[1]<=-500)
//        {
//            PID3_1.motor.set_rpm=-3;
//            PID3_2.motor.set_rpm=-3;
//            PID3_1.motor.calculate_continuous=0;
//            PID3_2.motor.calculate_continuous=0;
////            PID3_1.motor.set_pos=-2834;            //  8191X100/92/PI=2834
////            PID3_2.motor.set_pos=-2834;
//            PID3_1.motor.set_pos=-1417;            //  8191X100/92/PI=2834
//            PID3_2.motor.set_pos=-1417;
//        }

//        }


//        usart_printf("%.2f \r\n",back_setrpm);
    }

    else if (mode == MOTOR_STOP)
    {
        PID3_1.motor.set_rpm=0;
        PID3_2.motor.set_rpm=0;
    }
}



