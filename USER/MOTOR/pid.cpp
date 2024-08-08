#include <cstdlib>
#include <cstring>
#include <cstdint>
#include "pid.h"
#include "mycan.h"
#include "fdcan.h"
#include "usart.h"
#include "stdlib.h"
#include "motor.h"
#include "agv.h"
#include "remote.h"
//PID命名中，1左，2右
//strong,wake意思是按照轮子转幅大小调用的参数，比如向左转，左轮偏角大，左轮调用strong，右轮调用wake

PID_INIT pid3_1={
        .vel_kp=200.0f,
        .vel_ki=3.0f,
};

PID_INIT pid3_2={
        .vel_kp=200.0f,
        .vel_ki=3.0f,
};

PID_INIT pid2_1={
        .pos_kp_strong=3.0f,
        .pos_ki_strong=0.06f,
        .pos_kd_strong=50.0f,

        .pos_kp_wake=2.8f,
        .pos_ki_wake=0.10f,
        .pos_kd_wake=100.0f,
};
PID_INIT pid2_2={
        .pos_kp_strong=2.8f,
        .pos_ki_strong=0.05f,
        .pos_kd_strong=50.0f,

        .pos_kp_wake=2.6f,
        .pos_ki_wake=0.09f,
        .pos_kd_wake=100.0f,
};

PID_INIT pid_reset1={
        .vel_kp=250.0f,
        .vel_ki=0.04f,
        .vel_kd=100.0f
};

PID_INIT pid_reset2={
        .vel_kp=250.0f,
        .vel_ki=0.04f,
        .vel_kd=100.0f
};

float ramp_step[4]={20,0.2,0.3};//2006pos-3508v-2006v
float pid_start=0;

float limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
    return *a;
}

void update_target_pos(float target_new,float target_now,struct motor_init *motor)
{
    if (target_now < target_new)
    {
        motor->target_pos_new += ramp_step[0];
        if (motor->target_pos_new > target_new)
            motor->target_pos_new = target_new;
    }
    else if (target_now > target_new)
    {
        motor->target_pos_new -= ramp_step[0];
        if (motor->target_pos_new < target_new)
            motor->target_pos_new = target_new;
    }
}

void update_target_v(float target_new,float target_now,struct motor_init *motor)
{
    if( (motor==&motor3_1)||(motor==&motor3_2) )
    {
        if (target_now < target_new)
        {
            motor->target_v_new += ramp_step[1];
            if (motor->target_v_new > target_new)
                motor->target_v_new = target_new;
        }
        else if (target_now > target_new)
        {
            motor->target_v_new -= ramp_step[1];
            if (motor->target_v_new < target_new)
                motor->target_v_new = target_new;
        }
    }
    if( (motor==&motor2_1)||(motor==&motor2_2) )
    {
        if (target_now < target_new)
        {
            motor->target_v_new += ramp_step[2];
            if (motor->target_v_new > target_new)
                motor->target_v_new = target_new;
        }
        else if (target_now > target_new)
        {
            motor->target_v_new -= ramp_step[2];
            if (motor->target_v_new < target_new)
                motor->target_v_new = target_new;
        }
    }
}

float PIDControl_3508(struct PID_INIT* pid, float targetSpeed,float NowSpeed)
{
    if (pid==&pid3_1)
    {
        update_target_v(targetSpeed,NowSpeed,&motor3_1);
        pid->error[0] = motor3_1.target_v_new - NowSpeed;
    } else if(pid==&pid3_2)
    {
        update_target_v(targetSpeed,NowSpeed,&motor3_2);
        pid->error[0] = motor3_2.target_v_new - NowSpeed;
    }
    pid->integral += pid->error[0];
    pid->integral= limit(&pid->integral,3000);

    pid->derivative = pid->error[0] - pid->error[1];
    pid->derivative= limit(&pid->derivative,10000);

    pid->output = pid->vel_kp * pid->error[0] + pid->vel_ki *pid->integral
                  + pid->vel_kd * pid->derivative;
    pid->output= limit(&pid->output,15000);

    pid->error[1] = pid->error[0];

    return pid->output;
}

float PIDControl_2006_pos(struct PID_INIT* pid,float targetPos,float NowPos)
{
    if (((mode == MOTOR_AUTO) && (turn_angle <= 0))
    || ((mode == MOTOR_MANUAL) && (rc_ctrl.ch[3] <= 0)))//向左转，转角1大，2小
    {
        if ((pid == &pid2_1)&&((motor3_1.set_rpm!=0))) {
            update_target_pos(targetPos, NowPos, &motor2_1);
            pid->error[0] = motor2_1.target_pos_new - NowPos;
            pid->integral += pid->error[0];
            pid->integral = limit(&pid->integral, 100000);

            pid->derivative = pid->error[0] - pid->error[1];
            pid->derivative = limit(&pid->derivative, 10000);

            pid->output = pid->pos_kp_strong * pid->error[0] + pid->pos_ki_strong * pid->integral +
                          pid->pos_kd_strong * pid->derivative;
            pid->output = limit(&pid->output, 10000);
            pid->error[1] = pid->error[0];
        } else if ((pid == &pid2_2)&&(motor3_1.set_rpm!=0)) {
            update_target_pos(targetPos, NowPos, &motor2_2);
            pid->error[0] = motor2_2.target_pos_new - NowPos;
            pid->integral += pid->error[0];
            pid->integral = limit(&pid->integral, 100000);

            pid->derivative = pid->error[0] - pid->error[1];
            pid->derivative = limit(&pid->derivative, 10000);

            pid->output = pid->pos_kp_wake * pid->error[0] + pid->pos_ki_wake * pid->integral +
                          pid->pos_kd_wake * pid->derivative;
            pid->output = limit(&pid->output, 10000);
            pid->error[1] = pid->error[0];
        }
    }
    else if (((mode == MOTOR_AUTO) && (turn_angle > 0))
    ||((mode == MOTOR_MANUAL) && (rc_ctrl.ch[3] > 0)))//向右转，转角2大，1小
    {
        if ((pid == &pid2_2)&&(motor3_1.set_rpm!=0)) {
            update_target_pos(targetPos, NowPos, &motor2_2);
            pid->error[0] = motor2_2.target_pos_new - NowPos;
            pid->integral += pid->error[0];
            pid->integral = limit(&pid->integral, 100000);

            pid->derivative = pid->error[0] - pid->error[1];
            pid->derivative = limit(&pid->derivative, 10000);

            pid->output = pid->pos_kp_strong * pid->error[0] + pid->pos_ki_strong * pid->integral +
                          pid->pos_kd_strong * pid->derivative;
            pid->output = limit(&pid->output, 10000);
            pid->error[1] = pid->error[0];
        } else if ((pid == &pid2_1)&&(motor3_1.set_rpm!=0)) {
            update_target_pos(targetPos, NowPos, &motor2_1);
            pid->error[0] = motor2_1.target_pos_new - NowPos;
            pid->integral += pid->error[0];
            pid->integral = limit(&pid->integral, 100000);

            pid->derivative = pid->error[0] - pid->error[1];
            pid->derivative = limit(&pid->derivative, 10000);

            pid->output = pid->pos_kp_wake * pid->error[0] + pid->pos_ki_wake * pid->integral +
                          pid->pos_kd_wake * pid->derivative;
            pid->output = limit(&pid->output, 10000);
            pid->error[1] = pid->error[0];
        }
    }
    if (motor3_1.set_rpm==0)
    {
        PID_clear(pid);
    }
    return pid->output;
}

float PIDControl_2006_v(struct PID_INIT* pid,float targetSpeed,float NowSpeed)
{
    if (pid==&pid_reset1)
    {
        update_target_v(targetSpeed,NowSpeed,&motor2_1);
        pid->error[0] = motor2_1.target_v_new - NowSpeed;
    } else if(pid==&pid_reset2)
    {
        update_target_v(targetSpeed,NowSpeed,&motor2_2);
        pid->error[0] = motor2_2.target_v_new - NowSpeed;
    }
    pid->integral += pid->error[0];
    pid->integral= limit(&pid->integral,100000);

    pid->derivative = pid->error[0] - pid->error[1];
    pid->derivative= limit(&pid->derivative,10000);

    pid->output = pid->vel_kp * pid->error[0] + pid->vel_ki *pid->integral + pid->vel_kd * pid->derivative;
    pid->output= limit(&pid->output,10000);

    pid->error[1] = pid->error[0];
    return pid->output;
}



void PID_clear(struct PID_INIT *pid)
{
    pid->error[0]=pid->error[1]=0;
    pid->integral=0;
    pid->derivative=0;
    pid->output=0;
}