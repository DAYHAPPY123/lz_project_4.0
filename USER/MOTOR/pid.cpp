#include <cstdlib>
#include <cstring>
#include <cstdint>

#include "pid.h"
#include "mycan.h"
#include "fdcan.h"
#include "usart.h"
#include "stdlib.h"
#include "motor.h"
PID_INIT pid1={
        .vel_kp=100.0f,
        .vel_ki=2.0f,
};

PID_INIT pid2={
        .vel_kp=100.0f,
        .vel_ki=2.0f,
};

PID_INIT pid3={
        .pos_kp=2.2f,
        .pos_ki=0.005f,
        .pos_kd=15.0f,
};

PID_INIT pid4={
        .pos_kp=3.0f,
        .pos_ki=0.008f,
        .pos_kd=15.0f,
};

PID_INIT pid3_2={
        .vel_kp=30.0f,
        .vel_ki=0.05f,
        .vel_kd=0.0f
};

PID_INIT pid4_2={
        .vel_kp=30.0f,
        .vel_ki=0.05f,
        .vel_kd=0.0f
};

float ramp_step=50;

float limit(float a, float ABS_MAX)
{
    if (a > ABS_MAX)
        a = ABS_MAX;
    if (a < -ABS_MAX)
        a = -ABS_MAX;
    return a;
}

void update_target(float target_new,float target_now,struct motor_init *motor)
{
    if (target_now < target_new)
    {
        motor->target_new += ramp_step;
        if (motor->target_new > target_new)
            motor->target_new = target_new;
    }
    else if (target_now > target_new)
    {
        motor->target_new -= ramp_step;
        if (motor->target_new < target_new)
            motor->target_new = target_new;
    }
}

int16_t PIDControl_3508(struct PID_INIT* pid, float targetSpeed,float NowSpeed)
{
    pid->error[0] = targetSpeed - NowSpeed;

    pid->integral += pid->error[0];

    pid->derivative = pid->error[0] - pid->error[1];
    pid->output = pid->vel_kp * pid->error[0] + pid->vel_ki *pid->integral
                  + pid->vel_kd * pid->derivative;
    pid->output= limit(pid->output,10000);

    pid->error[1] = pid->error[0];

    return pid->output;
}

int16_t PIDControl_2006(struct PID_INIT* pid,float targetPos,float NowPos)
{
    if (pid==&pid3)
    {
        update_target(targetPos,NowPos,&motor6_3);
        pid->error[0] = motor6_3.target_new - NowPos;
    } else if(pid==&pid4)
    {
        update_target(targetPos,NowPos,&motor6_4);
        pid->error[0] = motor6_4.target_new - NowPos;
    }
    pid->integral += pid->error[0];
    pid->integral= limit(pid->integral,100000);

    pid->derivative = pid->error[0] - pid->error[1];
    pid->derivative= limit(pid->derivative,10000);

    pid->output = pid->pos_kp * pid->error[0] + pid->pos_ki *pid->integral + pid->pos_kd * pid->derivative;
    pid->output= limit(pid->output,10000);

    pid->error[1] = pid->error[0];
    return pid->output;
}

int16_t PIDControl_2006_v(struct PID_INIT* pid,float targetSpeed,float NowSpeed)
{
    pid->error[0] = targetSpeed - NowSpeed;

    pid->integral += pid->error[0];
    pid->integral= limit(pid->integral,1000);

    pid->derivative = pid->error[0] - pid->error[1];
    pid->derivative= limit(pid->derivative,10000);

    pid->output = pid->vel_kp * pid->error[0] + pid->vel_ki *pid->integral + pid->vel_kd * pid->derivative;
    pid->output= limit(pid->output,10000);

    pid->error[1] = pid->error[0];
    return pid->output;
}

