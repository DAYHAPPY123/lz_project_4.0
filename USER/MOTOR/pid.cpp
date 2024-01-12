#include <cstdlib>
#include <cstring>
#include <cstdint>

#include "pid.h"
#include "mycan.h"
#include "fdcan.h"
#include "usart.h"
#include "stdlib.h"

PID_INIT pid1={
        .vel_kp=100.0f,
        .vel_ki=2.0f,
};

PID_INIT pid2={
        .vel_kp=100.0f,
        .vel_ki=2.0f,
};

PID_INIT pid3={
        .pos_kp=50.0f,
        .pos_ki=0.1f,
};

PID_INIT pid4={
        .pos_kp=50.0f,
        .pos_ki=0.1f,
};

float ramp_step=10;

float limit(float a, float ABS_MAX)
{
    if (a > ABS_MAX)
        a = ABS_MAX;
    if (a < -ABS_MAX)
        a = -ABS_MAX;
    return a;
}

float update_target(float ramp_target,float target_now)
{
    if (target_now < ramp_target)
    {
        target_now += ramp_step;
        if (target_now >= ramp_target)
            target_now = ramp_target;
    }
    else if (target_now > ramp_target)
    {
        target_now -= ramp_step;
        if (target_now <= ramp_target)
            target_now = ramp_target;
    }
    return target_now;
}

int16_t PIDControl_3508(struct PID_INIT* pid, float targetSpeed,float NowSpeed)
{
    pid->error[0] = update_target(targetSpeed,NowSpeed) - NowSpeed;

    pid->integral += pid->error[0];

    pid->derivative = pid->error[0] - pid->error[1];
    pid->output = pid->vel_kp * pid->error[0] + pid->vel_ki *pid->integral
                  + pid->vel_kd * pid->derivative;
    pid->output= limit(pid->output,3000);

    pid->error[1] = pid->error[0];

    return pid->output;
}

int16_t PIDControl_6020(struct PID_INIT* pid,float targetPos,float NowPos)
{
    pid->error[0] = update_target(targetPos,NowPos) - NowPos;

    pid->integral += pid->error[0];
    pid->integral= limit(pid->integral,50000);

    pid->derivative = pid->error[0] - pid->error[1];

    pid->output = pid->pos_kp * pid->error[0] + pid->pos_ki *pid->integral + pid->pos_kd * pid->derivative;
    pid->output= limit(pid->output,20000);

    pid->error[1] = pid->error[0];

    return pid->output;
}



