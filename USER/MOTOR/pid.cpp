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
        .vel_kp=200.0f,
        .vel_ki=3.0f,
};

PID_INIT pid2={
        .vel_kp=200.0f,
        .vel_ki=3.0f,
};

PID_INIT pid3={
        .pos_kp=2.0f,
        .pos_ki=0.03f,
        .pos_kd=20.0f,
};

PID_INIT pid4={
        .pos_kp=2.0f,
        .pos_ki=0.03f,
        .pos_kd=20.0f,
};

PID_INIT pid3_2={
        .vel_kp=70.0f,
        .vel_ki=0.06f,
        .vel_kd=0.0f
};

PID_INIT pid4_2={
        .vel_kp=50.0f,
        .vel_ki=0.04f,
        .vel_kd=0.0f
};

float ramp_step[3]={30,0.15,0.0004};//2006pos-3508v-2006v

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

    if( (motor==&motor6_3)||(motor==&motor6_4) )
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

int16_t PIDControl_3508(struct PID_INIT* pid, float targetSpeed,float NowSpeed)
{
    if (pid==&pid1)
    {
        update_target_v(targetSpeed,NowSpeed,&motor3_1);
        pid->error[0] = motor3_1.target_v_new - NowSpeed;
    } else if(pid==&pid2)
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

int16_t PIDControl_2006_pos(struct PID_INIT* pid,float targetPos,float NowPos)
{
    if (pid==&pid3)
    {
        update_target_pos(targetPos,NowPos,&motor6_3);
        pid->error[0] = motor6_3.target_pos_new - NowPos;
    } else if(pid==&pid4)
    {
        update_target_pos(targetPos,NowPos,&motor6_4);
        pid->error[0] = motor6_4.target_pos_new - NowPos;
    }
    pid->integral += pid->error[0];
    pid->integral= limit(&pid->integral,100000);

    pid->derivative = pid->error[0] - pid->error[1];
    pid->derivative= limit(&pid->derivative,10000);

    pid->output = pid->pos_kp * pid->error[0] + pid->pos_ki *pid->integral + pid->pos_kd * pid->derivative;
    pid->output= limit(&pid->output,10000);

    pid->error[1] = pid->error[0];
    return pid->output;
}

int16_t PIDControl_2006_v(struct PID_INIT* pid,float targetSpeed,float NowSpeed)
{
    if (pid==&pid3_2)
    {
        update_target_v(targetSpeed,NowSpeed,&motor6_3);
        pid->error[0] = motor6_3.target_v_new - NowSpeed;
//        usart_printf("%.2f\r\n", motor6_3.target_v_new);
    } else if(pid==&pid4_2)
    {
        update_target_v(targetSpeed,NowSpeed,&motor6_4);
        pid->error[0] = motor6_4.target_v_new - NowSpeed;
    }
    pid->integral += pid->error[0];
    pid->integral= limit(&pid->integral,80000);

    pid->derivative = pid->error[0] - pid->error[1];
    pid->derivative= limit(&pid->derivative,10000);

    pid->output = pid->vel_kp * pid->error[0] + pid->vel_ki *pid->integral + pid->vel_kd * pid->derivative;
    pid->output= limit(&pid->output,10000);

    pid->error[1] = pid->error[0];
    return pid->output;
}

