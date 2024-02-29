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
        .pos_kp_strong=2.2f,
        .pos_ki_strong=0.04f,
        .pos_kd_strong=20.0f,

        .pos_kp_wake=2.2f,
        .pos_ki_wake=0.04f,
        .pos_kd_wake=20.0f,
};

PID_INIT pid2_2={
        .pos_kp_strong=2.2f,
        .pos_ki_strong=0.04f,
        .pos_kd_strong=20.0f,

        .pos_kp_wake=2.2f,
        .pos_ki_wake=0.04f,
        .pos_kd_wake=20.0f,
};

PID_INIT pid_reset1={
        .vel_kp=70.0f,
        .vel_ki=0.06f,
        .vel_kd=0.0f
};

PID_INIT pid_reset2={
        .vel_kp=50.0f,
        .vel_ki=0.04f,
        .vel_kd=0.0f
};

float ramp_step[3]={30,0.15,0.15};//2006pos-3508v-2006v

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
    if( (motor==&motor6_3)||(motor==&motor6_4) )
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

int16_t PIDControl_3508(struct PID_INIT* pid, float targetSpeed,float NowSpeed)
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

int16_t PIDControl_2006_pos(struct PID_INIT* pid,float targetPos,float NowPos)
{
    if (((mode == MOTOR_AUTO) && (turn_angle < 0)) || ((mode == MOTOR_MANUAL) && (rc_ctrl.rc.ch[2] < 0)))//向左转，转角1大，2小
    {
        if (pid == &pid2_1) {
            update_target_pos(targetPos, NowPos, &motor6_3);
            pid->error[0] = motor6_3.target_pos_new - NowPos;
            pid->integral += pid->error[0];
            pid->integral = limit(&pid->integral, 100000);

            pid->derivative = pid->error[0] - pid->error[1];
            pid->derivative = limit(&pid->derivative, 10000);

            pid->output = pid->pos_kp_strong * pid->error[0] + pid->pos_ki_strong * pid->integral +
                          pid->pos_kd_strong * pid->derivative;
            pid->output = limit(&pid->output, 10000);
            pid->error[1] = pid->error[0];
        } else if (pid == &pid2_2) {
            update_target_pos(targetPos, NowPos, &motor6_4);
            pid->error[0] = motor6_4.target_pos_new - NowPos;
            pid->integral += pid->error[0];
            pid->integral = limit(&pid->integral, 100000);

            pid->derivative = pid->error[0] - pid->error[1];
            pid->derivative = limit(&pid->derivative, 10000);

            pid->output = pid->pos_kp_wake * pid->error[0] + pid->pos_ki_wake * pid->integral +
                          pid->pos_kd_wake * pid->derivative;
            pid->output = limit(&pid->output, 10000);
            pid->error[1] = pid->error[0];
        }
    } else if (((mode == MOTOR_AUTO) && (turn_angle > 0)) ||((mode == MOTOR_MANUAL) && (rc_ctrl.rc.ch[2] > 0)))//向右转，转角2大，1小
    {
        if (pid == &pid2_2) {
            update_target_pos(targetPos, NowPos, &motor6_4);
            pid->error[0] = motor6_4.target_pos_new - NowPos;
            pid->integral += pid->error[0];
            pid->integral = limit(&pid->integral, 100000);

            pid->derivative = pid->error[0] - pid->error[1];
            pid->derivative = limit(&pid->derivative, 10000);

            pid->output = pid->pos_kp_strong * pid->error[0] + pid->pos_ki_strong * pid->integral +
                          pid->pos_kd_strong * pid->derivative;
            pid->output = limit(&pid->output, 10000);
            pid->error[1] = pid->error[0];
        } else if (pid == &pid2_1) {
            update_target_pos(targetPos, NowPos, &motor6_3);
            pid->error[0] = motor6_3.target_pos_new - NowPos;
            pid->integral += pid->error[0];
            pid->integral = limit(&pid->integral, 100000);

            pid->derivative = pid->error[0] - pid->error[1];
            pid->derivative = limit(&pid->derivative, 10000);

            pid->output = pid->pos_kp_wake * pid->error[0] + pid->pos_ki_wake * pid->integral +
                          pid->pos_kd_wake * pid->derivative;
            pid->output = limit(&pid->output, 10000);
            pid->error[1] = pid->error[0];
        }
    } else if (((mode == MOTOR_AUTO) && (turn_angle = 0)) ||((mode == MOTOR_MANUAL) && (rc_ctrl.rc.ch[2] = 0)))//不转
    {
        if (pid == &pid2_1) {
            update_target_pos(targetPos, NowPos, &motor6_3);
            pid->error[0] = motor6_3.target_pos_new - NowPos;
            pid->integral += pid->error[0];
            pid->integral = limit(&pid->integral, 100000);

            pid->derivative = pid->error[0] - pid->error[1];
            pid->derivative = limit(&pid->derivative, 10000);

            pid->output = pid->pos_kp_strong * pid->error[0] + pid->pos_ki_strong * pid->integral +
                          pid->pos_kd_strong * pid->derivative;
            pid->output = limit(&pid->output, 10000);
            pid->error[1] = pid->error[0];
        } else if (pid == &pid2_2) {
            update_target_pos(targetPos, NowPos, &motor6_4);
            pid->error[0] = motor6_4.target_pos_new - NowPos;
            pid->integral += pid->error[0];
            pid->integral = limit(&pid->integral, 100000);

            pid->derivative = pid->error[0] - pid->error[1];
            pid->derivative = limit(&pid->derivative, 10000);

            pid->output = pid->pos_kp_strong * pid->error[0] + pid->pos_ki_strong * pid->integral +
                          pid->pos_kd_strong * pid->derivative;
            pid->output = limit(&pid->output, 10000);
            pid->error[1] = pid->error[0];
        }
    }
    return pid->output;
}

int16_t PIDControl_2006_v(struct PID_INIT* pid,float targetSpeed,float NowSpeed)
{
    if (pid==&pid_reset1)
    {
        update_target_v(targetSpeed,NowSpeed,&motor6_3);
        pid->error[0] = motor6_3.target_v_new - NowSpeed;
    } else if(pid==&pid_reset2)
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

