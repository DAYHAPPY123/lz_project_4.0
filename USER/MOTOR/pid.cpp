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

float ramp_step[3]={20,0.2,0.3};//2006pos-3508v-2006v
float pid_start=0;

cPID PID3_1;
cPID PID3_2;
cPID PID2_1;
cPID PID2_2;

void cPID::Spd_Param_set(float kp, float ki, float kd)
{
    Spd.kp=kp;
    Spd.ki=ki;
    Spd.kd=kd;
}

void cPID::Pos_Param_set(float kp, float ki, float kd)
{
    Pos.kp=kp;
    Pos.ki=ki;
    Pos.kd=kd;
}

void cPID::ramp_Spd_set(float value)
{
    Spd.ramp=value;
}

void cPID::limit_Spd_set(float error_max,float integral_max
        ,float derivative_max,float output_max)
{
    Spd.error_max=error_max;
    Spd.integral_max=integral_max;
    Spd.derivative_max=derivative_max;
    Spd.output_max=output_max;
}

void cPID::ramp_Pos_set(float value)
{
    Pos.ramp=value;
}

void cPID::limit_Pos_set(float error_max,float integral_max
        ,float derivative_max,float output_max)
{
    Pos.error_max=error_max;
    Pos.integral_max=integral_max;
    Pos.derivative_max=derivative_max;
    Pos.output_max=output_max;
}

void cPID::update_target_p(float target_new, float target_now)
    {
        if (target_now < target_new)
        {
            motor.target_pos_new += Pos.ramp;
            if (motor.target_pos_new > target_new)
                motor.target_pos_new = target_new;
        }
        else if (target_now > target_new)
        {
            motor.target_pos_new -= Pos.ramp;
            if (motor.target_pos_new < target_new)
                motor.target_pos_new = target_new;
        }
    }

void cPID::update_target_v(float target_new,float target_now)
{
    if (target_now < target_new)
    {
        motor.target_spd_new += Spd.ramp;
        if (motor.target_spd_new > target_new)
            motor.target_spd_new = target_new;
    }
    else if (target_now > target_new)
    {
        motor.target_spd_new -= Spd.ramp;
        if (motor.target_spd_new < target_new)
            motor.target_spd_new = target_new;
    }
}

void cPID::PID_clear()
{
    Spd.now_error=0;
    Spd.last_error=0;
    Spd.integral=0;
    Spd.derivative=0;
    Spd.output=0;
    Spd.input=0;
    Spd.target=0;

    Pos.now_error=0;
    Pos.last_error=0;
    Pos.integral=0;
    Pos.derivative=0;
    Pos.output=0;
    Pos.input=0;
    Pos.target=0;
}

float limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
    return *a;
}

float cPID::Spd_calculate(float targetSpeed, float NowSpeed)
{
    cPID::update_target_v(targetSpeed,NowSpeed);
    Spd.now_error = motor.target_spd_new - NowSpeed;
    Spd.integral += Spd.now_error;
    Spd.integral= limit(&Spd.integral,Spd.integral_max);

    Spd.derivative = Spd.now_error - Spd.last_error;
    Spd.derivative= limit(&Spd.derivative,Spd.derivative_max);

    Spd.output = Spd.kp * Spd.now_error + Spd.ki *Spd.integral
                  + Spd.kd * Spd.derivative;
    Spd.output= limit(&Spd.output,Spd.output_max);

    Spd.last_error = Spd.now_error;

    return Spd.output;
}

float cPID::Pos_calculate(float targetPos, float NowPos)
{
    cPID::update_target_p(targetPos,NowPos);
    Pos.now_error = motor.target_pos_new - NowPos;
    Pos.integral += Pos.now_error;
    Pos.integral= limit(&Pos.integral,Pos.integral_max);

    Pos.derivative = Pos.now_error - Pos.last_error;
    Pos.derivative= limit(&Pos.derivative,Pos.derivative_max);

    Pos.output = Pos.kp * Pos.now_error + Pos.ki *Pos.integral
                 + Pos.kd * Pos.derivative;
    Pos.output= limit(&Pos.output,Pos.output_max);

    Pos.last_error = Pos.now_error;

    return Pos.output;
}

float cPID::Spd_output_get()
{
    return Spd.output;
}

float cPID::Pos_output_get()
{
    return Pos_output_get();
};