#ifndef LZ_PROJECT_3_0_PID_H
#define LZ_PROJECT_3_0_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor.h"

struct PID_INIT{
    float kp;
    float ki;
    float kd;
    float now_error;
    float last_error;
    float integral;
    float derivative;
    float output;
    float error_max;
    float integral_max;
    float derivative_max;
    float output_max;
    float input;
    float target;
    float ramp;
};

class cPID
{
private:
    PID_INIT Spd;
    PID_INIT Pos;
public:
    motor_init motor;
    void Spd_Param_set(float kp,float ki,float kd);
    void Pos_Param_set(float kp,float ki,float kd);
    void ramp_Spd_set(float value);
    void limit_Spd_set(float error_max,float integral_max
            ,float derivative_max,float output_max);
    void ramp_Pos_set(float value);
    void limit_Pos_set(float error_max,float integral_max
            ,float derivative_max,float output_max);
    void update_target_p(float target_new,float target_now);
    void update_target_v(float target_new,float target_now);
    void PID_clear();
    float Spd_calculate(float targetSpeed,float NowSpeed);
    float Pos_calculate(float targetPos,float NowPos);
    float Spd_output_get();
    float Pos_output_get();
};

extern cPID PID3_1;
extern cPID PID3_2;
extern cPID PID2_1;
extern cPID PID2_2;

#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_3_0_PID_H
