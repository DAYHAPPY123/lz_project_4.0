#ifndef LZ_PROJECT_3_0_PID_H
#define LZ_PROJECT_3_0_PID_H

#ifdef __cplusplus
extern "C" {
#endif

struct PID_INIT{
    float vel_kp;
    float vel_ki;
    float vel_kd;
    float error[2];
    float integral;
    float derivative;

    float output;

    float pos_kp_strong;
    float pos_ki_strong;
    float pos_kd_strong;

    float pos_kp_wake;
    float pos_ki_wake;
    float pos_kd_wake;
};

extern struct PID_INIT pid3_1;
extern struct PID_INIT pid3_2;
extern struct PID_INIT pid2_1;
extern struct PID_INIT pid2_2;
extern struct PID_INIT pid_reset1;
extern struct PID_INIT pid_reset2;

extern float limit(float *a, float ABS_MAX);

void update_target_pos(float* ramp_target,float* target_now);
int16_t PIDControl_3508(struct PID_INIT* pid, float targetSpeed,float NowSpeed);
int16_t PIDControl_2006_pos(struct PID_INIT* pid,float targetPos,float NowPos);
int16_t PIDControl_2006_v(struct PID_INIT* pid,float targetSpeed,float NowSpeed);

#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_3_0_PID_H
