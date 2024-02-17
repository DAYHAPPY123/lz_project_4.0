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

    float pos_kp;
    float pos_ki;
    float pos_kd;
};

#define STARTPID  0x70          //p

#define VEL_LOOP 0x73
#define VEL_KP 0x70            //p
#define VEL_KI 0x69            //i
#define VEL_KD 0x64            //d
#define VEL_MAXOUT 0x6F        //o
#define VEL_MAXINTEGRAL 0x61   //a

#define POS_LOOP 0x70           //p
#define POS_KP 0x70            //p
#define POS_KI 0x69            //i
#define POS_KD 0x64            //d

extern struct PID_INIT pid1;
extern struct PID_INIT pid2;
extern struct PID_INIT pid3;
extern struct PID_INIT pid4;
extern struct PID_INIT pid3_2;
extern struct PID_INIT pid4_2;

extern float limit(float a, float ABS_MAX);

void update_target(float* ramp_target,float* target_now);
int16_t PIDControl_3508(struct PID_INIT* pid, float targetSpeed,float NowSpeed);
int16_t PIDControl_2006_pos(struct PID_INIT* pid,float targetPos,float NowPos);
int16_t PIDControl_2006_v(struct PID_INIT* pid,float targetSpeed,float NowSpeed);

#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_3_0_PID_H
