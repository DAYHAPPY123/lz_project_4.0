#ifndef LZ_PROJECT_3_0_SERVOS_H
#define LZ_PROJECT_3_0_SERVOS_H
#ifdef __cplusplus
extern "C" {
#endif


void servos_init();
void servos_stop();
void servos_start();
void servos_control();
void servos_reset();

//正视
//#define servo0_start 50 //右,越大越下
//#define servo1_start 178 //左，越小越下

#define servo0_start 140 //左舵机,越大越下
#define servo1_start 110 //右舵机，越小越下

#define servos_speed 0.3
//50-116间取值

extern int actual_pos_input[2];
extern float angle_limit;

#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_3_0_SERVOS_H
