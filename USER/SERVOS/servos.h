#ifndef LZ_PROJECT_3_0_SERVOS_H
#define LZ_PROJECT_3_0_SERVOS_H
#ifdef __cplusplus
extern "C" {
#endif


void servos_init();
void servos_stop();
void servos_start();
void servos_control();
void ADC_detect();

//正视
#define servo0_start 50 //右,越小越上
#define servo1_start 50 //左，越大越上
//50-113间取值

#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_3_0_SERVOS_H
