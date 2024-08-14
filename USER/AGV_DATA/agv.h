#ifndef LZ_PROJECT_3_0_AGV_H
#define LZ_PROJECT_3_0_AGV_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#define AGV_RVSIZE 26
void agv_init();
void state_control();
void read_agv_data();
uint8_t find_max(float arry[], uint8_t length);

extern uint8_t agv_right[4];
extern uint8_t agv_left[4];
extern float turn_angle;
extern uint8_t agvRvBuff[AGV_RVSIZE];
extern float V_AB;

#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_3_0_AGV_H
