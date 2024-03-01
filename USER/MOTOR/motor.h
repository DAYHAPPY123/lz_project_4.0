#ifndef LZ_PROJECT_3_0_MOTOR_H
#define LZ_PROJECT_3_0_MOTOR_H
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

extern float left_angle;
extern float right_angle;
extern int16_t left_counter;
extern int16_t right_counter;
extern uint8_t mode;

#define car_length 200         //前后轮间距
#define car_width 260.9  //两后轮之间的间距259.4 262.4
#define PI 3.1415926
#define mid_counter_3 3200
#define mid_counter_4 (-3000)
#define MOTOR_MANUAL 0
#define MOTOR_AUTO 1
#define MOTOR_STOP 2

struct motor_init
{
    float set_rpm;
    int16_t vel;

    int16_t set_pos;
    int16_t pos;

    int16_t current;

    int16_t tmp;
    float rpm;

    double continuous;
    float calculate_continuous;
    float target_pos_new;
    float target_v_new;
    int online_flag;
};

typedef enum
{
    CAN_CHASSIS_Drive_ID = 0x200,

    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,

    CAN_CHASSIS_Turn_ID = 0x200,

    CAN_2006_M1_ID = 0x203,
    CAN_2006_M2_ID = 0x204,
} eCanMessageID;

void motor_io_init();
void angle_cal();
void backwheel_speed_cal(void);
void Speed_Send(void);
void motor_reset();

extern struct motor_init motor3_1;
extern struct motor_init motor3_2;
extern struct motor_init motor2_1;
extern struct motor_init motor2_2;

#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_3_0_MOTOR_H
