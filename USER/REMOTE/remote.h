#ifndef LZ_PROJECT_3_0_REMOTE_H
#define LZ_PROJECT_3_0_REMOTE_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SBUS_RX_BUF_NUM 50u
#define RC_FRAME_LENGTH 25u

#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)

#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)

typedef struct
{
    int16_t ch[16];
} RC_ctrl_t;

void REMOTEC_Init(void);
void REMOTEC_UartIrqHandler(void);
void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
extern RC_ctrl_t rc_ctrl;
void check_rc_connection();
extern uint8_t rc_start;

#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_3_0_REMOTE_H
