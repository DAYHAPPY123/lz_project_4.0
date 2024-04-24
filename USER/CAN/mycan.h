#ifndef LZ_PROJECT_3_0_MYCAN_H
#define LZ_PROJECT_3_0_MYCAN_H
#include "stdint.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

extern float counter_change2006_1;
extern float counter_change2006_2;

void CAN_FilterInit();
void CAN_Init(FDCAN_HandleTypeDef *hfdcan);
void PortSendMotorsCur(int16_t cur0, int16_t cur1, int16_t cur2, int16_t cur3);

#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_3_0_MYCAN_H
