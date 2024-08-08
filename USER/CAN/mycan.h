#ifndef LZ_PROJECT_3_0_MYCAN_H
#define LZ_PROJECT_3_0_MYCAN_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "fdcan.h"
extern float counter_change2006_1;
extern float counter_change2006_2;

void CAN_FilterInit();
void CAN_Init(FDCAN_HandleTypeDef *hfdcan);
void PortSendMotorsCur(int16_t cur0, int16_t cur1, int16_t cur2, int16_t cur3);
void PortSendMotorsCur_6020(int16_t volt0, int16_t volt1, int16_t volt2, int16_t volt3);

#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_3_0_MYCAN_H
