#ifndef LZ_PROJECT_3_0_TASK_H
#define LZ_PROJECT_3_0_TASK_H
#include "cstdint"

#ifdef __cplusplus
extern "C" {
#endif

void Init_Task(void *argument);
void control_task(void *argument);
void printf_task(void *argument);
void agv_task(void *argument);
void adc_task(void *argument);

#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_3_0_TASK_H
