#ifndef LZ_PROJECT_3_0_TASK_H
#define LZ_PROJECT_3_0_MYTASK_H
#ifdef __cplusplus
extern "C" {
#endif

#include "cstdint"
void init_task(void *argument);
void control_task(void *argument);
void printf_task(void *argument);
void agv_task(void *argument);
void error_task(void *argument);

#ifdef __cplusplus
}
#endif
#endif //LZ_PROJECT_3_0_TASK_H
