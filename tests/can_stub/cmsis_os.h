#ifndef TEST_CMSIS_OS_H
#define TEST_CMSIS_OS_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
    osOK = 0,
    osError = -1,
    osErrorTimeout = -2,
    osErrorResource = -3,
    osErrorParameter = -4
} osStatus_t;

typedef struct osMessageQueue *osMessageQueueId_t;
typedef struct osSemaphore *osSemaphoreId_t;

typedef struct { const char *name; } osMessageQueueAttr_t;
typedef struct { const char *name; } osSemaphoreAttr_t;

#define osWaitForever 0xFFFFFFFFu

typedef uint32_t UBaseType_t;
#ifdef __cplusplus
extern "C" {
#endif
extern int test_in_isr;
extern int test_isr_critical_errors;
void test_task_enter_critical(void);
void test_task_exit_critical(void);
UBaseType_t test_isr_enter_critical(void);
void test_isr_exit_critical(UBaseType_t saved);
#ifdef __cplusplus
}
#endif

#define taskENTER_CRITICAL() test_task_enter_critical()
#define taskEXIT_CRITICAL() test_task_exit_critical()
#define taskENTER_CRITICAL_FROM_ISR() test_isr_enter_critical()
#define taskEXIT_CRITICAL_FROM_ISR(saved) test_isr_exit_critical(saved)

#ifdef __cplusplus
extern "C" {
#endif

osMessageQueueId_t osMessageQueueNew(uint32_t count, uint32_t item_size,
                                     const osMessageQueueAttr_t *attr);
osStatus_t osMessageQueuePut(osMessageQueueId_t queue, const void *msg,
                             uint8_t prio, uint32_t timeout);
osStatus_t osMessageQueueGet(osMessageQueueId_t queue, void *msg,
                             uint8_t *prio, uint32_t timeout);
osStatus_t osMessageQueueReset(osMessageQueueId_t queue);

osSemaphoreId_t osSemaphoreNew(uint32_t max_count, uint32_t initial_count,
                               const osSemaphoreAttr_t *attr);
osStatus_t osSemaphoreAcquire(osSemaphoreId_t sem, uint32_t timeout);
osStatus_t osSemaphoreRelease(osSemaphoreId_t sem);

uint32_t osKernelGetTickCount(void);
void osDelay(uint32_t ticks);
void osDelayUntil(uint32_t ticks);

#ifdef __cplusplus
}
#endif

#endif
