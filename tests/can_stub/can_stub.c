#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include <string.h>
#include <windows.h>

struct osMessageQueue {
    uint8_t data[16][64];
    uint32_t item_size;
    uint32_t count;
    uint32_t head;
    uint32_t tail;
};

struct osSemaphore {
    uint32_t count;
    uint32_t max;
};

CAN_HandleTypeDef hcan1 = {0};
CAN_HandleTypeDef hcan2 = {0};
CAN_RxHeaderTypeDef test_rx_header;
uint8_t test_rx_data[8];
HAL_StatusTypeDef test_rx_status = HAL_OK;
HAL_StatusTypeDef test_start_status1 = HAL_OK;
HAL_StatusTypeDef test_start_status2 = HAL_OK;
HAL_StatusTypeDef test_notify_status = HAL_OK;
HAL_StatusTypeDef test_tx_status = HAL_OK;
uint32_t test_tx_count;
uint32_t test_tick;
uint32_t test_queue_get_calls;
uint32_t test_sem_acquire_count;
int test_queue_block_enabled;
int test_queue_block_release;
int test_in_isr;
int test_isr_critical_errors;

void test_task_enter_critical(void) {
    if (test_in_isr) { test_isr_critical_errors++; }
}
void test_task_exit_critical(void) {
    if (test_in_isr) { test_isr_critical_errors++; }
}
UBaseType_t test_isr_enter_critical(void) {
    return 0u;
}
void test_isr_exit_critical(UBaseType_t saved) {
    (void)saved;
}

static struct osMessageQueue q1 = {.item_size = 64u};
static struct osMessageQueue q2 = {.item_size = 64u};
static struct osSemaphore s1 = {3u, 3u};
static struct osSemaphore s2 = {3u, 3u};
osMessageQueueId_t can1_send_fifoHandle = &q1;
osMessageQueueId_t can2_send_fifoHandle = &q2;
osSemaphoreId_t CAN1CountingSemHandle = &s1;
osSemaphoreId_t CAN2CountingSemHandle = &s2;

osMessageQueueId_t osMessageQueueNew(uint32_t count, uint32_t item_size,
                                     const osMessageQueueAttr_t *attr) {
    (void)count; (void)item_size; (void)attr;
    return &q1;
}

osStatus_t osMessageQueuePut(osMessageQueueId_t queue, const void *msg,
                             uint8_t prio, uint32_t timeout) {
    (void)prio; (void)timeout;
    if (queue == NULL || msg == NULL || queue->count >= 16u) { return osErrorResource; }
    memcpy(queue->data[queue->tail], msg, queue->item_size);
    queue->tail = (queue->tail + 1u) % 16u;
    queue->count++;
    return osOK;
}

osStatus_t osMessageQueueGet(osMessageQueueId_t queue, void *msg,
                             uint8_t *prio, uint32_t timeout) {
    (void)prio;
    if (queue == NULL || msg == NULL) { return osErrorResource; }
    if (queue->count == 0u) {
        test_queue_get_calls++;
        if (timeout == osWaitForever && test_queue_block_enabled) {
            while (!test_queue_block_release) { Sleep(1); }
            test_queue_block_release = 0;
        }
        return osErrorResource;
    }
    memcpy(msg, queue->data[queue->head], queue->item_size);
    queue->head = (queue->head + 1u) % 16u;
    queue->count--;
    return osOK;
}

osStatus_t osMessageQueueReset(osMessageQueueId_t queue) {
    if (queue == NULL) { return osErrorParameter; }
    queue->count = 0u; queue->head = 0u; queue->tail = 0u;
    return osOK;
}

osSemaphoreId_t osSemaphoreNew(uint32_t max_count, uint32_t initial_count,
                               const osSemaphoreAttr_t *attr) {
    (void)max_count; (void)initial_count; (void)attr;
    return &s1;
}

osStatus_t osSemaphoreAcquire(osSemaphoreId_t sem, uint32_t timeout) {
    (void)timeout;
    if (sem == NULL || sem->count == 0u) { return osErrorTimeout; }
    test_sem_acquire_count++;
    sem->count--;
    return osOK;
}

osStatus_t osSemaphoreRelease(osSemaphoreId_t sem) {
    if (sem == NULL) { return osErrorParameter; }
    if (sem->count < sem->max) { sem->count++; }
    return osOK;
}

uint32_t osKernelGetTickCount(void) { return test_tick; }
void osDelay(uint32_t ticks) { test_tick += ticks; }
void osDelayUntil(uint32_t ticks) { test_tick = ticks; }

void test_can_reset_rx(void) {
    memset(&test_rx_header, 0, sizeof(test_rx_header));
    memset(test_rx_data, 0, sizeof(test_rx_data));
    test_rx_status = HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan) {
    return (hcan == &hcan1) ? test_start_status1 : test_start_status2;
}
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan) { (void)hcan; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t its) {
    (void)hcan; (void)its;
    return test_notify_status;
}
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan,
                                       const CAN_FilterTypeDef *filter) {
    (void)hcan; (void)filter;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t fifo,
                                       CAN_RxHeaderTypeDef *header, uint8_t data[8]) {
    (void)hcan; (void)fifo;
    if (test_rx_status != HAL_OK) { return test_rx_status; }
    *header = test_rx_header;
    memcpy(data, test_rx_data, 8u);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan,
                                       const CAN_TxHeaderTypeDef *header,
                                       const uint8_t data[8], uint32_t *mailbox) {
    (void)hcan; (void)header; (void)data;
    if (test_tx_status != HAL_OK) { return test_tx_status; }
    test_tx_count++;
    if (mailbox != NULL) { *mailbox = 0u; }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, uint32_t mailboxes) {
    (void)hcan; (void)mailboxes;
    return HAL_OK;
}

void test_can_set_queue_item_size(uint32_t size) {
    q1.item_size = size;
    q2.item_size = size;
}
