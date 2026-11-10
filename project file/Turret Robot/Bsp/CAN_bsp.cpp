//
// Created by Tessia on 2023/12/10.
//

#include "CAN_bsp.h"
#include "Common/app_time.h"

#define MAX_CAN_DEV_NUM (14 * 4)
#define CAN2_FILTER_BANK_OFFSET 14u
#define CAN_RECOVERY_PERIOD_MS 500u

can_device_receive *can_device_rx_list[2][MAX_CAN_DEV_NUM] = {0};

CAN_FilterTypeDef filter_cfg = {
    .FilterMode = CAN_FILTERMODE_IDLIST,
    .FilterScale = CAN_FILTERSCALE_16BIT,
    .FilterActivation = ENABLE,
    .SlaveStartFilterBank = CAN2_FILTER_BANK_OFFSET
};
uint32_t can1_filter[14][4] = {0};
uint32_t can1_device_num = 0;
uint32_t can2_filter[14][4] = {0};
uint32_t can2_device_num = 0;

extern osMessageQueueId_t can1_send_fifoHandle;
extern osMessageQueueId_t can2_send_fifoHandle;
extern osSemaphoreId_t CAN1CountingSemHandle;
extern osSemaphoreId_t CAN2CountingSemHandle;

static can_bsp_stats_t g_stats;
static bool g_started;
static uint16_t g_can1_inflight;
static uint16_t g_can2_inflight;
static uint32_t g_last_recovery_ms;
static bool g_bus_off;
static can_bsp_std_rx_observer_t g_observers[4];
static can_device_receive *g_std_receivers[2];
static uint32_t g_std_ids[2];
static volatile uint32_t g_tx_generation;
static bool g_can1_started;
static bool g_can2_started;
static uint32_t g_last_start_attempt_ms;
static volatile bool g_services_run = true;

static void stats_increment_task(uint32_t *field)
{
    taskENTER_CRITICAL();
    (*field)++;
    taskEXIT_CRITICAL();
}

static void stats_increment_isr(uint32_t *field)
{
    const UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
    (*field)++;
    taskEXIT_CRITICAL_FROM_ISR(saved);
}

static void filter_from_slot(uint32_t filter[14][4], uint32_t index)
{
    filter_cfg.FilterIdLow = filter[index / 4u][0] << 5;
    filter_cfg.FilterMaskIdLow = filter[index / 4u][1] << 5;
    filter_cfg.FilterIdHigh = filter[index / 4u][2] << 5;
    filter_cfg.FilterMaskIdHigh = filter[index / 4u][3] << 5;
}

/************************** transmit **************************/

can_device_transmit::can_device_transmit(CAN_HandleTypeDef *hcan_, uint32_t id_,
                                         uint8_t len_, uint8_t *buf_data_)
    : member{} {
    member.hcan = hcan_;
    member.id = id_;
    member.len = (len_ > 8u) ? 8u : len_;
    if (buf_data_ != nullptr) {
        (void)memcpy(member.buf_data, buf_data_, member.len);
    }
}

can_device_transmit::can_device_transmit() : member{} {
    member.hcan = &hcan1;
    member.len = 8u;
}

bool can_device_transmit::set_buf_address(uint8_t *buf_address) {
    if (buf_address == nullptr) { return false; }
    (void)memcpy(member.buf_data, buf_address, member.len);
    return true;
}

bool can_device_transmit::set_data(const uint8_t *data, uint8_t len) {
    if (data == nullptr || len > 8u) { return false; }
    member.len = len;
    (void)memset(member.buf_data, 0, sizeof(member.buf_data));
    (void)memcpy(member.buf_data, data, len);
    return true;
}

bool can_device_transmit::set_id(uint32_t id_) {
    if (id_ > 0x7FFu) { return false; }
    member.id = id_;
    return true;
}

osStatus_t can_device_transmit::can_send_msg(void) {
    osStatus_t state;
    if (!g_started || member.hcan == nullptr || member.len > 8u ||
        member.id > 0x7FFu) {
        stats_increment_task(&g_stats.tx_dropped);
        return osErrorParameter;
    }
    osMessageQueueId_t queue = nullptr;
    if (member.hcan == &hcan1) {
        queue = can1_send_fifoHandle;
    } else if (member.hcan == &hcan2) {
        queue = can2_send_fifoHandle;
    }
    if (queue == nullptr) {
        stats_increment_task(&g_stats.tx_dropped);
        return osErrorResource;
    }
    member.generation = g_tx_generation;
    stats_increment_task(&g_stats.tx_requests);
    state = osMessageQueuePut(queue, &member, 0u, 0u);
    if (state != osOK) {
        stats_increment_task(&g_stats.tx_queue_full);
        stats_increment_task(&g_stats.tx_dropped);
    }
    return state;
}

/************************** transmit tasks **************************/

static void send_one(CAN_HandleTypeDef *hcan, osMessageQueueId_t queue,
                     osSemaphoreId_t semaphore, uint16_t *inflight) {
    can_device_transmit_member msg = {0};
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox = 0u;

    /* Block until a message exists; never spin on an empty queue. */
    if (osMessageQueueGet(queue, &msg, nullptr, osWaitForever) != osOK) {
        return;
    }
    if (msg.generation != g_tx_generation) {
        stats_increment_task(&g_stats.tx_dropped);
        return;
    }
    if (osSemaphoreAcquire(semaphore, osWaitForever) != osOK) {
        stats_increment_task(&g_stats.tx_fail);
        return;
    }
    /* Flush/stop can happen while waiting for a mailbox. Re-check after acquire
     * so a stale dequeued command is never put on the bus. */
    if (!g_started || !g_services_run || msg.generation != g_tx_generation ||
        msg.hcan != hcan || msg.len > 8u || msg.id > 0x7FFu) {
        (void)osSemaphoreRelease(semaphore);
        stats_increment_task(&g_stats.tx_dropped);
        return;
    }

    tx_header.StdId = msg.id;
    tx_header.DLC = msg.len;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;

    taskENTER_CRITICAL();
    (*inflight)++;
    taskEXIT_CRITICAL();

    const HAL_StatusTypeDef status =
        HAL_CAN_AddTxMessage(hcan, &tx_header, msg.buf_data, &tx_mailbox);
    if (status != HAL_OK) {
        taskENTER_CRITICAL();
        if (*inflight > 0u) { (*inflight)--; }
        g_stats.tx_fail++;
        if (g_stats.tx_fail_streak < UINT32_MAX) { g_stats.tx_fail_streak++; }
        taskEXIT_CRITICAL();
        (void)osSemaphoreRelease(semaphore);
    } else {
        taskENTER_CRITICAL();
        g_stats.tx_ok++;
        g_stats.tx_fail_streak = 0u;
        taskEXIT_CRITICAL();
    }
}

void can1_sending_service(void *argument) {
    (void)argument;
    while (g_services_run) {
        if (!g_started) { osDelay(10); continue; }
        send_one(&hcan1, can1_send_fifoHandle, CAN1CountingSemHandle,
                 &g_can1_inflight);
    }
}

void can2_sending_service(void *argument) {
    (void)argument;
    while (g_services_run) {
        if (!g_started) { osDelay(10); continue; }
        send_one(&hcan2, can2_send_fifoHandle, CAN2CountingSemHandle,
                 &g_can2_inflight);
    }
}

static void tx_complete(CAN_HandleTypeDef *hcan, bool aborted) {
    osSemaphoreId_t semaphore = nullptr;
    uint16_t *inflight = nullptr;
    bool release = false;

    if (hcan == &hcan1) {
        semaphore = CAN1CountingSemHandle;
        inflight = &g_can1_inflight;
    } else if (hcan == &hcan2) {
        semaphore = CAN2CountingSemHandle;
        inflight = &g_can2_inflight;
    } else {
        return;
    }

    const UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
    if (*inflight > 0u) {
        (*inflight)--;
        release = true;
    } else {
        g_stats.tx_unexpected_completion++;
    }
    if (aborted) { g_stats.tx_aborted++; }
    taskEXIT_CRITICAL_FROM_ISR(saved);

    if (release && semaphore != nullptr) {
        (void)osSemaphoreRelease(semaphore);
    }
}

void can_tx_complete_callback(CAN_HandleTypeDef *hcan) {
    tx_complete(hcan, false);
}

static void can_tx_abort_callback(CAN_HandleTypeDef *hcan) {
    tx_complete(hcan, true);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) {
    can_tx_abort_callback(hcan);
}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) {
    can_tx_abort_callback(hcan);
}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) {
    can_tx_abort_callback(hcan);
}

/************************** receive **************************/

can_device_receive::can_device_receive(CAN_HandleTypeDef *hcan_, uint32_t id_,
                                       can_rx_callback *callback_)
    : rx_data{}, rx_dlc(0u), callback(callback_), is_callback_none(0), registered(false),
      hcan(hcan_), id(id_), index(0u) {
    if ((hcan != &hcan1 && hcan != &hcan2) || id > 0x7FFu) { return; }

    taskENTER_CRITICAL();
    uint32_t *device_num = (hcan == &hcan1) ? &can1_device_num : &can2_device_num;
    uint32_t (*filter)[4] = (hcan == &hcan1) ? can1_filter : can2_filter;
    const uint32_t bus = (hcan == &hcan1) ? 0u : 1u;
    if (*device_num >= MAX_CAN_DEV_NUM) {
        taskEXIT_CRITICAL();
        return;
    }
    for (uint32_t i = 0u; i < *device_num; ++i) {
        can_device_receive *existing = can_device_rx_list[bus][i];
        if (existing != nullptr && existing->matches(hcan, id)) {
            taskEXIT_CRITICAL();
            return;
        }
    }
    index = *device_num;
    filter[index / 4u][index % 4u] = id;
    filter_cfg.FilterFIFOAssignment =
        (hcan == &hcan1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    filter_cfg.FilterBank = (index / 4u) +
                            ((hcan == &hcan1) ? 0u : CAN2_FILTER_BANK_OFFSET);
    filter_from_slot(filter, index);
    const HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(hcan, &filter_cfg);
    if (status == HAL_OK) {
        can_device_rx_list[bus][index] = this;
        registered = true;
        (*device_num)++;
    }
    taskEXIT_CRITICAL();
}

bool can_device_receive::can_set_callback(can_rx_callback *callback_) {
    callback = callback_;
    return true;
}

void can_device_receive::can_modify_id(uint32_t id_) {
    if (!registered || id_ > 0x7FFu) { return; }
    taskENTER_CRITICAL();
    id = id_;
    uint32_t (*filter)[4] = (hcan == &hcan1) ? can1_filter : can2_filter;
    filter[index / 4u][index % 4u] = id;
    filter_from_slot(filter, index);
    filter_cfg.FilterFIFOAssignment =
        (hcan == &hcan1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    filter_cfg.FilterBank = (index / 4u) +
                            ((hcan == &hcan1) ? 0u : CAN2_FILTER_BANK_OFFSET);
    if (HAL_CAN_ConfigFilter(hcan, &filter_cfg) != HAL_OK) {
        registered = false;
        can_device_rx_list[(hcan == &hcan1) ? 0u : 1u][index] = nullptr;
    }
    taskEXIT_CRITICAL();
}

bool can_device_receive::matches(CAN_HandleTypeDef *hcan_, uint32_t id_) const {
    return registered && (hcan == hcan_) && (id == id_);
}

static can_device_receive *find_rx_device(CAN_HandleTypeDef *hcan, uint32_t id) {
    if (hcan != &hcan1 && hcan != &hcan2) { return nullptr; }
    const uint32_t bus = (hcan == &hcan1) ? 0u : 1u;
    const uint32_t count = (hcan == &hcan1) ? can1_device_num : can2_device_num;
    for (uint32_t i = 0u; i < count && i < MAX_CAN_DEV_NUM; ++i) {
        can_device_receive *dev = can_device_rx_list[bus][i];
        if (dev != nullptr && dev->matches(hcan, id)) { return dev; }
    }
    return nullptr;
}

static void handle_rx(CAN_HandleTypeDef *hcan, uint32_t fifo, uint32_t bus) {
    CAN_RxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    const uint32_t arrival_ms = app_time_now_ms();

    if (hcan != &hcan1 && hcan != &hcan2) {
        stats_increment_isr(&g_stats.rx_bad);
        return;
    }
    if (HAL_CAN_GetRxMessage(hcan, fifo, &header, data) != HAL_OK) {
        stats_increment_isr(&g_stats.rx_bad);
        return;
    }
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
        header.DLC > 8u) {
        stats_increment_isr(&g_stats.rx_bad);
        return;
    }

    /* FilterMatchIndex is a filter-bank match index, not our registration slot.
     * Look up by the validated CAN instance and standard ID instead of guessing
     * a CAN2 offset. */
    can_device_receive *dev = find_rx_device(hcan, header.StdId);
    if (dev == nullptr) {
        stats_increment_isr(&g_stats.rx_unmatched);
        return;
    }
    (void)memset(dev->rx_data, 0, sizeof(dev->rx_data));
    dev->rx_dlc = (uint8_t)header.DLC;
    (void)memcpy(dev->rx_data, data, header.DLC);

    bool delivered = false;
    for (uint32_t i = 0u; i < 4u; ++i) {
        if (g_observers[i] != nullptr) {
            delivered = true;
            g_observers[i]((uint8_t)(bus + 1u), header.StdId, data,
                           (uint8_t)header.DLC, arrival_ms);
        }
    }
    if (dev->callback != nullptr) {
        delivered = true;
        dev->is_callback_none = 0u;
        dev->callback(dev, dev->rx_data);
    } else {
        dev->is_callback_none = 1u;
    }
    if (delivered) {
        stats_increment_isr(&g_stats.rx_ok);
    } else {
        stats_increment_isr(&g_stats.rx_no_callback);
    }
    (void)bus;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    handle_rx(hcan, CAN_RX_FIFO0, 0u);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    handle_rx(hcan, CAN_RX_FIFO1, 1u);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == nullptr) { return; }
    stats_increment_isr(&g_stats.errors);
    if ((hcan->ErrorCode & HAL_CAN_ERROR_BOF) != 0u) {
        const UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
        g_bus_off = true;
        if (g_stats.bus_off < UINT32_MAX) { g_stats.bus_off++; }
        taskEXIT_CRITICAL_FROM_ISR(saved);
    }
}

/************************** public API **************************/

static bool start_one(CAN_HandleTypeDef *hcan, uint32_t fifo_notification) {
    if (HAL_CAN_Start(hcan) != HAL_OK) { return false; }
    const HAL_StatusTypeDef notify = HAL_CAN_ActivateNotification(
        hcan, fifo_notification | CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR |
                  CAN_IT_BUSOFF | CAN_IT_ERROR_PASSIVE | CAN_IT_ERROR_WARNING |
                  CAN_IT_LAST_ERROR_CODE);
    if (notify != HAL_OK) {
        (void)HAL_CAN_Stop(hcan);
        return false;
    }
    return true;
}

bool can_bsp_start(void) {
    g_services_run = true;
    if (g_started) { return true; }
    if (can1_send_fifoHandle == nullptr || can2_send_fifoHandle == nullptr ||
        CAN1CountingSemHandle == nullptr || CAN2CountingSemHandle == nullptr) {
        stats_increment_task(&g_stats.start_failures);
        return false;
    }
    stats_increment_task(&g_stats.start_attempts);
    if (!g_can1_started) {
        g_can1_started = start_one(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    if (!g_can2_started) {
        g_can2_started = start_one(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
    g_started = g_can1_started && g_can2_started;
    if (!g_started) {
        stats_increment_task(&g_stats.start_failures);
        return false;
    }
    taskENTER_CRITICAL();
    g_stats.started = true;
    taskEXIT_CRITICAL();
    return true;
}

bool can_bsp_is_started(void) { return g_started; }

void can_bsp_service(uint32_t now_ms) {
    if (!g_started) {
        if ((uint32_t)(now_ms - g_last_start_attempt_ms) >= CAN_RECOVERY_PERIOD_MS) {
            g_last_start_attempt_ms = now_ms;
            (void)can_bsp_start();
        }
        return;
    }
    if (!g_bus_off) { return; }
    if ((uint32_t)(now_ms - g_last_recovery_ms) < CAN_RECOVERY_PERIOD_MS) {
        return;
    }
    g_last_recovery_ms = now_ms;
    stats_increment_task(&g_stats.recovery_attempts);
    (void)HAL_CAN_Stop(&hcan1);
    (void)HAL_CAN_Stop(&hcan2);
    g_can1_started = start_one(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    g_can2_started = start_one(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
    if (g_can1_started && g_can2_started) {
        g_started = true;
        taskENTER_CRITICAL();
        g_bus_off = false;
        g_stats.recovered++;
        taskEXIT_CRITICAL();
    } else {
        g_started = false;
        stats_increment_task(&g_stats.recovery_failures);
    }
}

void can_bsp_flush_tx(void) {
    taskENTER_CRITICAL();
    g_tx_generation++;
    taskEXIT_CRITICAL();
    if (can1_send_fifoHandle != nullptr) {
        (void)osMessageQueueReset(can1_send_fifoHandle);
    }
    if (can2_send_fifoHandle != nullptr) {
        (void)osMessageQueueReset(can2_send_fifoHandle);
    }
    if (g_started) {
        (void)HAL_CAN_AbortTxRequest(
            &hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        (void)HAL_CAN_AbortTxRequest(
            &hcan2, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }
    stats_increment_task(&g_stats.tx_flushes);
}

bool can_bsp_register_std(uint8_t bus, uint32_t id) {
    if (bus < 1u || bus > 2u || id > 0x7FFu || g_started) { return false; }
    const uint32_t index = bus - 1u;
    if (g_std_receivers[index] != nullptr) { return g_std_ids[index] == id; }
    CAN_HandleTypeDef *hcan = (bus == 1u) ? &hcan1 : &hcan2;
    can_device_receive *receiver = new can_device_receive(hcan, id, nullptr);
    if (receiver == nullptr || !receiver->is_registered()) {
        delete receiver;
        return false;
    }
    g_std_receivers[index] = receiver;
    g_std_ids[index] = id;
    return true;
}

bool can_bsp_add_std_observer(can_bsp_std_rx_observer_t observer) {
    if (observer == nullptr) { return false; }
    for (uint32_t i = 0u; i < 4u; ++i) {
        if (g_observers[i] == observer) { return true; }
        if (g_observers[i] == nullptr) {
            g_observers[i] = observer;
            return true;
        }
    }
    return false;
}

uint32_t can_bsp_tx_generation(void) {
    return g_tx_generation;
}

void can_bsp_stop_services(void) {
    g_services_run = false;
}

void can_bsp_get_stats(can_bsp_stats_t *out) {
    if (out == nullptr) { return; }
    taskENTER_CRITICAL();
    *out = g_stats;
    taskEXIT_CRITICAL();
}
