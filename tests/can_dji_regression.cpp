#include <stdio.h>
#include <string.h>
#include <windows.h>

#include "CAN_bsp.h"
#include "DJI_motor_driver.h"
#include "can_bsp_api.h"

extern osMessageQueueId_t can1_send_fifoHandle;
extern osMessageQueueId_t can2_send_fifoHandle;

extern CAN_RxHeaderTypeDef test_rx_header;
extern uint8_t test_rx_data[8];
extern HAL_StatusTypeDef test_rx_status;
extern HAL_StatusTypeDef test_start_status1;
extern HAL_StatusTypeDef test_start_status2;
extern HAL_StatusTypeDef test_tx_status;
extern uint32_t test_tx_count;
extern uint32_t test_tick;
extern int test_isr_critical_errors;
extern uint32_t test_queue_get_calls;
extern uint32_t test_sem_acquire_count;
extern int test_queue_block_enabled;
extern int test_queue_block_release;
extern "C" void test_can_reset_rx(void);
extern "C" void test_can_set_queue_item_size(uint32_t size);

static int failures;
static int rx_count;
static int observer_count;
static uint8_t observer_bus;
static uint8_t observer_dlc;
static uint32_t observer_id;
static uint32_t observer_arrival;
static uint8_t rx_last[8];
static can_device_receive *rx_last_dev;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } \
} while (0)

static void observer_callback(uint8_t bus, uint32_t id, const uint8_t *data,
                              uint8_t dlc, uint32_t arrival_ms) {
    (void)data;
    if (id == 0x31u) {
        observer_count++;
        observer_bus = bus;
        observer_id = id;
        observer_dlc = dlc;
        observer_arrival = arrival_ms;
    }
}

static DWORD WINAPI sender_thread(LPVOID) {
    can1_sending_service(NULL);
    return 0u;
}

static void rx_callback(can_device_receive *dev, uint8_t *data) {
    rx_count++;
    rx_last_dev = dev;
    memcpy(rx_last, data, 8u);
}

static void stage_frame(CAN_HandleTypeDef *hcan, uint32_t fifo, uint32_t id,
                        const uint8_t data[8], uint32_t dlc, uint32_t ide,
                        uint32_t rtr) {
    (void)hcan; (void)fifo;
    memset(&test_rx_header, 0, sizeof(test_rx_header));
    test_rx_header.StdId = id;
    test_rx_header.IDE = ide;
    test_rx_header.RTR = rtr;
    test_rx_header.DLC = dlc;
    memcpy(test_rx_data, data, 8u);
    test_rx_status = HAL_OK;
}

/* The production CAN layer is linked here; only HAL/RTOS boundaries are stubs. */
extern "C" bool app_protection_output_allowed(void) { return false; }

extern "C" uint32_t app_time_now_ms(void) { return test_tick; }

int main(void) {
    uint8_t payload1[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t payload2[8] = {9, 10, 11, 12, 13, 14, 15, 16};
    uint8_t frame[8] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
    can_bsp_stats_t stats;
    can_device_transmit_member out;

    test_can_set_queue_item_size(sizeof(can_device_transmit_member));

    /* The 0x31 path uses the same CAN registration/observer, not a second RX path. */
    CHECK(can_bsp_add_std_observer(observer_callback));
    CHECK(can_bsp_register_std(1u, 0x31u));
    test_tick = 77u;
    /* Partial startup: CAN1 succeeds, CAN2 fails; retry must complete without
     * re-starting CAN1 blindly. */
    test_start_status1 = HAL_OK;
    test_start_status2 = HAL_ERROR;
    CHECK(!can_bsp_start());
    test_start_status2 = HAL_OK;
    CHECK(can_bsp_start());
    CHECK(can_bsp_is_started());
    can_bsp_flush_tx();
    CHECK(osMessageQueueReset(can1_send_fifoHandle) == osOK);

    /* Shared observer receives the validated standard frame with arrival time. */
    {
        uint8_t frame31[4] = {0xF4, 0x01, 0x88, 0x13};
        stage_frame(&hcan1, CAN_RX_FIFO0, 0x31u, frame31, 4u, CAN_ID_STD, CAN_RTR_DATA);
        test_in_isr = 1;
        HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
        test_in_isr = 0;
        CHECK(test_isr_critical_errors == 0);
        CHECK(observer_count == 1);
        CHECK(observer_bus == 1u && observer_id == 0x31u && observer_dlc == 4u);
        CHECK(observer_arrival == 77u);
    }

    /* Queue must own the payload, not the caller buffer pointer. */
    {
        uint8_t mutable_payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
        can_device_transmit tx(&hcan1, 0x200u, 8u, mutable_payload);
        CHECK(tx.can_send_msg() == osOK);
        mutable_payload[0] = 0xEEu;
        mutable_payload[7] = 0xFFu;
        CHECK(osMessageQueueGet(can1_send_fifoHandle, &out, nullptr, 0u) == osOK);
        CHECK(out.hcan == &hcan1 && out.id == 0x200u && out.len == 8u);
        CHECK(memcmp(out.buf_data, payload1, 8u) == 0);
    }

    /* CAN2 routing and independent queue: a frame with ID 0x301 is delivered to
     * the CAN2 registration, not the same-index CAN1 registration. */
    {
        can_device_receive can1_dev(&hcan1, 0x200u, rx_callback);
        can_device_receive can2_dev(&hcan2, 0x301u, rx_callback);
        CHECK(can1_dev.is_registered());
        CHECK(can2_dev.is_registered());

        stage_frame(&hcan2, CAN_RX_FIFO1, 0x301u, payload2, 8u, CAN_ID_STD, CAN_RTR_DATA);
        test_in_isr = 1;
        HAL_CAN_RxFifo1MsgPendingCallback(&hcan2);
        test_in_isr = 0;
        CHECK(test_isr_critical_errors == 0);
        CHECK(rx_count == 1 && rx_last_dev == &can2_dev);
        CHECK(memcmp(rx_last, payload2, 8u) == 0);

        /* Unknown ID must not crash or invoke a callback. */
        stage_frame(&hcan2, CAN_RX_FIFO1, 0x302u, frame, 8u, CAN_ID_STD, CAN_RTR_DATA);
        HAL_CAN_RxFifo1MsgPendingCallback(&hcan2);
        CHECK(rx_count == 1);

        /* Invalid frame type/DLC must be rejected before device dispatch. */
        stage_frame(&hcan2, CAN_RX_FIFO1, 0x301u, frame, 9u, CAN_ID_STD, CAN_RTR_DATA);
        HAL_CAN_RxFifo1MsgPendingCallback(&hcan2);
        CHECK(rx_count == 1);
        stage_frame(&hcan2, CAN_RX_FIFO1, 0x301u, frame, 8u, CAN_ID_STD, CAN_RTR_REMOTE);
        HAL_CAN_RxFifo1MsgPendingCallback(&hcan2);
        CHECK(rx_count == 1);

        /* A registration without callback must be observable but safe. */
        can_device_receive no_cb(&hcan2, 0x303u, nullptr);
        CHECK(no_cb.is_registered());
        stage_frame(&hcan2, CAN_RX_FIFO1, 0x303u, frame, 8u, CAN_ID_STD, CAN_RTR_DATA);
        HAL_CAN_RxFifo1MsgPendingCallback(&hcan2);
        CHECK(no_cb.is_callback_none == 1u);
    }

    /* CAN1 route + invalid DLC / HAL failure counters. */
    {
        test_can_reset_rx();
        stage_frame(&hcan1, CAN_RX_FIFO0, 0x200u, frame, 8u, CAN_ID_STD, CAN_RTR_DATA);
        HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
        CHECK(rx_count == 2);
        test_rx_status = HAL_ERROR;
        HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
        CHECK(rx_count == 2);
    }

    /* Queue admission is tested here; the blocking sender task is not run in a
     * host test, so HAL send failure is covered by the BSP's result checks and
     * the target-side integration test, not by a fixed-success stub. */
    {
        can_device_transmit tx(&hcan1, 0x201u, 8u, payload1);
        CHECK(tx.can_send_msg() == osOK);
    }

    /* Empty queue: the sender must block in the queue, not spin acquiring
     * mailbox semaphores. The stub records one queue wait and zero sem acquires. */
    {
        can_bsp_flush_tx();
        test_queue_get_calls = 0u;
        test_sem_acquire_count = 0u;
        test_queue_block_enabled = 1;
        test_queue_block_release = 0;
        HANDLE thread = CreateThread(NULL, 0, sender_thread, NULL, 0, NULL);
        CHECK(thread != NULL);
        Sleep(50);
        CHECK(test_queue_get_calls == 1u);
        CHECK(test_sem_acquire_count == 0u);
        test_queue_block_release = 1;
        can_bsp_stop_services();
        (void)WaitForSingleObject(thread, 2000u);
        CloseHandle(thread);
        test_queue_block_enabled = 0;
    }

    /* Completion/abort/error callbacks must also use ISR-safe critical macros. */
    test_in_isr = 1;
    HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
    HAL_CAN_TxMailbox0AbortCallback(&hcan1);
    hcan1.ErrorCode = HAL_CAN_ERROR_BOF;
    HAL_CAN_ErrorCallback(&hcan1);
    test_in_isr = 0;
    CHECK(test_isr_critical_errors == 0);

    /* Bus-off event is handled by the task-side finite recovery path. */
    test_in_isr = 1;
    hcan1.ErrorCode = HAL_CAN_ERROR_BOF;
    HAL_CAN_ErrorCallback(&hcan1);
    test_in_isr = 0;
    can_bsp_service(1000u);
    can_bsp_get_stats(&stats);
    CHECK(stats.recovered >= 1u);

    can_bsp_get_stats(&stats);
    CHECK(stats.started);
    CHECK(stats.tx_requests >= 2u);
    CHECK(stats.rx_ok >= 2u);
    CHECK(stats.rx_bad >= 1u);

    /* DJI lifecycle: constructor must not wait, must not auto-enable output. */
    {
        DJI_motor motor(&hcan1, nullptr, 2u, DJI_M3508);
        CHECK(motor.get_lifecycle() == DJI_MOTOR_WAIT_FEEDBACK);
        CHECK(!motor.is_feedback_ready());
        CHECK(!motor.is_output_enabled());

        /* DLC=0 must not count as feedback or prepare the offset. */
        for (int i = 0; i < 51; ++i) {
            uint8_t short_data[8] = {0x10, 0x20, 0, 0, 0, 0, 0, 0};
            stage_frame(&hcan1, CAN_RX_FIFO0, 0x202u, short_data, 0u,
                        CAN_ID_STD, CAN_RTR_DATA);
            test_in_isr = 1;
            HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
            test_in_isr = 0;
        }
        CHECK(!motor.is_feedback_ready());
        CHECK(motor.get_lifecycle() == DJI_MOTOR_WAIT_FEEDBACK);

        for (int i = 0; i < 51; ++i) {
            uint8_t data[8] = {0x10, 0x20, 0, 0, 0, 0, 0, 0};
            stage_frame(&hcan1, CAN_RX_FIFO0, 0x202u, data, 8u, CAN_ID_STD, CAN_RTR_DATA);
            test_in_isr = 1;
            HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
            test_in_isr = 0;
        }
        CHECK(motor.is_feedback_ready());
        CHECK(!motor.is_output_enabled());
        motor.motor_control(DJI_MOTOR_ENABLE_OUTPUT);
        CHECK(motor.is_output_enabled());
        motor.motor_control(DJI_MOTOR_DISABLE_OUTPUT);
        CHECK(!motor.is_output_enabled());

        DJI_motor no_feedback(&hcan1, nullptr, 3u, DJI_M3508);
        no_feedback.service(2000u);
        CHECK(no_feedback.get_lifecycle() == DJI_MOTOR_FAULT);
    }

    if (failures) {
        printf("%d CAN/DJI checks failed\n", failures);
        return 1;
    }
    puts("PASS CAN routing/payload/lifecycle and DJI no-wait lifecycle");
    return 0;
}
