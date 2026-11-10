#ifndef CAN_BSP_API_H
#define CAN_BSP_API_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Runtime CAN health snapshot. Counters are monotonic except the streak fields. */
typedef struct {
    bool started;
    uint32_t start_attempts;
    uint32_t start_failures;
    uint32_t tx_requests;
    uint32_t tx_ok;
    uint32_t tx_fail;
    uint32_t tx_dropped;
    uint32_t tx_queue_full;
    uint32_t tx_aborted;
    uint32_t tx_flushes;
    uint32_t rx_ok;
    uint32_t rx_bad;
    uint32_t rx_unmatched;
    uint32_t rx_no_callback;
    uint32_t errors;
    uint32_t bus_off;
    uint32_t recovery_attempts;
    uint32_t recovery_failures;
    uint32_t recovered;
    uint32_t tx_fail_streak;
    uint32_t tx_unexpected_completion;
} can_bsp_stats_t;

/** Called by the application after RTOS queues/semaphores and filters are ready. */
bool can_bsp_start(void);
bool can_bsp_is_started(void);

/** Task-context maintenance: finite bus-off recovery attempts; never called from ISR. */
void can_bsp_service(uint32_t now_ms);

/** Drop queued commands and abort pending mailboxes; used on output-loss transitions. */
void can_bsp_flush_tx(void);

/** Current TX generation; adapters enqueueing member structs directly must stamp it. */
uint32_t can_bsp_tx_generation(void);

/** Test/emergency stop for the blocking sender loops. Not used by normal App_Control. */
void can_bsp_stop_services(void);

/** Copy the current health snapshot. */
void can_bsp_get_stats(can_bsp_stats_t *out);

/**
 * Register a standard-ID receiver for a configured bus (1 or 2). Must be called
 * before can_bsp_start(); the CAN layer owns the receiver object/filter.
 * Returns false for an invalid bus, duplicate ID or registration failure.
 */
bool can_bsp_register_std(uint8_t bus, uint32_t id);

/** Observer callback runs in the CAN ISR; only copy/record there. */
typedef void (*can_bsp_std_rx_observer_t)(uint8_t bus, uint32_t id,
                                           const uint8_t *data, uint8_t dlc,
                                           uint32_t arrival_ms);

/** Register a shared observer for all registered standard-ID receivers. */
bool can_bsp_add_std_observer(can_bsp_std_rx_observer_t observer);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BSP_API_H */
