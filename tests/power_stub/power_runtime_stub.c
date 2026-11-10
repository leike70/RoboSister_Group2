#include <stdbool.h>
#include <stdint.h>

/* Host boundary for the production Power module's short critical section and
 * time arithmetic. No production logic is replaced here. */
uint32_t app_critical_enter(void) { return 0u; }
void app_critical_exit(uint32_t saved_mask) { (void)saved_mask; }

uint32_t app_time_elapsed_ms(uint32_t now_ms, uint32_t then_ms) {
    return (uint32_t)(now_ms - then_ms);
}

bool app_time_is_fresh(uint32_t now_ms, uint32_t stamp_ms, uint32_t timeout_ms) {
    return (uint32_t)(now_ms - stamp_ms) <= timeout_ms;
}
