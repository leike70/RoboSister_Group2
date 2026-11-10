#include <stdio.h>
#include "app_config.h"

#if defined(TEST_TURRET)
#define EXPECTED_LIMIT 40.0f
#define EXPECTED_ENERGY 400.0f
#define EXPECTED_KIND APP_VEHICLE_KIND_TURRET
#else
#define EXPECTED_LIMIT 50.0f
#define EXPECTED_ENERGY 500.0f
#define EXPECTED_KIND APP_VEHICLE_KIND_ENGINEER
#endif

uint32_t app_time_now_ms(void) { return 0u; }

int main(void) {
    const app_config_t *config = app_config_get();
    if (config == NULL ||
        config->vehicle_kind != EXPECTED_KIND ||
        config->power.chassis_power_limit_w != EXPECTED_LIMIT ||
        config->power.buffer_energy_limit_j != EXPECTED_ENERGY ||
        !config->power.chassis_power_limit_valid ||
        !config->power.buffer_energy_limit_valid) {
        puts("FAIL vehicle power/energy rule config");
        return 1;
    }
    if (config->power.source_bus_valid || config->power.hardware_verified ||
        config->power.source_timeout_valid) {
        puts("FAIL unknown power source must remain unverified");
        return 1;
    }
    puts("PASS vehicle power/energy rule config");
    return 0;
}
