#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "Common/app_critical.h"

static uint32_t mask;
static unsigned int barriers;
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1; }
void __DMB(void) { barriers++; }
void __set_PRIMASK(uint32_t value) { mask = value; }
#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "FAIL critical:%d: %s\n", __LINE__, #expr); exit(1); \
} } while (0)

int main(void)
{
    mask = 0;
    const uint32_t outer = app_critical_enter();
    CHECK(mask == 1 && outer == 0);
    const uint32_t inner = app_critical_enter();
    CHECK(mask == 1 && inner == 1);
    app_critical_exit(inner);
    CHECK(mask == 1);
    app_critical_exit(outer);
    CHECK(mask == 0);
    mask = 1;
    const uint32_t already_disabled = app_critical_enter();
    app_critical_exit(already_disabled);
    CHECK(mask == 1);
    CHECK(barriers == 6);
    puts("PASS production critical helper: nesting and previous IRQ mask restored");
    return 0;
}
