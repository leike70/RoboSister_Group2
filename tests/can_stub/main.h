#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stddef.h>
#include <stdint.h>

#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))
#define __RAM_FUNC

#endif
