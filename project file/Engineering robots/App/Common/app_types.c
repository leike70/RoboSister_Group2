/**
  ******************************************************************************
  * @file    app_types.c
  * @brief   应用层公共类型辅助函数。
  ******************************************************************************
  */
#include "Common/app_types.h"

float app_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}
