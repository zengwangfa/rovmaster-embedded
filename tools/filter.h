
#ifndef __FILTER_H_
#define __FILTER_H_

#include "../user/config.h"

// 冒泡中值滤波
uint32_t bubble_filter(uint32_t *value);
// 卡尔曼滤波
float kalman_filter(float *Original_Data);

#endif
