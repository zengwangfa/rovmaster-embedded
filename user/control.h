#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "config.h"

typedef struct
{
    int16_t fx;    // 摇杆 X轴动力 (X正方向为机头方向)
    int16_t fy;    // 摇杆 Y轴动力
    int16_t fz;    // 摇杆 Z轴动力
    int16_t yaw;   // 偏航角度
    int16_t force; // 合力大小

    float deg;     // 合力角度值
    float rad;     // 合力弧度角
    float percent; // 动力百分比

} rockerInfo_t; /* 摇杆信息 */

#endif
