#ifndef __FOCUS_H_
#define __FOCUS_H_

#include "../user/config.h"

#define FOCUS_CAMERA_UART_DEV "/dev/ttyS1" // 变焦镜头 使用的 UART 设备
#define UART_BAUD_115200 115200            // 变焦镜头 UART 波特率

#define CAMERA_CONTROL_DATA_LEN 6

#define FOCUS_ZOOM_STOP 0x00 // 停止信号

#define FOCUS_MAGNIFY 0x01 // 聚焦放大
#define FOCUS_LESSEN 0x02  // 聚焦缩小

#define ZOOM_IN 0x11  // 变焦拉近
#define ZOOM_OUT 0x12 // 变焦拉远

int focus_camera_thread_init(void);
#endif
