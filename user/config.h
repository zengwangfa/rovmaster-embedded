/**
 * @desc: 各类数据结构定义文件
 */

#ifndef __DATA_TYPE_H_
#define __DATA_TYPE_H_

#include <stdint.h> // 其中定义数据类型(eg. uint8...)

#define ROV_MASTER_VERSION "1.0.0"                        // 程序版本
#define ROV_CONFIG_FILE_PATH "/home/pi/rov-master/params" // 参数文件路径

// 简单 PWM 设备对应 PCA9685 PWM输出通道(接线板PWM口定义)
#define LIGHT_CHANNEL 10
#define YUNTAI_CHANNEL 8
#define ROBOT_ARM_CHANNEL 12
/* ------------------------【数据定义】---------------------------------*/

// 标准大气压 101.325kPa
#define STANDARD_ATMOSPHERIC_PRESSURE 101325
// 电池 参数
#define STANDARD_BATTERY_VOLTAGE 3.7f // 锂电池标准电压
#define FULL_BATTERY_VOLTAGE 4.2f     // 锂电池满电压

// 航行器 类型
#define FOUR_AXIS 0 // 4推标志
#define SIX_AXIS 1  // 6推标志

// 航行器 锁定
#define ROV_UNLOCK 1 // 全局解锁(启动)
#define ROV_LOCK 2   // 全局锁定(停止)

#define PI 3.141592f                     // 大写标明其为常量
#define Rad2Deg(rad) (rad * 180.0f / PI) // 弧度制转角度值
#define Deg2Rad(deg) (deg * PI / 180.0f) // 角度值转弧度制

typedef enum
{
    System_ERROR_STATUS = -1, // 错误模式
    System_NORMAL_STATUS = 1, // 正常模式
    System_DEBUG_STATUS = 2,  // 调试模式

} vehicleStatus_enum; //枚举系统状态

#endif
