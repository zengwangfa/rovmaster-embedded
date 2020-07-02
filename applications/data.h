#ifndef __RC_DATA_H_
#define __RC_DATA_H_

#include "../user/config.h"

/*
 * 接收数据包长度【包含包头、长度位、校验位】
 * 包头：2, 数据长度位: 1, 包体：16, 校验字：1
*/
#define RECV_DATA_LEN 20

/*
 * 返回数据包长度【包含包头、长度位、校验位】
 * 包头：2, 数据长度位: 1, 包体：22, 校验字：1
*/
#define RETURN_DATA_LEN 26

typedef struct
{
    /* 开关类 */
    uint8_t all_lock;   //总开关
    uint8_t depth_lock; //【深度】	 0x01 深度锁定、0x02 人工控制
    uint8_t sport_lock; //【运动】	 0x01 方向锁定、0x02 随水动

    /* 运动类 */
    uint8_t move_back;  //【前后】
    uint8_t left_right; //【平移】
    uint8_t up_down;    //【垂直】
    uint8_t rotate;     //【旋转】

    /* 设备类 */
    uint8_t power;  //【油门】	0x00~0xff (0~255) 表示的油门大小：4档位可调，LB加档，LT减档，可分别设置4个档位油门大小
    uint8_t light;  //【灯光】	0x01 打开、0x00 关闭（默认）
    uint8_t camera; //【聚焦】  0x01 聚焦、0x02 放焦、  0x11 放大、0x12 缩小、0x00表示不动作（默认）
    uint8_t yuntai; //【云台】	0x01 向上、0x02 向下、  0x00表示不动作（默认）
    uint8_t arm;    //【机械臂】0x01 张开、0x02 关闭、  0x00表示不动作（默认）

} cmd_t; /* 控制命令 */

// 转换需要返回上位机数据
void convert_rov_status_data(uint8_t *buff);
// 控制命令解析
void remote_control_data_analysis(uint8_t *buff, cmd_t *cmd);
// 计算校验和
uint8_t calculate_check_sum(uint8_t *buff, uint8_t len);

// 从文件中读取ROV所有参数
void read_rov_all_params(void);
// 将ROV所有参数写入文件
void write_rov_all_params(void);
// ROV所有设备参数初始化
void rov_all_params_init(void);

/* 上位机的控制数据 */
extern cmd_t cmd_data;
#endif
