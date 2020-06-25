/*
 * 匿名地面站数据传输
 *
 * Attention: 接收只需要调用 -> ANO_DT_Data_Receive_Prepare(uint8_t data); 或者 void ANO_DT_Data_Receive_Anl(uint8_t *data_buf, uint8_t num);
 *【统一接口】 发送只需要调用 -> ANO_SEND_StateMachine(void);
 *            保存参数需调用 -> void Save_Or_Reset_PID_Parameter(void);
 */

#include "ano_link.h"
#include "filter.h"

#include "../applications/PID.h"
#include "../applications/data.h"
#include "../applications/pwmDevices.h"
#include "../applications/sensor.h"
#include "../user/datatype.h"
#include "../user/debug.h"

#include <elog.h>
#include <stdio.h>

#define BYTE0(dwTemp) (*((char *)(&dwTemp) + 0))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

#define HardwareType 0.00 // 硬件种类  00为其他硬件版本
#define HardwareVER 1.00  // 硬件版本
#define SoftwareVER 1.00  // 软件版本
#define ProtocolVER 1     // 协议版本
#define BootloaderVER 1   // Bootloader版本

Vector3f_pid PID_Parameter[8] = {0};

uint8_t data_to_send[50];  // ANO地面站发送数据缓冲
uint8_t ANO_Send_PID_Flag; // PID发送标志位
void ANO_DT_Send_All_PID(void);
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(uint8_t *dataToSend, uint8_t len)
{
    ano_udp_send(dataToSend, len);
}
/*******************************************
* 函 数 名：ANO_DT_Data_Receive_Prepare
* 功    能：ANO地面站数据解析
* 输入参数：串口循环传入的一个数据data
* 返 回 值：none
* 注    意：none
********************************************/
void ANO_DT_Data_Receive_Prepare(uint8_t data) //ANO地面站数据解析
{

    static uint8_t RxBuffer[50];
    static uint8_t _data_len = 0, _data_cnt = 0;
    static uint8_t state = 0;

    if (state == 0 && data == 0xAA) //帧头1
    {
        state = 1;
        RxBuffer[0] = data;
    }
    else if (state == 1 && data == 0xAF) //帧头2
    {
        state = 2;
        RxBuffer[1] = data;
    }
    else if (state == 2 && data < 0XF1) //功能字节
    {
        state = 3;
        RxBuffer[2] = data;
    }
    else if (state == 3 && data < 50) //有效数据长度
    {
        state = 4;
        RxBuffer[3] = data;
        _data_len = data;
        _data_cnt = 0;
    }
    else if (state == 4 && _data_len > 0) //数据接收
    {
        _data_len--;
        RxBuffer[4 + _data_cnt++] = data;
        if (_data_len == 0)
            state = 5;
    }
    else if (state == 5) //校验和
    {
        state = 0;
        RxBuffer[4 + _data_cnt] = data;
        ANO_DT_Data_Receive_Anl(RxBuffer, _data_cnt + 5); //数据解析
    }
    else
        state = 0;
}

/*******************************************
* 函 数 名：ANO_DT_Send_Check
* 功    能：ANO地面站和校验
* 输入参数：head 功能字
						check_num 校验和
* 返 回 值：none
* 注    意：none
********************************************/
static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum)
{
    uint8_t sum = 0, i = 0;
    data_to_send[0] = 0xAA;
    data_to_send[1] = 0xAA;
    data_to_send[2] = 0xEF;
    data_to_send[3] = 2;
    data_to_send[4] = head;
    data_to_send[5] = check_sum;
    for (i = 0; i < 6; i++)
    {
        sum += data_to_send[i];
    }
    data_to_send[6] = sum;

    ANO_DT_Send_Data(data_to_send, 7);
}

/*******************************************
* 函 数 名：ANO_DT_Data_Receive_Anl
* 功    能：ANO数据解析
* 输入参数：data_buf: 数据包
						num: 数据包大小
* 返 回 值：none
* 注    意：none
********************************************/
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf, uint8_t num)
{
    uint8_t sum = 0, i = 0;
    for (i = 0; i < (num - 1); i++)
        sum += *(data_buf + i);
    if (!(sum == *(data_buf + num - 1)))
        return; //判断sum
    if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAF))
        return; //判断帧头
    if (*(data_buf + 2) == 0x01)
    {
        if (*(data_buf + 4) == 0x01) // ACC校准
        {
        }
        if (*(data_buf + 4) == 0x02) // GYRO校准
        {
        }
        if (*(data_buf + 4) == 0x04) // MAG校准
        {
        }
    }

    if (*(data_buf + 2) == 0x02)
    {
        if (*(data_buf + 4) == 0x01) //读取当前PID参数
        {
            ANO_Send_PID_Flag = 1;
        }
        if (*(data_buf + 4) == 0x02) //读取飞行模式设置请求
        {
        }
        if (*(data_buf + 4) == 0xA0) //读取下位机版本信息
        {
        }
        if (*(data_buf + 4) == 0xA1) //恢复默认参数
        {
            Total_PID_Init();       // 将PID参数重置为参数Control_Unit表里面参数
            write_rov_all_params(); // 写入PID参数
            log_i("reset pid params -> Success!");
        }
    }

    if (*(data_buf + 2) == 0x10) //接收PID1
    {
        Total_Controller.Roll_Gyro_Control.Kp = 0.001 * ((int16_t)(*(data_buf + 4) << 8) | *(data_buf + 5));
        Total_Controller.Roll_Gyro_Control.Ki = 0.001 * ((int16_t)(*(data_buf + 6) << 8) | *(data_buf + 7));
        Total_Controller.Roll_Gyro_Control.Kd = 0.001 * ((int16_t)(*(data_buf + 8) << 8) | *(data_buf + 9));
        Total_Controller.Pitch_Gyro_Control.Kp = 0.001 * ((int16_t)(*(data_buf + 10) << 8) | *(data_buf + 11));
        Total_Controller.Pitch_Gyro_Control.Ki = 0.001 * ((int16_t)(*(data_buf + 12) << 8) | *(data_buf + 13));
        Total_Controller.Pitch_Gyro_Control.Kd = 0.01 * ((int16_t)(*(data_buf + 14) << 8) | *(data_buf + 15));
        Total_Controller.Yaw_Gyro_Control.Kp = 0.001 * ((int16_t)(*(data_buf + 16) << 8) | *(data_buf + 17));
        Total_Controller.Yaw_Gyro_Control.Ki = 0.001 * ((int16_t)(*(data_buf + 18) << 8) | *(data_buf + 19));
        Total_Controller.Yaw_Gyro_Control.Kd = 0.001 * ((int16_t)(*(data_buf + 20) << 8) | *(data_buf + 21));
        ANO_DT_Send_Check(*(data_buf + 2), sum);
    }
    if (*(data_buf + 2) == 0x11) //接收PID2
    {
        Total_Controller.Roll_Angle_Control.Kp = 0.001 * ((int16_t)(*(data_buf + 4) << 8) | *(data_buf + 5));
        Total_Controller.Roll_Angle_Control.Ki = 0.001 * ((int16_t)(*(data_buf + 6) << 8) | *(data_buf + 7));
        Total_Controller.Roll_Angle_Control.Kd = 0.001 * ((int16_t)(*(data_buf + 8) << 8) | *(data_buf + 9));
        Total_Controller.Pitch_Angle_Control.Kp = 0.001 * ((int16_t)(*(data_buf + 10) << 8) | *(data_buf + 11));
        Total_Controller.Pitch_Angle_Control.Ki = 0.001 * ((int16_t)(*(data_buf + 12) << 8) | *(data_buf + 13));
        Total_Controller.Pitch_Angle_Control.Kd = 0.001 * ((int16_t)(*(data_buf + 14) << 8) | *(data_buf + 15));
        Total_Controller.Yaw_Angle_Control.Kp = 0.001 * ((int16_t)(*(data_buf + 16) << 8) | *(data_buf + 17));
        Total_Controller.Yaw_Angle_Control.Ki = 0.001 * ((int16_t)(*(data_buf + 18) << 8) | *(data_buf + 19));
        Total_Controller.Yaw_Angle_Control.Kd = 0.001 * ((int16_t)(*(data_buf + 20) << 8) | *(data_buf + 21));
        ANO_DT_Send_Check(*(data_buf + 2), sum);
    }
    if (*(data_buf + 2) == 0x12) //接收PID3
    {
        Total_Controller.High_Speed_Control.Kp = 0.001 * ((int16_t)(*(data_buf + 4) << 8) | *(data_buf + 5));
        Total_Controller.High_Speed_Control.Ki = 0.001 * ((int16_t)(*(data_buf + 6) << 8) | *(data_buf + 7));
        Total_Controller.High_Speed_Control.Kd = 0.001 * ((int16_t)(*(data_buf + 8) << 8) | *(data_buf + 9));
        Total_Controller.High_Position_Control.Kp = 0.001 * ((int16_t)(*(data_buf + 10) << 8) | *(data_buf + 11));
        Total_Controller.High_Position_Control.Ki = 0.001 * ((int16_t)(*(data_buf + 12) << 8) | *(data_buf + 13));
        Total_Controller.High_Position_Control.Kd = 0.001 * ((int16_t)(*(data_buf + 14) << 8) | *(data_buf + 15));
        ANO_DT_Send_Check(*(data_buf + 2), sum);
    }
    if (*(data_buf + 2) == 0x13) //接收PID4
    {
        ANO_DT_Send_Check(*(data_buf + 2), sum);
    }
    if (*(data_buf + 2) == 0x14) //接收PID5 【改为推进器方向标志】
    {

        ANO_DT_Send_Check(*(data_buf + 2), sum);
    }
    if (*(data_buf + 2) == 0x15) //接收PID6  (功能改为： 探照灯、云台、机械臂参数接收)
    {
        rovdev.light.nMax = (int16_t)(*(data_buf + 4) << 8) | *(data_buf + 5);  // 反向最大值
        rovdev.light.pMax = (int16_t)(*(data_buf + 6) << 8) | *(data_buf + 7);  // 正向最大值
        rovdev.light.speed = (int16_t)(*(data_buf + 8) << 8) | *(data_buf + 9); // 速度

        rovdev.yuntai.nMax = (int16_t)(*(data_buf + 10) << 8) | *(data_buf + 11);
        rovdev.yuntai.pMax = (int16_t)(*(data_buf + 12) << 8) | *(data_buf + 13);
        rovdev.yuntai.speed = (int16_t)(*(data_buf + 14) << 8) | *(data_buf + 15);
        rovdev.yuntai.med = (rovdev.yuntai.nMax + rovdev.yuntai.pMax) / 2;

        rovdev.robot_arm.nMax = (int16_t)(*(data_buf + 16) << 8) | *(data_buf + 17);
        rovdev.robot_arm.pMax = (int16_t)(*(data_buf + 18) << 8) | *(data_buf + 19);
        rovdev.robot_arm.speed = (int16_t)(*(data_buf + 20) << 8) | *(data_buf + 21);
        rovdev.robot_arm.med = (rovdev.robot_arm.nMax + rovdev.robot_arm.pMax) / 2;

        write_rov_all_params(); // 写入PID参数
        log_i("write pid params -> success!");
        ANO_DT_Send_Check(*(data_buf + 2), sum);
    }
}

//---------------这里是分隔符↓↓↓↓↓↓↓↓↓↓<以下为发送 子函数>↓↓↓↓↓↓↓↓↓↓这里是分隔符------------------//
/*******************************************
* 函 数 名：ANO_Data_Send_Version
* 功    能：发送基本版本信息（硬件种类、硬件、软件、协议、Bootloader版本）【第一组】
* 输入参数：none
* 返 回 值：none
* 注    意：none
********************************************/
void ANO_Data_Send_Version(int hardwareType, int hardwareVER, int softwareVER, int protocolVER, int bootloaderVER) //发送版本信息
{
    uint8_t _cnt = 0;
    int16_t _temp;
    int32_t _temp2;
    uint8_t sum = 0;
    uint8_t i = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x00;
    data_to_send[_cnt++] = 0;

    _temp = HardwareType;
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(HardwareVER * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(SoftwareVER * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp2 = (int)(ProtocolVER * 100);
    data_to_send[_cnt++] = BYTE1(_temp2);
    data_to_send[_cnt++] = BYTE0(_temp2);

    _temp2 = (int)(BootloaderVER * 100);
    data_to_send[_cnt++] = BYTE1(_temp2);
    data_to_send[_cnt++] = BYTE0(_temp2);

    data_to_send[3] = _cnt - 4;
    sum = 0;
    for (i = 0; i < _cnt; i++)
    {
        sum += data_to_send[i];
    }
    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}
/*******************************************
* 函 数 名：ANO_Data_Send_Status
* 功    能：发送基本信息（欧拉三角、高度、锁定状态）【第二组】
* 输入参数：none
* 返 回 值：none
* 注    意：none
********************************************/
void ANO_Data_Send_Status(float roll, float pitch, float yaw, float depth) //发送基本信息（姿态、锁定状态）
{
    uint8_t _cnt = 0;
    int16_t _temp;
    int32_t _temp2;
    uint8_t sum = 0;
    uint8_t i;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x01;
    data_to_send[_cnt++] = 0;

    _temp = (int)(roll * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(pitch * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(yaw * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp2 = (int)(depth * 100); // 单位cm
    data_to_send[_cnt++] = BYTE3(_temp2);
    data_to_send[_cnt++] = BYTE2(_temp2);
    data_to_send[_cnt++] = BYTE1(_temp2);
    data_to_send[_cnt++] = BYTE0(_temp2);

    data_to_send[_cnt++] = 0x01; // 飞行模式
    data_to_send[_cnt++] = 0;    // 2 - ControlCmd.All_Lock; //【匿名】上锁0、解锁1   自定义为：0x01解锁，0x02上锁

    data_to_send[3] = _cnt - 4;
    sum = 0;
    for (i = 0; i < _cnt; i++)
    {
        sum += data_to_send[i];
    }
    data_to_send[_cnt++] = sum;
    ANO_DT_Send_Data(data_to_send, _cnt);
}

/*******************************************
* 函 数 名：ANO_DT_Send_Senser
* 功    能：发送传感器原始数字量 (加速度、角速度、磁场)  【第三组】
* 输入参数：3组数据x,y,z轴
* 返 回 值：none
* 注    意：none
********************************************/
void ANO_DT_Send_Senser(float a_x, float a_y, float a_z, float g_x, float g_y, float g_z, float m_x, float m_y, float m_z)
{
    uint8_t _cnt = 0;
    int16_t _temp;
    uint8_t sum = 0;
    uint8_t i = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x02;
    data_to_send[_cnt++] = 0;

    _temp = (int)(a_x * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(a_y * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(a_z * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = g_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = g_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = g_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = m_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = m_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = m_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;

    sum = 0;
    for (i = 0; i < _cnt; i++)
    {
        sum += data_to_send[i];
    }
    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}

/*******************************************
* 函 数 名：ANO_DT_Send_High
* 功    能：发送高度数据 (气压计高度、超声波高低)  【第七组】
* 输入参数：pressure_high：气压计高度、ultrasonic_high：超声波高度
* 返 回 值：none
* 注    意：none
********************************************/
void ANO_DT_Send_High(int pressure_high, int ultrasonic_high)
{
    uint8_t _cnt = 0;
    int _temp;
    uint8_t sum = 0;
    uint8_t i = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x07;
    data_to_send[_cnt++] = 0;

    _temp = (int)(pressure_high * 100);
    data_to_send[_cnt++] = BYTE3(_temp);
    data_to_send[_cnt++] = BYTE2(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = (int)(ultrasonic_high * 100);
    data_to_send[_cnt++] = BYTE3(_temp);
    data_to_send[_cnt++] = BYTE2(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;
    sum = 0;
    for (i = 0; i < _cnt; i++)
    {
        sum += data_to_send[i];
    }
    data_to_send[_cnt++] = sum;
    ANO_DT_Send_Data(data_to_send, _cnt);
}

/*******************************************
* 函 数 名：ANO_DT_Send_PID
* 功    能：发送PID数据
* 输入参数：group:第几组PID数据
						3组PID数据p,i,d
* 返 回 值：none
* 注    意：第一组PID数据：group=1;
						以此类推
********************************************/
void ANO_DT_Send_PID(uint8_t group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p, float p3_i, float p3_d)
{
    uint8_t _cnt = 0;
    uint8_t sum = 0, i = 0;
    int16_t _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x10 + group - 1;
    data_to_send[_cnt++] = 0;

    _temp = (int16_t)(p1_p * 1000);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p1_i * 1000);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p1_d * 1000);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p2_p * 1000);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p2_i * 1000);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p2_d * 1000);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p3_p * 1000);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p3_i * 1000);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p3_d * 1000);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++)
    {
        sum += data_to_send[i];
    }

    data_to_send[_cnt++] = sum;
    ANO_DT_Send_Data(data_to_send, _cnt);
}

// 以PID数据发送其他参数
void ANO_DT_Send_Fake_PID(uint8_t group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p, float p3_i, float p3_d)
{
    uint8_t _cnt = 0;
    uint8_t sum = 0, i = 0;
    int16_t _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x10 + group - 1;
    data_to_send[_cnt++] = 0;

    _temp = (int16_t)(p1_p);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p1_i);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p1_d);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p2_p);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p2_i);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p2_d);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p3_p);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p3_i);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int16_t)(p3_d);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++)
    {
        sum += data_to_send[i];
    }

    data_to_send[_cnt++] = sum;
    ANO_DT_Send_Data(data_to_send, _cnt);
}
/*******************************************
* 函 数 名：ANO_DT_Send_RCData
* 功    能：发送PID数据
* 输入参数：group:第几组PID数据
						3组PID数据p,i,d
* 返 回 值：none
* 注    意：第一组PID数据：group=1;
********************************************/
void ANO_DT_Send_RCData(uint16_t thr, uint16_t yaw, uint16_t rol, uint16_t pit, uint16_t aux1, uint16_t aux2, uint16_t aux3, uint16_t aux4, uint16_t aux5, uint16_t aux6) //发送遥控器通道数据
{
    uint8_t _cnt = 0;
    uint8_t i = 0;
    uint8_t sum = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x03;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = BYTE1(thr);
    data_to_send[_cnt++] = BYTE0(thr);
    data_to_send[_cnt++] = BYTE1(yaw);
    data_to_send[_cnt++] = BYTE0(yaw);
    data_to_send[_cnt++] = BYTE1(rol);
    data_to_send[_cnt++] = BYTE0(rol);
    data_to_send[_cnt++] = BYTE1(pit);
    data_to_send[_cnt++] = BYTE0(pit);
    data_to_send[_cnt++] = BYTE1(aux1);
    data_to_send[_cnt++] = BYTE0(aux1);
    data_to_send[_cnt++] = BYTE1(aux2);
    data_to_send[_cnt++] = BYTE0(aux2);
    data_to_send[_cnt++] = BYTE1(aux3);
    data_to_send[_cnt++] = BYTE0(aux3);
    data_to_send[_cnt++] = BYTE1(aux4);
    data_to_send[_cnt++] = BYTE0(aux4);
    data_to_send[_cnt++] = BYTE1(aux5);
    data_to_send[_cnt++] = BYTE0(aux5);
    data_to_send[_cnt++] = BYTE1(aux6);
    data_to_send[_cnt++] = BYTE0(aux6);

    data_to_send[3] = _cnt - 4;
    sum = 0;
    for (i = 0; i < _cnt; i++)
    {
        sum += data_to_send[i];
    }
    data_to_send[_cnt++] = sum;
    ANO_DT_Send_Data(data_to_send, _cnt);
}

/*******************************************
* 函 数 名：ANO_Data_Send_Voltage_Current
* 功    能：发送电压、电流【第5组】
* 输入参数：电压、电流
* 返 回 值：none
* 注    意：none
********************************************/
void ANO_Data_Send_Voltage_Current(float volatge, float current)
{
    uint8_t _cnt = 0;
    int16_t _temp;
    uint8_t sum = 0;
    uint8_t i = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x05;
    data_to_send[_cnt++] = 0;

    _temp = (int)(volatge * 100); //电压
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = (int)(current * 100); //电流
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;
    sum = 0;
    for (i = 0; i < _cnt; i++)
    {
        sum += data_to_send[i];
    }
    data_to_send[_cnt++] = sum;
    ANO_DT_Send_Data(data_to_send, _cnt);
}

/*******************************************
* 函 数 名：ANO_Data_Send_User_Data
* 功    能：发送用户自定义数据
* 输入参数：
* 返 回 值：none
* 注    意：none
********************************************/
void ANO_Data_Send_User_Data(int32_t data1, int32_t data2, int32_t data3, int32_t data4)
{
    uint8_t _cnt = 0;
    uint8_t sum = 0;
    uint8_t i = 0;
    int32_t _temp;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xF1;
    data_to_send[_cnt++] = 0;

    _temp = (int)(data1);
    data_to_send[_cnt++] = BYTE3(_temp);
    data_to_send[_cnt++] = BYTE2(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = (int)(data2);
    data_to_send[_cnt++] = BYTE3(_temp);
    data_to_send[_cnt++] = BYTE2(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = (int)(data3);
    data_to_send[_cnt++] = BYTE3(_temp);
    data_to_send[_cnt++] = BYTE2(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = (int)(data4);
    data_to_send[_cnt++] = BYTE3(_temp);
    data_to_send[_cnt++] = BYTE2(_temp);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;
    for (i = 0; i < _cnt; i++)
    {
        sum += data_to_send[i];
    }
    data_to_send[_cnt++] = sum;
    ANO_DT_Send_Data(data_to_send, _cnt);
}

/*******************************************
* 函 数 名：ANO_SEND_StateMachine
* 功    能：依次发送各组数据
********************************************/

float temp, kalman_temp;
void ANO_SEND_StateMachine(void)
{
    static uint8_t cnt;

    switch (++cnt)
    {
    case 1: // 发送基本版本信息(硬件种类、硬件、软件、协议、Bootloader版本)
        ANO_Data_Send_Version((int)HardwareType, (int)HardwareVER, (int)SoftwareVER, (int)ProtocolVER, (int)BootloaderVER);
        break;
    case 2: // 发送基本信息(欧拉角、高度、锁定状态)
        ANO_Data_Send_Status(rovInfo.jy901.roll, rovInfo.jy901.pitch, rovInfo.jy901.yaw, rovInfo.depthSensor.depth);
        break;
    case 3: // 发送传感器数值(加速度、角速度、磁场)
        ANO_DT_Send_Senser(rovInfo.jy901.acc.x, rovInfo.jy901.acc.y, rovInfo.jy901.acc.z,
                           rovInfo.jy901.gyro.x, rovInfo.jy901.gyro.y, rovInfo.jy901.gyro.z,
                           rovInfo.jy901.mag.x, rovInfo.jy901.mag.y, rovInfo.jy901.mag.z);
        break;
    case 4: // 发送推进器接收机摇杆值
        ANO_DT_Send_RCData(rovdev.propellerPower.leftUp + 1500, rovdev.propellerPower.rightUp + 1500,
                           rovdev.propellerPower.leftDown + 1500, rovdev.propellerPower.rightDown + 1500,
                           rovdev.propellerPower.leftMiddle + 1500, rovdev.propellerPower.rightMiddle + 1500,
                           0, 0, 0, 0);
        break;
    case 5: // 发送电源信息(电压、电流)
        ANO_Data_Send_Voltage_Current(rovInfo.powerSource.voltage, rovInfo.powerSource.current);
        break;
    case 6: // 发送高度数据 (气压计高度、超声波高低)
        ANO_DT_Send_High(rovInfo.depthSensor.pressure, rovInfo.depthSensor.init_pressure);
        break;
    case 7: // 发送所有PID数据
        if (ANO_Send_PID_Flag)
        {
            ANO_Send_PID_Flag = 0; // 发送PID标志 清零
            ANO_DT_Send_All_PID();
        }
        break;
    case 8:

        temp = rovInfo.depthSensor.pressure;
        kalman_temp = rovInfo.depthSensor.pressure;
        kalman_filter(&kalman_temp);
        ANO_Data_Send_User_Data(rovInfo.depthSensor.pressure, kalman_temp, 0, 0);
        break;
    }

    if (cnt > 8)
        cnt = 0;
}

void ANO_DT_Send_All_PID(void)
{

    ANO_DT_Send_PID(1, Total_Controller.Roll_Gyro_Control.Kp,
                    Total_Controller.Roll_Gyro_Control.Ki,
                    Total_Controller.Roll_Gyro_Control.Kd,
                    Total_Controller.Pitch_Gyro_Control.Kp,
                    Total_Controller.Pitch_Gyro_Control.Ki,
                    Total_Controller.Pitch_Gyro_Control.Kd,
                    Total_Controller.Yaw_Gyro_Control.Kp,
                    Total_Controller.Yaw_Gyro_Control.Ki,
                    Total_Controller.Yaw_Gyro_Control.Kd);

    ANO_DT_Send_PID(2, Total_Controller.Roll_Angle_Control.Kp,
                    Total_Controller.Roll_Angle_Control.Ki,
                    Total_Controller.Roll_Angle_Control.Kd,
                    Total_Controller.Pitch_Angle_Control.Kp,
                    Total_Controller.Pitch_Angle_Control.Ki,
                    Total_Controller.Pitch_Angle_Control.Kd,
                    Total_Controller.Yaw_Angle_Control.Kp,
                    Total_Controller.Yaw_Angle_Control.Ki,
                    Total_Controller.Yaw_Angle_Control.Kd);

    ANO_DT_Send_PID(3, Total_Controller.High_Speed_Control.Kp,
                    Total_Controller.High_Speed_Control.Ki,
                    Total_Controller.High_Speed_Control.Kd,
                    Total_Controller.High_Position_Control.Kp,
                    Total_Controller.High_Position_Control.Ki,
                    Total_Controller.High_Position_Control.Kd,
                    0, 0, 0);

    ANO_DT_Send_PID(4, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    ANO_DT_Send_PID(5, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    ANO_DT_Send_Fake_PID(6, rovdev.light.nMax,
                         rovdev.light.pMax,
                         rovdev.light.speed,
                         rovdev.yuntai.nMax,
                         rovdev.yuntai.pMax,
                         rovdev.yuntai.speed,
                         rovdev.robot_arm.nMax,
                         rovdev.robot_arm.pMax,
                         rovdev.robot_arm.speed);
    log_i("read pid params -> success!");
}
