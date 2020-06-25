/*
 * @Description: system status 系统状态获取方法(CPU、内存、硬盘、网卡网速)
 */

#ifndef __SYS_STATUS_H_
#define __SYS_STATUS_H_

#include "../user/config.h"

typedef struct // 定义一个cpu occupy的结构体
{
    char name[8];
    uint32_t user;    // 用户态时间 单位：jiffies (1 jiffies = 10ms)
    uint32_t nice;    // 用户态时间(低优先级，nice>0)
    uint32_t system;  // 内核态时间
    uint32_t idle;    // 空闲时间
    uint32_t iowait;  // I/O等待时间
    uint32_t irq;     // 硬中断
    uint32_t softirq; // 软中断
} cpuInfo_t;

/* 网络数据 描述符 */
typedef struct
{
    uint64_t rp, rb;
    uint64_t tp, tb;

} netData_t;

// 内存状况
typedef struct
{
    uint32_t total;     // 内存总数
    uint32_t free;      // 完全未用到的物理内存
    uint32_t available; // 可用内存数 MemAvailable ≈ MemFree + Buffers(文件缓冲大小) + Cached(高速缓冲大小)
    float usage_rate;   // 使用率
} memory_t;

// 硬盘状况
typedef struct
{
    uint32_t total;
    uint32_t available;
    float usage_rate;
} disk_t;

// CPU 状态
typedef struct
{
    float usage_rate;
    float temperature;
} cpu_t;

// 网卡 状态
typedef struct
{
    char *name;     // 网卡名
    char ip[20];    // 保存ip地址
    float netspeed; // 网速
} net_t;

/* --------------------------------------------------------------------------------------------------- */

// 获取 CPU 温度
float get_cpu_temp(void);
// 获取 1s 内的 CPU 使用率
float get_cpu_usage(void);

// 获取 硬盘状态
void get_disk_status(disk_t *disk);
// 获取 内存状态
void get_memory_status(memory_t *memory);

// 获取对应网卡网速
float get_net_speed(char *eth);
// 获取对应网卡IP地址
void get_localip(const char *eth_name, char *ip);

#endif /* __SYS_STATUS_H_ */
