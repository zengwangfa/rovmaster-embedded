/**
 * @desc: system status 系统状态获取方法
 *
 * CPU状态(占用率、温度)
 * 内存状态(占用率、总大小等)
 * 硬盘状态(占用率、总大小等)
 * 网卡网速
 */

#define LOG_TAG "cpu_status"

#include "sys_status.h"

#include <elog.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#define TEMP_PATH "/sys/class/thermal/thermal_zone0/temp"

/**
 * @brief  获取 cpu温度值
 * @return float cpu温度值
 */
float get_cpu_temp(void)
{
    float temp;
    static FILE *fp = NULL;

    // thermal_zone0 表示 CPU0
    fp = fopen(TEMP_PATH, "r");
    if (NULL == fp) // 防止野指针访问，从而产生段错误
        return 0;
    fscanf(fp, "%f", &temp); // 读取第1行 cpu温度数据

    temp /= 1000.0f;

    fclose(fp);
    return temp;
}

/**
 * @brief  获取 CPU总运行时间
 * @param  cpuInfo_t 结构体指针
 * @retval uint32_t cpu总时间
 */
uint32_t get_cpuTotalTime(cpuInfo_t *cpuInfo)
{
    return (cpuInfo->user + cpuInfo->nice + cpuInfo->system +
            cpuInfo->idle + cpuInfo->iowait + cpuInfo->irq +
            cpuInfo->softirq);
}

/**
 * @brief  获取 cpu状态信息
 * @param  cpuInfo_t 结构体指针
 */
void get_cpuInfo(cpuInfo_t *cpuInfo)
{
    static FILE *fp = NULL;

    fp = fopen("/proc/stat", "r");
    if (NULL == fp) // 防止野指针访问，从而产生段错误
        return;

    fscanf(fp, "%s %u %u %u %u %u %u %u",
           cpuInfo->name,
           &cpuInfo->user, &cpuInfo->nice,
           &cpuInfo->system, &cpuInfo->idle,
           &cpuInfo->iowait, &cpuInfo->irq, &cpuInfo->softirq);

    fclose(fp);
}

/**
 * @brief  获取1s内的cpu使用率
 * @retval float cpu使用率
 */
float get_cpu_usage(void)
{
    float cpu_usage;
    uint32_t idle_diff;  // 两次空闲时间的差值
    uint32_t total_diff; // 两次总时间的差值

    cpuInfo_t cpuInfo1;
    cpuInfo_t cpuInfo2;

    /* 注解:
    * CPU总时间 T_total = user+system+nice+idle+iowait+irq+softirq
    * CPU在t1到t2时间段总的使用时间 total_diff = T_total2 - T_total1
    * CPU在t1到t2时间段空闲使用时间 idle_diff  = T_idle2  - T_idle1 
    * CPU在t1到t2时间段即时利用率 cpu_usage = 1 - CPU空闲使用时间 / CPU总的使用时间
    */

    // 第1次获取cpu使用情况
    get_cpuInfo(&cpuInfo1);

    sleep(1); // 休眠1s

    // 第2次获取cpu使用情况
    get_cpuInfo(&cpuInfo2);

    idle_diff = cpuInfo2.idle - cpuInfo1.idle;
    total_diff = get_cpuTotalTime(&cpuInfo2) - get_cpuTotalTime(&cpuInfo1);

    cpu_usage = (float)(total_diff - idle_diff) * 100.0f / total_diff;
    return cpu_usage;
}

/**
 * @brief  获取 内存情况
 * @param  memory_t 结构体指针
 */
void get_memory_status(memory_t *memory)
{
    static FILE *fp = NULL;
    char name1[20]; // 用于保存 内存名称(eg. total/available)
    char name2[20]; // 用于保存 单位    (eg. kB)

    fp = fopen("/proc/meminfo", "r");
    if (NULL == fp) // 防止野指针访问，从而产生段错误
        return;
    fscanf(fp, "%s %u %s", name1, &memory->total, name2);     // 读取第1行 total
    fscanf(fp, "%s %u %s", name1, &memory->free, name2);      // 读取第2行 free
    fscanf(fp, "%s %u %s", name1, &memory->available, name2); // 读取第3行 available

    memory->usage_rate = (float)(memory->total - memory->available) / memory->total * 100.0f;

    fclose(fp);
}

/**
 * @brief  获取 磁盘状态
 * @param  disk_t 结构体指针
 * @notice 在shell下输入 df 命令可以进行对照
 */
void get_disk_status(disk_t *disk)
{
    static FILE *fp = NULL;
    uint32_t b, c;
    // 用于保存各个文件系统的 总容量大小、已使用大小
    float disk_total = 0, disk_used = 0;
    char a[20], d[20], e[20], f[20], buf[100];

    fp = popen("df", "r");
    if (NULL == fp) // 防止野指针访问，从而产生段错误
        return;
    fgets(buf, sizeof(buf), fp); // 读取第1行描述信息(相当于跳过第1行)

    /*  eg.
        Filesystem     1K-blocks   Used Available Use% Mounted on
        udev               85324      0     85324   0% /dev
    */
    while (fscanf(fp, "%s %u %u %s %s %s", a, &b, &c, d, e, f) > 0)
    {
        disk_total += b;
        disk_used += c;
    }

    disk->total = disk_total / 1024; // 转换为Mb单位
    disk->available = (disk_total - disk_used) / 1024;
    disk->usage_rate = disk_used / disk_total * 100;

    pclose(fp);
}

/**
 * @brief  获取对应网卡的网络数据
 * @param  
 *  netData_t 网络数据结构体指针
 *  eth 网卡名
 */
void get_net_data(netData_t *net_data, char *eth)
{
    static FILE *fp = NULL;
    char name[20];
    uint64_t rb, rp, re, rd, rfi, rfr, rc, rm,
        tb, tp, te, td, tfi, tfr, tc, tm;

    fp = fopen("/proc/net/dev", "r");
    if (NULL == fp) // 防止野指针访问，从而产生段错误
        return;

    while (fscanf(fp,
                  "%s %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu",
                  name, &rb, &rp, &re, &rd, &rfi, &rfr, &rc, &rm, &tb, &tp,
                  &te, &td, &tfi, &tfr, &tc, &tm) > 0)
    {
        // 比较网卡名
        if (strcmp(name, eth) == 0)
        {
            net_data->rb = rb;
            net_data->rp = rp;
            net_data->tb = tb;
            net_data->tp = tp;
            break;
        }
    }
    /* 必须关闭文件
     * 打开大量文件并且不关闭, 很快会达到进程最大允许打开的文件数限制，这样就不能再打开文件。
     * 在Linux上，可以通过ulimit -n 来查看和更改当前session的限制数 
     */
    fclose(fp);
}

/**
 * @brief  获取对应网卡的网络数据
 * @param  网卡名
 * @notice 注意：这里进行了单位转换，转换为 kb(而不是 kB)
 */
float get_net_speed(char *eth)
{
    char ethc[20];
    float netspeed;
    netData_t nd1, nd2;

    sprintf(ethc, "%s:", eth); // 在网卡名后加上 冒号“:”

    get_net_data(&nd1, ethc);
    sleep(1);
    get_net_data(&nd2, ethc);

    netspeed = (float)((nd2.rb + nd2.tb) - (nd1.rb + nd1.tb)) / 1024 * 8; // 转换为 kbps

    return netspeed;
}

/**
  * @brief  获取对应网卡的IP地址
  * @param  eth_name:网卡名   ip:数组首地址
  */
void get_localip(const char *eth_name, char *ip)
{
    int fd;
    struct ifreq ifr;

    if (eth_name == NULL)
        return;

    strcpy(ifr.ifr_name, eth_name);
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) > 0)
    {
        if (!(ioctl(fd, SIOCGIFADDR, &ifr)))
            strcpy(ip, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
        close(fd);
    }
}
