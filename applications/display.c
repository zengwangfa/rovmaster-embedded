
/*
 * @Description: ROV状态数据回传与控制命令接收解析，获取 系统状态(CPU、内存、硬盘、网卡网速)
 */

#define LOG_TAG "display"

#include "../drivers/oled.h"
#include "../user/datatype.h"

#include "data.h"
#include "display.h"

#include <elog.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>

/**
  * @brief  oled显示系统状态(IP、内存、硬盘、CPU、网速)
  */
void oled_show_status(void)
{
    char str[20];

    sprintf(str, "IP  %s", rovInfo.net.ip);
    OLED_ShowString(0, 0, (uint8_t *)str, 12);

    sprintf(str, "Mem: %.1f%% of %d Mb", rovInfo.memory.usage_rate, rovInfo.memory.total / 1024);
    OLED_ShowString(0, 16, (uint8_t *)str, 12);

    sprintf(str, "Disk: %.1f%% of %0.1f G", rovInfo.disk.usage_rate, (float)rovInfo.disk.total / 1024);
    OLED_ShowString(0, 32, (uint8_t *)str, 12);

    sprintf(str, "CPU:%.1f%%  ", rovInfo.cpu.usage_rate);
    OLED_ShowString(0, 48, (uint8_t *)str, 12);

    if (rovInfo.net.netspeed < 512)
    {
        // 此时单位为 kbps
        sprintf(str, "%.1f kb/s", rovInfo.net.netspeed);
        OLED_ShowString(70, 48, (uint8_t *)str, 12);
    }
    else
    {
        // 转换单位为 Mbps
        sprintf(str, "%.1f Mb/s", rovInfo.net.netspeed / 1024);
        OLED_ShowString(70, 48, (uint8_t *)str, 12);
    }
}

void oled_show_logo(void)
{
    OLED_ShowPicture(31, 0, snail_bmp, 64, 64);
    sleep(1);
    OLED_Clear();
}

/*******************************************************************************************************************/
//
// 线程
//
/*******************************************************************************************************************/

/**
  * @brief  oled显示线程
  */
void *display_thread(void *arg)
{
    oled_show_logo();

    while (1)
    {
        oled_show_status();
        sleep(1);
    }
    return NULL;
}

/**
  * @brief  显示线程 初始化
  */
int display_thread_init(void)
{
    int fd;
    pthread_t display_tid;

    fd = oledSetup();
    // 判断 对应i2c接口及oled是否存在，不存在直接返回，不创建对应线程
    if (fd < 0)
    {
        // 错误日志打印
        ERROR_LOG(fd, "oled");
        return -1;
    }

    log_i("oled    init");
    pthread_create(&display_tid, NULL, &display_thread, NULL);
    pthread_detach(display_tid);

    return 0;
}
