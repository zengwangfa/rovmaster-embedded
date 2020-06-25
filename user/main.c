
/**
 * @desc: 主程序
 */
#define LOG_TAG "main"

#include "config.h"
#include "datatype.h"
#include "init.h"
#include <elog.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <wiringPi.h>

// <CTRL>+C 程序中断回调
static void signal_handler(int sig)
{

    log_w("force cancellation of threads and cleanup resources\n");

    exit(0);
}

// TODO 是否需要使用fork函数

void *mjpg_streamer_thread(void *arg)
{
    system("./tools/video.sh &"); // 打开视频推流脚本
    return NULL;
}

void pirnf_menu(void)
{
    printf("1. set arm");
}

static void help(char *progname)
{
    fprintf(stderr, "------------------ %s menu ------------------\n", progname);
    fprintf(stderr, "Usage:\n"
                    "  [1] light  <min> <max> <speed> | 设置探照灯 \n"
                    "  [2] yuntai <min> <max> <speed> | 设置云台 \n"
                    "  [3] arm    <min> <max> <speed> | 设置机械臂 \n"
                    "  [3] rovinfo | 打印rov传感器信息 \n");
}

void printf_pwmDev_info(easyPWM_dev_t *pwm)
{
    printf("%s %5d %5d %5d\n", pwm->name, pwm->nMax, pwm->pMax, pwm->speed);
}

void printf_rovinfo(void)
{
    printf(" depth sensor type  |  %s \n", rovInfo.depthSensor.name);          // 深度传感器类型
    printf(" water temperature  |  %.2f \n", rovInfo.depthSensor.temperature); // 水温
    printf("   init_pressure    |  %d \n", rovInfo.depthSensor.init_pressure); // 深度传感器初始压力值
    printf("   sensor_pressure  |  %d \n", rovInfo.depthSensor.pressure);      // 深度传感器当前压力值
    printf("        depth       |  %.2f \n", rovInfo.depthSensor.depth);       // 深度值
}

static void switch_menu(char *buf)
{
    int len;
    const char *p;
    /* 遇到空格分割符，则 len = 从开头到空格的长度
	 * 未遇到空格分割符，则len = strlen(buf) - 1 (-1 是为了扣掉回车符)
	*/
    len = ((p = strchr(buf, ' ')) == NULL) ? (strlen(buf) - 1) : (p - buf);

    if (len) // 只有会车符不处理
    {

        if (strncmp(buf, "help", len) == 0) // 帮助信息
            help("rov-master");
        else if (strncmp(buf, "light", len) == 0)
            printf_pwmDev_info(&rovdev.light);
        else if (strncmp(buf, "yuntai", len) == 0)
            printf_pwmDev_info(&rovdev.yuntai);
        else if (strncmp(buf, "arm", len) == 0)
            printf_pwmDev_info(&rovdev.robot_arm);
        else if (strncmp(buf, "rovinfo", len) == 0)
            printf_rovinfo();
        else
            printf("input error, you can enter \"help\" to list cmds");
    }
    // 遍历param文件，根据 空格分割符，查找参数名，从而读取参数
    // TODO 写入文件后，提醒文件读取参数
    /* // TODO cmdtp->handle
for (cmdtp = &__u_boot_cmd_start;
	     cmdtp != &__u_boot_cmd_end;
	     cmdtp++) {
		if (strncmp (cmd, cmdtp->name, len) == 0) {
			if (len == strlen (cmdtp->name))
				return cmdtp;	

			cmdtp_temp = cmdtp;	
			n_found++;
		}
	}
*/
    /*
        if (1 == len)
    {
        // TODO 检测命令为 yuntai、arm、light
        switch (buf[0]) // 当输入单个数字
        {
        case '1':
            printf("yuntai max min med speed\n");
            printf("yuntai 100  0  50   10\n");
            // TODO 写入文件后，提醒文件读取参数
            break;
        case '2':
            printf("robot arm \n");
            break;
        case '3':
            printf("light \n");
            break;
        }
    }
	*/
}

int main(int argc, char **argv)
{
    pthread_t mjpg_tid;
    char buf[50];

    system_init();

    /* 注册 signal_handler 忽略掉终端<CTRL>+C 产生的SIGINT退出信号，为了在退出前做一些清理工作 */
    if (signal(SIGINT, signal_handler) == SIG_ERR)
        log_e("could not register signal handler\n");

    pthread_create(&mjpg_tid, NULL, mjpg_streamer_thread, NULL);
    pthread_detach(mjpg_tid);

    while (1) // 此处while循环是为了本程序不退出
    {
        printf("> ");
        printf("you can enter \"help\" to list cmds");
        while (fgets(buf, sizeof(buf), stdin) != NULL)
        {
            switch_menu(buf); // 选择菜单
            printf("> ");
        }
    }
    return 0;
}
