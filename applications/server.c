/**
 * @desc:  数据链路服务器，用于传输与接收数据
 */

#define LOG_TAG "server"

#include "server.h"
#include "../drivers/sys_status.h"
#include "data.h"

#include <arpa/inet.h>
#include <elog.h>
#include <errno.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <wiringPi.h>

/* 接收数据包 */
static uint8_t recv_buff[RECV_DATA_LEN] = {0};
/* 返回数据包，包头位固定为：0xAA,0x55;  数据长度位：0x16 */
static uint8_t return_data[RETURN_DATA_LEN] = {0xAA, 0x55, 0x16};

static int server_sock = -1;
static int client_sock = -1;

void print_hex_data(const char *name, uint8_t *data, int len)
{
    printf("%s:", name);
    for (int i = 0; i < len; i++)
    {
        printf("%2x ", data[i]);
    }
    printf("\n");
}

/**
  * @brief  数据发送至上位机线程
  */
void *send_thread(void *arg)
{
    while (1)
    {
        /* 转换ROV状态数据，发送 */
        convert_rov_status_data(return_data);
        if (write(client_sock, return_data, RETURN_DATA_LEN) < 0)
        {
            // 发送失败，则关闭当前socket，client_sock 置为 -1，等待下次连接
            if (client_sock != -1)
            {
                log_i("IP [%s] client closed", arg);
                close(client_sock);
                client_sock = -1;
            }
            return NULL;
        }
        //print_hex_data("send", return_data, RETURN_DATA_LEN);
        sleep(1); // 1s更新一次
    }
    return NULL;
}

/**
  * @brief  接收上位机数据线程
  */
void *recv_thread(void *arg)
{
    while (1)
    {
        if (recv(client_sock, recv_buff, RECV_DATA_LEN, 0) < 0)
        {
            // 接收失败，则关闭当前socket，client_sock = -1，等待下次连接
            if (client_sock != -1)
            {
                log_i("IP [%s] client closed", arg);
                close(client_sock);
                client_sock = -1;
            }
            return NULL;
        }
        print_hex_data("recv", recv_buff, RECV_DATA_LEN);
        /* 接收遥控数据解析 */
        remote_control_data_analysis(recv_buff, &cmd_data);
    }
    return NULL;
}

/**
  * @brief  数据服务器线程
  * @notice 该线程创建两个线程用于接收与发送数据
  */
void *server_thread(void *arg)
{
    static int opt = 1;        // 套接字选项 = 1: 使能地址复用
    static uint16_t clientCnt; // 记录客户端连接的次数
    static socklen_t addrLen = sizeof(struct sockaddr);
    static char serverip[20]; // 保存本地 eth0 IP地址
    static char clientip[20]; // 保存客户端 IP地址

    static struct sockaddr_in serverAddr;
    static struct sockaddr_in clientAddr; // 用于保存客户端的地址信息

    pthread_t send_tid;
    pthread_t recv_tid;

    /* 1.初始化服务器socket */
    if ((server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
    {
        log_e("create server socket error:%s(errno:%d)\n", strerror(errno), errno);
        exit(1);
    }

    // 设置套接字, SO_REUSERADDR 允许重用本地地址和端口，允许绑定已被使用的地址（或端口号）
    if (setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
        log_e("setsockopt port for reuse error:%s(errno:%d)\n", strerror(errno), errno);
    }

    /* 2.设置服务器sockaddr_in结构 */
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(LISTEN_PORT);

    /* 3.绑定socket和端口 */
    if (bind(server_sock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        log_e("bind socket error:%s(errno:%d)", strerror(errno), errno);
        exit(1);
    }

    /* 4.监听,最大连接客户端数 BACKLOG */
    if (listen(server_sock, BACKLOG) < 0)
    {
        log_e("listen socket error :%s(errno:%d)", strerror(errno), errno);
        exit(1);
    }
    log_i("waiting for clients to connect ...");

    // 获取eth0的 ip地址
    get_localip("eth0", serverip);
    log_i("rov server start [%s:%d]", serverip, LISTEN_PORT);

    while (1)
    {
        /* 5.接受客户请求，并创建线程处理(可连续接收多个客户端，每个客户端创建2个线程进行处理) */
        if ((client_sock = accept(server_sock, (struct sockaddr *)&clientAddr, &addrLen)) < 0)
        {
            log_e("accept socket error:%s(errorno:%d)", strerror(errno), errno);
            continue;
        }
        // 获取客户端IP地址
        strncpy(clientip, inet_ntoa(clientAddr.sin_addr), sizeof(clientip));
        // 打印客户端连接次数及IP地址
        log_i("conneted success from clinet [NO.%d] IP: [%s]", ++clientCnt, clientip);

        pthread_create(&send_tid, NULL, send_thread, clientip);
        pthread_detach(send_tid);

        pthread_create(&recv_tid, NULL, recv_thread, clientip);
        pthread_detach(recv_tid);
    }
    close(server_sock);
}

/**
  * @brief  上位机数据通信服务器线程初始化
  */
int control_server_thread_init(void)
{
    pthread_t server_tid;

    pthread_create(&server_tid, NULL, server_thread, NULL);
    pthread_detach(server_tid);

    return 0;
}
