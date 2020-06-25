

#ifndef __SERVER_H_
#define __SERVER_H_

#include "../user/config.h"

#define LISTEN_PORT 8888 // 监听端口号
#define BACKLOG 10       // 最大连接数

// 上位机通信服务器线程初始化
int control_server_thread_init(void);

#endif
