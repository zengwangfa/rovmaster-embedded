#ifndef __DEBUG_H_
#define __DEBUG_H_

// ANO匿名地面站UDP服务器线程初始化
int anoUdp_server_thread_init(void);
// ANO匿名地面站UDP发送数据
void ano_udp_send(uint8_t *dataToSend, uint8_t len);

#endif
