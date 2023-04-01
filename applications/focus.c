/**
 * @desc: 变焦镜头控制
 */
#define LOG_TAG "focus"

#include "focus.h"
#include "data.h"


#include <pthread.h>
#include "wiringPi.h"
#include <wiringSerial.h>


static int fd;
static int is_ok;
/* 返回数据包，包头位固定为：0xAA,0x55;  数据长度位：0x02 */
uint8_t camera_control_data[CAMERA_CONTROL_DATA_LEN] = {0xAA, 0x55, 0x02};

/**
 * @brief  变焦镜头控制
 * @param  action 控制指令
 * @notice 0x01表示放大，0x02表示缩小， 0x11表示拉近，0x12表示拉远，0x00表示不动作（默认）
 */
void focus_zoom_camera_control(uint8_t *action)
{
    if (*action)
    {
        switch (*action)
        {
        case FOCUS_MAGNIFY: // 聚焦放大
            camera_control_data[3] = 0x01;
            camera_control_data[4] = 0x00;
            break;
        case FOCUS_LESSEN: // 聚焦缩小
            camera_control_data[3] = 0x02;
            camera_control_data[4] = 0x00;
            break;
        case ZOOM_IN: // 变焦拉近
            camera_control_data[3] = 0x00;
            camera_control_data[4] = 0x01;
            break;
        case ZOOM_OUT: // 变焦拉远
            camera_control_data[3] = 0x00;
            camera_control_data[4] = 0x02;
            break;
        default: // 设定变焦步进角度
            camera_control_data[3] = *action;
            camera_control_data[4] = *action;
            break;
        }
        // 计算校验位
        camera_control_data[5] = calculate_check_sum(camera_control_data, sizeof(camera_control_data) - 1);
        write(fd, camera_control_data, sizeof(camera_control_data));
        *action = 0x00;
    }
}

/**
 * @brief  接收变焦控制器反馈字符串线程(校验对变焦控制器控制是否成功)
 * @notice 变焦控制器初始化成功以及接受到指令后都会返回"ok"
 * 		   (因此可以通过向变焦控制器发送 设定步进角度 校验其是否存在及可控)
 */
void *focus_camera_receive_thread(void *arg)
{
    char rxBuffer[2];

    while (1)
    {
        delay(10);
        while (serialDataAvail(fd))
        {
            read(fd, rxBuffer, 2); // 获取是否有反馈
            if (!strcmp(rxBuffer, "ok"))
                is_ok = 1;
            memset(rxBuffer, 0, 2); // 清空
        }
    }
    return NULL;
}

/**
 * @brief  校验对变焦控制器控制是否成功
 * @param  数据字符
 * @notice 变焦控制器初始化成功以及接受到指令后都会返回"ok"
 * 		   (因此可以通过向变焦控制器发送 设定步进角度 校验其是否存在及可控)
 * @retval 0代表成功，-1为失败
 */
void *focus_camera_thread(void *arg)
{
    while (1)
    {
        focus_zoom_camera_control(&cmd_data.camera);
        delay(10);
    }
}

int focus_camera_thread_init(void)
{
    pthread_t focus_tid;
    pthread_t focus_recv_tid;
    uint8_t focus_cam_step_angle = 10; // 步进角度为10，即变焦控制器每接收到一次指令转10个单位，一个周期为360
    static uint8_t is_ok_flag = 0;

    
    // 先创建用于接收反馈的线程，用于后面初始化检验
    pthread_create(&focus_recv_tid, NULL, &focus_camera_receive_thread, NULL);
    pthread_detach(focus_recv_tid);

    // 小于0代表无法找到该uart接口，输入命令 sudo npi-config 使能该uart接口
    fd = serialOpen(FOCUS_CAMERA_UART_DEV, 115200);
    if (fd < 0)
    {
        printf("focus camera uart init failed");
        return -1;
    }
    focus_zoom_camera_control(&focus_cam_step_angle); // 发送设定步进角度 变焦控制器是否存在

    if(is_ok_flag==0)
    {
        is_ok_flag = 1;
        is_ok = 1;
    }
    delay(1000);                                      // 等待反馈
    if (is_ok)
    {
        is_ok = 0; //清零
        printf("focus camera init");
        pthread_create(&focus_tid, NULL, &focus_camera_thread, NULL);
        pthread_detach(focus_tid);
    }
    else
        printf("focus camera init failed");

    return 0;
}
