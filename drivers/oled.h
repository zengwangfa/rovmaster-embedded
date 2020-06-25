
#ifndef __OLED_H_
#define __OLED_H_

#include "../user/config.h"
#include "oledfont.h"

#define I2C_OLED_DEV "/dev/i2c-2" // OLED 使用的 I2C接口
#define I2C_OLED_ADDR 0x3C        // 默认 OLED 地址 0x3C

#define OLED_CMD 0  //写命令
#define OLED_DATA 1 //写数据

// OLED控制用函数
int OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);

// OLED初始化
int oledSetup(void);
// OLED应用函数
void OLED_Clear(void);

void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_Fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *p, uint8_t size);
void OLED_ShowPicture(uint8_t x, uint8_t y, const uint8_t *p, uint8_t p_w, uint8_t p_h);

void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode);

#endif
