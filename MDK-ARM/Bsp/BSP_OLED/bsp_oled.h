/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       oled.h
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-28-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __BSP_OLED__H
#define __BSP_OLED__H

#include "stm32f4xx.h"
#include "spi.h"
#include <stdint.h>

#define Max_Column      128
#define Max_Row         64

#define X_WIDTH         128
#define Y_WIDTH         64

#define OLED_CMD        0x00
#define OLED_DATA       0x01

#define CHAR_SIZE_WIDTH     6
#define VHAR_SIZE_HIGHT     12

#define OLED_CMD_Set()      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)
#define OLED_CMD_Clr()      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)

#define OLED_RST_Set()      HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET)
#define OLED_RST_Clr()      HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET)

typedef enum
{
    Pen_Clear = 0x00,
    Pen_Write = 0x01,
    Pen_Inversion = 0x02,
}Pen_Typedef;

/* function define */
void OLED_Init(void);
void OLED_WriteByte(uint8_t dat, uint8_t cmd);
void OLED_DisplayOn(void);
void OLED_DisplayOff(void);
void OLED_RefreshGram(void);
void OLED_Clear(Pen_Typedef pen);
void OLED_DrawPoint(int8_t x, int8_t y, Pen_Typedef pen);
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, Pen_Typedef pen);
void OLED_ShowChar(uint8_t row, uint8_t col, uint8_t chr);
void OLED_ShowNum(uint8_t row, uint8_t col, uint32_t num, uint8_t mode, uint8_t len);
void OLED_ShowString(uint8_t row, uint8_t col, uint8_t *chr);
void OLED_Printf(uint8_t row, uint8_t col, const char *fmt,...);
void OLED_ShowFloat(uint8_t row, uint8_t col, float num, uint8_t mode, uint8_t len1,uint8_t len2);

void OLED_LOGO(void);
uint8_t OLED_BTN_Get(void);

#endif

