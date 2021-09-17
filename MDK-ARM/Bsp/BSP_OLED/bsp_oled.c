/**
 ***************************************(C) COPYRIGHT 2021 DIODE***************************************
 * @file       oled.c
 * @brief     
 * @note       SPI1 + PB10
 * @Version    V1.0.0
 * @Date       2021.5      
 ***************************************(C) COPYRIGHT 20 DIODE***************************************
 */

#include "bsp_oled.h"
#include "oledfont.h"
#include "math.h"
#include <stdio.h>
#include <stdarg.h>

/**
 * OLED flash Addr:
 * [0]0 1 2 3 ... 127
 * [1]0 1 2 3 ... 127
 * [2]0 1 2 3 ... 127
 * [3]0 1 2 3 ... 127
 * [4]0 1 2 3 ... 127
 * [5]0 1 2 3 ... 127
 * [6]0 1 2 3 ... 127
 * [7]0 1 2 3 ... 127
**/

static uint8_t OLED_GRAM[128][8];

//short delay uesd in spi transmmit
void delay_ms(uint16_t delaytimes)
{
    uint16_t i;
    for (i = 0; i < delaytimes; i++ )
    {
        int a = 10000;  //delay based on mian clock, 168Mhz
        while (a-- );
    }
}


/**
 * @brief   write data/command to OLED
 * @param   dat: the data ready to write
 * @param   cmd: 0x00,command 0x01,data
 * @retval  
 */
void OLED_WriteByte(uint8_t dat, uint8_t cmd)
{
    if (cmd != 0)
        OLED_CMD_Set();
    else
        OLED_CMD_Clr();

    HAL_SPI_Transmit(&hspi1, &dat, 1, 10);
}


/**
 * @brief   set OLED cursor position
 * @param   x: the X-axis of cursor
 * @param   y: the Y-axis of cursor
 * @retval  
 */
static void oled_set_pos(uint8_t x, uint8_t y)
{
//    x += 2;
    OLED_WriteByte((0xb0 + y), OLED_CMD);              //set page address y
    OLED_WriteByte(((x&0xf0)>>4)|0x10, OLED_CMD);      //set column high address
    OLED_WriteByte((x&0xf0), OLED_CMD);                //set column low address
}

/**
 * @brief   turn on OLED display
 * @param   None
 * @param   None
 * @retval  
 */
void OLED_DisplayOn(void)
{
    OLED_WriteByte(0x8d, OLED_CMD);
    OLED_WriteByte(0x14, OLED_CMD);
    OLED_WriteByte(0xaf, OLED_CMD);
}

/**
 * @brief   turn off OLED display
 * @param   None
 * @param   None
 * @retval  
 */
void OLED_DisplayOff(void)
{
    OLED_WriteByte(0x8d, OLED_CMD);
    OLED_WriteByte(0x10, OLED_CMD);
    OLED_WriteByte(0xae, OLED_CMD);
}

/**
 * @brief   refresh the RAM of OLED
 * @param   None
 * @param   None
 * @retval  
 */
void OLED_RefreshGram(void)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        oled_set_pos(0, i);

        for (n = 0; n < 128; n++)
        {
            OLED_WriteByte(OLED_GRAM[n][i], OLED_DATA);
        }
    }
}

/**
 * @brief   clear the screen
 * @param   None
 * @param   None
 * @retval  
 */
void OLED_Clear(Pen_Typedef pen)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 128; n++)
        {
            if (pen == Pen_Write)
                OLED_GRAM[n][i] = 0xff;
            else if (pen == Pen_Clear)
                OLED_GRAM[n][i] = 0x00;
            else
                OLED_GRAM[n][i] = 0xff - OLED_GRAM[n][i];
        }
    }
}

/**
 * @brief   draw a point at (x, y)
 * @param   x: the X-axis of cursor
 * @param   y: the Y-axis of cursor
 * @param   pen: Pen_Clear, Pen_Write, Pen_Inversion @Pen_Typedef
 * @retval  None
 */
void OLED_DrawPoint(int8_t x, int8_t y, Pen_Typedef pen)
{
    uint8_t page = 0, row = 0;

    /* check the corrdinate */
    if ((x < 0) || (x > (X_WIDTH - 1)) || (y < 0) || (y > (Y_WIDTH - 1)))
        return;

    page = y / 8;
    row = y % 8;

    if (pen == Pen_Write)
        OLED_GRAM[x][page] |= 1 << row;
    else if (pen == Pen_Inversion)
        OLED_GRAM[x][page] ^= 1 << row;
    else
        OLED_GRAM[x][page] &= ~(1 << row);
}

/**
 * @brief   draw a line from (x1, y1) to (x2, y2)
 * @param   x1, y1: the start point of line
 * @param   x2, y2: the end of line
 * @param   pen: Pen_Clear, Pen_Write, Pen_Inversion @Pen_Typedef
 * @retval  None
 */
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, Pen_Typedef pen)
{
    uint8_t col = 0, row = 0;
    uint8_t x_st = 0, x_ed = 0, y_st = 0, y_ed = 0;
    float k = 0.0f, b = 0.0f;

    if (y1 == y2)
    {
        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x1);

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_DrawPoint(col, y1, pen);
        }
    }
    else if (x1 == x2)
    {
        (y1 <= y2) ? (y_st = y1):(y_st = y2);
        (y1 <= y2) ? (y_ed = y2):(y_ed = y1);

        for (row = y_st; row <= y_ed; row++)
        {
            OLED_DrawPoint(x1, row, pen);
        }
    }
    else
    {
        k = ((float)(y2 - y1)) / (x2 - x1);
        b = (float)y1 - k * x1;

        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x2);

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_DrawPoint(col, (uint8_t)(col * k + b), pen);
        }
    }
}


//To add: rectangle, fillrectangle, circle, fillcircle, 

/**
 * @brief   show a character
 * @param   row: row of character
 * @param   col: column of character
 * @param   chr: the character ready to show
 * @retval  None
 */
void OLED_ShowChar(uint8_t row, uint8_t col, uint8_t chr)
{
    uint8_t x = col * 6;
    uint8_t y = row * 12;
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';

    for (t = 0; t < 12; t++)
    {
        temp = asc2_1206[chr][t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp&0x80)
                OLED_DrawPoint(x, y, Pen_Clear);
            else
                OLED_DrawPoint(x, y, Pen_Write);

            temp <<= 1;
            y++;
            if ((y - y0) == 12)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

//m^n
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while (n--)
        result *= m;

    return result;
}

/**
 * @brief   show a number
 * @param   row: row of number
 * @param   col: column of number
 * @param   num: the number ready to show
 * @param   mode: 0x01, fill number with '0'; 0x00, fill number with spaces
 * @param   len: the length of the number
 * @retval  None
 */
void OLED_ShowNum(uint8_t row, uint8_t col, uint32_t num, uint8_t mode, uint8_t len)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t -1)) % 10;

        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                if (mode == 0)
                    OLED_ShowChar(row, col + t, ' ');
                else
                    OLED_ShowChar(row, col + t, '0');
                continue;
            }
            else
                enshow = 1;
        }

        OLED_ShowChar(row, col + t, temp + '0');
    }
}
/**
 * @brief   显示浮点数
 * @param   row: row of number
 * @param   col: column of number
 * @param   num: the number ready to show
 * @param   mode: 0x01, fill number with '0'; 0x00, fill number with spaces
 * @param   len1: 整数部分长度
 * @param   len2: 小数部分长度
 * @retval  None
 * @note	实际长度大于参数时，会舍去高位
 */
void OLED_ShowFloat(uint8_t row, uint8_t col, float num, uint8_t mode, uint8_t len1,uint8_t len2)
{
	uint16_t t_int,t_float;
	if(num<0)
	{
		num=-num;
		OLED_ShowChar(row,col,'-');
	}
	else
		OLED_ShowChar(row,col,'+');
	t_int = num;	// 计算整数部分
	t_float = (num-(int)num)*pow(10,len2);	// 计算小数部分

	
	OLED_ShowNum(row,col+1,t_int,0,len1);
	OLED_ShowChar(row,col+1+len1,'.');
	OLED_ShowNum(row,col+1+len1+1,t_float,0,len2);
}

/**
 * @brief   show a character string
 * @param   row: row of character string begin
 * @param   col: column of character string begin
 * @param   chr: the pointer to character string
 * @retval  None
 */
void OLED_ShowString(uint8_t row, uint8_t col, uint8_t *chr)
{
    uint8_t n =0;

    while (chr[n] != '\0')
    {
        OLED_ShowChar(row, col, chr[n]);
        col++;

        if (col > 20)
        {
            col = 0;
            row += 1;
        }
        n++;
    }
}

/**
 * @brief   formatted output in oled 128*64
 * @param   row: row of character string begin, 0 <= row <= 4;
 * @param   col: column of character string begin, 1<= col <= 20;
 * @param   *fmt: the pointer to format character string
 * @retval  None
 * @note    if the character length is more than one row at a time, the extra characters will be truncated
 */
void OLED_Printf(uint8_t row, uint8_t col, const char *fmt,...)
{
    uint8_t LCD_BUF[128] = {0};
    uint8_t remain_size = 0;
    va_list ap;

    if ((row > 4) || (col > 20) || (col < 1))
        return;

    va_start(ap, fmt);

    vsprintf((char *)LCD_BUF, fmt, ap);

    va_end(ap);

    remain_size = 21 - col;

    LCD_BUF[remain_size] = '\0';

    OLED_ShowString(row, col, LCD_BUF);
}

void OLED_LOGO(void)
{
    OLED_Clear(Pen_Clear);
    uint8_t temp_char = 0;
    uint8_t x = 0, y = 0;
    uint8_t i = 0;
    for(; y < 64; y += 8)
    {
        for(x = 0; x < 128; x++)
        {
            temp_char = LOGO_BMP[x][y/8];
            for(i = 0; i < 8; i++)
            {
                if(temp_char & 0x80) OLED_DrawPoint(x, y + i,Pen_Write);
                else OLED_DrawPoint(x,y + i,Pen_Clear);
                temp_char <<= 1;
            }
        }
    }
    OLED_RefreshGram();
}
/*
使用前，初始化ADC1 IN6
返回值	方向	数字量
0		不按	4095
1		中间	0
2		左		847.78
3		右		1731.10
4		上		2457.60
5		下		3280.06
*/
uint8_t OLED_BTN_Get(void)
{
    uint8_t key,err=100;
    int adc = ADC_Get(6);
    if (abs(adc-0)<err)
        key = 1;
    else if(abs(adc-847)<err)
        key = 2;
    else if(abs(adc-1730)<err)
        key = 3;
    else if(abs(adc-2457)<err)
        key = 4;
    else if(abs(adc-3280)<err)
        key = 5;
    else
        key = 0;
    return key;
}
/**
 * @brief   initialize the oled module
 * @param   None
 * @retval  None
 */
void OLED_Init(void)
{
    OLED_RST_Clr();
    HAL_Delay(500);
    OLED_RST_Set();

    OLED_WriteByte(0xae, OLED_CMD);    //turn off oled panel
    OLED_WriteByte(0x00, OLED_CMD);    //set low column address
    OLED_WriteByte(0x10, OLED_CMD);    //set high column address
    OLED_WriteByte(0x40, OLED_CMD);    //set start line address
    OLED_WriteByte(0x81, OLED_CMD);    //set contrast control resigter
    OLED_WriteByte(0xcf, OLED_CMD);    //set SEG output current brightness
    OLED_WriteByte(0xa1, OLED_CMD);    //set SEG/column mapping
    OLED_WriteByte(0xc8, OLED_CMD);    //set COM/row scan direction
    OLED_WriteByte(0xa6, OLED_CMD);    //set nomarl display
    OLED_WriteByte(0xa8, OLED_CMD);    //set multiplex display
    OLED_WriteByte(0x3f, OLED_CMD);    //1/64 duty
    OLED_WriteByte(0xd3, OLED_CMD);    //set display offset
    OLED_WriteByte(0x00, OLED_CMD);    //not offest
    OLED_WriteByte(0xd5, OLED_CMD);    //set display clock divide ratio/oscillator frequency
    OLED_WriteByte(0x80, OLED_CMD);    //set divide ratio 
    OLED_WriteByte(0xd9, OLED_CMD);    //set pre-charge period
    OLED_WriteByte(0xf1, OLED_CMD);    //pre-charge: 15 clocks, discharge: 1 clock
    OLED_WriteByte(0xda, OLED_CMD);    //set com pins hardware configuration 
    OLED_WriteByte(0x12, OLED_CMD);    //
    OLED_WriteByte(0xdb, OLED_CMD);    //set vcomh
    OLED_WriteByte(0x40, OLED_CMD);    //set vcom deselect level
    OLED_WriteByte(0x20, OLED_CMD);    //set page addressing mode
    OLED_WriteByte(0x02, OLED_CMD);    //
    OLED_WriteByte(0x8d, OLED_CMD);    //set charge pump enable/disable
    OLED_WriteByte(0x14, OLED_CMD);    //charge pump disable
    OLED_WriteByte(0xa4, OLED_CMD);    //disable entire dispaly on
    OLED_WriteByte(0xa6, OLED_CMD);    //disable inverse display on
    OLED_WriteByte(0xaf, OLED_CMD);    //turn on oled panel

    OLED_WriteByte(0xaf, OLED_CMD);    //display on

    OLED_Clear(Pen_Clear);
    oled_set_pos(0, 0);

}


