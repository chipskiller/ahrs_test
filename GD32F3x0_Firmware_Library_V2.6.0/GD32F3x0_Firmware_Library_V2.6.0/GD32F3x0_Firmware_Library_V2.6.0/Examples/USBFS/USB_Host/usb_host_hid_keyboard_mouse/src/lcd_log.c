/*!
    \file    lcd_log.c
    \brief   this file provides all the LCD Log firmware functions

    \version 2026-01-01, V2.6.0, firmware for GD32F3x0
*/

/*
    Copyright (c) 2026, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "lcd_log.h"
#include "usb_lcd_conf.h"
#include <stdio.h>

LCD_LOG_line LCD_Cache_Buffer[LCD_CACHE_DEPTH];
uint32_t LCD_LineColor;
uint16_t LCD_Cache_Buffer_xptr;
uint16_t LCD_Cache_Buffer_yptr_top;
uint16_t LCD_Cache_Buffer_yptr_bottom;

uint16_t LCD_Cache_Buffer_yptr_top_bak;
uint16_t LCD_Cache_Buffer_yptr_bottom_bak;

ControlStatus LCD_Cache_Buffer_yptr_invert;
ControlStatus LCD_Scroll_Active;
ControlStatus LCD_Lock;
ControlStatus LCD_Scrolled;
uint16_t LCD_Scroll_BackStep;

/*!
    \brief      initialize the LCD Log module
    \param[in]  none
    \param[out] none
    \retval     none
*/
void lcd_log_init(void)
{
    lcd_log_deinit();

    lcd_clear(LCD_COLOR_BLACK);
}

/*!
    \brief      de-initialize the LCD Log module
    \param[in]  none
    \param[out] none
    \retval     none
*/
void lcd_log_deinit(void)
{
    LCD_LineColor = LCD_LOG_TEXT_COLOR;
    LCD_Cache_Buffer_xptr = 0U;
    LCD_Cache_Buffer_yptr_top = 0U;
    LCD_Cache_Buffer_yptr_bottom = 0U;

    LCD_Cache_Buffer_yptr_top_bak = 0U;
    LCD_Cache_Buffer_yptr_bottom_bak = 0U;

    LCD_Cache_Buffer_yptr_invert = ENABLE;
    LCD_Scroll_Active = DISABLE;
    LCD_Lock = DISABLE;
    LCD_Scrolled = DISABLE;
    LCD_Scroll_BackStep = 0U;
}

/*!
    \brief      display the application header (title) on the LCD screen
    \param[in]  ptitle: pointer to the string to be displayed
    \param[in]  start_x: x location
    \param[out] none
    \retval     none
*/
void lcd_log_header_set(uint8_t *ptitle, uint16_t start_x)
{
    lcd_font_set(&font8x16);

    lcd_text_color_set(LCD_COLOR_BLUE);

    lcd_rectangle_fill(LCD_HEADER_X, LCD_HEADER_Y, LCD_FLAG_WIDTH, LCD_FLAG_HEIGHT);

    lcd_background_color_set(LCD_COLOR_BLUE);
    lcd_text_color_set(LCD_COLOR_RED);

    lcd_vertical_string_display(LCD_HEADER_LINE, start_x, ptitle);

    lcd_background_color_set(LCD_COLOR_BLACK);
}

/*!
    \brief      display the application footer (status) on the LCD screen
    \param[in]  pstatus: pointer to the string to be displayed
    \param[in]  start_x: x location
    \param[out] none
    \retval     none
*/
void lcd_log_footer_set(uint8_t *pstatus, uint16_t start_x)
{
    lcd_text_color_set(LCD_COLOR_BLUE);

    lcd_rectangle_fill(LCD_FOOTER_X, LCD_FOOTER_Y, LCD_FLAG_WIDTH, LCD_FLAG_HEIGHT);

    lcd_background_color_set(LCD_COLOR_BLUE);
    lcd_text_color_set(LCD_COLOR_RED);

    lcd_vertical_string_display(LCD_FOOTER_LINE, start_x, pstatus);

    lcd_background_color_set(LCD_COLOR_BLACK);
}

/*!
    \brief      clear the text zone
    \param[in]  start_x, start_y, width, height: zone location
    \param[out] none
    \retval     none
*/
void lcd_log_textzone_clear(uint16_t start_x,
                            uint16_t start_y,
                            uint16_t width,
                            uint16_t height)
{
    lcd_rectangle_fill(start_x, start_y, width, height);
}

#ifdef USE_LCD
/*!
    \brief      redirect the printf to the LCD
    \param[in]  ch: character to be displayed
    \param[in]  f: output file pointer
    \param[out] none
    \retval     none
*/
LCD_LOG_PUTCHAR {
    font_struct *cFont = lcd_font_get();
    uint32_t idx;

    if(DISABLE == LCD_Lock)
    {
        if(ENABLE == LCD_Scroll_Active) {
            LCD_Cache_Buffer_yptr_bottom = LCD_Cache_Buffer_yptr_bottom_bak;
            LCD_Cache_Buffer_yptr_top = LCD_Cache_Buffer_yptr_top_bak;
            LCD_Scroll_Active = DISABLE;
            LCD_Scrolled = DISABLE;
            LCD_Scroll_BackStep = 0;
        }

        if((LCD_Cache_Buffer_xptr < LCD_FLAG_HEIGHT / cFont->width) && ('\n' != ch)) {
            LCD_Cache_Buffer[LCD_Cache_Buffer_yptr_bottom].line[LCD_Cache_Buffer_xptr++] = (uint16_t)ch;
        } else {
            if(LCD_Cache_Buffer_yptr_top >= LCD_Cache_Buffer_yptr_bottom) {
                if(DISABLE == LCD_Cache_Buffer_yptr_invert) {
                    LCD_Cache_Buffer_yptr_top++;

                    if(LCD_CACHE_DEPTH == LCD_Cache_Buffer_yptr_top) {
                        LCD_Cache_Buffer_yptr_top = 0U;
                    }
                } else {
                    LCD_Cache_Buffer_yptr_invert = DISABLE;
                }
            }

            for(idx = LCD_Cache_Buffer_xptr; idx < LCD_FLAG_HEIGHT / cFont->width; idx++) {
                LCD_Cache_Buffer[LCD_Cache_Buffer_yptr_bottom].line[LCD_Cache_Buffer_xptr++] = ' ';
            }

            LCD_Cache_Buffer[LCD_Cache_Buffer_yptr_bottom].color = LCD_LineColor;
            LCD_Cache_Buffer_xptr = 0U;
            LCD_LOG_UpdateDisplay();
            LCD_Cache_Buffer_yptr_bottom ++;

            if(LCD_CACHE_DEPTH == LCD_Cache_Buffer_yptr_bottom) {
                LCD_Cache_Buffer_yptr_bottom = 0U;
                LCD_Cache_Buffer_yptr_top = 1U;
                LCD_Cache_Buffer_yptr_invert = ENABLE;
            }

            if('\n' != ch) {
                LCD_Cache_Buffer[LCD_Cache_Buffer_yptr_bottom].line[LCD_Cache_Buffer_xptr++] = (uint16_t)ch;
            }
        }
    }

    return ch;
}
#endif /* USE_LCD */

/*!
    \brief      update the text area display
    \param[in]  none
    \param[out] none
    \retval     none
*/
void LCD_LOG_UpdateDisplay(void)
{
    uint8_t cnt = 0U ;
    uint16_t length = 0U ;
    uint16_t ptr = 0U, index = 0U;

    font_struct *cFont = lcd_font_get();

    if((LCD_Cache_Buffer_yptr_bottom < (YWINDOW_SIZE - 1U)) && (LCD_Cache_Buffer_yptr_bottom >= LCD_Cache_Buffer_yptr_top)) {
        lcd_text_color_set(LCD_Cache_Buffer[cnt + LCD_Cache_Buffer_yptr_bottom].color);
        lcd_vertical_string_display((YWINDOW_MIN + LCD_Cache_Buffer_yptr_bottom) * cFont->height, 0U,
                                    (uint8_t *)(LCD_Cache_Buffer[cnt + LCD_Cache_Buffer_yptr_bottom].line));
    } else {
        if(LCD_Cache_Buffer_yptr_bottom < LCD_Cache_Buffer_yptr_top) {
            /* Virtual length for rolling */
            length = LCD_CACHE_DEPTH + LCD_Cache_Buffer_yptr_bottom ;
        } else {
            length = LCD_Cache_Buffer_yptr_bottom;
        }

        ptr = length - YWINDOW_SIZE + 1U;

        for(cnt = 0U; cnt < YWINDOW_SIZE; cnt ++) {
            index = (cnt + ptr) % LCD_CACHE_DEPTH;

            lcd_text_color_set(LCD_Cache_Buffer[index].color);
            lcd_vertical_string_display((cnt + YWINDOW_MIN) * cFont->height, 0U, (uint8_t *)(LCD_Cache_Buffer[index].line));
        }
    }
}
