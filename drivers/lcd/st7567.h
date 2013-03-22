/**************************************************************************************
 * drivers/lcd/st7567.h
 *
 * Definitions for the ST7567 128x64 Dot Matrix LCD
 * Driver with C
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: Manikandan <code@zilogic.com>
 *
 * Based on drivers/lcd/ssd1305.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References: SSD1305.pdf, "Solomon Systech SSD1305 136x64 Dot Matrix OLED/PLED
 *             Segment/Common Driver with Controller," Solomon Systech Limited,
 *             http://www.solomon-systech.com, May, 2008.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************************/

#ifndef __DRIVERS_LCD_ST7567_H
#define __DRIVERS_LCD_ST7567_H

/**************************************************************************************
 * Included Files
 **************************************************************************************/

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* Fundamental Commands *****************************************************/

#define ST7567_DISPOFF             0xae /* 0xae: Display OFF (sleep mode) */
#define ST7567_DISPON              0xaf /* 0xaf: Display ON in normal mode */

#define ST7567_SETSTARTLINE        0x40 /* 0x40-7f: Set display start line */
#  define ST7567_STARTLINE_MASK    0x3f

#define ST7567_SETPAGESTART        0xb0 /* 0xb0-b7: Set page start address */
#  define ST7567_PAGESTART_MASK    0x07

#define ST7567_SETCOLL             0x00 /* 0x00-0x0f: Set lower column address */
#  define ST7567_COLL_MASK         0x0f
#define ST7567_SETCOLH             0x10 /* 0x10-0x1f: Set higher column address */
#  define ST7567_COLH_MASK         0x0f

#define SSD1305_MAPCOL0            0xa0 /* 0xa0: Column address 0 is mapped to SEG0 */
#define SSD1305_MAPCOL128          0xa1 /* 0xa1: Column address 128 is mapped to SEG0 */

#define ST7567_DISPNORMAL          0xa6 /* 0xa6: Normal display */
#define ST7567_DISPINVERSE         0xa7 /* 0xa7: Inverse display */

#define ST7567_DISPRAM             0xa4 /* 0xa4: Resume to RAM content display */
#define ST7567_DISPENTIRE          0xa5 /* 0xa5: Entire display ON */

#define ST7567_BIAS_1_9            0xa2 /* 0xa2: Select BIAS setting 1/9 */
#define ST7567_BIAS_1_7            0xa3 /* 0xa3: Select BIAS setting 1/7 */

#define ST7567_ENTER_RMWMODE       0xe0 /* 0xe0: Enter the Read Modify Write mode */
#define ST7567_EXIT_RMWMODE        0xee /* 0xee: Leave the Read Modify Write mode */
#define ST7567_EXIT_SOFTRST        0xe2 /* 0xe2: Software RESET */

#define ST7567_SETCOMNORMAL        0xc0 /* 0xc0: Set COM output direction, normal mode */
#define ST7567_SETCOMREVERSE       0xc8 /* 0xc8: Set COM output direction, reverse mode */

#define ST7567_POWERCTRL_VF        0x29 /* 0x29: Control built-in power circuit */
#define ST7567_POWERCTRL_VR        0x2a /* 0x2a: Control built-in power circuit */
#define ST7567_POWERCTRL_VB        0x2c /* 0x2c: Control built-in power circuit */
#define ST7567_POWERCTRL           0x2f /* 0x2c: Control built-in power circuit */

#define ST7567_REG_RES_RR0         0x21 /* 0x21: Regulation Resistior ratio */
#define ST7567_REG_RES_RR1         0x22 /* 0x22: Regulation Resistior ratio */
#define ST7567_REG_RES_RR2         0x24 /* 0x24: Regulation Resistior ratio */

#define ST7567_SETEV               0x81 /* 0x81: Set contrast control */

#define ST7567_SETBOOSTER          0xf8 /* Set booster level */
#define ST7567_SETBOOSTER4X        0x00 /* Set booster level */
#define ST7567_SETBOOSTER5X        0x01 /* Set booster level */

#define ST7567_NOP                 0xe3 /* 0xe3: NOP Command for no operation */

#endif /* __DRIVERS_LCD_ST7567_H */
