/****************************************************************************
 * include/nuttx/lcd/hd4478ou.h
 *
 * Definitions for the Hitachi HD44780U LCD controller (as used in the
 * LCD1602).
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_LCD_HD4478OU_H
#define __INCLUDE_NUTTX_LCD_HD4478OU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Command set:
 *
 *   RS=0 R/W=0 : Command
 *   RS=0 R/W=1 : Busy/AD, Read CT (see below)
 *   RS=1 R/W=0 : Write data to CGRAM or DDRAM
 *   RS=1 R/W=0 : Read data from CGRAM or DDRAM
 */

#define HD4478OU_CLEAR            (0x01)     /* Screen Clear, Set AC to 0 */
#define HD4478OU_RETURN           (0x03)     /* DDRAM AD=0, return */
#define HD4478OU_INPUT            (0x04)     /* Set moving direction of cursor */
#  define HD4478OU_INPUT_SHIFT    (1 << 0)   /*   Shift */
#  define HD4478OU_INPUT_INCR     (1 << 1)   /*   Increment mode */
#  define HD4478OU_INPUT_DECR     (0x00)     /*   Decrement mode */
#define HD4478OU_DISPLAY          (0x08)     /* Set display, cursor, blink on/off */
#  define HD4478OU_DISPLAY_BLINK  (1 << 0)   /*   Blink on/off */
#  define HD4478OU_DISPLAY_CURSOR (1 << 1)   /*   Cursor on/off */
#  define HD4478OU_DISPLAY_ON     (1 << 2)   /*   Display on/off */
#define HD4478OU_SHIFT            (0x10)     /* Remove cursor and whole display */
#  define HD4478OU_SHIFT_RIGHT    (1 << 2)   /*   Shift right */
#  define HD4478OU_SHIFT_LEFT     (0x00)     /*   Shift right */
#  define HD4478OU_SHIFT_DISPLAY  (1 << 3)   /*   Display shift */
#  define HD4478OU_SHIFT_CURSOR   (0x00)     /*   Cursor shift */
#define HD4478OU_FUNC             (0x23)     /* Set DL, display line, font */
#  define HD4478OU_FUNC_F5X10     (1 << 2)   /*   5x10 Style */
#  define HD4478OU_FUNC_F5X7      (0x00)     /*   5x7 Style */
#  define HD4478OU_FUNC_N1        (1 << 3)   /*   N=2R */
#  define HD4478OU_FUNC_N0        (0x00)     /*   N=1R */
#  define HD4478OU_FUNC_DL8D      (1 << 4)   /*   DL=8D, 8-bit interface */
#  define HD4478OU_FUNC_DL4D      (0x00)     /*   DL=4D, 4-bit interface */
#define HD4478OU_CGRAM_AD(a)      (0x40|(a)) /* Set CGRAM AD, send receive data */
#define HD4478OU_DDRAM_AD(a)      (0x80|(a)) /* Set DDRAM AD, send receive data */

/* RS=0 R/W=1 : Execute internal function, read AD of CT */

#define HD4478OU_BF               (1 << 7)   /* Busy flag */
#define HD4478OU_AC_SHIFT         (0)        /* AD of CT */
#define HD4478OU_AC_MASK          (0x7f << HD4478OU_BUSY_AC_SHIFT)

/* DDRAM Addressing.
 *
 * Internally, the HD44780U supports a display size of up to 2x40 addressed
 * as follows:
 *
 * Column  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 ... 39
 * Row 0  00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f ... 27
 * Ro1 1  40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f ... 67
 */

#define HD4478OU_DDRAM_ROW0       0x00
#define HD4478OU_DDRAM_ROW1       0x40

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  up_lcd1602_initialize
 *
 * Description:
 *   The LCD1602 is an HD4478OU-based LCD from Wave share.
 *   This function initializes the LCD1602 hardware and registers the
 *   character driver as /dev/lcd1602.
 *
 *  NOTE:
 *  This common interface definition is provided, however, the underlying
 *  implemenataton is always board-specific for this LCD.
 *
 ****************************************************************************/

int up_lcd1602_initialize(void);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_HD4478OU_H */
