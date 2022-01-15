/****************************************************************************
 * include/nuttx/lcd/st7032.h
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

#ifndef __INCLUDE_NUTTX_LCD_ST7032_H
#define __INCLUDE_NUTTX_LCD_ST7032_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/* Configuration
 * CONFIG_I2C - Enables support for SPI drivers
 * CONFIG_LCD_ST7032 - Enables support for the ST7032 driver
 */

#if defined(CONFIG_I2C) && defined(CONFIG_LCD_ST7032)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ST7032 Address */

#define ST7032_I2C_ADDR               0x3e

/* Default contrast */

#define DEFAULT_CONTRAST              0x18

/* Default row and col */

#define ST7032_MAX_ROW                2     /* Default for JLX1602G-390 display */
#define ST7032_MAX_COL                16    /* Default for JLX1602G-390 display */

/* ST7032 register addresses */

#define ST7032_CTRLBIT_CO             0x80 /* Control Command bit */
#define ST7032_CTRLBIT_RS             0x40 /* Select Register: Instruction/Data Register */
#define ST7032_CLEAR_DISPLAY          0x01 /* Clear display */
#define ST7032_RETURN_HOME            0x02 /* Set DDRAM to 00H and return cursor to its original position */
#define ST7032_ENTRY_MODE_SET         0x04 /* Set cursor move direction and display shift */
#define ST7032_DISPLAY_ON_OFF         0x08 /* Display ON/OFF */
#define ST7032_CUR_DISP_SHIFT         0x10 /* Set cursor/display direction/shift without changing DDRAM */
#define ST7032_FUNCTION_SET           0x20 /* DL=interf. 8/4-bit; N=number lines 2/1; DH=Double Font Height IS= Inst. Table Select */
#define ST7032_SET_CGRAM_ADDR         0x40 /* Set CGRAM addr in addr counter */
#define ST7032_SET_DDRAM_ADDR         0x80 /* Set DDRAM addr in addr counter */

#define ST7032_INT_OSC_FREQ           0x10 /* Adjust internal oscillator frequency */
#define ST7032_SET_ICON_ADDR          0x40 /* Set ICON addr in addr counter */
#define ST7032_POWER_ICON_CTRL_SET    0x50 /* Icon, booster ON/OFF and contrast */
#define ST7032_FOLLOWER_CTRL          0x60 /* Set follower ON/OFF and amplified ratio */
#define ST7032_CONTRAST_SET           0x70 /* Contrast set for internal follower mode */
#define ST7032_LINE1_ADDR             0x80 /* First line address */
#define ST7032_LINE2_ADR              0xc0 /* Second line address */

/* Bits and flags definitions */

#define ENTRY_MODE_SET_S              0x01 /* Shift of entire display */
#define ENTRY_MODE_SET_ID             0x02 /* I/D : Increment / decrement of DDRAM address (cursor or blink) */
#define DISPLAY_ON_OFF_B              0x01 /* B : Cursor Blink ON/OFF control bit */
#define DISPLAY_ON_OFF_C              0x02 /* C : Cursor ON/OFF control bit */
#define DISPLAY_ON_OFF_D              0x04 /* D : Display ON/OFF control bit */
#define FUNCTION_SET_IS               0x01 /* IS : normal/extension instruction select */
#define FUNCTION_SET_DH               0x04 /* DH : Double height font type control bit*/
#define FUNCTION_SET_N                0x08 /* N : Display line number control bit */
#define FUNCTION_SET_DL               0x10 /* DL : Interface data length control bit */
#define CUR_DISP_SHIFT_RL             0x04 /* R/L: Right/Left */
#define CUR_DISP_SHIFT_SC             0x08 /* S/C: Screen/Cursor select bit */
#define INT_OSC_FREQ_F0               0x01 /* F2,F1,F0 : Internal OSC frequency adjust */
#define INT_OSC_FREQ_F1               0x02 /* F2,F1,F0 : Internal OSC frequency adjust */
#define INT_OSC_FREQ_F2               0x04 /* F2,F1,F0 : Internal OSC frequency adjust */
#define INT_OSC_FREQ_BS               0x08 /* BS: bias selection */
#define POWER_ICON_BOST_CTRL_BON      0x04 /* Bon: switch booster circuit */
#define POWER_ICON_BOST_CTRL_ION      0x08 /* Ion: set ICON display on/off */
#define FOLLOWER_CTRL_RAB0            0x01 /* Rab2,Rab1,Rab0 : V0 generator amplified ratio */
#define FOLLOWER_CTRL_RAB1            0x02 /* Rab2,Rab1,Rab0 : V0 generator amplified ratio */
#define FOLLOWER_CTRL_RAB2            0x04 /* Rab2,Rab1,Rab0 : V0 generator amplified ratio */
#define FOLLOWER_CTRL_FON             0x08 /* Fon: switch follower circuit */

#define ST7032_CONTRAST_MAX           0x3f /* limit range max value (0x00 - 0x3F) */
#define ST7032_CONTRAST_MIN           0x00 /* limit range min value (0x00 - 0x3F) */
#define ST7032_WRITE_DELAY_MS         30   /* see data sheet */
#define ST7032_HOME_CLEAR_DELAY_MS    1200 /* see data sheet */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: st7032_leds_register
 *
 * Description:
 *   Initialize the ST7032 device as a LEDS interface.
 *
 * Input Parameters:
 *   spi   - An instance of the SPI interface to use to communicate
 *           with the ST7032.
 *   devno - Device number to identify current display.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct i2c_master_s;  /* Forward reference */

int st7032_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_ST7032 */
#endif /* __INCLUDE_NUTTX_LCD_ST7032_H */
