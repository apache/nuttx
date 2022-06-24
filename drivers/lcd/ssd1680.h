/****************************************************************************
 * drivers/lcd/ssd1680.h
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

#ifndef __DRIVERS_LCD_SSD1680_H
#define __DRIVERS_LCD_SSD1680_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1680.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Limitations of the current configuration that I hope to fix someday */

#if !defined(CONFIG_LCD_SSD1680_2_13_V2) && !defined(CONFIG_LCD_SSD1680_2_90)
#  error "Unknown and unsupported SSD16800 LCD"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NX_BGCOLOR
#  define CONFIG_NX_BGCOLOR SSD1680_Y1_BLACK
#endif

/* SSD1680 Commands *********************************************************/

#define SSD1680_DRIVER_CONTROL 0x01
#define SSD1680_GATE_VOLTAGE 0x03
#define SSD1680_SOURCE_VOLTAGE 0x04
#define SSD1680_PROGOTP_INITIAL 0x08
#define SSD1680_PROGREG_INITIAL 0x09
#define SSD1680_READREG_INITIAL 0x0A
#define SSD1680_BOOST_SOFTSTART 0x0C
#define SSD1680_GATE_SCAN_POSITION 0x0F
#define SSD1680_DEEP_SLEEP 0x10
#define SSD1680_DATA_MODE 0x11
#define SSD1680_SW_RESET 0x12
#define SSD1680_TEMP_CONTROL 0x18
#define SSD1680_TEMP_WRITE 0x1A

/* Execute sequence written in Control Register 2
 * Wait till SSD1680 is not busy
 */

#define SSD1680_MASTER_ACTIVATE 0x20

#define SSD1680_DISP_CTRL1 0x21

/* Set control register 2.
 * 1 byte of data with following bits:
 *   0x80 - enable clock signal
 *   0x40 - enable analog
 *   0x20 - load temperature value (endless busy state. Don't use)
 *   0x10 - Load LUT (endless busy state. Probably OTP wasn't prepared)
 *   0x08 - Display Mode 2
 *   0x04 - disable OSC
 *   0x01 - disable clock signal
 *   0x02 - disable analog
 */

#define SSD1680_DISP_CTRL2 0x22

#define SSD1680_WRITE_RAM1 0x24
#define SSD1680_WRITE_RAM2 0x26
#define SSD1680_WRITE_VCOM 0x2C
#define SSD1680_READ_OTP 0x2D
#define SSD1680_READ_STATUS 0x2F
#define SSD1680_WRITE_LUT 0x32
#define SSD1680_DUMMY_LINE 0x3A
#define SSD1680_GATE_TIME 0x3B
#define SSD1680_WRITE_BORDER 0x3C
#define SSD1680_SET_RAMXPOS 0x44
#define SSD1680_SET_RAMYPOS 0x45
#define SSD1680_SET_RAMXCOUNT 0x4E
#define SSD1680_SET_RAMYCOUNT 0x4F
#define SSD1680_SET_ANALOG_BLOCK_CTRL 0x74
#define SSD1680_SET_DIGITAL_BLOCK_CTRL 0x7E
#define SSD1680_NOP 0x7F

/* Register Values **********************************************************/

#  define SSD1680_VALUE_G_VOLTAGE     0x15
#  define SSD1680_VALUE_S_VOLTAGE_1   0x41
#  define SSD1680_VALUE_S_VOLTAGE_2   0xA8
#  define SSD1680_VALUE_S_VOLTAGE_3   0x32

#if defined(CONFIG_LCD_SSD1680_2_13_V2) || defined(CONFIG_LCD_SSD1680_2_13_RED)
#  define SSD1680_VALUE_VCOM          0x55
#  define SSD1680_VALUE_DUMMY_LINE    0x30
#  define SSD1680_VALUE_GATE_TIME     0x0A
#elif defined(CONFIG_LCD_SSD1680_2_90) || defined(CONFIG_LCD_SSD1680_2_90_RED)
#  define SSD1680_VALUE_VCOM          0xA8
#  define SSD1680_VALUE_DUMMY_LINE    0x1A
#  define SSD1680_VALUE_GATE_TIME     0x08
#else
#  error "Not supported display"
#endif

/* Color Properties *********************************************************/
#if defined(CONFIG_LCD_SSD1680_2_13_V2) || defined(CONFIG_LCD_SSD1680_2_90)
/* 1 bit per pixel */
#  define SSD1680_DEV_COLORFMT  FB_FMT_Y1
#  define SSD1680_DEV_BPP       1
#  define SSD1680_NO_OF_PLANES  1
#elif defined (CONFIG_LCD_SSD1680_2_13_RED) || (CONFIG_LCD_SSD1680_2_90_RED)
/* 2 bits per pixel, it is not agrayscale, but indexed colours */
#  define SSD1680_DEV_COLORFMT  FB_FMT_Y2
#  define SSD1680_DEV_BPP       2
#  define SSD1680_NO_OF_PLANES  1
#else
#  error "Can't prepare macros for not supported display"
#endif

#if SSD1680_DEV_BPP == 1
#  define SSD1680_PDF           3
#  define SSD1680_PDV           8
#elif SSD1680_DEV_BPP == 2
#  define SSD1680_PDF           2
#  define SSD1680_PDV           4
#else
#  error "SSD1680: Bits per pixel not defined"
#endif

/* Display Resolution
 *
 * The SSD1680 display controller can handle a resolution of 176/296.
 * The portrait mode is default.
 * Display size vs its resolution:
 *   2.13 : 122 / 250
 *   2.90 : 128 / 296
 */

#if defined(CONFIG_LCD_SSD1680_2_13_V2) || defined(CONFIG_LCD_SSD1680_2_13_RED)
#  define SSD1680_DEV_GATE_LAYOUT 0x00
#  define SSD1680_DEV_NATIVE_XRES 122
#  define SSD1680_DEV_NATIVE_YRES 250
#elif defined(CONFIG_LCD_SSD1680_2_90) || defined(CONFIG_LCD_SSD1680_2_90_RED)
#  define SSD1680_DEV_GATE_LAYOUT 0x00
#  define SSD1680_DEV_NATIVE_XRES 128
#  define SSD1680_DEV_NATIVE_YRES 296
#else
#  error "Unknown resulution"
#endif

/* SSD1680 memory write algorithm */
#if defined(CONFIG_LCD_PORTRAIT)
#  define SSD1680_VAL_DATA_MODE 0x03
#elif defined(CONFIG_LCD_RPORTRAIT)
#  define SSD1680_VAL_DATA_MODE 0x02
#elif defined(CONFIG_LCD_LANDSCAPE)
#  define SSD1680_VAL_DATA_MODE 0x07
#elif defined(CONFIG_LCD_RLANDSCAPE)
#  define SSD1680_VAL_DATA_MODE 0x07
#else
#  define SSD1680_VAL_DATA_MODE 0x03
#endif

/* Rotating screen if the orientation is changed */
#if defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT)

/* Calculate number of bytes per row.
 * In case of tri-color display there are two 1-bit data arrays
 * In order to simplify communication with controller, we need to provide
 * a number that is multiplication of 8 (not 4 even if we have 2 bpp)
 */

#  define SSD1680_DEV_ROWSIZE \
  ((SSD1680_DEV_NATIVE_XRES + SSD1680_PDV - 1) >> SSD1680_PDF)
#  define SSD1680_DEV_FB_XRES SSD1680_DEV_NATIVE_XRES
#  define SSD1680_DEV_FB_YRES SSD1680_DEV_NATIVE_YRES
#else /* Landstape */
#  define SSD1680_DEV_ROWSIZE \
  ((SSD1680_DEV_NATIVE_YRES + SSD1680_PDV - 1) >> SSD1680_PDF)
#  define SSD1680_DEV_FB_XRES SSD1680_DEV_NATIVE_YRES
#  define SSD1680_DEV_FB_YRES SSD1680_DEV_NATIVE_XRES
#endif /* Landstape */

#define SSD1680_DEV_X_ROUND_UP \
(((SSD1680_DEV_NATIVE_XRES + SSD1680_PDV - 1) >> SSD1680_PDF) << SSD1680_PDF)
#define SSD1680_DEV_Y_ROUND_UP \
(((SSD1680_DEV_NATIVE_YRES + SSD1680_PDV - 1) >> SSD1680_PDF) << SSD1680_PDF)

#define SSD1680_DEV_FBSIZE ((SSD1680_DEV_ROWSIZE) * (SSD1680_DEV_FB_YRES))

/****************************************************************************
 * Public Type Definition
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __DRIVERS_LCD_SSD1680_H */
