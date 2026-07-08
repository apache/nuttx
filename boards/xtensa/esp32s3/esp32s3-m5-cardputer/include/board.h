/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-m5-cardputer/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_XTENSA_ESP32S3_ESP32S3_M5_CARDPUTER_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32S3_ESP32S3_M5_CARDPUTER_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The M5Stack Cardputer (StampS3 / ESP32-S3FN8) is fitted with a 40MHz
 * crystal.
 */

#define BOARD_XTAL_FREQUENCY    40000000

#ifdef CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY (CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY 80000000
#endif

/* M5Stack Cardputer full pin map
 *
 * The pin numbers below document the complete board wiring so that drivers
 * can be added without having to re-derive it from the M5Cardputer/M5Unified
 * libraries.  Peripherals that are not brought up by the current board logic
 * are marked "(pins only)".  See the board documentation for the per-config
 * peripheral status.
 */

/* USB-Serial-JTAG (NSH console over the USB-C port, native USB) */

#define CARDPUTER_GPIO_USB_DM   19  /* USB D- */
#define CARDPUTER_GPIO_USB_DP   20  /* USB D+ */

/* 56-key keyboard - 8x7 matrix scanned through a 74HC138 3-to-8 demux.
 *
 * The three select lines drive the demux address (one of eight rows is
 * pulled low at a time); the seven column lines are read back.
 */

#define CARDPUTER_GPIO_KB_A0    8   /* 74HC138 A0 (row select bit 0) */
#define CARDPUTER_GPIO_KB_A1    9   /* 74HC138 A1 (row select bit 1) */
#define CARDPUTER_GPIO_KB_A2    11  /* 74HC138 A2 (row select bit 2) */

#define CARDPUTER_GPIO_KB_C0    13  /* Column read 0 */
#define CARDPUTER_GPIO_KB_C1    15  /* Column read 1 */
#define CARDPUTER_GPIO_KB_C2    3   /* Column read 2 */
#define CARDPUTER_GPIO_KB_C3    4   /* Column read 3 */
#define CARDPUTER_GPIO_KB_C4    5   /* Column read 4 */
#define CARDPUTER_GPIO_KB_C5    6   /* Column read 5 */
#define CARDPUTER_GPIO_KB_C6    7   /* Column read 6 */

/* Display - ST7789v2 1.14" 240x135 TFT on SPI2 */

#define CARDPUTER_GPIO_LCD_SCLK 36  /* SPI2 clock */
#define CARDPUTER_GPIO_LCD_MOSI 35  /* SPI2 MOSI */
#define CARDPUTER_GPIO_LCD_CS   37  /* Chip select */
#define CARDPUTER_GPIO_LCD_DC   34  /* Data/command */
#define CARDPUTER_GPIO_LCD_RST  33  /* Reset */
#define CARDPUTER_GPIO_LCD_BL   38  /* Backlight enable */

/* microSD - SPI3 (CS=12, MOSI=14, SCK=40, MISO=39) */

#define CARDPUTER_GPIO_SD_SCLK  40  /* SPI3 clock */
#define CARDPUTER_GPIO_SD_MISO  39  /* SPI3 MISO */
#define CARDPUTER_GPIO_SD_MOSI  14  /* SPI3 MOSI */
#define CARDPUTER_GPIO_SD_CS    12  /* Chip select */

/* Audio out - NS4168 mono I2S Class-D amplifier (speaker) */

#define CARDPUTER_GPIO_SPK_BCLK 41  /* I2S bit clock */
#define CARDPUTER_GPIO_SPK_WS   43  /* I2S word/LR clock (shared, see note) */
#define CARDPUTER_GPIO_SPK_DOUT 42  /* I2S data to NS4168 */

/* Audio in - SPM1423 PDM microphone
 *
 * Note: the mic clock is the same pin as the speaker word-select (43); the
 * board shares it between the I2S TX and the PDM RX paths.
 */

#define CARDPUTER_GPIO_MIC_DATA 46  /* PDM data */
#define CARDPUTER_GPIO_MIC_CLK  43  /* PDM clock (shared with SPK_WS) */

/* IR transmitter (pins only) */

#define CARDPUTER_GPIO_IR_TX    44

/* On-board RGB LED - WS2812 addressable (needs the RMT driver; pins only) */

#define CARDPUTER_GPIO_RGB_LED  21

/* Grove HY2.0-4P port - I2C (also usable as GPIO) */

#define CARDPUTER_GPIO_I2C_SDA  2
#define CARDPUTER_GPIO_I2C_SCL  1

/* Battery voltage sense - ADC1 channel on GPIO10 (1:2 divider) */

#define CARDPUTER_GPIO_BAT_ADC  10

/* User / BOOT button (active low) */

#define CARDPUTER_GPIO_BTN      0

#endif /* __BOARDS_XTENSA_ESP32S3_ESP32S3_M5_CARDPUTER_INCLUDE_BOARD_H */
