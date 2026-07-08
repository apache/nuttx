/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-tab5/include/board.h
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

#ifndef __BOARDS_RISCV_ESP32P4_ESP32P4_TAB5_INCLUDE_BOARD_H
#define __BOARDS_RISCV_ESP32P4_ESP32P4_TAB5_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* M5Stack Tab5 (ESP32-P4 + ESP32-C6) full pin map
 *
 * Only the I2C bus and the BOOT button are wired up by the current board
 * logic.  All remaining pins are documented here (and in the board's
 * Documentation/.rst) so that future drivers can be added without having to
 * re-derive the wiring.  See the board documentation for the I2C device
 * address map and the peripheral status (supported vs. pins-only).
 */

/* Console / debug UART (PC_TX/PC_RX header) */

#define TAB5_GPIO_PC_TX        6   /* UART debug TX */
#define TAB5_GPIO_PC_RX        7   /* UART debug RX */

/* I2C0 - shared system bus (touch, audio, IMU, RTC, power-mon, IO exp) */

#define TAB5_GPIO_I2C0_SDA     31
#define TAB5_GPIO_I2C0_SCL     32

/* Grove HY2.0-4P port (secondary I2C / GPIO) */

#define TAB5_GPIO_GROVE_SDA    53
#define TAB5_GPIO_GROVE_SCL    54

/* BOOT button (active low) */

#define BUTTON_BOOT            35

/* Display - MIPI-DSI ILI9881C (NOT implemented; pins documented only)
 *
 * The DSI data/clock lanes are dedicated MIPI pins (not GPIO-muxed) powered
 * by VDD_MIPI_DPHY; only the backlight enable is a GPIO.  Panel reset is the
 * board power-on reset (BSP_LCD_RST = NC) driven by IO expander 0x43 P4.
 */

#define TAB5_GPIO_LCD_BL_EN    22  /* Backlight enable -> ME2212 boost EN */

/* Touch - GT911 (NOT implemented; pins documented only) */

#define TAB5_GPIO_TP_INT       23  /* GT911 touch interrupt */

/* Audio - ES8388 codec (out) + ES7210 mic array (in), I2S (pins only) */

#define TAB5_GPIO_I2S_DSDIN    26  /* I2S data to ES8388 */
#define TAB5_GPIO_I2S_SCLK     27  /* I2S bit clock */
#define TAB5_GPIO_I2S_LRCK     29  /* I2S word/LR clock */
#define TAB5_GPIO_I2S_MCLK     30  /* I2S / camera master clock */

/* Camera - SC2356 MIPI-CSI (pins only) */

#define TAB5_GPIO_CAM_MCLK     36  /* Camera master clock */

/* RS485 - SIT3088 half-duplex (pins only) */

#define TAB5_GPIO_RS485_TX     20
#define TAB5_GPIO_RS485_RX     21
#define TAB5_GPIO_RS485_DE     34  /* Direction (DE/RE) */

/* microSD - SDMMC 4-bit per M5Stack BSP (pins only; no SDMMC driver yet)
 *
 * SPI-mode mapping for the same slot would be:
 *   SCK=CLK(43), MOSI=CMD(44), MISO=DAT0(39), CS=DAT3(42).
 */

#define TAB5_GPIO_SD_D0        39  /* DAT0 (also SPI MISO) */
#define TAB5_GPIO_SD_D1        40  /* DAT1 (also SPI CS in NAND mode) */
#define TAB5_GPIO_SD_D2        41  /* DAT2 (also SPI SCK) */
#define TAB5_GPIO_SD_D3        42  /* DAT3 (also SPI MOSI) */
#define TAB5_GPIO_SD_CLK       43  /* SD clock */
#define TAB5_GPIO_SD_CMD       44  /* SD command */

/* M5-Bus expansion (pins only) */

#define TAB5_GPIO_M5_SCK       5
#define TAB5_GPIO_M5_PB_IN     16
#define TAB5_GPIO_M5_MOSI      18
#define TAB5_GPIO_M5_MISO      19
#define TAB5_GPIO_M5_TXD0      37
#define TAB5_GPIO_M5_RXD0      38
#define TAB5_GPIO_M5_PB_OUT    52

/* ESP32-C6 (Wi-Fi/BT companion) - SDIO2 bus GPIO8..15 (pins only)
 *
 * GPIO8-15 carry D3..D0, IO2, RST, CK, CMD of the SDIO2 link to the C6.
 */

#endif /* __BOARDS_RISCV_ESP32P4_ESP32P4_TAB5_INCLUDE_BOARD_H */
