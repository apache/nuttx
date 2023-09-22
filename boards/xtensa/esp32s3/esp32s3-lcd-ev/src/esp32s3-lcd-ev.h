/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-lcd-ev/src/esp32s3-lcd-ev.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_ESP32S3_LCD_EV_SRC_ESP32S3_LCD_EV_H
#define __BOARDS_XTENSA_ESP32S3_ESP32S3_LCD_EV_SRC_ESP32S3_LCD_EV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESP32-S3-LCD-EV GPIOs ****************************************************/

/* BOOT Button */

#define BUTTON_BOOT         0

/* I2C Port */

#define I2C_PORT            0

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library via board_app_initialize()
 *
 ****************************************************************************/

int esp32s3_bringup(void);

/****************************************************************************
 * Name: board_spiflash_init
 *
 * Description:
 *   Initialize the SPIFLASH and register the MTD device.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_SPIFLASH
int board_spiflash_init(void);
#endif

/****************************************************************************
 * Name:  board_ws2812_initialize
 *
 * Description:
 *   This function may called from application-specific logic during its
 *   to perform board-specific initialization of the ws2812 device
 *
 ****************************************************************************/

#if defined(CONFIG_WS2812) && !defined(CONFIG_WS2812_NON_SPI_DRIVER)
int board_ws2812_initialize(int devno, int spino, uint16_t nleds);
#endif

/****************************************************************************
 * Name: board_lcd_initialize
 *
 * Description:
 *   Initialize LCD.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ESP32S3_LCD) && defined(CONFIG_ESP32S3_BOARD_IOEXPANDER)
int board_lcd_initialize(void);
#endif

/****************************************************************************
 * Name: board_touchscreen_initialize
 *
 * Description:
 *   Initialize touchpad.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_BOARD_TOUCHPAD
int board_touchscreen_initialize(void);
#endif

/****************************************************************************
 * Name: board_ioexpander_set_pin
 *
 * Description:
 *   Configure pin mode through the IO expander.
 *
 * Input Parameters:
 *   input_mask  - pin bit mask which need to be set input, if set pin 0 to
 *                 to be input, please make: input_mask = (1 << 0)
 *   output_mask - pin bit mask which need to be set output, if set pin 1 to
 *                 to be input, please make: output_mask = (1 << 1)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_BOARD_IOEXPANDER
int board_ioexpander_set_pin(uint8_t input_mask, uint8_t output_mask);
#endif

/****************************************************************************
 * Name: board_ioexpander_output
 *
 * Description:
 *   Set pin output level through the IO expander.
 *
 * Input Parameters:
 *   pin   - pin number
 *   level - true for high level, false for low.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_BOARD_IOEXPANDER
int board_ioexpander_output(int pin, bool level);
#endif

/****************************************************************************
 * Name: board_ioexpander_initialize
 *
 * Description:
 *   Initialize IO expander driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_BOARD_IOEXPANDER
int board_ioexpander_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32S3_ESP32S3_LCD_EV_SRC_ESP32S3_LCD_EV_H */
