/****************************************************************************
 * boards/xtensa/esp32/ttgo_lora_esp32/src/ttgo_lora_esp32.h
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

#ifndef __BOARDS_XTENSA_ESP32_TTGO_LORA_ESP32_SRC_TTGO_LORA_ESP32_H
#define __BOARDS_XTENSA_ESP32_TTGO_LORA_ESP32_SRC_TTGO_LORA_ESP32_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TTGO-LoRa-SX1276-ESP32 GPIOs *********************************************/

/* OLED SSD1306 */

#define HAVE_SSD1306 1

#if !defined(CONFIG_ESP32_I2C) || !defined(CONFIG_ESP32_I2C0) || \
    !defined(CONFIG_LCD_SSD1306_I2C)
#  undef HAVE_SSD1306
#endif

#define GPIO_SSD1306_RST 16

/* BOOT Button */

#define BUTTON_BOOT  0

/* LED
 *
 * This is an externally connected LED used for testing.
 */

#define GPIO_LED1             2

/* MCP2515 Interrupt pin */

#define GPIO_MCP2515_IRQ      22

/* TIMERS */

#define TIMER0 0
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3

/* ONESHOT */

#define ONESHOT_TIMER         1
#define ONESHOT_RESOLUTION_US 1

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
 * Name: esp32_bringup
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

int esp32_bringup(void);

/****************************************************************************
 * Name: esp32_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 ****************************************************************************/

int esp32_mmcsd_initialize(int minor);

/****************************************************************************
 * Name: esp32_spiflash_init
 *
 * Description:
 *   Initialize the SPIFLASH and register the MTD device.
 ****************************************************************************/

int esp32_spiflash_init(void);

/****************************************************************************
 * Name: esp32_gpio_init
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int esp32_gpio_init(void);
#endif

/****************************************************************************
 * Name: board_spidev_initialize
 *
 * Description:
 *   Initialize SPI driver and register the /dev/spi device.
 *
 * Input Parameters:
 *   bus - The SPI bus number, used to build the device path as /dev/spiN
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_DRIVER
int board_spidev_initialize(int bus);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32_TTGO_LORA_ESP32_SRC_TTGO_LORA_ESP32_H */
