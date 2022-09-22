/****************************************************************************
 * boards/xtensa/esp32/esp32-lyrat/src/esp32-lyrat.h
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

#ifndef __BOARDS_XTENSA_ESP32_ESP32_LYRAT_SRC_ESP32_LYRAT_H
#define __BOARDS_XTENSA_ESP32_ESP32_LYRAT_SRC_ESP32_LYRAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESP32-LyraT GPIOs ********************************************************/

/* LEDs */

#define GPIO_LED1  22

/* Buttons */

/* As BOOT_BUTTON shares pins with I2S and the SD card it cannot be used
 * as an user button like regular ESP32 boards. As the ESP32-LyraT has
 * other buttons that can be used, BOOT_BUTTON is disabled in the buttons
 * driver to avoid conflict with the remaining peripherals.
 */

#define BUTTON_REC      36
#define BUTTON_MODE     39

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
 *
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

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32_ESP32_LYRAT_SRC_ESP32_LYRAT_H */
