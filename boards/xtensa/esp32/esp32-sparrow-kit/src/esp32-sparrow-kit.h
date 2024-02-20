/****************************************************************************
 * boards/xtensa/esp32/esp32-sparrow-kit/src/esp32-sparrow-kit.h
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

#ifndef __BOARDS_XTENSA_ESP32_ESP32_SPARROWKIT_SRC_ESP32_SPARROWKIT_H
#define __BOARDS_XTENSA_ESP32_ESP32_SPARROWKIT_SRC_ESP32_SPARROWKIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESP-SPARROW-KIT GPIOs ****************************************************/

/* LEDs */

#define GPIO_LED1  0
#define GPIO_LED2  2
#define GPIO_LED3  4

/* BOOT Button */

#define BUTTON_BOOT  0

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
 * Name: board_i2sdev_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the generic I2S audio driver.  This function will register
 *   the driver as /dev/audio/pcm[x] where x is determined by the I2S port
 *   number.
 *
 * Input Parameters:
 *   port  - The I2S port used for the device
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#if defined CONFIG_ESP32_I2S0 || defined CONFIG_ESP32_I2S1
int board_i2sdev_initialize(int port);
#endif

/****************************************************************************
 * Name: esp32_gpio_init
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int esp32_gpio_init(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32_ESP32_SPARROWKIT_SRC_ESP32_SPARROWKIT_H */
