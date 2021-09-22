/****************************************************************************
 * boards/xtensa/esp32s2/esp32s2-saola-1/src/esp32s2-saola-1.h
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

#ifndef __BOARDS_XTENSA_ESP32S2_ESP32S2_SAOLA_1_SRC_ESP32S2_SAOLA_1_H
#define __BOARDS_XTENSA_ESP32S2_ESP32S2_SAOLA_1_SRC_ESP32S2_SAOLA_1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESP32S2-DevKitC GPIOs ****************************************************/

/* BOOT Button */

#define BUTTON_BOOT  0

/* TIMERS */

#define TIMER0 0
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3

/* ONESHOT */

#define ONESHOT_TIMER         TIMER0
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
 * Name: esp32s2_bringup
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

int esp32s2_bringup(void);

/****************************************************************************
 * Name: esp32s2_gpio_init
 *
 * Description:
 *   Configure the GPIO driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int esp32s2_gpio_init(void);
#endif

/****************************************************************************
 * Name: board_oneshot_init
 *
 * Description:
 *   Configure the oneshot timer driver.
 *
 * Input Parameters:
 *   timer      - Timer instance to be used as oneshot timer.
 *   resolution - Oneshot timer resolution.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ONESHOT
int board_oneshot_init(int timer, uint16_t resolution);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32S2_ESP32S2_SAOLA_1_SRC_ESP32S2_SAOLA_1_H */
