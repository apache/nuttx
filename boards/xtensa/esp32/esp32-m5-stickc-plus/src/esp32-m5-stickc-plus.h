/****************************************************************************
 * boards/xtensa/esp32/esp32-m5-stickc-plus/src/esp32-m5-stickc-plus.h
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

#ifndef __BOARDS_XTENSA_ESP32_ESP32_M5_STICKC_PLUS_SRC_ESP32_M5_STICKC_PLUS_H
#define __BOARDS_XTENSA_ESP32_ESP32_M5_STICKC_PLUS_SRC_ESP32_M5_STICKC_PLUS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* M5StickC Plus GPIOs ******************************************************/

/* Buttons
 *
 * Both are input-only pins (GPIO34-39 have no internal pull resistors);
 * the board provides external pull-ups and a press reads low.  The power
 * button is wired to the AXP192 PMIC, not to a GPIO.
 */

#define BUTTON_A              37  /* Button A */
#define BUTTON_B              39  /* Button B */

/* Red LED, active low */

#define GPIO_LED1             10

/* Passive buzzer, driven by LEDC PWM */

#define GPIO_BUZZER           2

/* Internal I2C bus (I2C0 on GPIO21 SDA / GPIO22 SCL):
 * AXP192 PMIC (0x34), BM8563 RTC (0x51), MPU6886 IMU (0x68)
 */

#define AXP192_I2C_PORT       0
#define BM8563_I2C_PORT       0
#define MPU6886_I2C_PORT      0
#define MPU6886_I2C_ADDR      0x68

/* The ST7789V2 LCD pins are defined as DISPLAY_* in include/board.h, where
 * the common ESP32 LCD glue code expects them.
 */

/* TIMERS */

#define TIMER0 0
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3

/* ONESHOT */

#define ONESHOT_TIMER         1
#define ONESHOT_RESOLUTION_US 1

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: esp32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 ****************************************************************************/

int esp32_bringup(void);

/****************************************************************************
 * Name: esp32_pwm_setup
 *
 * Description:
 *   Initialize LEDC PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_LEDC
int esp32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: board_axp192_initialize
 *
 * Description:
 *   Power up the LCD rails of the AXP192 PMIC.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_I2C0
int board_axp192_initialize(void);
#endif

/****************************************************************************
 * Name: board_bm8563_initialize
 *
 * Description:
 *   Bind the BM8563 RTC as the system RTC and sync the system time to it.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PCF8563
int board_bm8563_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32_ESP32_M5_STICKC_PLUS_SRC_ESP32_M5_STICKC_PLUS_H */
