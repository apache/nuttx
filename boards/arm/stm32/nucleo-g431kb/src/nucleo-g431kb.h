/****************************************************************************
 * boards/arm/stm32/nucleo-g431kb/src/nucleo-g431kb.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32_NUCLEO_G431KB_SRC_NUCLEO_G431KB_H
#define __BOARDS_ARM_STM32_NUCLEO_G431KB_SRC_NUCLEO_G431KB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* The Nucleo G431KB board has four LEDs. Three of these are controlled by
 * logic on the board and are not available for software control:
 *
 * LD1 COM:  LD1 default status is red. LD1 turns to green to indicate that
 *           communications are in progress between the PC and the
 *           ST-LINK/V3.
 * LD3  OC:  indicates that the board power consumption on USB ST-LINK
 *           exceeds 500mA.
 * LD4 PWR:  green LED indicates that the board is powered.
 *
 * And one can be controlled by software:
 *
 * User LD2: green LED is a user LED connected to the I/O PA5 of the
 *           STM32G431KB.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED in
 * any way.  The following definition is used to access the LED.
 */

#define GPIO_LED1      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                        GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)

#define LED_DRIVER_PATH                "/dev/userleds"

/* PWM */

#define NUCLEOG431KB_PWM_TIMER   1
#define NUCLEOG431KB_PWM_PATH    "/dev/pwm0"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture specific initialization
 *
 *   CONFIG_LIB_BOARDCTL=y:
 *     If CONFIG_NSH_ARCHINITIALIZE=y:
 *       Called from the NSH library (or other application)
 *     Otherwise, assumed to be called from some other application.
 *
 *   Otherwise CONFIG_BOARD_LATE_INITIALIZE=y:
 *     Called from board_late_initialize().
 *
 *   Otherwise, bad news:  Never called
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_comp_setup
 *
 * Description:
 *  Initialize COMP peripheral for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_COMP
int stm32_comp_setup(void);
#endif

/****************************************************************************
 * Name: stm32_comp_setup
 *
 * Description:
 *  Initialize COMP peripheral for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_DAC
int stm32_dac_setup(void);
#endif

#endif /* __BOARDS_ARM_STM32_NUCLEO_G431KB_SRC_NUCLEO_G431KB_H */
