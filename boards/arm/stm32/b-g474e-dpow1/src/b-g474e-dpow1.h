/****************************************************************************
 * boards/arm/stm32/b-g474e-dpow1/src/b-g474e-dpow1.h
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

#ifndef __BOARDS_ARM_STM32_B_G474E_DPOW1_SRC_B_G474E_DPOW1_H
#define __BOARDS_ARM_STM32_B_G474E_DPOW1_SRC_B_G474E_DPOW1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* The B-G474E-DPOW1 Discovery kit has four user LEDs:
 *
 * | Symbol     | PCB | Color  |
 * |------------|-----|--------|
 * | BOARD_LED1 | LD2 | Blue   |
 * | BOARD_LED2 | LD3 | Orange |
 * | BOARD_LED3 | LD4 | Green  |
 * | BOARD_LED4 | LD5 | Red    |
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED.
 *
 * Pin assignments follow:
 */

#define GPIO_LED1                      (GPIO_OUTPUT | GPIO_PUSHPULL | \
                                        GPIO_SPEED_5MHz | \
                                        GPIO_OUTPUT_CLEAR | \
                                        GPIO_PORTA | GPIO_PIN15)
#define GPIO_LED2                      (GPIO_OUTPUT | GPIO_PUSHPULL | \
                                        GPIO_SPEED_5MHz | \
                                        GPIO_OUTPUT_CLEAR | \
                                        GPIO_PORTB | GPIO_PIN1)
#define GPIO_LED3                      (GPIO_OUTPUT | GPIO_PUSHPULL | \
                                        GPIO_SPEED_5MHz | \
                                        GPIO_OUTPUT_CLEAR | \
                                        GPIO_PORTB | GPIO_PIN7)
#define GPIO_LED4                      (GPIO_OUTPUT | GPIO_PUSHPULL | \
                                        GPIO_SPEED_5MHz | \
                                        GPIO_OUTPUT_CLEAR | \
                                        GPIO_PORTB | GPIO_PIN5)

#define LED_DRIVER_PATH                "/dev/userleds"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __BOARDS_ARM_STM32_B_G474E_DPOW1_SRC_B_G474E_DPOW1_H */
