/****************************************************************************
 * boards/arm/stm32f0l0g0/stm32g071b-disco/src/stm32g071b-disco.h
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

#ifndef __BOARDS_ARM_STM32F0L0G0_STM32G071B_DISCO_SRC_STM32G071B_DISCO_H
#define __BOARDS_ARM_STM32F0L0G0_STM32G071B_DISCO_SRC_STM32G071B_DISCO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

#define GPIO_LEDSINK      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_HIGH | \
                           GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN9)
#define GPIO_LEDSOURCE    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_HIGH | \
                           GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN8)
#define GPIO_LEDSPY       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_HIGH | \
                           GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN12)
#define GPIO_LEDCC        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_HIGH | \
                           GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN5)

#define LED_DRIVER_PATH "/dev/userleds"

/* Joystic definitions ******************************************************/

#define GPIO_JOY_SEL     (GPIO_INPUT | GPIO_PULLDOWN | GPIO_EXTI | \
                          GPIO_PORTC | GPIO_PIN0)
#define GPIO_JOY_DOWN    (GPIO_INPUT | GPIO_PULLDOWN | GPIO_EXTI | \
                          GPIO_PORTC | GPIO_PIN2)
#define GPIO_JOY_LEFT    (GPIO_INPUT | GPIO_PULLDOWN | GPIO_EXTI | \
                          GPIO_PORTC | GPIO_PIN1)
#define GPIO_JOY_RIGHT   (GPIO_INPUT | GPIO_PULLDOWN | GPIO_EXTI | \
                          GPIO_PORTC | GPIO_PIN3)
#define GPIO_JOY_UP      (GPIO_INPUT | GPIO_PULLDOWN | GPIO_EXTI | \
                          GPIO_PORTC | GPIO_PIN4)

/* OLED definitions *********************************************************/

#define GPIO_SSD1306_CS    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_HIGH | \
                            GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN3)

#define GPIO_SSD1306_CMD   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_HIGH | \
                            GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN7)

#define GPIO_SSD1306_RST   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_HIGH | \
                            GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN6)

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
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_djoy_initialization
 *
 * Description:
 *   Initialize and register the discrete joystick driver
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_DJOYSTICK
int stm32_djoy_initialization(void);
#endif

/****************************************************************************
 * Name: stm32_ina226_initialization
 *
 * Description:
 *   Initialize and configure the INA226
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_INA226
int stm32_ina226_initialization(void);
#endif

#endif /* __BOARDS_ARM_STM32F0L0G0_STM32G071B_DISCO_SRC_STM32G071B_DISCO_H */
