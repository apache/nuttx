/****************************************************************************
 * boards/arm/stm32/nucleo-f207zg/src/nucleo-f207zg.h
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

#ifndef __BOARDS_ARM_STM32_NUCLEO_F207ZG_SRC_NUCLEO_F207ZG_H
#define __BOARDS_ARM_STM32_NUCLEO_F207ZG_SRC_NUCLEO_F207ZG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* LED
 *
 * The Nucleo-144 board has numerous LEDs but only three, LD1 a Green LED,
 * LD2 a Blue LED and LD3 a Red LED, that can be controlled by software.
 * The following definitions assume the default Solder Bridges are installed.
 */

#define GPIO_LD1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR |GPIO_PORTB | GPIO_PIN0)
#define GPIO_LD2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN7)
#define GPIO_LD3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN14)

#define GPIO_LED_GREEN GPIO_LD1
#define GPIO_LED_BLUE  GPIO_LD2
#define GPIO_LED_RED   GPIO_LD3

#define LED_DRIVER_PATH "/dev/userleds"

/* Button definitions *******************************************************/

/* The Nucleo F207ZG supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:  user button connected to the I/O PC13 of the STM32F207ZGT6.
 *   B2 RESET: push button connected to NRST is used to RESET the
 *             STM32F207ZGT6.
 *
 * NOTE that EXTI interrupts are configured.
 */

#define MIN_IRQBUTTON  BUTTON_USER
#define MAX_IRQBUTTON  BUTTON_USER
#define NUM_IRQBUTTONS 1

#define GPIO_BTN_USER  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 * PG6  OTG_FS_PowerSwitchOn
 * PG7  OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz| \
                           GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|  \
                           GPIO_PUSHPULL|GPIO_PORTG|GPIO_PIN6)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT| \
                           GPIO_SPEED_100MHz|GPIO_PUSHPULL| \
                           GPIO_PORTG|GPIO_PIN7)

#else
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz| \
                           GPIO_PUSHPULL|GPIO_PORTG|GPIO_PIN7)
#endif

/* PWM */

#define NUCLEOF207ZG_PWMTIMER 1

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
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
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

#endif /* __BOARDS_ARM_STM32_NUCLEO_F207ZG_SRC_NUCLEO_F207ZG_H */
