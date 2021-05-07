/****************************************************************************
 * boards/arm/stm32/stm32_tiny/src/stm32_tiny.h
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

#ifndef __BOARDS_ARM_STM32_STM32_TINY_SRC_STM32_TINY_H
#define __BOARDS_ARM_STM32_STM32_TINY_SRC_STM32_TINY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many SPI modules does this chip support? The LM3S6918 supports 2 SPI
 * modules (others may support more -- in such case, the following must be
 * expanded).
 */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#endif

/* GPIOs ********************************************************************/

/* LEDs */

#define GPIO_LED        (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)

/* USB Soft Connect Pullup: PC.13 */

#define GPIO_USB_PULLUP (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)

/* NRF24L01 chip select:  PB.12 */

#define GPIO_NRF24L01_CS   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)

/* PWM
 *
 * Let's use the LED.  It is connected to PB.5, which can be used as
 * PWM output of channel 2 of timer 3
 * (STM32_TIM3_PARTIAL_REMAP must be enabled)
 */

#ifdef CONFIG_PWM
#  if defined(CONFIG_STM32_TIM3_PWM) && defined(CONFIG_STM32_TIM3_PARTIAL_REMAP) && CONFIG_STM32_TIM3_CHANNEL == 2
#    define STM32TINY_PWMTIMER 3
#  else
#    error To use the PWM device, the timer 3 partial remap must be enabled, and the PWM device must be configured on timer 3 / channel 2
#  endif
#endif

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
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Hy-Mini STM32v
 *   board.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the Hy-Mini STM32v board.
 *
 ****************************************************************************/

void stm32_usbinitialize(void);

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
 * Name: stm32_wlinitialize
 *
 * Description:
 *   Called to configure wireless module (nRF24L01).
 *
 ****************************************************************************/

void stm32_wlinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_STM32_TINY_SRC_STM32_TINY_H */
