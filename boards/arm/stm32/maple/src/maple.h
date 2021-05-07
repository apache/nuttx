/****************************************************************************
 * boards/arm/stm32/maple/src/maple.h
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

#ifndef __BOARDS_ARM_STM32_MAPLE_SRC_MAPLE_H
#define __BOARDS_ARM_STM32_MAPLE_SRC_MAPLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/stm32/chip.h>

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

/* GPIO settings for LEDs and USB */

#ifdef CONFIG_MAPLE_MINI
#  define GPIO_LED           (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                              GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#  define GPIO_USB_PULLUP    (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                              GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN9)
#else
#  define GPIO_LED           (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                              GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN5)
#  define GPIO_USB_PULLUP    (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                              GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN12)
#endif

/* The Maple configuration has been used to very the Sharp Memory LCD
 * on a custom board.  These are pin definitions for that custom
 * board interface.  If you should decide to integrate the Sharp
 * Memory LCD with your Maple board, you may need to changes these
 * settings.
 */

#define GPIO_MEMLCD_EXTCOMIN (GPIO_PORTA | GPIO_PIN13 | GPIO_OUTPUT_CLEAR | \
                              GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)
#define GPIO_MEMLCD_DISP     (GPIO_PORTA | GPIO_PIN14 | GPIO_OUTPUT_CLEAR | \
                              GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)
#define GPIO_MEMLCD_CS       (GPIO_PORTA | GPIO_PIN15 | GPIO_OUTPUT_CLEAR | \
                              GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ****************************************************************************/

void stm32_usbinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_MAPLE_SRC_MAPLE_H */
