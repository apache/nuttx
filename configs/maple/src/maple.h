/************************************************************************************
 * configs/maple/src/maple.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Laurent Latil <laurent@latil.nom.fr>
 *           Librae <librae8226@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __CONFIGS_MAPLE_SRC_MAPLE_H
#define __CONFIGS_MAPLE_SRC_MAPLE_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/stm32/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

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

/* GPIOs **************************************************************/
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

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ************************************************************************************/

void stm32_spidev_initialize(void);

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ************************************************************************************/

void stm32_usbinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_MAPLE_SRC_MAPLE_H */
