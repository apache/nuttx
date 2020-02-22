/****************************************************************************
 * boards/arm/nuc1xx/nutiny-nuc120/src/nutiny-nuc120.h
 * arch/arm/src/board/nutiny-nuc120.n
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

#ifndef __BOARDS_ARM_NUC1XX_NUTINY_NUC120_SRC_NUTINY_NUC120_H
#define __BOARDS_ARM_NUC1XX_NUTINY_NUC120_SRC_NUTINY_NUC120_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* NuTiny-EVB-120 GPIOs *****************************************************/

/* The NuTiny has a single green LED that can be controlled from software.
 * This LED is connected to PIN17 (PB.0).
 * It is pulled high so a low value will illuminate the LED.
 *
 * If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the NuTiny.
 * The following definitions describe how NuttX controls the LEDs:
 *
 *   SYMBOL                Meaning                 LED state
 *                                                 Initially all LED is OFF
 *   -------------------  -----------------------  ------------- ------------
 *   LED_STARTED          NuttX has been started   LED ON
 *   LED_HEAPALLOCATE     Heap has been allocated  LED ON
 *   LED_IRQSENABLED      Interrupts enabled       LED ON
 *   LED_STACKCREATED     Idle stack created       LED ON
 *   LED_INIRQ            In an interrupt          LED should glow
 *   LED_SIGNAL           In a signal handler      LED might glow
 *   LED_ASSERTION        An assertion failed      LED ON while handling the
 *                                                        assertion
 *   LED_PANIC            The system has crashed   LED Blinking at 2Hz
 *   LED_IDLE             NUC1XX is in sleep mode   (Optional, not used)
 */

#define GPIO_LED (GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN0)

/* Button definitions *******************************************************/

/* The NuTiny has no buttons */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nuc_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NuTiny-EVB-120
 *   board.
 *
 ****************************************************************************/

void weak_function nuc_spidev_initialize(void);

/****************************************************************************
 * Name: nuc_usbinitialize
 *
 * Description:
 *   Called from nuc_usbinitialize very early in inialization to setup
 *   USB-related GPIO pins for the NuTiny-EVB-120 board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_USB
void weak_function nuc_usbinitialize(void);
#endif

/****************************************************************************
 * Name: nuc_led_initialize
 *
 * Description:
 *   Initialize the on-board LED
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void nuc_led_initialize(void);
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_NUC1XX_NUTINY_NUC120_SRC_NUTINY_NUC120_H */
