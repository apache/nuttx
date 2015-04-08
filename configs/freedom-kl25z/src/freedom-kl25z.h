/****************************************************************************************************
 * configs/freedom-kl25z/src/freedom-kl25z.h
 * arch/arm/src/board/freedom-kl25z.c
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
 ****************************************************************************************************/

#ifndef __CONFIGS_FREEDOM_KL25Z_SRC_FREEDOM_KL25Z_H
#define __CONFIGS_FREEDOM_KL25Z_SRC_FREEDOM_KL25Z_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* Freedom KL25Z GPIOs ******************************************************************************/
/* The Freedom KL25Z has a single RGB LED driven by the KL25Z as follows:
 *
 *   ------------- --------
 *   RGB LED       KL25Z128
 *   ------------- --------
 *   Red Cathode   PTB18
 *   Green Cathode PTB19
 *   Blue Cathode  PTD1
 *
 * NOTE: PTD1 is also connected to the I/O header on J2 pin 10 (also known as D13).
 *
 * If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board the
 * Freedom KL25Z.  The following definitions describe how NuttX controls the LEDs:
 *
 *   SYMBOL                Meaning                 LED state
 *                                                 Initially all LED is OFF
 *   -------------------  -----------------------  --------------------------
 *   LED_STARTED          NuttX has been started
 *   LED_HEAPALLOCATE     Heap has been allocated
 *   LED_IRQSENABLED      Interrupts enabled
 *   LED_STACKCREATED     Idle stack created
 *   LED_INIRQ            In an interrupt
 *   LED_SIGNAL           In a signal handler
 *   LED_ASSERTION        An assertion failed
 *   LED_PANIC            The system has crashed
 *   LED_IDLE             K25Z1XX is in sleep mode  (Optional, not used)
 */

#define GPIO_LED_R (GPIO_OUTPUT | GPIO_OUTPUT_ONE | PIN_PORTB | PIN18)
#define GPIO_LED_G (GPIO_OUTPUT | GPIO_OUTPUT_ONE | PIN_PORTB | PIN19)
#define GPIO_LED_B (GPIO_OUTPUT | GPIO_OUTPUT_ONE | PIN_PORTD | PIN1)

/* Button definitions *******************************************************************************/
/* The Freedom KL25Z has no buttons */

/* Chip selects ************************************************************************************/

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: kl_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Freedom KL25Z board.
 *
 ****************************************************************************************************/

void weak_function kl_spiinitialize(void);

/****************************************************************************************************
 * Name: kl_usbinitialize
 *
 * Description:
 *   Called from kl_usbinitialize very early in inialization to setup USB-related
 *   GPIO pins for the Freedom KL25Z board.
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_USB
void weak_function kl_usbinitialize(void);
#endif

/****************************************************************************************************
 * Name: kl_ledinit
 *
 * Description:
 *   Initialize the on-board LED
 *
 ****************************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void kl_ledinit(void);
#endif


#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_FREEDOM_KL25Z_SRC_FREEDOM_KL25Z_H */

