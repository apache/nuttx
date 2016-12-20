/****************************************************************************************************
 * configs/freedom-kl26z/src/freedom-kl26z.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_FREEDOM_KL26Z_SRC_FREEDOM_KL26Z_H
#define __CONFIGS_FREEDOM_KL26Z_SRC_FREEDOM_KL26Z_H

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

/* Freedom KL26Z GPIOs ******************************************************************************/
/* The Freedom KL26Z has a single RGB LED driven by the KL26Z as follows:
 *
 *   ------------- --------
 *   RGB LED       KL26Z128
 *   ------------- --------
 *   Red Cathode   PTE29
 *   Green Cathode PTE31
 *   Blue Cathode  PTD5
 *
 * NOTE: PTD5 is also connected to the I/O header on J2 pin 12 (also known as D13).
 *
 * If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board the
 * Freedom KL26Z.  The following definitions describe how NuttX controls the LEDs:
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
 *   LED_IDLE             K26Z1XX is in sleep mode  (Optional, not used)
 */

#define GPIO_LED_R (GPIO_OUTPUT | GPIO_OUTPUT_ONE | PIN_PORTE | PIN29)
#define GPIO_LED_G (GPIO_OUTPUT | GPIO_OUTPUT_ONE | PIN_PORTE | PIN31)
#define GPIO_LED_B (GPIO_OUTPUT | GPIO_OUTPUT_ONE | PIN_PORTD | PIN5)

/* Button definitions *******************************************************************************/
/* The Freedom KL26Z has no buttons */

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
 * Name: kl_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Freedom KL26Z board.
 *
 ****************************************************************************************************/

void weak_function kl_spidev_initialize(void);

/****************************************************************************************************
 * Name: kl_usbinitialize
 *
 * Description:
 *   Called from kl_usbinitialize very early in inialization to setup USB-related
 *   GPIO pins for the Freedom KL26Z board.
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_USB
void weak_function kl_usbinitialize(void);
#endif

/****************************************************************************************************
 * Name: kl_led_initialize
 *
 * Description:
 *   Initialize the on-board LED
 *
 ****************************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void kl_led_initialize(void);
#endif

/************************************************************************************
 * Name: kl_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ************************************************************************************/

#ifdef CONFIG_PWM
int kl_pwm_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_FREEDOM_KL26Z_SRC_FREEDOM_KL26Z_H */
