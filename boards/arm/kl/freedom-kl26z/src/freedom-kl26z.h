/****************************************************************************
 * boards/arm/kl/freedom-kl26z/src/freedom-kl26z.h
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

#ifndef __BOARDS_ARM_KL_FREEDOM_KL26Z_SRC_FREEDOM_KL26Z_H
#define __BOARDS_ARM_KL_FREEDOM_KL26Z_SRC_FREEDOM_KL26Z_H

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

/* Freedom KL26Z GPIOs ******************************************************/

/* The Freedom KL26Z has a single RGB LED driven by the KL26Z as follows:
 *
 *   ------------- --------
 *   RGB LED       KL26Z128
 *   ------------- --------
 *   Red Cathode   PTE29
 *   Green Cathode PTE31
 *   Blue Cathode  PTD5
 *
 * NOTE:
 * PTD5 is also connected to the I/O header on J2 pin 12 (also known as D13).
 *
 * If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the Freedom KL26Z.
 * The following definitions describe how NuttX controls the LEDs:
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

/* Button definitions *******************************************************/

/* The Freedom KL26Z has no buttons */

/* Chip selects *************************************************************/

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
 * Name: kl_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Freedom KL26Z
 *   board.
 *
 ****************************************************************************/

void weak_function kl_spidev_initialize(void);

/****************************************************************************
 * Name: kl_usbinitialize
 *
 * Description:
 *   Called from kl_usbinitialize very early in inialization to setup
 *   USB-related GPIO pins for the Freedom KL26Z board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_USB
void weak_function kl_usbinitialize(void);
#endif

/****************************************************************************
 * Name: kl_led_initialize
 *
 * Description:
 *   Initialize the on-board LED
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void kl_led_initialize(void);
#endif

/****************************************************************************
 * Name: kl_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int kl_pwm_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_KL_FREEDOM_KL26Z_SRC_FREEDOM_KL26Z_H */
