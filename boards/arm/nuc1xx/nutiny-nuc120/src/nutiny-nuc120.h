/****************************************************************************
 * boards/arm/nuc1xx/nutiny-nuc120/src/nutiny-nuc120.h
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
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
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
