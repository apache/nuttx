/****************************************************************************
 * boards/arm/kinetis/teensy-3.x/src/teensy-3x.h
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
 ****************************************************************************/

#ifndef __BOARDS_ARM_KINETIS_TEENSY_3X_SRC_TEENSY_3X_H
#define __BOARDS_ARM_KINETIS_TEENSY_3X_SRC_TEENSY_3X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/kinetis/chip.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many SPI modules does this chip support? The LM3S6918 supports 2 SPI
 * modules (others may support more -- in such case, the following must be
 * expanded).
 */

#if KINETIS_NSPI < 1
#  undef CONFIG_KINETIS_SPI1
#  undef CONFIG_KINETIS_SPI2
#elif KINETIS_NSPI < 2
#  undef CONFIG_KINETIS_SPI2
#endif

/* Teensy 3.1 GPIOs *********************************************************/

/* A single LED is available driven by PTC5.
 * The LED is grounded so bringing PTC5 high will illuminate the LED.
 */

#define GPIO_LED          (GPIO_HIGHDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTC | PIN5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

FAR struct i2c_master_s* i2c_dev;

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the KwikStik-K40 board.
 *
 ****************************************************************************/

extern void weak_function kinetis_spidev_initialize(void);

/****************************************************************************
 * Name: kinetis_i2cdev_initialize
 *
 * Description:
 *   Called to configure I2C
 *
 ****************************************************************************/

void kinetis_i2cdev_initialize(void);

/****************************************************************************
 * Name: kinetis_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the KwikStik-K40 board.
 *
 ****************************************************************************/

extern void weak_function kinetis_usbinitialize(void);

/****************************************************************************
 * Name: kinetis_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int kinetis_pwm_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_KINETIS_TEENSY_3X_SRC_TEENSY_3X_H */
