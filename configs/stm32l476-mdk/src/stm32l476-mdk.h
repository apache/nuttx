/************************************************************************************
 * configs/stm32l476-mdk/src/stm32l476-mdk.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Authors: Frank Bennett
 *            Gregory Nutt <gnutt@nuttx.org>
 *            Sebastien Lorquet <sebastien@lorquet.fr>
 *            dev@ziggurat29.com
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

#ifndef __CONFIGS_STM32L476_MDK_SRC_STM32L476_MDK_H
#define __CONFIGS_STM32L476_MDK_SRC_STM32L476_MDK_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#define HAVE_PROC             1
#define HAVE_RTC_DRIVER       1
#define HAVE_USERLED_DRIVER   1

#if !defined(CONFIG_FS_PROCFS)
#  undef HAVE_PROC
#endif

#if defined(HAVE_PROC) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No procfs support
#  undef HAVE_PROC
#endif

/* Check if we can support the RTC driver */

#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* Check if we have the LED driver */

#if !defined(CONFIG_USERLED) || !defined(CONFIG_USERLED_LOWER)
#  undef HAVE_USERLED_DRIVER
#endif

/* LEDs.
 * The Reference Moto Mod contains three LEDs.  Two LEDs, are by convention,
 * used to indicate the Reference Moto Mod battery state of charge, and the
 * other is available for you to use in your applications.
 *
 *   1. The red LED on PD7.  Part of the (rear-firing) red/green LED.
 *   2. The green LED on PE7.  Part of the (rear-firing) red/green LED.
 *   3. The white (top-firing) LED on PE8
 *
 * When the I/O is HIGH value, the LED is OFF.
 * When the I/O is LOW, the LED is ON.
 */

#define GPIO_LED_RED \
  (GPIO_PORTD | GPIO_PIN7 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PUSHPULL | \
   GPIO_PULLUP | GPIO_SPEED_50MHz)

#define GPIO_LED_GREEN \
  (GPIO_PORTE | GPIO_PIN7 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PUSHPULL | \
   GPIO_PULLUP | GPIO_SPEED_50MHz)

#define GPIO_LED_WHITE \
  (GPIO_PORTE | GPIO_PIN8 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PUSHPULL | \
   GPIO_PULLUP | GPIO_SPEED_50MHz)

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON   BUTTON_POWER
#define MAX_IRQBUTTON   BUTTON_POWER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_POWER  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN2)

/* SPI1 off */

#define GPIO_SPI1_MOSI_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN15)
#define GPIO_SPI1_MISO_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN14)
#define GPIO_SPI1_SCK_OFF  (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN13)
#define GPIO_SPI1_NSS_OFF  (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN12)

/* Devices on the onboard I2C bus.
 *
 * Note that these are unshifted addresses.
 */

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32_SPI1
extern struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32_SPI2
extern struct spi_dev_s *g_spi2;
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ************************************************************************************/

void stm32_spiinitialize(void);

#endif /* __CONFIGS_STM32L476_MDK_SRC_STM32L476_MDK_H */
