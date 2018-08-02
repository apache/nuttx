/************************************************************************************
 * configs/nucleo-l432kc/src/nucleo-l432kc.h
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
 *   Authors: Frank Bennett
 *            Gregory Nutt <gnutt@nuttx.org>
 *            Sebastien Lorquet <sebastien@lorquet.fr>
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

#ifndef __CONFIGS_NUCLEO_L432KC_SRC_NUCLEO_L432KC_H
#define __CONFIGS_NUCLEO_L432KC_SRC_NUCLEO_L432KC_H

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

/* LED.  User LD3: the green LED is a user LED connected to Arduino signal D13
 * corresponding to MCU I/O PB3 (pin 26)
 * target.
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

#define GPIO_LD3 \
  (GPIO_PORTB | GPIO_PIN3 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | \
   GPIO_SPEED_50MHz)
#define LED_DRIVER_PATH "/dev/userleds"

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32L4_SPI1
extern struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32L4_SPI2
extern struct spi_dev_s *g_spi2;
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32l4_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ************************************************************************************/

void stm32l4_spiinitialize(void);

/************************************************************************************
 * Name: stm32l4_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ************************************************************************************/

void stm32l4_usbinitialize(void);

/************************************************************************************
 * Name: stm32l4_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ************************************************************************************/

#ifdef CONFIG_PWM
int stm32l4_pwm_setup(void);
#endif

/************************************************************************************
 * Name: stm32l4_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ************************************************************************************/

#ifdef CONFIG_ADC
int stm32l4_adc_setup(void);
#endif

/************************************************************************************
 * Name: stm32_dac7571initialize
 *
 * Description:
 *   Initialize and register the DAC7571 driver.
 *
 ************************************************************************************/

#ifdef CONFIG_DAC7571
int stm32_dac7571initialize(FAR const char *devpath);
#endif

/****************************************************************************
 * Name: board_timer_driver_initialize
 *
 * Description:
 *   Initialize and register a timer
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int board_timer_driver_initialize(FAR const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: stm32l4_qencoder_initialize
 *
 * Description:
 *   Initialize and register a qencoder
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_QENCODER
int stm32l4_qencoder_initialize(FAR const char *devpath, int timer);
#endif

#endif /* __CONFIGS_NUCLEO_L432KC_SRC_NUCLEO_L432KC_H */
