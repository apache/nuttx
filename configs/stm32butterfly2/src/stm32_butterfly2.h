/*****************************************************************************
 * configs/stm32butterfly2/src/stm32_butterfly2.h
 *
 *   Copyright (C) 2016 Michał Łyszczek. All rights reserved.
 *   Author: Michał Łyszczek <michal.lyszczek@gmail.com>
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
 ****************************************************************************/

#ifndef __CONFIGS_STM32_BUTTERFLY2_SRC_STM32_BUTTERFLY2_H
#define __CONFIGS_STM32_BUTTERFLY2_SRC_STM32_BUTTERFLY2_H 1

/*****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32_gpio.h"

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SD Card pins */

#define GPIO_SD_CS      (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz |\
                         GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN4)
#define GPIO_SD_CD      (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_EXTI |\
                         GPIO_PORTB | GPIO_PIN9)

/* USB pins */

#define GPIO_OTGFS_PWRON (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz |\
                          GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN15)

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: stm32_led_initialize
 *
 * Description:
 *      Initializes low level gpio pins for board LEDS
 ****************************************************************************/

void stm32_led_initialize(void);

/*****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 * Note:
 *   Here only CS pins are configured as SPI pins are configured by driver
 *   itself.
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI1
void stm32_spidev_initialize(void);
#else
static inline void stm32_spidev_initialize(void) {}
#endif

/*****************************************************************************
 * Name: stm32_mmcsd_initialize
 *
 * Description:
 *   Initializes SPI-based SD card
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD
int stm32_mmcsd_initialize(int minor);
#else
static inline int stm32_mmcsd_initialize(int minor) { (void)minor; return 0; }
#endif

/*****************************************************************************
 * Name: stm32_usb_initialize
 *
 * Description:
 *   Initializes USB pins
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void stm32_usb_initialize(void);
#else
static inline void stm32_usb_initialize(void) {}
#endif

/*****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Initializes USB host functionality.
 ****************************************************************************/

#ifdef CONFIG_USBHOST
int stm32_usbhost_initialize(void);
#else
static inline int stm32_usbhost_initialize(void) { return 0; }
#endif

/************************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_ADC
int stm32_adc_setup(void);
#else
static inline int stm32_adc_setup(void) { return 0; }
#endif

#endif /* __CONFIGS_STM32_BUTTERFLY2_SRC_STM32_BUTTERFLY2_H */

