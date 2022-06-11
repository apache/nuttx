/****************************************************************************
 * boards/arm/stm32/stm32butterfly2/src/stm32_butterfly2.h
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

#ifndef __BOARDS_ARM_STM32_STM32_BUTTERFLY2_SRC_STM32_BUTTERFLY2_H
#define __BOARDS_ARM_STM32_STM32_BUTTERFLY2_SRC_STM32_BUTTERFLY2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32_gpio.h"

/****************************************************************************
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

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_led_initialize
 *
 * Description:
 *      Initializes low level gpio pins for board LEDS
 ****************************************************************************/

void stm32_led_initialize(void);

/****************************************************************************
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
static inline void stm32_spidev_initialize(void)
{
}
#endif

/****************************************************************************
 * Name: stm32_mmcsd_initialize
 *
 * Description:
 *   Initializes SPI-based SD card
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD
int stm32_mmcsd_initialize(int minor);
#else
static inline int stm32_mmcsd_initialize(int minor)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_usb_initialize
 *
 * Description:
 *   Initializes USB pins
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void stm32_usb_initialize(void);
#else
static inline void stm32_usb_initialize(void)
{
}
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Initializes USB host functionality.
 ****************************************************************************/

#ifdef CONFIG_USBHOST
int stm32_usbhost_initialize(void);
#else
static inline int stm32_usbhost_initialize(void)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_ADC
int stm32_adc_setup(void);
#else
static inline int stm32_adc_setup(void)
{
  return 0;
}
#endif

#endif /* __BOARDS_ARM_STM32_STM32_BUTTERFLY2_SRC_STM32_BUTTERFLY2_H */
