/****************************************************************************
 * boards/arm/stm32l4/stm32l4r9ai-disco/src/stm32l4r9ai-disco.h
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

#ifndef __BOARDS_ARM_STM32L4_STM32L4R9AI_DISCO_SRC_STM32L4R9AI_DISCO_H
#define __BOARDS_ARM_STM32L4_STM32L4R9AI_DISCO_SRC_STM32L4R9AI_DISCO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include "stm32l4_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_PROC             1
#define HAVE_RTC_DRIVER       1
#define HAVE_USBDEV           1
#define HAVE_USBHOST          1
#define HAVE_USBMONITOR       1

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

/* This is the on-chip progmem memory driver minor number */

#define PROGMEM_MTD_MINOR 1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32L4_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#endif

/* Can't support USB device if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

/* USB OTG FS
 *
 * PA9   OTG_FS_VBUS VBUS sensing
 * PB13  OTG_FS_PowerSwitchOn
 * PB14  OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)
#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN13)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|\
                           GPIO_SPEED_100MHz|GPIO_PUSHPULL|\
                           GPIO_PORTB|GPIO_PIN14)

#else
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN14)
#endif

/* LED.
 * LD1: the orange LED on PB0
 * LD2: the green LED on PH4
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

#define GPIO_LED_RED \
  (GPIO_PORTB | GPIO_PIN0 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PUSHPULL | \
   GPIO_PULLUP | GPIO_SPEED_50MHz)

#define GPIO_LED_GRN \
  (GPIO_PORTH | GPIO_PIN4 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PUSHPULL | \
   GPIO_PULLUP | GPIO_SPEED_50MHz)

/* Buttons
 *
 *  There is a 4 way d-pad 'joystick' with center button
 *  connected to PC13 and others to PB4,3,2,1
 *                 C L D R U
 */

#define MIN_IRQBUTTON   BUTTON_CENTER
#define MAX_IRQBUTTON   BUTTON_UP
#define NUM_IRQBUTTONS  5

#define GPIO_BTN_CENTER \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTC | GPIO_PIN13)
#define GPIO_BTN_LEFT \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTB | GPIO_PIN4)
#define GPIO_BTN_DOWN \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTB | GPIO_PIN2)
#define GPIO_BTN_RIGHT \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTB | GPIO_PIN3)
#define GPIO_BTN_UP \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTB | GPIO_PIN1)

/* SPI1 off */

/* XXX is this used on disco? */

#define GPIO_SPI1_MOSI_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN15)
#define GPIO_SPI1_MISO_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN14)
#define GPIO_SPI1_SCK_OFF  (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN13)
#define GPIO_SPI1_NSS_OFF  (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN12)

/* Devices on the onboard I2C bus.  */

#define DISCO_I2C_OBDEV_DSI       0x70
#define DISCO_I2C_OBDEV_CS43L22   0x94
#define DISCO_I2C_OBDEV_CAMERA    0x60
#define DISCO_I2C_OBDEV_MFX_V3    0x84

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32L4_SPI1
extern struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32L4_SPI2
extern struct spi_dev_s *g_spi2;
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int stm32l4_adc_setup(void);

/****************************************************************************
 * Name: stm32l4_adc_measure_voltages
 *
 * Description:
 *   Read internal reference voltage, internal VBAT and one external voltage.
 *
 ****************************************************************************/

int stm32l4_adc_measure_voltages(uint32_t *vrefint,
                                 uint32_t *vbat,
                                 uint32_t *vext);

/****************************************************************************
 * Name: stm32l4_dac_setup
 *
 * Description:
 *   Initialize DAC and register the DAC driver.
 *
 ****************************************************************************/

int stm32l4_dac_setup(void);

/****************************************************************************
 * Name: stm32_dfsdm_setup
 *
 * Description:
 *   Initialize DFSDM and register the ADC drivers for DFSDM filters.
 *
 ****************************************************************************/

#if defined(CONFIG_ADC) && defined(CONFIG_STM32L4_DFSDM)
int stm32_dfsdm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

void stm32_spiinitialize(void);

/****************************************************************************
 * Name: stm32l4_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ****************************************************************************/

void stm32l4_usbinitialize(void);

#endif /* __BOARDS_ARM_STM32L4_STM32L4R9AI_DISCO_SRC_STM32L4R9AI_DISCO_H */
