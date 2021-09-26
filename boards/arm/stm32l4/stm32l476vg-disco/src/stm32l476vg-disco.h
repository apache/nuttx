/****************************************************************************
 * boards/arm/stm32l4/stm32l476vg-disco/src/stm32l476vg-disco.h
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

#ifndef __BOARDS_ARM_STM32L4_STM32L476VG_DISCO_SRC_STM32L476VG_DISCO_H
#define __BOARDS_ARM_STM32L4_STM32L476VG_DISCO_SRC_STM32L476VG_DISCO_H

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
#define HAVE_N25QXXX          1
#define HAVE_N25QXXX_NXFFS    1
#define HAVE_N25QXXX_SMARTFS  1
#define HAVE_N25QXXX_CHARDEV  1
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

/* N25QXXX QuadSPI FLASH */

#ifndef CONFIG_MTD_N25QXXX
#  undef HAVE_N25QXXX
#  undef HAVE_N25QXXX_NXFFS
#  undef HAVE_N25QXXX_SMARTFS
#  undef HAVE_N25QXXX_CHARDEV
#endif

#ifndef CONFIG_STM32L4_QSPI
#  undef HAVE_N25QXXX
#  undef HAVE_N25QXXX_NXFFS
#  undef HAVE_N25QXXX_SMARTFS
#  undef HAVE_N25QXXX_CHARDEV
#endif

#ifndef CONFIG_FS_NXFFS
#  undef HAVE_N25QXXX_NXFFS
#endif

#if !defined(CONFIG_MTD_SMART) || !defined(CONFIG_FS_SMARTFS)
#  undef HAVE_N25QXXX_SMARTFS
#endif

#if defined(HAVE_N25QXXX_NXFFS) && defined(HAVE_N25QXXX_SMARTFS)
#  undef HAVE_N25QXXX_NXFFS
#endif

#if defined(HAVE_N25QXXX_NXFFS) || defined(HAVE_N25QXXX_SMARTFS)
#  undef HAVE_N25QXXX_CHARDEV
#endif

/* If both the N25QXXX FLASH and SmartFS, then this is the minor device
 * number of the Smart block driver (/dev/smartN)
 */

#define N25QXXX_SMART_MINOR 0

/* If the N25QXXX FLASH is enabled but not SmartFS, then the N25QXXX will be
 * wrapped as a character device.  This is the minor number of both the
 * block device (/dev/mtdblockN) and the character device (/dev/mtdN).
 */

#define N25QXXX_MTD_MINOR 0

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
 * PC11  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 * PC9   OTG_FS_PowerSwitchOn
 * PC10  OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_OPENDRAIN|GPIO_PORTC|GPIO_PIN11)
#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN9)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|\
                           GPIO_SPEED_100MHz|GPIO_PUSHPULL|\
                           GPIO_PORTC|GPIO_PIN10)

#else
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN10)
#endif

/* LED.
 * LD4: the red LED on PB2
 * LD5: the green LED on PE8
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

#define GPIO_LED_RED \
  (GPIO_PORTB | GPIO_PIN2 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PUSHPULL | \
   GPIO_PULLUP | GPIO_SPEED_50MHz)

#define GPIO_LED_GRN \
  (GPIO_PORTE | GPIO_PIN8 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PUSHPULL | \
   GPIO_PULLUP | GPIO_SPEED_50MHz)

/* Buttons
 *
 *  There is a 4 way d-pad 'joystick' with center button
 *  connected to PA0,1,5,2,3
 *                 C L D R U
 */

#define MIN_IRQBUTTON   BUTTON_CENTER
#define MAX_IRQBUTTON   BUTTON_UP
#define NUM_IRQBUTTONS  5

#define GPIO_BTN_CENTER \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)
#define GPIO_BTN_LEFT \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTA | GPIO_PIN1)
#define GPIO_BTN_DOWN \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTA | GPIO_PIN5)
#define GPIO_BTN_RIGHT \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTA | GPIO_PIN2)
#define GPIO_BTN_UP \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTA | GPIO_PIN3)

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

/* Devices on the onboard I2C bus.
 *
 * Note that these are unshifted addresses.
 */

/* XXX IS this 'unshifted'? */

#define DISCO_I2C_OBDEV_CS43L22   0x94

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32_SPI1
extern struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32_SPI2
extern struct spi_dev_s *g_spi2;
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

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

#endif /* __BOARDS_ARM_STM32L4_STM32L476VG_DISCO_SRC_STM32L476VG_DISCO_H */
