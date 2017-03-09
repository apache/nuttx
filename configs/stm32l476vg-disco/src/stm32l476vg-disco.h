/************************************************************************************
 * configs/stm32l476vg-disco/src/stm32l476vg-disco.h
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

#ifndef __CONFIGS_STM32L476VG_DISCO_SRC_STM32L476VG_DISCO_H
#define __CONFIGS_STM32L476VG_DISCO_SRC_STM32L476VG_DISCO_H

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

/* This is the on-chip progmem memroy driver minor number */

#define PROGMEM_MTD_MINOR 1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32L4_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB device monitor if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBDEV_TRACE) || !defined(CONFIG_SYSTEM_USBMONITOR)
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

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ************************************************************************************/

void stm32_spiinitialize(void);

/************************************************************************************
 * Name: stm32l4_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ************************************************************************************/

void stm32l4_usbinitialize(void);

#endif /* __CONFIGS_STM32L476VG_DISCO_SRC_STM32L476VG_DISCO_H */
