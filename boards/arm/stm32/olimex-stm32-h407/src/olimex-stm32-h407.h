/****************************************************************************
 * boards/arm/stm32/olimex-stm32-h407/src/olimex-stm32-h407.h
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

#ifndef __BOARDS_ARM_STM32_OLIMEX_STM32_H407_SRC_OLIMEX_STM32_H407_H
#define __BOARDS_ARM_STM32_OLIMEX_STM32_H407_SRC_OLIMEX_STM32_H407_H

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

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1
#define HAVE_SDIO       1
#define HAVE_RTC_DRIVER 1
#
/* Can't support USB host or device features if USB OTG HS is not enabled */

#ifndef CONFIG_STM32_OTGHS
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

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO
 * support is not enabled.  Can't support MMC/SD features if the upper
 * half MMC/SD SDIO driver is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_STM32_SDIO)
#  undef HAVE_SDIO
#endif

#if !defined(CONFIG_MMCSD_SDIO)
#  undef HAVE_SDIO
#endif

#undef  SDIO_MINOR     /* Any minor number, default 0 */
#define SDIO_SLOTNO 0  /* Only one slot */

#ifdef HAVE_SDIO
#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != 0
#    warning Only one MMC/SD slot, slot 0
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  endif

#  if defined(CONFIG_NSH_MMCSDMINOR)
#    define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define SDIO_MINOR 0
#  endif
#endif

/* Check if we can support the RTC driver */

#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* Olimex-STM32-P407 GPIOs **************************************************/

/* LEDs */

#define GPIO_LED_STATUS    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                            GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN12)

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON      BUTTON_BUT
#define MAX_IRQBUTTON      BUTTON_BUT
#define NUM_IRQBUTTONS     1

#define GPIO_BTN_BUT      (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTA | \
                           GPIO_PIN0)

/* USB OTG FS - USB-A connector
 *
 * PC4  OTG_FS_VBUS VBUS sensing
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN4)

/* USB OTG HS - miniUSB connector
 *
 * PB13  OTG_HS_VBUS VBUS sensing (also connected to the green LED)
 * PA8  OTG_HS_PowerSwitchOn
 * PB5  OTG_HS_Overcurrent
 */

#define GPIO_OTGHS_VBUS   (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_100MHz | \
                           GPIO_OPENDRAIN | GPIO_PORTB | GPIO_PIN13)
#define GPIO_OTGHS_PWRON  (GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_FLOAT | \
                           GPIO_SPEED_100MHz | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN8)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGHS_OVER (GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT | \
                           GPIO_SPEED_100MHz | GPIO_PUSHPULL | GPIO_PORTB | \
                           GPIO_PIN5)

#else
#  define GPIO_OTGHS_OVER (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_100MHz | \
                           GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN5)
#endif

/* MicroSD SDIO
 *
 * ---------- --------  -------  ---------------------------
 * PIO        SIGNAL    Pulled   Comments
 * ---------- --------  -------  -----------------------
 * --         NCD                Card detect,
 *                               combined with pins settings CD/PC11
 * PC9        DAT1      UP 33K    Also interrupt
 * PC8        DAT0      UP 33K   "        " "" "    "
 * PC12       CLK        ----    "        " "" "    "
 * PD2        CMD       UP 33K   "        " "" "    "
 * PC11       CD/DAT3   DOWN 1M  "        " "" "    "
 * PC10       DAT2      UP 33K   Also Read wait operation
 * --         WrProtect          Not Supported
 * ---------- --------  -------  ----------------------
 */

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
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture specific initialization
 *
 *   CONFIG_BOARDCTL=y:
 *     If CONFIG_NSH_ARCHINITIALIZE=y:
 *       Called from the NSH library (or other application)
 *     Otherse, assumed to be called from some other application.
 *
 *   Otherwise CONFIG_BOARD_LATE_INITIALIZE=y:
 *     Called from board_late_initialize().
 *
 *   Otherwise, bad news:  Never called
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

#if defined(HAVE_SDIO)
int stm32_sdio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the Olimex-STM32-H405 board.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGFS) || defined(CONFIG_STM32_OTGHS)
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_CAN_CHARDRIVER
int stm32_can_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_OLIMEX_STM32_H407_SRC_OLIMEX_STM32_H407_H */
