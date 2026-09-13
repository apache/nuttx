/****************************************************************************
 * boards/arm/n32h7/n32h762iil7/src/n32h762iil7.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM_N32H7_N32H762IIL7_SRC_N32H762IIL7_H
#define __BOARDS_ARM_N32H7_N32H762IIL7_SRC_N32H762IIL7_H

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

#define HAVE_PROC            1
#define HAVE_SDIO            1
#define HAVE_USBDEV          1
#define HAVE_USBHOST         1
#define HAVE_USBMONITOR      1
#define HAVE_MTDCONFIG       1
#define HAVE_PROGMEM_CHARDEV 1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_N32H7_USBHS
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

#if !defined(CONFIG_N32H7_PROGMEM) || !defined(CONFIG_MTD_PROGMEM)
#  undef HAVE_PROGMEM_CHARDEV
#endif

/* This is the on-chip progmem memory driver minor number */

#define PROGMEM_MTD_MINOR 0

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO
 * support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_N32H7_SDMMC)
#  undef HAVE_SDIO
#endif

#define SDIO_SLOTNO 0  /* Only one slot */

#ifdef HAVE_SDIO
#  if !defined(CONFIG_NSH_MMCSDSLOTNO)
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  elif CONFIG_NSH_MMCSDSLOTNO != 0
#    warning "Only one MMC/SD slot, slot 0"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  endif
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define N32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define N32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Check if we can support the RTC driver */

#define HAVE_RTC_DRIVER 0
#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* LED
 *
 * The N32H762IIL7 board has numerous LEDs but only three, LD1 a Green LED,
 * LD2 a Blue LED and LD3 a Red LED, that can be controlled by software. The
 * following definitions assume the default Solder Bridges are installed.
 */

#define GPIO_LD1       (GPIO_PORTA | GPIO_PIN1)
#define GPIO_LD2       (GPIO_PORTA | GPIO_PIN2)
#define GPIO_LD3       (GPIO_PORTA | GPIO_PIN3)

#define GPIO_LED_RED    GPIO_LD1
#define GPIO_LED_GREEN  GPIO_LD2
#define GPIO_LED_BLUE   GPIO_LD3

#define LED_DRIVER_PATH "/dev/userleds"

/* BUTTONS
 *
 * The Blue pushbutton B1, labeled "User", is connected to GPIO PC13.
 * A high value will be sensed when the button is depressed.
 * Note:
 *    1) That the EXTI is included in the definition to enable an interrupt
 *       on this IO.
 *    2) The following definitions assume the default Solder Bridges are
 *       installed.
 */

#define GPIO_BTN_WKUP  (GPIO_PORTA | GPIO_PIN0 | GPIO_INPUT | GPIO_FLOAT)
#define GPIO_BTN_KEY1  (GPIO_PORTC | GPIO_PIN0 | GPIO_INPUT | GPIO_PULLUP)
#define GPIO_BTN_KEY2  (GPIO_PORTC | GPIO_PIN1 | GPIO_INPUT | GPIO_PULLUP)
#define GPIO_BTN_KEY3  (GPIO_PORTC | GPIO_PIN4 | GPIO_INPUT | GPIO_PULLUP)

#define BTN_DRIVER_PATH "/dev/buttons"

/* PWM */

#define N32H762IIL7_PWM0TIMER 1
#define N32H762IIL7_PWM1TIMER 12

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: n32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y &&
 *   CONFIG_NSH_ARCHINIT:
 *     Called from the NSH library
 *
 ****************************************************************************/

int n32_bringup(void);

/****************************************************************************
 * Name: n32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the
 *   Nucleo-H743ZI board.
 *
 ****************************************************************************/

#ifdef CONFIG_N32H7_SPI
void n32_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: n32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int n32_adc_setup(void);
#endif

/****************************************************************************
 * Name: n32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO-Driver.
 *
 ****************************************************************************/

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
int n32_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: n32_usbinitialize
 *
 * Description:
 *   Called from n32_usbinitialize very early in inialization to setup
 *   USB-related GPIO pins for the NUCLEO-H743ZI board.
 *
 ****************************************************************************/

#ifdef CONFIG_N32H7_USBHS
void weak_function n32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: n32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if (defined(CONFIG_N32H7_USBHS1_HOST) || defined(CONFIG_N32H7_USBHS2_HOST)) && defined(CONFIG_USBHOST)
int n32_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: n32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int n32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: n32_mtd_initialize
 *
 * Description:
 *   Initialize MTD drivers.
 *
 ****************************************************************************/
#ifdef CONFIG_MTD

#ifdef HAVE_PROGMEM_CHARDEV
int n32_progmem_init(void);
#endif /* HAVE_PROGMEM_CHARDEV */
#endif

/****************************************************************************
 * Name: n32_cordic_setup
 *
 * Description:
 *  Initialize CORDIC peripheral for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_MATH_CORDIC
int n32_cordic_setup(void);
#endif

/****************************************************************************
 * Name: n32_sdmmc_initialize
 *
 * Description:
 *   Initialize SDMMC-based MMC/SD card support
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_SDIO
int n32_sdmmc_initialize(int minor);
#endif

#endif /* __BOARDS_ARM_N32H7_N32H762IIL7_SRC_N32H762IIL7_H */
