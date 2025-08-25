/****************************************************************************
 * boards/arm/samv7/pic32czca70-curiosity/src/sam_board.h
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

#ifndef __BOARDS_ARM_SAMV7_PIC32CZCA70_CURIOSITY_SRC_SAM_BOARD_H
#define __BOARDS_ARM_SAMV7_PIC32CZCA70_CURIOSITY_SRC_SAM_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

#define HAVE_LED_DRIVER      1
#define HAVE_HSMCI           1
#define HAVE_AUTOMOUNTER     1
#define HAVE_USB             1
#define HAVE_USBDEV          1
#define HAVE_PROGMEM_CHARDEV 1

/* HSMCI */

/* Can't support MMC/SD if the card interface is not enabled */

#if !defined(CONFIG_SAMV7_HSMCI0)
#  undef HAVE_HSMCI
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_HSMCI) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled. No MMC/SD support
#  undef HAVE_HSMCI
#endif

/* We need GPIO interrupts on GPIOD to support card detect interrupts */

#if defined(HAVE_HSMCI) && !defined(CONFIG_SAMV7_GPIOC_IRQ)
#  warning GPIOC interrupts not enabled. No MMC/SD support.
#  undef HAVE_HSMCI
#endif

/* USB Device
 *
 * CONFIG_SAMV7_UDP and CONFIG_USBDEV must be defined, or there is no USB
 * device.
 */

#if !defined(CONFIG_SAMV7_UDP) || !defined(CONFIG_USBDEV)
#  undef HAVE_USB
#  undef HAVE_USBDEV
#endif

/* On-chip Programming Memory */

#if !defined(CONFIG_SAMV7_PROGMEM) || !defined(CONFIG_MTD_PROGMEM)
#  undef HAVE_PROGMEM_CHARDEV
#endif

/* This is the on-chip progmem memory driver minor number */

#define PROGMEM_MTD_MINOR 0

/* MMC/SD minor numbers */

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

#ifndef CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#endif

#if CONFIG_NSH_MMCSDSLOTNO != 0
#  error SAMV71 has only one MMC/SD slot (CONFIG_NSH_MMCSDSLOTNO)
#  undef CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#endif

#define HSMCI0_SLOTNO CONFIG_NSH_MMCSDSLOTNO
#define HSMCI0_MINOR  CONFIG_NSH_MMCSDMINOR

/* Automounter.  Currently only works with HSMCI. */

#if !defined(CONFIG_FS_AUTOMOUNTER) || !defined(HAVE_HSMCI)
#  undef HAVE_AUTOMOUNTER
#  undef CONFIG_SAMV7_HSMCI0_AUTOMOUNT
#endif

#ifndef CONFIG_SAMV7_HSMCI0_AUTOMOUNT
#  undef HAVE_AUTOMOUNTER
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define SAMV71_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define SAMV71_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Do we need to install the LED driver */

#if defined(CONFIG_ARCH_LEDS) || !defined(CONFIG_USERLED) || \
    !defined(CONFIG_USERLED_LOWER)
#  undef HAVE_LED_DRIVER
#endif

#ifdef HAVE_LED_DRIVER
#  ifdef CONFIG_EXAMPLES_LEDS_DEVPATH
#    define LED_DRIVER_PATH CONFIG_EXAMPLES_LEDS_DEVPATH
#  else
#    define LED_DRIVER_PATH "/dev/userleds"
#  endif
#endif

/* Configuration ************************************************************/

/* PIC32CZ CA70 GPIO Pin Definitions ****************************************/

/* LEDs
 *
 * There is one green and one red LED on PIC32CZ CA70 Evaluation board. The
 * LEDs can be activated by driving the connected I/O line to GND.
 *
 *   ------ ----------
 *   PIC32  Functions
 *   GPIO
 *   ------ ----------
 *   PD23   Green LED0
 *   PD29   Red LED1
 *   ------ ----------
 */

#define GPIO_LED0     (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                       GPIO_PORT_PIOD | GPIO_PIN29)
#define GPIO_LED1     (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                       GPIO_PORT_PIOD | GPIO_PIN23)

/* Buttons
 *
 * PIC32CZ CA70 Curiosity contains three mechanical buttons. One button is
 * the RESET button connected to the PIC32CZ reset line and the others are
 * generic user configurable buttons. When a button is pressed it will drive
 * the I/O line to GND.
 *
 *   ------ -----------
 *   PIC32  Function
 *   GPIO
 *   ------ -----------
 *   RESET  RESET
 *   PA01   SW0
 *   PA09   SW1
 *   ------ -----------
 */

#define GPIO_SW0      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                       GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN1)
#define GPIO_SW1      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                       GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN9)

#define IRQ_SW0       SAM_IRQ_PA1
#define IRQ_SW1       SAM_IRQ_PA9

/* HSMCI SD Card Detect
 *
 * PIC32CZ CA70 Curiosity kit has one micro SD card connector which is
 * connected to the High Speed Multimedia Card Interface (HSMCI) of the MCU.
 * SD card connector:
 *
 *   ------ -----------------
 *   PIC32  Function
 *   GPIO
 *   ------ -----------------
 *   PA30   MCDA0 (DAT0)
 *   PA31   MCDA1 (DAT1)
 *   PA26   MCDA2 (DAT2)
 *   PA27   MCDA3 (DAT3)
 *   PA25   MCCK (CLK)
 *   PA28   MCCDA (CMD)
 *   PC16   Card Detect (C/D)
 *   ------ -----------------
 */

#define GPIO_HSMCI0_CD (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
                        GPIO_INT_BOTHEDGES | GPIO_PORT_PIOC | GPIO_PIN16)
#define IRQ_HSMCI0_CD   SAM_IRQ_PC16

/* USB Host
 *
 * The PIC32 CZ CA70 Curiosity board has a Micro-USB connector for use with
 * the USB module labeled as TARGET USB on the kit. In USB host mode VBUS
 * voltage is provided by the kit and has to be enabled by setting the
 * "VBUS Host Enable" pin (PC25) low.
 */

#define GPIO_VBUSON (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                     GPIO_PORT_PIOC | GPIO_PIN25)

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
 * Name:  sam_usbinitialize
 *
 * Description:
 *   Called from stm32_boardinitialize very early in initialization to setup
 *   USB- related GPIO pins for the SAMV71-XULT board.
 *
 ****************************************************************************/

#ifdef HAVE_USB
void sam_usbinitialize(void);
#endif

/****************************************************************************
 * Name: sam_sdcard_initialize
 *
 * Description:
 *  Initialize SD Card (HSMCI driver) and performs mount if selected.
 *
 ****************************************************************************/

#ifdef HAVE_HSMCI
int sam_sdcard_initialize(void);
#endif

/****************************************************************************
 * Name: sam_flash_init
 *
 * Description:
 *   Initialize the embedded SAME70 FLASH programming memory.
 *
 ****************************************************************************/

#ifdef HAVE_PROGMEM_CHARDEV
int sam_flash_init(void);
#endif

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int sam_bringup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMV7_PIC32CZCA70_CURIOSITY_SRC_SAM_BOARD_H */
