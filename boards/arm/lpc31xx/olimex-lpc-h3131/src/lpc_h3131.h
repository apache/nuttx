/****************************************************************************
 * boards/arm/lpc31xx/olimex-lpc-h3131/src/lpc_h3131.h
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

#ifndef __BOARDS_ARM_LPC31XX_OLIMEX_LPC_H3131_SRC_LPC_H3131_H
#define __BOARDS_ARM_LPC31XX_OLIMEX_LPC_H3131_SRC_LPC_H3131_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "lpc31_ioconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PORT and SLOT number probably depend on the board configuration */

#define HAVE_MMCSD      1
#undef  HAVE_USBDEV
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1

/* Can't support MMC/SD features if mountpoints are disabled or if
 * SDIO support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_LPC31_MCI)
#  undef HAVE_MMCSD
#endif

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

/* Can't support USB host features if USB host is not enabled */

#if !defined(CONFIG_LPC31_USBOTG) || !defined(CONFIG_USBHOST)
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

/* LPC-H3131 GPIOs **********************************************************/

/* BUTTONS.  There are no user accessible buttons on the LPC-H3131 */

/* LEDs
 *
 * SIGNAL COLOR  GPIO   ILLUMINATION
 * ------ ------ ------ -----------------------
 * LED1   Yellow GPIO17 High output illuminates
 * LED2   Green  GPIO18 High output illuminates
 */

#define GPIO_LED1       IOCONFIG_GPIO_GPIO17
#define GPIO_LED2       IOCONFIG_GPIO_GPIO18

/* USB HOST
 *
 * SIGNAL      GPIO
 * ----------- -------
 * #OTG_PWR_E  GPIO19
 * #OTG_OVRCR  GPIO20
 */

#define GPIO_NOTG_PWR_E IOCONFIG_GPIO_GPIO19
#define GPIO_NOTG_OVRCR IOCONFIG_GPIO_GPIO20

/* SPI Chip Selects */

/* SPI NOR flash is the only device on SPI. SPI_CS_OUT0 is its chip select */

#define SPINOR_CS IOCONFIG_SPI_CSOUT0

/* USB Soft Connect Pullup -- NONE */

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
 * Name: lpc31_meminitialize
 *
 * Description:
 *   Initialize external memory resources (sram, sdram, nand, nor, etc.)
 *
 ****************************************************************************/

#ifdef CONFIG_LPC31_EXTDRAM
void lpc31_meminitialize(void);
#endif

/****************************************************************************
 * Name: lpc31_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LPC-H3131 board.
 *
 ****************************************************************************/

void weak_function lpc31_spidev_initialize(void);

/****************************************************************************
 * Name: lpc31_usbdev_initialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the LPC-H3131 board.
 *
 ****************************************************************************/

#ifdef HAVE_USBDEV
void weak_function lpc31_usbdev_initialize(void);
#endif

/****************************************************************************
 * Name: lpc31_usbhost_bootinitialize
 *
 * Description:
 *   Called from lpc31_boardinitialize very early in inialization to setup
 *   USB host-related GPIO pins for the LPC-H3131 board.
 *
 ****************************************************************************/

#ifdef HAVE_USBHOST
void weak_function lpc31_usbhost_bootinitialize(void);
#endif

/****************************************************************************
 * Name: lpc31_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ****************************************************************************/

#ifdef HAVE_USBHOST
int lpc31_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: lpc31_mmcsd_initialize
 *
 * Description:
 *   Create the SDIO-based MMC/SD device
 *
 ****************************************************************************/

#ifdef HAVE_MMCSD
int lpc31_mmcsd_initialize(int slot, int minor)
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LPC31XX_OLIMEX_LPC_H3131_SRC_LPC_H3131_H */
