/****************************************************************************
 * boards/arm/sam34/sam4s-xplained-pro/src/sam4s-xplained-pro.h
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

#ifndef __BOARDS_ARM_SAM34_SAM4S_XPLAINED_PRO_SRC_SAM4S_XPLAINED_PRO_H
#define __BOARDS_ARM_SAM34_SAM4S_XPLAINED_PRO_SRC_SAM4S_XPLAINED_PRO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "hardware/sam_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_HSMCI      1
#define HAVE_PROC       1
#define HAVE_USBDEV     1

#if defined(CONFIG_MTD_NAND) && defined(CONFIG_SAM34_EXTNAND)
#define HAVE_NAND       1
#endif
#undef  HAVE_USBMONITOR

#if defined(CONFIG_MMCSD_SPI)
#  define HAVE_MMCSD_SPI 1
#endif

/* HSMCI */

/* Can't support MMC/SD if the card interface is not enabled */

#if !defined(CONFIG_SAM34_HSMCI)
#  undef HAVE_HSMCI
#endif

#if !defined(CONFIG_FS_PROCFS)
#  undef HAVE_PROC
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_HSMCI) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No MMC/SD support
#  undef HAVE_HSMCI
#endif

#if defined(HAVE_PROC) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No procfs support
#  undef HAVE_PROC
#endif

/* We need PIO interrupts on PIOC to support card detect interrupts */

#if defined(HAVE_HSMCI) && !defined(CONFIG_SAM34_GPIOC_IRQ)
#  warning PIOC interrupts not enabled.  No MMC/SD support.
#  undef HAVE_HSMCI
#endif

/* MMC/SD minor numbers */

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

#ifndef CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#endif

/* USB Device */

/* CONFIG_SAM34_UDP and CONFIG_USBDEV must be defined, or there is no USB
 * device.
 */

#if !defined(CONFIG_SAM34_UDP) || !defined(CONFIG_USBDEV) ||!defined(CONFIG_CDCACM)
#  undef HAVE_USBDEV
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#if !defined(CONFIG_USBMONITOR) && !defined(CONFIG_USBDEV_TRACE)
#  undef HAVE_USBMONITOR
#endif

/* There is one LED on board the SAM4S Xplained board Pro that can be
 * controlled by software in the SAM4S:
 *
 *   LED              GPIO
 *   ---------------- -----
 *   LED0 Yellow LED   PC23
 *
 * It can be illuminated by driving the GPIO output to ground (low).
 *
 * If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on
 * board the SAM4S Xplained Pro, otherwise it can controlled by the user
 * with functions defined into boards file src/sam_userleds.c.
 *
 * The following definitions describe how NuttX
 * controls the LEDs:
 *
 *   SYMBOL                Meaning                   LED state
 *                                                   LED0
 *   -------------------  -----------------------  -----------
 *   LED_STARTED          NuttX has been started     OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF
 *   LED_STACKCREATED     Idle stack created         ON
 *   LED_INIRQ            In an interrupt            No change
 *   LED_SIGNAL           In a signal handler        No change
 *   LED_ASSERTION        An assertion failed        No change
 *   LED_PANIC            The system has crashed     OFF
 *   LED_IDLE             MCU is is sleep mode       Not used
 */

#define GPIO_D301    (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOC | GPIO_PIN23)

/* Mechanical buttons:
 *
 * The SAM4S Xplained Pro has two mechanical buttons.
 * One button is the RESET button connected to the SAM4S reset line and the
 * other is a generic user configurable button labeled SW0.
 * When a button is pressed it will drive the I/O line to GND.
 *
 *   PA2 SW0
 */

#define GPIO_SW0     (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                      GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN2)
#define IRQ_SW0      SAM_IRQ_PA2

/* HSMCI SD Card Detect PC12 */

#define GPIO_MCI_CD   (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_PORT_PIOC | GPIO_PIN12)
#define MCI_CD_IRQ    SAM_IRQ_PC12

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* SPI0 */

#ifdef HAVE_MMCSD_SPI
#define GPIO_SPISD_NPCS0 (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                        GPIO_PORT_PIOA | GPIO_PIN11)
#define SPISD_PORT      SPI0_CS0

#define GPIO_SPI_CD   (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_PORT_PIOC | GPIO_PIN19)
#define SD_SPI_IRQ    SAM_IRQ_PC19
#endif /* HAVE_MMCSD_SPI */

/* NAND */

#ifdef HAVE_NAND
#  define NAND_MINOR 0
#  define SAM_SMC_CS0 0 /* GPIO_SMC_NCS0  connect SAM_SMC_CS0_BASE */
int sam_nand_automount(int minor);
#endif /* HAVE_NAND */

/****************************************************************************
 * Name: sam_hsmci_initialize
 *
 * Description:
 *   Initialize HSMCI support
 *
 ****************************************************************************/

#ifdef HAVE_HSMCI
int sam_hsmci_initialize(void);
#else
#  define sam_hsmci_initialize()
#endif

int sam_sdinitialize(int port, int minor);

/****************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

#if defined(HAVE_HSMCI) && defined(CONFIG_MMCSD_HAVE_CARDDETECT)
bool sam_cardinserted(int slotno);
#endif

/****************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if the card in the MMCSD slot is write protected
 *
 ****************************************************************************/

#ifdef HAVE_HSMCI
bool sam_writeprotected(int slotno);
#else
#  define sam_writeprotected(slotno) (false)
#endif

/****************************************************************************
 * Name: sam_watchdog_initialize()
 *
 * Description:
 *   Perform architecture-specific initialization of the Watchdog hardware.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sam_watchdog_initialize(void);

#endif /* __BOARDS_ARM_SAM34_SAM4S_XPLAINED_PRO_SRC_SAM4S_XPLAINED_PRO_H */
