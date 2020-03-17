/****************************************************************************
 *  boards/arm/sama5/sama5d2-xult/src/sama5d2-xult.h
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_SAMA5_SAMA5D2_XULT_SRC_SAMA5D2_XULT_H
#define __BOARDS_ARM_SAMA5_SAMA5D2_XULT_SRC_SAMA5D2_XULT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "hardware/sam_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_HSMCI      1
#define HAVE_AT25       1
#define HAVE_NAND       1
#define HAVE_USBHOST    1
#define HAVE_USBDEV     1
#define HAVE_USBMONITOR 1
#define HAVE_NETWORK    1

/* HSMCI */

/* Can't support MMC/SD if the card interface(s) are not enable */

#if !defined(CONFIG_SAMA5_HSMCI0) && !defined(CONFIG_SAMA5_HSMCI1)
#  undef HAVE_HSMCI
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_HSMCI) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No MMC/SD support
#  undef HAVE_HSMCI
#endif

/* We need PIO interrupts on PIOD to support card detect interrupts */

#if defined(HAVE_HSMCI) && !defined(CONFIG_SAMA5_PIOD_IRQ)
#  warning PIOD interrupts not enabled.  No MMC/SD support.
#  undef HAVE_HSMCI
#endif

/* NAND FLASH */

/* Can't support the NAND device if NAND flash is not configured on EBI CS3 */

#ifndef CONFIG_SAMA5_EBICS3_NAND
#  undef HAVE_NAND
#endif

/* Can't support NAND features if mountpoints are disabled or if we were not
 * asked to mount the NAND part
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAMA5D3XPLAINED_NAND_BLOCKMOUNT)
#  undef HAVE_NAND
#endif

/* Can't support NAND if the MTD feature is not enabled */

#if !defined(CONFIG_MTD) || !defined(CONFIG_MTD_NAND)
#  undef HAVE_NAND
#endif

/* If we are going to mount the NAND, then they user must also have told
 * us what to do with it by setting one of CONFIG_SAMA5D3XPLAINED_NAND_FTL or
 * CONFIG_SAMA5D3XPLAINED_NAND_NXFFS.
 */

#ifndef CONFIG_MTD
#  undef CONFIG_SAMA5D3XPLAINED_NAND_NXFFS
#  undef CONFIG_SAMA5D3XPLAINED_NAND_FTL
#endif

#if !defined(CONFIG_FS_NXFFS) || !defined(CONFIG_NXFFS_NAND)
#  undef CONFIG_SAMA5D3XPLAINED_NAND_NXFFS
#endif

#if !defined(CONFIG_SAMA5D3XPLAINED_NAND_FTL) && !defined(CONFIG_SAMA5D3XPLAINED_NAND_NXFFS)
#  undef HAVE_NAND
#endif

#if defined(CONFIG_SAMA5D3XPLAINED_NAND_FTL) && defined(CONFIG_SAMA5D3XPLAINED_NAND_NXFFS)
#  warning Both CONFIG_SAMA5D3XPLAINED_NAND_FTL and CONFIG_SAMA5D3XPLAINED_NAND_NXFFS are set
#  warning Ignoring CONFIG_SAMA5D3XPLAINED_NAND_NXFFS
#  undef CONFIG_SAMA5D3XPLAINED_NAND_NXFFS
#endif

/* AT25 Serial FLASH */

/* Can't support the AT25 device if it SPI0 or AT25 support are not enabled */

#if !defined(CONFIG_SAMA5_SPI0) || !defined(CONFIG_MTD_AT25)
#  undef HAVE_AT25
#endif

/* Can't support AT25 features if mountpoints are disabled or if we were not
 * asked to mount the AT25 part
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAMA5D3XPLAINED_AT25_AUTOMOUNT)
#  undef HAVE_AT25
#endif

/* If we are going to mount the AT25, then they user must also have told
 * us what to do with it by setting one of these.
 */

#ifndef CONFIG_FS_NXFFS
#  undef CONFIG_SAMA5D3XPLAINED_AT25_NXFFS
#endif

#if !defined(CONFIG_SAMA5D3XPLAINED_AT25_FTL) && !defined(CONFIG_SAMA5D3XPLAINED_AT25_NXFFS)
#  undef HAVE_AT25
#endif

#if defined(CONFIG_SAMA5D3XPLAINED_AT25_FTL) && defined(CONFIG_SAMA5D3XPLAINED_AT25_NXFFS)
#  warning Both CONFIG_SAMA5D3XPLAINED_AT25_FTL and CONFIG_SAMA5D3XPLAINED_AT25_NXFFS are set
#  warning Ignoring CONFIG_SAMA5D3XPLAINED_AT25_NXFFS
#  undef CONFIG_SAMA5D3XPLAINED_AT25_NXFFS
#endif

/* Assign minor device numbers.  For example, if we also use MINOR number 0
 * for the AT25, it should appear as /dev/mtdblock0
 */

#define _NAND_MINOR 0

#ifdef HAVE_NAND
#  define NAND_MINOR  _NAND_MINOR
#  define _AT25_MINOR (_NAND_MINOR+1)
#else
#  define _AT25_MINOR _NAND_MINOR
#endif

#ifdef HAVE_AT25
#  define AT25_MINOR  _AT25_MINOR
#endif

/* MMC/SD minor numbers:  The NSH device minor extended is extended to
 * support two devices.  If CONFIG_NSH_MMCSDMINOR is zero, these will be:
 * /dev/mmcsd0 and /dev/mmcsd1.
 */

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

#ifdef HAVE_HSMCI

#  define HSMCI0_SLOTNO 0
#  define HSMCI1_SLOTNO 1

#  ifdef CONFIG_SAMA5_HSMCI0
#     define HSMCI0_MINOR  CONFIG_NSH_MMCSDMINOR
#     define HSMCI1_MINOR  (CONFIG_NSH_MMCSDMINOR+1)
#  else
#     define HSMCI1_MINOR  CONFIG_NSH_MMCSDMINOR
#  endif
#else
#endif

/* USB Host / USB Device */

/* Either CONFIG_SAMA5_UHPHS or CONFIG_SAMA5_UDPHS must be defined,
 * or there is no USB of any kind.
 */

#if !defined(CONFIG_SAMA5_UHPHS)
#  undef CONFIG_SAMA5_OHCI
#  undef CONFIG_SAMA5_EHCI
#endif

#if !defined(CONFIG_SAMA5_UDPHS)
#  undef HAVE_USBDEV
#endif

/* CONFIG_USBDEV and CONFIG_USBHOST must also be defined */

#if !defined(CONFIG_USBDEV)
#  undef HAVE_USBDEV
#endif

#if defined(CONFIG_USBHOST)
#  if !defined(CONFIG_SAMA5_OHCI) && !defined(CONFIG_SAMA5_EHCI)
#    warning CONFIG_USBHOST is defined, but neither CONFIG_SAMA5_OHCI nor CONFIG_SAMA5_EHCI are defined
#  endif
#else
#  undef CONFIG_SAMA5_OHCI
#  undef CONFIG_SAMA5_EHCI
#endif

#if !defined(CONFIG_SAMA5_OHCI) && !defined(CONFIG_SAMA5_EHCI)
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

/* Networking */

#if !defined(CONFIG_NET) || (!defined(CONFIG_SAMA5_EMACA) && !defined(CONFIG_SAMA5_GMAC))
#  undef HAVE_NETWORK
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define SAMA5_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define SAMA5_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* LEDs *********************************************************************/

/* There is an RGB LED on board the SAMA5D2-XULT.
 * The RED component is driven by the SDHC_CD pin (PA13) and so will not
 * be used.
 * The LEDs are provided VDD_LED and so bringing the LED low will illuminate
 * the LED.
 *
 *   ------------------------------ ------------------- ---------------------
 *   SAMA5D2 PIO                    SIGNAL              USAGE
 *   ------------------------------ ------------------- ---------------------
 *   PA13                           SDHC_CD_PA13        Red LED
 *   PB5                            LED_GREEN_PB5       Green LED
 *   PB0                            LED_BLUE_PB0        Blue LED
 *   ------------------------------ ------------------- ---------------------
 */

#define PIO_LED_GREEN (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_SET | \
                       PIO_PORT_PIOB | PIO_PIN5)
#define PIO_LED_BLUE  (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_SET | \
                       PIO_PORT_PIOB | PIO_PIN0)

/* Buttons ******************************************************************/

/* A single button, PB_USER (PB6), is available on the SAMA5D2-XULT
 *
 *  ------------------------------ ------------------- ----------------------
 *  SAMA5D2 PIO                    SIGNAL              USAGE
 *  ------------------------------ ------------------- ----------------------
 *  PB6                            USER_PB_PB6         PB_USER push button
 *  ------------------------------ ------------------- ----------------------
 *
 *  Closing PB_USER will bring PB6 to ground so 1) PB6 should have a weak
 *  pull-up, and 2) when PB_USER is pressed, a low value will be senses.
 */

#define PIO_BTN_USER (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOB | PIO_PIN6)
#define IRQ_BTN_USER  SAM_IRQ_PB6

/* HSMCI Card Slots *********************************************************/

/* The SAMA5D2-XULT provides a SD memory card slots:
 *  a full size SD card slot (J19)
 *
 * The full size SD card slot connects via HSMCI0.  The card detect discrete
 * is available on PD17 (pulled high).  The write protect discrete is tied to
 * ground (via PP6) and not available to software.  The slot supports 8-bit
 * wide transfer mode, but the NuttX driver currently uses only the 4-bit
 * wide transfer mode
 *
 *   PD17 MCI0_CD
 *   PD1  MCI0_DA0
 *   PD2  MCI0_DA1
 *   PD3  MCI0_DA2
 *   PD4  MCI0_DA3
 *   PD5  MCI0_DA4
 *   PD6  MCI0_DA5
 *   PD7  MCI0_DA6
 *   PD8  MCI0_DA7
 *   PD9  MCI0_CK
 *   PD0  MCI0_CDA
 */

#define PIO_MCI0_CD  (PIO_INPUT | PIO_CFG_DEFAULT | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOA | PIO_PIN11)
#define IRQ_MCI0_CD   SAM_IRQ_PA11

/* USB Ports ****************************************************************/

/* The SAMA5D2-XULT features two USB communication ports:
 *
 *   1. Port A Host High Speed (EHCI) and Full Speed (OHCI) multiplexed with
 *      USB Device High Speed Micro AB connector, J23
 *
 *   2. Port B Host High Speed (EHCI) and Full Speed (OHCI) standard type A
 *      connector, J13
 *
 * The USB host port (only) is equipped with 500-mA high-side power
 * switch for self-powered and bus-powered applications.
 *
 * The USB device port A (J6) features a VBUS insert detection function.
 *
 *
 * Port A
 *
 *   PIO  Signal Name Function
 *   ---- ----------- -------------------------------------------------------
 *   PE9  VBUS_SENSE VBus detection
 *
 *     Note: No VBus power switch enable on port A.  I think that this limits
 *     this port to a device port or as a host port for self-powered devices
 *     only.
 */

#define PIO_USBA_VBUS_SENSE \
                     (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOA | PIO_PIN31)
#define IRQ_USBA_VBUS_SENSE \
                     SAM_IRQ_PA31

/* Port B
 *
 *   PIO  Signal Name Function
 *   ---- ----------- -------------------------------------------------------
 *   PE3  EN5V_USBB   VBus power enable via MN3 SP2526A-2E dual power
 *                    switch.  PE3 (EN5V_USBB)connects to ENB pin of MN3.
 *                    MN3 OUTB (5V_USBB) is provided to pin 1 of J13 USB
 *                    A connector
 *
 *                    Active high for SP2526A-1; active low for SP2526A-2
 */

#define PIO_USBB_VBUS_ENABLE \
                     (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_SET | \
                      PIO_PORT_PIOB | PIO_PIN10)

/*  Ports B
 *
 *   PIO  Signal Name Function
 *   ---- ----------- -------------------------------------------------------
 *   PE5  OVCUR_USB   Over-current indication from B
 */

#define PIO_USBB_VBUS_OVERCURRENT \
                     (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOA | PIO_PIN29)
#define IRQ_USBB_VBUS_OVERCURRENT \
                     SAM_IRQ_PA29

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int sam_bringup(void);

/****************************************************************************
 * Name: sam_usbinitialize
 *
 * Description:
 *   Called from sam_usbinitialize very early in initialization to setup
 *   USB-related PIO pins for the SAMA5D2-XULT board.
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_UHPHS) || defined(CONFIG_SAMA5_UDPHS)
void weak_function sam_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ****************************************************************************/

#ifdef HAVE_USBHOST
int sam_usbhost_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM__SAMA5SAMA5D2_XULT_SRC_SAMA5D2_XULT_H */
