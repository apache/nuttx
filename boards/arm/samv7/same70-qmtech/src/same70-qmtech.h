/****************************************************************************
 * boards/arm/samv7/same70-qmtech/src/same70-qmtech.h
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

#ifndef __BOARDS_ARM_SAMV7_SAME70_QMTECH_SRC_SAME70_QMTECH_H
#define __BOARDS_ARM_SAMV7_SAME70_QMTECH_SRC_SAME70_QMTECH_H

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

/* Configuration ************************************************************/

#define HAVE_HSMCI           1
#define HAVE_AUTOMOUNTER     1
#define HAVE_PROGMEM_CHARDEV 1

/* HSMCI */

/* Can't support MMC/SD if the card interface is not enabled */

#if !defined(CONFIG_SAMV7_HSMCI0)
#  undef HAVE_HSMCI
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_HSMCI) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No MMC/SD support
#  undef HAVE_HSMCI
#endif

/* We need PIO interrupts on GPIOD to support card detect interrupts */

#if defined(HAVE_HSMCI) && !defined(CONFIG_SAMV7_GPIOD_IRQ)
#  warning PIOD interrupts not enabled.  No MMC/SD support.
#  undef HAVE_HSMCI
#endif

/* MMC/SD minor numbers */

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

#ifndef CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#endif

#if CONFIG_NSH_MMCSDSLOTNO != 0
#  error SAME70 has only one MMC/SD slot (CONFIG_NSH_MMCSDSLOTNO)
#  undef CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#endif

#define HSMCI0_SLOTNO CONFIG_NSH_MMCSDSLOTNO
#define HSMCI0_MINOR  CONFIG_NSH_MMCSDMINOR

/* Automounter.  Currently only works with HSMCI. */

#if !defined(CONFIG_FS_AUTOMOUNTER) || !defined(HAVE_HSMCI)
#  undef HAVE_AUTOMOUNTER
#  undef CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT
#endif

#ifndef CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT
#  undef HAVE_AUTOMOUNTER
#endif

#ifdef HAVE_AUTOMOUNTER
#  ifdef CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT
  /* HSMCI0 Automounter defaults */

#    ifndef CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT_FSTYPE
#      define CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT_FSTYPE "vfat"
#    endif

#    ifndef CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT_BLKDEV
#      define CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT_BLKDEV "/dev/mmcds0"
#    endif

#    ifndef CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT_MOUNTPOINT
#      define CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT_MOUNTPOINT "/mnt/sdcard0"
#    endif

#    ifndef CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT_DDELAY
#      define CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT_DDELAY 1000
#    endif

#    ifndef CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT_UDELAY
#      define CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT_UDELAY 2000
#    endif
#  endif /* CONFIG_SAME70QMTECH_HSMCI0_AUTOMOUNT */
#endif /* HAVE_AUTOMOUNTER */

/* On-chip Programming Memory */

#if !defined(CONFIG_SAMV7_PROGMEM) || !defined(CONFIG_MTD_PROGMEM)
#  undef HAVE_PROGMEM_CHARDEV
#endif

/* This is the on-chip progmem memory driver minor number */

#define PROGMEM_MTD_MINOR 0

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define SAME70_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define SAME70_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* SAME70-QMTECH GPIO Pin Definitions ***************************************/

/* LEDs
 *
 * A single LED is available driven by PA15.
 */

#define GPIO_LED0     (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                       GPIO_PORT_PIOA | GPIO_PIN15)

/* Buttons
 *
 * SAM E70 QMTECH contains two mechanical buttons. One button is the RESET
 * button connected to the SAM E70 reset line and the other, PA11, is a
 * generic user configurable button.
 * When a button is pressed it will drive the I/O line to GND.
 *
 * NOTE: There are no pull-up resistors connected to the generic user buttons
 * so it is necessary to enable the internal pull-up in the SAM E70 to use
 * the button.
 */

#define GPIO_SW0      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                       GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN21)
#define IRQ_SW0       SAM_IRQ_PA21

/* HSMCI SD Card Detect
 *
 * The SAM E70 QMTECH has one standard SD card connector that is connected
 * to the High Speed Multimedia Card Interface (HSMCI) of the SAM E70. SD
 * card connector:
 *
 *   ------ ----------------- ---------------------
 *   SAME70 SAME70            Shared functionality
 *   Pin    Function
 *   ------ ----------------- ---------------------
 *   PA30   MCDA0 (DAT0)
 *   PA31   MCDA1 (DAT1)
 *   PA26   MCDA2 (DAT2)
 *   PA27   MCDA3 (DAT3)      Camera
 *   PA25   MCCK (CLK)        Shield
 *   PA28   MCCDA (CMD)
 *   PD17   Card Detect (C/D) Shield
 *   ------ ----------------- ---------------------
 */

#define GPIO_MCI0_CD (GPIO_INPUT | GPIO_CFG_PULLDOWN | GPIO_CFG_DEGLITCH | \
                      GPIO_INT_BOTHEDGES | GPIO_PORT_PIOD | GPIO_PIN17)
#define IRQ_MCI0_CD   SAM_IRQ_PD17

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
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int sam_bringup(void);
#endif

/****************************************************************************
 * Name: sam_afec_initialize
 *
 * Description:
 *   Initialize and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_AFEC
int sam_afec_setup(void);
#endif

/****************************************************************************
 * Name: sam_dacdev_initialize
 *
 * Description:
 *   Called to configure DAC peripheral module
 *
 ****************************************************************************/

#if defined(CONFIG_SAMV7_DAC0) || defined(CONFIG_SAMV7_DAC1)
int sam_dacdev_initialize(void);
#endif

/****************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAME70-QMTECH
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_SPI
void sam_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: sam_hsmci_initialize
 *
 * Description:
 *   Initialize HSMCI support
 *
 ****************************************************************************/

#ifdef HAVE_HSMCI
int sam_hsmci_initialize(int slot, int minor);
#else
# define sam_hsmci_initialize(s,m) (-ENOSYS)
#endif

/****************************************************************************
 * Name: sam_progmem_init
 *
 * Description:
 *   Initialize the FLASH and register the MTD device.
 ****************************************************************************/

#ifdef HAVE_PROGMEM_CHARDEV
int sam_progmem_init(void);
#endif

/****************************************************************************
 * Name: sam_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_MCAN
int sam_can_setup(void);
#endif

/****************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

#ifdef HAVE_HSMCI
bool sam_cardinserted(int slotno);
#else
#  define sam_cardinserted(slotno) (false)
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
#endif

/****************************************************************************
 * Name:  sam_automount_initialize
 *
 * Description:
 *   Configure auto-mounters for each enable and so configured HSMCI
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
void sam_automount_initialize(void);
#endif

/****************************************************************************
 * Name:  sam_automount_event
 *
 * Description:
 *   The HSMCI card detection logic has detected an insertion or removal
 *   event.  It has already scheduled the MMC/SD block driver operations.
 *   Now we need to schedule the auto-mount event which will occur with a
 *   substantial delay to make sure that everything has settle down.
 *
 * Input Parameters:
 *   slotno - Identifies the HSMCI0 slot: HSMCI0 or HSMCI1_SLOTNO.
 *       There is a terminology problem here:  Each HSMCI supports two slots,
 *      slot A and slot B. Only slot A is used.  So this is not a really a
 *      slot, but an HSCMI peripheral number.
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
void sam_automount_event(int slotno, bool inserted);
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

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMV7_SAME70_QMTECH_SRC_SAME70_QMTECH_H */
