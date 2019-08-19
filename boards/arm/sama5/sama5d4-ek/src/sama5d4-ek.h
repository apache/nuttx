/****************************************************************************
 * boards/arm/sama5/sama5d4-ek/src/sama5d4-ek.h
 *
 *   Copyright (C) 2014-2016, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

#ifndef __BOARDS_ARM_SAMA5_SAMA5D4_EK_SRC_SAMA5D4_EK_H
#define __BOARDS_ARM_SAMA5_SAMA5D4_EK_SRC_SAMA5D4_EK_H

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

#define HAVE_HSMCI       1
#define HAVE_AT25        1
#define HAVE_NAND        1
#define HAVE_AUTOMOUNTER 1
#define HAVE_USBHOST     1
#define HAVE_USBDEV      1
#define HAVE_USBOVCUR    1
#define HAVE_USBMONITOR  1
#define HAVE_NETWORK     1
#define HAVE_MAXTOUCH    1
#define HAVE_WM8904      1
#define HAVE_AUDIO_NULL  1
#define HAVE_PMIC        1
#define HAVE_ROMFS       1
#define HAVE_I2CTOOL     1

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

/* We need PIO interrupts on PIOE to support card detect interrupts */

#if defined(HAVE_HSMCI) && !defined(CONFIG_SAMA5_PIOE_IRQ)
#  warning PIOE interrupts not enabled.  No MMC/SD support.
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

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAMA5D4EK_NAND_BLOCKMOUNT)
#  undef HAVE_NAND
#endif

/* Can't support NAND if the MTD feature is not enabled */

#if !defined(CONFIG_MTD) || !defined(CONFIG_MTD_NAND)
#  undef HAVE_NAND
#endif

/* If we are going to mount the NAND, then they user must also have told
 * us what to do with it by setting one of CONFIG_SAMA5D4EK_NAND_FTL or
 * CONFIG_SAMA5D4EK_NAND_NXFFS.
 */

#ifndef CONFIG_MTD
#  undef CONFIG_SAMA5D4EK_NAND_NXFFS
#  undef CONFIG_SAMA5D4EK_NAND_FTL
#endif

#if !defined(CONFIG_FS_NXFFS) || !defined(CONFIG_NXFFS_NAND)
#  undef CONFIG_SAMA5D4EK_NAND_NXFFS
#endif

#if !defined(CONFIG_SAMA5D4EK_NAND_FTL) && !defined(CONFIG_SAMA5D4EK_NAND_NXFFS)
#  undef HAVE_NAND
#endif

#if defined(CONFIG_SAMA5D4EK_NAND_FTL) && defined(CONFIG_SAMA5D4EK_NAND_NXFFS)
#  warning Both CONFIG_SAMA5D4EK_NAND_FTL and CONFIG_SAMA5D4EK_NAND_NXFFS are set
#  warning Ignoring CONFIG_SAMA5D4EK_NAND_NXFFS
#  undef CONFIG_SAMA5D4EK_NAND_NXFFS
#endif

/* AT25 Serial FLASH */

/* Can't support the AT25 device if it SPI0 or AT25 support are not enabled */

#if !defined(CONFIG_SAMA5_SPI0) || !defined(CONFIG_MTD_AT25)
#  undef HAVE_AT25
#endif

/* Can't support AT25 features if mountpoints are disabled or if we were not
 * asked to mount the AT25 part
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAMA5D4EK_AT25_BLOCKMOUNT)
#  undef HAVE_AT25
#endif

/* If we are going to mount the AT25, then they user must also have told
 * us what to do with it by setting one of these.
 */

#ifndef CONFIG_FS_NXFFS
#  undef CONFIG_SAMA5D4EK_AT25_NXFFS
#endif

#if !defined(CONFIG_SAMA5D4EK_AT25_FTL) && !defined(CONFIG_SAMA5D4EK_AT25_CHARDEV) && \
    !defined(CONFIG_SAMA5D4EK_AT25_NXFFS)
#  undef HAVE_AT25
#endif

#if defined(CONFIG_SAMA5D4EK_AT25_FTL) && defined(CONFIG_SAMA5D4EK_AT25_CHARDEV)
#  warning Both CONFIG_SAMA5D4EK_AT25_CHARDEV and CONFIG_SAMA5D4EK_AT25_FTL are set
#  warning Ignoring CONFIG_SAMA5D4EK_AT25_FTL
#  undef CONFIG_SAMA5D4EK_AT25_FTL
#endif

#if defined(CONFIG_SAMA5D4EK_AT25_FTL) && defined(CONFIG_SAMA5D4EK_AT25_NXFFS)
#  warning Both CONFIG_SAMA5D4EK_AT25_FTL and CONFIG_SAMA5D4EK_AT25_NXFFS are set
#  warning Ignoring CONFIG_SAMA5D4EK_AT25_NXFFS
#  undef CONFIG_SAMA5D4EK_AT25_NXFFS
#endif

#if defined(CONFIG_SAMA5D4EK_AT25_CHARDEV) && defined(CONFIG_SAMA5D4EK_AT25_NXFFS)
#  warning Both CONFIG_SAMA5D4EK_AT25_CHARDEV and CONFIG_SAMA5D4EK_AT25_NXFFS are set
#  warning Ignoring CONFIG_SAMA5D4EK_AT25_NXFFS
#  undef CONFIG_SAMA5D4EK_AT25_NXFFS
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

/* MMC/SD minor numbers:  The NSH device minor extended is extended to support
 * two devices.  If CONFIG_NSH_MMCSDMINOR is zero, these will be:  /dev/mmcsd0
 * and /dev/mmcsd1.
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

/* Automounter.  Currently only works with HSMCI. */

#if !defined(CONFIG_FS_AUTOMOUNTER) || !defined(HAVE_HSMCI)
#  undef HAVE_AUTOMOUNTER
#endif

#ifndef CONFIG_SAMA5_HSMCI0
#  undef CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT
#endif

#ifndef CONFIG_SAMA5_HSMCI1
#  undef CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT
#endif

#if !defined(CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT) && \
    !defined(CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT)
#  undef HAVE_AUTOMOUNTER
#endif

#ifdef HAVE_AUTOMOUNTER
#  ifdef CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT
  /* HSMCI0 Automounter defaults */

#    ifndef CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_FSTYPE
#      define CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_FSTYPE "vfat"
#    endif

#    ifndef CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_BLKDEV
#      define CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_BLKDEV "/dev/mmcds0"
#    endif

#    ifndef CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_MOUNTPOINT
#      define CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_MOUNTPOINT "/mnt/sdcard0"
#    endif

#    ifndef CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_DDELAY
#      define CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_DDELAY 1000
#    endif

#    ifndef CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_UDELAY
#      define CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_UDELAY 2000
#    endif
#  endif

#  ifdef CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT
  /* HSMCI1 Automounter defaults */

#    ifndef CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_FSTYPE
#      define CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_FSTYPE "vfat"
#    endif

#    ifndef CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_BLKDEV
#      define CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_BLKDEV "/dev/mmcds0"
#    endif

#    ifndef CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_MOUNTPOINT
#      define CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_MOUNTPOINT "/mnt/sdcard0"
#    endif

#    ifndef CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_DDELAY
#      define CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_DDELAY 1000
#    endif

#    ifndef CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_UDELAY
#      define CONFIG_SAMA5D4EK_HSMCI1_AUTOMOUNT_UDELAY 2000
#    endif
#  endif
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

#if defined(HAVE_USBHOST) && !defined(CONFIG_SAMA5_UHPHS_RHPORT1) && \
    !defined(CONFIG_SAMA5_UHPHS_RHPORT2) && !defined(CONFIG_SAMA5_UHPHS_RHPORT3)
#  undef HAVE_USBHOST
#  warning No ports defined for USB host
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_SAMA5_UHPHS_RHPORT1
#  undef CONFIG_SAMA5_UHPHS_RHPORT2
#  undef CONFIG_SAMA5_UHPHS_RHPORT3
#endif

/* No overcurrent support if no USB host or no interrupts of PIOD */

#if !defined(HAVE_USBHOST)
#  undef HAVE_USBOVCUR
#endif

#if defined(HAVE_USBHOST) && !defined(CONFIG_SAMA5_PIOE_IRQ)
#  undef HAVE_USBOVCUR
#  warning CONFIG_SAMA5_PIOE_IRQ need for USB host overcurrent support
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

#if !defined(CONFIG_NET) || !defined(CONFIG_SAMA5_EMACB)
#  undef HAVE_NETWORK
#endif

/* maXTouch controller */

#ifndef CONFIG_INPUT_MXT
#  undef HAVE_MAXTOUCH
#endif

#ifdef HAVE_MAXTOUCH
#  ifndef CONFIG_SAMA5_TWI0
#    warning CONFIG_SAMA5_TWI0 is required for touchscreen support
#    undef HAVE_MAXTOUCH
#  endif

#  ifndef CONFIG_SAMA5_PIOE_IRQ
#    warning PIOE interrupts not enabled.  No touchsreen support.
#    undef HAVE_MAXTOUCH
#  endif
#endif

/* Audio */

/* PCM/WM8904 driver */

#ifndef CONFIG_AUDIO_WM8904
#  undef HAVE_WM8904
#endif

#ifdef HAVE_WM8904
#  ifdef CONFIG_SAMA5D4_MB_REVC
#    warning WM8904 should not be used with the Rev C. board
#  endif

#  ifndef CONFIG_SAMA5_TWI0
#    warning CONFIG_SAMA5_TWI0 is required for audio support
#    undef HAVE_WM8904
#  endif

#  ifndef CONFIG_SAMA5_SSC0
#    warning CONFIG_SAMA5_SSC0 is required for audio support
#    undef HAVE_WM8904
#  endif

#  if !defined(CONFIG_SAMA5_PIOE_IRQ)
#    warning CONFIG_SAMA5_PIOE_IRQ is required for audio support
#    undef HAVE_WM8904
#  endif

#  ifndef CONFIG_AUDIO_FORMAT_PCM
#    warning CONFIG_AUDIO_FORMAT_PCM is required for audio support
#    undef HAVE_WM8904
#  endif

#  ifndef CONFIG_SAMA5D4EK_WM8904_I2CFREQUENCY
#    warning Defaulting to maximum WM8904 I2C frequency
#    define CONFIG_SAMA5D4EK_WM8904_I2CFREQUENCY 400000
#  endif

#  if CONFIG_SAMA5D4EK_WM8904_I2CFREQUENCY > 400000
#    warning WM8904 I2C frequency cannot exceed 400KHz
#    undef CONFIG_SAMA5D4EK_WM8904_I2CFREQUENCY
#    define CONFIG_SAMA5D4EK_WM8904_I2CFREQUENCY 400000
#  endif
#endif

/* PCM/null driver */

#ifndef CONFIG_AUDIO_NULL
#  undef HAVE_AUDIO_NULL
#endif

#ifdef HAVE_WM8904
#  undef HAVE_AUDIO_NULL
#endif

#ifdef HAVE_AUDIO_NULL
#  ifndef CONFIG_AUDIO_FORMAT_PCM
#    warning CONFIG_AUDIO_FORMAT_PCM is required for audio support
#    undef HAVE_AUDIO_NULL
#  endif
#endif

/* PMIC */

#if !defined(CONFIG_SAMA5_TWI0) || !defined(CONFIG_SAMA5D4_MB_REVC)
#  undef HAVE_PMIC
#endif

#ifndef CONFIG_EXPERIMENTAL
#  undef HAVE_PMIC /* REVISIT: Disable anyway because it does not yet work */
#endif

/* ROMFS */

#ifndef CONFIG_FS_ROMFS
#  undef HAVE_ROMFS
#endif

#ifndef CONFIG_SAMA5D4EK_ROMFS_MOUNT
#  undef HAVE_ROMFS
#endif

/* Do we need to register I2C drivers on behalf of the I2C tool? */

#if !defined(CONFIG_SYSTEM_I2CTOOL) || !defined(CONFIG_I2C_DRIVER)
#  undef HAVE_I2CTOOL
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

/* There are 3 LEDs on the SAMA5D4-EK:
 *
 * ------------------------------ ------------------- -----------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -----------------------
 * PE28/NWAIT/RTS4/A19            1Wire_PE28          1-WIRE ROM, LCD, D8 (green)
 * PE8/A8/TCLK3/PWML3             LED_USER_PE8        LED_USER (D10)
 * PE9/A9/TIOA2                   LED_POWER_PE9       LED_POWER (D9, Red)
 * ------------------------------ ------------------- -----------------------
 *
 * - D8: D8 is shared with other functions and cannot be used if the
 *   1-Wire ROM is used.
 *   I am not sure of the LCD function, but the LED may not be available
 *   if the LCD is used either.  We will avoid using D8 just for simplicity.
 * - D10:  Nothing special here.  A low output illuminates.
 * - D9: The Power ON LED.  Connects to the via an IRLML2502 MOSFET.
 *   This LED will be on when power is applied but otherwise;
 *   a low output value will turn it off.
 */

#define PIO_LED_USER  (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_SET | \
                       PIO_PORT_PIOE | PIO_PIN8)
#define PIO_LED_POWER (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_CLEAR | \
                       PIO_PORT_PIOE | PIO_PIN9)

/* Buttons ******************************************************************/

/* A single button, PB_USER1 (PB2), is available on the SAMA5D4-EK:
 *
 * ------------------------------ ------------------- -----------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -----------------------
 * PE13/A13/TIOB1/PWML2           PB_USER1_PE13       PB_USER1
 * ------------------------------ ------------------- -----------------------
 *
 * Closing JP2 will bring PE13 to ground so
 * 1) PE13 should have a weak pull-up, and
 * 2) when PB2 is pressed, a low value will be senses.
 */

#define PIO_BTN_USER (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOE | PIO_PIN13)
#define IRQ_BTN_USER  SAM_IRQ_PE13

/* TM7000 LCD/Touchscreen ***************************************************/

/* The TM7000 LCD is available for the SAMA5D4-EK.  See documentation
 * available on the Precision Design Associates website:
 * http://www.pdaatl.com/doc/tm7000.pdf
 *
 * The TM7000 features an touchscreen controol
 *
 *   - 7 inch LCD at 800x480 18-bit RGB resolution and white backlight
 *   - Projected Capacitive Multi-Touch Controller based on the Atmel
 *     MXT768E maXTouch™ IC
 *   - 4 Capacitive “Navigation” Keys available via an Atmel AT42QT1070
 *     QTouch™ Button Sensor IC
 *   - 200 bytes of non-volatile serial EEPROM
 *
 * Both the MXT768E and the AT42QT1070 are I2C devices with interrupting
 * PIO pins:
 *
 * ------------------------ -----------------
 * SAMA5D4-EK               TM7000
 * ------------------------ -----------------
 * J9 pin 5 LCD_PE24        J4 pin 5 ~CHG_mxt
 * J9 pin 6 LCD_PE25        J4 pin 6 ~CHG_QT
 * J9 pin 7 LCD_TWCK0_PA31  J4 pin 7 SCL_0
 * J9 pin 8 LCD_TWD0_PA30   J4 pin 8 SDA_0
 * ------------------------ -----------------
 *
 * The schematic indicates the MXT468E address is 0x4c/0x4d.
 */

#define PIO_CHG_MXT  (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_FALLING | PIO_PORT_PIOE | PIO_PIN24)
#define IRQ_CHG_MXT   SAM_IRQ_PE24

#define PIO_CHG_QT   (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_FALLING | PIO_PORT_PIOE | PIO_PIN25)
#define IRQ_CHG_QT    SAM_IRQ_PE25

/* The touchscreen communicates on TWI0, I2C address 0x4c */

#define MXT_TWI_BUS      0
#define MXT_I2C_ADDRESS  0x4c

/* HSMCI Card Slots *********************************************************/

/* The SAMA5D4-EK provides a two SD memory card slots:  (1) a full size SD
 * card slot (J10), and (2) a microSD memory card slot (J11).
 *
 * The full size SD card slot connects via HSMCI0.  The card detect discrete
 * is available on PE5 (pulled high).  The write protect discrete is tied to
 * ground and is not available to software.  The slot supports 8-bit wide
 * transfer mode, but the NuttX driver currently uses only the 4-bit wide
 * transfer mode
 *
 * ------------------------------ ------------------- -----------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -----------------------
 * PC4/SPI0_NPCS1/MCI0_CK/PCK1    PC4                 MCI0_CK, ISI_MCK, EXP
 * PC5/D0/MCI0_CDA                PC5                 MCI0_CDA, NAND_IO0
 * PC6/D1/MCI0_DA0                PC6                 MCI0_DA0, NAND_IO1
 * PC7/D2/MCI0_DA1                PC7                 MCI0_DA1, NAND_IO2
 * PC8/D3/MCI0_DA2                PC8                 MCI0_DA2, NAND_IO3
 * PC9/D4/MCI0_DA3                PC9                 MCI0_DA3, NAND_IO4
 * PC10/D5/MCI0_DA4               PC10                MCI0_DA4, NAND_IO5
 * PC11/D6/MCI0_DA5               PC11                MCI0_DA5, NAND_IO6
 * PC12/D7/MCI0_DA6               PC12                MCI0_DA6, NAND_IO7
 * PC13/NRD/NANDOE/MCI0_DA7       PC13                MCI0_DA7, NAND_RE
 * PE5/A5/CTS3                    MCI0_CD_PE5         MCI0_CD
 * ------------------------------ ------------------- -----------------------
 */

#define PIO_MCI0_CD  (PIO_INPUT | PIO_CFG_DEFAULT | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOE | PIO_PIN5)
#define IRQ_MCI0_CD   SAM_IRQ_PE5

/* The microSD connects vi HSMCI1.  The card detect discrete is available on
 * PE6 (pulled high)  NOTE that PE15 must be controlled to provide power
 * to the HSMCI1 slot (the HSMCI0 slot is always powered).
 *
 * ------------------------------ ------------------- -----------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -----------------------
 * PE14/A14/TCLK1/PWMH3           MCI1_CD_PE14        MCI1_CD             ???
 * PE15/A15/SCK3/TIOA0            MCI1_PWR_PE15       MCI1_PWR
 * PE18/A18/TIOA5/MCI1_CK         PE18                MCI1_CK, EXP
 * PE19/A19/TIOB5/MCI1_CDA        PE19                MCI1_CDA, EXP
 * PE20/A20/TCLK5/MCI1_DA0        PE20                MCI1_DA0, EXP
 * PE21/A23/TIOA4/MCI1_DA1        PE21                MCI1_DA1, EXP
 * PE22/A24/TIOB4/MCI1_DA2        PE22                MCI1_DA2, EXP
 * PE23/A25/TCLK4/MCI1_DA3        PE23                MCI1_DA3, EXP
 * PE6/A6/TIOA3                   MCI1_CD_PE6         MCI1_CD
 * ------------------------------ ------------------- -----------------------
 */

#define PIO_MCI1_CD  (PIO_INPUT | PIO_CFG_DEFAULT | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOE | PIO_PIN6)
#define IRQ_MCI1_CD   SAM_IRQ_PE6

#define IRQ_MCI1_PWR (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_SET | \
                      PIO_PORT_PIOE | PIO_PIN15)

/* USB Ports ****************************************************************/

/* The SAMA5D4-EK features three USB communication ports:
 *
 *   * Port A Host High Speed (EHCI) and Full Speed (OHCI) multiplexed with
 *     USB Device High Speed Micro AB connector, J1
 *
 *   * Port B Host High Speed (EHCI) and Full Speed (OHCI) standard type A
 *     connector, J5 upper port
 *
 *   * Port C Host Full Speed (OHCI)  and Full Speed (OHCI) standard type A
 *     connector, J5 lower port
 *
 * The three  USB host ports are equipped with 500-mA high-side power
 * switch for self-powered and bus-powered applications.
 *
 * The USB device port A (J5) features a VBUS insert detection function.
 *
 * Port A
 * ------
 *
 *   PIO  Signal Name    Function
 *   ---- -------------- ----------------------------------------------------
 *   PE10 USBA_EN5V_PE10 VBus power enable (via MN2 power switch) to VBus pin
 *                       of the OTG connector (host)
 *   PE31 USBA_VBUS_PE31 VBus sensing from the VBus pin of the OTG connector
 *                      (device)
 */

#ifdef CONFIG_SAMA5_UHPHS_RHPORT1
  #define PIO_USBA_VBUS_ENABLE \
                     (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_CLEAR | \
                      PIO_PORT_PIOE | PIO_PIN10)
#endif

#ifdef HAVE_USBDEV
#  define PIO_USBA_VBUS_SENSE \
                     (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOE | PIO_PIN31)
#  define IRQ_USBA_VBUS_SENSE \
                     SAM_IRQ_PE31
#endif

/* Port B
 * ------
 *
 *   PIO  Signal Name    Function
 *   ---- -------------- ----------------------------------------------------
 *   PE11 USBB_EN5V_PE11 VBus power enable (via MN4 power switch).  To the A1
 *                       pin of J5 Dual USB A connector
 */

#ifdef CONFIG_SAMA5_UHPHS_RHPORT2
#  define PIO_USBB_VBUS_ENABLE \
                     (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_CLEAR | \
                      PIO_PORT_PIOE | PIO_PIN11)
#endif

/* Port C
 * ------
 *
 *   PIO  Signal Name    Function
 *   ---- -------------- ----------------------------------------------------
 *   PE12 USBC_EN5V_PE12 VBus power enable (via MN4 power switch).  To the B1
 *                       pin of J5 Dual USB A connector
 */

#ifdef CONFIG_SAMA5_UHPHS_RHPORT3
#  define PIO_USBC_VBUS_ENABLE \
                     (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_CLEAR | \
                      PIO_PORT_PIOE | PIO_PIN12)
#endif

/* Both Ports B and C
 * ------------------
 *
 *   PIO  Signal Name   Function
 *   ---- ------------- -----------------------------------------------------
 *   PD9  USB_OVCUR_PD9 Combined over-current indication from port A and B
 */

#ifdef HAVE_USBOVCUR
#  define PIO_USBBC_VBUS_OVERCURRENT \
                     (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOD | PIO_PIN9)
#  define IRQ_USBBC_VBUS_OVERCURRENT \
                     SAM_IRQ_PD9
#endif

/* Ethernet */

#ifdef CONFIG_SAMA5_EMACB
/* ETH0/1: Ethernet 10/100 (EMAC) Ports
 *
 * Networking support via the can be added to NSH by selecting the following
 * configuration options.  The SAMA5D44 supports two different 10/100Base-T
 * Ethernet MAC peripherals.
 *
 * ------------------------------ ------------------- -----------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -----------------------
 * PB0/G0_TXCK                    PB0                 G0_TXCK, EXP
 * PB1/G0_RXCK/SCK2/ISI_PCK       ISI_PCK_PB1         ISI_PCK
 * PB2/G0_TXEN                    PB2                 G0_TXEN,EXP
 * PB3/G0_TXER/CTS2/ISI_VSYNC     ISI_VSYNC_PB3       ISI_VSYNC
 * PB4/G0_CRS/RXD2/ISI_HSYNC      ISI_HSYNC_PB4       ISI_HSYNC
 * PB5/G0_COL/TXD2/PCK2           ISI_PWD_PB5         ISI_PWD
 * PB6/G0_RXDV                    PB6                 G0_RXDV, EXP
 * PB7/G0_RXER                    PB7                 G0_RXER, EXP
 * PB8/G0_RX0                     PB8                 G0_RX0, EXP
 * PB9/G0_RX1                     PB9                 G0_RX1, EXP
 * PB10/G0_RX2/PCK2/PWML1         PB10                AUDIO_PCK2, EXP
 * PB11/G0_RX3/RTS2/PWMH1         ISI_RST_PB11        ISI_RST
 * PB12/G0_TX0                    PB12                G0_TX0, EXP
 * PB13/G0_TX1                    PB13                G0_TX1, EXP
 * PB14/G0_TX2/SPI2_NPCS1/PWMH0   ZIG_SPI2_NPCS1      ZIG_SPI2_NPCS1
 * PB15/G0_TX3/SPI2_NPCS2/PWML0   HDMI_RST_PB15       HDMI_RST
 * PB16/G0_MDC                    PB16                G0_MDC, EXP
 * PB17/G0_MDIO                   PB17                G0_MDIO, EXP
 * PE1/A1/MCI0_DB0                G0_IRQ_PE1          G0_IRQ
 * ------------------------------ ------------------- -----------------------
 * PA2/LCDDAT2/G1_TXCK            PA                  LCDDAT2, G1_TXCK
 * PA3/LCDDAT3/G1_RXCK            PA3                 LCDDAT3
 * PA4/LCDDAT4/G1_TXEN            PA4                 LCDDAT4, G1_TXEN
 * PA5/LCDDAT5/G1_TXER            PA5                 LCDDAT5
 * PA6/LCDDAT6/G1_CRS             PA6                 LCDDAT6
 * PA9/LCDDAT9/G1_COL             PA9                 LCDDAT9
 * PA10/LCDDAT10/G1_RXDV          PA10                LCDDAT10, G1_RXDV
 * PA11/LCDDAT11/G1_RXER          PA11                LCDDAT11, G1_RXER
 * PA12/LCDDAT12/G1_RX0           PA12                LCDDAT12, G1_RX0
 * PA13/LCDDAT13/G1_RX1           PA13                LCDDAT13, G1_RX1
 * PA14/LCDDAT14/G1_TX0           PA14                LCDDAT14, G1_TX0
 * PA15/LCDDAT15/G1_TX1           PA15                LCDDAT15, G1_TX1
 * PA18/LCDDAT18/G1_RX2           PA18                LCDDAT18
 * PA19/LCDDAT19/G1_RX3           PA19                LCDDAT19
 * PA20/LCDDAT20/G1_TX2           PA20                LCDDAT20
 * PA21/LCDDAT21/G1_TX3           PA21                LCDDAT21
 * PA22/LCDDAT22/G1_MDC           PA22                LCDDAT22, G1_MDC
 * PA23/LCDDAT23/G1_MDIO          PA23                LCDDAT23, G1_MDIO
 * PE2/A2/MCI0_DB1                G1_IRQ_PE2          G1_IRQ
 * ------------------------------ ------------------- -----------------------
 *
 * EMAC2 connects (directly) to a KSZ8081RNB PHY (U10) and is available at
 * the ETH0 connector.
 *
 * EMAC1 connects (indirectly) to another KSZ8081RNB PHY (U7) and is
 * available at the ETH1 connector.
 * The ETH1 signals go through a line driver that is enabled via
 * LCD_ETH1_CONFIG when an LCD is detected:
 *
 * - LCD_ETH1_CONFIG = 0: LCD 5v disable
 * - LCD_ETH1_CONFIG = 1 & LCD_DETECT# =0: LCD 5v enable.
 *
 * The sense of KSZ8081 interrupt is configurable but is, by default,
 * active low.
 */

#ifdef CONFIG_SAMA5_EMAC0
#  define PIO_INT_ETH0 (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                        PIO_INT_FALLING | PIO_PORT_PIOE | PIO_PIN1)
#  define IRQ_INT_ETH0 SAM_IRQ_PE1
#endif

#ifdef CONFIG_SAMA5_EMAC1
#  define PIO_INT_ETH1 (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                        PIO_INT_FALLING | PIO_PORT_PIOE | PIO_PIN2)
#  define IRQ_INT_ETH1 SAM_IRQ_PE2
#endif
#endif

/* WM8904 Audio Codec *******************************************************/

/* SAMA5D4 Interface
 * ---- ------------------ ---------------- ---------- ----------------------
 * PIO  USAGE              BOARD SIGNAL     WM8904 PIN NOTE
 * ---- ------------------ ---------------- ---------- ----------------------
 * PA30 TWD0               AUDIO_TWD0_PA30  SDA        Pulled up, See J23 note below
 * PA31 TWCK0              AUDIO_TWCK0_PA31 SCLK       Pulled up
 * PB10 AUDIO_PCK2/EXP     AUDIO_PCK2_PB10  MCLK
 * PB27 AUDIO/HDMI_TK0/EXP AUDIO_TK0_PB27   BCLK/GPIO4 TK0/RK0 are mutually exclusive
 * PB26 AUDIO_RK0          AUDIO_RK0_PB26   "  "/"   " " "/" " " " "      " "       "
 * PB30 AUDIO_RF/ZIG_TWCK2 AUDIO_RF0_PB30   LRCLK      TF0/RF0 are mutually exclusive
 * PB31 AUDIO/HDMI_TF0/EXP AUDIO_TF0_PB31   "   "      " "/" " " " "      " "       "
 * PB29 AUDIO_RD0/ZIG_TWD2 AUDIO_RD0_PB29   ADCDAT
 * PB28 AUDIO/HDMI_TD0/EXP AUDIO_TD0_PB28   ACDAT
 * PE4  AUDIO_IRQ          AUDIO_IRQ_PE4    IRQ/GPIO1  Audio interrupt
 * ---- ------------------ ---------------- ---------- ----------------------
 * Note that jumper J23 must be closed to connect AUDIO_TWD0_PA30
 */

/* Pin Disambiguation */

#define PIO_SSC0_TD    PIO_SSC0_TD_2

/* Audio Interrupt. All interrupts are default, active high level.
 * Pull down internally in the WM8904.
 * So we want no pull-up/downs and we want to interrupt on the high level.
 */

#define PIO_INT_WM8904 (PIO_INPUT | PIO_CFG_DEFAULT | PIO_CFG_DEGLITCH | \
                        PIO_INT_HIGHLEVEL | PIO_PORT_PIOE | PIO_PIN4)
#define IRQ_INT_WM8904 SAM_IRQ_PE4

/* The MW8904 communicates on TWI0, I2C address 0x1a for control operations */

#define WM8904_TWI_BUS      0
#define WM8904_I2C_ADDRESS  0x1a

/* The MW8904 transfers data on SSC0 */

#define WM8904_SSC_BUS      0

/* SPI Chip Selects *********************************************************/

/* The SAMA5D4-EK includes an Atmel AT25DF321A, 32-megabit,
 * 2.7-volt SPI serial FLASH on board.
 * The connection is as follows:
 *
 *   AT25DF321A SAMA5D4-EK      SAMA5
 *   ---------- --------------- --------------------------------
 *   SI         AT25_SPI0_SI    PC1 PC1/SPI0_MOSI/PWML2/ISI_D9
 *   SO         AT25_SPI0_SO    PC0 PC0/SPI0_MISO/PWMH2/ISI_D8
 *   SCK        AT25_SPI0_SPCK  PC2 PC2/SPI0_SPCK/PWMH3/ISI_D10
 *   /CS        AT25_SPI0_NCPS0 PC3 PC3/SPI0_NPCS0/PWML3/ISI_D11
 *
 * AT25_SPI0_NCPS0 goes to the AT25DF321A as via a NL17SZ126 if JP6 is closed
 */

#define PIO_AT25_NPCS0 (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_SET | \
                        PIO_PORT_PIOC | PIO_PIN3)
#define AT25_PORT      SPI0_CS0

/* ACT8865 power management chip ********************************************/

/* The PMIC communicates on TWI0, I2C address 0x5b */

#define PMIC_TWI_BUS       0
#define PMIC_I2C_ADDRESS   0x5b
#define PMIC_I2C_FREQUENCY 400000 /* 400KHz max */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select PIO pins for the SAMA5D4-EK board.
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_SPI0) || defined(CONFIG_SAMA5_SPI1)
void weak_function sam_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: sam_sdram_config
 *
 * Description:
 *   Configures DDR2 (MT47H128M16RT 128MB or, optionally,  MT47H64M16HR)
 *
 *   Per the SAMA5D4-EK User guide:
 *   "Two SDRAM/DDR2 used as main system memory.
 *   MT47H128M16 - 2 Gb - 16 Meg x 16 x 8 banks, the board provides up to
 *   2 Gb on- board, soldered DDR2 SDRAM.
 *   The memory bus is 32 bits wide and operates with up to 166 MHz."
 *
 *   From the Atmel Code Example:
 *     MT47H64M16HR : 8 Meg x 16 x 8 banks
 *     Refresh count: 8K
 *     Row address: A[12:0] (8K)
 *     Column address A[9:0] (1K)
 *     Bank address BA[2:0] a(24,25) (8)
 *
 *  This logic was taken from Atmel sample code for the SAMA5D4-EK.
 *
 *  Input Parameters:
 *    None
 *
 *  Assumptions:
 *    The DDR memory regions is configured as strongly ordered memory.
 *    When we complete initialization of SDRAM and it is ready for use,
 *    we will make DRAM into normal memory.
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_DDRCS) && !defined(CONFIG_SAMA5_BOOT_SDRAM)
void sam_sdram_config(void);
#else
#  define sam_sdram_config()
#endif

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int sam_bringup(void);

/****************************************************************************
 * Name: sam_nand_automount
 *
 * Description:
 *   Initialize and configure the NAND on CS3
 *
 ****************************************************************************/

#ifdef HAVE_NAND
int sam_nand_automount(int minor);
#endif

/****************************************************************************
 * Name: sam_at25_automount
 *
 * Description:
 *   Initialize and configure the AT25 serial FLASH
 *
 ****************************************************************************/

#ifdef HAVE_AT25
int sam_at25_automount(int minor);
#endif

/****************************************************************************
 * Name: sam_hsmci_initialize
 *
 * Description:
 *   Initialize and configure one HSMCI slot
 *
 ****************************************************************************/

#ifdef HAVE_HSMCI
int sam_hsmci_initialize(int slotno, int minor);
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
 *      There is a terminology problem here:
 *      Each HSMCI supports two slots, slot A and slot B.
 *      Only slot A is used.
 *      So this is not a really a slot, but an HSCMI peripheral number.
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
#endif

/****************************************************************************
 * Name: sam_usbinitialize
 *
 * Description:
 *   Called from sam_usbinitialize very early in inialization to setup
 *   USB-related PIO pins for the SAMA5D4-EK board.
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

/****************************************************************************
 * Name: sam_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.
 *   This function will register the driver as /dev/inputN where N is the
 *   minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.
 *   Otherwise, a negated errno value is returned to indicate the nature of
 *   the failure.
 *
 ****************************************************************************/

#ifdef HAVE_MAXTOUCH
int sam_tsc_setup(int minor);
#endif

/****************************************************************************
 * Name: sam_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int sam_pwm_setup(void);
#endif

/****************************************************************************
 * Name: sam_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int sam_adc_setup(void);
#endif

/****************************************************************************
 * Name: sam_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ****************************************************************************/

#ifdef HAVE_NETWORK
void weak_function sam_netinitialize(void);
#endif

/****************************************************************************
 * Name: sam_wm8904_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the WM8904 device.  This function will register the driver
 *   as /dev/wm8904[x] where x is determined by the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef HAVE_WM8904
int sam_wm8904_initialize(int minor);
#endif /* HAVE_WM8904 */

/****************************************************************************
 * Name: sam_audio_null_initialize
 *
 * Description:
 *   Set up to use the NULL audio device for PCM unit-level testing.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef HAVE_AUDIO_NULL
int sam_audio_null_initialize(int minor);
#endif /* HAVE_AUDIO_NULL */

/****************************************************************************
 * Name: sam_pmic_initialize
 *
 * Description:
 *   Currently, this function only disables the PMIC.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef HAVE_PMIC
void sam_pmic_initialize(void);
#else
#  define sam_pmic_initialize()
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMA5_SAMA5D4_EK_SRC_SAMA5D4_EK_H */
