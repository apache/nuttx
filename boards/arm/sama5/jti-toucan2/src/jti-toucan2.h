/****************************************************************************
 * boards/arm/sama5/jti-toucan2/src/giant-board.h
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

#ifndef __BOARDS_ARM_SAMA5_JTI_TOUCAN2_SRC_JTI_TOUCAN2_H
#define __BOARDS_ARM_SAMA5_JTI_TOUCAN2_SRC_JTI_TOUCAN2_H

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

//#define HAVE_USBHOST    1
//#define HAVE_USBDEV     1
//#define HAVE_USBMONITOR 1
//#define HAVE_AT25		1
//#define HAVE_CAN		1
//#define HAVE_PWM		1


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

#if !defined(CONFIG_SAMA5D2XULT_AT25_FTL) && !defined(CONFIG_SAMA5D2XULT_AT25_NXFFS)
#  undef HAVE_AT25
#endif

#if defined(CONFIG_SAMA5D2XULT_AT25_FTL) && defined(CONFIG_SAMA5D2XULT_AT25_NXFFS)
#  warning Both CONFIG_SAMA5D2XULT_AT25_FTL and CONFIG_SAMA5D2XULT_AT25_NXFFS are set
#  warning Ignoring CONFIG_SAMA5D2XULT_AT25_NXFFS
#  undef CONFIG_SAMA5D2XULT_AT25_NXFFS
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

#if !defined(CONFIG_NET)
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

/* JTi Toucan2 pinout
 *
 * There is one main connector, J2.
 *
 *
 * Pin 1	Ground
 * Pin 2	CAN2 HI
 * Pin 3	CAN2 LO
 * Pin 4	RS232 TX 
 * Pin 5	0-12V switch input1, active high 
 * Pin 6	0-12V switch input4, active high 
 * Pin 7	0-12V switch input3, active high 
 * Pin 8	0-12V switch input5, active high 
 * Pin 9 	0-5V Analogue Input 2 (ADC 5)
 * Pin 10 	0-5V Analogue Input 3 (ADC 6)
 * Pin 11	K-thermocouple input+ (onboard MAX31855K)
 * Pin 12	Lowside driver output (onboard 1A lowside driver driven by PWM)
 * Pin 13	Kline/Lin (onboard TJA1021)
 * Pin 14	Power in (8-40V DC)
 * Pin 15 	CAN1 HI
 * Pin 16	CAN1 LO
 * Pin 17	RS232 RX
 * Pin 18	0-5V output1, controlled from a filtered/buffered processor PWM output
 * Pin 19	0-5V output2, controlled from a filtered/buffered processor PWM output
 * Pin 20	0-12V switch input2, active high 
 * Pin 21	0-12V switch input6, active high 
 * Pin 22 	0-5V Analogue Input 1 (ADC 4)
 * Pin 23 	0-5V Analogue Input 4 (ADC 7)
 * Pin 24	K-thermocouple input - (onboard MAX31855K)
 * Pin 25	5V reference output from onboard linear regulator
 * Pin 26	Ground (typically used to power off-board sensors along with +5V from pin 25)

*/ 


//#define SDMMC_INIT_CLKDIV          (119 << SDMMC_MR_CLKDIV_SHIFT)

/* MCK = 96MHz, CLKDIV = 3, MCI_SPEED = 96MHz / 2 * (3+1) = 12 MHz */

//#define SDMMC_MMCXFR_CLKDIV        (3 << SDMMC_MR_CLKDIV_SHIFT)

/* MCK = 96MHz, CLKDIV = 1, MCI_SPEED = 96MHz / 2 * (1+1) = 24 MHz */

//#define SDMMC_SDXFR_CLKDIV         (1 << SDMMC_MR_CLKDIV_SHIFT)
//#define SDMMC_SDWIDEXFR_CLKDIV     SDMMC_SDXFR_CLKDIV

/* SDMMC Card Slots *********************************************************/

/* The Giant Board provides a SD memory card slot:
 *  a full size SD card slot (J6)
 *
 * The full size SD card slot connects via SDMMC1.  The card detect discrete
 * is available on PA21 (pulled high) and shared with DAT3.  The write
 * protect discrete is not connected and not available to software.  The
 * slot only supports 4-bit wide transfer mode, and the NuttX driver
 * currently uses only the 4-bit wide transfer mode.
 *
 *   PA18 SDMMC1_DAT0
 *   PA19 SDMMC1_DAT1
 *   PA20 SDMMC1_DAT2
 *   PA21 SDMMC1_DAT3/SDMMC1_CD
 *   PA22 SDMMC1_CK
 *   PA28 SDMMC1_CMD
 */

//#define IRQ_SDMMC1_CD   SAM_IRQ_PA30


/****************************************************************************
 * Public Data
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

#if defined(CONFIG_PWM)
int sam_pwm_setup(int pwm_channel);
#endif

#if defined (CONFIG_CAN)
int sam_can_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMA5_JTI_TOUCAN2_SRC_JTI_TOUCAN2_H */
