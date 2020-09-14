/****************************************************************************
 *  boards/arm/sama5/giant-board/src/giant-board.h
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

#ifndef __BOARDS_ARM_SAMA5_GIANT_BOARD_SRC_GIANT_BOARD_H
#define __BOARDS_ARM_SAMA5_GIANT_BOARD_SRC_GIANT_BOARD_H

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

#define HAVE_SDMMC      1
#define HAVE_USBHOST    1
#define HAVE_USBDEV     1
#define HAVE_USBMONITOR 1
#define HAVE_NETWORK    1

/* SDMMC */

/* Can't support MMC/SD if the card interface(s) are not enable */

#if !defined(CONFIG_SAMA5_SDMMC) && !defined(CONFIG_SAMA5_SDMMC0)
#  undef HAVE_SDMMC
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_SDMMC) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No MMC/SD support
#  undef HAVE_SDMCC
#endif

/* We need PIO interrupts on PIOD to support card detect interrupts */

#if defined(HAVE_SDMMC) && !defined(CONFIG_SAMA5_PIOA_IRQ)
#  warning PIOA interrupts not enabled.  No MMC/SD support.
#  undef HAVE_SDMMC
#endif

/* MMC/SD minor numbers:  The NSH device minor extended is extended to
 * support two devices.  If CONFIG_NSH_MMCSDMINOR is zero, these will be:
 * /dev/mmcsd0 and /dev/mmcsd1.
 */

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

#ifdef HAVE_SDMMC

#  if ( defined(CONFIG_SAMA5_SDMMC0) && defined(CONFIG_SAMA5_SDMMC1) )
#    define SDMMC0_SLOTNO 0
#    define SDMMC1_SLOTNO 1
#  else
#    if ( defined(CONFIG_SAMA5_SDMMC0) )
#      define SDMMC0_SLOTNO 0
#    endif
#    if ( defined(CONFIG_SAMA5_SDMMC1) )
#      define SDMMC1_SLOTNO 0
#    endif
#  endif

#  ifdef CONFIG_SAMA5_SDMMC0
#     define SDMMC0_MINOR  CONFIG_NSH_MMCSDMINOR
#     define SDMMC1_MINOR  (CONFIG_NSH_MMCSDMINOR+1)
#  else
#     define SDMMC1_MINOR  CONFIG_NSH_MMCSDMINOR
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

/* Giant Board pinout
 *
 * Orientation is on pinout diagram, USB connector up.
 * https://groboards.com/giant-board/
 *
 * The Giant Board doesn't have pin numbers, instead the pins are labeled
 * with the first function in this list. (AD4, AD2, etc.)
 *
 * J1 - left pins
 *
 * Reset
 * 3.3V
 * VREF
 * GND
 * AD4 / PD23
 * AD2 / PD21
 * AD1 / PD20
 * AD5 / PD24
 * AD3 / PD22
 * AD0 / PD19
 * SCK / PA14
 * MOSI / PA15
 * MISO / PA16
 * RX / PD2
 * TX / PD3
 * GND
 *
 * J2 - right pins
 *
 * VBAT
 * EN
 * VBUS
 * PD13
 * PD31
 * PWM1 / PB0
 * PWM3 / PB7
 * PWML1 / PB1
 * PWM2 / PB5
 * PB3 / PWMEXTRG
 * SCL / PC0
 * SDA / PB31
 *
 */

/* J1 - left pins */

#define GB_PIO_AD4          PIO_ADC_AD4         /* AD4 / PD23 */
#define GB_PIO_PD23         PIO_ADC_AD4         /* AD4 / PD23 */

#define GB_PIO_AD2          PIO_ADC_AD2         /* AD2 / PD21 */
#define GB_PIO_PD21         PIO_ADC_AD2         /* AD2 / PD21 */

#define GB_PIO_AD1          PIO_ADC_AD1         /* AD1 / PD20 */
#define GB_PIO_PD20         PIO_ADC_AD1         /* AD1 / PD20 */

#define GB_PIO_AD5          PIO_ADC_AD5         /* AD5 / PD24 */
#define GB_PIO_PD24         PIO_ADC_AD5         /* AD5 / PD24 */

#define GB_PIO_AD3          PIO_ADC_AD3         /* AD3 / PD22 */
#define GB_PIO_PD22         PIO_ADC_AD3         /* AD3 / PD22 */

#define GB_PIO_AD0          PIO_ADC_AD0         /* AD0 / PD19 */
#define GB_PIO_PD19         PIO_ADC_AD0         /* AD0 / PD19 */

#define GB_PIO_SCK          PIO_SPI0_SPCK_1     /* SCK / PA14 */
#define GB_PIO_PA14         PIO_SPI0_SPCK_1     /* SCK / PA14 */

#define GB_PIO_MOSI         PIO_SPI0_MOSI_1     /* MOSI / PA15 */
#define GB_PIO_PA15         PIO_SPI0_MOSI_1     /* MOSI / PA15 */

#define GB_PIO_MISO         PIO_SPI0_MISO_1     /* MISO / PA16 */
#define GB_PIO_PA16         PIO_SPI0_MISO_1     /* MISO / PA16 */

#define GB_PIO_RX           PIO_UART0_TXD       /* RX / PD2 */
#define GB_PIO_PD2          PIO_UART0_TXD       /* RX / PD2 */

#define GB_PIO_TX           PIO_UART0_RXD       /* TX / PD3 */
#define GB_PIO_PD3          PIO_UART0_RXD       /* TX / PD3 */

/* J2 - right pins */

#define GB_PIO_PD13         PIO_TC1_CLK_2       /* PD13 */

#define GB_PIO_PD31         PIO_TC3_CLK_3       /* PD31 */

#define GB_PIO_PWM1         PIO_PWM0_H1         /* PWM1 / PB0 */
#define GB_PIO_PB0          PIO_PWM0_H1         /* PWM1 / PB0 */

#define GB_PIO_PWM3         PIO_PWM0_H3         /* PWM3 / PB7 */
#define GB_PIO_PB7          PIO_PWM0_H3         /* PWM3 / PB7 */

#define GB_PIO_PWML1        PIO_PWM0_L1         /* PWML1 / PB1 */
#define GB_PIO_PB1          PIO_PWM0_L1         /* PWML1 / PB1 */

#define GB_PIO_PWM2         PIO_PWM0_H2         /* PWM2 / PB5 */
#define GB_PIO_PB5          PIO_PWM0_H2         /* PWM2 / PB5 */

#define GB_PIO_PB3          PIO_PWM0_EXTRG0     /* PB3 / PWMEXTRG */
#define GB_PIO_PWMEXTRG     PIO_PWM0_EXTRG0     /* PB3 / PWMEXTRG */

#define GB_PIO_SCL          PIO_TWI0_CK_2       /* SCL / PC0 */
#define GB_PIO_PC0          PIO_TWI0_CK_2       /* SCL / PC0 */

#define GB_PIO_SDA          PIO_TWI0_D_2        /* SDA / PB31 */
#define GB_PIO_PB31         PIO_TWI0_D_2        /* SDA / PB31 */

/* LEDs *********************************************************************/

/* There is an orange status LED on board the Giant Board
 * driven by pin (PA6).
 *
 *   ------------------------------ ------------------- ---------------------
 *   SAMA5D2 PIO                    SIGNAL              USAGE
 *   ------------------------------ ------------------- ---------------------
 *   PA6                            STATUS_LED          Orange LED
 *   ------------------------------ ------------------- ---------------------
 */

#define PIO_LED_ORANGE (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_SET | \
                       PIO_PORT_PIOA | PIO_PIN6)

/* SDMMC clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV+1)).
 *
 *   MCI_SPEED = MCK / (2*(CLKDIV+1))
 *   CLKDIV = MCI / MCI_SPEED / 2 - 1
 *
 * Where CLKDIV has a range of 0-255.
 */

/* MCK = 96MHz, CLKDIV = 119, MCI_SPEED = 96MHz / 2 * (119+1) = 400 KHz */

#define SDMMC_INIT_CLKDIV          (119 << SDMMC_MR_CLKDIV_SHIFT)

/* MCK = 96MHz, CLKDIV = 3, MCI_SPEED = 96MHz / 2 * (3+1) = 12 MHz */

#define SDMMC_MMCXFR_CLKDIV        (3 << SDMMC_MR_CLKDIV_SHIFT)

/* MCK = 96MHz, CLKDIV = 1, MCI_SPEED = 96MHz / 2 * (1+1) = 24 MHz */

#define SDMMC_SDXFR_CLKDIV         (1 << SDMMC_MR_CLKDIV_SHIFT)
#define SDMMC_SDWIDEXFR_CLKDIV     SDMMC_SDXFR_CLKDIV

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

#define IRQ_SDMMC1_CD   SAM_IRQ_PA30

/* USB Ports ****************************************************************/

/* The Giant Board features two USB communication ports:
 *
 *   1. Port A Host High Speed (EHCI) and Full Speed (OHCI) multiplexed with
 *      USB Device High Speed Micro AB connector, J3
 *
 *   2. Port B Host High Speed (EHCI) and Full Speed (OHCI) High Speed Micro
 *      AB connector, J7
 *
 */

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
 * Name: sam_sdmmc_initialize
 *
 * Description:
 *   Initialize and configure one SDMMC slot
 *
 ****************************************************************************/

#ifdef HAVE_SDMMC
int sam_sdmmc_initialize(int slotno, int minor);
#endif

/****************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected SDMMC slot
 *
 ****************************************************************************/

#ifdef HAVE_SDMMC
bool sam_cardinserted(int slotno);
#endif

/****************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if the card in the MMCSD slot is write protected
 *
 ****************************************************************************/

#ifdef HAVE_SDMMC
bool sam_writeprotected(int slotno);
#endif

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
#endif /* __BOARDS_ARM_SAMA5_GIANT_BOARD_SRC_GIANT_BOARD_H */
