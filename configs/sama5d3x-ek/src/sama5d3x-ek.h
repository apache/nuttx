/************************************************************************************
 * configs/sama5d3x-ek/src/sama5d3x-ek.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __CONFIGS_SAMA5D3X_EK_SRC_SAMA5D3X_EK_H
#define __CONFIGS_SAMA5D3X_EK_SRC_SAMA5D3X_EK_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "chip/sam_pinmap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ************************************************************/

#define HAVE_HSMCI_MTD  1
#define HAVE_AT25_MTD   1

/* HSMCI */
/* Can't support MMC/SD if the card interface(s) are not enable */

#if !defined(CONFIG_SAMA5_HSMCI0) && !defined(CONFIG_SAMA5_HSMCI1)
#  undef HAVE_HSMCI_MTD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_HSMCI_MTD) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No MMC/SD support
#  undef HAVE_HSMCI_MTD
#endif

/* We need PIO interrupts on PIOD to support card detect interrupts */

#if defined(HAVE_HSMCI_MTD) && !defined(CONFIG_SAMA5_PIOD_IRQ)
#  warning PIOD interrupts not enabled.  No MMC/SD support.
#  undef HAVE_HSMCI_MTD
#endif

/* AT25 Serial FLASH */
/* Can't support the AT25 device if it SPI0 or AT25 support are not enabled */

#if !defined(CONFIG_SAMA5_SPI0) || !defined(CONFIG_MTD_AT25)
#  undef HAVE_AT25_MTD
#endif

/* Can't support AT25 features if mountpoints are disabled or if we were not
 * asked to mount the AT25 part
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAMA5_AT25_AUTOMOUNT)
#  undef HAVE_AT25_MTD
#endif

/* If we are going to mount the AT25, then they user must also have told
 * us what to do with it by setting one of these.
 */

#if !defined(CONFIG_SAMA5_AT25_FTL) && !defined(CONFIG_SAMA5_AT25_NXFFS)
#  undef HAVE_AT25_MTD
#endif

#if defined(CONFIG_SAMA5_AT25_FTL) && defined(CONFIG_SAMA5_AT25_NXFFS)
#  warning Both CONFIG_SAMA5_AT25_FTL and CONFIG_SAMA5_AT25_NXFFS are set
#  warning Ignoring CONFIG_SAMA5_AT25_NXFFS
#  undef CONFIG_SAMA5_AT25_NXFFS
#endif

/* USB Host / USB Device */
/* Either CONFIG_SAMA5_UHPHS or CONFIG_SAMA5_UDPHS must be defined, or there is
 * no USB of any kind.
 */

#if !defined(CONFIG_SAMA5_UHPHS)
#  undef CONFIG_SAMA5_OHCI
#  undef CONFIG_SAMA5_EHCI
#endif

#if !defined(CONFIG_SAMA5_UDPHS)
#endif

/* CONFIG_USBDEV and CONFIG_USBHOST must also be defined */

#if defined(CONFIG_USBDEV)
#else
#endif

#if defined(CONFIG_USBHOST)
#  if !defined(CONFIG_SAMA5_OHCI) && !defined(CONFIG_SAMA5_EHCI)
#    warning CONFIG_USBHOST is defined, but neither CONFIG_SAMA5_OHCI nor CONFIG_SAMA5_EHCI are defined
#  endif
#else
#  undef CONFIG_SAMA5_OHCI
#  undef CONFIG_SAMA5_EHCI
#endif

#if defined(CONFIG_SAMA5_OHCI) && defined(CONFIG_SAMA5_EHCI)
#  warning Both CONFIG_SAMA5_OHCI and CONFIG_SAMA5_EHCI are defined
#  undef CONFIG_SAMA5_EHCI
#endif

/* LEDs *****************************************************************************/
/* There are two LEDs on the SAMA5D3 series-CM board that can be controlled
 * by software.  A  blue LED is controlled via PIO pins.  A red LED normally
 * provides an indication that power is supplied to the board but can also
 * be controlled via software.
 *
 *   PE25.  This blue LED is pulled high and is illuminated by pulling PE25
 *   low.
 *
 *   PE24.  The red LED is also pulled high but is driven by a transistor so
 *   that it is illuminated when power is applied even if PE24 is not
 *   configured as an output.  If PE24 is configured as an output, then the
 *   LCD is illuminated by a high output.
 */

#define PIO_BLUE     (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_SET | \
                      PIO_PORT_PIOE | PIO_PIN25)
#define PIO_RED      (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_CLEAR | \
                      PIO_PORT_PIOE | PIO_PIN24)

/* Buttons **************************************************************************/
/* There are five push button switches on the SAMA5D3X-EK base board:
 *
 *   1. One Reset, board reset (BP1)
 *   2. One Wake up, push button to bring the processor out of low power mode
 *     (BP2)
 *   3. One User momentary Push Button
 *   4. One Disable CS Push Button
 *
 * Only the momentary push button is controllable by software (labeled
 * "PB_USER1" on the board):
 *
 *   - PE27.  Pressing the switch connect PE27 to grounded.  Therefore, PE27
 *     must be pulled high internally.  When the button is pressed the SAMA5
 *     will sense "0" is on PE27.
 */

#define PIO_USER1    (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOE | PIO_PIN27)
#define IRQ_USER1     SAM_IRQ_PE27

/* HSMCI Card Slots *****************************************************************/
/* The SAMA5D3x-EK provides a two SD memory card slots:  (1) a full size SD card
 * slot (J7 labeled MCI0), and (2) a microSD memory card slot (J6 labeled MCI1).
 *
 * The full size SD card slot connects via HSMCI0.  The card detect discrete
 * is available on PB17 (pulled high).  The write protect descrete is tied to
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
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOD | PIO_PIN17)
#define IRQ_MCI0_CD   SAM_IRQ_PD17

/* The microSD connects vi HSMCI1.  The card detect discrete is available on
 * PB18 (pulled high):
 *
 *   PD18  MCI1_CD
 *   PB20  MCI1_DA0
 *   PB21  MCI1_DA1
 *   PB22  MCI1_DA2
 *   PB23  MCI1_DA3
 *   PB24  MCI1_CK
 *   PB19  MCI1_CDA
 */

#define PIO_MCI1_CD  (PIO_INPUT | PIO_CFG_DEFAULT | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOD | PIO_PIN18)
#define IRQ_MCI1_CD   SAM_IRQ_PD18

/* SPI Chip Selects *****************************************************************/
/* Both the Ronetix and Embest versions of the SAMAD3x CPU modules include an
 * Atmel AT25DF321A, 32-megabit, 2.7-volt SPI serial flash.  The SPI
 * connection is as follows:
 *
 *   AT25DF321A      SAMA5
 *   --------------- -----------------------------------------------
 *   SI              PD11 SPI0_MOSI
 *   SO              PD10 SPI0_MIS0
 *   SCK             PD12 SPI0_SPCK
 *   /CS             PD13 via NL17SZ126 if JP1 is closed (See below)
 *
 * JP1 and JP2 seem to related to /CS on the Ronetix board, but the usage is
 * less clear.  For the Embest module, JP1 must be closed to connect /CS to
 * PD13; on the Ronetix schematic, JP11 seems only to bypass a resistor (may
 * not be populated?).  I think closing JP1 is correct in either case.
 */

#define PIO_AT25_NPCS0 (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_SET | \
                        PIO_PORT_PIOD | PIO_PIN13)
#define AT25_PORT      SPI0_CS0

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select PIO pins for the SAMA4D3x-EK board.
 *
 ************************************************************************************/

#if defined(CONFIG_SAMA5_SPI0) || defined(CONFIG_SAMA5_SPI1)
void weak_function sam_spiinitialize(void);
#endif

/************************************************************************************
 * Name: board_sdram_config
 *
 * Description:
 *   Configures DDR2 (MT47H128M16RT 128MB or, optionally,  MT47H64M16HR)
 *
 *   Per the SAMA5D3x-EK User guide: "Two SDRAM/DDR2 used as main system memory.
 *   MT47H128M16 - 2 Gb - 16 Meg x 16 x 8 banks, the board provides up to 2 Gb on-
 *   board, soldered DDR2 SDRAM. The memory bus is 32 bits wide and operates with
 *   up to 166 MHz."
 *
 *   From the Atmel Code Example:
 *     MT47H64M16HR : 8 Meg x 16 x 8 banks
 *     Refresh count: 8K
 *     Row address: A[12:0] (8K)
 *     Column address A[9:0] (1K)
 *     Bank address BA[2:0] a(24,25) (8)
 *
 *  This logic was taken from Atmel sample code for the SAMA5D3x-EK.
 *
 *  Input Parameters:
 *     devtype - Either DDRAM_MT47H128M16RT or DDRAM_MT47H64M16HR
 *
 *  Assumptions:
 *    The DDR memory regions is configured as strongly ordered memory.  When we
 *    complete initialization of SDRAM and it is ready for use, we will make DRAM
 *    into normal memory.
 *
 ************************************************************************************/

#if defined(CONFIG_SAMA5_DDRCS) && !defined(CONFIG_SAMA5_BOOT_SDRAM)
void sam_sdram_config(void);
#else
#  define board_sdram_config(t)
#endif

/****************************************************************************
 * Name: sam_at25_initialize
 *
 * Description:
 *   Initialize and configure the AT25 SPI Flash
 *
 ****************************************************************************/

#ifdef HAVE_AT25_MTD
int sam_at25_initialize(int minor);
#endif

/****************************************************************************
 * Name: sam_hsmci_initialize
 *
 * Description:
 *   Initialize and configure one HSMCI slot
 *
 ****************************************************************************/

#ifdef HAVE_HSMCI_MTD
int sam_hsmci_initialize(int slotno, int minor);
#endif

/************************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI_MTD
bool sam_cardinserted(int slotno);
#endif

/************************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if the card in the MMCSD slot is write protected
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI_MTD
bool sam_writeprotected(int slotno);
#endif

/************************************************************************************
 * Name: sam_usbinitialize
 *
 * Description:
 *   Called from sam_usbinitialize very early in inialization to setup USB-related
 *   GPIO pins for the STM32F4Discovery board.
 *
 ************************************************************************************/

#if defined(CONFIG_SAMA5_UHPHS) || defined(CONFIG_SAMA5_UDPHS)
void weak_function sam_usbinitialize(void);
#endif

/****************************************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host functionality. This function will
 *   start a thread that will monitor for device connection/disconnection events.
 *
 ****************************************************************************************************/

#if defined(CONFIG_SAMA5_OHCI) || defined(CONFIG_SAMA5_EHCI)
int sam_usbhost_initialize(void);
#endif

/************************************************************************************
 * Name: up_ledinit
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledinit(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SAMA5D3X_EK_SRC_SAMA5D3X_EK_H */

