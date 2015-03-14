/************************************************************************************
 * configs/sama5d3-xplained/src/sama5d3-xplained.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_SAMA5D3_XPLAINED_SRC_SAMA5D3_XPLAINED_H
#define __CONFIGS_SAMA5D3_XPLAINED_SRC_SAMA5D3_XPLAINED_H

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

/* USB Host / USB Device */
/* Either CONFIG_SAMA5_UHPHS or CONFIG_SAMA5_UDPHS must be defined, or there is
 * no USB of any kind.
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

#ifndef CONFIG_SYSTEM_USBMONITOR
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

/* LEDs *****************************************************************************/
/* There are two LEDs on the SAMA5D3 series-CM board that can be controlled
 * by software.  A  blue LED is controlled via PIO pins.  A red LED normally
 * provides an indication that power is supplied to the board but can also
 * be controlled via software.
 *
 *   PE23.  This blue LED is pulled high and is illuminated by pulling PE23
 *   low.
 *
 *   PE24.  The red LED is also pulled high but is driven by a transistor so
 *   that it is illuminated when power is applied even if PE24 is not
 *   configured as an output.  If PE24 is configured as an output, then the
 *   LCD is illuminated by a high output.
 */

#define PIO_BLUE     (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_SET | \
                      PIO_PORT_PIOE | PIO_PIN23)
#define PIO_RED      (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_CLEAR | \
                      PIO_PORT_PIOE | PIO_PIN24)

/* Buttons **************************************************************************/
/* There are five push button switches on the SAMA5D3-Xplained base board:
 *
 *   1. One Reset, board reset (BP1)
 *   2. One Wake up, push button to bring the processor out of low power mode
 *     (BP2)
 *   3. One User momentary Push Button
 *   4. One Disable CS Push Button
 *
 * Only the user push button is controllable by software (labeled
 * "PB_USER1" on the board):
 *
 *   - PE29.  Pressing the switch connects PE29 to ground.  Therefore, PE29
 *     must be pulled high internally.  When the button is pressed the SAMA5
 *     will sense "0" is on PE29.
 */

#define PIO_USER     (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOE | PIO_PIN29)
#define IRQ_USER      SAM_IRQ_PE29

/* HSMCI Card Slots *****************************************************************/
/* The SAMA5D3-Xplained provides a two SD memory card slots:  (1) a full size SD card
 * slot (J10), and (2) a microSD memory card slot (J11).
 *
 * The full size SD card slot connects via HSMCI0.  The card detect discrete
 * is available on PD17 (pulled high).  The write protect descrete is tied to
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
 * PBD8 (pulled high):
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

/* USB Ports ************************************************************************/
/* The SAMA5D3 series-MB features three USB communication ports:
 *
 *   1. Port A Host High Speed (EHCI) and Full Speed (OHCI) multiplexed with
 *      USB Device High Speed Micro AB connector, J6
 *
 *   2. Port B Host High Speed (EHCI) and Full Speed (OHCI) standard type A
 *      connector, J7 upper port
 *
 *   3. Port C Host Full Speed (OHCI) only standard type A connector, J7
 *      lower port
 *
 * The two USB host ports (only) are equipped with 500-mA high-side power
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
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOE | PIO_PIN9)
#define IRQ_USBA_VBUS_SENSE \
                     SAM_IRQ_PE9

/* Port B
 *
 *   PIO  Signal Name Function
 *   ---- ----------- -------------------------------------------------------
 *   PE4  EN5V_USBB   VBus power enable (via MN3 AIC1526 Dual USB High-Side
 *                    Power Switch).  To the A1 pin of J7 Dual USB A
 *                    connector
 */

#define PIO_USBB_VBUS_ENABLE \
                     (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_CLEAR | \
                      PIO_PORT_PIOE | PIO_PIN4)

/* Port C
 *
 *   PIO  Signal Name Function
 *   ---- ----------- -------------------------------------------------------
 *   PE3  EN5V_USBC   VBus power enable (via MN3 power switch).  To the B1
 *                    pin of J7 Dual USB A connector
 */

#define PIO_USBC_VBUS_ENABLE \
                     (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_CLEAR | \
                      PIO_PORT_PIOE | PIO_PIN3)

/*  Both Ports B and C
 *
 *   PIO  Signal Name Function
 *   ---- ----------- -------------------------------------------------------
 *   PE5  OVCUR_USB   Combined over-current indication from port A and B
 */

#define PIO_USBBC_VBUS_OVERCURRENT \
                     (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOE | PIO_PIN5)
#define IRQ_USBBC_VBUS_OVERCURRENT \
                     SAM_IRQ_PE5

/* Ethernet */

#ifdef CONFIG_SAMA5_EMACA
  /* ETH1: Ethernet 10/100 (EMAC A) Port
   *
   * The main board contains a MICREL PHY device (KSZ8051) operating at 10/100 Mbps.
   * The board supports MII and RMII interface modes.
   *
   * The two independent PHY devices embedded on CM and MB boards are connected to
   * independent RJ-45 connectors with built-in magnetic and status LEDs.
   *
   * At the De-Assertion of Reset:
   *   PHY ADD[2:0]:001
   *   CONFIG[2:0]:001,Mode:RMII
   *   Duplex Mode:Half Duplex
   *   Isolate Mode:Disable
   *   Speed Mode:100Mbps
   *   Nway Auto-Negotiation:Enable
   *
   * The KSZ8051 PHY interrupt is available on PE30 INT_ETH1.  The sense of
   * the interrupt is configurable but is, by default, active low.
   */

#define PIO_INT_ETH1 (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_FALLING | PIO_PORT_PIOE | PIO_PIN30)
#define IRQ_INT_ETH1 SAM_IRQ_PE30

#endif

#ifdef CONFIG_SAMA5_GMAC
  /* ETH0: Tri-Speed Ethernet PHY
   *
   * The SAMA5D3 series-CM board is equipped with a MICREL PHY devices (MICREL
   * KSZ9021/31) operating at 10/100/1000 Mbps. The board supports RGMII interface
   * mode. The Ethernet interface consists of 4 pairs of low voltage differential
   * pair signals designated from GRX± and GTx± plus control signals for link
   * activity indicators. These signals can be used to connect to a 10/100/1000
   * BaseT RJ45 connector integrated on the main board.
   *
   * The KSZ9021/31 interrupt is available on PB35 INT_GETH0.  The sense of
   * the interrupt is configurable but is, by default, active low.
   */

#define PIO_INT_ETH0 (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_FALLING | PIO_PORT_PIOB | PIO_PIN25)
#define IRQ_INT_ETH0 SAM_IRQ_PB25

#endif

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

/* Itead Joystick Shield
 *
 * See http://imall.iteadstudio.com/im120417014.html for more information
 * about this joystick.
 *
 *   --------- ----------------- ---------------------------------
 *   ARDUINO   ITEAD             SAMA5D3 XPLAINED
 *   PIN NAME  SIGNAL            CONNECTOR  SIGNAL
 *   --------- ----------------- ---------- ----------------------
 *    D3       Button E Output   J18 pin 4  PC8
 *    D4       Button D Output   J18 pin 5  PC28
 *    D5       Button C Output   J18 pin 6  PC7
 *    D6       Button B Output   J18 pin 7  PC6
 *    D7       Button A Output   J18 pin 8  PC5
 *    D8       Button F Output   J15 pin 1  PC4
 *    D9       Button G Output   J15 pin 2  PC3
 *    A0       Joystick Y Output J17 pin 1  PC18  AD0 (function 4)
 *    A1       Joystick X Output J17 pin 2  PD21  AD1 (function 1)
 *   --------- ----------------- ---------- ----------------------
 *
 * All buttons are pulled on the shield.  A sensed low value indicates
 * when the button is pressed.
 */

#define ADC_XOUPUT   1 /* X output is on ADC channel 1 */
#define ADC_YOUPUT   0 /* Y output is on ADC channel 0 */

#define PIO_BUTTON_A (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOC | PIO_PIN5)
#define IRQ_BUTTON_A  SAM_IRQ_PC5
#define PIO_BUTTON_B (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOC | PIO_PIN6)
#define IRQ_BUTTON_B  SAM_IRQ_PC6
#define PIO_BUTTON_C (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOC | PIO_PIN7)
#define IRQ_BUTTON_C  SAM_IRQ_PC7
#define PIO_BUTTON_D (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOC | PIO_PIN28)
#define IRQ_BUTTON_D  SAM_IRQ_PC28
#define PIO_BUTTON_E (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOC | PIO_PIN8)
#define IRQ_BUTTON_E  SAM_IRQ_PC8
#define PIO_BUTTON_F (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOC | PIO_PIN4)
#define IRQ_BUTTON_F  SAM_IRQ_PC4
#define PIO_BUTTON_G (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOC | PIO_PIN3)
#define IRQ_BUTTON_G  SAM_IRQ_PC3

/* Itead Joystick Signal interpretation:
 *
 *   --------- ----------------------- ---------------------------
 *   BUTTON     TYPE                    NUTTX ALIAS
 *   --------- ----------------------- ---------------------------
 *   Button A  Large button A          JUMP/BUTTON 3
 *   Button B  Large button B          FIRE/BUTTON 2
 *   Button C  Joystick select button  SELECT/BUTTON 1
 *   Button D  Tiny Button D           BUTTON 6
 *   Button E  Tiny Button E           BUTTON 7
 *   Button F  Large Button F          BUTTON 4
 *   Button G  Large Button G          BUTTON 5
 *   --------- ----------------------- ---------------------------
 */

#define PIO_BUTTON_1 PIO_BUTTON_C
#define IRQ_BUTTON_1 IRQ_BUTTON_C
#define PIO_BUTTON_2 PIO_BUTTON_B
#define IRQ_BUTTON_2 IRQ_BUTTON_B
#define PIO_BUTTON_3 PIO_BUTTON_A
#define IRQ_BUTTON_3 IRQ_BUTTON_A
#define PIO_BUTTON_4 PIO_BUTTON_F
#define IRQ_BUTTON_4 IRQ_BUTTON_F
#define PIO_BUTTON_5 PIO_BUTTON_G
#define IRQ_BUTTON_5 IRQ_BUTTON_G
#define PIO_BUTTON_6 PIO_BUTTON_D
#define IRQ_BUTTON_6 IRQ_BUTTON_D
#define PIO_BUTTON_7 PIO_BUTTON_E
#define IRQ_BUTTON_7 IRQ_BUTTON_E

#define PIO_SELECT   PIO_BUTTON_1
#define IRQ_SELECT   IRQ_BUTTON_1
#define PIO_FIRE     PIO_BUTTON_2
#define IRQ_FIRE     IRQ_BUTTON_2
#define PIO_JUMP     PIO_BUTTON_3
#define IRQ_JUMP     IRQ_BUTTON_3

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
 *   Called to configure SPI chip select PIO pins for the SAMA5D3-Xplained board.
 *
 ************************************************************************************/

#if defined(CONFIG_SAMA5_SPI0) || defined(CONFIG_SAMA5_SPI1)
void weak_function sam_spiinitialize(void);
#endif

/************************************************************************************
 * Name: sam_sdram_config
 *
 * Description:
 *   Configures DDR2 (MT47H128M16RT 128MB or, optionally,  MT47H64M16HR)
 *
 *   Per the SAMA5D3-Xplained User guide: "Two SDRAM/DDR2 used as main system memory.
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
 *  This logic was taken from Atmel sample code for the SAMA5D3-Xplained.
 *
 *  Input Parameters:
 *     None
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
#  define sam_sdram_config()
#endif

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

/************************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI
bool sam_cardinserted(int slotno);
#endif

/************************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if the card in the MMCSD slot is write protected
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI
bool sam_writeprotected(int slotno);
#endif

/************************************************************************************
 * Name: sam_usbinitialize
 *
 * Description:
 *   Called from sam_usbinitialize very early in initialization to setup USB-related
 *   PIO pins for the SAMA5D3-Xplained board.
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

#ifdef HAVE_USBHOST
int sam_usbhost_initialize(void);
#endif

/************************************************************************************
 * Name: sam_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ************************************************************************************/

#ifdef HAVE_NETWORK
void weak_function sam_netinitialize(void);
#endif

/************************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, && CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ************************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

/************************************************************************************
 * Name: board_adc_initialize
 *
 * Description:
 *   Initialize and register the ADC driver
 *
 ************************************************************************************/

#ifdef CONFIG_SAMA5_ADC
int board_adc_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SAMA5D3_XPLAINED_SRC_SAMA5D3_XPLAINED_H */

