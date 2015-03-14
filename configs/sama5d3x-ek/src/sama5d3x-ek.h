/************************************************************************************
 * configs/sama5d3x-ek/src/sama5d3x-ek.h
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

#define HAVE_HSMCI      1
#define HAVE_AT24       1
#define HAVE_AT25       1
#define HAVE_NAND       1
#define HAVE_USBHOST    1
#define HAVE_USBDEV     1
#define HAVE_USBMONITOR 1
#define HAVE_NETWORK    1
#define HAVE_CAMERA     1
#define HAVE_WM8904     1

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

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAMA5D3XEK_NAND_BLOCKMOUNT)
#  undef HAVE_NAND
#endif

/* Can't support NAND if the MTD feature is not enabled */

#if !defined(CONFIG_MTD) || !defined(CONFIG_MTD_NAND)
#  undef HAVE_NAND
#endif

/* If we are going to mount the NAND, then they user must also have told
 * us what to do with it by setting one of CONFIG_SAMA5D3xEK_NAND_FTL or
 * CONFIG_SAMA5D3xEK_NAND_NXFFS.
 */

#ifndef CONFIG_MTD
#  undef CONFIG_SAMA5D3xEK_NAND_NXFFS
#  undef CONFIG_SAMA5D3xEK_NAND_FTL
#endif

#if !defined(CONFIG_FS_NXFFS) || !defined(CONFIG_NXFFS_NAND)
#  undef CONFIG_SAMA5D3xEK_NAND_NXFFS
#endif

#if !defined(CONFIG_SAMA5D3xEK_NAND_FTL) && !defined(CONFIG_SAMA5D3xEK_NAND_NXFFS)
#  undef HAVE_NAND
#endif

#if defined(CONFIG_SAMA5D3xEK_NAND_FTL) && defined(CONFIG_SAMA5D3xEK_NAND_NXFFS)
#  warning Both CONFIG_SAMA5D3xEK_NAND_FTL and CONFIG_SAMA5D3xEK_NAND_NXFFS are set
#  warning Ignoring CONFIG_SAMA5D3xEK_NAND_NXFFS
#  undef CONFIG_SAMA5D3xEK_NAND_NXFFS
#endif

/* AT25 Serial FLASH */
/* Can't support the AT25 device if it SPI0 or AT25 support are not enabled */

#if !defined(CONFIG_SAMA5_SPI0) || !defined(CONFIG_MTD_AT25)
#  undef HAVE_AT25
#endif

/* Can't support AT25 features if mountpoints are disabled or if we were not
 * asked to mount the AT25 part
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAMA5D3xEK_AT25_BLOCKMOUNT)
#  undef HAVE_AT25
#endif

/* If we are going to mount the AT25, then they user must also have told
 * us what to do with it by setting one of these.
 */

#ifndef CONFIG_FS_NXFFS
#  undef CONFIG_SAMA5D3xEK_AT25_NXFFS
#endif

#if !defined(CONFIG_SAMA5D3xEK_AT25_FTL) && !defined(CONFIG_SAMA5D3xEK_AT25_NXFFS)
#  undef HAVE_AT25
#endif

#if defined(CONFIG_SAMA5D3xEK_AT25_FTL) && defined(CONFIG_SAMA5D3xEK_AT25_NXFFS)
#  warning Both CONFIG_SAMA5D3xEK_AT25_FTL and CONFIG_SAMA5D3xEK_AT25_NXFFS are set
#  warning Ignoring CONFIG_SAMA5D3xEK_AT25_NXFFS
#  undef CONFIG_SAMA5D3xEK_AT25_NXFFS
#endif

/* AT24 Serial EEPROM
 *
 * A AT24C512 Serial EEPPROM was used for tested I2C.  There are other I2C/TWI
 * devices on-board, but the serial EEPROM is the simplest test.
 *
 * There is, however, no AT24 EEPROM on board the SAMA5D3x-EK:  The Serial
 * EEPROM was mounted on an external adaptor board and connected to the
 * SAMA5D3x-EK thusly:
 *
 *   - VCC -- VCC
 *   - GND -- GND
 *   - TWCK0(PA31) -- SCL
 *   - TWD0(PA30)  -- SDA
 *
 * By default, PA30 and PA31 are SWJ-DP pins, it can be used as a pin for TWI
 * peripheral in the end application.
 */

#define AT24_BUS 0

#if !defined(CONFIG_MTD_AT24XX) || !defined(CONFIG_SAMA5_TWI0)
#  undef HAVE_AT24
#endif

/* Can't support AT25 features if mountpoints are disabled or if we were not
 * asked to mount the AT25 part
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAMA5D3xEK_AT24_BLOCKMOUNT)
#  undef HAVE_AT24
#endif

/* If we are going to mount the AT25, then they user must also have told
 * us what to do with it by setting one of these.
 */

#ifndef CONFIG_FS_NXFFS
#  undef CONFIG_SAMA5D3xEK_AT24_NXFFS
#endif

#if !defined(CONFIG_SAMA5D3xEK_AT24_FTL) && !defined(CONFIG_SAMA5D3xEK_AT24_NXFFS)
#  undef HAVE_AT24
#endif

#if defined(CONFIG_SAMA5D3xEK_AT24_FTL) && defined(CONFIG_SAMA5D3xEK_AT24_NXFFS)
#  warning Both CONFIG_SAMA5D3xEK_AT24_FTL and CONFIG_SAMA5D3xEK_AT24_NXFFS are set
#  warning Ignoring CONFIG_SAMA5D3xEK_AT24_NXFFS
#  undef CONFIG_SAMA5D3xEK_AT24_NXFFS
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
#  define _AT24_MINOR (_AT25_MINOR+1)
#else
#  define _AT24_MINOR _AT25_MINOR
#endif

#ifdef HAVE_AT24
#  define AT24_MINOR  _AT24_MINOR
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

/* Audio */
/* PCM/WM8904 driver */

#ifndef CONFIG_AUDIO_WM8904
#  undef HAVE_WM8904
#endif

#ifdef HAVE_WM8904
#  ifndef CONFIG_SAMA5_TWI0
#    warning CONFIG_SAMA5_TWI0 is required for audio support
#    undef HAVE_WM8904
#  endif

#  ifndef CONFIG_SAMA5_SSC0
#    warning CONFIG_SAMA5_SSC0 is required for audio support
#    undef HAVE_WM8904
#  endif

#  if !defined(CONFIG_SAMA5_PIOD_IRQ)
#    warning CONFIG_SAMA5_PIOD_IRQ is required for audio support
#    undef HAVE_HSMCI
#  endif

#  ifndef CONFIG_AUDIO_FORMAT_PCM
#    warning CONFIG_AUDIO_FORMAT_PCM is required for audio support
#    undef HAVE_WM8904
#  endif

#  ifndef CONFIG_SAMA5D3xEK_WM8904_I2CFREQUENCY
#    warning Defaulting to maximum WM8904 I2C frequency
#    define CONFIG_SAMA5D3xEK_WM8904_I2CFREQUENCY 400000
#  endif

#  if CONFIG_SAMA5D3xEK_WM8904_I2CFREQUENCY > 400000
#    warning WM8904 I2C frequency cannot exceed 400KHz
#    undef CONFIG_SAMA5D3xEK_WM8904_I2CFREQUENCY
#    define CONFIG_SAMA5D3xEK_WM8904_I2CFREQUENCY 400000
#  endif
#endif

/* Camera */

#define OV2640_BUS 1

#ifndef CONFIG_SAMA5D3xEK_OV2640_DEMO
#  undef HAVE_CAMERA
#endif

#if defined(HAVE_CAMERA) && !defined(CONFIG_SAMA5_ISI)
#  warning OV2640 camera demo requires CONFIG_SAMA5_ISI
#  undef HAVE_CAMERA
#endif

#if defined(HAVE_CAMERA) && !defined(CONFIG_SAMA5_TWI1)
#  warning OV2640 camera demo requires CONFIG_SAMA5_TWI1
#  undef HAVE_CAMERA
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
 *
 *     N.B. PE24 Drives the RED Led on the CM (SODIMM200), but unfortunately
 *     it is also connected to ISI_RST on the MB (Main Board) and controlling
 *     it will reset a Camera connected to the ISI
 */

#define PIO_BLUE     (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_SET | \
                      PIO_PORT_PIOE | PIO_PIN25)

#ifndef CONFIG_SAMA5D3xEK_NOREDLED
#  define PIO_RED    (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_CLEAR | \
                      PIO_PORT_PIOE | PIO_PIN24)
#endif

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
 *   - PE27.  Pressing the switch connects PE27 to ground.  Therefore, PE27
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

/* USB Ports ************************************************************************/
/* The SAMA5D3 series-MB features three USB communication ports:
 *
 *   1. Port A Host High Speed (EHCI) and Full Speed (OHCI) multiplexed with
 *      USB Device High Speed Micro AB connector, J20
 *
 *   2. Port B Host High Speed (EHCI) and Full Speed (OHCI) standard type A
 *      connector, J19 upper port
 *
 *   3. Port C Host Full Speed (OHCI) only standard type A connector, J19
 *      lower port
 *
 * All three USB host ports are equipped with 500 mA high-side power switch
 * for self-powered and buspowered applications. The USB device port feature
 * VBUS inserts detection function.
 *
 * Port A
 *
 *   PIO  Signal Name Function
 *   ---- ----------- -------------------------------------------------------
 *   PD29  VBUS_SENSE VBus detection
 *   PD25  EN5V_USBA  VBus power enable (via MN15 AIC1526 Dual USB High-Side
 *                    Power Switch.  The other channel of the switch is for
 *                    the LCD)
 */

#define PIO_USBA_VBUS_SENSE \
                     (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOD | PIO_PIN29)
#define IRQ_USBA_VBUS_SENSE \
                     SAM_IRQ_PD29

#define PIO_USBA_VBUS_ENABLE \
                     (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_CLEAR | \
                      PIO_PORT_PIOD | PIO_PIN25)

/* Port B
 *
 *   PIO  Signal Name Function
 *   ---- ----------- -------------------------------------------------------
 *   PD26 EN5V_USBB   VBus power enable (via MN14 AIC1526 Dual USB High-Side
 *                    Power Switch).  To the A1 pin of J19 Dual USB A
 *                    connector
 */

#define PIO_USBB_VBUS_ENABLE \
                     (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_CLEAR | \
                      PIO_PORT_PIOD | PIO_PIN26)

/* Port C
 *
 *   PIO  Signal Name Function
 *   ---- ----------- -------------------------------------------------------
 *   PD27 EN5V_USBC   VBus power enable (via MN14 AIC1526 Dual USB High-Side
 *                    Power Switch).  To the B1 pin of J19 Dual USB A
 *                    connector
 */

#define PIO_USBC_VBUS_ENABLE \
                     (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_CLEAR | \
                      PIO_PORT_PIOD | PIO_PIN27)

/*  Both Ports B and C
 *
 *   PIO  Signal Name Function
 *   ---- ----------- -------------------------------------------------------
 *   PD28 OVCUR_USB   Combined overrcurrent indication from port A and B
 */

#define PIO_USBBC_VBUS_OVERCURRENT \
                     (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOD | PIO_PIN28)
#define IRQ_USBBC_VBUS_OVERCURRENT \
                     SAM_IRQ_PD28

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

/* WM8904 Audio Codec ***************************************************************/
/* SAMA5D3-EK Interface
 *   ------------- ---------------- -----------------
 *   WM8904        SAMA5D3          NuttX Pin Name
 *   ------------- ---------------- -----------------
 *    3 SDA        PA30 TWD0        PIO_TWI0_D
 *    2 SCLK       PA31 TWCK0       PIO_TWI0_CK
 *   28 MCLK       PD30 PCK0        PIO_PMC_PCK0
 *   29 BCLK/GPIO4 PC16 TK          PIO_SSC0_TK
 *   "" "        " PC19 RK          PIO_SSC0_RK
 *   30 LRCLK      PC17 TF          PIO_SSC0_TF
 *   "" "   "      PC20 RF          PIO_SSC0_RF
 *   31 ADCDAT     PC21 RD          PIO_SSC0_RD
 *   32 DACDAT     PC18 TD          PIO_SSC0_TD
 *    1 IRQ/GPIO1  PD16 INT_AUDIO   N/A
 *   ------------- ---------------- -----------------
 */

/* Audio Interrupt. All interrupts are default, active high level.  Pull down
 * internally in the WM8904.  So we want no pull-up/downs and we want to
 * interrupt on the high level.
 */

#define PIO_INT_WM8904 (PIO_INPUT | PIO_CFG_DEFAULT | PIO_CFG_DEGLITCH | \
                        PIO_INT_HIGHLEVEL | PIO_PORT_PIOD | PIO_PIN16)
#define IRQ_INT_WM8904 SAM_IRQ_PD16

/* The MW8904 communicates on TWI0, I2C address 0x1a for control operations */

#define WM8904_TWI_BUS      0
#define WM8904_I2C_ADDRESS  0x1a

/* The MW8904 transfers data on SSC0 */

#define WM8904_SSC_BUS      0

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
 *   Called to configure SPI chip select PIO pins for the SAMA5D3x-EK board.
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
 *    None
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
 * Name: sam_at24_automount
 *
 * Description:
 *   Initialize and configure the AT24 serial EEPROM
 *
 ****************************************************************************/

#ifdef HAVE_AT24
int sam_at24_automount(int minor);
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
 *   Called from sam_usbinitialize very early in inialization to setup USB-related
 *   PIO pins for the SAMA5D3x-EK board.
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

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SAMA5D3X_EK_SRC_SAMA5D3X_EK_H */

