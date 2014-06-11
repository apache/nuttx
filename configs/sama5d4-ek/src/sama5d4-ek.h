/************************************************************************************
 * configs/sama5d4-ek/src/sama5d4-ek.h
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

#ifndef __CONFIGS_SAMA5D4_EK_SRC_SAMA5D4_EK_H
#define __CONFIGS_SAMA5D4_EK_SRC_SAMA5D4_EK_H

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

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAMA5D4EK_NAND_AUTOMOUNT)
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

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAMA5D4EK_AT25_AUTOMOUNT)
#  undef HAVE_AT25
#endif

/* If we are going to mount the AT25, then they user must also have told
 * us what to do with it by setting one of these.
 */

#ifndef CONFIG_FS_NXFFS
#  undef CONFIG_SAMA5D4EK_AT25_NXFFS
#endif

#if !defined(CONFIG_SAMA5D4EK_AT25_FTL) && !defined(CONFIG_SAMA5D4EK_AT25_NXFFS)
#  undef HAVE_AT25
#endif

#if defined(CONFIG_SAMA5D4EK_AT25_FTL) && defined(CONFIG_SAMA5D4EK_AT25_NXFFS)
#  warning Both CONFIG_SAMA5D4EK_AT25_FTL and CONFIG_SAMA5D4EK_AT25_NXFFS are set
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

#if !defined(CONFIG_NET) || !defined(CONFIG_SAMA5_EMACB)
#  undef HAVE_NETWORK
#endif

/* LEDs *****************************************************************************/
/* There are 3 LEDs on the SAMA5D4-EK:
 *
 * ------------------------------ ------------------- -------------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -------------------------
 * PE28/NWAIT/RTS4/A19            1Wire_PE28          1-WIRE ROM, LCD, D8 (green)
 * PE8/A8/TCLK3/PWML3             LED_USER_PE8        LED_USER (D10)
 * PE9/A9/TIOA2                   LED_POWER_PE9       LED_POWER (D9, Red)
 * ------------------------------ ------------------- -------------------------
 *
 * - D8: D8 is shared with other functions and cannot be used if the 1-Wire ROM
 *   is used.  I am not sure of the LCD function, but the LED may not be available
 *   if the LCD is used either.  We will avoid using D8 just for simplicity.
 * - D10:  Nothing special here.  A low output illuminates.
 * - D9: The Power ON LED.  Connects to the via an IRLML2502 MOSFET.  This LED will
 *   be on when power is applied but otherwise; a low output value will turn it
 *   off.
 */

#define PIO_LED_USER  (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_SET | \
                       PIO_PORT_PIOE | PIO_PIN8)
#define PIO_LED_POWER (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_SET | \
                       PIO_PORT_PIOE | PIO_PIN9)

/* Buttons **************************************************************************/
/* A single button, PB_USER1 (PB2), is available on the SAMA5D4-EK:
 *
 * ------------------------------ ------------------- -------------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -------------------------
 * PE13/A13/TIOB1/PWML2           PB_USER1_PE13       PB_USER1
 * ------------------------------ ------------------- -------------------------
 *
 * Closing JP2 will bring PE13 to ground so 1) PE13 should have a weak pull-up,
 * and 2) when PB2 is pressed, a low value will be senses.
 */

#define PIO_BTN_USER (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOE | PIO_PIN13)
#define IRQ_BTN_USER  SAM_IRQ_PE13

/* HSMCI Card Slots *****************************************************************/
/* The SAMA4D4-EK provides a two SD memory card slots:  (1) a full size SD
 * card slot (J10), and (2) a microSD memory card slot (J11).
 *
 * The full size SD card slot connects via HSMCI0.  The card detect discrete
 * is available on PE5 (pulled high).  The write protect discrete is tied to
 * ground and is not available to software.  The slot supports 8-bit wide
 * transfer mode, but the NuttX driver currently uses only the 4-bit wide
 * transfer mode
 *
 * ------------------------------ ------------------- -------------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -------------------------
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
 * ------------------------------ ------------------- -------------------------
 */

#define PIO_MCI0_CD  (PIO_INPUT | PIO_CFG_DEFAULT | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOE | PIO_PIN5)
#define IRQ_MCI0_CD   SAM_IRQ_PE5

/* The microSD connects vi HSMCI1.  The card detect discrete is available on
 * PE14 (pulled high):
 *
 * ------------------------------ ------------------- -------------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -------------------------
 * PE14/A14/TCLK1/PWMH3           MCI1_CD_PE14        MCI1_CD
 * PE15/A15/SCK3/TIOA0            MCI1_PWR_PE15       MCI1_PWR
 * PE18/A18/TIOA5/MCI1_CK         PE18                MCI1_CK, EXP
 * PE19/A19/TIOB5/MCI1_CDA        PE19                MCI1_CDA, EXP
 * PE20/A20/TCLK5/MCI1_DA0        PE20                MCI1_DA0, EXP
 * PE21/A23/TIOA4/MCI1_DA1        PE21                MCI1_DA1, EXP
 * PE22/A24/TIOB4/MCI1_DA2        PE22                MCI1_DA2, EXP
 * PE23/A25/TCLK4/MCI1_DA3        PE23                MCI1_DA3, EXP
 * PE6/A6/TIOA3                   MCI1_CD_PE6         MCI1_CD
 * ------------------------------ ------------------- -------------------------
 */

#define PIO_MCI1_CD  (PIO_INPUT | PIO_CFG_DEFAULT | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOE | PIO_PIN14)
#define IRQ_MCI1_CD   SAM_IRQ_PE14

/* USB Ports ************************************************************************/
/* The SAMA5D4 series-MB features three USB communication ports:
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

#ifdef CONFIG_SAMA4_EMACB
/* ETH0/1: Ethernet 10/100 (EMAC) Ports
 *
 * Networking support via the can be added to NSH by selecting the following
 * configuration options.  The SAMA5D44 supports two different 10/100Base-T
 * Ethernet MAC peripherals.
 *
 * ------------------------------ ------------------- -------------------------
 * SAMA5D4 PIO                    SIGNAL              USAGE
 * ------------------------------ ------------------- -------------------------
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
 * ------------------------------ ------------------- -------------------------
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
 * ------------------------------ ------------------- -------------------------
 *
 * EMAC2 connects (directly) to a KSZ8081RNB PHY (U10) and is available at
 * the ETH0 connector.
 *
 * EMAC1 connects (indirectly) to another KSZ8081RNB PHY (U7) and is available
 * at the ETH1 connector.  The ETH1 signals go through a line driver that is
 * enabled via LCD_ETH1_CONFIG when an LCD is detected:
 *
 * - LCD_ETH1_CONFIG = 0: LCD 5v disable
 * - LCD_ETH1_CONFIG = 1 & LCD_DETECT# =0: LCD 5v enable
 */

#ifdef CONFIG_SAMA4_EMAC0
#  define PIO_INT_ETH0 (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                        PIO_INT_BOTHEDGES | PIO_PORT_PIOE | PIO_PIN1)
#  define IRQ_INT_ETH0 SAM_IRQ_PE1
#endif

#ifdef CONFIG_SAMA4_EMAC1
#  define PIO_INT_ETH1 (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                        PIO_INT_BOTHEDGES | PIO_PORT_PIOE | PIO_PIN2)
#  define IRQ_INT_ETH1 SAM_IRQ_PE2
#endif
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
 *   Called to configure SPI chip select PIO pins for the SAMA4D4-EK board.
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
 *   Per the SAMA4D4-EK User guide: "Two SDRAM/DDR2 used as main system memory.
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
 *  This logic was taken from Atmel sample code for the SAMA4D4-EK.
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
 *   Called from sam_usbinitialize very early in inialization to setup USB-related
 *   PIO pins for the SAMA4D4-EK board.
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
 * Name: board_led_initialize
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_led_initialize(void);
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

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SAMA5D4_EK_SRC_SAMA5D4_EK_H */

