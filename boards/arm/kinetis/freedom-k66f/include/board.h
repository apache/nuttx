/****************************************************************************
 * boards/arm/kinetis/freedom-k66f/include/board.h
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

#ifndef __BOARDS_ARM_KINETIS_FREEDOM_K66F_INCLUDE_BOARD_H
#define __BOARDS_ARM_KINETIS_FREEDOM_K66F_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#include <arch/chip/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Freedom K66F uses a 12Mhz external Oscillator.
 * The Kinetis MCU startup from an internal digitally-controlled oscillator
 * (DCO). NuttX will enable the main external oscillator (EXTAL0/XTAL0).
 * The external oscillator/resonator can range from 32.768 KHz up to 50 MHz.
 * The default external source for the MCG oscillator inputs is 12 MHz
 * oscillator
 *
 * X501 a High-frequency, low-power Xtal
 */

#define BOARD_EXTAL_LP       1
#define BOARD_EXTAL_FREQ     12000000       /* 12MHz Oscillator */
#define BOARD_XTAL32_FREQ    32768          /* 32KHz RTC Oscillator */

/* PLL Configuration.  Either the external clock or crystal frequency is used
 * to select the PRDIV value.
 * Only reference clock frequencies are supported that will produce a
 * KINETIS_MCG_PLL_REF_MIN >= PLLIN <=KINETIS_MCG_PLL_REF_MAX  reference
 * clock to the PLL.
 *
 *   PLL Input frequency:   PLLIN  = REFCLK / PRDIV = 12 MHz  / 1  = 12 MHz
 *   PLL Output frequency:  PLLOUT = PLLIN  * VDIV  = 12 MHz  * 30 = 360 MHz
 *   MCG Frequency:         PLLOUT = 180 MHz = 360 MHz /
 *                                             KINETIS_MCG_PLL_INTERNAL_DIVBY
 *
 * PRDIV register value is the divider minus KINETIS_MCG_C5_PRDIV_BASE.
 * VDIV  register value is offset by KINETIS_MCG_C6_VDIV_BASE.
 */

#define BOARD_PRDIV          1    /* PLL External Reference Divider */
#define BOARD_VDIV           30   /* PLL VCO Divider (frequency multiplier) */

/* Define additional MCG_C2 Setting */

#define BOARD_MCG_C2_FCFTRIM 0              /* Do not enable FCFTRIM */
#define BOARD_MCG_C2_LOCRE0  MCG_C2_LOCRE0  /* Enable reset on loss of clock */

#define BOARD_PLLIN_FREQ     (BOARD_EXTAL_FREQ / BOARD_PRDIV)
#define BOARD_PLLOUT_FREQ    (BOARD_PLLIN_FREQ * BOARD_VDIV)
#define BOARD_MCG_FREQ       (BOARD_PLLOUT_FREQ/KINETIS_MCG_PLL_INTERNAL_DIVBY)

/* SIM CLKDIV1 dividers */

#define BOARD_OUTDIV1        1            /* Core        = MCG,    180   MHz */
#define BOARD_OUTDIV2        3            /* Bus         = MCG / 3, 60   MHz */
#define BOARD_OUTDIV3        3            /* FlexBus     = MCG / 3, 60   MHz */
#define BOARD_OUTDIV4        7            /* Flash clock = MCG / 7, 25.7 MHz */

#define BOARD_CORECLK_FREQ   (BOARD_MCG_FREQ / BOARD_OUTDIV1)
#define BOARD_BUS_FREQ       (BOARD_MCG_FREQ / BOARD_OUTDIV2)
#define BOARD_FLEXBUS_FREQ   (BOARD_MCG_FREQ / BOARD_OUTDIV3)
#define BOARD_FLASHCLK_FREQ  (BOARD_MCG_FREQ / BOARD_OUTDIV4)

/* Use BOARD_MCG_FREQ as the output SIM_SOPT2 MUX selected by
 * SIM_SOPT2[PLLFLLSEL]
 */

#define BOARD_SOPT2_PLLFLLSEL   SIM_SOPT2_PLLFLLSEL_MCGPLLCLK
#define BOARD_SOPT2_FREQ        BOARD_MCG_FREQ

/* N.B. The above BOARD_SOPT2_FREQ precludes use of USB with a 12 MHz Xtal
 * Divider output clock = Divider input clock × [ (USBFRAC+1) / (USBDIV+1) ]
 *     SIM_CLKDIV2_FREQ = BOARD_SOPT2_FREQ × [ (USBFRAC+1) / (USBDIV+1) ]
 *                48Mhz = 168Mhz X [(1 + 1) / (6 + 1)]
 *                48Mhz = 168Mhz / (6 + 1) * (1 + 1)
 */

#if (BOARD_MCG_FREQ == 168000000L)
#  define BOARD_SIM_CLKDIV2_USBFRAC     2
#  define BOARD_SIM_CLKDIV2_USBDIV      7
#  define BOARD_SIM_CLKDIV2_FREQ        (BOARD_SOPT2_FREQ / \
                                         BOARD_SIM_CLKDIV2_USBDIV * \
                                         BOARD_SIM_CLKDIV2_USBFRAC)
#endif

/* Divider output clock = Divider input clock*((PLLFLLFRAC+1)/(PLLFLLDIV+1))
 *  SIM_CLKDIV3_FREQ = BOARD_SOPT2_FREQ × [ (PLLFLLFRAC+1) / (PLLFLLDIV+1)]
 *            90 MHz = 180 MHz X [(0 + 1) / (1 + 1)]
 *            90 MHz = 180 MHz / (1 + 1) * (0 + 1)
 */

#define BOARD_SIM_CLKDIV3_PLLFLLFRAC  1
#define BOARD_SIM_CLKDIV3_PLLFLLDIV   2
#define BOARD_SIM_CLKDIV3_FREQ        (BOARD_SOPT2_FREQ / \
                                       BOARD_SIM_CLKDIV3_PLLFLLDIV * \
                                       BOARD_SIM_CLKDIV3_PLLFLLFRAC)

#define BOARD_LPUART0_CLKSRC SIM_SOPT2_LPUARTSRC_MCGCLK
#define BOARD_LPUART0_FREQ   BOARD_SIM_CLKDIV3_FREQ

#define BOARD_TPM_CLKSRC     SIM_SOPT2_TPMSRC_MCGCLK
#define BOARD_TPM_FREQ       BOARD_SIM_CLKDIV3_FREQ

/* SDHC clocking ************************************************************/

/* SDCLK configurations corresponding to various modes of operation.
 * Formula is:
 *
 *   SDCLK  frequency = (base clock) / (prescaler * divisor)
 *
 * The SDHC module is always configure configured so that the core clock is
 * the base clock.
 * Possible values for prescaler and divisor are:
 *
 *   SDCLKFS: {2, 4, 8, 16, 32, 63, 128, 256}
 *   DVS:     {1..16}
 */

/* Identification mode:
 * Optimal 400KHz, Actual 180MHz / (32 * 15) = 375 Khz
 */

#define BOARD_SDHC_IDMODE_PRESCALER    SDHC_SYSCTL_SDCLKFS_DIV32
#define BOARD_SDHC_IDMODE_DIVISOR      SDHC_SYSCTL_DVS_DIV(15)

/* MMC normal mode:
 * Optimal 20MHz, Actual 180MHz / (2 * 5) = 18 MHz
 */

#define BOARD_SDHC_MMCMODE_PRESCALER   SDHC_SYSCTL_SDCLKFS_DIV2
#define BOARD_SDHC_MMCMODE_DIVISOR     SDHC_SYSCTL_DVS_DIV(5)

/* SD normal mode (1-bit):
 * Optimal 20MHz, Actual 180MHz / (2 * 5) = 18 MHz
 */

#define BOARD_SDHC_SD1MODE_PRESCALER   SDHC_SYSCTL_SDCLKFS_DIV2
#define BOARD_SDHC_SD1MODE_DIVISOR     SDHC_SYSCTL_DVS_DIV(5)

/* SD normal mode (4-bit):
 * Optimal 25MHz, Actual 180MHz / (2 * 4) = 22.5 MHz (with DMA)
 * SD normal mode (4-bit):
 * Optimal 20MHz, Actual 180MHz / (2 * 4) = 22.5 MHz (no DMA)
 */

#ifdef CONFIG_SDIO_DMA
#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV2
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(4)
#else
#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV2
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(4)
#endif

/* PWM Configuration */

/* FTM0 Channels */

/* Channels can be modified using kinetis_k66pinmux.h */

#define GPIO_FTM0_CH0OUT PIN_FTM0_CH0_1
#define GPIO_FTM0_CH1OUT PIN_FTM0_CH1_1
#define GPIO_FTM0_CH2OUT PIN_FTM0_CH2_2
#define GPIO_FTM0_CH3OUT PIN_FTM0_CH3_1
#define GPIO_FTM0_CH4OUT PIN_FTM0_CH4_1
#define GPIO_FTM0_CH5OUT PIN_FTM0_CH5_1

/* PWM Configuration */

/* FTM3 Channels */

/* Channels can be modified using kinetis_k66pinmux.h */

#define GPIO_FTM3_CH0OUT PIN_FTM3_CH0_1
#define GPIO_FTM3_CH1OUT PIN_FTM3_CH1_1
#define GPIO_FTM3_CH2OUT PIN_FTM3_CH2_1
#define GPIO_FTM3_CH3OUT PIN_FTM3_CH3_1
#define GPIO_FTM3_CH4OUT PIN_FTM3_CH4_1

/* LED definitions **********************************************************/

/* The Freedom K66F has a single RGB LED driven by the K66F as follows:
 *
 *   LED    K66
 *   ------ -------------------------------------------------------
 *   RED    PTB22/SPI2_SOUT/FB_AD29/CMP2_OUT
 *   BLUE   PTB21/SPI2_SCK/FB_AD30/CMP1_OUT
 *   GREEN  PTE26/ENET_1588_CLKIN/UART4_CTS_b/RTC_CLKOUT/USB0_CLKIN
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs
 * in any way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_R       0
#define BOARD_LED_G       1
#define BOARD_LED_B       2
#define BOARD_NLEDS       3

/* LED bits for use with board_userled_all() */

#define BOARD_LED_R_BIT   (1 << BOARD_LED_R)
#define BOARD_LED_G_BIT   (1 << BOARD_LED_G)
#define BOARD_LED_B_BIT   (1 << BOARD_LED_B)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the Freedom K66F.  The following definitions describe how NuttX controls
 * the LEDs:
 *
 *   SYMBOL                Meaning                      LED state
 *                                                      RED   GREEN  BLUE
 *   -------------------  ----------------------------  -----------------
 */

#define LED_STARTED       1 /* NuttX has been started    OFF   OFF    OFF */
#define LED_HEAPALLOCATE  2 /* Heap has been allocated   OFF   OFF    ON  */
#define LED_IRQSENABLED   0 /* Interrupts enabled        OFF   OFF    ON  */
#define LED_STACKCREATED  3 /* Idle stack created        OFF   ON     OFF */
#define LED_INIRQ         0 /* In an interrupt          (no change)       */
#define LED_SIGNAL        0 /* In a signal handler      (no change)       */
#define LED_ASSERTION     0 /* An assertion failed      (no change)       */
#define LED_PANIC         4 /* The system has crashed    FLASH OFF    OFF */
#undef  LED_IDLE            /* K66 is in sleep mode     (Not used)        */

/* Button definitions *******************************************************/

/* Two push buttons, SW2 and SW3, are available on FRDM-K66F board, where SW2
 * is connected to PTC6 and SW3 is connected to PTA4.
 * Besides the general purpose input/output functions, SW2 and SW3 can be
 * low-power wake up signal. Also, only SW3 can be a non-maskable interrupt.
 *
 *   Switch GPIO Function
 *   ------ ---------------------------------------------------------------
 *   SW2    PTC6/SPI0_SOUT/PD0_EXTRG/I2S0_RX_BCLK/FB_AD9/I2S0_MCLK/LLWU_P10
 *   SW3    PTA4/FTM0_CH1/NMI_b/LLWU_P3
 */

#define BUTTON_SW2        0
#define BUTTON_SW3        1
#define NUM_BUTTONS       2

#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT    (1 << BUTTON_SW3)

/* Alternative pin resolution ***********************************************/

/* If there are alternative configurations for various pins in the
 * kinetis_k66pinmux.h header file, those alternative pins will be labeled
 * with a suffix like _1, _2, etc.  The logic in this file must select the
 * correct pin configuration for the board by defining a pin configuration
 * (with no suffix) that maps to the correct alternative.
 */

/* The primary serial port interface signals are PTB16 UART0_RX and PTB17
 *  UART0_TX.
 * These signals are connected to the OpenSDAv2 circuit.
 */

#define PIN_UART0_RX      PIN_UART0_RX_3
#define PIN_UART0_TX      PIN_UART0_TX_3

/* An alternative serial port might use a standard serial shield mounted
 * on the Freedom Board.  In this case, Arduino pin D1 provides UART TX and
 * pin D0 privies UART RX.
 *
 * The I/O headers on the FRDM-K66F board are arranged to enable
 * compatibility with Arduino shield. The outer rows of pins (even numbered
 * pins) on the headers, share the same mechanical spacing and placement with
 * the I/O headers on the Arduino Revision 3 (R3) standard.
 *
 * The Arduino D0 and D1 pins then correspond to pins 2 and 4 on the J1 I/O
 * connector:
 *
 *  Arduino Pin              FRDM-K66F J1 Connector
 *  ------------------------ -----------------------
 *  UART RX, Arduino D0 pin  Pin 2, PTC3, UART1_RX
 *  UART TX, Arduino D1 pin  Pin 4, PTC4, UART1_TX
 *  ------------------------ -----------------------
 *
 */

#define PIN_UART1_RX      PIN_UART1_RX_1
#define PIN_UART1_TX      PIN_UART1_TX_1

/* Bluetooth header
 *
 *  J199 Pin Name   K66   Name
 *  -------- ----- ------ ---------
 *      3    BT_TX  PTC14 UART4_RX
 *      4    BT_RX  PTC15 UART4_TX
 *  -------- ----- ------ ---------
 */

#define PIN_UART4_RX      PIN_UART4_RX_1
#define PIN_UART4_TX      PIN_UART4_TX_1

/* LPUART
 *
 *  J1 Pin     Name        K66   Name
 *  -------- ------------ ------ ---------
 *      7    I2S_RX_BCLK  PTE9 LPUART0_RX
 *      11   I2S_RX_FS    PTE8 LPUART0_TX
 *  -------- ----- ------ ---------
 */

#define PIN_LPUART0_RX      PIN_LPUART0_RX_1
#define PIN_LPUART0_TX      PIN_LPUART0_TX_1

/* I2C INERTIAL SENSOR (Gyroscope)
 *
 *  Pin Name   K66   Name
 *  ---- ----- ------ ---------
 *   11  SCL    PTD8  2C0_SCL
 *   12  SDA    PTD9  2C0_SDA
 */

#define PIN_I2C0_SCL      PIN_I2C0_SCL_3
#define PIN_I2C0_SDA      PIN_I2C0_SDA_3

/* RF/WIFI
 *
 *  J6 Pin Name   K66   Name
 *  ------ ----- ------ ---------
 *   1     GND
 *   2     P3V3
 *   3     CE     PTB20 PTB20
 *   4     CS     PTD4  SPI1_PCS0 (use as GPIO)
 *   5     SCK    PTD5  SPI1_SCK
 *   6     MOSI   PTD6  SPI1_MOSI
 *   7     MISO   PTD7  SPI1_MISO
 *   8     IRQ    PTC18 PTC18
 */

#define PIN_SPI1_SCK     PIN_SPI1_SCK_3
#define PIN_SPI1_OUT     PIN_SPI1_SOUT_3
#define PIN_SPI1_SIN     PIN_SPI1_SIN_3

/* Ethernet MAC/KSZ8081 PHY
 *  ------------ ----------- ---------------------------------------
 * KSZ8081      Board         K66F Pin
 * Pin Signal   Signal(s)     Function                  pinmux Name
 * --- -------- ------------ ---------------------------------------
 *  1  VDD_1V2  VDDPLL_1.2V  ---                       ---
 *  2  VDDA_3V3 VDDA_ENET    ---                       ---
 *  3  RXM      ENET1_RX-    ---                       ---
 *  4  RXP      ENET1_RX+    ---                       ---
 *  5  TXM      ENET1_TX-    ---                       ---
 *  6  TXP      ENET1_TX+    ---                       ---
 *  7  X0       RMII_XTAL0   ---                       ---
 *  8  XI       RMII_XTAL1   ---                       ---
 *  9  REXT     ---          Apparently not connected  ---
 * 10  MDIO     RMII0_MDIO   PTB0/RMII0_MDIO           PIN_RMII0_MDIO
 * 11  MDC      RMII0_MDC    PTB1/RMII0_MDC            PIN_RMII0_MDC
 * 12  RXD1     RMII0_RXD_1  PTA12/RMII0_RXD1          PIN_RMII0_RXD1
 * 13  RXD0     RMII0_RXD_0  PTA13/RMII0_RXD0          PIN_RMII0_RXD0
 * 14  VDDIO    VDDIO_ENET   ---                       ---
 * 15  CRS_DIV               PTA14/RMII0_CRS_DV        PIN_RMII0_CRS_DV
 * 16  REF_CLK  PTE26        PTE26(Ethernet clock)     PTE26/ENET_1588_CLKIN
 * 17  RXER     RMII0_RXER   PTA5/RMII0_RXER           PIN_RMII0_RXER
 * 18  INTRP    RMII0_INT_B, J14 Pin 2, Apparently not ---
 *              PHY_INT_1    available unless jumpered ---
 * 19  TXEN     RMII0_TXEN   PTA15/RMII0_TXEN          PIN_RMII0_TXEN
 * 20  TXD0     RMII0_TXD_0  PTA16/RMII0_TXD0          PIN_RMII0_TXD0
 * 21  TXD1     RMII0_TXD_1  PTA17/RMII0_TXD1          PIN_RMII0_TXD1
 * 22  GND1     ---          ---                       ---
 * 24  nRST     PHY_RST_B    ---                       ---
 * 25  GND2     ---          ---                       ---
 * --- -------- ------------ --------------------------------------------
 */

#define PIN_RMII0_MDIO  PIN_RMII0_MDIO_1
#define PIN_RMII0_MDC   PIN_RMII0_MDC_1

#endif /* __BOARDS_ARM_KINETIS_FREEDOM_K66F_INCLUDE_BOARD_H */
