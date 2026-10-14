/****************************************************************************
 * boards/arm/rm57/rm57l843-launchxl2/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM_RM57_RM57L843_LAUNCHXL2_INCLUDE_BOARD_H
#define __BOARDS_ARM_RM57_RM57L843_LAUNCHXL2_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Values below are taken directly from TI's HALCoGen HL_system.c
 * and HL_system.h, i.e. the PLL configuration this HALCoGen project was
 * actually generated with - not recomputed from scratch. Verify against
 * the LAUNCHXL2-RM57L schematic if the crystal is ever changed.
 */

/* 16 MHz crystal */
#define BOARD_FCLKIN_FREQUENCY 16000000

/* PLLCTL1/PLLCTL2 encode:
 *   NR (REFCLKDIV+1)     = 8
 *   PLLMUL (raw field)   = 0x9500
 *   OD (ODPLL+1)         = 1
 *   R  (PLLDIV+1, final) = 1
 *
 * HALCoGen computes the resulting PLL1/GCLK frequency as 300 MHz -
 * taken as-is rather than re-derived from the PLLMUL fixed-point
 * encoding.
 */

#define BOARD_PLL_NR             8
#define BOARD_PLL_PLLMUL         0x9500
#define BOARD_PLL_OD             1
#define BOARD_PLL_R              1
#define BOARD_PLL_FREQUENCY      300000000

/* HCLKCNTL = 1 -> HCLK = GCLK / (1 + 1) = 150 MHz */

#define BOARD_HCLK_FREQUENCY     150000000

/* VCLK1 is the input clock to the SCI baud rate generator */

#define BOARD_VCLK_FREQUENCY     75000000

/* Flash read wait-states (FRDCNTL RWAIT field) */

#define BOARD_FLASH_RWAIT        3

/* RTI1 clock, used for the NuttX system tick */

#define BOARD_RTICLK_FREQUENCY   75000000

#ifdef CONFIG_RM57_EMAC

/* EMAC internal logic clock (VCLKA4_DIVR_EMAC, SPNS215 Section 6.6.3):
 * MII requires exactly 25 MHz, RMII requires exactly 50 MHz.
 *
 * MII: sourced from VCLK (75 MHz, see BOARD_VCLK_FREQUENCY above)
 * divided by 3 via the VCLKA4R field - an exact match given this
 * board's PLL parameters (HCLK=150MHz, VCLK=HCLK/2=75MHz,
 * 75MHz/3=25MHz exactly).
 *
 * RMII: 75 MHz has no integer divisor that produces 50 MHz, so RMII
 * cannot be sourced from VCLK.  SPNS215 Table 6-17 offers PLL2
 * post_ODCLK/8 or post_ODCLK/16 as the only other source options for
 * this clock domain.  This port does not independently verify PLL2's
 * output frequency (the PLL2 register configuration in
 * rm57_clockconfig.c's rm57_setup_pll() is transcribed from HALCoGen
 * and has not been cross-checked bit-for-bit against SYS2_PLLCTL3's
 * documented field layout), so the values below are a starting point
 * only.  Before relying on CONFIG_RM57_EMAC_RMII on real hardware,
 * measure VCLKA4_DIVR_EMAC on the ECLK1 test pin via the system
 * module's CLKTEST register (SEL_ECP_PIN = 0b10111, "EMAC Clock
 * Output") and correct BOARD_VCLKA4_SRC/BOARD_VCLKA4_DIV below if the
 * measured frequency is not exactly 50 MHz.
 */

#  ifdef CONFIG_RM57_EMAC_RMII
#    define BOARD_VCLKA4_SRC        0xf  /* PLL2 post_ODCLK/16 (UNVERIFIED) */
#    define BOARD_VCLKA4_DIV        0    /* VCLKA4R divide-by-1 (UNVERIFIED) */
#    define BOARD_VCLKA4_FREQUENCY  50000000
#  else
#    define BOARD_VCLKA4_SRC        0x8  /* VCLK */
#    define BOARD_VCLKA4_DIV        2    /* VCLKA4R divide-by-3: 75MHz/3=25MHz */
#    define BOARD_VCLKA4_FREQUENCY  25000000
#  endif

/* EMAC MDIO module clock source (VCLK3, SPNU562A Section 31.2.1): a
 * moderate peripheral bus rate the MDIO module's own CLKDIV field
 * further divides down to the target MDIO clock (typically 1MHz, see
 * CONFIG_RM57_EMAC_MDIO_FREQ).
 */

#  define BOARD_VCLK3_DIV           1    /* VCLK3R divide-by-2: HCLK/2=75MHz */
#  define BOARD_VCLK3_FREQUENCY     75000000

/* PHY reference clock, supplied by the MCU on the ECLK1 terminal.
 *
 * This board has no crystal or oscillator of its own for the PHY: per
 * the docs/sprr397.pdf schematic (sheet 12), the DP83630's XIN (pin 34)
 * is driven by the ECLK1T net, which comes from MCU ball A12 (ECLK1)
 * through the 56 ohm series resistor RP8A, while XOUT (pin 33) and
 * CLK_OUT (pin 24) are left unconnected.  The board's only crystal is
 * the MCU's 16MHz Y1.  So without the ECLK1 setup in
 * rm57_clockconfig.c the PHY has no clock at all and is completely
 * dead - it does not answer MDIO, and MDIO_ALIVE reads 0 with every
 * other register looking perfectly configured.
 *
 * ECLK = VCLK / (ECPDIV + 1), so a divider of 3 off this board's 75MHz
 * VCLK gives the 25 MHz the PHY needs in MII mode, exactly.
 */

#  define BOARD_ECLK1_DIV           3    /* VCLK/3: 75MHz/3=25MHz */
#  define BOARD_ECLK1_FREQUENCY     25000000

#endif /* CONFIG_RM57_EMAC */

/* PIN Multiplexor Initializer **********************************************/

/* Balls listed here are switched away from their reset function by
 * rm57_iomm_initialize() during boot; everything else keeps its reset
 * function.  The definitions come from
 * arch/arm/src/rm57/hardware/rm57l843_pinmux.h.
 *
 * SCI1/LIN1 needs no entry: its RX/TX balls are dedicated on RM57L843 and
 * are not multiplexed at all.  The remaining SCI modules share their balls
 * with N2HET1/N2HET2, so they only work once re-multiplexed here:
 *
 *   SCI2/LIN2  P4  (N2HET2[19]) -> LIN2RX, T5  (N2HET2[20]) -> LIN2TX
 *   SCI3       W3  (N2HET1[6])  -> SCI3RX, N2  (N2HET1[13]) -> SCI3TX
 *   SCI4       A13 (N2HET1[17]) -> SCI4RX, B13 (N2HET1[19]) -> SCI4TX
 *
 * All of these balls are brought out on the LaunchPad BoosterPack headers.
 */

#if defined(CONFIG_RM57_SCI2) || defined(CONFIG_RM57_LIN)
#  define BOARD_PINMUX_LIN2 \
  PINMUX_P4_LIN2RX_PIN, \
  PINMUX_T5_LIN2TX_PIN,
#else
#  define BOARD_PINMUX_LIN2
#endif

#ifdef CONFIG_RM57_SCI3
#  define BOARD_PINMUX_SCI3 \
  PINMUX_W3_SCI3RX_PIN, \
  PINMUX_N2_SCI3TX_PIN,
#else
#  define BOARD_PINMUX_SCI3
#endif

#ifdef CONFIG_RM57_SCI4
#  define BOARD_PINMUX_SCI4 \
  PINMUX_A13_SCI4RX_PIN, \
  PINMUX_B13_SCI4TX_PIN,
#else
#  define BOARD_PINMUX_SCI4
#endif

/* DCAN1-DCAN3 have dedicated balls and need no entry here: DCAN1 on
 * A10 (TX) / B10 (RX), DCAN2 on H2 / H1, DCAN3 on M18 / M19.  DCAN4
 * shares ball F2 (TX) with GIOB[2] and ball W10 (RX) with GIOB[3];
 * claim them for CAN whenever DCAN4 is enabled.
 *
 * The LaunchPad carries no CAN transceiver for any of the four modules -
 * the balls run straight to the BoosterPack headers at CMOS levels.  A
 * real bus therefore needs an external transceiver; without one the only
 * usable modes are the DCAN loopback/silent test modes, which still drive
 * the TX ball so the frames can be observed with a logic analyzer.
 */

#ifdef CONFIG_RM57_DCAN4
#  define BOARD_PINMUX_DCAN4 \
  PINMUX_F2_DCAN4TX_PIN, \
  PINMUX_W10_DCAN4RX_PIN,
#else
#  define BOARD_PINMUX_DCAN4
#endif

/* MibSPI1 and MibSPI3, the two modules this board brings out complete,
 * need no entry here either.  MibSPI1's SPICLK/SIMO[0]/SOMI[0] (F18, F19,
 * G18) are not multiplexed, and its chip selects (R2, F3, G3) as well as
 * every MibSPI3 ball (V9, W8, V8, V10, V5, B2, C3) reset to their MibSPI
 * function.  Several of those balls double as alternates for MII signals,
 * which is harmless: Ethernet uses the dedicated set (see below).  The
 * exception is I2C1, which takes B2 and C3 away from MibSPI3 (see below).
 *
 * MibSPI2 and MibSPI4 are alternate functions of N2HET balls and would
 * need entries; src/rm57_spi.c does not route them.
 */

/* I2C1 and I2C2 have no dedicated balls (docs/sprr397.pdf sheets 7, 9,
 * 13 and 14):
 *
 *   I2C1  B2 (MIBSPI3NCS[2]) -> SDA  BoosterPack site 1 J2.8, J15.17
 *         C3 (MIBSPI3NCS[3]) -> SCL  BoosterPack site 1 J2.9, J15.21
 *   I2C2  G17 (MIBSPI5SIMO[3]) -> SDA  proto header J10.9, J14.30
 *         G16 (MIBSPI5SOMI[3]) -> SCL  proto header J10.8, J14.36
 *
 * The LaunchPad has no pull-up resistors on any of them; the I2C pins'
 * internal ones (~20 uA) are far too weak for a real bus, so a target
 * needs external pull-ups.
 */

#ifdef CONFIG_RM57_I2C1
#  define BOARD_PINMUX_I2C1 \
  PINMUX_B2_I2C1_SDA_PIN, \
  PINMUX_C3_I2C1_SCL_PIN,
#else
#  define BOARD_PINMUX_I2C1
#endif

#ifdef CONFIG_RM57_I2C2
#  define BOARD_PINMUX_I2C2 \
  PINMUX_G17_I2C2_SDA_PIN, \
  PINMUX_G16_I2C2_SCL_PIN,
#else
#  define BOARD_PINMUX_I2C2
#endif

/* EMAC/MDIO.
 *
 * Most MII signals on RM57L843 exist on two balls: a dedicated one that
 * carries nothing else, and an alternate that is shared with N2HET1/
 * MIBSPI.  For signals that are inputs to the MCU an input-select entry
 * (PINMUX_SIG_<signal>_<ball>_PIN, PINMMR87-91) picks which of the two
 * the peripheral actually listens to; for outputs a function-select
 * entry (PINMUX_<ball>_<signal>_PIN, PINMMR0-36) switches that ball to
 * the signal.  Balls that are dedicated need neither.
 *
 * This board wires Ethernet entirely to the dedicated set.  Read off the
 * docs/sprr397.pdf schematic, sheet 3 (U1B, MCU pinout):
 *
 *   MDIO   F4    MDCLK  T9    MII_TXCLK U7   MII_RXCLK T4
 *   MII_COL W4   MII_CRS V4   MII_RX_DV U6   MII_RX_ER U5
 *   MII_RXD[0..3] U4 T3 U3 V3
 *   MII_TXD[0..3] U8 R1 T2 G4   MII_TXEN E4
 *
 * The shared alternates (G3 for MDIO, V5 for MDCLK, K19/B11/N19/P1/A14/
 * G19/H18/F3/B4/D19/E18/R2/J19/J18/H19 for the data path) are drawn on
 * sheet 3 too, but their nets run to the BoosterPack headers on sheets
 * 7/8/14 - not to the PHY.
 *
 * That leaves almost nothing to program: every PINMMR byte resets to
 * 01h, which already selects exactly the dedicated balls this board
 * uses.  An earlier version of this table selected the *alternate* ball
 * (02h) for every MII input, which pointed the MAC's receive path at
 * BoosterPack pins and silently broke RX.
 *
 * The one entry that is genuinely required is the MII/RMII selector:
 * PINMMR160[24] resets to 1 = RMII (SPNU562A Section 6.5.3), and this
 * board is hard-wired for MII - the DP83630's MII_MODE strap (RX_DV) is
 * pulled low by RP11C on sheet 12, and there is no jumper to change it.
 *
 * The explicit SIG_* input-selects below are redundant with the reset
 * value but are kept so this table documents the ball choice rather than
 * relying on a silent default.
 */

/* RMII is not wired on this board: sheet 12 connects the full MII signal
 * set and straps the PHY for MII.  The RMII entry is kept only so the
 * Kconfig choice still builds; it selects RMII in the IOMM and nothing
 * else.
 */

#ifdef CONFIG_RM57_EMAC
#  ifdef CONFIG_RM57_EMAC_RMII
#    define BOARD_PINMUX_EMAC \
  PINMUX_ETHERNET_RMII_PIN,
#  else
#    define BOARD_PINMUX_EMAC \
  PINMUX_SIG_MDIO_F4_PIN, \
  PINMUX_SIG_MII_TX_CLK_U7_PIN, \
  PINMUX_SIG_MII_COL_W4_PIN, \
  PINMUX_SIG_MII_CRS_V4_PIN, \
  PINMUX_SIG_MII_RXCLK_T4_PIN, \
  PINMUX_SIG_MII_RXD_3_V3_PIN, \
  PINMUX_SIG_MII_RXD_2_U3_PIN, \
  PINMUX_SIG_MII_RXD_1_T3_PIN, \
  PINMUX_SIG_MII_RXD_0_U4_PIN, \
  PINMUX_SIG_MII_RX_DV_U6_PIN, \
  PINMUX_SIG_MII_RX_ER_U5_PIN, \
  PINMUX_T4_MII_RXCLK_PIN, \
  PINMUX_U7_MII_TX_CLK_PIN, \
  PINMUX_ETHERNET_MII_PIN,
#  endif
#else
#  define BOARD_PINMUX_EMAC
#endif

/* Left undefined when no multiplexed pin is in use, so that no pin-mux
 * table is generated at all.
 */

#if defined(CONFIG_RM57_SCI2) || defined(CONFIG_RM57_LIN) || \
    defined(CONFIG_RM57_SCI3) || defined(CONFIG_RM57_SCI4) || \
    defined(CONFIG_RM57_DCAN4) || defined(CONFIG_RM57_EMAC) || \
    defined(CONFIG_RM57_I2C1) || defined(CONFIG_RM57_I2C2)
#  define BOARD_PINMUX_INITIALIZER \
  BOARD_PINMUX_LIN2 \
  BOARD_PINMUX_SCI3 \
  BOARD_PINMUX_SCI4 \
  BOARD_PINMUX_DCAN4 \
  BOARD_PINMUX_I2C1 \
  BOARD_PINMUX_I2C2 \
  BOARD_PINMUX_EMAC
#endif

/* LED definitions **********************************************************/

/* The LAUNCHXL2-RM57L has two user LEDs, labeled B6 and B7 on the board
 * silkscreen (driven by GIOB[6] and GIOB[7] respectively; see
 * src/rm57l843-launchxl2.h for the GIO pin definitions).
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_B6         0
#define BOARD_LED_B7         1
#define BOARD_NLEDS          2

/* LED bits for use with board_userled_all() */

#define BOARD_LED_B6_BIT     (1 << BOARD_LED_B6)
#define BOARD_LED_B7_BIT     (1 << BOARD_LED_B7)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * src/rm57_autoleds.c. Both LEDs are driven together to encode
 * OS-related events as follows:
 */

#define LED_STARTED         0 /* NuttX has been started */
#define LED_HEAPALLOCATE    0 /* Heap has been allocated */
#define LED_IRQSENABLED     0 /* Interrupts enabled */
#define LED_STACKCREATED    1 /* Idle stack created */
#define LED_INIRQ           2 /* In an interrupt */
#define LED_SIGNAL          2 /* In a signal handler */
#define LED_ASSERTION       2 /* An assertion failed */
#define LED_PANIC           3 /* The system has crashed */
#undef  LED_IDLE              /* MCU is in sleep mode: Not used */

/* Button definitions *******************************************************/

/* Button index values for use with board_buttons() */

#define BOARD_BUTTON_B4       0
#define BOARD_BUTTON_B5       1
#define BOARD_NUM_BUTTONS     2

/* Button bits for use with board_buttons() */

#define BOARD_BUTTON_B4_BIT   (1 << BOARD_BUTTON_B4)
#define BOARD_BUTTON_B5_BIT   (1 << BOARD_BUTTON_B5)

#endif /* __BOARDS_ARM_RM57_RM57L843_LAUNCHXL2_INCLUDE_BOARD_H */
