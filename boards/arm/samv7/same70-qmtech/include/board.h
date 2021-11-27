/****************************************************************************
 * boards/arm/samv7/same70-qmtech/include/board.h
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

#ifndef __BOARDS_ARM_SAMV7_SAME70_QMTECH_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAMV7_SAME70_QMTECH_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* After power-on reset, the SAME70N device is running out of the Master
 * Clock using the Fast RC Oscillator running at 4 MHz.
 *
 *   MAINOSC:  Frequency = 12MHz (crystal)
 *
 * 300MHz Settings:
 *   PLLA: PLL Divider = 1, Multiplier = 20 to generate PLLACK = 240MHz
 *   Master Clock (MCK): Source = PLLACK,
 *                       Prescalar = 1 to generate MCK = 120MHz
 *   CPU clock: 120MHz
 *
 * There can be two on-board crystals.  However, the 32.768 crystal is not
 * populated on the stock SAME70.  The fallback is to use th on-chip, slow RC
 * oscillator which has a frequency of 22-42 KHz, nominally 32 KHz.
 */

#undef  BOARD_HAVE_SLOWXTAL                   /* Slow crystal not populated */
#define BOARD_SLOWCLK_FREQUENCY    (32000)    /* 32 KHz RC oscillator (nominal)  */
#define BOARD_MAINOSC_FREQUENCY    (12000000) /* 12 MHz main oscillator */

/* Main oscillator register settings.
 *
 * The main oscillator could be either the embedded 4/8/12 MHz fast RC
 * oscillators or an external 3-20 MHz crystal or ceramic resonator.
 * The external clock source is selected by default in sam_clockconfig.c.
 * Here we need to specify the main oscillator start-up time.
 *
 * REVISIT... this is old information:
 * The start up time should be should be:
 *
 *   Start Up Time = 8 * MOSCXTST / SLCK = 56 Slow Clock Cycles.
 */

#define BOARD_CKGR_MOR_MOSCXTST    (62 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */
#define BOARD_CKGR_MOR_MOSCXTENBY  (PMC_CKGR_MOR_MOSCXTEN)             /* Crystal Oscillator Enable */

/* PLLA configuration.
 *
 *   Divider = 1
 *   Multiplier = 25
 *
 * Yields:
 *
 *   PLLACK = 25 * 12MHz / 1 = 300MHz
 */

#define BOARD_CKGR_PLLAR_STMODE    PMC_CKGR_PLLAR_STMODE_FAST
#define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#define BOARD_CKGR_PLLAR_MUL       PMC_CKGR_PLLAR_MUL(24)
#define BOARD_CKGR_PLLAR_DIV       PMC_CKGR_PLLAR_DIV_BYPASS

/* PMC master clock register settings.
 *
 *  BOARD_PMC_MCKR_CSS - The source of main clock input.  This may be one of:
 *
 *    PMC_MCKR_CSS_SLOW   Slow Clock
 *    PMC_MCKR_CSS_MAIN   Main Clock
 *    PMC_MCKR_CSS_PLLA   PLLA Clock
 *    PMC_MCKR_CSS_UPLL   Divided UPLL Clock
 *
 *  BOARD_PMC_MCKR_PRES - Source clock pre-scaler.  May be one of:
 *
 *    PMC_MCKR_PRES_DIV1  Selected clock
 *    PMC_MCKR_PRES_DIV2  Selected clock divided by 2
 *    PMC_MCKR_PRES_DIV4  Selected clock divided by 4
 *    PMC_MCKR_PRES_DIV8  Selected clock divided by 8
 *    PMC_MCKR_PRES_DIV16 Selected clock divided by 16
 *    PMC_MCKR_PRES_DIV32 Selected clock divided by 32
 *    PMC_MCKR_PRES_DIV64 Selected clock divided by 64
 *    PMC_MCKR_PRES_DIV3  Selected clock divided by 3
 *
 *  The prescaler determines (1) the CPU clock and (2) the input into the
 *  second divider that then generates the Master Clock (MCK).  MCK is the
 *  source clock of the peripheral clocks.
 *
 *  BOARD_PMC_MCKR_MDIV - MCK divider.  May be one of:
 *
 *    PMC_MCKR_MDIV_DIV1  Master Clock = Prescaler Output Clock / 1
 *    PMC_MCKR_MDIV_DIV2  Master Clock = Prescaler Output Clock / 2
 *    PMC_MCKR_MDIV_DIV4  Master Clock = Prescaler Output Clock / 4
 *    PMC_MCKR_MDIV_DIV3  Master Clock = Prescaler Output Clock / 3
 */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA   /* Source = PLLA */
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV1  /* Prescaler = /1 */
#define BOARD_PMC_MCKR_MDIV        PMC_MCKR_MDIV_DIV2  /* MCK divider = /2 */

/* USB clocking */

#define BOARD_PMC_MCKR_UPLLDIV2    0           /* UPLL clock not divided by 2 */

/* Resulting frequencies */

#define BOARD_PLLA_FREQUENCY       (300000000) /* PLLACK:  25 * 12Mhz / 1 */
#define BOARD_CPU_FREQUENCY        (300000000) /* CPU:     PLLACK / 1 */
#define BOARD_MCK_FREQUENCY        (150000000) /* MCK:     PLLACK / 1 / 2 */
#undef  BOARD_UPLL_FREQUENCY                   /* To be provided */

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV) + CLOCKODD + 2).
 *
 *   MCI_SPEED = MCK / (2*CLKDIV + CLOCKODD + 2)
 *
 * Where CLKDIV has a range of 0-255.
 */

/* MCK = 150MHz, CLKDIV = 186,
 * MCI_SPEED = 150MHz / (2*186 + 1 + 2) = 400 KHz
 */

#define HSMCI_INIT_CLKDIV          ((186 << HSMCI_MR_CLKDIV_SHIFT) | HSMCI_MR_CLKODD)

/* MCK = 150MHz, CLKDIV = 3 w/CLOCKODD,
 * MCI_SPEED = 150MHz /(2*3 + 0 + 2) = 18.75 MHz
 */

#define HSMCI_MMCXFR_CLKDIV        (2 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 150MHz, CLKDIV = 2,
 * MCI_SPEED = 150MHz /(2*2 + 0 + 2) = 25 MHz
 */

#define HSMCI_SDXFR_CLKDIV         (2 << HSMCI_MR_CLKDIV_SHIFT)
#define HSMCI_SDWIDEXFR_CLKDIV     HSMCI_SDXFR_CLKDIV

/* FLASH wait states.
 *
 * Wait states Max frequency at 105 centigrade (STH conditions)
 *
 *           VDDIO
 *      1.62V     2.7V
 * --- -------  -------
 *  0   26 MHz   30 MHz
 *  1   52 MHz   62 MHz
 *  2   78 MHz   93 MHz
 *  3  104 MHz  124 MHz
 *  4  131 MHz  150 MHz
 *  5  150 MHz  --- MHz
 *
 * Given: VDDIO=3.3V, VDDCORE=1.2V, MCK=150MHz
 */

#define BOARD_FWS                  4

/* LED definitions **********************************************************/

/* LEDs
 *
 * A single LED is available driven by PA15.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED0        0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_LED0_BIT    (1 << BOARD_LED0)

/* This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *   ------------------- ---------------------------- ------
 *   SYMBOL                  Meaning                  LED
 *   ------------------- ---------------------------- ------
 */

#define LED_STARTED      0 /* NuttX has been started  OFF      */
#define LED_HEAPALLOCATE 0 /* Heap has been allocated OFF      */
#define LED_IRQSENABLED  0 /* Interrupts enabled      OFF      */
#define LED_STACKCREATED 1 /* Idle stack created      ON       */
#define LED_INIRQ        2 /* In an interrupt         N/C      */
#define LED_SIGNAL       2 /* In a signal handler     N/C      */
#define LED_ASSERTION    2 /* An assertion failed     N/C      */
#define LED_PANIC        3 /* The system has crashed  FLASH    */
#undef  LED_IDLE           /* MCU is is sleep mode    Not used */

/* Thus is LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/* Button definitions *******************************************************/

/* Buttons
 *
 * SAM E70 QMTECH contains two mechanical buttons. One button is the RESET
 * button connected to the SAM E70 reset line and the other, PA21, is a
 * generic user configurable button.
 * When a button is pressed it will drive the I/O line to GND.
 *
 * NOTE:
 * There are no pull-up resistors connected to the generic user buttons
 * so it is necessary to enable the internal pull-up in the SAM E70 to
 * use the button.
 */

#define BUTTON_SW0        0
#define NUM_BUTTONS       1

#define BUTTON_SW0_BIT    (1 << BUTTON_SW0)

/* PIO Disambiguation *******************************************************/

/* Serial Console
 *
 * The SAME70-QMTECH has no on-board RS-232 drivers so it will be necessary
 * to use either the VCOM or an external RS-232 driver.
 * Here are some options.
 *
 *  - Arduino Serial Shield:  One option is to use an Arduino-compatible
 *    serial shield.  This will use the RXD and TXD signals available at pins
 *    0 an 1, respectively, of the Arduino "Digital Low" connector.  On the
 *    SAME70-QMTECH board, this corresponds to UART3:
 *
 *    ------ ------ ------- ------- --------
 *    Pin on SAME70 Arduino Arduino SAME70
 *    J503   PIO    Name    Pin     Function
 *    ------ ------ ------- ------- --------
 *      1    PD28   RX0     0       URXD3
 *      2    PD30   TX0     1       UTXD3
 *    ------ ------ ------- ------- --------
 *
 *    There are alternative pin selections only for UART3 TXD:
 */

#define GPIO_UART3_TXD  GPIO_UART3_TXD_1

/* - Arduino Communications.  Additional UART/USART connections are available
 *  on the Arduino Communications connection J505:
 *
 *   ------ ------ ------- ------- --------
 *   Pin on SAME70 Arduino Arduino SAME70
 *   J503   PIO    Name    Pin     Function
 *   ------ ------ ------- ------- --------
 *     3    PD18   RX1     0       URXD4
 *     4    PD19   TX1     0       UTXD4
 *     5    PD15   RX2     0       RXD2
 *     6    PD16   TX2     0       TXD2
 *     7    PB0    RX3     0       RXD0
 *     8    PB1    TX3     1       TXD0
 *   ------ ------ ------- ------- --------
 *
 *    There are alternative pin selections only for UART4 TXD:
 */

#define GPIO_UART4_TXD  GPIO_UART4_TXD_1

/*  - SAMV7-XULT EXTn connectors.  USART pins are also available the EXTn
 *    connectors.  The following are labelled in the User Guide for USART
 *    functionality:
 *
 *    ---- -------- ------ --------
 *    EXT1 EXTI1    SAME70 SAME70
 *    Pin  Name     PIO    Function
 *    ---- -------- ------ --------
 *    13   USART_RX PB00   RXD0
 *    14   USART_TX PB01   TXD0
 *
 *    ---- -------- ------ --------
 *    EXT2 EXTI2    SAME70 SAME70
 *    Pin  Name     PIO    Function
 *    ---- -------- ------ --------
 *    13   USART_RX PA21   RXD1
 *    14   USART_TX PB04   TXD1
 *
 *    There are no alternative pin selections for USART0 or USART1.
 */

/*  - VCOM.  The Virtual Com Port gateway is available on USART1:
 *
 *    ------ --------
 *    SAME70 SAME70
 *    PIO    Function
 *    ------ --------
 *    PB04   TXD1
 *    PA21   RXD1
 *    ------ --------
 *
 *    There are no alternative pin selections for USART1.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name:  sam_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the SAM4e-EK board.
 *   Because of the various rotations, clearing the display in the normal
 *   way by writing a sequences of runs that covers the entire display can
 *   be very slow.  Here the display is cleared by simply setting all GRAM
 *   memory to the specified color.
 *
 ****************************************************************************/

void sam_lcdclear(uint16_t color);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMV7_SAME70_QMTECH_INCLUDE_BOARD_H */
