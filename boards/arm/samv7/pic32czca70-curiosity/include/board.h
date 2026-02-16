/****************************************************************************
 * boards/arm/samv7/pic32czca70-curiosity/include/board.h
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

#ifndef __BOARDS_ARM_SAMV7_PIC32CZCA70_CURIOSITY_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAMV7_PIC32CZCA70_CURIOSITY_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* After power-on reset, the PIC32CZ CA70 device is running out of the Master
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
 * There are two on-board crystals:
 */

#define BOARD_HAVE_SLOWXTAL        1          /* Slow crystal is populated */
#define BOARD_SLOWCLK_FREQUENCY    (32768)    /* 32.768 kHz slow crystal oscillator */
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

/* MCK = 150MHz,
 * CLKDIV = 186,
 * MCI_SPEED = 150MHz / (2*186 + 1 + 2) = 400 KHz
 */

#define HSMCI_INIT_CLKDIV          ((186 << HSMCI_MR_CLKDIV_SHIFT) | HSMCI_MR_CLKODD)

/* MCK = 150MHz,
 * CLKDIV = 3 w/CLOCKODD,
 * MCI_SPEED = 150MHz /(2*3 + 0 + 2) = 18.75 MHz
 */

#define HSMCI_MMCXFR_CLKDIV        (2 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 150MHz,
 * CLKDIV = 2,
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
 * There is one green and one red LED on PIC32CZ CA70 Evaluation board. The
 * LEDs can be activated by driving the connected I/O line to GND.
 *
 *   ------ ----------
 *   PIC32  Functions
 *   GPIO
 *   ------ ----------
 *   PD23   Green LED0
 *   PD29   Red LED1
 *   ------ ----------
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs
 * in any way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED0        0
#define BOARD_LED1        1
#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_LED0_BIT    (1 << BOARD_LED0)
#define BOARD_LED1_BIT    (1 << BOARD_LED1)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                         LED2   LED1
 *   ------------------------  --------------------------  ------ ------
 */

#define LED_STARTED          0 /* NuttX has been started   OFF    OFF    */
#define LED_HEAPALLOCATE     0 /* Heap has been allocated  OFF    OFF    */
#define LED_IRQSENABLED      0 /* Interrupts enabled       OFF    OFF    */
#define LED_STACKCREATED     1 /* Idle stack created       ON     OFF    */
#define LED_INIRQ            2 /* In an interrupt           No change    */
#define LED_SIGNAL           2 /* In a signal handler       No change    */
#define LED_ASSERTION        2 /* An assertion failed       No change    */
#define LED_PANIC            3 /* The system has crashed   N/C  Blinking */
#undef  LED_IDLE               /* MCU is is sleep mode      Not used     */

/* Thus if LED0 is statically on, NuttX has successfully booted and is,
 * apparently, running normally.  If LED1 is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 *
 * NOTE: That LED0 is not used after completion of booting and may
 * be used by other board-specific logic.
 */

/* Button definitions *******************************************************/

/* Buttons
 *
 * PIC32CZ CA70 Curiosity contains three mechanical buttons. One button is
 * the RESET button connected to the PIC32CZ reset line and the others are
 * generic user configurable buttons. When a button is pressed it will drive
 * the I/O line to GND.
 *
 *   ------ -----------
 *   PIC32  Function
 *   GPIO
 *   ------ -----------
 *   RESET  RESET
 *   PA01   SW0
 *   PA09   SW1
 *   ------ -----------
 */

#define BUTTON_SW0        0
#define BUTTON_SW1        1
#define NUM_BUTTONS       2

#define BUTTON_SW0_BIT    (1 << BUTTON_SW0)
#define BUTTON_SW1_BIT    (1 << BUTTON_SW1)

/* PIO Disambiguation *******************************************************/

/* UART3
 *
 * Serial console over USB connector is on PA05 (RX) and PA06 (TX) pins
 */

#define GPIO_UART1_TXD GPIO_UART1_TXD_3

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMV7_PIC32CZCA70_CURIOSITY_INCLUDE_BOARD_H */
