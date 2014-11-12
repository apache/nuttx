/****************************************************************************
 * configs/efm32gg-stk3700/include/board.h
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
 ****************************************************************************/

#ifndef __CONFIGS_EFM32GG_STK3700_INCLUDE_BOARD_H
#define __CONFIGS_EFM32GG_STK3700_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "chip/efm32_cmu.h"
#include "chip/efm32_usart.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Clocking *****************************************************************/
/* Clock Sources
 *   - 1-28 MHz High Frequency RC Oscillator (HFRCO)
 *   - 4-32 MHz High Frequency Crystal Oscillator (HFXO)
 *   - 32.768 kHz Low Frequency RC Oscillator (LFRCO)
 *   - 32.768 kHz Low Frequency Crystal Oscillator (LFXO)
 *   - 1KHz Ultra Low Frequency RC Oscillator (ULFRCO)
 *
 * The device boots with 14 MHz HFRCO as the HFCLK source.
 */

#define BOARD_HAVE_HFXO        1        /* Have High frequency crystal oscillator */
#define BOARD_HAVE_LFXO        1        /* Have Loq frequency crystal oscillator */

#define BOARD_HFRCO_FREQUENCY  14000000 /* 14MHz on reset */
#define BOARD_HFXO_FREQUENCY   48000000 /* 48MHz crystal on board */
#define BOARD_LFRCO_FREQUENCY  32768    /* Low frequency oscillator */
#define BOARD_LFXO_FREQUENCY   32768    /* 32MHz crystal on board */
#define BOARD_ULFRCO_FREQUNCY  1000     /* Ultra low frequency oscillator */

/* HFCLK - High Frequency Clock
 *
 * HFCLK is the selected High Frequency Clock. This clock is used by the CMU
 * and drives the two prescalers that generate HFCORECLK and HFPERCLK. The
 * HFCLK can be driven by a high-frequency oscillator (HFRCO or HFXO) or one
 * of the low-frequency oscillators (LFRCO or LFXO). By default the HFRCO is
 * selected.
 */

#define BOARD_HFCLKSEL            _CMU_CMD_HFCLKSEL_HFXO
#define BOARD_HFCLKDIV            0     /* Does not apply to EFM32G */
#define BOARD_HFCLK_FREQUENCY     BOARD_HFXO_FREQUENCY

/* HFCORECLK - High Frequency Core Clock
 *
 * HFCORECLK is a prescaled version of HFCLK. This clock drives the Core
 * Modules, which consists of the CPU and modules that are tightly coupled
 * to the CPU, e.g. MSC, DMA etc.  The frequency of HFCORECLK is set using
 * the CMU_HFCORECLKDIV register.
 */

#define BOARD_HFCORECLKDIV        _CMU_HFCORECLKDIV_HFCORECLKDIV_DEFAULT
#define BOARD_HFCORECLK_FREQUENCY BOARD_HFXO_FREQUENCY

/* HFPERCLK - High Frequency Peripheral Clock
 *
 * Like HFCORECLK, HFPERCLK can also be a prescaled version of HFCLK. This
 * clock drives the High-Frequency Peripherals. The frequency of HFPERCLK is
 * set using the CMU_HFPERCLKDIV register.
 */

#define BOARD_HFPERCLKDIV        _CMU_HFPERCLKDIV_HFPERCLKDIV_DEFAULT
#define BOARD_HFPERCLK_FREQUENCY BOARD_HFXO_FREQUENCY

/* LFACLK - Low Frequency A Clock
 *
 * LFACLK is the selected clock for the Low Energy A Peripherals. There are
 * four selectable sources for LFACLK: LFRCO, LFXO, HFCORECLK/2 and ULFRCO.
 * From reset, the LFACLK source is set to LFRCO. However, note that the
 * LFRCO is disabled from reset. The selection is configured using the LFA
 * field in CMU_LFCLKSEL. The HFCORECLK/2 setting allows the Low Energy A
 * Peripherals to be used as high-frequency peripherals.
 *
 * Use _CMU_LFCLKSEL_LFA_DISABLED to disable.
 * ULFRCO is a special case.
 */

#if BOARD_HAVE_LFXO
#   define BOARD_LFACLKSEL           _CMU_LFCLKSEL_LFA_LFXO
#   undef  BOARD_LFACLK_ULFRCO
#   define BOARD_LFACLK_FREQUENCY    BOARD_LFXO_FREQUENCY
#else
#   define BOARD_LFACLKSEL           _CMU_LFCLKSEL_LFA_LFRCO
#   undef  BOARD_LFACLK_ULFRCO
#   define BOARD_LFACLK_FREQUENCY    BOARD_LFRCO_FREQUENCY
#endif

/* LFBCLK - Low Frequency B Clock
 *
 * LFBCLK is the selected clock for the Low Energy B Peripherals. There are
 * four selectable sources for LFBCLK: LFRCO, LFXO, HFCORECLK/2 and ULFRCO.
 * From reset, the LFBCLK source is set to LFRCO. However, note that the
 * LFRCO is disabled from reset. The selection is configured using the LFB
 * field in CMU_LFCLKSEL. The HFCORECLK/2 setting allows the Low Energy B
 * Peripherals to be used as high-frequency peripherals.
 *
 * Use _CMU_LFCLKSEL_LFA_DISABLED to disable.
 * ULFRCO is a special case.
 */

#define BOARD_LFBCLKSEL           _CMU_LFCLKSEL_LFB_LFXO
#undef  BOARD_LFBCLK_ULFRCO
#define BOARD_LFBCLK_FREQUENCY    BOARD_LFXO_FREQUENCY

/* PCNTnCLK - Pulse Counter n Clock
 *
 * Each available pulse counter is driven by its own clock, PCNTnCLK where
 * n is the pulse counter instance number. Each pulse counter can be
 * configured to use an external pin (PCNTn_S0) or LFACLK as PCNTnCLK.
 */

/* WDOGCLK - Watchdog Timer Clock
 *
 * The Watchdog Timer (WDOG) can be configured to use one of three different
 * clock sources: LFRCO, LFXO or ULFRCO. ULFRCO (Ultra Low Frequency RC
 * Oscillator) is a separate 1 kHz RC oscillator that also runs in EM3.
 */

/* AUXCLK - Auxiliary Clock
 *
 * AUXCLK is a 1-28 MHz clock driven by a separate RC oscillator, AUXHFRCO.
 * This clock is used for flash programming and Serial Wire Output (SWO).
 * During flash programming this clock will be active. If the AUXHFRCO has
 * not been enabled explicitly by software, the MSC will automatically
 * start and stop it. The AUXHFRCO is enabled by writing a 1 to AUXHFRCOEN
 * in CMU_OSCENCMD. This explicit enabling is required when SWO is used.
 */

/* LEDs *********************************************************************/
/* The EFM32 Giant Gecko Start Kit has two yellow LEDs marked LED0 and LED1.
 * These LEDs are controlled by GPIO pins on the EFM32.  The LEDs are
 * connected to pins PE2 and PE3 in an active high configuration:
 *
 * ------------------------------------- --------------------
 * EFM32 PIN                             BOARD SIGNALS
 * ------------------------------------- --------------------
 * E2/BCK_VOUT/EBI_A09 #0/               MCU_PE2 UIF_LED0
 *   TIM3_CC2 #1/U1_TX #3/ACMP0_O #1
 * E3/BCK_STAT/EBI_A10 #0/U1_RX #3/      MCU_PE3 UIF_LED1
 *   ACMP1_O #1
 * ------------------------------------- --------------------
 *
 * All LEDs are grounded and so are illuminated by outputting a high
 * value to the LED.
 */

/* LED index values for use with efm32_setled() */

#define BOARD_LED0        0
#define BOARD_LED1        1
#define BOARD_NLEDS       2

/* LED bits for use with efm32_setleds() */

#define BOARD_LED0_BIT    (1 << BOARD_LED0)
#define BOARD_LED1_BIT    (1 << BOARD_LED1)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/efm32_autoleds.c.  The LEDs are used to
 * encode OS-related events as follows:
 *
 *      SYMBOL            Val    Meaning                     LED state
 *                                                         LED0     LED1
 *      ----------------- ---   -----------------------  -------- --------   */
#define LED_STARTED       0  /* NuttX has been started     OFF      OFF      */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated    OFF      OFF      */
#define LED_IRQSENABLED   0  /* Interrupts enabled         OFF      OFF      */
#define LED_STACKCREATED  1  /* Idle stack created         ON       OFF      */
#define LED_INIRQ         2  /* In an interrupt              No change       */
#define LED_SIGNAL        2  /* In a signal handler          No change       */
#define LED_ASSERTION     2  /* An assertion failed          No change       */
#define LED_PANIC         3  /* The system has crashed     OFF      Blinking */
#undef  LED_IDLE             /* MCU is is sleep mode         Not used        */

/* Buttons ******************************************************************/
/* The EFM32 Giant Gecko Start Kit has two buttons marked PB0 and PB1. They
 * are connected to the EFM32, and are debounced by RC filters with a time
 * constant of 1ms. The buttons are connected to pins PB9 and PB10:
 *
 * ------------------------------------- --------------------
 * EFM32 PIN                             BOARD SIGNALS
 * ------------------------------------- --------------------
 * B9/EBI_A03/U1_TX #2                   MCU_PB9  UIF_PB0
 * B10/EBI_A04/U1_RX #2                  MCU_PB10 UIF_PB1
 * ------------------------------------- --------------------
 *
 * Buttons are connected to ground so they will read low when closed.
 */

#define BUTTON_PB0        0
#define BUTTON_PB1        1
#define NUM_BUTTONS       2

#define BUTTON_PB0_BIT    (1 << BUTTON_PB0)
#define BUTTON_PB1_BIT    (1 << BUTTON_PB1)

/* Pin routing **************************************************************/
/* UART0:
 *
 *   The kit contains a board controller that is responsible for performing
 *   various board level tasks, such as handling the debugger and the Advanced
 *   Energy Monitor. An interface is provided between the EFM32 and the board
 *   controller in the form of a UART connection. The connection is enabled by
 *   setting the EFM_BC_EN (PF7) line high, and using the lines EFM_BC_TX
 *   (PE0) and EFM_BC_RX (PE1) for communicating.
 *
 *   U0_TX #1 PE0 MCU_PE0, UART0_TX #0, EFM_BC_RX, BC_UART_RX
 *   U0_RX #1 PE1 MCU_PE1, UART0_TX #1, EFM_BC_TX, BC_UART_TX
 */

#define BOARD_UART0_RX_GPIO          (GPIO_PORTE|GPIO_PIN1)
#define BOARD_UART0_TX_GPIO          (GPIO_PORTE|GPIO_PIN0)
#define BOARD_UART0_ROUTE_LOCATION   _USART_ROUTE_LOCATION_LOC1

/* LEUART0:
 *
 *   LEU0_TX #0 PD4 Available on TP122 and EXP pin 12
 *   LEU0_RX #0 PD5 Available on TP123 and EXP pin 14
 */

#define BOARD_LEUART0_RX_GPIO        (GPIO_PORTD|GPIO_PIN5)
#define BOARD_LEUART0_TX_GPIO        (GPIO_PORTD|GPIO_PIN4)
#define BOARD_LEUART0_ROUTE_LOCATION _LEUART_ROUTE_LOCATION_LOC0

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  efm32_ledinit, efm32_setled, and efm32_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then the following interfaces
 *   are available to control the LEDs from user applications.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void efm32_ledinit(void);
void efm32_setled(int led, bool ledon);
void efm32_setleds(uint8_t ledset);
#endif

#endif /* __CONFIGS_EFM32GG_STK3700_INCLUDE_BOARD_H */
