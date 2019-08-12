/****************************************************************************
 * boards/arm/efm32/efm32-g8xx-stk/include/board.h
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

#ifndef __BOARDS_ARM_EFM32_EFM32_G8XX_STK_INCLUDE_BOARD_H
#define __BOARDS_ARM_EFM32_EFM32_G8XX_STK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "hardware/efm32_cmu.h"
#include "hardware/efm32_usart.h"

/****************************************************************************
 * Pre-processor Definitions
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
#define BOARD_HFXO_FREQUENCY   32000000 /* 32MHz crystal on board */
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

#define BOARD_LFACLKSEL           _CMU_LFCLKSEL_LFA_LFXO
#undef  BOARD_LFACLK_ULFRCO
#define BOARD_LFACLK_FREQUENCY    BOARD_LFXO_FREQUENCY

/* LFBCLK - Low Frequency B Clock
 *
 * LFBCLK is the selected clock for the Low Energy B Peripherals. There are
 * four selectable sources for LFBCLK: LFRCO, LFXO, HFCORECLK/2 and ULFRCO.
 * From reset, the LFBCLK source is set to LFRCO. However, note that the
 * LFRCO is disabled from reset. The selection is configured using the LFB
 * field in CMU_LFCLKSEL. The HFCORECLK/2 setting allows the Low Energy B
 * Peripherals to be used as high-frequency peripherals.
 *
 * Use _CMU_LFCLKSEL_LFA_DISABLED to disable
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

/* The EFM32 Gecko Starter Kit supports 4 yellow LEDs.  One side is grounded
 * so these LEDs are illuminated by outputting a high value.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED0        0
#define BOARD_LED1        1
#define BOARD_LED2        2
#define BOARD_LED3        3
#define BOARD_NLEDS       4

/* LED bits for use with board_userled_all() */

#define BOARD_LED0_BIT    (1 << BOARD_LED0)
#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 4 LEDs on
 * board the EFM32 Gecko Starter Kit.  The following definitions describe
 * how NuttX controls the LEDs in this configuration:
 */

#define LED_STARTED       0  /* LED0 */
#define LED_HEAPALLOCATE  1  /* LED1 */
#define LED_IRQSENABLED   2  /* LED0 + LED1 */
#define LED_STACKCREATED  3  /* LED2 */
#define LED_INIRQ         4  /* LED0 + LED2 */
#define LED_SIGNAL        5  /* LED1 + LED3 */
#define LED_ASSERTION     6  /* LED0 + LED2 + LED2 */
#define LED_PANIC         7  /* N/C  + N/C  + N/C + LED3 */

/* Pin routing **************************************************************/

/* UART0:
 *
 *   U0_RX #1 PE1  **AVAILABLE at TP130**
 *   U0_TX #1 PE0  **AVAILABLE at TP129**
 */

#define BOARD_UART0_RX_GPIO        (GPIO_PORTE|GPIO_PIN1)
#define BOARD_UART0_TX_GPIO        (GPIO_PORTE|GPIO_PIN0)
#define BOARD_UART0_ROUTE_LOCATION _USART_ROUTE_LOCATION_LOC1

/* LEUART0:
 *
 *   LEU0_RX #0 PD5  **AVAILABLE at TP123 and EXP port pin 14**
 *   LEU0_TX #0 PD4  **AVAILABLE at TP122 and EXP port pin 12**
 */

#define BOARD_LEUART0_RX_GPIO        (GPIO_PORTD|GPIO_PIN5)
#define BOARD_LEUART0_TX_GPIO        (GPIO_PORTD|GPIO_PIN4)
#define BOARD_LEUART0_ROUTE_LOCATION _LEUART_ROUTE_LOCATION_LOC0

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __BOARDS_ARM_EFM32_EFM32_G8XX_STK_INCLUDE_BOARD_H */
