/****************************************************************************
 * boards/mips/pic32mz/ev49n51a/include/board.h
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

#ifndef __BOARDS_MIPS_PIC32MZ_EV49N51A_INCLUDE_BOARD_H
#define __BOARDS_MIPS_PIC32MZ_EV49N51A_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************
 *
 * Unlike EC/EF, PIC32MZ-W1's system PLL is NOT configured from these
 * BOARD_PLL_* macros by pic32mz_lowinit.c - it is configured directly by
 * pic32mz_wfi32_pwrclk.c (a fixed, hardcoded SPLLCON value, matching
 * Microchip's own reference firmware for this chip/board family). The
 * macros below exist only so that:
 *
 *   1. pic32mz_lowinit.c's self-consistency sanity check
 *      (CALC_SYSCLOCK == BOARD_CPU_CLOCK) passes.
 *   2. pic32mz_prefetch() picks the correct number of flash wait states
 *      for the real CPU clock.
 *
 * They were derived by decoding the actual SPLLCON value
 * (WFI32_SPLLCON_VALUE in pic32mz_wfi32_pwrclk.c) field-by-field
 * (SPLLREFDIV=5, SPLLFBDIV=150, SPLLPOSTDIV1=6), combined with the
 * documented CPU_CLOCK_FREQUENCY=200000000 from Microchip's reference
 * firmware (definitions.h) to solve for the crystal frequency - NOT
 * taken from a schematic. If pic32mz_wfi32_pwrclk.c's SPLLCON value ever
 * changes, these must be recomputed to match.
 */

#define BOARD_POSC_FREQ        40000000  /* Primary OSC XTAL frequency (derived, see above) */

#define BOARD_PLL_INPUT        BOARD_POSC_FREQ
#define BOARD_PLL_IDIV         5         /* SPLLREFDIV */
#define BOARD_PLL_MULT         150       /* SPLLFBDIV */
#define BOARD_PLL_ODIV         6         /* SPLLPOSTDIV1 */

#define BOARD_CPU_CLOCK        200000000 /* CPU clock: 200MHz, per Microchip's
                                           * reference firmware for this chip */

/* Peripheral clocks.
 *
 * Only PBCLK4 is set explicitly by pic32mz_wfi32_pwrclk.c (to divide-by-10,
 * i.e. 20MHz, the value used by Microchip's example firmware);
 * pic32mz_pbclk() will then apply the same BOARD_PB4DIV value redundantly
 * (harmless, see pic32mz_lowinit.c).  PBCLK1-3/5 divisors below are this
 * NuttX port's own choice, chosen conservatively; they can be tuned later
 * once real peripherals are exercised.  Peripheral to bus assignments are
 * from DS70005425 Table 11-1.
 */

#define BOARD_PB1DIV           5         /* Divider = 5 */
#define BOARD_PBCLK1           40000000  /* PBCLK1 = 200MHz/5 = 40MHz (Timers, UART3) */

#define BOARD_PBCLK2_ENABLE    1
#define BOARD_PB2DIV           2         /* Divider = 2 */
#define BOARD_PBCLK2           100000000 /* PBCLK2 = 200MHz/2 = 100MHz (Ports, I2C1, ADC, CAN) */

#define BOARD_PBCLK3_ENABLE    1
#define BOARD_PB3DIV           4         /* Divider = 4 */
#define BOARD_PBCLK3           50000000  /* PBCLK3 = 200MHz/4 = 50MHz (UART1/2, SPI, I2C2, IC, OC) */

#define BOARD_PBCLK4_ENABLE    1
#define BOARD_PB4DIV           10        /* Divider = 10 (matches pic32mz_wfi32_pwrclk.c) */
#define BOARD_PBCLK4           20000000  /* PBCLK4 = 200MHz/10 = 20MHz (RTCC, DSCON) */

#define BOARD_PBCLK5_ENABLE    1
#define BOARD_PB5DIV           2         /* Divider = 2 */
#define BOARD_PBCLK5           100000000 /* PBCLK5 = 200MHz/2 = 100MHz (Flash, Crypto, SQI) */

#undef BOARD_PBCLK6_ENABLE

/* PIC32MZ-W1 has no PB7/PB8 bus - leave both undefined (see
 * pic32mz_lowinit.c/hardware/pic32mz_osc.h for why these must stay
 * undefined on this chip).
 */

#undef BOARD_PBCLK7_ENABLE
#undef BOARD_PBCLK8_ENABLE

/* Watchdog pre-scaler (not yet used - WDT is left disabled by the
 * reference config; revisit if/when WDT support is added for this chip).
 */

#define BOARD_WD_PRESCALER     1048576

/* Ethernet MII management clock (MDC).
 *
 * The MIIM module is clocked at 100 MHz (PBCLK5), as on the other PIC32MZ
 * parts.  The LAN8720A accepts MDC up to 2.5 MHz.
 */

#define BOARD_EMAC_MIIM_DIV    40        /* 100MHz/40 = 2.5MHz */

/* LED definitions **********************************************************
 *
 * Two user LEDs (EV49N51A schematic 02-01134 rev 2), both active high
 * (anode on the pin, 330R to GND):
 *
 *   D201 red   - RK1 (module pad 34)
 *   D202 green - RK3 (module pad 35)
 *
 * When CONFIG_ARCH_LEDS is defined, they are used by the OS as follows:
 *
 *   SYMBOL            Meaning                 GREEN   RED
 *   ----------------- ----------------------- ------- -------
 *   LED_STARTED       NuttX has been started  OFF     OFF
 *   LED_HEAPALLOCATE  Heap has been allocated OFF     OFF
 *   LED_IRQSENABLED   Interrupts enabled      OFF     OFF
 *   LED_STACKCREATED  Idle stack created      ON      OFF
 *   LED_INIRQ         In an interrupt         N/C     GLOW
 *   LED_SIGNAL        In a signal handler     N/C     GLOW
 *   LED_ASSERTION     An assertion failed     N/C     GLOW
 *   LED_PANIC         The system has crashed  N/C     FLASH
 */

#define BOARD_LED_RED    0
#define BOARD_LED_GREEN  1
#define BOARD_NLEDS      2

#define BOARD_LED_RED_BIT   (1 << BOARD_LED_RED)
#define BOARD_LED_GREEN_BIT (1 << BOARD_LED_GREEN)

#define LED_STARTED      0
#define LED_HEAPALLOCATE 1
#define LED_IRQSENABLED  2
#define LED_STACKCREATED 3
#define LED_INIRQ        4
#define LED_SIGNAL       4
#define LED_ASSERTION    4
#define LED_PANIC        4

/* UARTS ********************************************************************
 *
 * The console is UART1 on 3-pin header J203 (1=U1RX, 2=U1TX, 3=GND); an
 * external 3.3V USB-UART adapter is required, there is no on-board
 * debugger/USB-UART bridge.  J203 is wired to module pads 29 (U1TX, RA9)
 * and 30 (U1RX, RA8), which are UART1's dedicated, non-PPS pins (RA8/RA9
 * have no RPn function).  BOARD_U1RX_PPS/BOARD_U1TX_PPS are therefore
 * intentionally left undefined so that no PPS routing is programmed.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_MIPS_PIC32MZ_EV49N51A_INCLUDE_BOARD_H */
