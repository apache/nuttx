/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_lowinit.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <nuttx/arch.h>
#include <arch/pic32mz/cp0.h>
#include <arch/board/board.h>

#include "mips_internal.h"
#include "hardware/pic32mz_features.h"
#include "hardware/pic32mz_prefetch.h"
#include "hardware/pic32mz_osc.h"
#include "hardware/pic32mz_ioport.h"

#include "pic32mz_config.h"
#include "pic32mz_lowconsole.h"
#include "pic32mz_lowinit.h"

#ifdef CONFIG_ARCH_CHIP_WFI32E01
#  include "hardware/pic32mzw1_pmuclk.h"
#  include "pic32mz_wfi32_pwrclk.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum Frequencies ******************************************************/

#if (CONFIG_PIC32MZ_ECC_OPTION == 3) || (CONFIG_PIC32MZ_ECC_OPTION == 2)
#  define SYSCLK_MAX1   0
#  define SYSCLK_MAX2   74000000
#  define SYSCLK_MAX3   140000000
#  define SYSCLK_MAX4   200000000
#else
#  define SYSCLK_MAX1   0
#  define SYSCLK_MAX2   60000000
#  define SYSCLK_MAX3   120000000
#  define SYSCLK_MAX4   200000000
#endif

#define MAX_PBCLK       100000000 /* Max peripheral bus speed (Hz) */
#define MAX_PBCLK7      200000000 /* Max peripheral bus speed (Hz) for PBCLK7 */

/* Sanity checks ************************************************************/

/* Make sure that the selected clock parameters are sane */

#define CALC_SYSCLOCK  (((BOARD_PLL_INPUT / BOARD_PLL_IDIV) * BOARD_PLL_MULT) / BOARD_PLL_ODIV)
#if CALC_SYSCLOCK != BOARD_CPU_CLOCK
#  error "Bad BOARD_CPU_CLOCK calculation in board.h"
#endif

#define CALC_PBCLK1  (CALC_SYSCLOCK / BOARD_PB1DIV)
#if CALC_PBCLK1 != BOARD_PBCLK1
#  error "Bad BOARD_PBCLK1 calculation in board.h"
#endif

#if CALC_PBCLK1 > MAX_PBCLK
#  error "PBCLK1 exceeds maximum value"
#endif

#ifdef BOARD_PBCLK2_ENABLE
#  define CALC_PBCLK2  (CALC_SYSCLOCK / BOARD_PB2DIV)
#  if CALC_PBCLK2 != BOARD_PBCLK2
#    error "Bad BOARD_PBCLK2 calculation in board.h"
#  endif

#  if CALC_PBCLK2 > MAX_PBCLK
#    error "PBCLK2 exceeds maximum value"
#  endif
#endif

#ifdef BOARD_PBCLK3_ENABLE
#  define CALC_PBCLK3  (CALC_SYSCLOCK / BOARD_PB3DIV)
#  if CALC_PBCLK3 != BOARD_PBCLK3
#    error "Bad BOARD_PBCLK3 calculation in board.h"
#  endif

#  if CALC_PBCLK3 > MAX_PBCLK
#    error "PBCLK3 exceeds maximum value"
#  endif
#endif

#ifdef BOARD_PBCLK4_ENABLE
#  define CALC_PBCLK4  (CALC_SYSCLOCK / BOARD_PB4DIV)
#  if CALC_PBCLK4 != BOARD_PBCLK4
#    error "Bad BOARD_PBCLK4 calculation in board.h"
#  endif

#  if CALC_PBCLK4 > MAX_PBCLK
#    error "PBCLK4 exceeds maximum value"
#  endif
#endif

#ifdef BOARD_PBCLK5_ENABLE
#  define CALC_PBCLK5  (CALC_SYSCLOCK / BOARD_PB5DIV)
#  if CALC_PBCLK5 != BOARD_PBCLK5
#    error "Bad BOARD_PBCLK5 calculation in board.h"
#  endif

#  if CALC_PBCLK5 > MAX_PBCLK
#    error "PBCLK5 exceeds maximum value"
#  endif
#endif

#ifdef BOARD_PBCLK6_ENABLE
#  define CALC_PBCLK6  (CALC_SYSCLOCK / BOARD_PB6DIV)
#  if CALC_PBCLK6 != BOARD_PBCLK6
#    error "Bad BOARD_PBCLK6 calculation in board.h"
#  endif

#  if CALC_PBCLK6 > MAX_PBCLK
#    error "PBCLK6 exceeds maximum value"
#  endif
#endif

#ifdef BOARD_PBCLK7_ENABLE
#  define CALC_PBCLK7  (CALC_SYSCLOCK / BOARD_PB7DIV)
#  if CALC_PBCLK7 != BOARD_PBCLK7
#    error "Bad BOARD_PBCLK7 calculation in board.h"
#  endif

#  if CALC_PBCLK7 > MAX_PBCLK7
#    error "PBCLK7 exceeds maximum value"
#  endif
#endif

#ifdef BOARD_PBCLK8_ENABLE
#  define CALC_PBCLK8  (CALC_SYSCLOCK / BOARD_PB8DIV)
#  if CALC_PBCLK8 != BOARD_PBCLK8
#    error "Bad BOARD_PBCLK8 calculation in board.h"
#  endif

#  if CALC_PBCLK8 > MAX_PBCLK
#    error "PBCLK8 exceeds maximum value"
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_prefetch
 *
 * Description:
 *   Configure the prefetch module setting:
 *
 *   1. The optimal number of FLASH wait states.
 *   2. Enable prefetch on CPU instructions and data
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static inline void pic32mz_prefetch(void)
{
#ifdef CONFIG_ARCH_CHIP_PIC32MZW1
  /* PIC32MZ-W1 differs from EC/EF here (DS70005425 Reg 9-1): PREFEN only
   * accepts 00 (off) and 01 (cacheable regions), and the data sheet gives
   * no SYSCLK-to-PFMWS table.  The wait-state count comes from Kconfig;
   * its default (5 at 200 MHz) is the value used by Microchip's OOB
   * example [EX].
   */

  modifyreg32(PIC32MZ_PRECON, PRECON_PREFEN_MASK | PRECON_PFMWS_MASK,
              (1 << PRECON_PREFEN_SHIFT) |
              PRECON_PFMWS(CONFIG_PIC32MZ_W1_FLASH_WAITSTATES));
#else
  unsigned int nwaits;
  uint32_t regval;

  /* Configure pre-fetch cache FLASH wait states */

  if (BOARD_CPU_CLOCK > SYSCLK_MAX1 && BOARD_CPU_CLOCK <= SYSCLK_MAX2)
    {
      nwaits = 0;

      /* Don't enable predictive prefetch for wait states = 0 */

      regval = PRECON_PREFEN_DISABLE;
    }
  else if (BOARD_CPU_CLOCK > SYSCLK_MAX2 && BOARD_CPU_CLOCK <= SYSCLK_MAX3)
    {
      nwaits = 1;
      regval = PRECON_PREFEN_CPUID;
    }
  else if (BOARD_CPU_CLOCK > SYSCLK_MAX3 && BOARD_CPU_CLOCK <= SYSCLK_MAX4)
    {
      nwaits = 2;
      regval = PRECON_PREFEN_CPUID;
    }
  else
    {
      /* For devices with 252 MHz SYSCLK */

      nwaits = 4;
      regval = PRECON_PREFEN_CPUID;
    }

  regval |= PRECON_PFMWS(nwaits);

  /* Set the FLASH wait states and enable prefetch on CPU instructions
   * and data when required.
   */

  putreg32(regval, PIC32MZ_PRECON);
#endif
}

/****************************************************************************
 * Name: pic32mz_pbclk
 *
 * Description:
 *   Configure peripheral bus clocking
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static inline void pic32mz_pbclk(void)
{
  uint32_t regval;

  /* Perform the unlock sequence */

  putreg32(UNLOCK_SYSKEY_0, PIC32MZ_SYSKEY);
  putreg32(UNLOCK_SYSKEY_1, PIC32MZ_SYSKEY);

  /* PBCLK1
   *   Peripherals: OSC2 pin
   *
   * NOTES:
   *   - PBCLK1 is used by system modules and cannot be turned off
   *   - PBCLK1 divided by 2 is available on the OSC2 pin in certain clock
   *     modes.
   */

  regval = (PBDIV_ON | PBDIV(BOARD_PB1DIV));
  putreg32(regval, PIC32MZ_PB1DIV);

  /* PBCLK2
   *   Peripherals: PMP, I2C, UART, SPI
   */

#ifdef BOARD_PBCLK2_ENABLE
  regval = (PBDIV_ON | PBDIV(BOARD_PB2DIV));
#else
  regval = 0;
#endif
  putreg32(regval, PIC32MZ_PB2DIV);

  /* PBCLK3
   *   Peripherals: ADC, Comparator, Timers, Output Compare, Input Compare
   *
   * NOTES:
   *   - Timer 1 uses SOSC
   */

#ifdef BOARD_PBCLK3_ENABLE
  regval = (PBDIV_ON | PBDIV(BOARD_PB3DIV));
#else
  regval = 0;
#endif
  putreg32(regval, PIC32MZ_PB3DIV);

/* PBCLK4
 *   Peripherals: Ports
 */

#ifdef BOARD_PBCLK4_ENABLE
  regval = (PBDIV_ON | PBDIV(BOARD_PB4DIV));
#else
  regval = 0;
#endif
  putreg32(regval, PIC32MZ_PB4DIV);

/* PBCLK5
 *   Peripherals: Flash, Crypto, RND, USB, CAN, Ethernet, SQI
 *
 * NOTES:
 *   - PBCLK5 is used to fetch data from/to the Flash Controller, while the
 *     FRC clock is used for programming
 */

#ifdef BOARD_PBCLK5_ENABLE
  regval = (PBDIV_ON | PBDIV(BOARD_PB5DIV));
#else
  regval = 0;
#endif
  putreg32(regval, PIC32MZ_PB5DIV);

/* PBCLK6
 *   Peripherals:
 */

#if defined(CONFIG_ARCH_CHIP_PIC32MZW1) && !defined(BOARD_PBCLK6_ENABLE)
  /* On PIC32MZ-W1, PBCLK6 clocks the CPU and cannot be turned off
   * (DS70005425 Reg 11-9 note 2).  Writing 0 here (divide-by-1, "off")
   * hangs the CPU (verified on hardware), so leave the reset divider.
   */

#else
#ifdef BOARD_PBCLK6_ENABLE
  regval = (PBDIV_ON | PBDIV(BOARD_PB6DIV));
#else
  regval = 0;
#endif
  putreg32(regval, PIC32MZ_PB6DIV);
#endif

#ifndef CONFIG_ARCH_CHIP_PIC32MZW1
  /* PIC32MZ-W1 has no PB7/PB8 bus (CPU clocking and EBI are not
   * independently divided on this family); PIC32MZ_PB7DIV/PB8DIV are
   * unimplemented placeholders that alias PB6DIV's real address on this
   * chip (see hardware/pic32mz_osc.h), so they must never be written
   * here.
   */

/* PBCLK7
 *   Peripherals:  CPU, Deadman timer
 */

#ifdef BOARD_PBCLK7_ENABLE
  regval = (PBDIV_ON | PBDIV(BOARD_PB7DIV));
#else
  regval = 0;
#endif
  putreg32(regval, PIC32MZ_PB7DIV);

/* PBCLK8
 *   Peripherals: EBI
 */

#ifdef BOARD_PBCLK8_ENABLE
  regval = (PBDIV_ON | PBDIV(BOARD_PB8DIV));
#else
  regval = 0;
#endif
  putreg32(regval, PIC32MZ_PB8DIV);
#endif /* !CONFIG_ARCH_CHIP_PIC32MZW1 */
}

/****************************************************************************
 * Name: pic32mz_adcdisable
 *
 * Description:
 *   Disable adc inputs in all pins.
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void pic32mz_adcdisable(void)
{
#ifdef CONFIG_ARCH_CHIP_PIC32MZW1
  /* Only PORTA, PORTB and PORTK physically exist on this family;
   * CHIP_NPORTS is kept at 10 for compatibility with the shared
   * GPIO_PORTx letter-index encoding (see hardware/pic32mz_ioport.h),
   * so the generic per-letter loop below cannot be used as-is here -
   * PORTC-PORTJ's "addresses" are not real IOPORT registers on this
   * chip and must not be written.
   */

  putreg32(0xffffffff, PIC32MZ_IOPORTA_K1BASE +
                       PIC32MZ_IOPORT_ANSELCLR_OFFSET);
  putreg32(0xffffffff, PIC32MZ_IOPORTB_K1BASE +
                       PIC32MZ_IOPORT_ANSELCLR_OFFSET);
  putreg32(0xffffffff, PIC32MZ_IOPORTK_K1BASE +
                       PIC32MZ_IOPORT_ANSELCLR_OFFSET);
#else
  putreg32(0xffffffff, PIC32MZ_IOPORTA_K1BASE +
                       PIC32MZ_IOPORT_ANSELCLR_OFFSET);
#if CHIP_NPORTS > 1
  putreg32(0xffffffff, PIC32MZ_IOPORTB_K1BASE +
                       PIC32MZ_IOPORT_ANSELCLR_OFFSET);
#endif
#if CHIP_NPORTS > 2
  putreg32(0xffffffff, PIC32MZ_IOPORTC_K1BASE +
                       PIC32MZ_IOPORT_ANSELCLR_OFFSET);
#endif
#if CHIP_NPORTS > 3
  putreg32(0xffffffff, PIC32MZ_IOPORTD_K1BASE +
                       PIC32MZ_IOPORT_ANSELCLR_OFFSET);
#endif
#if CHIP_NPORTS > 4
  putreg32(0xffffffff, PIC32MZ_IOPORTE_K1BASE +
                       PIC32MZ_IOPORT_ANSELCLR_OFFSET);
#endif
#if CHIP_NPORTS > 5
  putreg32(0xffffffff, PIC32MZ_IOPORTF_K1BASE +
                       PIC32MZ_IOPORT_ANSELCLR_OFFSET);
#endif
#if CHIP_NPORTS > 6
  putreg32(0xffffffff, PIC32MZ_IOPORTG_K1BASE +
                       PIC32MZ_IOPORT_ANSELCLR_OFFSET);
#endif
#if CHIP_NPORTS > 7
  putreg32(0xffffffff, PIC32MZ_IOPORTH_K1BASE +
                       PIC32MZ_IOPORT_ANSELCLR_OFFSET);
#endif
#if CHIP_NPORTS > 8
  putreg32(0xffffffff, PIC32MZ_IOPORTJ_K1BASE +
                       PIC32MZ_IOPORT_ANSELCLR_OFFSET);
#endif
#endif /* CONFIG_ARCH_CHIP_PIC32MZW1 */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_lowinit
 *
 * Description:
 *   This performs basic low-level initialization of the system.
 *
 * Assumptions:
 *   Interrupts have not yet been enabled.
 *
 ****************************************************************************/

void pic32mz_lowinit(void)
{
#ifdef CONFIG_ARCH_CHIP_WFI32E01
  /* On PIC32MZ-W1, the system PLL and Wi-Fi coprocessor PMU are NOT
   * configured by DEVCFG fuses (unlike EC/EF) and must be brought up by
   * software before anything else, including flash wait-state
   * calculations, which depend on the final CPU clock.
   */

#ifdef CONFIG_PIC32MZ_W1_BOOTTRACE
  pic32mz_wfi32_trace_init();
  pic32mz_wfi32_trace('\r');
  pic32mz_wfi32_trace('\n');
  pic32mz_wfi32_trace('R');
  pic32mz_wfi32_trace_hex(getreg32(PIC32MZ_RCON));
  pic32mz_wfi32_trace(' ');
  pic32mz_wfi32_trace('A');
#endif

  pic32mz_wfi32_pmu_initialize();

#ifdef CONFIG_PIC32MZ_W1_BOOTTRACE
  /* Clear the reset cause only after the PMU set-up has looked at it, so
   * that the next boot trace shows what caused that reset.
   */

  putreg32(0xffffffff, PIC32MZ_RCONCLR);
#endif

  pic32mz_wfi32_clk_initialize();
#endif

#ifdef CONFIG_PIC32MZ_W1_BOOTTRACE
  pic32mz_wfi32_trace('B');
#endif

  /* Initialize FLASH wait states */

  pic32mz_prefetch();

#ifdef CONFIG_PIC32MZ_W1_BOOTTRACE
  pic32mz_wfi32_trace('F');
#endif

  /* Configure peripheral clocking */

  pic32mz_pbclk();

#ifdef CONFIG_PIC32MZ_W1_BOOTTRACE
  pic32mz_wfi32_trace('P');
#endif

  /* Init IO pins (Disable all ADC circuits) */

  pic32mz_adcdisable();

#ifdef CONFIG_PIC32MZ_W1_BOOTTRACE
  pic32mz_wfi32_trace('D');
  pic32mz_wfi32_trace('\r');
  pic32mz_wfi32_trace('\n');
  pic32mz_wfi32_trace_end();
#endif

  /* Initialize a console (probably a serial console) */

  pic32mz_consoleinit();

#ifdef CONFIG_PIC32MZ_W1_BOOTTRACE
  mips_lowputc('c');
#endif

  /* Perform early serial initialization (so that we will have debug output
   * available as soon as possible).
   */

#ifdef USE_EARLYSERIALINIT
  mips_earlyserialinit();
#endif

#ifdef CONFIG_PIC32MZ_W1_BOOTTRACE
  mips_lowputc('e');
#endif

  /* Perform board-level initialization */

  pic32mz_boardinitialize();

#ifdef CONFIG_PIC32MZ_W1_BOOTTRACE
  mips_lowputc('b');
  mips_lowputc('\r');
  mips_lowputc('\n');
#endif
}
