/****************************************************************************
 * arch/mips/src/pic32/pic32mz-lowinit.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <arch/pic32mz/cp0.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip/pic32mz-features.h"
#include "chip/pic32mz-prefetch.h"
#include "chip/pic32mz-osc.h"

#include "pic32mz-config.h"
#include "pic32mz-lowconsole.h"
#include "pic32mz-lowinit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Maximum Frequencies ******************************************************/

#if CONFIG_PIC32MZ_ECC_OPTION == 3
#  define MAX_FLASH_HZ       83000000 /* Maximum FLASH speed (Hz) without ECC */
#else
#  define MAX_FLASH_HZ       66000000 /* Maximum FLASH speed (Hz) with ECC */
#endif

#define MAX_PBCLK           100000000 /* Max peripheral bus speed (Hz) */
#define MAX_PBCLK7          200000000 /* Max peripheral bus speed (Hz) for PBCLK7 */

/* Sanity checks ************************************************************/

/* Make sure that the selected clock parameters are sane */

#define CALC_SYSCLOCK  (((BOARD_PLL_INPUT / BOARD_PLL_IDIV) * BOARD_PLL_MULT) / BOARD_PLL_ODIV)
#if CALC_SYSCLOCK != BOARD_CPU_CLOCK
#  error "Bad BOARD_CPU_CLOCK calculcation in board.h"
#endif

#define CALC_PBCLK1  (CALC_SYSCLOCK / BOARD_PB1DIV)
#if CALC_PBCLK1 != BOARD_PBCLK1
#  error "Bad BOARD_PBCLK1 calculcation in board.h"
#endif

#if CALC_PBCLK1 > MAX_PBCLK
#  error "PBCLK1 exceeds maximum value"
#endif

#ifdef BOARD_PBCLK2_ENABLE
#  define CALC_PBCLK2  (CALC_SYSCLOCK / BOARD_PB2DIV)
#  if CALC_PBCLK2 != BOARD_PBCLK2
#    error "Bad BOARD_PBCLK2 calculcation in board.h"
#  endif

#  if CALC_PBCLK2 > MAX_PBCLK
#    error "PBCLK2 exceeds maximum value"
#  endif
#endif

#ifdef BOARD_PBCLK3_ENABLE
#  define CALC_PBCLK3  (CALC_SYSCLOCK / BOARD_PB3DIV)
#  if CALC_PBCLK3 != BOARD_PBCLK3
#    error "Bad BOARD_PBCLK3 calculcation in board.h"
#  endif

#  if CALC_PBCLK3 > MAX_PBCLK
#    error "PBCLK3 exceeds maximum value"
#  endif
#endif

#ifdef BOARD_PBCLK4_ENABLE
#  define CALC_PBCLK4  (CALC_SYSCLOCK / BOARD_PB4DIV)
#  if CALC_PBCLK4 != BOARD_PBCLK4
#    error "Bad BOARD_PBCLK4 calculcation in board.h"
#  endif

#  if CALC_PBCLK4 > MAX_PBCLK
#    error "PBCLK4 exceeds maximum value"
#  endif
#endif

#ifdef BOARD_PBCLK5_ENABLE
#  define CALC_PBCLK5  (CALC_SYSCLOCK / BOARD_PB5DIV)
#  if CALC_PBCLK5 != BOARD_PBCLK5
#    error "Bad BOARD_PBCLK5 calculcation in board.h"
#  endif

#  if CALC_PBCLK5 > MAX_PBCLK
#    error "PBCLK5 exceeds maximum value"
#  endif
#endif

#ifdef BOARD_PBCLK6_ENABLE
#  define CALC_PBCLK6  (CALC_SYSCLOCK / BOARD_PB6DIV)
#  if CALC_PBCLK6 != BOARD_PBCLK6
#    error "Bad BOARD_PBCLK6 calculcation in board.h"
#  endif

#  if CALC_PBCLK6 > MAX_PBCLK
#    error "PBCLK6 exceeds maximum value"
#  endif
#endif

#ifdef BOARD_PBCLK7_ENABLE
#  define CALC_PBCLK7  (CALC_SYSCLOCK / BOARD_PB7DIV)
#  if CALC_PBCLK7 != BOARD_PBCLK7
#    error "Bad BOARD_PBCLK7 calculcation in board.h"
#  endif

#  if CALC_PBCLK7 > MAX_PBCLK7
#    error "PBCLK7 exceeds maximum value"
#  endif
#endif

#ifdef BOARD_PBCLK8_ENABLE
#  define CALC_PBCLK8  (CALC_SYSCLOCK / BOARD_PB8DIV)
#  if CALC_PBCLK8 != BOARD_PBCLK8
#    error "Bad BOARD_PBCLK8 calculcation in board.h"
#  endif

#  if CALC_PBCLK8 > MAX_PBCLK
#    error "PBCLK8 exceeds maximum value"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

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
  unsigned int nwaits;
  unsigned int residual;
  uint32_t regval;

  /* Configure pre-fetch cache FLASH wait states (assuming ECC is enabled).
   * REVISIT: Is this calculation right?  It seems like residual should be
   *
   *    residual = BOARD_CPU_CLOCK / nwaits
   *
   * This logic uses:
   *
   *    BOARD_CPU_CLOCK - nwaits * MAX_FLASH_HZ
   */

  residual = BOARD_CPU_CLOCK;
  nwaits   = 0;

  while (residual > MAX_FLASH_HZ)
    {
      nwaits++;
      residual -= MAX_FLASH_HZ;
    }

  DEBUGASSERT(nwaits < 8);

  /* Set the FLASH wait states and enabled prefetch on CPU instructions and
   * data.
   */

  regval = (PRECON_PREFEN_CPUID | PRECON_PFMWS(nwaits));
  putreg32(regval, PIC32MZ_PRECON);
}

/****************************************************************************
 * Name: pic32mz_k0cache
 *
 * Description:
 *   Enable caching in KSEG0.
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static inline void pic32mz_k0cache(void)
{
  register uint32_t regval;

  /* Enable cache on KSEG 0 in the CP0 CONFIG register*/

  asm("\tmfc0 %0,$16,0\n" :  "=r"(regval));
  regval &= ~CP0_CONFIG_K23_MASK;
  regval |= CP0_CONFIG_K23_CACHEABLE;
  asm("\tmtc0 %0,$16,0\n" : : "r" (regval));

  UNUSED(regval);
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

#ifdef BOARD_PBCLK6_ENABLE
  regval = (PBDIV_ON | PBDIV(BOARD_PB6DIV));
#else
  regval = 0;
#endif
  putreg32(regval, PIC32MZ_PB6DIV);

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
  /* Initialize FLASH wait states */

  pic32mz_prefetch();

  /* Enable caching in KSEG0 */

  pic32mz_k0cache();

  /* Configure peripheral clocking */

  pic32mz_pbclk();

  /* Initialize a console (probably a serial console) */

  pic32mz_consoleinit();

  /* Perform early serial initialization (so that we will have debug output
   * available as soon as possible).
   */

#ifdef USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif

  /* Perform board-level initialization */

  pic32mz_boardinitialize();
}
