/****************************************************************************
 * arch/arm/src/ra8m1/ra_clockconfig.c
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

/* Clock generation set-up, from the BOARD_* clock options in board.h.
 *
 * The sequence follows RA8M1 User's Manual section 8.11 (initial system
 * clock setting, oscillator and PLL settings, peripheral module-dedicated
 * clock setting) and the FSP reference start-up code:
 *
 *   1. start the oscillators that are used (HOCO, MOSC); the MOCO is
 *      already running because the MCU starts from it,
 *   2. start the PLLs that are used,
 *   3. set the flash wait states for the final ICLK and enable the flash
 *      cache,
 *   4. set the dividers and switch the system clock source,
 *   5. configure the SCI clock.
 *
 * A flat (no TrustZone) image runs in the secure state, so every register
 * here is the secure instance and PRCR_S protects them.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "arm_internal.h"
#include "ra_clockconfig.h"
#include "hardware/ra8m1_fcache.h"
#include "hardware/ra8m1_system.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SCKDIVCR/SCKDIVCR2 divider encoding */

#define RA_SCKDIV_CODE(d) \
  ((d) == 1 ? 0 : (d) == 2 ? 1 : (d) == 4 ? 2 : (d) == 8 ? 3 : \
   (d) == 16 ? 4 : (d) == 32 ? 5 : (d) == 64 ? 6 : (d) == 3 ? 8 : \
   (d) == 6 ? 9 : 10)

#define RA_SCKDIVCR_VALUE \
  ((RA_SCKDIV_CODE(BOARD_FCLK_DIVIDER)  << R_SYSTEM_SCKDIVCR_FCK_SHIFT)  | \
   (RA_SCKDIV_CODE(BOARD_ICLK_DIVIDER)  << R_SYSTEM_SCKDIVCR_ICK_SHIFT)  | \
   (RA_SCKDIV_CODE(BOARD_PCLKE_DIVIDER) << R_SYSTEM_SCKDIVCR_PCKE_SHIFT) | \
   (RA_SCKDIV_CODE(BOARD_BCLK_DIVIDER)  << R_SYSTEM_SCKDIVCR_BCK_SHIFT)  | \
   (RA_SCKDIV_CODE(BOARD_PCLKA_DIVIDER) << R_SYSTEM_SCKDIVCR_PCKA_SHIFT) | \
   (RA_SCKDIV_CODE(BOARD_PCLKB_DIVIDER) << R_SYSTEM_SCKDIVCR_PCKB_SHIFT) | \
   (RA_SCKDIV_CODE(BOARD_PCLKC_DIVIDER) << R_SYSTEM_SCKDIVCR_PCKC_SHIFT) | \
   (RA_SCKDIV_CODE(BOARD_PCLKD_DIVIDER) << R_SYSTEM_SCKDIVCR_PCKD_SHIFT))

#define RA_SCKDIVCR2_VALUE \
  (RA_SCKDIV_CODE(BOARD_CPUCLK_DIVIDER) << R_SYSTEM_SCKDIVCR2_CPUCK_SHIFT)

#define RA_SCKDIVCR2_SWITCH_VALUE \
  (RA_SCKDIV_CODE(RA_CPUCLK_SWITCH_DIVIDER) << R_SYSTEM_SCKDIVCR2_CPUCK_SHIFT)

/* System clock source: BOARD_SYSCLK_SOURCE is already the SCKSCR.CKSEL
 * value (board.h sets it to one of the R_SYSTEM_SCKSCR_CKSEL_* codes), so
 * it is written to the register as-is; ra_clockconfig.h has already
 * checked it is one of the four this port supports.
 */

#define RA_SCKSCR_VALUE   BOARD_SYSCLK_SOURCE

/* MOSC: drive capability by frequency (as FSP), and the wait time.  MOMCR
 * bit 4 must be written as 1.  MOSCWTCR.MSTS counts LOCO based cycles;
 * pick the shortest setting that is at least the requested time (in
 * tenths of a microsecond, from the manual's table).
 */

#if RA_USE_MOSC
#  if RA_MOSC_FREQUENCY >= 24000000
#    define RA_MOSC_MODRV0  5
#  else
#    define RA_MOSC_MODRV0  3
#  endif

#  if BOARD_MOSC_EXTERNAL_CLOCK
#    define RA_MOMCR_VALUE  (R_SYSTEM_MOMCR_AGCEN | R_SYSTEM_MOMCR_MOSEL | \
                             (RA_MOSC_MODRV0 << R_SYSTEM_MOMCR_MODRV0_SHIFT))
#    define RA_MOSCWTCR_VALUE 0
#  else
#    define RA_MOMCR_VALUE  (R_SYSTEM_MOMCR_AGCEN | \
                             (RA_MOSC_MODRV0 << R_SYSTEM_MOMCR_MODRV0_SHIFT))
#    define RA_MOSC_WAIT_X10 (BOARD_MOSC_STABILIZATION_US * 10)
#    define RA_MOSCWTCR_VALUE \
  (RA_MOSC_WAIT_X10 <= 114    ? 0 : RA_MOSC_WAIT_X10 <= 1335   ? 1 : \
   RA_MOSC_WAIT_X10 <= 2556   ? 2 : RA_MOSC_WAIT_X10 <= 4997   ? 3 : \
   RA_MOSC_WAIT_X10 <= 9880   ? 4 : RA_MOSC_WAIT_X10 <= 20866  ? 5 : \
   RA_MOSC_WAIT_X10 <= 40398  ? 6 : RA_MOSC_WAIT_X10 <= 81902  ? 7 : \
   RA_MOSC_WAIT_X10 <= 163689 ? 8 : 9)
#  endif
#endif

/* PLL1 and PLL2 settings.  PLLMUL holds the multiplier minus 1.  The
 * fraction (PLLMULNF) and the source select bit (PLSRCSEL) are written
 * from board.h as-is: BOARD_PLLn_MULTIPLIER_FRACTION and BOARD_PLLn_SOURCE
 * are already the real, correctly shifted register field values (see
 * board.h and ra_clockconfig.h), not raw codes needing a shift here.  The
 * output dividers are held as the divider minus 1.
 */

#if RA_USE_PLL1
#  define RA_PLLCCR_VALUE \
  (((BOARD_PLL1_MULTIPLIER - 1) << R_SYSTEM_PLLCCR_PLLMUL_SHIFT) | \
   BOARD_PLL1_MULTIPLIER_FRACTION | BOARD_PLL1_SOURCE | \
   ((BOARD_PLL1_INPUT_DIVIDER - 1) << R_SYSTEM_PLLCCR_PLIDIV_SHIFT))

#  define RA_PLLCCR2_VALUE \
  (((BOARD_PLL1_DIVIDER_P - 1) << R_SYSTEM_PLLCCR2_PLODIVP_SHIFT) | \
   ((BOARD_PLL1_DIVIDER_Q - 1) << R_SYSTEM_PLLCCR2_PLODIVQ_SHIFT) | \
   ((BOARD_PLL1_DIVIDER_R - 1) << R_SYSTEM_PLLCCR2_PLODIVR_SHIFT))
#endif

#if RA_USE_PLL2
#  define RA_PLL2CCR_VALUE \
  (((BOARD_PLL2_MULTIPLIER - 1) << R_SYSTEM_PLL2CCR_PLL2MUL_SHIFT) | \
   BOARD_PLL2_MULTIPLIER_FRACTION | BOARD_PLL2_SOURCE | \
   ((BOARD_PLL2_INPUT_DIVIDER - 1) << R_SYSTEM_PLL2CCR_PL2IDIV_SHIFT))

#  define RA_PLL2CCR2_VALUE \
  (((BOARD_PLL2_DIVIDER_P - 1) << R_SYSTEM_PLL2CCR2_PL2ODIVP_SHIFT) | \
   ((BOARD_PLL2_DIVIDER_Q - 1) << R_SYSTEM_PLL2CCR2_PL2ODIVQ_SHIFT) | \
   ((BOARD_PLL2_DIVIDER_R - 1) << R_SYSTEM_PLL2CCR2_PL2ODIVR_SHIFT))
#endif

/* SCICLK: source select codes (SCICKCR.SCICKSEL) and divider codes
 * (SCICKDIVCR.SCICKDIV, non-linear like the system clock dividers).
 */

#if RA_USE_SCICLK
#  if BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_HOCO
#    define RA_SCICKSEL_VALUE   0
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_MOSC
#    define RA_SCICKSEL_VALUE   3
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL1P
#    define RA_SCICKSEL_VALUE   5
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL2P
#    define RA_SCICKSEL_VALUE   6
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL1Q
#    define RA_SCICKSEL_VALUE   7
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL1R
#    define RA_SCICKSEL_VALUE   8
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL2Q
#    define RA_SCICKSEL_VALUE   9
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL2R
#    define RA_SCICKSEL_VALUE   10
#  else
#    define RA_SCICKSEL_VALUE   1
#  endif

#  define RA_SCICKDIV_VALUE \
  (BOARD_SCICLK_DIVIDER == 1 ? 0 : BOARD_SCICLK_DIVIDER == 2 ? 1 : \
   BOARD_SCICLK_DIVIDER == 4 ? 2 : BOARD_SCICLK_DIVIDER == 6 ? 3 : \
   BOARD_SCICLK_DIVIDER == 8 ? 4 : BOARD_SCICLK_DIVIDER == 3 ? 5 : 6)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_delay_us
 *
 * Description:
 *   Busy wait for at least the given time.  No timer is available this
 *   early, so this counts loop iterations.  Each iteration takes at least
 *   one CPU clock cycle, so assuming one cycle per iteration can only make
 *   the wait longer than requested, never shorter.
 *
 * Input Parameters:
 *   us        - Time to wait, in microseconds.
 *   cpuclk_hz - An upper bound for the CPU clock frequency during the wait.
 *
 ****************************************************************************/

#if BOARD_SYSCLK_SOURCE == R_SYSTEM_SCKSCR_CKSEL_PLL
static void ra_delay_us(uint32_t us, uint32_t cpuclk_hz)
{
  uint32_t loops = us * (cpuclk_hz / 1000000);

  if (loops == 0)
    {
      loops = 1;
    }

  __asm__ __volatile__
    (
      "1:\n"
      "\tsubs %0, %0, #1\n"
      "\tbne 1b\n"
      : "+r" (loops)
      :
      : "cc"
    );
}
#endif

/****************************************************************************
 * Name: ra_wait_stable
 *
 * Description:
 *   Wait until the given OSCSF flag is set, meaning that the oscillator or
 *   PLL is stable and can be used.
 *
 ****************************************************************************/

#if RA_USE_HOCO || RA_USE_MOSC || RA_USE_PLL1 || RA_USE_PLL2
static void ra_wait_stable(uint8_t flag)
{
  while ((getreg8(R_SYSTEM_OSCSF) & flag) == 0)
    {
    }
}
#endif

/****************************************************************************
 * Name: ra_hoco_start
 ****************************************************************************/

#if RA_USE_HOCO
static void ra_hoco_start(void)
{
  /* The HOCO is stopped after reset unless OFS1.HOCOEN starts it.  Its
   * frequency register (HOCOCR2) comes out of reset holding
   * OFS1.HOCOFRQ0, the same BOARD_HOCO_FREQUENCY that RA_HOCO_FREQUENCY
   * comes from, so only the start is needed here.
   */

  if ((getreg8(R_SYSTEM_HOCOCR) & R_SYSTEM_HOCOCR_HCSTP) != 0)
    {
      putreg8(0, R_SYSTEM_HOCOCR);
    }

  ra_wait_stable(R_SYSTEM_OSCSF_HOCOSF);
}
#endif

/****************************************************************************
 * Name: ra_mosc_start
 ****************************************************************************/

#if RA_USE_MOSC
static void ra_mosc_start(void)
{
  /* MOMCR and MOSCWTCR may only be written while the oscillator is
   * stopped.  After that, start it and wait for it to stabilize.
   */

  if ((getreg8(R_SYSTEM_MOSCCR) & R_SYSTEM_MOSCCR_MOSTP) != 0)
    {
      putreg8(RA_MOMCR_VALUE, R_SYSTEM_MOMCR);
      putreg8(RA_MOSCWTCR_VALUE, R_SYSTEM_MOSCWTCR);
      putreg8(0, R_SYSTEM_MOSCCR);
    }

  ra_wait_stable(R_SYSTEM_OSCSF_MOSCSF);
}
#endif

/****************************************************************************
 * Name: ra_pll1_start
 ****************************************************************************/

#if RA_USE_PLL1
static void ra_pll1_start(void)
{
  /* PLL1 is stopped after reset, so its settings can be written */

  putreg16(RA_PLLCCR_VALUE, R_SYSTEM_PLLCCR);
  putreg16(RA_PLLCCR2_VALUE, R_SYSTEM_PLLCCR2);
  putreg8(0, R_SYSTEM_PLLCR);

  ra_wait_stable(R_SYSTEM_OSCSF_PLLSF);
}
#endif

/****************************************************************************
 * Name: ra_pll2_start
 ****************************************************************************/

#if RA_USE_PLL2
static void ra_pll2_start(void)
{
  putreg16(RA_PLL2CCR_VALUE, R_SYSTEM_PLL2CCR);
  putreg16(RA_PLL2CCR2_VALUE, R_SYSTEM_PLL2CCR2);
  putreg8(0, R_SYSTEM_PLL2CR);

  ra_wait_stable(R_SYSTEM_OSCSF_PLL2SF);
}
#endif

/****************************************************************************
 * Name: ra_sysclk_switch
 *
 * Description:
 *   Set the dividers and switch the system clock source, keeping the
 *   frequencies inside their limits at every step.
 *
 *   The MCU starts from the MOCO with every divider at 1.  All the
 *   configurable sources are at least as fast as the MOCO, so the dividers
 *   are written first: nothing can run faster than configured while the
 *   source changes.  The dividers of ICLK and below are written before the
 *   CPUCLK one so that CPUCLK never drops below ICLK.
 *
 *   Each register is read back after being written, as the manual asks
 *   (section 8.12.1), before the next step.
 *
 ****************************************************************************/

static void ra_sysclk_switch(void)
{
  putreg32(RA_SCKDIVCR_VALUE, R_SYSTEM_SCKDIVCR);
  getreg32(R_SYSTEM_SCKDIVCR);

#if BOARD_SYSCLK_SOURCE == R_SYSTEM_SCKSCR_CKSEL_PLL
  /* The PLL is selected with CPUCLK at or below the package limit.  If the
   * final CPUCLK is above it, start with a slower CPU divider, wait for the
   * PLL to settle and only then lower the divider (RA8M1 User's Manual
   * figure 8.13).
   */

#  if RA_CPUCLK_FREQUENCY > RA_PLL_SWITCH_MAX_CPUCLK
  putreg8(RA_SCKDIVCR2_SWITCH_VALUE, R_SYSTEM_SCKDIVCR2);
  getreg8(R_SYSTEM_SCKDIVCR2);

  putreg8(RA_SCKSCR_VALUE, R_SYSTEM_SCKSCR);
  getreg8(R_SYSTEM_SCKSCR);

  ra_delay_us(RA_CLOCK_SETTLE_US, RA_CPUCLK_FREQUENCY);

  putreg8(RA_SCKDIVCR2_VALUE, R_SYSTEM_SCKDIVCR2);
  getreg8(R_SYSTEM_SCKDIVCR2);

  ra_delay_us(RA_CLOCK_SETTLE_US, RA_CPUCLK_FREQUENCY);
#  else
  putreg8(RA_SCKDIVCR2_VALUE, R_SYSTEM_SCKDIVCR2);
  getreg8(R_SYSTEM_SCKDIVCR2);

  putreg8(RA_SCKSCR_VALUE, R_SYSTEM_SCKSCR);
  getreg8(R_SYSTEM_SCKSCR);

  ra_delay_us(RA_CLOCK_SETTLE_US, RA_CPUCLK_FREQUENCY);
#  endif
#else
  putreg8(RA_SCKDIVCR2_VALUE, R_SYSTEM_SCKDIVCR2);
  getreg8(R_SYSTEM_SCKDIVCR2);

  putreg8(RA_SCKSCR_VALUE, R_SYSTEM_SCKSCR);
  getreg8(R_SYSTEM_SCKSCR);
#endif
}

/****************************************************************************
 * Name: ra_sciclk_set
 *
 * Description:
 *   Select the SCICLK source and divider with the switching handshake of
 *   the manual (section 8.2.48): request the switch, wait until SCICLK is
 *   stopped, write the divider and source, release the request, and wait
 *   until SCICLK runs again.
 *
 *   Changing the divider from a value other than 1 needs the SCI modules
 *   in the module-stop state.  That is the case here: they are all stopped
 *   after reset and this runs before any of them is started.
 *
 ****************************************************************************/

#if RA_USE_SCICLK
static void ra_sciclk_set(void)
{
  modifyreg8(R_SYSTEM_SCICKCR, 0, R_SYSTEM_SCICKCR_CKSREQ);
  while ((getreg8(R_SYSTEM_SCICKCR) & R_SYSTEM_SCICKCR_CKSRDY) == 0)
    {
    }

  putreg8(RA_SCICKDIV_VALUE, R_SYSTEM_SCICKDIVCR);
  putreg8(RA_SCICKSEL_VALUE | R_SYSTEM_SCICKCR_CKSREQ, R_SYSTEM_SCICKCR);

  putreg8(RA_SCICKSEL_VALUE, R_SYSTEM_SCICKCR);
  while ((getreg8(R_SYSTEM_SCICKCR) & R_SYSTEM_SCICKCR_CKSRDY) != 0)
    {
    }
}
#endif

/****************************************************************************
 * Name: ra_fcache_enable
 *
 * Description:
 *   Enable the flash cache, as the FSP reference start-up code does
 *   (R_BSP_FlashCacheEnable()): invalidate it and wait for the
 *   invalidation to finish (RA8M1 User's Manual, Flash Memory chapter,
 *   "Operation") before enabling it.
 *
 ****************************************************************************/

static void ra_fcache_enable(void)
{
  putreg16(R_FCACHE_FCACHEIV_FCACHEIV, R_FCACHE_FCACHEIV);
  while ((getreg16(R_FCACHE_FCACHEIV) & R_FCACHE_FCACHEIV_FCACHEIV) != 0)
    {
    }

  putreg16(R_FCACHE_FCACHEE_FCACHEEN, R_FCACHE_FCACHEE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_clockconfig
 *
 * Description:
 *   Called to initialize the RA8M1.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the BOARD_* clock options in board.h.
 *
 ****************************************************************************/

void ra_clockconfig(void)
{
  /* Unlock the clock generation (PRC0) and low power/module stop (PRC1)
   * registers.
   */

  putreg16((R_SYSTEM_PRCR_S_PRKEY_V0XA5 | R_SYSTEM_PRCR_S_PRC0 |
            R_SYSTEM_PRCR_S_PRC1), R_SYSTEM_PRCR_S);

  /* The MCU is in high-speed operating mode after reset, which is what
   * every configuration allowed here needs, so OPCCR is left alone.
   */

  /* Start the oscillators the configuration uses before anything that
   * depends on them (the PLLs).  The MOCO is already running.
   */

#if RA_USE_HOCO
  ra_hoco_start();
#endif

#if RA_USE_MOSC
  ra_mosc_start();
#endif

#if RA_USE_PLL1
  ra_pll1_start();
#endif

#if RA_USE_PLL2
  ra_pll2_start();
#endif

  /* The flash needs its wait states for the final ICLK.  Setting them
   * before the clock speeds up is safe: extra wait states at a lower
   * frequency only cost time.
   */

  putreg8(RA_FLASH_WAIT_STATES, R_FCACHE_FLWT);

  ra_fcache_enable();

  ra_sysclk_switch();

#if RA_USE_SCICLK
  ra_sciclk_set();
#endif

  /* Lock the protected registers again */

  putreg16(R_SYSTEM_PRCR_S_PRKEY_V0XA5, R_SYSTEM_PRCR_S);
}
