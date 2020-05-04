/****************************************************************************
 * arch/avr/src/sam34/sam4l_clockconfig.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This file is derived from nuttx/arch/avr/src/at32uc3/at32uc3_clkinit.c
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

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_arch.h"

#include "arm_internal.h"
#include "hardware/sam4l_pm.h"
#include "hardware/sam4l_scif.h"
#include "hardware/sam4l_bpm.h"
#include "hardware/sam4l_bscif.h"
#include "hardware/sam4l_flashcalw.h"

#include "sam4l_periphclks.h"
#include "sam_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_ARCH_RAMFUNCS
# error "CONFIG_ARCH_RAMFUNCS must be defined"
#endif

/* Board/Clock Setup *******************************************************/
/* Verify dividers */

#if ((BOARD_CPU_SHIFT > BOARD_PBA_SHIFT) || (BOARD_CPU_SHIFT > BOARD_PBB_SHIFT) || \
     (BOARD_CPU_SHIFT > BOARD_PBC_SHIFT) || (BOARD_CPU_SHIFT > BOARD_PBD_SHIFT))
#  error BOARD_PBx_SHIFT must be greater than or equal to BOARD_CPU_SHIFT
#endif

/* Nominal frequencies in on-chip RC oscillators.  These may frequencies
 * may vary with temperature changes.
 */

#define SAM_RCSYS_FREQUENCY            115000 /* Nominal frequency of RCSYS (Hz) */
#define SAM_RC32K_FREQUENCY             32768 /* Nominal frequency of RC32K (Hz) */
#define SAM_RC80M_FREQUENCY          80000000 /* Nominal frequency of RC80M (Hz) */
#define SAM_RCFAST4M_FREQUENCY        4000000 /* Nominal frequency of RCFAST4M (Hz) */
#define SAM_RCFAST8M_FREQUENCY        8000000 /* Nominal frequency of RCFAST8M (Hz) */
#define SAM_RCFAST12M_FREQUENCY      12000000 /* Nominal frequency of RCFAST12M (Hz) */
#define SAM_RC1M_FREQUENCY            1000000 /* Nominal frequency of RC1M (Hz) */

/* Oscillator 0.  This might be the system clock or the source clock for
 * either PLL0 or DFPLL.  It might also be needed if OSC0 is the source
 * clock for GCLK9.
 *
 * By selecting CONFIG_SAM34_OSC0, you can also force the clock to be enabled
 * at boot time.
 */

#if defined(CONFIG_SAM34_OSC0) || defined(BOARD_SYSCLK_SOURCE_OSC0) || \
    defined(BOARD_DFLL0_SOURCE_OSC0) || defined(BOARD_PLL0_SOURCE_OSC0) || \
    defined(BOARD_GLCK9_SOURCE_OSC0)
#  define NEED_OSC0                  1
#endif

#ifdef NEED_OSC0
#  if !defined(BOARD_OSC0_STARTUP_US)
#    error BOARD_OSC0_STARTUP_US is not defined
#  elif BOARD_OSC0_STARTUP_US == 0
#    define SAM_OSC0_STARTUP_VALUE   SCIF_OSCCTRL0_STARTUP_0
#    define SAM_OSC0_STARTUP_TIMEOUT 8
#  elif BOARD_OSC0_STARTUP_US <= 557
#    define SAM_OSC0_STARTUP_VALUE   SCIF_OSCCTRL0_STARTUP_64
#    define SAM_OSC0_STARTUP_TIMEOUT 80
#  elif BOARD_OSC0_STARTUP_US <= 1100
#    define SAM_OSC0_STARTUP_VALUE   SCIF_OSCCTRL0_STARTUP_128
#    define SAM_OSC0_STARTUP_TIMEOUT 160
#  elif BOARD_OSC0_STARTUP_US <= 18000
#    define SAM_OSC0_STARTUP_VALUE   SCIF_OSCCTRL0_STARTUP_2K
#    define SAM_OSC0_STARTUP_TIMEOUT 2560
#  elif BOARD_OSC0_STARTUP_US <= 36000
#    define SAM_OSC0_STARTUP_VALUE   SCIF_OSCCTRL0_STARTUP_4K
#    define SAM_OSC0_STARTUP_TIMEOUT 5120
#  elif BOARD_OSC0_STARTUP_US <= 71000
#    define SAM_OSC0_STARTUP_VALUE   SCIF_OSCCTRL0_STARTUP_8K
#    define SAM_OSC0_STARTUP_TIMEOUT 10240
#  elif BOARD_OSC0_STARTUP_US <= 143000
#    define SAM_OSC0_STARTUP_VALUE   SCIF_OSCCTRL0_STARTUP_16K
#    define SAM_OSC0_STARTUP_TIMEOUT 20480
#  elif BOARD_OSC0_STARTUP_US <= 285000
#    define SAM_OSC0_STARTUP_VALUE   SCIF_OSCCTRL0_STARTUP_32K
#    define SAM_OSC0_STARTUP_TIMEOUT 40960
#  else
#    error BOARD_OSC0_STARTUP_US is out of range
#  endif

#  ifdef BOARD_OSC0_ISXTAL
#    define SAM_OSC0_MODE_VALUE      SCIF_OSCCTRL0_MODE
#    if BOARD_OSC0_FREQUENCY < 2000000
#      define SAM_OSC0_GAIN_VALUE    SCIF_OSCCTRL0_GAIN(0)
#    elif BOARD_OSC0_FREQUENCY < 4000000
#      define SAM_OSC0_GAIN_VALUE    SCIF_OSCCTRL0_GAIN(1)
#    elif BOARD_OSC0_FREQUENCY < 8000000
#      define SAM_OSC0_GAIN_VALUE    SCIF_OSCCTRL0_GAIN(2)
#    elif BOARD_OSC0_FREQUENCY < 16000000
#      define SAM_OSC0_GAIN_VALUE    SCIF_OSCCTRL0_GAIN(3)
#    else
#      define SAM_OSC0_GAIN_VALUE    ((0x1u << 4) | SCIF_OSCCTRL0_GAIN(0))
#    endif
#  else
#    define SAM_OSC0_MODE_VALUE      0
#    define SAM_OSC0_GAIN_VALUE      0
#  endif
#endif

/* OSC32.  The 32K oscillator may be the source clock for DFPLL0 or
 * the source clock for GLK9 that might be used to driver PLL0.
 *
 * By selecting CONFIG_SAM34_OSC32K, you can also force the clock to be
 * enabled at boot time.  OSC32 may needed by other devices as well
 * (AST, WDT, PICUART, RTC).
 */

#if defined(CONFIG_SAM34_OSC32K) || defined(BOARD_DFLL0_SOURCE_OSC32K) || \
    defined(BOARD_GLCK9_SOURCE_OSC32K)
#  define NEED_OSC32K                1
#endif

#ifdef NEED_OSC32K
#  if !defined(BOARD_OSC32_STARTUP_US)
#    error BOARD_OSC32_STARTUP_US is not defined
#  elif BOARD_OSC32_STARTUP_US == 0
#    define SAM_OSC32_STARTUP_VALUE  BSCIF_OSCCTRL32_STARTUP_0
#  elif BOARD_OSC32_STARTUP_US   <= 1100
#    define SAM_OSC32_STARTUP_VALUE  BSCIF_OSCCTRL32_STARTUP_128
#  elif BOARD_OSC32_STARTUP_US   <= 72300
#    define SAM_OSC32_STARTUP_VALUE  BSCIF_OSCCTRL32_STARTUP_8K
#  elif BOARD_OSC32_STARTUP_US   <= 143000
#    define SAM_OSC32_STARTUP_VALUE  BSCIF_OSCCTRL32_STARTUP_16K
#  elif BOARD_OSC32_STARTUP_US   <= 570000
#    define SAM_OSC32_STARTUP_VALUE  BSCIF_OSCCTRL32_STARTUP_64K
#  elif BOARD_OSC32_STARTUP_US   <= 1100000
#    define SAM_OSC32_STARTUP_VALUE  BSCIF_OSCCTRL32_STARTUP_128K
#  elif BOARD_OSC32_STARTUP_US   <= 2300000
#    define SAM_OSC32_STARTUP_VALUE  BSCIF_OSCCTRL32_STARTUP_256K
#  elif BOARD_OSC32_STARTUP_US   <= 4600000
#    define SAM_OSC32_STARTUP_VALUE BSCIF_OSCCTRL32_STARTUP_512K
#  else
#    error BOARD_OSC32_STARTUP_US is out of range
#  endif

#  ifdef BOARD_OSC32_ISXTAL
#    define SAM_OSC32_MODE_VALUE     BSCIF_OSCCTRL32_MODE_XTAL
#  else
#    define SAM_OSC32_MODE_VALUE     BSCIF_OSCCTRL32_MODE_EXTCLK
#  endif

#  ifndef BOARD_OSC32_SELCURR
#    define BOARD_OSC32_SELCURR      BSCIF_OSCCTRL32_SELCURR_300
#  endif
#endif

/* RC80M.  This might be the system clock or the source clock for the DFPLL
 * or it could be the source for GCLK9 that drives PLL0.
 *
 * By selecting CONFIG_SAM34_RC80M, you can also force the clock to be enabled
 * at boot time.
 */

#if defined(CONFIG_SAM34_RC80M) || defined(BOARD_SYSCLK_SOURCE_RC80M) || \
    defined(BOARD_DFLL0_SOURCE_RC80M) || defined(BOARD_GLCK9_SOURCE_RC80M)
#  define NEED_RC80M                 1
#endif

/* RCFAST.  The 12/8/4 fast RC oscillator may be used as the system clock
 * or as the source for GLCK9 that drives PLL0.
 * If not then, it may be enabled by setting the CONFIG_SAM34_RCFASTxM
 * configuration variable.
 */

#if defined(CONFIG_SAM34_RCFAST12M)
#  undef CONFIG_SAM34_RCFAST8M
#  undef CONFIG_SAM34_RCFAST4M
#elif defined(CONFIG_SAM34_RCFAST8M)
#  undef CONFIG_SAM34_RCFAST4M
#endif

#if defined(BOARD_SYSCLK_SOURCE_FCFAST12M)
#  if defined(CONFIG_SAM34_RCFAST8M) || defined(CONFIG_SAM34_RCFAST4M)
#    error BOARD_SYSCLK_SOURCE_FCFAST12M inconsistent with CONFIG_SAM34_RCFAST8/4M
#  endif
#  define NEED_RCFAST                1
#  define SAM_RCFAST_RANGE           SCIF_RCFASTCFG_FRANGE_12MHZ
#  define SAM_RCFAST_FREQUENCY       SAM_RCFAST12M_FREQUENCY
#elif defined(BOARD_SYSCLK_SOURCE_FCFAST8M)
#  if defined(CONFIG_SAM34_RCFAST12M) || defined(CONFIG_SAM34_RCFAST4M)
#    error BOARD_SYSCLK_SOURCE_FCFAST8M inconsistent with CONFIG_SAM34_RCFAST12/4M
#  endif
#  define NEED_RCFAST                1
#  define SAM_RCFAST_RANGE           SCIF_RCFASTCFG_FRANGE_8MHZ
#  define SAM_RCFAST_FREQUENCY       SAM_RCFAST8M_FREQUENCY
#elif defined(BOARD_SYSCLK_SOURCE_FCFAST4M)
#  if defined(CONFIG_SAM34_RCFAST12M) || defined(CONFIG_SAM34_RCFAST8M)
#    error BOARD_SYSCLK_SOURCE_FCFAST4M inconsistent with CONFIG_SAM34_RCFAST12/8M
#  endif
#  define NEED_RCFAST                1
#  define SAM_RCFAST_RANGE           SCIF_RCFASTCFG_FRANGE_4MHZ
#  define SAM_RCFAST_FREQUENCY       SAM_RCFAST4M_FREQUENCY
#elif defined(CONFIG_SAM34_RCFAST12M)
#  define NEED_RCFAST                1
#  define SAM_RCFAST_RANGE           SCIF_RCFASTCFG_FRANGE_12MHZ
#  define SAM_RCFAST_FREQUENCY       SAM_RCFAST12M_FREQUENCY
#elif defined(CONFIG_SAM34_RCFAST8M)
#  define NEED_RCFAST                1
#  define SAM_RCFAST_RANGE           SCIF_RCFASTCFG_FRANGE_8MHZ
#  define SAM_RCFAST_FREQUENCY       SAM_RCFAST8M_FREQUENCY
#elif defined(CONFIG_SAM34_RCFAST4M)
#  define NEED_RCFAST                1
#  define SAM_RCFAST_RANGE           SCIF_RCFASTCFG_FRANGE_4MHZ
#  define SAM_RCFAST_FREQUENCY       SAM_RCFAST4M_FREQUENCY
#endif

/* RC1M.  The 1M RC oscillator may be used as the system block or
 * may be the source clock for GLCK9 that drives PLL0
 *
 * By selecting CONFIG_SAM34_RC1M, you can also force the clock to be
 * enabled at boot time.
 */

#if defined(CONFIG_SAM34_RC1M) || defined(BOARD_SYSCLK_SOURCE_RC1M) || \
    defined(BOARD_GLCK9_SOURCE_RC1M)
#  define NEED_RC1M                  1
#endif

/* RC32K.  The 32KHz RC oscillator may be used as the input to DFLL0
 * or as the input to GCLK9 that drives PLL0.
 *
 * By selecting CONFIG_SAM34_RC32K, you can also force the clock to be
 * enabled at boot time.
 */

#if defined(CONFIG_SAM34_RC32K) || defined(BOARD_DFLL0_SOURCE_RC32K) || \
    defined(BOARD_GLCK9_SOURCE_RC32K)
#  define NEED_RC32K                 1
#endif

/* GCLK9.  May used as a source clock for PLL0 */

#ifdef BOARD_PLL0_SOURCE_GCLK9
#  define NEED_GLCK9                 1
#endif

#ifdef NEED_GLCK9
#  if defined(BOARD_GLCK9_SOURCE_RCSYS)
#    define SAM_GCLK9_SOURCE_VALUE   SCIF_GCCTRL_OSCSEL_RCSYS
#    define SAM_GCLK9_FREQUENCY      SAM_RCSYS_FREQUENCY
#  elif defined(BOARD_GLCK9_SOURCE_OSC32K)
#    define SAM_GCLK9_SOURCE_VALUE   SCIF_GCCTRL_OSCSEL_OSC32K
#    define SAM_GCLK9_FREQUENCY      BOARD_OSC32_FREQUENCY
#  elif defined(BOARD_GLCK9_SOURCE_DFLL0)
#    error BOARD_GLCK9_SOURCE_DFLL0 is not supported
#  elif defined(BOARD_GLCK9_SOURCE_OSC0)
#    define SAM_GCLK9_SOURCE_VALUE   SCIF_GCCTRL_OSCSEL_OSC0
#    define SAM_GCLK9_FREQUENCY      BOARD_OSC0_FREQUENCY
#  elif defined(BOARD_GLCK9_SOURCE_RC80M)
#    define SAM_GCLK9_SOURCE_VALUE   SCIF_GCCTRL_OSCSEL_RC80M
#    define SAM_GCLK9_FREQUENCY      SAM_RC80M_FREQUENCY
#  elif defined(BOARD_GLCK9_SOURCE_RCFAST)
#    error BOARD_GLCK9_SOURCE_RCFAST is not supported (needs RCFAST configuration)
#  elif defined(BOARD_GLCK9_SOURCE_RC1M)
#    define SAM_GCLK9_SOURCE_VALUE   SCIF_GCCTRL_OSCSEL_RC1M
#    define SAM_GCLK9_FREQUENCY      SAM_RCFAST_FREQUENCY
#  elif defined(BOARD_GLCK9_SOURCE_CPUCLK)
#    define SAM_GCLK9_SOURCE_VALUE   SCIF_GCCTRL_OSCSEL_CPUCLK
#    define SAM_GCLK9_FREQUENCY      BOARD_CPU_FREQUENCY
#  elif defined(BOARD_GLCK9_SOURCE_HSBCLK)
#    error BOARD_GLCK9_SOURCE_HSBCLK is not supported (REVISIT)
#  elif defined(BOARD_GLCK9_SOURCE_PBACLK)
#    define SAM_GCLK9_SOURCE_VALUE   SCIF_GCCTRL_OSCSEL_PBACLK
#    define SAM_GCLK9_FREQUENCY      BOARD_PBA_FREQUENCY
#  elif defined(BOARD_GLCK9_SOURCE_PBBCLK)
#    define SAM_GCLK9_SOURCE_VALUE   SCIF_GCCTRL_OSCSEL_PBBCLK
#    define SAM_GCLK9_FREQUENCY      BOARD_PBB_FREQUENCY
#  elif defined(BOARD_GLCK9_SOURCE_PBCCLK)
#    define SAM_GCLK9_SOURCE_VALUE   SCIF_GCCTRL_OSCSEL_PBCCLK
#    define SAM_GCLK9_FREQUENCY      BOARD_PBC_FREQUENCY
#  elif defined(BOARD_GLCK9_SOURCE_PBDCLK)
#    define SAM_GCLK9_SOURCE_VALUE   SCIF_GCCTRL_OSCSEL_PBDCLK
#    define SAM_GCLK9_FREQUENCY      BOARD_PBD_FREQUENCY
#  elif defined(BOARD_GLCK9_SOURCE_RC32K)
#    define SAM_GCLK9_SOURCE_VALUE   SCIF_GCCTRL_OSCSEL_RC32K
#    define SAM_GCLK9_FREQUENCY      SAM_RC32K_FREQUENCY
#  else
#    error Missing GCLK9 source
#  endif
#endif

/* PLL0 */

#ifdef BOARD_SYSCLK_SOURCE_PLL0
/* PLL0 source */

#  if defined(BOARD_PLL0_SOURCE_OSC0)
#    define SAM_PLL0_SOURCE            SCIF_PLL0_PLLOSC_OSC0
#    define SAM_PLL0_SOURCE_FREQUENCY  BOARD_OSC0_FREQUENCY
#  elif defined(BOARD_PLL0_SOURCE_GCLK9)
#    define SAM_PLL0_SOURCE            SCIF_PLL0_PLLOSC_GCLK9
#    define SAM_PLL0_SOURCE_FREQUENCY  SAM_GCLK9_FREQUENCY
#  else
#    error Missing PLL0 source
#  endif

/* PLL0 Multipler and Divider */

#  if !defined(BOARD_PLL0_MUL)
#    error BOARD_PLL0_MUL is not defined
#  elif BOARD_PLL0_MUL <= 2 || BOARD_PLL0_MUL > 16
#    error BOARD_PLL0_MUL is out of range
#  endif

#  if !defined(BOARD_PLL0_DIV)
#    error BOARD_PLL0_DIV is not defined
#  elif BOARD_PLL0_DIV < 1 || BOARD_PLL0_DIV > 15
#    error BOARD_PLL0_DIV is out of range
#  endif

/* PLL0 frequency ranges */

#  define SAM_PLL0_MIN_FREQUENCY      40000000
#  define SAM_PLL0_MAX_FREQUENCY     240000000

/* PLL0 VCO frequency */

#  define SAM_PLL0_VCO_DIV1_FREQUENCY \
    (SAM_PLL0_SOURCE_FREQUENCY * BOARD_PLL0_MUL / BOARD_PLL0_DIV)

#  if (SAM_PLL0_VCO_DIV1_FREQUENCY < SAM_PLL0_MIN_FREQUENCY) || \
      (SAM_PLL0_VCO_DIV1_FREQUENCY > SAM_PLL0_MAX_FREQUENCY)
#    error PLL0 VCO frequency is out of range
#  endif

/* PLL0 Options:
 *
 * PLL0 supports an option to divide the frequency output by 2.  We
 * will do this division to bring the internal VCO frequency up to the
 * minimum value
 *
 * PLL0 operates in two frequency ranges as determined by
 * SCIF_PLL0_PLLOPT_FVO:
 *
 * 0: 80MHz  < fvco < 180MHz
 * 1: 160MHz < fvco < 240MHz
 *
 * Select the correct frequency range using the recommended threshold
 * value.
 */

#  if SAM_PLL0_VCO_DIV1_FREQUENCY < (2*SAM_PLL0_MIN_FREQUENCY) && BOARD_PLL0_MUL <= 8
#    define SAM_PLL0_VCO_FREQUENCY   (2 * SAM_PLL0_VCO_DIV1_FREQUENCY)
#    define SAM_PLL0_MUL             (2 * BOARD_PLL0_MUL)

#    if SAM_PLL0_VCO_FREQUENCY > (SAM_PLL0_VCO_RANGE_THRESHOLD / 2)
#      define SAM_PLL0_OPTIONS       (SCIF_PLL0_PLLOPT_DIV2 | SCIF_PLL0_PLLOPT_FVO)
#    else
#      define SAM_PLL0_OPTIONS       SCIF_PLL0_PLLOPT_DIV2
#    endif

#  else
#    define SAM_PLL0_VCO_FREQUENCY   SAM_PLL0_VCO_DIV1_FREQUENCY
#    define SAM_PLL0_MUL             BOARD_PLL0_MUL

#    if SAM_PLL0_VCO_FREQUENCY > SAM_PLL0_VCO_RANGE_THRESHOLD
#      define SAM_PLL0_OPTIONS       SCIF_PLL0_PLLOPT_FVO
#    else
#      define SAM_PLL0_OPTIONS       0
#    endif
#  endif
#endif

/* DFLL0 */

#ifdef BOARD_SYSCLK_SOURCE_DFLL0
/* DFLL0 reference clock */

#  if defined(BOARD_DFLL0_SOURCE_RCSYS)
#    define SAM_DFLLO_REFCLK         SCIF_GCCTRL_OSCSEL_RCSYS
#  elif defined(BOARD_DFLL0_SOURCE_OSC32K)
#    define SAM_DFLLO_REFCLK         SCIF_GCCTRL_OSCSEL_OSC32K
#  elif defined(BOARD_DFLL0_SOURCE_OSC0)
#    define SAM_DFLLO_REFCLK         SCIF_GCCTRL_OSCSEL_OSC0
#  elif defined(BOARD_DFLL0_SOURCE_RC80M)
#    define SAM_DFLLO_REFCLK         SCIF_GCCTRL_OSCSEL_RC80M
#  elif defined(BOARD_DFLL0_SOURCE_RC32K)
#    define SAM_DFLLO_REFCLK         SCIF_GCCTRL_OSCSEL_RC32K
#  else
#    error No DFLL0 source for reference clock defined
#  endif

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_picocache
 *
 * Description:
 *   Initialize the PICOCACHE.
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_PICOCACHE
static inline void sam_picocache(void)
{
  /* Enable clocking to the PICOCACHE */

  sam_hsb_enableperipheral(PM_HSBMASK_HRAMC1);
  sam_pbb_enableperipheral(PM_PBBMASK_HRAMC1);

  /* Enable the PICOCACHE and wait for it to become ready */

  putreg32(PICOCACHE_CTRL_CEN, SAM_PICOCACHE_CTRL);
  while ((getreg32(SAM_PICOCACHE_SR) & PICOCACHE_SR_CSTS) == 0);
}
#else
#  define sam_picocache()
#endif

/****************************************************************************
 * Name: sam_enableosc0
 *
 * Description:
 *   Initialize OSC0 settings per the definitions in the board.h file.
 *
 ****************************************************************************/

#ifdef NEED_OSC0
static inline void sam_enableosc0(void)
{
  uint32_t regval;

  /* Enable and configure OSC0 */

  regval = SAM_OSC0_STARTUP_VALUE | SAM_OSC0_GAIN_VALUE | SAM_OSC0_MODE_VALUE |
           SCIF_OSCCTRL0_OSCEN;
  putreg32(SCIF_UNLOCK_KEY(0xaa) | SCIF_UNLOCK_ADDR(SAM_SCIF_OSCCTRL0_OFFSET),
           SAM_SCIF_UNLOCK);
  putreg32(regval, SAM_SCIF_OSCCTRL0);

  /* Wait for OSC0 to be ready */

  while (getreg32(SAM_SCIF_PCLKSR) & SCIF_INT_OSC0RDY) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enableosc32
 *
 * Description:
 *   Initialize the 32KHz oscillator per settings in the board.h header
 *   file.
 *
 ****************************************************************************/

#ifdef NEED_OSC32K
static inline void sam_enableosc32(void)
{
  uint32_t regval;

  /* Set up the OSCCTRL32 register using settings from the board.h file.
   * Also  enable the oscillator and provide bother the 32KHz and 1KHz output.
   */

  regval = SAM_OSC32_STARTUP_VALUE | BOARD_OSC32_SELCURR | SAM_OSC32_MODE_VALUE |
           BSCIF_OSCCTRL32_EN1K |  BSCIF_OSCCTRL32_EN32K |
           BSCIF_OSCCTRL32_OSC32EN;

  putreg32(BSCIF_UNLOCK_KEY(0xaa) | BSCIF_UNLOCK_ADDR(SAM_BSCIF_OSCCTRL32_OFFSET),
           SAM_BSCIF_UNLOCK);
  putreg32(regval, SAM_BSCIF_OSCCTRL32);

  /* Wait for OSC32 to be ready */

  while ((getreg32(SAM_BSCIF_PCLKSR) & BSCIF_INT_OSC32RDY) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enablerc80m
 *
 * Description:
 *   Initialize the 80 MHz RC oscillator per settings in the board.h header
 *   file.
 *
 ****************************************************************************/

#ifdef NEED_RC80M
static inline void sam_enablerc80m(void)
{
  uint32_t regval;

  /* Configure and enable RC80M */

  regval = getreg32(SAM_SCIF_RC80MCR);
  putreg32(SCIF_UNLOCK_KEY(0xaa) | SCIF_UNLOCK_ADDR(SAM_SCIF_RC80MCR_OFFSET),
           SAM_SCIF_UNLOCK);
  putreg32(regval | SCIF_RC80MCR_EN, SAM_SCIF_RC80MCR);

  /* Wait for OSC32 to be ready */

  while (getreg32(SAM_SCIF_RC80MCR) & SCIF_RC80MCR_EN) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enablerc80m
 *
 * Description:
 *   Initialize the 12/8/4 RC fast oscillator per settings in the board.h
 *   header file.
 *
 ****************************************************************************/

#ifdef NEED_RCFAST
static inline void sam_enablercfast(void)
{
  uint32_t regval;

  /* Configure and enable RCFAST */

  regval  = getreg32(SAM_SCIF_RCFASTCFG);
  regval &= ~SCIF_RCFASTCFG_FRANGE_MASK;
  regval |= (SAM_RCFAST_RANGE | SCIF_RCFASTCFG_EN);

  putreg32(SCIF_UNLOCK_KEY(0xaa) | SCIF_UNLOCK_ADDR(SAM_SCIF_RCFASTCFG_OFFSET),
           SAM_SCIF_UNLOCK);
  putreg32(regval, SAM_SCIF_RCFASTCFG);

  /* Wait for RCFAST to be ready */

  while (getreg32(SAM_SCIF_RCFASTCFG) & SCIF_RCFASTCFG_EN) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enablerc1m
 *
 * Description:
 *   Initialize the 1M RC oscillator per settings in the board.h header
 *   file.
 *
 ****************************************************************************/

#ifdef NEED_RC1M
static inline void sam_enablerc1m(void)
{
  uint32_t regval;

  /* Configure and enable RC1M */

  regval  = getreg32(SAM_BSCIF_RC1MCR);
  regval &= ~BSCIF_RCFASTCFG_FRANGE_MASK;
  regval |= (SAM_RCFAST_RANGE | BSCIF_RCFASTCFG_EN);

  putreg32(BSCIF_UNLOCK_KEY(0xaa) | BSCIF_UNLOCK_ADDR(SAM_BSCIF_RC1MCR_OFFSET),
           SAM_BSCIF_UNLOCK);
  putreg32(regval  | BSCIF_RC1MCR_CLKOEN, SAM_BSCIF_RC1MCR);

  /* Wait for RCFAST to be ready */

  while (getreg32(SAM_BSCIF_RC1MCR) & BSCIF_RC1MCR_CLKOEN) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enablerc32k
 *
 * Description:
 *   Initialize the 23KHz RC oscillator per settings in the board.h header
 *   file.
 *
 ****************************************************************************/

#ifdef NEED_RC32K
static inline void sam_enablerc32k(void)
{
  uint32_t regval;

  /* Configure and enable RC32K */

  regval  = getreg32(SAM_BSCIF_RC32KCR);
  putreg32(BSCIF_UNLOCK_KEY(0xaa) | BSCIF_UNLOCK_ADDR(SAM_BSCIF_RC32KCR_OFFSET),
           SAM_BSCIF_UNLOCK);
  putreg32(regval | BSCIF_RC32KCR_EN32K | BSCIF_RC32KCR_EN, SAM_BSCIF_RC32KCR);

  /* Wait for RCFAST to be ready */

  while (getreg32(SAM_BSCIF_RC32KCR) & BSCIF_RC32KCR_EN) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enableglck9
 *
 * Description:
 *   Enable GLCK9.
 *
 ****************************************************************************/

#ifdef NEED_GLCK9
static inline void sam_enableglck9(void)
{
  /* Enable the generic clock using the source specified in the board.h
   * file.  No division is used so that the GCLK9 frequency is the same
   * as the source frequency.
   */

  putreg32(SAM_GCLK9_SOURCE_VALUE | SCIF_GCCTRL_CEN, SAM_SCIF_GCCTRL9);
}
#endif

/****************************************************************************
 * Name: sam_enablepll0 (and its helper sam_pll0putreg())
 *
 * Description:
 *   Initialize PLL0 settings per the definitions in the board.h file.
 *
 ****************************************************************************/

#ifdef BOARD_SYSCLK_SOURCE_PLL0
static inline void sam_pll0putreg(uint32_t regval, uint32_t regaddr,
                                  uint32_t regoffset)
{
  putreg32(SCIF_UNLOCK_KEY(0xaa) | SCIF_UNLOCK_ADDR(regoffset),
           SAM_SCIF_UNLOCK);
  putreg32(regval, regaddr);
}

static inline void sam_enablepll0(void)
{
  uint32_t regval;

  /* Clear the PLL0 control register */

  sam_pll0putreg(0, SAM_SCIF_PLL0, SAM_SCIF_PLL0_OFFSET);

  /* Write the selected options */

  regval  = getreg32(SAM_SCIF_PLL0);
  regval &= SCIF_PLL0_PLLOPT_MASK;
  regval |= SAM_PLL0_OPTIONS;
  sam_pll0putreg(regval, SAM_SCIF_PLL0, SAM_SCIF_PLL0_OFFSET);

  /* Set up the multiers and dividers */

  regval  = getreg32(SAM_SCIF_PLL0);
  regval &= ~(SCIF_PLL0_PLLOSC_MASK | SCIF_PLL0_PLLDIV_MASK | SCIF_PLL0_PLLMUL_MASK);
  regval |= ((SAM_PLL0_MUL - 1) << SCIF_PLL0_PLLMUL_SHIFT) |
            (BOARD_DFLL0_DIV << SCIF_PLL0_PLLDIV_SHIFT) |
            SCIF_PLL0_PLLCOUNT_MAX | SAM_PLL0_SOURCE;
  sam_pll0putreg(regval, SAM_SCIF_PLL0, SAM_SCIF_PLL0_OFFSET);

  /* And, finally, enable PLL0 */

  regval  = getreg32(SAM_SCIF_PLL0);
  regval |= SCIF_PLL_PLLEN;
  sam_pll0putreg(regval, SAM_SCIF_PLL0, SAM_SCIF_PLL0_OFFSET);

  /* Wait for PLL0 to become locked */

  while ((getreg32(SAM_SCIF_PCLKSR) & SCIF_INT_PLL0LOCK) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enabledfll0 (and its helper sam_dfll0_putreg32())
 *
 * Description:
 *   Initialize DFLL0 settings per the definitions in the board.h file.
 *
 ****************************************************************************/

#ifdef BOARD_SYSCLK_SOURCE_DFLL0
static inline void sam_dfll0_putreg32(uint32_t regval, uint32_t regaddr,
                                     uint32_t regoffset)
{
  /* Wait until DFLL0 is completes the last setting */

  while ((getreg32(SAM_SCIF_PCLKSR) & SCIF_INT_DFLL0RDY) == 0);

  /* Then unlock the register and write the next value */

  putreg32(SCIF_UNLOCK_KEY(0xaa) | SCIF_UNLOCK_ADDR(regoffset),
           SAM_SCIF_UNLOCK);
  putreg32(regval, regaddr);
}

static inline void sam_enabledfll0(void)
{
  uint32_t regval;
  uint32_t conf;

  /* Set up generic clock source with specified reference clock
   * and divider.
   */

  putreg32(0, SAM_SCIF_GCCTRL0);

  /* Set the generic clock 0 source */

  regval  = getreg32(SAM_SCIF_GCCTRL0);
  regval &= ~SCIF_GCCTRL_OSCSEL_MASK;
  regval |= SAM_DFLLO_REFCLK;
  putreg32(regval, SAM_SCIF_GCCTRL0);

  /* Get the generic clock 0 divider */

  regval  = getreg32(SAM_SCIF_GCCTRL0);
  regval &= ~(SCIF_GCCTRL_DIVEN | SCIF_GCCTRL_DIV_MASK);

#if BOARD_DFLL0_DIV > 1
  regval |= SCIF_GCCTRL_DIVEN;
  regval |= SCIF_GCCTRL_DIV(((BOARD_DFLL0_DIV + 1) / 2) - 1);
#endif

  putreg32(regval, SAM_SCIF_GCCTRL0);

  /* Sync before reading a dfll conf register */

  putreg32(SCIF_DFLL0SYNC_SYNC, SAM_SCIF_DFLL0SYNC);
  while ((getreg32(SAM_SCIF_PCLKSR) & SCIF_INT_DFLL0RDY) == 0);

  /* Select Closed Loop Mode */

  conf  = getreg32(SAM_SCIF_DFLL0CONF);
  conf &= ~SCIF_DFLL0CONF_RANGE_MASK;
  conf |= SCIF_DFLL0CONF_MODE;

  /* Select the DFLL0 Frequency Range */

#if BOARD_DFLL0_FREQUENCY < SCIF_DFLL0CONF_MAX_RANGE3
  conf |= SCIF_DFLL0CONF_RANGE3;
#elif BOARD_DFLL0_FREQUENCY < SCIF_DFLL0CONF_MAX_RANGE2
  conf |= SCIF_DFLL0CONF_RANGE2;
#elif BOARD_DFLL0_FREQUENCY < SCIF_DFLL0CONF_MAX_RANGE1
  conf |= SCIF_DFLL0CONF_RANGE1;
#else
  conf |= SCIF_DFLL0CONF_RANGE0;
#endif

  /* Enable the reference generic clock 0 */

  regval  = getreg32(SAM_SCIF_GCCTRL0);
  regval |= SCIF_GCCTRL_CEN;
  putreg32(regval, SAM_SCIF_GCCTRL0);

  /* Enable DFLL0.  Here we assume DFLL0RDY because the DFLL was disabled
   * before this function was called.
   */

  putreg32(SCIF_UNLOCK_KEY(0xaa) | SCIF_UNLOCK_ADDR(SAM_SCIF_DFLL0CONF_OFFSET),
           SAM_SCIF_UNLOCK);
  putreg32(SCIF_DFLL0CONF_EN, SAM_SCIF_DFLL0CONF);

  /* Configure DFLL0.  Note that now we do have to wait for DFLL0RDY before
   * every write.
   *
   * Set the initial coarse and fine step lengths to 4. If this is set
   * too high, DFLL0 may fail to lock.
   */

  sam_dfll0_putreg32(SCIF_DFLL0STEP_CSTEP(4) | SCIF_DFLL0STEP_FSTEP(4),
                     SAM_SCIF_DFLL0STEP,
                     SAM_SCIF_DFLL0STEP_OFFSET);

  /* Set the DFLL0 multipler register */

  sam_dfll0_putreg32(BOARD_DFLL0_MUL, SAM_SCIF_DFLL0MUL,
                     SAM_SCIF_DFLL0MUL_OFFSET);

  /* Set the multipler and spread spectrum generator control registers */

  sam_dfll0_putreg32(0, SAM_SCIF_DFLL0SSG, SAM_SCIF_DFLL0SSG_OFFSET);

  /* Finally, set the DFLL0 configuration */

  sam_dfll0_putreg32(conf | SCIF_DFLL0CONF_EN,
                     SAM_SCIF_DFLL0CONF, SAM_SCIF_DFLL0CONF_OFFSET);

  /* Wait until we are locked on the fine value */

  while ((getreg32(SAM_SCIF_PCLKSR) & SCIF_INT_DFLL0LOCKF) == 0);
}
#endif

/****************************************************************************
 * Name: sam_setdividers
 *
 * Description:
 *   Configure derived clocks.
 *
 ****************************************************************************/

static inline void sam_setdividers(void)
{
  uint32_t cpusel;
  uint32_t pbasel;
  uint32_t pbbsel;
  uint32_t pbcsel;
  uint32_t pbdsel;

  /* Get the register setting for each divider value */

#if BOARD_CPU_SHIFT > 0
  cpusel = (PM_CPUSEL(BOARD_CPU_SHIFT - 1)) | PM_CPUSEL_DIV;
#else
  cpusel = 0;
#endif

#if BOARD_PBA_SHIFT > 0
  pbasel = (PM_PBSEL(BOARD_PBA_SHIFT - 1)) | PM_PBSEL_DIV;
#else
  pbasel = 0;
#endif

#if BOARD_PBB_SHIFT  >0
  pbbsel = (PM_PBSEL(BOARD_PBB_SHIFT - 1)) | PM_PBSEL_DIV;
#else
  pbbsel = 0;
#endif

#if BOARD_PBC_SHIFT > 0
  pbcsel = (PM_PBSEL(BOARD_PBC_SHIFT - 1)) | PM_PBSEL_DIV;
#else
  pbcsel = 0;
#endif

#if BOARD_PBD_SHIFT > 0
  pbdsel = (PM_PBSEL(BOARD_PBD_SHIFT - 1)) | PM_PBSEL_DIV;
#else
  pbdsel = 0;
#endif

  /* Then set the divider values. */

  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_CPUSEL_OFFSET), SAM_PM_UNLOCK);
  putreg32(cpusel, SAM_PM_CPUSEL);

  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_PBASEL_OFFSET), SAM_PM_UNLOCK);
  putreg32(pbasel, SAM_PM_PBASEL);

  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_PBBSEL_OFFSET), SAM_PM_UNLOCK);
  putreg32(pbbsel, SAM_PM_PBBSEL);

  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_PBCSEL_OFFSET), SAM_PM_UNLOCK);
  putreg32(pbcsel, SAM_PM_PBCSEL);

  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_PBDSEL_OFFSET), SAM_PM_UNLOCK);
  putreg32(pbdsel, SAM_PM_PBDSEL);
}

/****************************************************************************
 * Name: sam_enable_fastwakeup
 *
 * Description:
 *   Enable FLASH fast wakeup mode.
 *
 ****************************************************************************/

static inline void sam_enable_fastwakeup(void)
{
  uint32_t regval;

  regval  = getreg32(SAM_BPM_PMCON);
  regval |= BPM_PMCON_FASTWKUP;
  putreg32(BPM_UNLOCK_KEY(0xaa) | BPM_UNLOCK_ADDR(SAM_BPM_PMCON_OFFSET),
          SAM_BPM_UNLOCK);
  putreg32(regval, SAM_BPM_PMCON);
}

/****************************************************************************
 * Name: set_flash_waitstate
 *
 * Description:
 *   Setup one or two FLASH wait states.
 *
 ****************************************************************************/

static inline void set_flash_waitstate(bool waitstate)
{
  uint32_t regval;

  /* Set or clear the FLASH wait state (FWS) bit in the FLASH control
   * register (FCR).
   */

  regval = getreg32(SAM_FLASHCALW_FCR);

  if (waitstate)
    {
      regval |= FLASHCALW_FCR_FWS;
    }
  else
    {
      regval &= ~FLASHCALW_FCR_FWS;
    }

  putreg32(regval, SAM_FLASHCALW_FCR);
}

/****************************************************************************
 * Name: sam_flash_readmode
 *
 * Description:
 *   Send a FLASH command to enable to disable high speed FLASH read mode.
 *
 ****************************************************************************/

static inline void sam_flash_readmode(uint32_t command)
{
  uint32_t regval;

  /* Make sure that any previous FLASH operation is completed */

  while ((getreg32(SAM_FLASHCALW_FSR) & FLASHCALW_FSR_FRDY) == 0);

  /* Write the specified FLASH command to the FCMD register */

  regval  = getreg32(SAM_FLASHCALW_FCMD);
  regval &= ~FLASHCALW_FCMD_CMD_MASK;
  regval |= (FLASHCALW_FCMD_KEY | command);
  putreg32(regval, SAM_FLASHCALW_FCMD);

  /* Wait for this FLASH operation to complete */

  while ((getreg32(SAM_FLASHCALW_FSR) & FLASHCALW_FSR_FRDY) == 0);
}

/****************************************************************************
 * Name: sam_flash_config
 *
 * Description:
 *   Configure FLASH read mode and wait states.
 *
 *   Maximum CPU frequency for 0 and 1 FLASH wait states (FWS) in various modes
 *   (Table 42-30 in the big data sheet).
 *
 *     ------- ------------------- ---------- ----------
 *     Power     Flash Read Mode     Flash     Maximum
 *     Sclaing                        Wait    Operating
 *     Mode    HSEN HSDIS FASTWKUP   States   Frequency
 *     ------- ---- ----- -------- ---------- ----------
 *       PS0          X       X        1        12MHz
 *       " "          X                0        18MHz
 *       " "          X                1        36MHz
 *       PS1          X       X        1        12MHz
 *       " "          X                0         8MHz
 *       " "          X                1        12MHz
 *       PS2     X                     0        24Mhz
 *       " "     X                     1        48MHz
 *     ------- ---- ----- -------- ---------- ----------
 *
 ****************************************************************************/

static inline void sam_flash_config(uint32_t cpuclock, uint32_t psm, bool fastwkup)
{
  bool waitstate;
  uint32_t command;

#ifdef CONFIG_SAM34_FLASH_HSEN
  /* High speed flash read mode (with power scaling mode == 2).  Set one
   * wait state if the CPU clock frequency exceeds the threshold value
   * and enable high speed read mode.
   */

  waitstate = (cpuclock > FLASH_MAXFREQ_PS2_HSEN_FWS0);
  command   = FLASHCALW_FCMD_CMD_HSEN;
#else
  /* Assume that we will select no wait states and that we will disable high-
   * speed read mode.
   */

  waitstate = false;
  command   = FLASHCALW_FCMD_CMD_HSDIS;

  /* Handle power scaling mode == 0 FLASH configuration */

  if (psm == 0)
    {
      /* Power scaling mode 0.  We need to set wait state the CPU clock if
       * the CPU frequency exceeds a threshold.
       */

      if (cpuclock > FLASH_MAXFREQ_PS0_HSDIS_FWS0)
        {
          /* Set one wait state */

          waitstate = true;

          /* Enable high speed read mode if the frequency exceed the maximum
           * for the low speed configuration.  This mode is not documented
           * in the data sheet, but I see that they do this in some Atmel
           * code examples.
           */

          if (cpuclock > FLASH_MAXFREQ_PS0_HSDIS_FWS1)
            {
              /* Enable high speed read mode. */

              command = FLASHCALW_FCMD_CMD_HSEN;
            }
        }

      /* The is below the threshold that requires one wait state.  But we
       * have to check a few more things.
       */

      else
        {
          /* If FLASH wake-up mode is selected and the we are in the lower
           * operating frequency for this mode, then set 1 waitate and
           * disable high speed read mode.
           */

          if ((fastwkup == true) &&
              (cpuclock <= FLASH_MAXFREQ_PS1_HSDIS_FASTWKUP_FWS1))
            {
              /* Set one wait state */

              waitstate = true;
            }
        }
    }

  /* Otherwise, this is power scaling mode 1 */

  else /* if (psm == 1) */
    {
      /* If we are in the lower operating frequency range, then select
       * zero wait states.  Otherwise, select one wait state.
       */

      if (cpuclock > FLASH_MAXFREQ_PS1_HSDIS_FWS0)
        {
          /* Set one wait state */

          waitstate = true;
        }
    }

#endif

  /* Set 0 or 1 waitstates */

  set_flash_waitstate(waitstate);

  /* Enable/disable the high-speed read mode. */

  sam_flash_readmode(command);
}

/****************************************************************************
 * Name: sam_mainclk
 *
 * Description:
 *   Select the main clock.
 *
 ****************************************************************************/

static inline void sam_mainclk(uint32_t mcsel)
{
  uint32_t regval;

  regval = getreg32(SAM_PM_MCCTRL);
  regval &= ~PM_MCCTRL_MCSEL_MASK;
  regval |= mcsel;

  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_MCCTRL_OFFSET),
           SAM_PM_UNLOCK);
  putreg32(regval, SAM_PM_MCCTRL);
}

/****************************************************************************
 * Name: sam_setpsm (and its helper, sam_instantiatepsm())
 *
 * Description:
 *   Switch to the selected power scaling mode.
 *
 ****************************************************************************/

static __ramfunc__ void sam_instantiatepsm(uint32_t regval)
{
  /* Set the BMP PCOM register (containing the new power scaling mode) */

  putreg32(BPM_UNLOCK_KEY(0xaa) | BPM_UNLOCK_ADDR(SAM_BPM_PMCON_OFFSET),
           SAM_BPM_UNLOCK);
  putreg32(regval, SAM_BPM_PMCON);

  /* Wait for new power scaling mode to become active.  There should be
   * timeout on this wait.
   */

  while ((getreg32(SAM_BPM_SR) & BPM_INT_PSOK) == 0);
}

static inline void sam_setpsm(uint32_t psm)
{
  uint32_t regval;

  /* Setup the PMCON register content for the new power scaling mode */

  regval  = getreg32(SAM_BPM_PMCON);
  regval &= ~BPM_PMCON_PS_MASK;
  regval |= (psm | BPM_PMCON_PSCM | BPM_PMCON_PSCREQ);

  /* Then call the RAMFUNC sam_setpsm() to set the new power scaling mode */

  sam_instantiatepsm(regval);
}

/****************************************************************************
 * Name: sam_usbclock
 *
 * Description:
 *   Setup the USBB GCLK.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV
static inline void sam_usbclock(void)
{
  uint32_t regval = 0;

#if defined(SAM_CLOCK_USB_PLL0) || defined(SAM_CLOCK_USB_PLL1)
  regval |= PM_GCCTRL_PLLSEL;
#endif
#if defined(SAM_CLOCK_USB_OSC1) || defined(SAM_CLOCK_USB_PLL1)
  regval |= PM_GCCTRL_OSCSEL;
#endif
#if SAM_CLOCK_USB_DIV > 0


  u_avr32_pm_gcctrl.GCCTRL.diven  = diven;
  u_avr32_pm_gcctrl.GCCTRL.div    = div;
#endif
  putreg32(regval, SAM_PM_GCCTRL(SAM_PM_GCLK_USBB))

  /* Enable USB GCLK */

  regval = getreg32(SAM_PM_GCCTRL(SAM_PM_GCLK_USBB))
  regval |= PM_GCCTRL_CEN;
  putreg32(regval, SAM_PM_GCCTRL(SAM_PM_GCLK_USBB))
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_clockconfig
 *
 * Description:
 *   Called to initialize the SAM3/4.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void sam_clockconfig(void)
{
  uint32_t psm;
  bool fastwkup;

  /* Enable clocking to the PICOCACHE */

  sam_picocache();

  /* Configure dividers for derived clocks.  These divider definitions must
   * be provided in the board.h header file.
   */

  sam_setdividers();

  /* Select a power scaling mode and possible fast wakeup so that we get the
   * best possible flash performance.  The following table shows the maximum
   * CPU frequency for 0 and 1 FLASH wait states (FWS) in various modes
   * (Table 42-30 in the big data sheet).
   *
   *   ------- ------------------- ---------- ----------
   *   Power     Flash Read Mode     Flash     Maximum
   *   Sclaing                        Wait    Operating
   *   Mode    HSEN HSDIS FASTWKUP   States   Frequency
   *   ------- ---- ----- -------- ---------- ----------
   *     PS0          X       X        1        12MHz
   *     " "          X                0        18MHz
   *     " "          X                1        36MHz
   *     PS1          X       X        1        12MHz
   *     " "          X                0         8MHz
   *     " "          X                1        12MHz
   *     PS2     X                     0        24Mhz
   *     " "     X                     1        48MHz
   *   ------- ---- ----- -------- ---------- ----------
   */

#ifdef CONFIG_SAM34_FLASH_HSEN
  /* The high speed FLASH mode has been enabled.  Select power scaling
   * mode 2, no fast wakeup.
   */

  psm      = BPM_PMCON_PS2;
  fastwkup = false;

#elif BOARD_CPU_FREQUENCY <= FLASH_MAXFREQ_PS1_HSDIS_FWS1
  /* Not high speed mode and frequency is below the thrshold.  We can go to
   * power scaling mode 1.
   */

  psm = BPM_PMCON_PS1;

#  if BOARD_CPU_FREQUENCY > FLASH_MAXFREQ_PS1_HSDIS_FWS0
  /* We need to enable fast wakeup */

  sam_enable_fastwakeup()
  fastwkup = true;
#  endif
#else
  /* Power scaling mode 0, disable high speed mode, no fast wakeup */

  psm      = BPM_PMCON_PS0;
  fastwkup = false;
#endif

  /* Enable clock sources:
   *
   * OSC0:  Might by the system clock or the source clock for PLL0 or DFLL0
   * OSC32: Might be source clock for DFLL0
   */

#ifdef NEED_OSC0
  /* Enable OSC0 using the settings in board.h */

  sam_enableosc0();
#endif

#ifdef NEED_OSC32K
  /* Enable the 32KHz oscillator using the settings in board.h */

  sam_enableosc32();
#endif

#ifdef NEED_RC80M
  /* Enable the 32KHz oscillator using the settings in board.h */

  sam_enablerc80m();
#endif

#ifdef NEED_RCFAST
  /* Enable the 12/8/4MHz RC fast oscillator using the settings in board.h */

  sam_enablercrcfast();
#endif

#ifdef NEED_RC1M
  /* Enable the 1MHz RC oscillator using the settings in board.h */

  sam_enablerc1m();
#endif

#ifdef NEED_RC32K
  /* Enable the 32KHz RC oscillator using the settings in board.h */

  sam_enablerc32k();
#endif

#ifdef NEED_GLCK9
  /* Enable the GLCK9 */

  sam_enableglck9();
#endif

  /* Switch to the system clock selected by the settings in the board.h
   * header file.
   */

#if defined(BOARD_SYSCLK_SOURCE_RCSYS)
  /* Since this function only executes at power up, we know that we are
   * already running from RCSYS.
   */

  // sam_mainclk(PM_MCCTRL_MCSEL_RCSYS);
#elif defined(BOARD_SYSCLK_SOURCE_OSC0)

  /* Configure FLASH read mode and wait states */

  sam_flash_config(BOARD_CPU_FREQUENCY, psm, fastwkup);

  /* Then switch the main clock to OSC0 */

  sam_mainclk(PM_MCCTRL_MCSEL_OSC0);

#elif defined(BOARD_SYSCLK_SOURCE_PLL0)

  /* Enable PLL0 using the settings in board.h */

  sam_enablepll0();

  /* Configure FLASH read mode and wait states */

  sam_flash_config(BOARD_CPU_FREQUENCY, psm, fastwkup);

  /* Then switch the main clock to PLL0 */

  sam_mainclk(PM_MCCTRL_MCSEL_PLL);

#elif defined(BOARD_SYSCLK_SOURCE_DFLL0)

  /* Enable PLL0 using the settings in board.h */

  sam_enabledfll0();

  /* Configure FLASH read mode and wait states */

  sam_flash_config(BOARD_CPU_FREQUENCY, psm, fastwkup);

  /* Then switch the main clock to DFLL0 */

  sam_mainclk(PM_MCCTRL_MCSEL_DFLL);

#elif defined(BOARD_SYSCLK_SOURCE_RC80M)

  /* Configure FLASH read mode and wait states */

  sam_flash_config(BOARD_CPU_FREQUENCY, psm, fastwkup);

  /* Then switch the main clock to RCM80 */

  sam_mainclk(PM_MCCTRL_MCSEL_RC80M);

#elif defined(BOARD_SYSCLK_SOURCE_FCFAST12M) || defined(BOARD_SYSCLK_SOURCE_FCFAST8M) || \
      defined(BOARD_SYSCLK_SOURCE_FCFAST4M)

  /* Configure FLASH read mode and wait states */

  sam_flash_config(BOARD_CPU_FREQUENCY, psm, fastwkup);

  /* Then switch the main clock to RCFAST */

  sam_mainclk(PM_MCCTRL_MCSEL_RCFAST);

#elif defined(BOARD_SYSCLK_SOURCE_RC1M)

  /* Configure FLASH read mode and wait states */

  sam_flash_config(BOARD_CPU_FREQUENCY, psm, fastwkup);

  /* Then switch the main clock to RC1M */

  sam_mainclk(PM_MCCTRL_MCSEL_RC1M);

#else
#  error "No SYSCLK source provided"
#endif

  /* Switch to the selected power scaling mode */

  sam_setpsm(psm);

  /* Enable all selected peripheral cloks */

  sam_init_periphclks();

  /* Configure clocking to the USB controller */

#ifdef CONFIG_USBDEV
  sam_usbc_enableclk();
#endif
}
