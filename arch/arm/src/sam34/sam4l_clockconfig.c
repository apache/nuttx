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

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "up_internal.h"
#include "chip/sam4l_pm.h"
#include "chip/sam4l_scif.h"
#include "chip/sam4l_bpm.h"
#include "chip/sam4l_bscif.h"
#include "chip/sam4l_flashcalw.h"

#include "sam_clockconfig.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/
/* Nominal frequencies in on-chip RC oscillators.  These may frequencies
 * may vary with temperature changes.
 */

#define SAM_RCSYS_FREQUENCY       115000 /* Nominal frequency of RCSYS (Hz) */
#define SAM_RC32K_FREQUENCY        32768 /* Nominal frequency of RC32K (Hz) */
#define SAM_RC80M_FREQUENCY     80000000 /* Nominal frequency of RC80M (Hz) */
#define SAM_RCFAST4M_FREQUENCY   4000000 /* Nominal frequency of RCFAST4M (Hz) */
#define SAM_RCFAST8M_FREQUENCY   8000000 /* Nominal frequency of RCFAST8M (Hz) */
#define SAM_RCFAST12M_FREQUENCY 12000000 /* Nominal frequency of RCFAST12M (Hz) */
#define SAM_RC1M_FREQUENCY       1000000 /* Nominal frequency of RC1M (Hz) */

/* Oscillator 0.  This might be the system clock or the source clock for
 * either PLL0 or DFPLL.
 *
 * By selecting CONFIG_SAM_OSC0, you can also force the clock to be enabled
 * at boot time.
 */

#if defined(CONFIG_SAM_OSC0) || defined(BOARD_SYSCLK_SOURCE_OSC0) || \
    defined(BOARD_DFLL0_SOURCE_OSC0) || defined(BOARD_PLL0_SOURCE_OSC0)
#  define NEED_OSC0               1
#endif

#ifdef NEED_OSC0
#  if !defined(BOARD_OSC0_STARTUP_US)
#    error BOARD_OSC0_STARTUP_US is not defined
#  elif BOARD_OSC0_STARTUP_US == 0
#    define OSC0_STARTUP_VALUE    SCIF_OSCCTRL0_STARTUP_0
#    define OSC0_STARTUP_TIMEOUT  8
#  elif BOARD_OSC0_STARTUP_US <= 557
#    define OSC0_STARTUP_VALUE    SCIF_OSCCTRL0_STARTUP_64
#    define OSC0_STARTUP_TIMEOUT  80
#  elif BOARD_OSC0_STARTUP_US <= 1100
#    define OSC0_STARTUP_VALUE    SCIF_OSCCTRL0_STARTUP_128
#    define OSC0_STARTUP_TIMEOUT  160
#  elif BOARD_OSC0_STARTUP_US <= 18000
#    define OSC0_STARTUP_VALUE    SCIF_OSCCTRL0_STARTUP_2K
#    define OSC0_STARTUP_TIMEOUT  2560
#  elif BOARD_OSC0_STARTUP_US <= 36000
#    define OSC0_STARTUP_VALUE    SCIF_OSCCTRL0_STARTUP_4K
#    define OSC0_STARTUP_TIMEOUT  5120
#  elif BOARD_OSC0_STARTUP_US <= 71000
#    define OSC0_STARTUP_VALUE    SCIF_OSCCTRL0_STARTUP_8K
#    define OSC0_STARTUP_TIMEOUT  10240
#  elif BOARD_OSC0_STARTUP_US <= 143000
#    define OSC0_STARTUP_VALUE    SCIF_OSCCTRL0_STARTUP_16K
#    define OSC0_STARTUP_TIMEOUT  20480
#  elif BOARD_OSC0_STARTUP_US <= 285000
#    define OSC0_STARTUP_VALUE    SCIF_OSCCTRL0_STARTUP_32K
#    define OSC0_STARTUP_TIMEOUT  40960
#  else
#    error BOARD_OSC0_STARTUP_US is out of range
#  endif

#  ifdef BOARD_OSC0_IS_XTAL
#    define OSC0_MODE_VALUE       SCIF_OSCCTRL0_MODE
#    if BOARD_OSC0_FREQUENCY < 2000000
#      define OSC0_GAIN_VALUE     SCIF_OSCCTRL0_GAIN(0)
#    elif BOARD_OSC0_FREQUENCY < 4000000
#      define OSC0_GAIN_VALUE     SCIF_OSCCTRL0_GAIN(1)
#    elif BOARD_OSC0_FREQUENCY < 8000000
#      define OSC0_GAIN_VALUE     SCIF_OSCCTRL0_GAIN(2)
#    elif BOARD_OSC0_FREQUENCY < 16000000
#      define OSC0_GAIN_VALUE     SCIF_OSCCTRL0_GAIN(3)
#    else
#      define OSC0_GAIN_VALUE     ((0x1u << 4) | SCIF_OSCCTRL0_GAIN(0))
#    endif
#  else
#    define OSC0_MODE_VALUE       0
#    define OSC0_GAIN_VALUE       0
#  endif
#endif

/* OSC32.  The 32K oscillator may be the source clock for DFPLL0.
 *
 * By selecting CONFIG_SAM_OSC32K, you can also force the clock to be
 * enabled at boot time.  OSC32 may needed by other devices as well
 * (AST, WDT, PICUART, RTC).
 */

#if defined(CONFIG_SAM_OSC32K) || defined(BOARD_DFLL0_SOURCE_OSC32K)
#  define NEED_OSC32K             1
#endif

#ifdef NEED_OSC32K
#  if !defined(BOARD_OSC32_STARTUP_US)
#    error BOARD_OSC32_STARTUP_US is not defined
#  elif BOARD_OSC32_STARTUP_US == 0
#    define OSC32_STARTUP_VALUE   BSCIF_OSCCTRL32_STARTUP_0
#  elif BOARD_OSC32_STARTUP_US   <= 1100
#    define OSC32_STARTUP_VALUE   BSCIF_OSCCTRL32_STARTUP_128
#  elif BOARD_OSC32_STARTUP_US   <= 72300
#    define OSC32_STARTUP_VALUE   BSCIF_OSCCTRL32_STARTUP_8K
#  elif BOARD_OSC32_STARTUP_US   <= 143000
#    define OSC32_STARTUP_VALUE   BSCIF_OSCCTRL32_STARTUP_16K
#  elif BOARD_OSC32_STARTUP_US   <= 570000
#    define OSC32_STARTUP_VALUE   BSCIF_OSCCTRL32_STARTUP_64K
#  elif BOARD_OSC32_STARTUP_US   <= 1100000
#    define OSC32_STARTUP_VALUE   BSCIF_OSCCTRL32_STARTUP_128K
#  elif BOARD_OSC32_STARTUP_US   <= 2300000
#    define OSC32_STARTUP_VALUE   BSCIF_OSCCTRL32_STARTUP_256K
#  elif BOARD_OSC32_STARTUP_US   <= 4600000
#    define OSC32_STARTUP_VALUE   BSCIF_OSCCTRL32_STARTUP_512K
#  else
#    error BOARD_OSC32_STARTUP_US is out of range
#  endif

#  ifdef BOARD_OSC32_IS_XTAL
#    define OSC32_MODE_VALUE      BSCIF_OSCCTRL32_MODE_XTAL
#  else
#    define OSC32_MODE_VALUE      BSCIF_OSCCTRL32_MODE_EXTCLK
#  endif

#  ifndef BOARD_OSC32_SELCURR
#    define BOARD_OSC32_SELCURR   BSCIF_OSCCTRL32_SELCURR_300
#  endif
#endif

/* RC80M.  This might be the system clock or the source clock for the DFPLL.
 *
 * By selecting CONFIG_SAM_RC80M, you can also force the clock to be enabled
 * at boot time.
 */

#if defined(CONFIG_SAM_RC80M) || defined(BOARD_SYSCLK_SOURCE_RC80M) || \
    defined(BOARD_DFLL0_SOURCE_RC80M)
#  define NEED_RC80M              1
#endif

/* RCFAST.  The 12/8/4 fast RC oscillator may be used as the system clock.
 * If not then, it may be enabled by setting the CONFIG_SAM_RCFASTxM
 * configuration variable.
 */

#if defined(CONFIG_SAM_RCFAST12M)
#  undef CONFIG_SAM_RCFAST8M
#  undef CONFIG_SAM_RCFAST4M
#elif defined(CONFIG_SAM_RCFAST8M)
#  undef CONFIG_SAM_RCFAST4M
#endif

#if defined(BOARD_SYSCLK_SOURCE_FCFAST12M)
#  if defined(CONFIG_SAM_RCFAST8M) || defined(CONFIG_SAM_RCFAST4M)
#    error BOARD_SYSCLK_SOURCE_FCFAST12M inconsistent with CONFIG_SAM_RCFAST8/4M
#  endif
#  define NEED_RCFAST
#  define RCFAST_RANGE SCIF_RCFASTCFG_FRANGE_12MHZ
#elif defined(BOARD_SYSCLK_SOURCE_FCFAST8M)
#  if defined(CONFIG_SAM_RCFAST12M) || defined(CONFIG_SAM_RCFAST4M)
#    error BOARD_SYSCLK_SOURCE_FCFAST8M inconsistent with CONFIG_SAM_RCFAST12/4M
#  endif
#  define NEED_RCFAST
#  define RCFAST_RANGE SCIF_RCFASTCFG_FRANGE_8MHZ
#elif defined(BOARD_SYSCLK_SOURCE_FCFAST4M)
#  if defined(CONFIG_SAM_RCFAST12M) || defined(CONFIG_SAM_RCFAST8M)
#    error BOARD_SYSCLK_SOURCE_FCFAST4M inconsistent with CONFIG_SAM_RCFAST12/8M
#  endif
#  define NEED_RCFAST
#  define RCFAST_RANGE SCIF_RCFASTCFG_FRANGE_4MHZ
#elif defined(CONFIG_SAM_RCFAST12M)
#  define NEED_RCFAST
#  define RCFAST_RANGE SCIF_RCFASTCFG_FRANGE_12MHZ
#elif defined(CONFIG_SAM_RCFAST8M)
#  define NEED_RCFAST
#  define RCFAST_RANGE SCIF_RCFASTCFG_FRANGE_8MHZ
#elif defined(CONFIG_SAM_RCFAST4M)
#  define NEED_RCFAST
#  define RCFAST_RANGE SCIF_RCFASTCFG_FRANGE_4MHZ
#endif

/* RC1M.  The 1M RC oscillator may be used as the system block.
 *
 * By selecting CONFIG_SAM_RC1M, you can also force the clock to be
 * enabled at boot time.
 */

#if defined(CONFIG_SAM_RC1M) || defined(BOARD_SYSCLK_SOURCE_RC1M)
#  define NEED_RC1M  1
#endif

/* RC32K.  The 32KHz RC oscillator may be used as the input to DFLL0.
 *
 * By selecting CONFIG_SAM_RC32K, you can also force the clock to be
 * enabled at boot time.
 */

#if defined(CONFIG_SAM_RC32K) || defined(BOARD_DFLL0_SOURCE_RC32K)
#  define NEED_RC32K  1
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
 * Name: sam_picocache
 *
 * Description:
 *   Initialiaze the PICOCACHE.
 *
 ****************************************************************************/

#ifdef CONFIG_SAM_PICOCACHE
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
 *   Initialiaze OSC0 settings per the definitions in the board.h file.
 *
 ****************************************************************************/

#ifdef NEED_OSC0
static inline void sam_enableosc0(void)
{
  irqstate_t flags;
  uint32_t regval;

  regval = OSC0_STARTUP_VALUE | OSC0_GAIN_VALUE | OSC0_MODE_VALUE |
           SCIF_OSCCTRL0_OSCEN;

  /* The following two statements must be atomic */

  flags = irqsave();
  putreg32(SCIF_UNLOCK_KEY(0xaa) | SCIF_UNLOCK_ADDR(SAM_SCIF_OSCCTRL0_OFFSET),
           SAM_SCIF_UNLOCK);
  putreg32(regval, SAM_SCIF_OSCCTRL0);
  irqrestore(flags);

  /* Wait for OSC0 to be ready */

  while (getreg32(SAM_SCIF_PCLKSR) & SCIF_INT_OSC0RDY) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enableosc32
 *
 * Description:
 *   Initialiaze the 32KHz oscillator per settings in the board.h header
 *   file.
 *
 ****************************************************************************/

#ifdef NEED_OSC32K
static inline void sam_enableosc32(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Set up the OSCCTRL32 register using settings from the board.h file.
   * Also  enable the oscillator and provide bother the 32KHz and 1KHz output.
   */

  regval = OSC32_STARTUP_VALUE | BOARD_OSC32_SELCURR | OSC32_MODE_VALUE |
           BSCIF_OSCCTRL32_EN1K |  BSCIF_OSCCTRL32_EN32K |
           BSCIF_OSCCTRL32_OSC32EN;

  /* The following two statements must be atomic */

  flags = irqsave();
  putreg32(BSCIF_UNLOCK_KEY(0xaa) | BSCIF_UNLOCK_ADDR(SAM_BSCIF_OSCCTRL32_OFFSET),
           SAM_BSCIF_UNLOCK);
  putreg32(regval, SAM_BSCIF_OSCCTRL32);
  irqrestore(flags);

  /* Wait for OSC32 to be ready */

  while (getreg32(SAM_BSCIF_PCLKSR) & BSCIF_INT_OSC32RDY) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enablerc80m
 *
 * Description:
 *   Initialiaze the 80 MHz RC oscillator per settings in the board.h header
 *   file.
 *
 ****************************************************************************/

#ifdef NEED_RC80M
static inline void sam_enablerc80m(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* The following two statements must be atomic */

  flags = irqsave();
  regval = getreg32(SAM_SCIF_RC80MCR);
  putreg32(SCIF_UNLOCK_KEY(0xaa) | SCIF_UNLOCK_ADDR(SAM_SCIF_RC80MCR_OFFSET),
           SAM_SCIF_UNLOCK);
  putreg32(regval | SCIF_RC80MCR_EN, SAM_SCIF_RC80MCR);
  irqrestore(flags);

  /* Wait for OSC32 to be ready */

  while (getreg32(SAM_SCIF_RC80MCR) & SCIF_RC80MCR_EN) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enablerc80m
 *
 * Description:
 *   Initialiaze the 12/8/4 RC fast oscillator per settings in the board.h
 *   header file.
 *
 ****************************************************************************/

#ifdef NEED_RCFAST
static inline void sam_enablercfast(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* The following  must be atomic */

  flags   = irqsave();
  regval  = getreg32(SAM_SCIF_RCFASTCFG);
  regval &= ~SCIF_RCFASTCFG_FRANGE_MASK;
  regval |= (RCFAST_RANGE | SCIF_RCFASTCFG_EN);

  putreg32(SCIF_UNLOCK_KEY(0xaa) | SCIF_UNLOCK_ADDR(SAM_SCIF_RCFASTCFG_OFFSET),
           SAM_SCIF_UNLOCK);
  putreg32(regval, SAM_SCIF_RCFASTCFG);
  irqrestore(flags);

  /* Wait for RCFAST to be ready */

  while (getreg32(SAM_SCIF_RCFASTCFG) & SCIF_RCFASTCFG_EN) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enablerc1m
 *
 * Description:
 *   Initialiaze the 1M RC oscillator per settings in the board.h header
 *   file.
 *
 ****************************************************************************/

#ifdef NEED_RC1M
static inline void sam_enablerc1m(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* The following  must be atomic */

  flags   = irqsave();
  regval  = getreg32(SAM_BSCIF_RC1MCR);
  regval &= ~BSCIF_RCFASTCFG_FRANGE_MASK;
  regval |= (RCFAST_RANGE | BSCIF_RCFASTCFG_EN);

  putreg32(BSCIF_UNLOCK_KEY(0xaa) | BSCIF_UNLOCK_ADDR(SAM_BSCIF_RC1MCR_OFFSET),
           SAM_BSCIF_UNLOCK);
  putreg32(regval  | BSCIF_RC1MCR_CLKOEN, SAM_BSCIF_RC1MCR);
  irqrestore(flags);

  /* Wait for RCFAST to be ready */

  while (getreg32(SAM_BSCIF_RC1MCR) & BSCIF_RC1MCR_CLKOEN) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enablerc32k
 *
 * Description:
 *   Initialiaze the 23KHz RC oscillator per settings in the board.h header
 *   file.
 *
 ****************************************************************************/

#ifdef NEED_RC32K
void sam_enablerc32k(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* The following  must be atomic */

  flags   = irqsave();
  regval  = getreg32(SAM_BSCIF_RC32KCR);
  putreg32(BSCIF_UNLOCK_KEY(0xaa) | BSCIF_UNLOCK_ADDR(SAM_BSCIF_RC32KCR_OFFSET),
           SAM_BSCIF_UNLOCK);
  putreg32(regval | BSCIF_RC32KCR_EN32K | BSCIF_RC32KCR_EN, SAM_BSCIF_RC32KCR);
  irqrestore(flags);

  /* Wait for RCFAST to be ready */

  while (getreg32(SAM_BSCIF_RC32KCR) & BSCIF_RC32KCR_EN) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enablepll0
 *
 * Description:
 *   Initialiaze PLL0 settings per the definitions in the board.h file.
 *
 ****************************************************************************/

#ifdef SAM_CLOCK_PLL0
static inline void sam_enablepll0(void)
{
  /* Setup PLL0 */

  regval = (SAM_PLL0_DIV << PM_PLL_PLLDIV_SHIFT) | (SAM_PLL0_MUL << PM_PLL_PLLMUL_SHIFT) | (16 << PM_PLL_PLLCOUNT_SHIFT)

  /* Select PLL0/1 oscillator */

#if SAM_CLOCK_PLL_OSC1
  regval |= PM_PLL_PLLOSC;
#endif

  putreg32(regval, SAM_PM_PLL0);

  /* Set PLL0 options */

  regval = getreg32(SAM_PM_PLL0);
  regval &= ~PM_PLL_PLLOPT_MASK
#if SAM_PLL0_FREQ < 160000000
  regval |= PM_PLL_PLLOPT_VCO;
#endif
#if SAM_PLL0_DIV2 != 0
  regval |= PM_PLL_PLLOPT_XTRADIV;
#endif
#if SAM_PLL0_WBWM != 0
  regval |= PM_PLL_PLLOPT_WBWDIS;
#endif
  putreg32(regval, SAM_PM_PLL0)

  /* Enable PLL0 */

  regval = getreg32(SAM_PM_PLL0);
  regval |= PM_PLL_PLLEN;
  putreg32(regval, SAM_PM_PLL0)

  /* Wait for PLL0 locked. */

  while ((getreg32(SAM_PM_POSCSR) & PM_POSCSR_LOCK0) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enablepll1
 *
 * Description:
 *   Initialiaze PLL1 settings per the definitions in the board.h file.
 *
 ****************************************************************************/

#ifdef SAM_CLOCK_PLL1
static inline void sam_enablepll1(void)
{
  /* Setup PLL1 */

  regval = (SAM_PLL1_DIV << PM_PLL_PLLDIV_SHIFT) | (SAM_PLL1_MUL << PM_PLL_PLLMUL_SHIFT) | (16 << PM_PLL_PLLCOUNT_SHIFT)

  /* Select PLL0/1 oscillator */

#if SAM_CLOCK_PLL_OSC1
  regval |= PM_PLL_PLLOSC;
#endif

  putreg32(regval, SAM_PM_PLL1);

  /* Set PLL1 options */

  regval = getreg32(SAM_PM_PLL1);
  regval &= ~PM_PLL_PLLOPT_MASK
#if SAM_PLL1_FREQ < 160000000
  regval |= PM_PLL_PLLOPT_VCO;
#endif
#if SAM_PLL1_DIV2 != 0
  regval |= PM_PLL_PLLOPT_XTRADIV;
#endif
#if SAM_PLL1_WBWM != 0
  regval |= PM_PLL_PLLOPT_WBWDIS;
#endif
  putreg32(regval, SAM_PM_PLL1)

  /* Enable PLL1 */

  regval = getreg32(SAM_PM_PLL1);
  regval |= PM_PLL_PLLEN;
  putreg32(regval, SAM_PM_PLL1)

  /* Wait for PLL1 locked. */

  while ((getreg32(SAM_PM_POSCSR) & PM_POSCSR_LOCK1) == 0);
}
#endif

/****************************************************************************
 * Name: sam_setdividers
 *
 * Description:
 *   Configure derived clocks.
 *
 ****************************************************************************/

static inline void sam_setdividers(uint32_t cpudiv, uint32_t pbadiv,
                                   uint32_t pbbdiv, uint32_t pbcdiv,
                                   uint32_t pbddiv)
{
  irqstate_t flags;
  uint32_t cpusel = 0;
  uint32_t pbasel = 0;
  uint32_t pbbsel = 0;
  uint32_t pbcsel = 0;
  uint32_t pbdsel = 0;

  /* Get the register setting for each divider value */

  if (cpudiv > 0)
    {
      cpusel = (PM_CPUSEL(cpudiv - 1)) | PM_CPUSEL_DIV;
    }

  if (pbadiv > 0)
    {
      pbasel = (PM_PBSEL(pbadiv - 1)) | PM_PBSEL_DIV;
    }

  if (pbbdiv > 0)
    {
      pbbsel = (PM_PBSEL(pbbdiv - 1)) | PM_PBSEL_DIV;
    }

  if (pbcdiv > 0)
    {
      pbcsel = (PM_PBSEL(pbcdiv - 1)) | PM_PBSEL_DIV;
    }

  if (pbddiv > 0)
    {
      pbdsel = (PM_PBSEL(pbddiv - 1)) | PM_PBSEL_DIV;
    }

  /* Then set the divider values. The following operations need to be atomic
   * for the unlock-write sequeuences.
   */

  flags = irqsave();

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

  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_fws
 *
 * Description:
 *   Setup FLASH wait states.
 *
 ****************************************************************************/

static void sam_fws(uint32_t cpuclock)
{
  uint32_t regval;

  regval = getreg32(SAM_FLASHCALW_FCR);
  if (cpuclock > SAM_FLASHCALW_FWS0_MAXFREQ)
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
  putreg32(regval, SAM_PM_MCCTRL);
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
  uint32_t regval;
  uint32_t bpmps;
  bool fastwkup;

  /* Enable clocking to the PICOCACHE */

  sam_picocache();

  /* Configure dividers for derived clocks.  These divider definitions must
   * be provided in the board.h header file.
   */

  sam_setdividers(BOARD_CPU_SHIFT, BOARD_PBA_SHIFT, BOARD_PBB_SHIFT,
                  BOARD_PBC_SHIFT, BOARD_PBD_SHIFT);


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

#ifdef CONFIG_SAM_FLASH_HSEN
  /* The high speed FLASH mode has been enabled.  Select power scaling mode 2 */

  bpmps    = BPM_PMCON_PS2;
  fastwkup = false;
#else
  /* Not high speed mode.  Check if we can go to power scaling mode 1. */

  if (BOARD_CPU_FREQUENCY <= FLASH_MAXFREQ_PS1_HSDIS_FWS1)
    {
      /* Yes.. Do we also need to enable fast wakeup? */

      bpmps = BPM_PMCON_PS1;
      if (BOARD_CPU_FREQUENCY > FLASH_MAXFREQ_PS1_HSDIS_FWS0)
        {
          /* Yes.. enable fast wakeup */

          regval  = getreg32(SAM_BPM_PMCON);
          regval |= BPM_PMCON_FASTWKUP;
          putreg32(BPM_UNLOCK_KEY(0xaa) | BPM_UNLOCK_ADDR(SAM_BPM_PMCON_OFFSET), SAM_BPM_UNLOCK);
          putreg32(regval, SAM_BPM_PMCON);

          /* We need to remember that we did this */

          fastwkup = true;
        }
    }
  else
    {
      bpmps = BPM_PMCON_PS0;
    }
#endif

  /* Enable clock sources:
   *
   * OSC0:  Might by the system clock or the source clock for PLL0 or DFLL0
   * OSC32: Might be source clock for DFLL0
   */

#if NEED_OSC0
  /* Enable OSC0 using the settings in board.h */

  sam_enableosc0();
#endif

#ifdef NEED_OSC32K
  /* Enable the 32KHz oscillator using the settings in board.h */

  sam_enableosc32();
#endif

#ifdef NEED_OSC32K
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

  /* Enable the PLL or FDLL */
#warning Missing Logic

  /* Switch to the system clock selected by the settings in the board.h
   * header file.
   */

#if defined(BOARD_SYSCLK_SOURCE_RCSYS)
  /* Since this function only executes at power up, we know that we are
   * already running from RCSYS.
   */
#endif

#ifdef NEED_OSC0
  /* Set up FLASH wait states */

  sam_fws(SAM_FOSC0);

  /* Then switch the main clock to OSC0 */

  sam_mainclk(PM_MCCTRL_MCSEL_OSC0);
#endif

#ifdef SAM_CLOCK_PLL0
  /* Enable PLL0 using the settings in board.h */

  sam_enablepll0();

  /* Set up FLASH wait states */

  sam_fws(SAM_CPU_CLOCK);

  /* Then switch the main clock to PLL0 */

  sam_mainclk(PM_MCCTRL_MCSEL_PLL0);
#endif

#ifdef SAM_CLOCK_PLL1
  /* Enable PLL1 using the settings in board.h */

  sam_enablepll1();
#endif

  /* Set up the USBB GCLK */

#ifdef CONFIG_USBDEV
  void sam_usbclock();
#endif
}
