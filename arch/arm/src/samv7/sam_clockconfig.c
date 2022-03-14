/****************************************************************************
 * arch/arm/src/samv7/sam_clockconfig.c
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_clockconfig.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_eefc.h"
#include "hardware/sam_wdt.h"
#include "hardware/sam_supc.h"
#include "hardware/sam_matrix.h"
#include "hardware/sam_utmi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Board Settings ***********************************************************/

/* PMC register settings based on the board configuration values defined
 * in board.h
 */

#define BOARD_CKGR_MOR      (BOARD_CKGR_MOR_MOSCXTENBY | PMC_CKGR_MOR_MOSCRCEN | \
                             BOARD_CKGR_MOR_MOSCXTST | PMC_CKGR_MOR_KEY)
#define BOARD_CKGR_PLLAR    (BOARD_CKGR_PLLAR_DIV | BOARD_CKGR_PLLAR_COUNT | \
                             BOARD_CKGR_PLLAR_MUL | PMC_CKGR_PLLAR_ONE)

#define BOARD_PMC_MCKR_FAST (PMC_MCKR_CSS_MAIN | BOARD_PMC_MCKR_PRES | \
                             BOARD_PMC_MCKR_MDIV | BOARD_PMC_MCKR_UPLLDIV2)
#define BOARD_PMC_MCKR      (BOARD_PMC_MCKR_CSS | BOARD_PMC_MCKR_PRES | \
                             BOARD_PMC_MCKR_MDIV | BOARD_PMC_MCKR_UPLLDIV2)

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
 * Name: sam_efcsetup
 *
 * Description:
 *   Configure wait states for embedded flash access
 *
 ****************************************************************************/

static inline void sam_efcsetup(void)
{
  uint32_t regval = EEFC_FMR_FWS(BOARD_FWS) | EEFC_FMR_CLOE;
  putreg32(regval, SAM_EEFC_FMR);
}

/****************************************************************************
 * Name: sam_wdtsetup
 *
 * Description:
 *   Disable the watchdog timer
 *
 ****************************************************************************/

static inline void sam_wdtsetup(void)
{
#if !defined(CONFIG_SAMV7_WDT) || \
    (defined(CONFIG_WDT_ENABLED_ON_RESET) && defined(CONFIG_WDT_DISABLE_ON_RESET))
  putreg32(WDT_MR_WDDIS, SAM_WDT_MR);
#endif

#if !defined(CONFIG_SAMV7_RSWDT) || \
    (defined(CONFIG_RSWDT_ENABLED_ON_RESET) && defined(CONFIG_RSWDT_DISABLE_ON_RESET))
  putreg32(WDT_MR_WDDIS, SAM_RSWDT_MR);
#endif
}

/****************************************************************************
 * Name: sam_supcsetup
 *
 * Description:
 *   Select the external slow clock
 *
 ****************************************************************************/

static inline void sam_supcsetup(void)
{
#ifdef BOARD_HAVE_SLOWXTAL
  /* Check if the 32-kHz is already selected.  The slow clock defaults to
   * the RC oscillator, but the software can enable the crystal oscillator
   * and select it as the slow clock source.
   */

  if ((getreg32(SAM_SUPC_SR) & SUPC_SR_OSCSEL) == 0)
    {
      volatile uint32_t delay;

      putreg32((SUPC_CR_XTALSEL | SUPR_CR_KEY), SAM_SUPC_CR);
      for (delay = 0;
           (getreg32(SAM_SUPC_SR) & SUPC_SR_OSCSEL) == 0 &&
            delay < UINT32_MAX;
           delay++);
    }
#endif
}

/****************************************************************************
 * Name: sam_pmcwait
 *
 * Description:
 *   Wait for the specified PMC status bit to become "1"
 *
 ****************************************************************************/

static void sam_pmcwait(uint32_t bit)
{
  volatile uint32_t delay;

  for (delay = 0;
       (getreg32(SAM_PMC_SR) & bit) == 0 && delay < UINT32_MAX;
       delay++);
}

/****************************************************************************
 * Name: sam_pmcsetup
 *
 * Description:
 *   Initialize clocking
 *
 ****************************************************************************/

static inline void sam_pmcsetup(void)
{
  uint32_t regval;

  /* Enable main oscillator (if it has not already been selected) */

  if ((getreg32(SAM_PMC_CKGR_MOR) & PMC_CKGR_MOR_MOSCSEL) == 0)
    {
      /* "When the MOSCXTEN bit and the MOSCXTCNT are written in CKGR_MOR to
       *  enable the main oscillator, the MOSCXTS bit in the Power Management
       *  Controller Status Register (PMC_SR) is cleared and the counter
       *  starts counting down on the slow clock divided by 8 from the
       *  MOSCXTCNT value. ... When the counter reaches 0, the MOSCXTS bit
       *  is set, indicating that the main clock is valid."
       */

      putreg32(BOARD_CKGR_MOR, SAM_PMC_CKGR_MOR);
      sam_pmcwait(PMC_INT_MOSCXTS);
    }

  /* "Switch to the main oscillator.  The selection is made by writing the
   *  MOSCSEL bit in the Main Oscillator Register (CKGR_MOR). The switch of
   *  the Main Clock source is glitch free, so there is no need to run out
   *  of SLCK, PLLACK or UPLLCK in order to change the selection. The
   *  MOSCSELS bit of the power Management Controller Status Register
   *  (PMC_SR) allows knowing when the switch sequence is done."
   *
   *   MOSCSELS: Main Oscillator Selection Status
   *             0 = Selection is done
   *             1 = Selection is in progress
   */

  putreg32((BOARD_CKGR_MOR | PMC_CKGR_MOR_MOSCSEL), SAM_PMC_CKGR_MOR);
  sam_pmcwait(PMC_INT_MOSCSELS);

  /* "Select the master clock. "The Master Clock selection is made by writing
   *  the CSS field (Clock Source Selection) in PMC_MCKR (Master Clock
   *  Register). The prescaler supports the division by a power of 2 of the
   *  selected clock between 1 and 64, and the division by 3. The PRES field
   *  in PMC_MCKR programs the prescaler. Each time PMC_MCKR is written to
   *  define a new Master Clock, the MCKRDY bit is cleared in PMC_SR.
   *  It reads 0 until the Master Clock is established.
   */

  regval  = getreg32(SAM_PMC_MCKR);
  regval &= ~PMC_MCKR_CSS_MASK;
  regval |= PMC_MCKR_CSS_MAIN;
  putreg32(regval, SAM_PMC_MCKR);
  sam_pmcwait(PMC_INT_MCKRDY);

  /* Setup PLLA and wait for LOCKA */

  putreg32(BOARD_CKGR_PLLAR, SAM_PMC_CKGR_PLLAR);
  sam_pmcwait(PMC_INT_LOCKA);

#ifdef CONFIG_SAMV7_USBDEVHS
  /* UTMI configuration:
   * Enable port0, select 12/16 MHz MAINOSC crystal source
   */

#if BOARD_MAINOSC_FREQUENCY == 12000000
  putreg32(UTMI_CKTRIM_FREQ_XTAL12, SAM_UTMI_CKTRIM);
#elif BOARD_MAINOSC_FREQUENCY == 16000000
  putreg32(UTMI_CKTRIM_FREQ_XTAL16, SAM_UTMI_CKTRIM);
#else
#  error ERROR: Unrecognized MAINSOSC frequency
#endif

#ifdef CONFIG_SAMV7_USBDEVHS_LOWPOWER
  /* Enable UTMI Clocking. The USBHS can work in two modes:
   *
   * - Normal mode where High speed, Full speed and Low speed are available.
   * - Low-power mode where only Full speed and Low speed are available.
   *
   * Only the Low-power mode is mode is supported by the logic here.  Normal
   * mode logic is handled in the function sam_usbclock().
   */

  /* UTMI Low-power mode, Full/Low Speed mode
   *
   * Enable the 48MHz FS Clock.
   */

  putreg32(PMC_USBCLK, SAM_PMC_SCER);

  /* Select the UTMI PLL as the USB PLL clock input (480MHz) with divider
   * to get to 48MHz.  UPLL output frequency is determined only by the
   * 12/16MHz crystal selection above.
   */

  regval = PMC_USB_USBS_UPLL;

  if ((getreg32(SAM_PMC_MCKR) & PMC_MCKR_PLLADIV2) != 0)
    {
      /* Divider = 480 MHz / 2 / 48 MHz = 5 */

      regval |=  PMC_USB_USBDIV(4);
    }
  else
    {
      /* Divider = 480 MHz / 1 / 48 MHz = 10 */

      regval |=  PMC_USB_USBDIV(9);
    }

  putreg32(regval, SAM_PMC_USB);

  /* Enable the UTMI PLL with the maximum start-up time */

  regval = PMC_CKGR_UCKR_UPLLEN | PMC_CKGR_UCKR_UPLLCOUNT_MAX;
  putreg32(regval, SAM_PMC_CKGR_UCKR);

  /* Wait for LOCKU */

  sam_pmcwait(PMC_INT_LOCKU);

#endif /* CONFIG_SAMV7_USBDEVHS_LOWPOWER */
#endif /* CONFIG_SAMV7_USBDEVHS */

  /* Switch to the fast clock and wait for MCKRDY */

  putreg32(BOARD_PMC_MCKR_FAST, SAM_PMC_MCKR);
  sam_pmcwait(PMC_INT_MCKRDY);

  putreg32(BOARD_PMC_MCKR, SAM_PMC_MCKR);
  sam_pmcwait(PMC_INT_MCKRDY);
}

/****************************************************************************
 * Name: sam_enabledefaultmaster and sam_disabledefaultmaster
 *
 * Description:
 *   Enable/disable default master access
 *
 ****************************************************************************/

static inline void sam_enabledefaultmaster(void)
{
#warning REVISIT
#if 0 /* REVISIT -- this is stuff left over from SAM3/4 */
  uint32_t regval;

  /* Set default master: SRAM0 -> Cortex-M7 System */

  regval  = getreg32(SAM_MATRIX_SCFG0);
  regval |= (MATRIX_SCFG0_FIXEDDEFMSTR_ARMS | MATRIX_SCFG_DEFMSTRTYPE_FIXED);
  putreg32(regval, SAM_MATRIX_SCFG0);

  /* Set default master: SRAM1 -> Cortex-M7 System */

  regval  = getreg32(SAM_MATRIX_SCFG1);
  regval |= (MATRIX_SCFG1_FIXEDDEFMSTR_ARMS | MATRIX_SCFG_DEFMSTRTYPE_FIXED);
  putreg32(regval, SAM_MATRIX_SCFG1);

  /* Set default master: Internal flash0 -> Cortex-M7 Instruction/Data */

  regval  = getreg32(SAM_MATRIX_SCFG3);
  regval |= (MATRIX_SCFG3_FIXEDDEFMSTR_ARMC | MATRIX_SCFG_DEFMSTRTYPE_FIXED);
  putreg32(regval, SAM_MATRIX_SCFG3);
#endif
}

#if 0 /* Not used -- this is stuff left over from SAM3/4 */
static inline void sam_disabledefaultmaster(void)
{
  uint32_t regval;

  /* Clear default master: SRAM0 -> Cortex-M7 System */

  regval  = getreg32(SAM_MATRIX_SCFG0);
  regval &= ~MATRIX_SCFG_DEFMSTRTYPE_MASK;
  putreg32(regval, SAM_MATRIX_SCFG0);

  /* Clear default master: SRAM1 -> Cortex-M7 System */

  regval  = getreg32(SAM_MATRIX_SCFG1);
  regval &= ~MATRIX_SCFG_DEFMSTRTYPE_MASK;
  putreg32(regval, SAM_MATRIX_SCFG1);

  /* Clear default master: Internal flash0 -> Cortex-M7 Instruction/Data */

  regval  = getreg32(SAM_MATRIX_SCFG3);
  regval &= ~MATRIX_SCFG_DEFMSTRTYPE_MASK;
  putreg32(regval, SAM_MATRIX_SCFG3);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_clockconfig
 *
 * Description:
 *   Called to initialize the SAMV7.  This does whatever setup is needed
 *   to put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  (After power-on reset, the
 *   SAMV7 is initially running on a 4MHz internal RC clock).  This function
 *   also performs other low-level chip initialization of the chip including
 *   EFC, master clock, IRQ & watchdog configuration.
 *
 ****************************************************************************/

void sam_clockconfig(void)
{
  /* Configure embedded flash access */

  sam_efcsetup();

  /* Configure the watchdog timer */

  sam_wdtsetup();

  /* Setup the supply controller to use the external slow clock */

  sam_supcsetup();

  /* Initialize clocking */

  sam_pmcsetup();

  /* Optimize CPU setting for speed */

  sam_enabledefaultmaster();
}

/****************************************************************************
 * Name: sam_usbclock
 *
 * Description:
 *   Enable USBHS clocking.
 *
 ****************************************************************************/

#if defined(CONFIG_SAMV7_USBDEVHS) && !defined(CONFIG_SAMV7_USBDEVHS_LOWPOWER)
void sam_usbclock(void)
{
  uint32_t regval;

  /* Enable UTMI Clocking. The USBHS can work in two modes:
   *
   * - Normal mode where High speed, Full speed and Low speed are available.
   * - Low-power mode where only Full speed and Low speed are available.
   *
   * Only the normal mode is supported by this logic of this function.  Low-
   * power mode was handled in the sam_clockconfig().
   */

  /* UTMI normal mode, High/Full/Low Speed
   *
   * Disable the 48MHz USB FS Clock.  It is not used in this configuration
   */

  putreg32(PMC_USBCLK, SAM_PMC_SCDR);

  /* Select the UTMI PLL as the USB PLL clock input (480MHz) with a divider
   * of 1.  UPLL output frequency is determined only by the 12/16MHz crystal
   * selection set in sam_clockconfig().
   */

  putreg32(PMC_USB_USBS_UPLL, SAM_PMC_USB);

  /* Enable the UTMI PLL with the maximum start-up time */

  regval = PMC_CKGR_UCKR_UPLLEN | PMC_CKGR_UCKR_UPLLCOUNT_MAX;
  putreg32(regval, SAM_PMC_CKGR_UCKR);

  /* Wait for LOCKU */

  sam_pmcwait(PMC_INT_LOCKU);
}

#endif /* CONFIG_SAMV7_USBDEVHS && !CONFIG_SAMV7_USBDEVHS_LOWPOWER */
