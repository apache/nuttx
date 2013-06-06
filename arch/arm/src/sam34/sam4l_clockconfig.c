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
#include "chip/sam4l_flashcalw.h"

#include "sam_clockconfig.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#if defined(SAM_CLOCK_OSC0) || \
    (defined (SAM_CLOCK_PLL0) && defined(SAM_CLOCK_PLL0_OSC0)) || \
    (defined (SAM_CLOCK_PLL1) && defined(SAM_CLOCK_PLL1_OSC0))
#  define NEED_OSC0
#endif

#if defined(SAM_CLOCK_OSC1) || \
    (defined (SAM_CLOCK_PLL0) && defined(SAM_CLOCK_PLL0_OSC1)) || \
    (defined (SAM_CLOCK_PLL1) && defined(SAM_CLOCK_PLL1_OSC1))
#  define NEED_OSC1
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
 * Name: sam_enableosc32
 *
 * Description:
 *   Initialiaze the 32KHz oscillator.  This oscillator is used by the RTC
 *   logic to provide the sysem timer.
 *
 ****************************************************************************/

#ifdef SAM_CLOCK_OSC32
static inline void sam_enableosc32(void)
{
  uint32_t regval;

  /* Select the 32KHz oscillator crystal */

  regval = getreg32(SAM_PM_OSCCTRL32);
  regval &= ~PM_OSCCTRL32_MODE_MASK;
  regval |= PM_OSCCTRL32_MODE_XTAL;
  putreg32(regval, SAM_PM_OSCCTRL32);

  /* Enable the 32-kHz clock */

  regval = getreg32(SAM_PM_OSCCTRL32);
  regval &= ~PM_OSCCTRL32_STARTUP_MASK;
  regval |= PM_OSCCTRL32_EN|(SAM_OSC32STARTUP << PM_OSCCTRL32_STARTUP_SHIFT);
  putreg32(regval, SAM_PM_OSCCTRL32);
}
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
  uint32_t regval;

  /* Enable OSC0 in the correct crystal mode by setting the mode value in OSCCTRL0 */

  regval  = getreg32(SAM_PM_OSCCTRL0);
  regval &= ~PM_OSCCTRL_MODE_MASK;
#if SAM_FOSC0 < 900000
  regval |= PM_OSCCTRL_MODE_XTALp9;  /* Crystal XIN 0.4-0.9MHz */
#elif SAM_FOSC0 < 3000000
  regval |= PM_OSCCTRL_MODE_XTAL3;   /* Crystal XIN 0.9-3.0MHz */
#elif SAM_FOSC0 < 8000000
  regval |= PM_OSCCTRL_MODE_XTAL8;   /* Crystal XIN 3.0-8.0MHz */
#else
  regval |= PM_OSCCTRL_MODE_XTALHI;  /* Crystal XIN above 8.0MHz */
#endif
  putreg32(regval, SAM_PM_OSCCTRL0);

  /* Enable OSC0 using the startup time provided in board.h.  This startup time
   * is critical and depends on the characteristics of the crystal.
   */

  regval  = getreg32(SAM_PM_OSCCTRL0);
  regval &= ~PM_OSCCTRL_STARTUP_MASK;
  regval |= (SAM_OSC0STARTUP << PM_OSCCTRL_STARTUP_SHIFT);
  putreg32(regval, SAM_PM_OSCCTRL0);

  /* Enable OSC0 */

  regval = getreg32(SAM_PM_MCCTRL);
  regval |= PM_MCCTRL_OSC0EN;
  putreg32(regval, SAM_PM_MCCTRL);

  /* Wait for OSC0 to be ready */

  while ((getreg32(SAM_PM_POSCSR) & PM_POSCSR_OSC0RDY) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enableosc1
 *
 * Description:
 *   Initialiaze OSC0 settings per the definitions in the board.h file.
 *
 ****************************************************************************/

#ifdef NEED_OSC1
static inline void sam_enableosc1(void)
{
  uint32_t regval;

  /* Enable OSC1 in the correct crystal mode by setting the mode value in OSCCTRL1 */

  regval  = getreg32(SAM_PM_OSCCTRL1);
  regval &= ~PM_OSCCTRL_MODE_MASK;
#if SAM_FOSC1 < 900000
  regval |= PM_OSCCTRL_MODE_XTALp9;  /* Crystal XIN 0.4-0.9MHz */
#elif SAM_FOSC1 < 3000000
  regval |= PM_OSCCTRL_MODE_XTAL3;   /* Crystal XIN 0.9-3.0MHz */
#elif SAM_FOSC1 < 8000000
  regval |= PM_OSCCTRL_MODE_XTAL8;   /* Crystal XIN 3.0-8.0MHz */
#else
  regval |= PM_OSCCTRL_MODE_XTALHI;  /* Crystal XIN above 8.0MHz */
#endif
  putreg32(regval, SAM_PM_OSCCTRL1);

  /* Enable OSC1 using the startup time provided in board.h.  This startup time
   * is critical and depends on the characteristics of the crystal.
   */

  regval  = getreg32(SAM_PM_OSCCTRL1);
  regval &= ~PM_OSCCTRL_STARTUP_MASK;
  regval |= (SAM_OSC1STARTUP << PM_OSCCTRL_STARTUP_SHIFT);
  putreg32(regval, SAM_PM_OSCCTRL1);

  /* Enable OSC1 */

  regval = getreg32(SAM_PM_MCCTRL);
  regval |= PM_MCCTRL_OSC1EN;
  putreg32(regval, SAM_PM_MCCTRL);

  /* Wait for OSC1 to be ready */

  while ((getreg32(SAM_PM_POSCSR) & PM_POSCSR_OSC1RDY) == 0);
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
  /* Enable clocking to the PICOCACHE */

  sam_picocache();

  /* Configure dividers derived clocks.  These divider definitions must be
   * provided in the board.h header file.
   */

  sam_setdividers(BOARD_SYSCLK_CPU_DIV, BOARD_SYSCLK_PBA_DIV,
                  BOARD_SYSCLK_PBB_DIV, BOARD_SYSCLK_PBC_DIV,
                  BOARD_SYSCLK_PBD_DIV);

#ifdef SAM_CLOCK_OSC32
  /* Enable the 32KHz oscillator (need by the RTC module) */

  sam_enableosc32();
#endif

#ifdef NEED_OSC0
  /* Enable OSC0 using the settings in board.h */

  sam_enableosc0();

  /* Set up FLASH wait states */

  sam_fws(SAM_FOSC0);

  /* Then switch the main clock to OSC0 */

  sam_mainclk(PM_MCCTRL_MCSEL_OSC0);
#endif

#ifdef NEED_OSC1
  /* Enable OSC1 using the settings in board.h */

  sam_enableosc1();
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
