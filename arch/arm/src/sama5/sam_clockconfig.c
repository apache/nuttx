/****************************************************************************
 * arch/arm/src/sama5/sam_clockconfig.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/sama5/chip.h>

#include "up_arch.h"
#include "up_internal.h"

#include "sam_periphclks.h"
#include "sam_clockconfig.h"
#include "chip/sam_pmc.h"
#include "chip/sam_sfr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Do we need to setup the POLL?  Yes if we are booting from ISRAM or NOR
 * FLASH on CS0.
 */

#undef NEED_PLLSETUP
#if defined(CONFIG_SAMA5_BOOT_ISRAM) || defined(CONFIG_SAMA5_BOOT_CS0FLASH)
#  define NEED_PLLSETUP 1
#endif

/* Do we need to configure the UPLL */

#if !defined(CONFIG_SAMA5_EHCI) && !defined(CONFIG_SAMA5_OHCI) && \
    !defined(CONFIG_SAMA5_UDPHS)

   /* No... ignore the board setup */

#  undef BOARD_USE_UPLL
#endif

/* Problems have been seen when reconfiguring the PLL while executing out
 * of NOR FLASH on CS0.  In that case, we required RAM function support.  The
 * critical functions will be copied from NOR into ISRAM for execution.  This
 * prevents any strange behavior from the NOR while we reconfigure the PLL.
 */

#if defined(CONFIG_SAMA5_BOOT_CS0FLASH) && !defined(CONFIG_ARCH_RAMFUNCS)
# error "CONFIG_ARCH_RAMFUNCS must be defined for this logic"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Types
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
 * Name: sam_pmcwait
 *
 * Description:
 *   Wait for the specide PMC status bit to become "1"
 *
 ****************************************************************************/

#if defined(NEED_PLLSETUP) || defined(BOARD_USE_UPLL)
static void __ramfunc__ sam_pmcwait(uint32_t bit)
{
  /* There is no timeout on this wait.  Why not?  Because the symptoms there
   * is no fallback if the wait times out and if the wait does time out, it
   * can be very difficult to determine what happened.  Much better to just
   * hang here.
   */

  while ((getreg32(SAM_PMC_SR) & bit) == 0);
}
#endif

/****************************************************************************
 * Name: sam_enablemosc
 *
 * Description:
 *   Enable the main oscillator
 *
 ****************************************************************************/

#if defined(NEED_PLLSETUP)
static void __ramfunc__ sam_enablemosc(void)
{
  uint32_t regval;

  /* Switch from the internal 12MHz RC to the main external oscillator */

  if ((getreg32(SAM_PMC_CKGR_MOR) & PMC_CKGR_MOR_MOSCSEL) == 0)
    {
      /* Enable main external oscillator */

      regval  = getreg32(SAM_PMC_CKGR_MOR);
      regval |= PMC_CKGR_MOR_MOSCXTEN | PMC_CKGR_MOR_KEY;
      putreg32(regval, SAM_PMC_CKGR_MOR);

      /* Wait for the main clock to become ready  */

      while ((getreg32(SAM_PMC_CKGR_MCFR) & PMC_CKGR_MCFR_MAINFRDY) == 0);

      /* Disable external OSC 12 MHz bypass */

      regval  = getreg32(SAM_PMC_CKGR_MOR);
      regval &= ~PMC_CKGR_MOR_MOSCXTBY;
      regval |= PMC_CKGR_MOR_KEY;
      putreg32(regval, SAM_PMC_CKGR_MOR);

      /* Switch main clock source to the external oscillator */

      regval  = getreg32(SAM_PMC_CKGR_MOR);
      regval |= (PMC_CKGR_MOR_MOSCSEL | PMC_CKGR_MOR_KEY);
      putreg32(regval, SAM_PMC_CKGR_MOR);

      /* Wait for the main clock status change for the external oscillator
       * selection.
       */

      sam_pmcwait(PMC_INT_MOSCSELS);

      /* And handle the case where MCK is running on main CLK */

      sam_pmcwait(PMC_INT_MCKRDY);
    }
}
#endif

/****************************************************************************
 * Name: sam_selectmosc
 *
 * Description:
 *   Select the main oscillator as the input clock for processor clock (PCK)
 *   and the main clock (MCK).  The PCK and MCK differ only by the MDIV
 *   divisor that permits the MCK to run at a lower rate.
 *
 ****************************************************************************/

#if defined(NEED_PLLSETUP)
static void __ramfunc__ sam_selectmosc(void)
{
  uint32_t regval;

  /* Select the main oscillator as the input clock for PCK and MCK */

  regval  = getreg32(SAM_PMC_MCKR);
  regval &= ~PMC_MCKR_CSS_MASK;
  regval |= PMC_MCKR_CSS_MAIN;
  putreg32(regval, SAM_PMC_MCKR);

  /* Wait for main clock to be ready */

  sam_pmcwait(PMC_INT_MCKRDY);
}
#endif

/****************************************************************************
 * Name: sam_pllasetup
 *
 * Description:
 *   Select the main oscillator as the input clock for processor clock (PCK)
 *   and the main clock (MCK).  The PCK and MCK differ only by the MDIV
 *   divisor that permits the MCK to run at a lower rate.
 *
 ****************************************************************************/

#if defined(NEED_PLLSETUP)
static void __ramfunc__ sam_pllasetup(void)
{
  uint32_t regval;

  /* Configure PLLA */

#ifdef SAMA5_HAVE_PLLAR_DIV
  regval = (BOARD_CKGR_PLLAR_DIV      | BOARD_CKGR_PLLAR_COUNT |
            BOARD_CKGR_PLLAR_OUT      | BOARD_CKGR_PLLAR_MUL   |
            PMC_CKGR_PLLAR_ONE);
#else
  regval = (PMC_CKGR_PLLAR_DIV_BYPASS | BOARD_CKGR_PLLAR_COUNT |
            BOARD_CKGR_PLLAR_OUT      | BOARD_CKGR_PLLAR_MUL   |
            PMC_CKGR_PLLAR_ONE);
#endif

  putreg32(regval, SAM_PMC_CKGR_PLLAR);

  /* Set the PLL Charge Pump Current Register to zero */

  putreg32(0, SAM_PMC_PLLICPR);

  /* And wait for the PLL to lock on */

  sam_pmcwait(PMC_INT_LOCKA);
}
#endif

/****************************************************************************
 * Name: sam_plladivider
 *
 * Description:
 *   Configure MCK PLLA divider
 *
 ****************************************************************************/

#if defined(NEED_PLLSETUP)
static void __ramfunc__ sam_plladivider(void)
{
  uint32_t regval;

  /* Is the PLLA divider currently set? */

  regval = getreg32(SAM_PMC_MCKR);
  if ((regval & PMC_MCKR_PLLADIV2) != 0)
    {
#if BOARD_PMC_MCKR_PLLADIV == 0
      /* The divider is set and we are configured to clear it */

      regval &= ~PMC_MCKR_PLLADIV2;
#else
      /* The divider is already set */

      return;
#endif
    }
  else
    {
#if BOARD_PMC_MCKR_PLLADIV == 0
      /* The divider is already cleared */

      return;
#else
      /* The divider is clear and we are configured to set it */

      regval |= PMC_MCKR_PLLADIV2;
#endif
    }

  /* Set/clear the divider */

  putreg32(regval, SAM_PMC_MCKR);

  /* We changed the PLLA divider.  Wait for the main clock to be ready again */

  sam_pmcwait(PMC_INT_MCKRDY);
}
#endif

/****************************************************************************
 * Name: sam_mckprescaler
 *
 * Description:
 *   Configure main clock (MCK) Prescaler
 *
 ****************************************************************************/

#if defined(NEED_PLLSETUP)
static void __ramfunc__ sam_mckprescaler(void)
{
  uint32_t regval;

  /* Set the main clock prescaler */

  regval  = getreg32(SAM_PMC_MCKR);
  regval &= ~PMC_MCKR_PRES_MASK;
  regval |= BOARD_PMC_MCKR_PRES;
  putreg32(regval, SAM_PMC_MCKR);

  /* Wait for the main clock to be ready again */

  sam_pmcwait(PMC_INT_MCKRDY);
}
#endif

/****************************************************************************
 * Name: sam_mckdivider
 *
 * Description:
 *   Configure main clock (MCK) divider (MDIV).  This divider allows the MCK
 *   to run at a lower rate then PCK.
 *
 ****************************************************************************/

#if defined(NEED_PLLSETUP)
static void __ramfunc__ sam_mckdivider(void)
{
  uint32_t regval;

  /* Set the main clock divider */

  regval  = getreg32(SAM_PMC_MCKR);
  regval &= ~PMC_MCKR_MDIV_MASK;
  regval |= BOARD_PMC_MCKR_MDIV;
  putreg32(regval, SAM_PMC_MCKR);

  /* Wait for the main clock to be ready again */

  sam_pmcwait(PMC_INT_MCKRDY);
}
#endif

/****************************************************************************
 * Name: sam_h32mxdivider
 *
 * Description:
 *   Set the H32MX divider.
 *
 *   0: The AHB 32-bit Matrix frequency is equal to the AHB 64-bit Matrix
 *      frequency. It is possible only if the AHB 64-bit Matrix frequency
 *      does not exceed 90 MHz.
 *   1: H32MXDIV2 The AHB 32-bit Matrix frequency is equal to the AHB 64-bit
 *      Matrix frequency divided by 2.
 *
 ****************************************************************************/

#if defined(PMC_MCKR_H32MXDIV) && defined(NEED_PLLSETUP)
static void __ramfunc__ sam_h32mxdivider(void)
{
  uint32_t regval;

  regval = getreg32(SAM_PMC_MCKR);

  /* Check the 64-bit Matrix frequency (MCK, right?) */

  if (BOARD_MCK_FREQUENCY <= 90000000)
    {
      regval &= ~PMC_MCKR_H32MXDIV;
    }
  else
    {
      regval |= PMC_MCKR_H32MXDIV;
    }

  putreg32(regval, SAM_PMC_MCKR);
}
#else
#  define sam_h32mxdivider()
#endif

/****************************************************************************
 * Name: sam_selectplla
 *
 * Description:
 *   Select the PLLA output as the input clock for PCK and MCK.
 *
 ****************************************************************************/

#if defined(NEED_PLLSETUP)
static void __ramfunc__ sam_selectplla(void)
{
  uint32_t regval;

  /* Select the PLLA output as the main clock input */

  regval  = getreg32(SAM_PMC_MCKR);
  regval &= ~PMC_MCKR_CSS_MASK;
  regval |= BOARD_PMC_MCKR_CSS;
  putreg32(regval, SAM_PMC_MCKR);

  /* Wait for the main clock to be ready again */

  sam_pmcwait(PMC_INT_MCKRDY);
}
#endif

/****************************************************************************
 * Name: sam_usbclockconfig
 *
 * Description:
 *   Configure clocking for USB.
 *
 ****************************************************************************/

static inline void sam_usbclockconfig(void)
{
#if defined(CONFIG_SAMA5_EHCI) || defined(CONFIG_SAMA5_OHCI) || \
    defined(CONFIG_SAMA5_UDPHS)

  /* We can either get the clock from the UPLL or from PLLA.  In this latter
   * case, however, the PLLACK frequency must be a multiple of 48MHz.
   */

#if defined(BOARD_USE_UPLL)
  uint32_t regval;

  /* The USB Host High Speed requires a 480 MHz clock (UPLLCK) for the
   * embedded High-speed transceivers. UPLLCK is the output of the 480 MHz
   * UTMI PLL (UPLL).  The source clock of the UTMI PLL is the Main OSC output:
   * Either the 12MHz internal oscillator on a 12MHz crystal.  The Main OSC
   * must be 12MHz because the UPLL has a built-in 40x multiplier.
   *
   * For High-speed operations, the user has to perform the following:
   *
   *   1) Enable UHP peripheral clock, bit (1 << AT91C_ID_UHPHS) in
   *      PMC_PCER register.
   *   2) Write CKGR_PLLCOUNT field in PMC_UCKR register.
   *   3) Enable UPLL, bit AT91C_CKGR_UPLLEN in PMC_UCKR register.
   *   4) Wait until UTMI_PLL is locked. LOCKU bit in PMC_SR register
   *   5) Enable BIAS, bit AT91C_CKGR_BIASEN in PMC_UCKR register.
   *   6) Select UPLLCK as Input clock of OHCI part, USBS bit in PMC_USB
   *      register.
   *   7) Program the OHCI clocks (UHP48M and UHP12M) with USBDIV field in
   *      PMC_USB register. USBDIV must be 9 (division by 10) if UPLLCK is
   *      selected.
   *   8) Enable OHCI clocks, UHP bit in PMC_SCER register.
   *
   * Steps 2 through 7 performed here.  1 and 8 are performed in the EHCI
   * driver is initialized.
   */

  /* 2) Write CKGR_PLLCOUNT field in PMC_UCKR register. */

  regval = PMC_CKGR_UCKR_UPLLCOUNT(BOARD_CKGR_UCKR_UPLLCOUNT);
  putreg32(regval, SAM_PMC_CKGR_UCKR);

  /* 3) Enable UPLL, bit AT91C_CKGR_UPLLEN in PMC_UCKR register. */

  regval |= PMC_CKGR_UCKR_UPLLEN;
  putreg32(regval, SAM_PMC_CKGR_UCKR);

  /* 4) Wait until UTMI_PLL is locked. LOCKU bit in PMC_SR register */

  sam_pmcwait(PMC_INT_LOCKU);

  /* 5) Enable BIAS, bit AT91C_CKGR_BIASEN in PMC_UCKR register. */

  regval |= PMC_CKGR_UCKR_BIASCOUNT(BOARD_CKGR_UCKR_BIASCOUNT);
  putreg32(regval, SAM_PMC_CKGR_UCKR);

  regval |= PMC_CKGR_UCKR_BIASEN;
  putreg32(regval, SAM_PMC_CKGR_UCKR);

  /* 6) Select UPLLCK as Input clock of OHCI part, USBS bit in PMC_USB
   *    register.
   */

  regval = PMC_USB_USBS_UPLL;
  putreg32(regval, SAM_PMC_USB);

  /* 7) Program the OHCI clocks (UHP48M and UHP12M) with USBDIV field in
   *    PMC_USB register. USBDIV must be 9 (division by 10) if UPLLCK is
   *    selected.
   *
   * REVISIT:  The divisor of 10 produces a rate that is too high with
   * SAMA5D3.  A divisor of 5, however, seems to work just fine for the
   * SAMA5D3.  The SAMA5D4, on the other hand, needs the divisor of 10.
   * No idea why?  Let the board.h file decide which to use.
   */

  regval |= PMC_USB_USBDIV(BOARD_UPLL_OHCI_DIV-1);
  putreg32(regval, SAM_PMC_USB);

#else /* BOARD_USE_UPLL */
  /* For OHCI Full-speed operations only, the user has to perform the
   * following:
   *
   *   1) Enable UHP peripheral clock, bit (1 << AT91C_ID_UHPHS) in PMC_PCER
   *      register.
   *   2) Select PLLACK as Input clock of OHCI part, USBS bit in PMC_USB
   *      register.
   *   3) Program the OHCI clocks (UHP48M and UHP12M) with USBDIV field in
   *      PMC_USB register. USBDIV value is calculated regarding the PLLACK
   *      value and USB Full-speed accuracy.
   *   4) Enable the OHCI clocks, UHP bit in PMC_SCER register.
   *
   * Steps 2 and 3 are done here.  1 and 2 are performed with the OHCI
   * driver is initialized.
   */

  putreg32(BOARD_OHCI_INPUT | BOARD_OHCI_DIVIDER << PMC_USB_USBDIV_SHIFT,
          SAM_PMC_USB);

#endif /* BOARD_USE_UPLL */
#endif /* CONFIG_SAMA5_EHCI ||CONFIG_SAMA5_OHCI) || CONFIG_SAMA5_UDPHS */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_clockconfig
 *
 * Description:
 *   Called to initialize the SAM3/4.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  (After power-on reset, the SAMA5
 *   is initially running on a 12MHz internal RC clock).  This function also
 *   performs other low-level chip initialization of the chip including master
 *   clock, IRQ & watchdog configuration.
 *
 * Boot Sequence
 *
 *   This logic may be executing in ISRAM or in external mmemory: CS0, DDR,
 *   CS1, CS2, or CS3.  It may be executing in CS0 or ISRAM through the
 *   action of the SAMA5 "first level bootloader;"  it might be executing in
 *   CS1-3 through the action of some second level bootloader that provides
 *   configuration for those memories.
 *
 *   The system always boots from the ROM memory at address 0x0000:0000,
 *   starting the internal first level bootloader.  That bootloader can be
 *   configured to work in different ways using the BMS pin and the contents
 *   of the Boot Sequence Configuration Register (BSC_CR).
 *
 *   If the BMS_BIT is read "0", then the first level bootloader will
 *   support execution of code in the memory connected to CS0 on the EBI
 *   interface (presumably NOR flash).  The following sequence is performed
 *   by the first level bootloader if BMS_BIT is "0":
 *
 *     - The main clock is the on-chip 12 MHz RC oscillator,
 *     - The Static Memory Controller is configured with timing allowing
 *       code execution in CS0 external memory at 12 MHz
 *     - AXI matrix is configured to remap EBI CS0 address at 0x0
 *     - 0x0000:0000 is loaded in the Program Counter register
 *
 *   The user software in the external memory must perform the next
 *   operation in order to complete the clocks and SMC timings configuration
 *   to run at a higher clock frequency:
 *
 *     - Enable the 32768 Hz oscillator if best accuracy is needed
 *     - Reprogram the SMC setup, cycle, hold, mode timing registers for EBI
 *       CS0, to adapt them to the new clock.
 *     - Program the PMC (Main Oscillator Enable or Bypass mode)
 *     - Program and Start the PLL
 *     - Switch the system clock to the new value
 *
 *  If the BMS_BIT is read "1", then the first level bootloader will
 *  perform:
 *
 *     - Basic chip initialization: XTal or external clock frequency
 *       detection:
 *
 *       a. Stack Setup for ARM supervisor mode
 *       b. Main Oscillator Detection:  The bootloader attempts to use an
 *          external crystal.  If this is not successful, then  the 12 MHz
 *          Fast RC internal oscillator is used as the main osciallator.
 *       c. Main Clock Selection: The Master Clock source is switched from
 *          to the main oscillator without prescaler. PCK and MCK are now
 *          the Main Clock.
 *       d. PLLA Initialization: PLLA is configured to get a PCK at 96 MHz
 *          and an MCK at 48 MHz. If an external clock or crystal frequency
 *          running at 12 MHz is found, then the PLLA is configured to allow
 *          USB communication.
 *
 *     - Attempt to retrieve a valid code from external non-volatile
 *       memories (NVM): SPI0 CS0 Flash Boot, SD Card Boot, NAND Flash Boot,
 *       SPI0 CS1 Flash Boot, or TWI EEPROM Boot.  Different heuristics are
 *       used with each media type.  If a valid image is found, it is copied
 *       to internal SRAM and started.
 *
 *     - In case no valid application has been found on any NVM, the SAM-BA
 *       Monitor is started.
 *
 ****************************************************************************/

void __ramfunc__ sam_clockconfig(void)
{
#ifdef CONFIG_SAMA5_BOOT_CS0FLASH
  bool config = false;
#endif

  /* Initialize clocking.
   *
   * Check first:  Are we running in CS0?
   */

#ifdef CONFIG_SAMA5_BOOT_CS0FLASH
  /* Yes... did we get here via the first level bootloader?  */

  if ((getreg32(SAM_SFR_EBICFG) & SFR_EBICFG_BMS) == 0)
    {
      /* Yes.. Perform the following operations in order to complete the
       * clocks and SMC timings configuration to run at a higher clock
       * frequency:
       *
       *   - Enable the 32768 Hz oscillator if best accuracy is needed
       *   - Reprogram the SMC setup, cycle, hold, mode timing registers for EBI
       *     CS0, to adapt them to the new clock.
       *
       * Then below:
       *
       *   - Program the PMC (Main Oscillator Enable or Bypass mode)
       *   - Program and Start the PLL
       *   - Switch the system clock to the new value
       */

      /* Enable the 32768 Hz oscillator */
      /* REVISIT! */

      /* Reprogram the SMC setup, cycle, hold, mode timing registers for EBI
       * CS0, to adapt them to the new clock.
       */

      board_norflash_config();
      config = true;
    }
#endif

  /* If we are running from DDRAM or CS1-3, then we will not modify the
   * clock configuration.  In these cases, we have to assume that some
   * secondary bootloader started us here and that the bootloader has
   * configured clocking appropriately.
   *
   * If we are running in CS0, then we may have been started by either
   * the first or second level bootloader.  In either case, we need to
   * update the PLLA settings in order to get a higher performance
   * clock.
   */

#if defined(NEED_PLLSETUP)
#ifdef CONFIG_SAMA5_BOOT_CS0FLASH
  if (config)
#endif /* CONFIG_SAMA5_BOOT_CS0FLASH */
    {
      /* Enable main oscillator (if it has not already been selected) */

      sam_enablemosc();

      /* Select the main oscillator as the input clock for processor clock
       * (PCK) and the main clock (MCK).  The PCK and MCK differ only by the
       * MDIV divisor that permits the MCK to run at a lower rate.
       */

      sam_selectmosc();

      /* Setup PLLA */

      sam_pllasetup();

      /* Configure the MCK PLLA divider. */

      sam_plladivider();

      /* Configure the MCK Prescaler */

      sam_mckprescaler();

      /* Configure MCK Divider */

      sam_mckdivider();

      /* Configure the H32MX Divider */

      sam_h32mxdivider();

      /* Finally, elect the PLLA output as the input clock for PCK and MCK. */

      sam_selectplla();
    }
#endif /* NEED_PLLSETUP */

#ifdef ATSAMA5D2
  /* Enable clocking to the PIO module */

  sam_pio_enableclk();
#endif

  /* Setup USB clocking */

  sam_usbclockconfig();
}
