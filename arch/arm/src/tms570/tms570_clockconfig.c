/****************************************************************************
 * arch/arm/src/tms570/tms570_clockconfig.c
 *
 *   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Some logic in this file was inspired/leveraged from TI's Project0 which
 * has a compatible BSD license:
 *
 *   Copyright (c) 2012, Texas Instruments Incorporated
 *   All rights reserved.
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
#include <assert.h>

#include "arm_internal.h"
#include "hardware/tms570_esm.h"
#include "hardware/tms570_sys.h"
#include "hardware/tms570_sys2.h"
#include "hardware/tms570_pcr.h"
#include "hardware/tms570_flash.h"
#include "hardware/tms570_iomm.h"
#include "hardware/tms570_pinmux.h"

#include "tms570_selftest.h"
#include "tms570_clockconfig.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef BOARD_VCLK_DIVIDER
#  error BOARD_VCLK_DIVIDER is not defined
#endif

#if BOARD_VCLK_DIVIDER == 1
#  define SYS_CLKCNTL_VCLKR SYS_CLKCNTL_VCLKR_DIV1
#elif BOARD_VCLK_DIVIDER == 2
#  define SYS_CLKCNTL_VCLKR SYS_CLKCNTL_VCLKR_DIV2
#else
#  error Invalid value for BOARD_VCLK_DIVIDER
#endif

#ifndef BOARD_VCLK2_DIVIDER
#  error BOARD_VCLK2_DIVIDER is not defined
#endif

#if BOARD_VCLK2_DIVIDER == 1
#  define SYS_CLKCNTL_VCLKR2 SYS_CLKCNTL_VCLKR2_DIV1
#elif BOARD_VCLK2_DIVIDER == 2
#  define SYS_CLKCNTL_VCLKR2 SYS_CLKCNTL_VCLKR2_DIV2
#else
#  error Invalid value for SYS_CLKCNTL_VCLKR2_DIV2
#endif

#ifndef BOARD_RTICLK_DIVIDER
#  error BOARD_RTICLK_DIVIDER is not defined
#endif

#if BOARD_RTICLK_DIVIDER == 1
#  define SYS_RCLKSRC_RTI1DIV SYS_RCLKSRC_RTI1DIV_DIV1
#elif BOARD_RTICLK_DIVIDER == 2
#  define SYS_RCLKSRC_RTI1DIV SYS_RCLKSRC_RTI1DIV_DIV2
#elif BOARD_RTICLK_DIVIDER == 4
#  define SYS_RCLKSRC_RTI1DIV SYS_RCLKSRC_RTI1DIV_DIV4
#elif BOARD_RTICLK_DIVIDER == 8
#  define SYS_RCLKSRC_RTI1DIV SYS_RCLKSRC_RTI1DIV_DIV8
#else
#  error Invalid value for BOARD_RTICLK_DIVIDER
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct tms570_pinmux_s g_pinmux_table[] =
{
  BOARD_PINMUX_INITIALIZER
};

#define NPINMUX (sizeof(g_pinmux_table) / sizeof(struct tms570_pinmux_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#define ESM_SR1_PLL1SLIP     0x400
#define ESM_SR4_PLL2SLIP     0x400
#define DCC1CNT1_CLKSRC_PLL1 0x0000a000u
#define DCC1CNT1_CLKSRC_PLL2 0x0000a001u

/****************************************************************************
 * Name: check_frequency
 *
 * Description:
 *   This function is used to verify is the Main Clock frequency correct
 ****************************************************************************/

static uint32_t check_frequency(uint32_t cnt1_clksrc)
{
  uint32_t regval = 0;

  /* Setup DCC1: Global Control register configuration */

  regval =  (uint32_t)0x5u |                   /* Disable  DCC1 */
            (uint32_t)((uint32_t)0x5u << 4u) | /* No Error Interrupt */
            (uint32_t)((uint32_t)0xau << 8u) | /* Single Shot mode */
            (uint32_t)((uint32_t)0x5u << 12u); /* No Done Interrupt */
  putreg32(regval, TMS570_DCC_BASE);

  /* Clear ERR and DONE bits */

  regval = 3u;
  putreg32(regval, TMS570_DCC_BASE + 0x14);

  /* DCC1 Clock0 Counter Seed value configuration */

  regval = 68u;
  putreg32(regval, TMS570_DCC_BASE + 0x08);

  /* DCC1 Clock0 Valid Counter Seed value configuration */

  regval = 4u;
  putreg32(regval, TMS570_DCC_BASE + 0x0c);

  /* DCC1 Clock1 Counter Seed value configuration */

  regval =  972u;
  putreg32(regval, TMS570_DCC_BASE + 0x10);

  /* DCC1 Clock1 Source 1 Select */

  regval = (uint32_t)((uint32_t)10u << 12u) |  /* DCC Enable / Disable Key */
                      (uint32_t) cnt1_clksrc;  /* DCC1 Clock Source 1 */
  putreg32(regval, TMS570_DCC_BASE + 0x24);

  regval = (uint32_t)15;  /* DCC1 Clock Source 0 */
  putreg32(regval, TMS570_DCC_BASE + 0x28);

  /* DCC1 Global Control register configuration */

  regval = (uint32_t)0xau |                    /* Enable  DCC1 */
           (uint32_t)((uint32_t)0x5u << 4u) |  /* No Error Interrupt */
           (uint32_t)((uint32_t)0xau << 8u) |  /* Single Shot mode */
           (uint32_t)((uint32_t)0x5u << 12u);  /* No Done Interrupt */

  putreg32(regval, TMS570_DCC_BASE);
  while (getreg32(TMS570_DCC_BASE + 0x14) == 0u)
    {
      /* Wait */
    }

  return (getreg32(TMS570_DCC_BASE + 0x14) & 0x01u);
}

/****************************************************************************
 * Name: _errata_sswf021_45_both_plls
 *
 * Description:
 *   This function is used to verify that PLL1 and PLL2 lock after
 *   system start-up. If PLL does not lock after system start-up
 *   and PLL slip occur then this function should be called at the
 *   beginning of tms570_clockconfig. (Errata sheet TMS570)
 *
 ****************************************************************************/

uint32_t _errata_sswf021_45_both_plls(uint32_t count)
{
  uint32_t failcode = 0u;
  uint32_t retries;
  uint32_t clkcntrlsave;
  uint32_t regval;

  /* Save CLKCNTL, */

  clkcntrlsave = getreg32(TMS570_SYS_CLKCNTL);

  /* First set VCLK2 = HCLK */

  regval = clkcntrlsave & 0x000f0100u;
  putreg32(regval, TMS570_SYS_CLKCNTL);

  /* Now set VCLK = HCLK and enable peripherals */

  regval = SYS_CLKCNTL_PENA;
  putreg32(regval, TMS570_SYS_CLKCNTL);

  for (retries = 0u; (retries < count) || (count == 0u); retries++)
    {
      failcode = 0u;

      /* Disable PLL1 and PLL2 */

      regval =  0x00000002u | 0x00000040u;
      putreg32(regval, TMS570_SYS_CSDISSET);

      while ((getreg32(TMS570_SYS_CSDIS) & regval) != regval)
        {
        }

      /* Clear Global Status Register */

      regval = 0x00000301u;
      putreg32(regval, TMS570_SYS_GLBSTAT);

      /* Clear the ESM PLL slip flags */

      putreg32(ESM_SR1_PLL1SLIP, TMS570_ESM_SR1);
      putreg32(ESM_SR4_PLL2SLIP, TMS570_ESM_SR4);

      /* Set both PLLs to OSCIN/1*27/(2*1) */

      regval = 0x20001a00u;
      putreg32(regval, TMS570_SYS_PLLCTL1);

      regval = 0x3fc0723du;
      putreg32(regval, TMS570_SYS_PLLCTL2);

      regval = 0x20001a00u;
      putreg32(regval, 0xffffe100);

      regval =  0x00000002u | 0x00000040u;
      putreg32(regval, TMS570_SYS_CSDISCLR);

      /* Check for (PLL1 valid or PLL1 slip) and (PLL2 valid or PLL2 slip) */

      while ((((getreg32(TMS570_SYS_CSVSTAT) & SYS_CLKSRC_PLL1) == 0u) &&
              ((getreg32(TMS570_ESM_SR1) & ESM_SR1_PLL1SLIP) == 0u)) ||
             (((getreg32(TMS570_SYS_CSVSTAT) & SYS_CLKSRC_PLL2) == 0u) &&
              ((getreg32(TMS570_ESM_SR4) & ESM_SR4_PLL2SLIP) == 0u)))
        {
          /* Wait */
        }

      /* If PLL1 valid, check the frequency */

      if ((getreg32(TMS570_ESM_SR1) & ESM_SR1_PLL1SLIP) != 0u)
        {
          failcode |= 1u;
        }
      else
        {
          failcode |= check_frequency(DCC1CNT1_CLKSRC_PLL1);
        }

      /* If PLL2 valid, check the frequency */

      if ((getreg32(TMS570_ESM_SR4) & ESM_SR4_PLL2SLIP) != 0u)
        {
          failcode |= 2u;
        }
      else
        {
          failcode |= (check_frequency(DCC1CNT1_CLKSRC_PLL2) << 1U);
        }

      if (failcode == 0u)
        {
          break;
        }
    }

  /* Disable plls */

  regval = 0x00000002u | 0x00000040u;
  putreg32(regval, TMS570_SYS_CSDISSET);

  while ((getreg32(TMS570_SYS_CSDIS) & regval) != regval)
    {
    }

  /* Restore CLKCNTL, VCLKR and PENA first */

  clkcntrlsave = getreg32(TMS570_SYS_CLKCNTL);

  /* First set VCLK2 = HCLK */

  regval = clkcntrlsave & 0x000f0100u;
  putreg32(regval, TMS570_SYS_CLKCNTL);

  putreg32(clkcntrlsave, TMS570_SYS_CLKCNTL);
  return failcode;
}

/****************************************************************************
 * Name: tms570_pll_setup
 *
 * Description:
 *   Configure PLL control registers.  The PLL takes (127 + 1024  NR)
 *   oscillator cycles to acquire lock.  This initialization sequence
 *   performs all the actions that are not required to be done at full
 *   application speed while the PLL locks.
 *
 ****************************************************************************/

static void tms570_pll_setup(void)
{
  uint32_t regval;

  /* Configure PLL control registers */

  /* Setup pll control register 1
   *
   * REFCLKDIV controls input clock divider:
   *
   *  NR = REFCLKDIV+1
   *  Fintclk = Fclkin / NR
   *
   * PLLMUL controls multipler on divided input clock (Fintclk):
   *
   *  Non-modulated:
   *    NF = (PLLMUL + 256) / 256
   *  Modulated:
   *    NF = (PLLMUL + MULMOD + 256) / 256
   *
   *  Foutputclk = Fintclk x NF (150MHz - 550MHz)
   *
   * ODPLL controls internal PLL output divider:
   *
   *   OD = ODPLL+1
   *   Fpostodclk = Foutputclock / OD
   *
   * Final divisor, R, controls PLL output:
   *
   *   R = PLLDIV + 1
   *   Fpllclock = Fpostodclk / R
   *
   * Or:
   *
   *   Fpllclock = = (Fclkin / NR) x NF / OD / R
   *
   * For example, if the clock source is a 16MHz crystal, then
   *
   *   Fclkin = 16,000,000
   *   NR     = 6   (REFCLKDIV=5)
   *   NF     = 120 (PLLMUL = 119 * 256)
   *   OD     = 1   (ODPLL = 0)
   *   R      = 32  (PLLDIV=31)
   *
   * Then:
   *
   *   Fintclk      = 16 MHz / 6      = 2.667 MHz
   *   Foutputclock = 2.667 MHz * 120 = 320 MHz
   *   Fpostodclock = 320 MHz / 2     = 160 MHz
   *   Fpllclock    = 160 MHz / 2     = 80 MHz
   *
   * NOTE: That R is temporary set to the maximum (32) here.
   */

  /* Turn off PLL1 and PLL2 */

  regval =  SYS_CSDIS_CLKSRC_PLL1 | SYS_CSDIS_CLKSRC_PLL2;
  putreg32(regval, TMS570_SYS_CSDISSET);

  while ((getreg32(TMS570_SYS_CSDIS) & regval) != regval)
    {
    }

  /* Check for OSC failure */

  regval = getreg32(TMS570_SYS_GLBSTAT);
  if ((regval & SYS_GLBSTAT_OSC_ERR_MASK) == SYS_GLBSTAT_OSC_ERR_MASK)
    {
      regval = SYS_GHVSRC_GHVSRC_LPOHIGH | SYS_GHVSRC_HVLPM_LPOHIGH |
               SYS_GHVSRC_GHVWAKE_LPOHIGH;
      putreg32(regval, TMS570_SYS_GHVSRC);

      regval =  SYS_CSDIS_CLKSRC_PLL1 | SYS_CSDIS_CLKSRC_PLL2 |
                SYS_CSDIS_CLKSRC_OSC;
      putreg32(regval, TMS570_SYS_CSDISSET);

      while ((getreg32(TMS570_SYS_CSDIS) & regval) != regval)
        {
        }

      putreg32(SYS_GLBSTAT_OSC_ERR_CLR, TMS570_SYS_GLBSTAT);

      regval =  SYS_CSDIS_CLKSRC_OSC;
      putreg32(regval, TMS570_SYS_CSDISCLR);

      while ((getreg32(TMS570_SYS_CSDIS) & regval) != regval)
        {
        }
    }

  /* Setup pll control register 1 */

  regval = SYS_PLLCTL1_PLLMUL((BOARD_PLL_NF - 1) << 8) |
           SYS_PLLCTL1_REFCLKDIV(BOARD_PLL_NR - 1) |
           SYS_PLLCTL1_PLLDIV_MAX |
           (SYS_PLLCTL1_MASKSLIP_ENABLE);
  putreg32(regval, TMS570_SYS_PLLCTL1);

  /* Setup pll control register 2 */

  regval = SYS_PLLCTL2_SPRAMOUNT(61) |
           SYS_PLLCTL2_ODPLL(BOARD_PLL_OD - 1) |
           SYS_PLLCTL2_MULMOD(7) |
           SYS_PLLCTL2_SPRRATE(255);
  putreg32(regval, TMS570_SYS_PLLCTL2);

  /* Enable PLL(s) to start up or Lock.
   *
   * On wakeup, only clock sources 0, 4, and 5 are enabled: Oscillator, Low
   * and high Frequency LPO.  Clear bit 1 to enable the PLL.  Only the
   * external clock remains disabled.
   */

  regval =  SYS_CSDIS_CLKSRC_PLL1 | SYS_CSDIS_CLKSRC_PLL2;
  putreg32(regval, TMS570_SYS_CSDISCLR);

  while ((getreg32(TMS570_SYS_CSDIS) & regval) != 0)
    {
    }
}

/****************************************************************************
 * Name: tms570_peripheral_initialize
 *
 * Description:
 *   Release peripherals from reset and enable clocks to all peripherals.
 *
 ****************************************************************************/

static void tms570_peripheral_initialize(void)
{
  uint32_t regval;
  uint32_t clkcntl;

  /* Disable Peripherals by clearing the PENA bit in the CLKCNTRL register
   * before peripheral powerup
   */

  clkcntl  = getreg32(TMS570_SYS_CLKCNTL);
  clkcntl &= ~SYS_CLKCNTL_PENA;
  putreg32(clkcntl, TMS570_SYS_CLKCNTL);

  /* Release peripherals from reset and enable clocks to all peripherals.
   * Power-up all peripherals by clearing the power down bit for each
   * quadrant of each peripheral.
   *
   * REVISIT: Should we only enable peripherals that are configured?
   */

  regval = PCR_PSPWERDWN0_PS0_QALL  | PCR_PSPWERDWN0_PS1_QALL  |
           PCR_PSPWERDWN0_PS2_QALL  | PCR_PSPWERDWN0_PS3_QALL  |
           PCR_PSPWERDWN0_PS4_QALL  | PCR_PSPWERDWN0_PS5_QALL  |
           PCR_PSPWERDWN0_PS6_QALL  | PCR_PSPWERDWN0_PS7_QALL;
  putreg32(regval, TMS570_PCR_PSPWRDWNCLR0);

  regval = PCR_PSPWERDWN1_PS8_QALL  | PCR_PSPWERDWN1_PS9_QALL  |
           PCR_PSPWERDWN1_PS10_QALL | PCR_PSPWERDWN1_PS11_QALL |
           PCR_PSPWERDWN1_PS12_QALL | PCR_PSPWERDWN1_PS13_QALL |
           PCR_PSPWERDWN1_PS14_QALL | PCR_PSPWERDWN1_PS15_QALL;
  putreg32(regval, TMS570_PCR_PSPWRDWNCLR1);

  regval = PCR_PSPWERDWN2_PS16_QALL | PCR_PSPWERDWN2_PS17_QALL |
           PCR_PSPWERDWN2_PS18_QALL | PCR_PSPWERDWN2_PS19_QALL |
           PCR_PSPWERDWN2_PS20_QALL | PCR_PSPWERDWN2_PS21_QALL |
           PCR_PSPWERDWN2_PS22_QALL | PCR_PSPWERDWN2_PS23_QALL;
  putreg32(regval, TMS570_PCR_PSPWRDWNCLR2);

  regval = PCR_PSPWERDWN3_PS24_QALL | PCR_PSPWERDWN3_PS25_QALL |
           PCR_PSPWERDWN3_PS26_QALL | PCR_PSPWERDWN3_PS27_QALL |
           PCR_PSPWERDWN3_PS28_QALL | PCR_PSPWERDWN3_PS29_QALL |
           PCR_PSPWERDWN3_PS30_QALL | PCR_PSPWERDWN3_PS31_QALL;
  putreg32(regval, TMS570_PCR_PSPWRDWNCLR3);

  /* Enable Peripherals */

  clkcntl |= SYS_CLKCNTL_PENA;
  putreg32(clkcntl, TMS570_SYS_CLKCNTL);
}

/****************************************************************************
 * Name: tms570_pin_multiplex
 *
 * Description:
 *   Configure the field for a single pin in a PINMMR register
 *
 ****************************************************************************/

static void tms570_pin_multiplex(const struct tms570_pinmux_s *pinmux)
{
  uintptr_t regaddr;
  uint32_t  regval;

  regaddr = TMS570_IOMM_PINMMR(pinmux->mmrndx);
  regval  = getreg32(regaddr);
  regval &= ~(0xff << pinmux->shift);
  regval |= ((uint32_t)(pinmux->value) << pinmux->shift);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: tms570_io_multiplex
 *
 * Description:
 *   Configure the all pins in the board-provided pinmux table.
 *
 ****************************************************************************/

static void tms570_io_multiplex(void)
{
  int i;

  /* Enable access to pin multiplexing registers */

  putreg32(IOMM_KICK0_UNLOCK, TMS570_IOMM_KICK0);
  putreg32(IOMM_KICK1_UNLOCK, TMS570_IOMM_KICK1);

  /* Configure each pin selected by the board-specific logic */

  for (i = 0; i < NPINMUX; i++)
    {
      tms570_pin_multiplex(&g_pinmux_table[i]);
    }

  /* Disable access to pin multiplexing registers */

  putreg32(IOMM_KICK0_LOCK, TMS570_IOMM_KICK0);
  putreg32(IOMM_KICK1_LOCK, TMS570_IOMM_KICK1);
}

/****************************************************************************
 * Name: tms570_lpo_trim
 *
 * Description:
 *   Configure the LPO such that HF LPO is as close to 10MHz as possible.
 *
 ****************************************************************************/

static void tms570_lpo_trim(void)
{
  uint32_t regval;
  uint32_t lotrim;

  /* The LPO trim value may be available in TI OTP */

  lotrim = (getreg32(TMS570_TITCM_LPOTRIM) & TMS570_TITCM_LPOTRIM_MASK) >>
    TMS570_TITCM_LPOTRIM_SHIFT;

  /* Use if the LPO trim value TI OTP if programmed.  Otherwise, use a
   * default value.
   */

  if (lotrim != 0xffff)
    {
      regval = SYS_LPOMONCTL_BIASENABLE | lotrim;
    }
  else
    {
      regval = SYS_LPOMONCTL_BIASENABLE |
               SYS_LPOMONCTL_HFTRIM_100p00 |
               SYS_LPOMONCTL_100p00;
    }

  putreg32(regval, TMS570_SYS_LPOMONCTL);
}

/****************************************************************************
 * Name: tms570_flash_setup
 *
 * Description:
 *   Set up flash address and data wait states based on the target CPU clock
 *   frequency The number of address and data wait states for the target CPU
 *   clock frequency are specified in the specific part's datasheet.
 *
 ****************************************************************************/

static void tms570_flash_setup(void)
{
  uint32_t regval;

  /* Setup flash read mode, address wait states and data wait states
   *
   *   ENPIPE=1, Bit 0, Enable pipeline mode.
   *   ASWSTEN=0/1, Bit 1, Address Setup Wait State is enabled/disabled.
   *   RWAIT=BOARD_RWAIT, Bits 8-11, Wait states added to FLASH read access
   */

  regval  = FLASH_FRDCNTL_ENPIPE | FLASH_FRDCNTL_RWAIT(BOARD_RWAIT);
#if defined(BOARD_ASWAIT) && BOARD_ASWAIT > 0
  regval |= FLASH_FRDCNTL_ASWSTEN;
#endif
  putreg32(regval, TMS570_FLASH_FRDCNTL);

  /* Setup flash access wait states for bank 7
   *
   *  AUTOSTART_GRACE=2, Bits 0-7, Auto-suspend Startup Grace Period
   *  AUTOSUSPEN=0, Bit 8, Auto suspend is disabled.
   *  EWAIT=4, Bits 16-19, EEPROM wait states
   */

  putreg32(FLASH_FSMWRENA_ENABLE, TMS570_FLASH_FSMWRENA);
  regval = FLASH_EEPROMCFG_GRACE(2) | FLASH_EEPROMCFG_EWAIT(BOARD_EWAIT);
  putreg32(regval, TMS570_FLASH_EEPROMCFG);
#if 0
  putreg32(FLASH_FSMWRENA_DISABLE, TMS570_FLASH_FSMWRENA);
#endif
  putreg32(0x0a, TMS570_FLASH_FSMWRENA);

  /* Setup flash bank power modes */

  regval = FLASH_FBFALLBACK_BANKPWR0_ACTIV |
           FLASH_FBFALLBACK_BANKPWR1_ACTIV |
           FLASH_FBFALLBACK_BANKPWR7_ACTIV;
  putreg32(regval, TMS570_FLASH_FBFALLBACK);
}

/****************************************************************************
 * Name: tms570_clocksrc_configure
 *
 * Description:
 *   Finalize PLL configuration, enable and configure clocks sources.
 *
 ****************************************************************************/

static void tms570_clocksrc_configure(void)
{
  uint32_t regval;
  uint32_t csvstat;
  uint32_t csdis;

  /* Disable / Enable clock domains.  Writing a '1' to the CDDIS register
   * turns the clock off.
   *
   *   GCLK          Bit 0  On
   *   HCLK/VCLK_sys Bit 1  On
   *   VCLK_periph   Bit 2  On
   *   VCLK2         Bit 3  On
   *   VCLKA1        Bit 4  On
   *   RTICLK1       Bit 6  On
   *   TCLK_EQEP     Bit 9  On
   */

  putreg32(0, TMS570_SYS_CDDISSET);

  /* Work Around for Errata SYS#46: Errata Description: Clock Source
   * Switching Not Qualified with Clock Source Enable And Clock Source Valid
   * Workaround: Always check the CSDIS register to make sure the clock
   * source is turned on and check the CSVSTAT register to make sure the
   * clock source is valid. Then write to GHVSRC to switch the clock.
   */

  do
    {
      /* Get the set of valid clocks */

      csvstat = getreg32(TMS570_SYS_CSVSTAT);

      /* Get the (inverted) state of each clock.  Inverted so that '1' means
       * ON not OFF.
       */

      csdis = (getreg32(TMS570_SYS_CSDIS) ^ SYS_CSDIS_CLKSROFFALL) &
               SYS_CSDIS_CLKSROFFALL;
    }
  while ((csvstat & csdis) != csdis);

  regval  = getreg32(TMS570_SYS_PLLCTL1);
  regval &= ~SYS_PLLCTL1_PLLDIV_MASK;
  regval |= SYS_PLLCTL1_PLLDIV(BOARD_PLL_R - 1);
  putreg32(regval, TMS570_SYS_PLLCTL1);

  /* Map device clock domains to desired sources and configure top-level
   * dividers.  All clock domains were working off the default clock sources
   * until this point.
   *
   * Setup GCLK, HCLK and VCLK clock source for normal operation, power down
   * mode and after wakeup
   */

  regval = SYS_GHVSRC_GHVSRC_PLL1 | SYS_GHVSRC_HVLPM_PLL1 |
           SYS_GHVSRC_GHVWAKE_PLL1;
  putreg32(regval, TMS570_SYS_GHVSRC);

  /* Setup RTICLK1 and RTICLK2 clocks */

  regval = SYS_RCLKSRC_RTI1SRC_VCLK;
  putreg32(regval, TMS570_SYS_RCLKSRC);

  /* Now the PLLs are locked and the PLL outputs can be sped up.  The R-
   * divider was programmed to be 0xF. Now this divider is changed to
   * programmed value
   */

  /* Setup asynchronous peripheral clock sources for AVCLK1 */

#if defined(CONFIG_ARCH_CHIP_TMS570LS3137ZWT)
  regval = SYS_VCLKASRC_VCLKA2S_VCLK | SYS_VCLKASRC_VCLKA1S_VCLK;
#else
  regval = SYS_VCLKASRC_VCLKA1S_VCLK;
#endif
  putreg32(regval, TMS570_SYS_VCLKASRC);

#if defined(CONFIG_ARCH_CHIP_TMS570LS3137ZWT)
  regval = SYS_VCLKASRC_VCLKA4S_VCLK | SYS_VCLKASRC_VCLKA3R_VCLK;
  putreg32(regval, TMS570_SYS2_VCLKACON1);
#endif

  /* Setup synchronous peripheral clock dividers for VCLK1, VCLK2, VCLK3 */

  regval  = getreg32(TMS570_SYS_CLKCNTL);
  regval &= ~(SYS_CLKCNTL_VCLKR2_MASK);
  regval |= SYS_CLKCNTL_VCLKR2;
  putreg32(regval, TMS570_SYS_CLKCNTL);

  regval  = getreg32(TMS570_SYS_CLKCNTL);
  regval &= ~(SYS_CLKCNTL_VCLKR_MASK);
  regval |= SYS_CLKCNTL_VCLKR;
  putreg32(regval, TMS570_SYS_CLKCNTL);

#if defined(CONFIG_ARCH_CHIP_TMS570LS3137ZWT)
  regval = getreg32(TMS570_SYS2_CLK2CNTRL);
  regval &= ~(SYS_CLKC2NTL_VCLK3R_MASK);
  regval |= SYS_CLK2CNTL_VCLK3R_DIV2;
  putreg32(regval, TMS570_SYS2_CLK2CNTRL);
#endif
}

/****************************************************************************
 * Name: tms570_eclk_configure
 *
 * Description:
 *   Configure the External Clock (ECLK) pin.
 *
 ****************************************************************************/

static void tms570_eclk_configure(void)
{
  uint32_t regval;

  /* Configure ECLK pins
   *
   * PC1 0=ECLK is in GIO mode
   * PC4 0=ECLK pin is driven to logic low
   * PC2 1=ECLK pin is an output
   * PC7 0=CLK pin is configured in push/pull mode
   * PC8 0=ECLK pull enable is active
   * PC9 1=ECLK pull up is selected, when pull up/pull down logic is enabled
   */

  putreg32(0, TMS570_SYS_PC1);
  putreg32(0, TMS570_SYS_PC4);
  putreg32(SYS_PC2_ECPCLKDIR, TMS570_SYS_PC2);
  putreg32(0, TMS570_SYS_PC7);
  putreg32(0, TMS570_SYS_PC8);
  putreg32(SYS_PC9_ECPCLKPS, TMS570_SYS_PC9);

  /* Setup ECLK:
   *
   * ECPDIV=7   Bits 0-15, ECP divider value = 8
   * ECPINSEL=0 Bits 16-17, Select ECP input clock source is tied low
   * ECPCOS=0   Bit 23, ECLK output is disabled in suspend mode
   * ECPINSEL=0 Bit 24, VCLK is selected as the ECP clock source
   */

  regval = SYS_ECPCNTL_ECPDIV(8 - 1) | SYS_ECPCNTL_ECPINSEL_LOW;
  putreg32(regval, TMS570_SYS_ECPCNTL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_clockconfig
 *
 * Description:
 *   Called to initialize TMS570 clocking.  This does whatever setup is
 *   needed to put the SoC in a usable state.  This includes, but is not
 *   limited to, the initialization of clocking using the settings in the
 *   board.h header file.
 *
 ****************************************************************************/

void tms570_clockconfig(void)
{
#ifdef CONFIG_TMS570_SELFTEST
  int check;
#endif /* CONFIG_TMS570_SELFTEST */

  /* Configure PLL control registers and enable PLLs. */

  tms570_pll_setup();

#ifdef CONFIG_TMS570_SELFTEST
  /* Run eFuse controller start-up checks and start eFuse controller ECC
   * self-test.
   */

  tms570_efc_selftest_start();
#endif /* CONFIG_TMS570_SELFTEST */

  /* Enable clocks to peripherals and release peripheral reset */

  tms570_peripheral_initialize();

  /* Configure device-level multiplexing and I/O multiplexing */

  tms570_io_multiplex();

#ifdef CONFIG_TMS570_SELFTEST
  /* Wait for eFuse controller self-test to complete and check results */

  check = tms570_efc_selftest_complete();
  DEBUGASSERT(check == 0);
  UNUSED(check);
#endif /* CONFIG_TMS570_SELFTEST */

  /* Set up flash address and data wait states. */

  tms570_flash_setup();

  /* Configure the LPO such that HF LPO is as close to 10MHz as possible */

  tms570_lpo_trim();

  /* Finalize PLL configuration, enable and configure clocks sources. */

  tms570_clocksrc_configure();

  /* Configure ECLK */

  tms570_eclk_configure();
}
