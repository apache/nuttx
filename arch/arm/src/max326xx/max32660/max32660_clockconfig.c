/****************************************************************************
 * arch/arm/src/max32660/max326_clockconfig.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <debug.h>

#include <nuttx/irq.h>

#include "arm_arch.h"

#include "hardware/max326_gcr.h"
#include "hardware/max326_pwrseq.h"
#include "hardware/max326_flc.h"
#include "max326_periphclks.h"
#include "max326_clockconfig.h"

#include <arch/board/board.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This describes the initial clock configuration.  It is needed by
 * max326_start.c
 */

const struct clock_setup_s g_initial_clock_setup =
{
  .ovr    = 2,            /* Output voltage range for internal regulator */
  .clksrc = CLKSRC_HFIO,  /* See enum clock_source_e.  Determines Fsysosc */
  .psc    = 0,            /* System Oscillator Prescaler.  Derives Fsysclk */
  .hfio   = true,         /* True: Enable the High frequency internal oscillator. */
#ifdef BOARD_HAVE_X32K
  .x32k   = true,         /* True: Enable the 32.768KHz ext crystal oscillator. */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_sysosc_frequency
 *
 * Description:
 *   Return the SYSOSC frequency.  This is simply the frequency of the
 *   selected clock source.
 *
 ****************************************************************************/

static uint32_t max326_sysosc_frequency(void)
{
  uint32_t regval;

  /* The clock source is in the GCR_CLKCTRL:CLKCTRL field */

  regval = getreg32(MAX326_GCR_CLKCTRL);
  switch ((regval & GCR_CLKCTRL_CLKSEL_MASK) >> GCR_CLKCTRL_CLKSEL_SHIFT)
    {
      default:
        DEBUGPANIC();
      case CLKSRC_HFIO:
        return max326_hfio_frequency();

      case CLKSRC_8KHZ:
        return 8000000; /* Nominally 8MHz */

      case CLKSRC_32KHZ:
        return 32768;
    }
}

/****************************************************************************
 * Name: max326_sysclk_frequency
 *
 * Description:
 *   Return the SYSCLK frequency.  The SYSCLOCK is simply divided down from
 *   the SYSOSC.
 *
 ****************************************************************************/

static uint32_t max326_sysclk_frequency(void)
{
  uint32_t regval;
  uint32_t psc;

  /* The divider is in the GCR_CLKCTRL:PSC setting:
   *
   * Fsysclk = Fsysclk / (2^psc)
  */

  regval = getreg32(MAX326_GCR_CLKCTRL);
  psc    = (regval & GCR_CLKCTRL_PSC_MASK) >> GCR_CLKCTRL_PSC_SHIFT;
  return max326_sysosc_frequency() >> psc;
}

/****************************************************************************
 * Name: max326_enable_hfio
 *
 * Description:
 *   Enable High-Frequency Internal Oscillator (HFIO) if it is needed.
 *
 ****************************************************************************/

static void max326_enable_hfio(FAR const struct clock_setup_s *clksetup)
{
  uint32_t regval;

 /* Check if the HFIO is needed. */

  if (clksetup->hfio)
    {
      /* Yes.. Enable the HFIO. */

      regval  = getreg32(MAX326_GCR_CLKCTRL);
      regval |= GCR_CLKCTRL_HIRCEN;
      putreg32(regval, MAX326_GCR_CLKCTRL);

      /* Wait for the oscillator to become ready */

      while ((getreg32(MAX326_GCR_CLKCTRL) & GCR_CLKCTRL_HIRCRDY) == 0)
        {
        }
    }
}

/****************************************************************************
 * Name: max326_disable_hfio
 *
 * Description:
 *   Disable the High-Frequency Internal Oscillator (HFIO) if it is not used.
 *
 ****************************************************************************/

static void max326_disable_hfio(FAR const struct clock_setup_s *clksetup)
{
  uint32_t regval;

  /* Check if the HFIO is used. */

  if (!clksetup->hfio)
    {
      /* No.. Disable the HFIO */

      regval  = getreg32(MAX326_GCR_CLKCTRL);
      regval &= ~GCR_CLKCTRL_HIRCEN;
      putreg32(regval, MAX326_GCR_CLKCTRL);
    }
}

/****************************************************************************
 * Name: max326_select_hfio
 *
 * Description:
 *   Select the High-Frequency Internal Oscillator (HFIO) as the SYSOSC
 *   clock source.
 *
 ****************************************************************************/

static void max326_select_hfio(void)
{
  uint32_t regval;

  /* Select the HIFO as the SYSOSC clock source. */

  regval  = getreg32(MAX326_GCR_CLKCTRL);
  regval &= ~GCR_CLKCTRL_CLKSEL_MASK;
  regval |= GCR_CLKCTRL_CLKSEL_HIRC;
  putreg32(regval, MAX326_GCR_CLKCTRL);
}

/****************************************************************************
 * Name: max326_select_lirc8k
 *
 * Description:
 *   Select the 8kHz Internal Ultra-Low Power Nano-Ring Oscillator as the
 *   SYSOSC clock source.
 *
 ****************************************************************************/

static void max326_select_lirc8k(void)
{
  uint32_t regval;

  /* The 8kHz nano-ring oscillator is "always-on":  "This oscillator is
   * enabled at device powerup by hardware and cannot be disabled by
   * application firmware.
   */

  /* Make sure that the oscillator is ready */

  while ((getreg32(MAX326_GCR_CLKCTRL) & GCR_CLKCTRL_LIRC8KRDY) == 0)
    {
    }

  /* Select the 8kHz nano-ring oscillator as the SYSOSC clock source. */

  regval  = getreg32(MAX326_GCR_CLKCTRL);
  regval &= ~GCR_CLKCTRL_CLKSEL_MASK;
  regval |= GCR_CLKCTRL_CLKSEL_LIRC8K;
  putreg32(regval, MAX326_GCR_CLKCTRL);
}

/****************************************************************************
 * Name: max326_enable_x32k
 *
 * Description:
 *   Enable the 32.768kHz External Crystal Oscillator if is it needed.
 *
 ****************************************************************************/

#ifdef BOARD_HAVE_X32K
static void max326_enable_x32k(FAR const struct clock_setup_s *clksetup)
{
  uint32_t regval;

  /* Check if the 2.768kHz External Crystal Oscillator is needed. */

  regval = getreg32(MAX326_GCR_CLKCTRL);
  if (clksetup->x32k)
    {
      /* Yes.. Enable the 32.768kHz external oscillator. */

      regval |= GCR_CLKCTRL_X32KEN;
      putreg32(regval, MAX326_GCR_CLKCTRL);

      /* Wait for the oscillator to become ready */

      while ((getreg32(MAX326_GCR_CLKCTRL) & GCR_CLKCTRL_X32KRDY) == 0)
        {
        }
    }
}
#endif

/****************************************************************************
 * Name: max326_disable_x32k
 *
 * Description:
 *   Disable the 32.768 KHz crystal oscillator if it is not used
 *
 ****************************************************************************/

#ifdef BOARD_HAVE_X32K
static void max326_disable_x32k(FAR const struct clock_setup_s *clksetup)
{
  uint32_t regval;

  /* Check if the 32.768 KHz crystal oscillator is used */

  if (!clksetup->x32k)
    {
      /* No.. Disable the 32.768kHz external oscillator. */

      regval  = getreg32(MAX326_GCR_CLKCTRL);
      regval &= ~GCR_CLKCTRL_X32KEN;
      putreg32(regval, MAX326_GCR_CLKCTRL);
    }
}
#endif

/****************************************************************************
 * Name: max326_select_x32k
 *
 * Description:
 *   Select the 32.768kHz External Crystal Oscillator as the SYSOSC clock
 *   source.
 *
 ****************************************************************************/

#ifdef BOARD_HAVE_X32K
static void max326_select_x32k(void)
{
  uint32_t regval;

  /* Select the 32.768kHz external oscillator as the SYSOSC clock
   * source.
   */

  regval  = getreg32(MAX326_GCR_CLKCTRL);
  regval &= ~GCR_CLKCTRL_CLKSEL_MASK;
  regval |= GCR_CLKCTRL_CLKSEL_X32K;
  putreg32(regval, MAX326_GCR_CLKCTRL);
}
#endif

/****************************************************************************
 * Name: max326_set_ovr
 *
 * Description:
 *   Set the operating voltage range selection.  If the OVR setting is
 *   different from the previous setting, then upon return, we will be
 *   running on either the 32.768kHz external oscillator or the 8kHz nano-
 *   ring oscillator with SYCLK prescaler == 0.
 *
 ****************************************************************************/

static void max326_set_ovr(FAR const struct clock_setup_s *clksetup)
{
  uint32_t ovr;
  uint32_t regval;

#ifndef BOARD_HAVE_X32K
  DEBUGASSERT(clksetup->ovr != CLKSRC_32KHZ);
#endif

  /* First check of the OVR setting is being changed */

  regval = getreg32(MAX326_PWRSEQ_LPCTRL);
  ovr    = (regval & PWRSEQ_LPCTRL_OVR_MASK) >> PWRSEQ_LPCTRL_OVR_SHIFT;
  if (ovr != clksetup->ovr)
    {
      /* Switch to the internal LDO for VCORE
       *
       * NOTE: "If using an external supply for VCORE, ensure the external
       * supply is set to the same voltage as the current OVR setting. The
       * external supply must be equal to or greater than the set OVR
       * voltage."
       */

      regval &= ~PWRSEQ_LPCTRL_LDODIS;
      putreg32(regval, MAX326_PWRSEQ_LPCTRL);

      /* Disable any SYSCLK divider */

      regval  = getreg32(MAX326_GCR_CLKCTRL);
      regval &= ~GCR_CLKCTRL_PSC_MASK;
      putreg32(regval, MAX326_GCR_CLKCTRL);

#ifdef BOARD_HAVE_X32K
      /* Select the 32.768kHz External Crystal Oscillator as the SYSOSC
       * clock source (if it was enabled)
       */

      if (clksetup->x32k)
        {
          max326_select_x32k();
        }
      else
#endif
        {
          /* Select the 8kHz Internal Ultra-Low Power Nano-Ring Oscillator
           * as the SYSOSC clock source.
           */

          max326_select_lirc8k();
        }

     /* Wait for SYSOSC to become ready */

      while ((getreg32(MAX326_GCR_CLKCTRL) & GCR_CLKCTRL_CLKRDY) == 0)
        {
        }

      /* Change the OVR setting to the desired range */

      regval  = getreg32(MAX326_PWRSEQ_LPCTRL);
      regval &= ~PWRSEQ_LPCTRL_OVR_MASK;
      regval |= PWRSEQ_LPCTRL_OVR(clksetup->ovr);
      putreg32(regval, MAX326_PWRSEQ_LPCTRL);
    }

  /* Set the Flash Low Voltage Enable according to the OVR setting */

  regval = getreg32(MAX326_FLC_CTRL);
  if (clksetup->ovr < 2)
    {
      regval |= FLC_CTRL_LVE;
    }
  else
    {
      regval &= ~FLC_CTRL_LVE;
    }

  putreg32(regval, MAX326_FLC_CTRL);
}

/****************************************************************************
 * Name: max326_set_clksrc
 *
 * Description:
 *   Select the requested clock source.
 *
 ****************************************************************************/

static void max326_set_clksrc(FAR const struct clock_setup_s *clksetup)
{
  uint32_t regval;
  uint32_t clksrc;

  /* Set the system clock source if it is different from the currently
   * selected clock source.
   */

  regval = getreg32(MAX326_GCR_CLKCTRL);
  clksrc = (regval & GCR_CLKCTRL_CLKSEL_MASK) >> GCR_CLKCTRL_CLKSEL_SHIFT;

  if (clksrc != clksetup->clksrc)
    {
      /* Set the selected clock source */

      switch (clksetup->clksrc)
        {
          case CLKSRC_HFIO:   /* High frequency internal oscillator */
            /* Select the High-Frequency Internal Oscillator (HFIO) as the
             * SYSOSC clock source.
             */

            DEBUGASSERT(clksetup->hfio);
            max326_select_hfio();
            break;

          case CLKSRC_8KHZ:   /* 8kHz Internal Ultra-Low Power Nano-Ring Oscillator */
            /* Select the 8kHz Internal Ultra-Low Power Nano-Ring Oscillator
             * as the SYSOSC clock source.
             */

            max326_select_lirc8k();
            break;

          case CLKSRC_32KHZ:  /* 32.768kHz External Crystal Oscillator */
#ifdef BOARD_HAVE_X32K
            /* Select the 32.768kHz External Crystal Oscillator as the
             * SYSOSC clock source.
             */

            DEBUGASSERT(clksetup->x32k);
            max326_select_x32k();
            break;
#endif

          default:
            DEBUGPANIC();
            return;
        }

     /* Wait for SYSOSC to become ready */

      while ((getreg32(MAX326_GCR_CLKCTRL) & GCR_CLKCTRL_CLKRDY) == 0)
        {
        }
    }

  /* Set the SYSCLK prescaler */

  regval  = getreg32(MAX326_GCR_CLKCTRL);
  regval &= ~GCR_CLKCTRL_PSC_MASK;
  regval |= GCR_CLKCTRL_PSC(clksetup->psc);
  putreg32(regval, MAX326_GCR_CLKCTRL);
}

/****************************************************************************
 * Name: max326_set_fwsdefault
 *
 * Description:
 *   Set the the FLASH wait states to the default value (5)
 *
 ****************************************************************************/

static void max326_set_fwsdefault(void)
{
  uint32_t regval;

  /* Set the number of Flash Wait States to the POR default value of 5. */

  regval  = getreg32(MAX326_GCR_MEMCTRL);
  regval &= ~GCR_MEMCTRL_FWS_MASK;
  regval |= GCR_MEMCTRL_FWS(5);
  putreg32(regval, MAX326_GCR_MEMCTRL);
}

/****************************************************************************
 * Name: max326_set_fws
 *
 * Description:
 *   Set an optimal value for the FLASH wait states.
 *
 ****************************************************************************/

static void max326_set_fws(void)
{
  uint32_t regval;
  uint32_t sysclk;
  uint32_t fws;

  /* Get the SYSCLK frequency */

  sysclk = max326_sysclk_frequency();

  /* Set the FLASH states according to the SYSCLK frequency.
   *
   * This does not quite match the settings in Table 4.2 of the User Guide.
   * that table has 1 or 2 wait states at 24MHz and 2 or 3 wait states at
   * 48MH.  Perhaps there is more to the wait state calculations than raw
   * SYSCLK frequency?
   */

  if (sysclk < 24000000)
    {
      fws = 1;
    }
  else if (sysclk < 48000000)
    {
      fws = 2;
    }
  else if (sysclk < 72000000)
    {
      fws = 3;
    }
  else if (sysclk <= 96000000)
    {
      fws = 4;
    }
  else
    {
      fws = 5;
    }

  regval  = getreg32(MAX326_GCR_MEMCTRL);
  regval &= ~GCR_MEMCTRL_FWS_MASK;
  regval |= GCR_MEMCTRL_FWS(fws);
  putreg32(regval, MAX326_GCR_MEMCTRL);
}

/****************************************************************************
 * Name: max326_periph_reset
 *
 * Description:
 *   Set an optimal value for the FLASH wait states.
 *
 ****************************************************************************/

static void max326_periph_reset(void)
{
  uint32_t regval;

  /* Initiate the peripheral reset */

  regval  = getreg32(MAX326_GCR_RST0);
  regval |= GCR_RST0_PERIPH;
  putreg32(regval, MAX326_GCR_RST0);

  /* Wait for the peripheral reset to complete */

  while ((getreg32(MAX326_GCR_RST0) & GCR_RST0_PERIPH) != 0)
    {
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_clockconfig
 *
 * Description:
 *   Called to initialize the MAX3266xx.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 *****************************************************************************/

void max326_clockconfig(FAR const struct clock_setup_s *clksetup)
{
  /* Set the the FLASH wait states to the default value (5) */

  max326_set_fwsdefault();

#ifdef BOARD_HAVE_X32K
  /* Enable the 32.768 KHz crystal oscillator if it is needed */

  max326_enable_x32k(clksetup);
#endif

  /* Enable the High frequency internal oscillator if it is needed */

  max326_enable_hfio(clksetup);

  /* Set the operating voltage range selection.  If the OVR setting is
   * different from the previous setting, then upon return, we will be
   * running on either the 32.768kHz external oscillator or the 8kHz nano-
   * ring oscillator with SYCLK prescaler == 0.
   */

  max326_set_ovr(clksetup);

  /* Select the requested clock source. */

  max326_set_clksrc(clksetup);

#ifdef BOARD_HAVE_X32K
  /* Disable the 32.768 KHz crystal oscillator if it is not used */

  max326_disable_x32k(clksetup);
#endif

  /* Disable the High frequency internal oscillator if it is not used. */

  max326_disable_hfio(clksetup);

  /* Set an optimal value for the FLASH wait states */

  max326_set_fws();

  /* Perform a peripheral reset */

  max326_periph_reset();

  /* Disable most clocks to peripherals by default to reduce power */

#ifndef CONFIG_MAX326XX_DMA
  max326_dma_disableclk();
#endif
#if !defined(CONFIG_MAX326XX_SPIM0) && !defined(CONFIG_MAX326XX_SPIS0)
  max326_spi0_disableclk();
#endif
#if !defined(CONFIG_MAX326XX_SPIM1) && !defined(CONFIG_MAX326XX_SPIS1)
  max326_spi1_disableclk();
#endif
#ifndef CONFIG_MAX326XX_UART0
  max326_uart0_disableclk();
#endif
#ifndef CONFIG_MAX326XX_UART1
  max326_uart1_disableclk();
#endif
#if !defined(CONFIG_MAX326XX_I2CM0) && !defined(CONFIG_MAX326XX_I2CS0)
  max326_i2c0_disableclk();
#endif
#if !defined(CONFIG_MAX326XX_I2CM1) && !defined(CONFIG_MAX326XX_I2CS1)
  max326_i2c1_disableclk();
#endif
#ifndef CONFIG_MAX326XX_TMR32_0
  max326_tmr0_disableclk();
#endif
#ifndef CONFIG_MAX326XX_TMR32_1
  max326_tmr1_disableclk();
#endif
#ifndef CONFIG_MAX326XX_TMR32_0
  max326_tmr2_disableclk();
#endif
}

/****************************************************************************
 * Name: max326_hfio_frequency
 *
 * Description:
 *   Return the High-Frequency Internal Oscillator (HFIO) frequency.
 *
 *****************************************************************************/

uint32_t max326_hfio_frequency(void)
{
  uint32_t regval;

  /* The HFIO frequency depends of the PWRSEQ_LP_CTRL:OVR setting */

  regval = getreg32(MAX326_PWRSEQ_LPCTRL);
  switch ((regval & PWRSEQ_LPCTRL_OVR_MASK) >> PWRSEQ_LPCTRL_OVR_SHIFT)
    {
      case 0:
        return 24000000; /* Nominally 24MHz */

      case 1:
        return 48000000; /* Nominally 48MHz */

      default:
        DEBUGPANIC();
      case 2:
        return 96000000; /* Nominally 96MHz */
    }
}

/****************************************************************************
 * Name: max326_cpu_frequency
 *
 * Description:
 *   Return the current CPU frequency.
 *
 *****************************************************************************/

uint32_t max326_cpu_frequency(void)
{
  return max326_sysclk_frequency();
}

/****************************************************************************
 * Name: max326_pclk_frequency
 *
 * Description:
 *   Return the current peripheral clock frequency.
 *
 *****************************************************************************/

uint32_t max326_pclk_frequency(void)
{
  /* Fpclk   = Fsysclk / 2 */

  return max326_sysclk_frequency() >> 1;
}
