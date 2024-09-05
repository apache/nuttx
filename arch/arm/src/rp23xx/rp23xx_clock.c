/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_clock.c
 *
 * Based upon the software originally developed by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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
#include <debug.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "hardware/address_mapped.h"
#include "hardware/regs/clocks.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/resets.h"
#include "hardware/structs/watchdog.h"

#include "rp23xx_clock.h"
#include "rp23xx_xosc.h"
#include "rp23xx_pll.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t rp23xx_clock_freq[CLK_COUNT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline bool has_glitchless_mux(int clk_index)
{
  return clk_index == clk_sys ||
         clk_index == clk_ref;
}

#if defined(CONFIG_RP23XX_CLK_GPOUT_ENABLE)
static bool rp23xx_clock_configure_gpout(int clk_index,
                                        uint32_t src,
                                        uint32_t div_int,
                                        uint32_t div_frac)
{
  if (clk_index > RP23XX_CLOCKS_NDX_GPOUT3 ||
      clk_index < RP23XX_CLOCKS_NDX_GPOUT0 ||
      (src >> RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT) > 0xa)
    {
      return false;
    }

  putreg32((div_int << RP23XX_CLOCKS_CLK_GPOUT0_DIV_INT_SHIFT) |
            (div_frac & RP23XX_CLOCKS_CLK_GPOUT0_DIV_FRAC_MASK),
           &clocks_hw->clk[clk_index].div);
  putreg32((src << RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_SHIFT) |
            RP23XX_CLOCKS_CLK_GPOUT0_CTRL_ENABLE,
           &clocks_hw->clk[clk_index].ctrl);

  return true;
}
#endif

bool rp23xx_clock_configure(int clk_index,
                            uint32_t src, uint32_t auxsrc,
                            uint32_t src_freq, uint32_t freq)
{
  uint32_t div;

  ASSERT(src_freq >= freq);

  if (freq > src_freq)
    {
      return false;
    }

  /* Div register is 16.16 int.frac divider so multiply by 2^16
   * (left shift by 16)
   */

  div = (uint32_t) (((uint64_t) src_freq << 16) / freq);

  /* If increasing divisor, set divisor before source. Otherwise set source
   * before divisor. This avoids a momentary overspeed when e.g. switching
   * to a faster source and increasing divisor to compensate.
   */

  if (div > clocks_hw->clk[clk_index].div)
    {
      clocks_hw->clk[clk_index].div = div;
    }

  if (has_glitchless_mux(clk_index) &&
      src == CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX)
    {
      /* If switching a glitchless slice (ref or sys) to an aux source,
       * switch away from aux *first* to avoid passing glitches when
       * changing aux mux.
       * Assume (!!!) glitchless source 0 is no faster than the aux source.
       */

      hw_clear_bits(&clocks_hw->clk[clk_index].ctrl,
                    CLOCKS_CLK_REF_CTRL_SRC_BITS);

      while (!(clocks_hw->clk[clk_index].selected & 1u))
        ;
    }
  else
    {
      /* If no glitchless mux, cleanly stop the clock to avoid glitches
       * propagating when changing aux mux. Note it would be a really bad
       * idea to do this on one of the glitchless clocks (clk_sys, clk_ref).
       */

      hw_clear_bits(&clocks_hw->clk[clk_index].ctrl,
                    CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);

      if (rp23xx_clock_freq[clk_index] > 0)
        {
          /* Delay for 3 cycles of the target clock, for ENABLE propagation.
           * Note XOSC_COUNT is not helpful here because XOSC is not
           * necessarily running, nor is timer... so, 3 cycles per loop:
           */

          volatile unsigned int delay_cyc;

          delay_cyc = rp23xx_clock_freq[clk_sys] /
                      rp23xx_clock_freq[clk_index] + 1;

          while (--delay_cyc > 0);
        }
    }

  /* Set aux mux first, and then glitchless mux if this clock has one */

  hw_write_masked(&clocks_hw->clk[clk_index].ctrl, auxsrc<<CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB,
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS);

  if (has_glitchless_mux(clk_index))
    {
      hw_write_masked(&clocks_hw->clk[clk_index].ctrl, src<<CLOCKS_CLK_SYS_CTRL_SRC_LSB,
                      CLOCKS_CLK_REF_CTRL_SRC_BITS);
      while (!(clocks_hw->clk[clk_index].selected & (1u << src)));
    }

  hw_set_bits(&clocks_hw->clk[clk_index].ctrl,
              CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);

  /* Now that the source is configured, we can trust that the user-supplied
   * divisor is a safe value.
   */

  clocks_hw->clk[clk_index].div = div;

  /* Store the configured frequency */

  rp23xx_clock_freq[clk_index] = freq;

  return true;
}

void clocks_init(void)
{
  /* Start tick in watchdog */

  watchdog_hw->ctrl = (BOARD_XOSC_FREQ / MHZ) | WATCHDOG_CTRL_ENABLE_BITS;

  /* Disable resus that may be enabled from previous software */

  clocks_hw->resus.ctrl = 0;

  /* Enable the xosc */

  rp23xx_xosc_init();

  /* Before we touch PLLs, switch sys and ref cleanly away from their
   * aux sources.
   */

  hw_clear_bits(&clocks_hw->clk[clk_sys].ctrl,
                CLOCKS_CLK_SYS_CTRL_SRC_BITS);
  while (clocks_hw->clk[clk_sys].selected != 1);
  hw_clear_bits(&clocks_hw->clk[clk_ref].ctrl,
                CLOCKS_CLK_REF_CTRL_SRC_BITS);
  while (clocks_hw->clk[clk_ref].selected != 1);

  /* Configure PLLs
   *                   REF     FBDIV VCO     POSTDIV
   * PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 5 / 2 = 150MHz
   * PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz
   */

  hw_set_bits(&resets_hw->reset,
              RESETS_RESET_PLL_SYS_BITS | RESETS_RESET_PLL_USB_BITS);
  hw_clear_bits(&resets_hw->reset,
                RESETS_RESET_PLL_SYS_BITS | RESETS_RESET_PLL_USB_BITS);
  while (~resets_hw->reset_done &
         (RESETS_RESET_PLL_SYS_BITS | RESETS_RESET_PLL_USB_BITS));

  rp23xx_pll_init(pll_sys_hw, 1, 1500 * MHZ, 5, 2);
  rp23xx_pll_init(pll_usb_hw, 1, 480 * MHZ, 5, 2);

  /* Configure clocks */

  /* CLK_REF = XOSC (12MHz) / 1 = 12MHz */

  rp23xx_clock_configure(clk_ref,
                         CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                         0,
                         BOARD_XOSC_FREQ,
                         BOARD_REF_FREQ);

  /* CLK SYS = PLL SYS (125MHz) / 1 = 125MHz */

  rp23xx_clock_configure(clk_sys,
                         CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                         CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                         BOARD_PLL_SYS_FREQ,
                         BOARD_SYS_FREQ);

  /* CLK USB = PLL USB (48MHz) / 1 = 48MHz */

  rp23xx_clock_configure(clk_usb,
                         0,
                         CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                         BOARD_PLL_USB_FREQ,
                         BOARD_USB_FREQ);

  /* CLK ADC = PLL USB (48MHZ) / 1 = 48MHz */

  rp23xx_clock_configure(clk_adc,
                         0,
                         CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                         BOARD_PLL_USB_FREQ,
                         BOARD_ADC_FREQ);

  /* CLK PERI = clk_sys. */

  rp23xx_clock_configure(clk_peri,
                         0,
                         CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                         BOARD_SYS_FREQ,
                         BOARD_PERI_FREQ);

  /* CLK HSTX = clk_sys. */

  rp23xx_clock_configure(clk_hstx,
                         0,
                         CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLK_SYS,
                         BOARD_SYS_FREQ,
                         BOARD_PERI_FREQ);

#if defined(CONFIG_RP23XX_CLK_GPOUT_ENABLE)
  uint32_t src;

  #if defined(CONFIG_RP23XX_CLK_GPOUT0)
    #if defined(CONFIG_RP23XX_CLK_GPOUT0_SRC_REF)
      src = RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_REF;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT0_SRC_SYS)
      src = RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_SYS;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT0_SRC_USB)
      src = RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_USB;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT0_SRC_ADC)
      src = RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_ADC;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT0_SRC_RTC)
      src = RP23XX_CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_CLK_RTC;
    #else
      src = 0;
    #endif
    rp23xx_clock_configure_gpout(RP23XX_CLOCKS_NDX_GPOUT0,
                                 src,
                                 CONFIG_RP23XX_CLK_GPOUT0_DIVINT,
                                 CONFIG_RP23XX_CLK_GPOUT0_DIVFRAC);
  #endif

  #if defined(CONFIG_RP23XX_CLK_GPOUT1)
    #if defined(CONFIG_RP23XX_CLK_GPOUT1_SRC_REF)
      src = RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_REF;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT1_SRC_SYS)
      src = RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_SYS;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT1_SRC_USB)
      src = RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_USB;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT1_SRC_ADC)
      src = RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_ADC;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT1_SRC_RTC)
      src = RP23XX_CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_CLK_RTC;
    #else
      src = 0;
    #endif
    rp23xx_clock_configure_gpout(RP23XX_CLOCKS_NDX_GPOUT1,
                                 src,
                                 CONFIG_RP23XX_CLK_GPOUT1_DIVINT,
                                 CONFIG_RP23XX_CLK_GPOUT1_DIVFRAC);
  #endif

  #if defined(CONFIG_RP23XX_CLK_GPOUT2)
    #if defined(CONFIG_RP23XX_CLK_GPOUT2_SRC_REF)
      src = RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_REF;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT2_SRC_SYS)
      src = RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_SYS;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT2_SRC_USB)
      src = RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_USB;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT2_SRC_ADC)
      src = RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_ADC;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT2_SRC_RTC)
      src = RP23XX_CLOCKS_CLK_GPOUT2_CTRL_AUXSRC_CLK_RTC;
    #else
      src = 0;
    #endif
    rp23xx_clock_configure_gpout(RP23XX_CLOCKS_NDX_GPOUT2,
                                 src,
                                 CONFIG_RP23XX_CLK_GPOUT2_DIVINT,
                                 CONFIG_RP23XX_CLK_GPOUT2_DIVFRAC);
  #endif

  #if defined(CONFIG_RP23XX_CLK_GPOUT3)
    #if defined(CONFIG_RP23XX_CLK_GPOUT3_SRC_REF)
      src = RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_REF;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT3_SRC_SYS)
      src = RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_SYS;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT3_SRC_USB)
      src = RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_USB;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT3_SRC_ADC)
      src = RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_ADC;
    #elif defined(CONFIG_RP23XX_CLK_GPOUT3_SRC_RTC)
      src = RP23XX_CLOCKS_CLK_GPOUT3_CTRL_AUXSRC_CLK_RTC;
    #else
      src = 0;
    #endif
    rp23xx_clock_configure_gpout(RP23XX_CLOCKS_NDX_GPOUT3,
                                 src,
                                 CONFIG_RP23XX_CLK_GPOUT3_DIVINT,
                                 CONFIG_RP23XX_CLK_GPOUT3_DIVFRAC);
  #endif

#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rp23xx_clockconfig(void)
{
  /* Reset all peripherals to put system into a known state,
   * - except for QSPI pads and the XIP IO bank, as this is fatal if running
   *   from flash
   * - and the PLLs, as this is fatal if clock muxing has not been reset on
   *   this boot
   */

  hw_set_bits(&resets_hw->reset,
              RESETS_RESET_BITS & ~(RESETS_RESET_IO_QSPI_BITS |
              RESETS_RESET_PADS_QSPI_BITS |
              RESETS_RESET_PWM_BITS |
              RESETS_RESET_PLL_USB_BITS |
              RESETS_RESET_JTAG_BITS |
              RESETS_RESET_PLL_SYS_BITS));

  /* Remove reset from peripherals which are clocked only by clk_sys and
   * clk_ref. Other peripherals stay in reset until we've configured clocks.
   */

  hw_clear_bits(&resets_hw->reset,
                RESETS_RESET_BITS & ~(RESETS_RESET_ADC_BITS |
                RESETS_RESET_PWM_BITS |
                RESETS_RESET_HSTX_BITS |
                RESETS_RESET_SPI0_BITS |
                RESETS_RESET_SPI1_BITS |
                RESETS_RESET_UART0_BITS |
                RESETS_RESET_UART1_BITS |
                RESETS_RESET_USBCTRL_BITS));

  while (~resets_hw->reset_done &
         (RESETS_RESET_BITS & ~(RESETS_RESET_ADC_BITS |
                                RESETS_RESET_PWM_BITS |
                                RESETS_RESET_HSTX_BITS |
                                RESETS_RESET_SPI0_BITS |
                                RESETS_RESET_SPI1_BITS |
                                RESETS_RESET_UART0_BITS |
                                RESETS_RESET_UART1_BITS |
                                RESETS_RESET_USBCTRL_BITS)))
    ;

  /* After calling preinit we have enough runtime to do the exciting maths
   * in clocks_init
   */

  clocks_init();

  /* Peripheral clocks should now all be running */

  hw_clear_bits(&resets_hw->reset, RESETS_RESET_BITS);
  while (~resets_hw->reset_done & RESETS_RESET_BITS)
    ;
}
