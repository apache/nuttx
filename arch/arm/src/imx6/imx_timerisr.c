/****************************************************************************
 * arch/arm/src/imx6/imx_timerisr.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
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
#include <time.h>

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "arm_arch.h"
#include "gic.h"
#include "hardware/imx_ccm.h"
#include "hardware/imx_gpt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The Peripheral Clock (ipg_clk) is selected as the GPT clock source.  NOTE
 * that the ipg_clk may be turned off in low power modes, stopping the timer
 * which is probably what you want.
 *
 * REVISIT: Here we assume that the Peripheral Clock is 66MHz.  That is:
 *
 *   PLL528 -> CBCDR: ahb_podf -> CBCDR: ipg_podf ->ipg_clk_root
 *             3-bit timer        3-bit timer
 *             default=4          default=2
 *
 * So, Peripheral Clock Frequency = 528 / 4 / 2 = 66 MHz
 */

#define GPT_CLOCK        66000000
#define GPT_CLKSRC_VALUE GPT_CR_CLKSRC_PERIPHCLK

/* The desired timer interrupt frequency is provided by the definition
 * CLK_TCK (see include/time.h).  CLK_TCK defines the desired number of
 * system clock ticks per second.  That value is a user configurable setting
 * that defaults to 100 (100 ticks per second = 10 MS interval).
 *
 * We should be able to use a prescaler of 1.
 */

#define GPT_PR_VALUE     1

/* Timer counter comparison settings:
 *
 * - OCR3 will interrupt at CLK_TCK ticks/second after the timer counter
 *   has been reset.
 * - OCR2 will interrupt at 2*CLK_TCK ticks/second after the timer counter
 *   has been reset.
 * - OCR2 will interrupt at 3*CLK_TCK ticks/second after the timer counter
 *   has been reset and then will reset the timer, starting the 3 interrupt
 *   sequence again.
 *
 * Using three comparisons virtually eliminates the possibility of timer
 * interrupt overrun.
 */

#define GPT_OCR3_VALUE   ((1 * GPT_CLOCK + (CLK_TCK >> 1)) / CLK_TCK)
#define GPT_OCR2_VALUE   ((2 * GPT_CLOCK + (CLK_TCK >> 1)) / CLK_TCK)
#define GPT_OCR1_VALUE   ((3 * GPT_CLOCK + (CLK_TCK >> 1)) / CLK_TCK)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  imx_output_compare
 *
 * Description:
 *   Handle one pending output compare interrupt.
 *
 ****************************************************************************/

static void imx_output_compare(uint32_t sr, uint32_t of)
{
  /* Check for a pending output compare interrupt */

  if ((sr & of) != 0)
    {
      /* Process timer interrupt event */

      nxsched_process_timer();
    }
}

/****************************************************************************
 * Function:  imx_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int imx_timerisr(int irq, uint32_t *regs, FAR void *arg)
{
  /* Sample the SR (once) */

  uint32_t sr = getreg32(IMX_GPT_SR);

  /* Clear GPT status register */

  putreg32(sr, IMX_GPT_SR);

  /* Process all pending output compare interrupt */

  imx_output_compare(sr, GPT_INT_OF1);
  imx_output_compare(sr, GPT_INT_OF2);
  imx_output_compare(sr, GPT_INT_OF3);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;
  uint32_t cr;

  /* Disable GPT interrupts at the GIC */

  up_disable_irq(IMX_IRQ_GPT);

  /* Make certain that the ipg_clock and ipg_clk_highfreq are enabled for
   * the GPT module.  Here we set BOTH the ipg_clk and ipg_clk_highfreq so
   * that clocking is on in all modes (except STOP).
   */

  regval  = getreg32(IMX_CCM_CCGR1);
  regval &= ~(CCM_CCGR1_CG10_MASK | CCM_CCGR1_CG11_MASK);
  regval |= (CCM_CCGR1_CG10(CCM_CCGR_ALLMODES) |
             CCM_CCGR1_CG11(CCM_CCGR_ALLMODES));
  putreg32(regval, IMX_CCM_CCGR1);

  /* Disable GPT by setting EN=0 in GPT_CR register */

  cr  = getreg32(IMX_GPT_CR);
  cr &= ~GPT_CR_EN;
  putreg32(cr, IMX_GPT_CR);

  /* Disable GPT interrupt register (GPT_IR) */

  putreg32(0, IMX_GPT_IR);

  /* Configure Output Mode to unconnected/ disconnected—Write zeros in OM3,
   * OM2, and OM1 in GPT_CR.
   */

  cr &= ~(GPT_CR_OM1_MASK   | GPT_CR_OM2_MASK   | GPT_CR_OM3_MASK);
  cr |=  (GPT_CR_OM1_DISCON | GPT_CR_OM2_DISCON | GPT_CR_OM3_DISCON);
  putreg32(cr, IMX_GPT_CR);

  /* Disable Input Capture Modes—Write zeros in IM1 and IM2 in GPT_CR */

  cr &= ~(GPT_CR_IM1_MASK     | GPT_CR_IM2_MASK);
  cr |=  (GPT_CR_IM1_DISABLED | GPT_CR_IM2_DISABLED);
  putreg32(cr, IMX_GPT_CR);

  /* Change clock source CLKSRC to the desired value in GPT_CR register */

  cr &= ~GPT_CR_CLKSRC_MASK;
  cr |= GPT_CLKSRC_VALUE;
  putreg32(cr, IMX_GPT_CR);

  /* Assert the SWR bit in GPT_CR register.  The SWR bit is cleared when the
   * reset procedure finishes.  Setting the SWR bit resets all of the
   * registers to their default reset values, except for the CLKSRC, EN,
   * ENMOD, STOPEN, WAITEN, and DBGEN bits in the GPT Control Register.
   */

  putreg32(cr | GPT_CR_SWR, IMX_GPT_CR);

  /* Clear GPT status register */

  putreg32(GPT_INT_ALL, IMX_GPT_SR);

  /* Configure the prescaler and output compare registers */

  putreg32(GPT_OCR1_VALUE, IMX_GPT_OCR1);
  putreg32(GPT_OCR2_VALUE, IMX_GPT_OCR2);
  putreg32(GPT_OCR3_VALUE, IMX_GPT_OCR3);

  putreg32(GPT_PR_VALUE - 1, IMX_GPT_PR);

  /* Configure restart mode.  Interrupts will be received on OC3, then OC2,
   * then OC1 when the counter will be reset to zero and the whole sequence
   * starts again.
   *
   * FFR=0:  Restart mode
   */

  cr &= ~GPT_CR_FFR;
  putreg32(cr, IMX_GPT_CR);

  /* Set ENMOD=1 in GPT_CR register, to bring GPT counter to 0x00000000.  If
   * the ENMOD bit is 1, then the Main Counter and Prescaler Counter values
   * are reset to 0 *after* GPT is enabled (EN=1).
   */

  cr |= GPT_CR_ENMOD;
  putreg32(cr, IMX_GPT_CR);

  /* Enable GPT (EN=1) in GPT_CR register */

  cr |= GPT_CR_EN;
  putreg32(cr, IMX_GPT_CR);

  /* Configure as a (rising) edge-triggered interrupt */

  arm_gic_irq_trigger(IMX_IRQ_GPT, true);

  /* Attach the timer interrupt vector */

  irq_attach(IMX_IRQ_GPT, (xcpt_t)imx_timerisr, NULL);

  /* Enable all three GPT output compare interrupts */

  putreg32(GPT_INT_OF1 | GPT_INT_OF2 | GPT_INT_OF3, IMX_GPT_IR);

  /* And enable the timer interrupt at the GIC */

  up_enable_irq(IMX_IRQ_GPT);
}
