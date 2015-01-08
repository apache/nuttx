/****************************************************************************
 * arch/arm/src/tiva/tiva_timer.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "chip/tiva_syscontrol.h"
#include "chip/tiva_timer.h"

#include "tiva_enableclks.h"
#include "tiva_enablepwr.h"
#include "tiva_periphrdy.h"
#include "tiva_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure retains the fixed, well-known attrutes of a GPTM module */

struct tiva_gptmattr_s
{
  uintptr_t base;  /* Register base address */
  int irq[2];      /* Timer A/B interrupt numbers */
};

/* This structure represents the state of a GPTM module */

struct tiva_gptmstate_s
{
  /* Constant time attributes and configuration */

  const struct tiva_gptmattr_s   *attr;
  const struct tiva_gptmconfig_s *config;

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  /* Register level debug */

   bool wrlast;                /* Last was a write */
   uintptr_t addrlast;         /* Last address */
   uint32_t vallast;           /* Last value */
   int ntimes;                 /* Number of times */
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_TIVA_TIMER0
static const struct tiva_gptmattr_s g_gptm0_attr =
{
  .base = TIVA_TIMER0_BASE,
  .irq  = { TIVA_IRQ_TIMER0A, TIVA_IRQ_TIMER0B },
};

static struct tiva_gptmstate_s g_gptm0_state;
#endif

#ifdef CONFIG_TIVA_TIMER1
static const struct tiva_gptmattr_s g_gptm1_attr =
{
  .base = TIVA_TIMER1_BASE,
  .irq  = { TIVA_IRQ_TIMER1A, TIVA_IRQ_TIMER1B },
};

static struct tiva_gptmstate_s g_gptm1_state;
#endif

#ifdef CONFIG_TIVA_TIMER2
static const struct tiva_gptmattr_s g_gptm2_attr =
{
  .base = TIVA_TIMER2_BASE,
  .irq  = { TIVA_IRQ_TIMER2A, TIVA_IRQ_TIMER2B },
};

static struct tiva_gptmstate_s g_gptm2_state;
#endif

#ifdef CONFIG_TIVA_TIMER3
static const struct tiva_gptmattr_s g_gptm3_attr =
{
  .base = TIVA_TIMER3_BASE,
  .irq  = { TIVA_IRQ_TIMER3A, TIVA_IRQ_TIMER3B },
};

static struct tiva_gptmstate_s g_gptm3_state;
#endif

#ifdef CONFIG_TIVA_TIMER4
static const struct tiva_gptmattr_s g_gptm4_attr =
{
  .base = TIVA_TIMER4_BASE,
  .irq  = { TIVA_IRQ_TIMER4A, TIVA_IRQ_TIMER4B },
};

static struct tiva_gptmstate_s g_gptm4_state;
#endif

#ifdef CONFIG_TIVA_TIMER5
static const struct tiva_gptmattr_s g_gptm5_attr =
{
  .base = TIVA_TIMER5_BASE,
  .irq  = { TIVA_IRQ_TIMER5A, TIVA_IRQ_TIMER5B },
};

static struct tiva_gptmstate_s g_gptm5_state;
#endif

#ifdef CONFIG_TIVA_TIMER6
static const struct tiva_gptmattr_s g_gptm6_attr =
{
  .base = TIVA_TIMER6_BASE,
  .irq  = { TIVA_IRQ_TIMER6A, TIVA_IRQ_TIMER6B },
};

static struct tiva_gptmstate_s g_gptm6_state;
#endif

#ifdef CONFIG_TIVA_TIMER7
static const struct tiva_gptmattr_s g_gptm7_attr =
{
  .base = TIVA_TIMER7_BASE,
  .irq  = { TIVA_IRQ_TIMER7A, TIVA_IRQ_TIMER7B },
};

static struct tiva_gptmstate_s g_gptm7_state;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: tiva_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval  - The value to be written
 *   regaddr - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ************************************************************************************/

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
static bool tiva_timer_checkreg(struct tiva_gptmstate_s *priv, bool wr,
                                uint32_t regval, uintptr_t regaddr)
{
  if (wr      == priv->wrlast &&   /* Same kind of access? */
      regval  == priv->vallast &&  /* Same value? */
      regaddr == priv->addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          lldbg("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast   = wr;
      priv->vallast  = regval;
      priv->addrlast = regaddr;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: tiva_getreg
 *
 * Description:
 *   Read one 32-bit GPTM register
 *
 ****************************************************************************/

static uint32_t tiva_getreg(struct tiva_gptmstate_s *priv, unsigned int offset)
{
  uintptr_t regaddr = priv->attr->base + offset;
  uint32_t regval =  getreg32(regaddr);

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  if (tiva_timer_checkreg(priv, false, regval, regaddr))
    {
      lldbg("%08x->%08x\n", regaddr, regval);
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: tiva_putreg
 *
 * Description:
 *   Write one 32-bit GPTM register
 *
 ****************************************************************************/

static void tiva_putreg(struct tiva_gptmstate_s *priv, unsigned int offset,
                        uint32_t regval)
{
  uintptr_t regaddr = priv->attr->base + offset;

#ifdef CONFIG_TIVA_TIMER_REGDEBUG
  if (tiva_timer_checkreg(priv, true, regval, regaddr))
    {
       lldbg("%08x<-%08x\n", regaddr, regval);
    }
#endif

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: tiva_oneshot_periodic_mode32
 *
 * Description:
 *   Configure a 32-bit timer to operate in one-short or periodic mode
 *
 ****************************************************************************/

static int tiva_oneshot_periodic_mode32(struct tiva_gptmstate_s *priv,
                                        const struct tiva_timer32config_s *timer)
{
  uint32_t regval;

  /* The GPTM is configured for One-Shot and Periodic modes by the following
   * sequence:
   *
   * 1. Ensure the timer is disabled (the TAEN bit in the GPTMCTL register
   *    is cleared) before making any changes.
   *
   *    NOTE: The TAEN bit was cleared when the timer was reset prior to
   *    calling this function.
   *
   * 2. Write the GPTM Configuration Register (GPTMCFG) to select 32-bit
   *    operation.
   */

   tiva_putreg(priv, TIVA_TIMER_CFG_OFFSET, TIMER_CFG_CFG_32);

  /* 3. Configure the TAMR field in the GPTM Timer n Mode Register
   *   (GPTMTAMR):
   *
   *    a. Write a value of 0x1 for One-Shot mode.
   *    b. Write a value of 0x2 for Periodic mode.
   *
   *    When Timer A and TimerB are concatenated, the GPTMTBMR register is
   *    ignored and GPTMTAMR controls the modes for both Timer A and Timer B
   */

  regval  = tiva_getreg(priv, TIVA_TIMER_TAMR_OFFSET);
  regval &= ~TIMER_TnMR_TnMR_MASK;

  if (priv->config->mode == TIMER32_MODE_ONESHOT)
    {
      regval |= TIMER_TnMR_TnMR_ONESHOT;
    }
  else /* if (priv->config->mode == TIMER32_MODE_PERIODIC) */
    {
      regval |= TIMER_TnMR_TnMR_PERIODIC;
    }

  tiva_putreg(priv, TIVA_TIMER_TAMR_OFFSET, regval);

  /* 4. Optionally configure the TnSNAPS, TnWOT, TnMTE, and TnCDIR bits in
   *    the GPTMTnMR register to select whether to capture the value of the
   *    free-running timer at time-out, use an external trigger to start
   *    counting, configure an additional trigger or interrupt, and count up
   *    or down. In addition, if using CCP pins, the TCACT field can be
   *    programmed to configure the compare action.
   */
#warning Missing logic

  /* 5. Load the start value into the GPTM Timer n Interval Load Register
   *    (GPTMTAILR).
   *
   *   When a GPTM is configured to one of the 32-bit modes, GPTMTAILR
   *   appears as a 32-bit register; the upper 16-bits correspond to bits
   *   15:0 of the GPTM Timer B Interval Load (GPTMTBILR) register.
   *   Writes to GPTMTBILR are ignored.
   */

  tiva_putreg(priv, TIVA_TIMER_TAILR_OFFSET, timer->u.periodic.interval);

  /* 6. If interrupts are required, set the appropriate bits in the GPTM
   *    Interrupt Mask Register (GPTMIMR).
   */
#warning Missing Logic

  /* 7. Set the TAEN bit in the GPTMCTL register to enable the timer and
   *    start counting.
   * 8. Poll the GPTMRIS register or wait for the interrupt to be generated
   *    (if enabled). In both cases, the status flags are cleared by writing
   *    a 1 to the appropriate bit of the GPTM Interrupt Clear Register
   *   (GPTMICR).
   *
   * NOTE: This timer is started until tiva_gptm_enable() is called.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: tiva_oneshot_periodic_mode16
 *
 * Description:
 *   Configure 16-bit timer A/B to operate in one-short or periodic mode
 *
 ****************************************************************************/

static int tiva_oneshot_periodic_mode16(struct tiva_gptmstate_s *priv,
                                       const struct tiva_timer16config_s *timer,
                                       int tmndx)
{
  unsigned int regoffset;
  uint32_t regval;

  /* The GPTM is configured for One-Shot and Periodic modes by the following
   * sequence:
   *
   * 1. Ensure the timer is disabled (the TnEN bit in the GPTMCTL register
   *    is cleared) before making any changes.
   *
   *    NOTE: The TnEN bit was cleared when the timer was reset prior to
   *    calling this function.
   *
   * 2. Write the GPTM Configuration Register (GPTMCFG) to select 16-bit
   *    operation.
   *
   *    NOTE: 16-bit mode of operation was already selected in
   *    tiva_gptm_configure() before this function was called.
   *
   * 3. Configure the TnMR field in the GPTM Timer n Mode Register
   *   (GPTMTnMR):
   *    a. Write a value of 0x1 for One-Shot mode.
   *    b. Write a value of 0x2 for Periodic mode.
   */

  regoffset = tmndx ? TIVA_TIMER_TBMR_OFFSET : TIVA_TIMER_TAMR_OFFSET;
  regval    = tiva_getreg(priv, regoffset);
  regval   &= ~TIMER_TnMR_TnMR_MASK;

  if (timer->mode == TIMER16_MODE_ONESHOT)
    {
      regval |= TIMER_TnMR_TnMR_ONESHOT;
    }
  else /* if (timer->mode == TIMER16_MODE_PERIODIC) */
    {
      regval |= TIMER_TnMR_TnMR_PERIODIC;
    }

  tiva_putreg(priv, regoffset, regval);

  /* 4. Optionally configure the TnSNAPS, TnWOT, TnMTE, and TnCDIR bits in
   *    the GPTMTnMR register to select whether to capture the value of the
   *    free-running timer at time-out, use an external trigger to start
   *    counting, configure an additional trigger or interrupt, and count up
   *    or down. In addition, if using CCP pins, the TCACT field can be
   *    programmed to configure the compare action.
   */
#warning Missing logic

  /* 5. Load the start value into the GPTM Timer n Interval Load Register
   *    (GPTMTnILR).
   */

  regoffset = tmndx ? TIVA_TIMER_TBILR_OFFSET : TIVA_TIMER_TAILR_OFFSET;
  tiva_putreg(priv, regoffset, timer->u.periodic.interval);

  /* 6. If interrupts are required, set the appropriate bits in the GPTM
   *    Interrupt Mask Register (GPTMIMR).
   */
#warning Missing Logic

  /* 7. Set the TnEN bit in the GPTMCTL register to enable the timer and
   *    start counting.
   * 8. Poll the GPTMRIS register or wait for the interrupt to be generated
   *    (if enabled). In both cases, the status flags are cleared by writing
   *    a 1 to the appropriate bit of the GPTM Interrupt Clear Register
   *   (GPTMICR).
   *
   * NOTE: This timer is started until tiva_gptm_enable() is called.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: tiva_rtc_mode32
 *
 * Description:
 *   Configure a 32-bit timer to operate in RTC mode
 *
 ****************************************************************************/

static int tiva_rtc_mode32(struct tiva_gptmstate_s *priv,
                           const struct tiva_timer32config_s *timer)
{
  /* To use the RTC mode, the timer must have a 32.768-KHz input signal on
   * an even CCP input. To enable the RTC feature, follow these steps:
   *
   * 1. Ensure the timer is disabled (the TAEN bit is cleared) before making
   *    any changes.
   *
   *    NOTE: The TAEN bit was cleared when the timer was reset prior to
   *    calling this function.
   *
   * 2. If the timer has been operating in a different mode prior to this,
   *    clear any residual set bits in the GPTM Timer n Mode (GPTMTnMR)
   *    register before reconfiguring.
   *
   *    NOTE: The TAMR and TBMR registers were cleared when the timer
   *    was reset prior to calling this function.
   *
   * 3. Write the GPTM Configuration Register (GPTMCFG) with a to select
   *    the 32-bit RTC mode.
   */

   tiva_putreg(priv, TIVA_TIMER_CFG_OFFSET, TIMER_CFG_CFG_RTC);

  /* 4. Write the match value to the GPTM Timer n Match Register
   *    (GPTMTnMATCHR).
   */

  /* 5. Set/clear the RTCEN and TnSTALL bit in the GPTM Control Register
   *    (GPTMCTL) as needed.
   */

  /* 6. If interrupts are required, set the RTCIM bit in the GPTM Interrupt
   *    Mask Register (GPTMIMR).
   */

  /* 7. Set the TAEN bit in the GPTMCTL register to enable the timer and
   *    start counting.
   *
   * When the timer count equals the value in the GPTMTnMATCHR register,
   * the GPTM asserts the RTCRIS bit in the GPTMRIS register and continues
   * counting until Timer A is disabled or a hardware reset. The interrupt
   * is cleared by writing the RTCCINT bit in the GPTMICR register. Note
   * that if the GPTMTnILR register is loaded with a new value, the timer
   * begins counting at this new value and continues until it reaches
   * 0xFFFF.FFFF, at which point it rolls over.
   *
   * NOTE: The timer will not be enabled until tiva_gptm_enable() is called.
   */
#warning Missing Logic
  return -ENOSYS;
}

/****************************************************************************
 * Name: tiva_input_edgecount_mode16
 *
 * Description:
 *   Configure 16-bit timer A/B to operate in Input Edge-Count mode
 *
 ****************************************************************************/

static int tiva_input_edgecount_mode16(struct tiva_gptmstate_s *priv,
                                       const struct tiva_timer16config_s *timer,
                                       int tmndx)
{
  /* A timer is configured to Input Edge-Count mode by the following sequence:
   *
   * 1. Ensure the timer is disabled (the TnEN bit is cleared) before making
   *    any changes.
   *
   *    NOTE: The TnEN bit was cleared when the timer was reset prior to
   *    calling this function.
   *
   * 2. Write the GPTM Configuration Register (GPTMCFG) to select 16-bit
   *    operation.
   *
   *    NOTE: 16-bit mode of operation was already selected in
   *    tiva_gptm_configure() before this function was called.
   *
   * 3. In the GPTM Timer Mode (GPTMTnMR) register, write the TnCMR field to
   *    0x0 and the TnMR field to 0x3.
   */

  /* 4. Configure the type of event(s) that the timer captures by writing
   *    the TnEVENT field of the GPTM Control (GPTMCTL) register.
   */

  /* 5. Program registers according to count direction:
   *   - In down-count mode, the GPTMTnMATCHR and GPTMTnPMR registers are
   *    configured so that the difference between the value in the GPTMTnILR
   *    and GPTMTnPR registers and the GPTMTnMATCHR and GPTMTnPMR registers
   *    equals the number of edge events that must be counted.
   *   - In up-count mode, the timer counts from 0x0 to the value in the
   *    GPTMTnMATCHR and GPTMTnPMR registers. Note that when executing an
   *    up-count, the value of the GPTMTnPR and GPTMTnILR must be greater
   *    than the value of GPTMTnPMR and GPTMTnMATCHR.
   */

  /* 6. If interrupts are required, set the CnMIM bit in the GPTM Interrupt
   *    Mask (GPTMIMR) register.
   */
#warning Missing Logic

  /* 7. Set the TnEN bit in the GPTMCTL register to enable the timer and
   *     begin waiting for edge events.
   * 8. Poll the CnMRIS bit in the GPTMRIS register or wait for the
   *    interrupt to be generated (if enabled). In both cases, the status
   *    flags are cleared by writing a 1 to the CnMCINT bit of the GPTM
   *    Interrupt Clear (GPTMICR) register.
   *
   * When counting down in Input Edge-Count Mode, the timer stops after the
   * programmed number of edge events has been detected. To re-enable the
   * timer, ensure that the TnEN bit is cleared and repeat steps 4 through 8.
   *
   * NOTE: This timer is started until tiva_gptm_enable() is called.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: tiva_input_time_mode16
 *
 * Description:
 *   Configure 16-bit timer A/B to operate in Input Time mode
 *
 ****************************************************************************/

static int tiva_input_time_mode16(struct tiva_gptmstate_s *priv,
                                  const struct tiva_timer16config_s *timer,
                                  int tmndx)
{
  /* A timer is configured to Input Edge Time mode by the following sequence:
   *
   * 1. Ensure the timer is disabled (the TnEN bit is cleared) before making
   *    any changes.
   *
   *    NOTE: The TnEN bit was cleared when the timer was reset prior to
   *    calling this function.
   *
   * 2. Write the GPTM Configuration Register (GPTMCFG) to select 16-bit
   *    operation.
   *
   *    NOTE: 16-bit mode of operation was already selected in
   *    tiva_gptm_configure() before this function was called.
   *
   * 3. In the GPTM Timer Mode (GPTMTnMR) register, write the TnCMR field to
   *    0x1 and the TnMR field to 0x3 and select a count direction by
   *    programming the TnCDIR bit.
   */

  /* 4. Configure the type of event that the timer captures by writing the
   *    TnEVENT field of the GPTM Control (GPTMCTL) register.
   */

  /* 5. If a prescaler is to be used, write the prescale value to the GPTM
   *    Timer n Prescale Register (GPTMTnPR).
   */

  /* 6. Load the timer start value into the GPTM Timer n Interval Load
   *    (GPTMTnILR) register.
   */

  /* 7. If interrupts are required, set the CnEIM bit in the GPTM Interrupt
   *    Mask (GPTMIMR) register.
   */
#warning Missing Logic

  /* 8. Set the TnEN bit in the GPTM Control (GPTMCTL) register to enable the
   *    timer and start counting.
   * 9. Poll the CnERIS bit in the GPTMRIS register or wait for the interrupt
   *    to be generated (if enabled). In both cases, the status flags are
   *    cleared by writing a 1 to the CnECINT bit of the GPTM Interrupt
   *    Clear (GPTMICR) register. The time at which the event happened can
   *    be obtained by reading the GPTM Timer n (GPTMTnR) register.
   *
   * In Input Edge Timing mode, the timer continues running after an edge
   * event has been detected, but the timer interval can be changed at any
   * time by writing the GPTMTnILR register and clearing the TnILD bit in
   * the GPTMTnMR register. The change takes effect at the next cycle after
   * the write.
   *
   * NOTE: This timer is started until tiva_gptm_enable() is called.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: tiva_pwm_mode16
 *
 * Description:
 *   Configure 16-bit timer A/B to operate in PWM mode
 *
 ****************************************************************************/

static int tiva_pwm_mode16(struct tiva_gptmstate_s *priv,
                           const struct tiva_timer16config_s *timer,
                           int tmndx)
{
  /* A timer is configured to PWM mode using the following sequence:
   *
   * 1. Ensure the timer is disabled (the TnEN bit is cleared) before making
   *    any changes.
   *
   *    NOTE: The TnEN bit was cleared when the timer was reset prior to
   *    calling this function.
   *
   * 2. Write the GPTM Configuration Register (GPTMCFG) to select 16-bit
   *    operation.
   *
   *    NOTE: 16-bit mode of operation was already selected in
   *    tiva_gptm_configure() before this function was called.
   *
   * 3. In the GPTM Timer Mode (GPTMTnMR) register, set the TnAMS bit to
   *    0x1, the TnCMR bit to 0x0, and the TnMR field to 0x2.
   */

  /* 4. Configure the output state of the PWM signal (whether or not it is
   *    inverted) in the TnPWML field of the GPTM Control (GPTMCTL) register.
   */

  /* 5. If a prescaler is to be used, write the prescale value to the GPTM
   *    Timer n Prescale Register (GPTMTnPR).
   */

  /* 6. If PWM interrupts are used, configure the interrupt condition in the
   *    TnEVENT field in the GPTMCTL register and enable the interrupts by
   *    setting the TnPWMIE bit in the GPTMTnMR register. Note that edge
   *    detect interrupt behavior is reversed when the PWM output is
   *    inverted.
   */

  /* 7. Load the timer start value into the GPTM Timer n Interval Load
   *    (GPTMTnILR) register.
   */

  /* 8. Load the GPTM Timer n Match (GPTMTnMATCHR) register with the match
   *    value.
   */
#warning Missing Logic

  /* 9. Set the TnEN bit in the GPTM Control (GPTMCTL) register to enable
   *    the timer and begin generation of the output PWM signal.
   *
   * In PWM Time mode, the timer continues running after the PWM signal has
   * been generated. The PWM period can be adjusted at any time by writing
   * the GPTMTnILR register, and the change takes effect at the next cycle
   * after the write.
   *
   * NOTE: This timer is started until tiva_gptm_enable() is called.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: tiva_timer16_configure
 *
 * Description:
 *   Configure the 32-bit timer to operate in the provided mode.
 *
 ****************************************************************************/

static int tiva_timer32_configure(struct tiva_gptmstate_s *priv,
                                  const struct tiva_timer32config_s *timer)
{
  switch (priv->config->mode)
    {
    case TIMER32_MODE_ONESHOT:       /* 32-bit programmable one-shot timer */
    case TIMER32_MODE_PERIODIC:      /* 32-bit programmable periodic timer */
      return tiva_oneshot_periodic_mode32(priv, timer);

    case TIMER32_MODE_RTC:           /* 32-bit RTC with external 32.768-KHz
                                      * input */
      return tiva_rtc_mode32(priv, timer);

    default:
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: tiva_timer16_configure
 *
 * Description:
 *   Configure 16-bit timer A or B to operate in the provided mode.
 *
 ****************************************************************************/

static int tiva_timer16_configure(struct tiva_gptmstate_s *priv,
                                  const struct tiva_timer16config_s *timer,
                                  int tmndx)
{
  /* Configure the timer per the selected mode */

  switch (timer->mode)
    {
    case TIMER16_MODE_NONE:
      return OK;

    case TIMER16_MODE_ONESHOT:       /* 16-bit programmable one-shot timer */
    case TIMER16_MODE_PERIODIC:      /* 16-bit programmable periodic timer */
      return tiva_oneshot_periodic_mode16(priv, timer, tmndx);

    case TIMER16_MODE_COUNT_CAPTURE: /* 16-bit input-edge count-capture
                                      * mode w/8-bit prescaler */
      return tiva_input_edgecount_mode16(priv, timer, tmndx);

    case TIMER16_MODE_TIME_CAPTURE:  /* 16-bit input-edge time-capture
                                      * mode w/8-bit prescaler */
      return tiva_input_time_mode16(priv, timer, tmndx);

    case TIMER16_MODE_PWM:           /* 16-bit PWM output mode w/8-bit
                                      * prescaler */
      return tiva_pwm_mode16(priv, timer, tmndx);

    default:
      return -EINVAL;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_gptm_configure
 *
 * Description:
 *   Configure a general purpose timer module to operate in the provided
 *   modes.
 *
 ****************************************************************************/

TIMER_HANDLE tiva_gptm_configure(const struct tiva_gptmconfig_s *gptm)
{
  static const struct tiva_gptmattr_s *attr;
  static struct tiva_gptmstate_s *priv;
  uint32_t regval;
  int ret;

  DEBUGASSERT(gptm);

  /* Select the GPTM module. */

  switch (gptm->gptm)
    {
#ifdef CONFIG_TIVA_TIMER0
    case 0:
      /* Enable GPTM0 clocking and power */

      
      attr = &g_gptm0_attr;
      priv = &g_gptm0_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER1
    case 1:
      attr = &g_gptm1_attr;
      priv = &g_gptm1_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER2
    case 2:
      attr = &g_gptm2_attr;
      priv = &g_gptm2_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER3
    case 3:
      attr = &g_gptm3_attr;
      priv = &g_gptm3_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER4
    case 4:
      attr = &g_gptm4_attr;
      priv = &g_gptm4_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER5
    case 5:
      attr = &g_gptm5_attr;
      priv = &g_gptm5_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER6
    case 6:
      attr = &g_gptm6_attr;
      priv = &g_gptm6_state;
      break;
#endif

#ifdef CONFIG_TIVA_TIMER7
    case 7:
      attr = &g_gptm7_attr;
      priv = &g_gptm7_state;
      break;
#endif

    default:
      return (TIMER_HANDLE)NULL;
    }

  /* Initialize the state structure */

  memset(priv, 0, sizeof(struct tiva_gptmstate_s));
  priv->attr   = attr;
  priv->config = gptm;

  /* Enable power and clocking to the GPTM module
   *
   * - Enable Power (TM4C129 family only):  Applies power (only) to the GPTM
   *   module.  This is not an essential step since enabling clocking
   *   will also apply power.  The only significance is that the GPTM state
   *   will be retained if the GPTM clocking is subsequently disabled.
   * - Enable Clocking (All families):  Applies both power and clocking to
   *   the GPTM module, bringing it a fully functional state.
   */

  tiva_gptm_enableclk(gptm->gptm);
  tiva_gptm_enablepwr(gptm->gptm);

  /* Wait for the gptm to become ready before modifying its registers */

  while (!tiva_gpio_periphrdy(gptm->gptm));

  /* Reset the time to be certain that it is in the disabled state */

   regval  = tiva_getreg(priv, TIVA_SYSCON_SRTIMER);
   regval |= SYSCON_SRTIMER(gptm->gptm);
   tiva_putreg(priv, TIVA_SYSCON_SRTIMER, regval);

   regval &= ~SYSCON_SRTIMER(gptm->gptm);
   tiva_putreg(priv, TIVA_SYSCON_SRTIMER, regval);

  /* Wait for the reset to complete */

  while (!tiva_emac_periphrdy());
  up_udelay(250);

  /* Select the alternal timer clock source is so reuested.  The general
   * purpose timer has the capability of being clocked by either the system
   * clock or an alternate clock source. By setting the ALTCLK bit in the
   * GPTM Clock Configuration (GPTMCC) register, software can selects an
   * alternate clock source as programmed in the Alternate Clock
   * Configuration (ALTCLKCFG) register in the System Control Module. The
   * alternate clock source options available are PIOSC, RTCOSC and LFIOSC.
   *
   * NOTE: The actual alternate clock source selection is a global property
   * and cannot be configure on a timer-by-timer basis here.  That selection
   * must be done by common logic early in the initialization sequence.
   *
   * In any case, the caller must provide us with the correct source
   * frequency in gptm->frequency field.
   */

  if (gptm->alternate)
    {
      /* Enable the alternate clock source */

      regval  = tiva_getreg(priv, TIVA_TIMER_CC_OFFSET);
      regval |= TIMER_CC_ALTCLK;
      tiva_putreg(priv, TIVA_TIMER_CC_OFFSET, regval);
    }

  /* Then [re-]configure the timer into the new configuration */

  if (gptm->mode != TIMER16_MODE)
    {
      const struct tiva_gptm32config_s *gptm32 =
        (const struct tiva_gptm32config_s *)gptm;

      /* Configure the 32-bit timer */

      ret = tiva_timer32_configure(priv, &gptm32->config);
    }
  else
    {
      const struct tiva_gptm16config_s *gptm16 =
        (const struct tiva_gptm16config_s *)gptm;

      /* Write the GPTM Configuration Register (GPTMCFG) to select 16-bit
       * operation.
       */

      tiva_putreg(priv, TIVA_TIMER_CFG_OFFSET, TIMER_CFG_CFG_16);

      /* Configure both 16-bit timers */

      ret = tiva_timer16_configure(priv, &gptm16->config[TIMER_A], TIMER_A);
      if (ret == OK)
        {
          ret = tiva_timer16_configure(priv, &gptm16->config[TIMER_B],
                                       TIMER_B);
        }
    }

  /* Return the timer handler if successfully configured */

  return ret < 0 ? (TIMER_HANDLE)NULL : (TIMER_HANDLE)priv;
}
