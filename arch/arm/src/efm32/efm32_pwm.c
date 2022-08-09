/****************************************************************************
 * arch/arm/src/efm32/efm32_pwm.c
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
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/efm32_cmu.h"
#include "hardware/efm32_timer.h"
#include "efm32_timer.h"
#include "efm32_config.h"
#include "efm32_gpio.h"

/* This module then only compiles if there is at least one enabled timer
 * intended for use with the PWM upper half driver.
 */

#if defined(CONFIG_EFM32_TIMER0_PWM)  || \
    defined(CONFIG_EFM32_TIMER1_PWM)  || \
    defined(CONFIG_EFM32_TIMER2_PWM)  || \
    defined(CONFIG_EFM32_TIMER3_PWM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM/Timer Definitions ****************************************************/

/* The following definitions are used to identify the various time types */

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwm_dumpgpio(p,m) efm32_dumpgpio(p,m)
#else
#  define pwm_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one PWM timer */

struct efm32_pwmtimer_s
{
  const struct pwm_ops_s *ops;     /* PWM operations */
  uint8_t                 timid;   /* Timer ID {1,...,14} */
  uint8_t                 channel; /* Timer output channel: {1,..4} */
  uint8_t                 pinloc;  /* Timer output channel pin location */
#ifdef CONFIG_PWM_PULSECOUNT
  uint8_t                 irq;     /* Timer update IRQ */
  uint8_t                 prev;    /* The previous value of the RCR (pre-loaded) */
  uint8_t                 curr;    /* The current value of the RCR (pre-loaded) */
  uint32_t                count;   /* Remaining pulse count */
#endif
  uint32_t                base;    /* The base address of the timer */
  uint32_t                pincfg;  /* Output pin configuration */
  uint32_t                pclk;    /* The frequency of the peripheral clock
                                    * that drives the timer module. */
#ifdef CONFIG_PWM_PULSECOUNT
  void                   *handle;  /* Handle used for upper-half callback */
#endif
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Register access */

static uint32_t pwm_getreg(struct efm32_pwmtimer_s *priv, int offset);
static void pwm_putreg(struct efm32_pwmtimer_s *priv, int offset,
                       uint32_t value);

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct efm32_pwmtimer_s *priv, const char *msg);
#else
#  define pwm_dumpregs(priv,msg)
#endif

/* Timer management */

static int pwm_timer(struct efm32_pwmtimer_s *priv,
                     const struct pwm_info_s *info);

#if defined(CONFIG_PWM_PULSECOUNT) && (defined(CONFIG_EFM32_TIMER0_PWM) || \
                                       defined(CONFIG_EFM32_TIMER1_PWM) || \
                                       defined(CONFIG_EFM32_TIMER2_PWM) || \
                                       defined(CONFIG_EFM32_TIMER3_PWM) \
                                       )
static int pwm_interrupt(int irq, void *context, void *arg);
static uint8_t pwm_pulsecount(uint32_t count);

#endif

/* PWM driver methods */

static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);

#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info,
                     void *handle);
#else
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);
#endif

static int pwm_stop(struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver
 */

static const struct pwm_ops_s g_pwmops =
{
  .setup      = pwm_setup,
  .shutdown   = pwm_shutdown,
  .start      = pwm_start,
  .stop       = pwm_stop,
  .ioctl      = pwm_ioctl,
};

#ifdef CONFIG_EFM32_TIMER0_PWM
static struct efm32_pwmtimer_s g_pwm0dev =
{
  .ops        = &g_pwmops,
  .timid      = 0,
  .channel    = CONFIG_EFM32_TIMER0_CHANNEL,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = EFM32_IRQ_TIMER0,
#endif
  .base       = EFM32_TIMER0_BASE,
  .pincfg     = BOARD_PWM_TIMER0_PINCFG,
  .pinloc     = BOARD_PWM_TIMER0_PINLOC,
  .pclk       = BOARD_PWM_TIMER0_CLKIN,
};
#endif

#ifdef CONFIG_EFM32_TIMER1_PWM
static struct efm32_pwmtimer_s g_pwm1dev =
{
  .ops        = &g_pwmops,
  .timid      = 0,
  .channel    = CONFIG_EFM32_TIMER1_CHANNEL,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = EFM32_IRQ_TIMER1,
#endif
  .base       = EFM32_TIMER1_BASE,
  .pincfg     = BOARD_PWM_TIMER1_PINCFG,
  .pinloc     = BOARD_PWM_TIMER1_PINLOC,
  .pclk       = BOARD_PWM_TIMER1_CLKIN,
};
#endif

#ifdef CONFIG_EFM32_TIMER2_PWM
static struct efm32_pwmtimer_s g_pwm2dev =
{
  .ops        = &g_pwmops,
  .timid      = 0,
  .channel    = CONFIG_EFM32_TIMER2_CHANNEL,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = EFM32_IRQ_TIMER2,
#endif
  .base       = EFM32_TIMER2_BASE,
  .pincfg     = BOARD_PWM_TIMER2_PINCFG,
  .pinloc     = BOARD_PWM_TIMER2_PINLOC,
  .pclk       = BOARD_PWM_TIMER2_CLKIN,
};
#endif

#ifdef CONFIG_EFM32_TIMER3_PWM
static struct efm32_pwmtimer_s g_pwm3dev =
{
  .ops        = &g_pwmops,
  .timid      = 0,
  .channel    = CONFIG_EFM32_TIMER3_CHANNEL,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq        = EFM32_IRQ_TIMER3,
#endif
  .base       = EFM32_TIMER3_BASE,
  .pincfg     = BOARD_PWM_TIMER3_PINCFG,
  .pinloc     = BOARD_PWM_TIMER3_PINLOC,
  .pclk       = BOARD_PWM_TIMER3_CLKIN,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_getreg
 *
 * Description:
 *   Read the value of an PWM timer register.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t pwm_getreg(struct efm32_pwmtimer_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: pwm_putreg
 *
 * Description:
 *   Read the value of an PWM timer register.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pwm_putreg(struct efm32_pwmtimer_s *priv,
                       int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: pwm_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct efm32_pwmtimer_s *priv, const char *msg)
{
  /* TODO debug pwm_dumpregs */

#if 0
  pwminfo("%s:\n", msg);
  pwminfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
          pwm_getreg(priv, STM32_GTIM_CR1_OFFSET),
          pwm_getreg(priv, STM32_GTIM_CR2_OFFSET),
          pwm_getreg(priv, STM32_GTIM_SMCR_OFFSET),
          pwm_getreg(priv, STM32_GTIM_DIER_OFFSET));
  pwminfo("   SR: %04x EGR:  %04x CCMR1: %04x CCMR2: %04x\n",
          pwm_getreg(priv, STM32_GTIM_SR_OFFSET),
          pwm_getreg(priv, STM32_GTIM_EGR_OFFSET),
          pwm_getreg(priv, STM32_GTIM_CCMR1_OFFSET),
          pwm_getreg(priv, STM32_GTIM_CCMR2_OFFSET));
  pwminfo(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
          pwm_getreg(priv, STM32_GTIM_CCER_OFFSET),
          pwm_getreg(priv, STM32_GTIM_CNT_OFFSET),
          pwm_getreg(priv, STM32_GTIM_PSC_OFFSET),
          pwm_getreg(priv, STM32_GTIM_ARR_OFFSET));
  pwminfo(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
          pwm_getreg(priv, STM32_GTIM_CCR1_OFFSET),
          pwm_getreg(priv, STM32_GTIM_CCR2_OFFSET),
          pwm_getreg(priv, STM32_GTIM_CCR3_OFFSET),
          pwm_getreg(priv, STM32_GTIM_CCR4_OFFSET));
#if defined(CONFIG_STM32_TIM1_PWM) || defined(CONFIG_STM32_TIM8_PWM)
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      pwminfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
          pwm_getreg(priv, STM32_ATIM_RCR_OFFSET),
          pwm_getreg(priv, STM32_ATIM_BDTR_OFFSET),
          pwm_getreg(priv, STM32_ATIM_DCR_OFFSET),
          pwm_getreg(priv, STM32_ATIM_DMAR_OFFSET));
    }
  else
#endif
    {
      pwminfo("  DCR: %04x DMAR: %04x\n",
          pwm_getreg(priv, STM32_GTIM_DCR_OFFSET),
          pwm_getreg(priv, STM32_GTIM_DMAR_OFFSET));
    }
#endif
}
#endif

/****************************************************************************
 * Name: pwm_timer
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   priv - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_timer(struct efm32_pwmtimer_s *priv,
                     const struct pwm_info_s *info)
{
  /* Register contents */

  uint32_t regval;
  uint32_t cc_offet = EFM32_TIMER_CC_OFFSET(priv->channel);

  DEBUGASSERT(priv != NULL && info != NULL);

#ifdef CONFIG_PWM_PULSECOUNT
  pwminfo("TIMER%d channel: %d frequency: %d duty: %08x count: %d\n",
          priv->timid, priv->channel, info->frequency,
          info->duty, info->count);
#else
  pwminfo("TIMER%d channel: %d frequency: %d duty: %08x\n",
          priv->timid, priv->channel, info->frequency, info->duty);
#endif
  DEBUGASSERT(info->frequency > 0 && info->duty >= 0 &&
              info->duty < uitoub16(100));

  efm32_timer_reset(priv->base);

#ifdef CONFIG_PWM_PULSECOUNT
#error "Not implemented ! Sorry"
#endif

  if (efm32_timer_set_freq(priv->base, priv->pclk, info->frequency) < 0)
    {
      pwmerr("ERROR: Cannot set TIMER frequency %dHz from clock %dHz\n",
             info->frequency, priv->pclk);
      return -EINVAL;
    }

  regval = ((uint32_t)(priv->pinloc)) << _TIMER_ROUTE_LOCATION_SHIFT;

  switch (priv->channel)
    {
    case 0:
      regval |= _TIMER_ROUTE_CC0PEN_MASK;
      break;

    case 1:
      regval |= _TIMER_ROUTE_CC1PEN_MASK;
      break;

    case 2:
      regval |= _TIMER_ROUTE_CC2PEN_MASK;
      break;

    default:
      DEBUGPANIC();
    }

  pwm_putreg(priv, EFM32_TIMER_ROUTE_OFFSET, regval);

  regval = (info->duty * pwm_getreg(priv, EFM32_TIMER_TOP_OFFSET)) >> 16;
  pwm_putreg(priv, cc_offet + EFM32_TIMER_CC_CCV_OFFSET, regval);

  /* pwm_putreg(priv, cc_offet + EFM32_TIMER_CC_CCVB_OFFSET, regval); */

  regval = (_TIMER_CC_CTRL_MODE_PWM   << _TIMER_CC_CTRL_MODE_SHIFT)   | \
           (_TIMER_CC_CTRL_CMOA_CLEAR << _TIMER_CC_CTRL_CMOA_SHIFT)   | \
           (_TIMER_CC_CTRL_COFOA_SET  << _TIMER_CC_CTRL_COFOA_SHIFT)   ;

  pwm_putreg(priv, cc_offet + EFM32_TIMER_CC_CTRL_OFFSET, regval);

  /* Start Timer */

  pwm_putreg(priv, EFM32_TIMER_CMD_OFFSET, TIMER_CMD_START);
  pwm_dumpregs(priv, "After starting");
  return OK;
}

/****************************************************************************
 * Name: pwm_interrupt
 *
 * Description:
 *   Handle timer interrupts.
 *
 * Input Parameters:
 *   Standard interrupt handler arguments.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

#if defined(CONFIG_PWM_PULSECOUNT) && (defined(CONFIG_EFM32_TIMER0_PWM) || \
                                       defined(CONFIG_EFM32_TIMER1_PWM) || \
                                       defined(CONFIG_EFM32_TIMER2_PWM) || \
                                       defined(CONFIG_EFM32_TIMER3_PWM) \
                                       )
#warning "not yet implemented"
static int pwm_interrupt(int irq, void *context, void *arg)
{
  /* TODO pwm_interrupt */
#if 0
  struct efm32_pwmtimer_s *priv = (struct efm32_pwmtimer_s *)arg;
  uint32_t regval;

  DEBUGASSERT(priv != NULL);

  /* Verify that this is an update interrupt.  Nothing else is expected. */

  regval = pwm_getreg(priv, STM32_ATIM_SR_OFFSET);
  DEBUGASSERT((regval & ATIM_SR_UIF) != 0);

  /* Clear the UIF interrupt bit */

  pwm_putreg(priv, STM32_ATIM_SR_OFFSET, regval & ~ATIM_SR_UIF);

  /* Calculate the new count by subtracting the number of pulses
   * since the last interrupt.
   */

  if (priv->count <= priv->prev)
    {
      /* We are finished.  Turn off the mast output to stop the output as
       * quickly as possible.
       */

      regval  = pwm_getreg(priv, STM32_ATIM_BDTR_OFFSET);
      regval &= ~ATIM_BDTR_MOE;
      pwm_putreg(priv, STM32_ATIM_BDTR_OFFSET, regval);

      /* Disable first interrupts, stop and reset the timer */

      pwm_stop((struct pwm_lowerhalf_s *)priv);

      /* Then perform the callback into the upper half driver */

      pwm_expired(priv->handle);

      priv->handle = NULL;
      priv->count  = 0;
      priv->prev   = 0;
      priv->curr   = 0;
    }
  else
    {
      /* Decrement the count of pulses remaining using the number of
       * pulses generated since the last interrupt.
       */

      priv->count -= priv->prev;

      /* Set up the next RCR.  Set 'prev' to the value of the RCR that
       * was loaded when the update occurred (just before this interrupt)
       * and set 'curr' to the current value of the RCR register (which
       * will bet loaded on the next update event).
       */

      priv->prev = priv->curr;
      priv->curr = pwm_pulsecount(priv->count - priv->prev);
      pwm_putreg(priv, STM32_ATIM_RCR_OFFSET, (uint32_t)priv->curr - 1);
    }

  /* Now all of the time critical stuff is done so we can do some debug
   * output
   */

  pwminfo("Update interrupt SR: %04x prev: %d curr: %d count: %d\n",
          regval, priv->prev, priv->curr, priv->count);

  return OK;
#else
  return -ENODEV;
#endif
}
#endif

/****************************************************************************
 * Name: pwm_pulsecount
 *
 * Description:
 *   Pick an optimal pulse count to program the RCR.
 *
 * Input Parameters:
 *   count - The total count remaining
 *
 * Returned Value:
 *   The recommended pulse count
 *
 ****************************************************************************/

#if defined(CONFIG_PWM_PULSECOUNT) && (defined(CONFIG_EFM32_TIMER0_PWM) || \
                                       defined(CONFIG_EFM32_TIMER1_PWM) || \
                                       defined(CONFIG_EFM32_TIMER2_PWM) || \
                                       defined(CONFIG_EFM32_TIMER3_PWM) \
                                       )
static uint8_t pwm_pulsecount(uint32_t count)
{
  /* The the remaining pulse count is less than or equal to the maximum, the
   * just return the count.
   */

  if (count <= ATIM_RCR_REP_MAX)
    {
      return count;
    }

  /* Otherwise, we have to be careful.  We do not want a small number of
   * counts at the end because we might have trouble responding fast enough.
   * If the remaining count is less than 150% of the maximum, then return
   * half of the maximum.  In this case the final sequence will be between 64
   * and 128.
   */

  else if (count < (3 * ATIM_RCR_REP_MAX / 2))
    {
      return (ATIM_RCR_REP_MAX + 1) >> 1;
    }

  /* Otherwise, return the maximum.  The final count will be 64 or more */

  else
    {
      return ATIM_RCR_REP_MAX;
    }
}
#endif

/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   APB1 or 2 clocking for the GPIOs has already been configured by the RCC
 *   logic at power up.
 *
 ****************************************************************************/

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct efm32_pwmtimer_s *priv = (struct efm32_pwmtimer_s *)dev;

  pwminfo("TIMER%d pincfg: %08x\n", priv->timid, priv->pincfg);
  pwm_dumpregs(priv, "Initially");

  /* Configure the PWM output pin, but do not start the timer yet */

  /* Dnable TIMER clock */

  switch (priv->timid)
    {
    case 0:
      modifyreg32(EFM32_CMU_HFPERCLKEN0, 0, CMU_HFPERCLKEN0_TIMER0);
      break;

    case 1:
      modifyreg32(EFM32_CMU_HFPERCLKEN0, 0, CMU_HFPERCLKEN0_TIMER1);
      break;

    case 2:
      modifyreg32(EFM32_CMU_HFPERCLKEN0, 0, CMU_HFPERCLKEN0_TIMER2);
      break;

    case 3:
      modifyreg32(EFM32_CMU_HFPERCLKEN0, 0, CMU_HFPERCLKEN0_TIMER3);
      break;

    default:
      DEBUGPANIC();
      break;
    }

  efm32_configgpio(priv->pincfg);
  pwm_putreg(priv, EFM32_TIMER_ROUTE_OFFSET, BOARD_PWM_TIMER0_PINLOC);
  pwm_dumpgpio(priv->pincfg, "PWM setup");
  return OK;
}

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct efm32_pwmtimer_s *priv = (struct efm32_pwmtimer_s *)dev;
  uint32_t pincfg;

  pwminfo("TIMER%d pincfg: %08x\n", priv->timid, priv->pincfg);

  /* Make sure that the output has been stopped */

  pwm_stop(dev);

  /* Then put the GPIO pin back to the default state */

  pincfg = priv->pincfg & (GPIO_PORT_MASK | GPIO_PIN_MASK);

  pincfg |= (_GPIO_DISABLE);

  efm32_configgpio(pincfg);
  return OK;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info,
                     void *handle)
{
  struct efm32_pwmtimer_s *priv = (struct efm32_pwmtimer_s *)dev;

  /* Save the handle */

  priv->handle = handle;

  /* Start the time */

  return pwm_timer(priv, info);
}
#else
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct efm32_pwmtimer_s *priv = (struct efm32_pwmtimer_s *)dev;
  return pwm_timer(priv, info);
}
#endif

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   This function is called to stop the pulsed output at anytime.  This
 *   method is also called from the timer interrupt handler when a repetition
 *   count expires... automatically stopping the timer.
 *
 ****************************************************************************/

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct efm32_pwmtimer_s *priv = (struct efm32_pwmtimer_s *)dev;
  irqstate_t flags;

  pwminfo("TIMER%d\n", priv->timid);

  /* Disable interrupts momentary to stop any ongoing timer processing and
   * to prevent any concurrent access to the reset register.
   */

  flags = enter_critical_section();

  /* Reset the timer - stopping the output and putting the timer back
   * into a state where pwm_start() can be called.
   */

  pwm_putreg(priv, EFM32_TIMER_CMD_OFFSET, TIMER_CMD_STOP);

  leave_critical_section(flags);

  pwm_dumpregs(priv, "After stop");
  return OK;
}

/****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_ioctl(struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  struct efm32_pwmtimer_s *priv = (struct efm32_pwmtimer_s *)dev;

  /* There are no platform-specific ioctl commands */

  pwminfo("TIMER%d\n", priv->timid);
#endif
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efm32_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family but is somewhere in
 *     the range of {1,..,14}.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *efm32_pwminitialize(int timer)
{
  struct efm32_pwmtimer_s *lower;

  pwminfo("TIMER%d\n", timer);

  switch (timer)
    {
#ifdef CONFIG_EFM32_TIMER0_PWM
      case 0:
        lower = &g_pwm0dev;
        break;
#endif

#ifdef CONFIG_EFM32_TIMER1_PWM
      case 1:
        lower = &g_pwm1dev;
        break;
#endif
#ifdef CONFIG_EFM32_TIMER2_PWM
      case 2:
        lower = &g_pwm2dev;
        break;
#endif
#ifdef CONFIG_EFM32_TIMER3_PWM
      case 3:
        lower = &g_pwm3dev;
        break;
#endif

      default:
        pwmerr("ERROR: No such timer configured\n");
        return NULL;
    }

  /* Attach but disable the timer update interrupt */

#ifdef CONFIG_PWM_PULSECOUNT
  irq_attach(lower->irq, pwm_interrupt, lower);
  up_disable_irq(lower->irq);
#endif

  return (struct pwm_lowerhalf_s *)lower;
}

#endif /* CONFIG_EFM32_TIMn_PWM, n = 0,..,3 */
