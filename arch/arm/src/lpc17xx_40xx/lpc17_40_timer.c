/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_timer.c
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
#include "chip.h"
#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_timer.h"
#include "hardware/lpc176x_pinconfig.h"
#include "lpc17_40_gpio.h"
#include "lpc176x_gpio.h"

/* This module then only compiles if there is at least one enabled timer
 * intended for use with the TIMER upper half driver.
 */

#if defined(CONFIG_LPC17_40_TMR0)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM/Timer Definitions ****************************************************/

/* The following definitions are used to identify the various time types */

#define TIMTYPE_BASIC      0  /* Basic timers: TIM6-7 */
#define TIMTYPE_GENERAL16  1  /* General 16-bit timers: TIM2-5 on F1 */
#define TIMTYPE_COUNTUP16  2  /* General 16-bit count-up timers: TIM9-14 on F4 */
#define TIMTYPE_GENERAL32  3  /* General 32-bit timers: TIM2-5 on F4 */
#define TIMTYPE_ADVANCED   4  /* Advanced timers:  TIM1-8 */

#define TIMTYPE_TIM1       TIMTYPE_ADVANCED

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwm_dumpgpio(p,m) lpc17_40_dumpgpio(p,m)
#else
#  define pwm_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one PWM timer */

struct lpc17_40_timer_s
{
  const struct pwm_ops_s *ops;     /* PWM operations */
  uint8_t                 timid;   /* Timer ID {0,...,7} */
  uint8_t                 channel; /* Timer output channel: {1,..4} */
  uint8_t                 timtype; /* See the TIMTYPE_* definitions */
  uint32_t                base;    /* The base address of the timer */
  uint32_t                pincfg;  /* Output pin configuration */
  uint32_t                pclk;    /* The frequency of the peripheral clock
                                    * that drives the timer module. */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Register access */

static uint32_t timer_getreg(struct lpc17_40_timer_s *priv, int offset);
static void timer_putreg(struct lpc17_40_timer_s *priv,
                         int offset, uint32_t value);

#ifdef CONFIG_DEBUG_PWM_INFO
static void timer_dumpregs(struct lpc17_40_timer_s *priv,
                           const char *msg);
#else
#  define timer_dumpregs(priv,msg)
#endif

/* Timer management */

static int timer_timer(struct lpc17_40_timer_s *priv,
                       const struct pwm_info_s *info);

/* PWM driver methods */

static int timer_setup(struct pwm_lowerhalf_s *dev);
static int timer_shutdown(struct pwm_lowerhalf_s *dev);

static int timer_start(struct pwm_lowerhalf_s *dev,
                       const struct pwm_info_s *info);

static int timer_stop(struct pwm_lowerhalf_s *dev);
static int timer_ioctl(struct pwm_lowerhalf_s *dev,
                       int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver
 */

static const struct pwm_ops_s g_pwmops =
{
  .setup      = timer_setup,
  .shutdown   = timer_shutdown,
  .start      = timer_start,
  .stop       = timer_stop,
  .ioctl      = timer_ioctl,
};

#ifdef CONFIG_LPC17_40_TMR0
static struct lpc17_40_timer_s g_pwm1dev =
{
  .ops        = &g_pwmops,
  .timid      = 1,
  .channel    = CONFIG_LPC17_40_MAT0_PIN,
  .timtype    = TIMTYPE_TIM1,
  .base       = LPC17_40_TMR1_BASE,
  .pincfg     = GPIO_MAT0p1_2,
  .pclk       = (0x1 << 12),
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_getreg
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

static uint32_t timer_getreg(struct lpc17_40_timer_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: timer_putreg
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

static void timer_putreg(struct lpc17_40_timer_s *priv, int offset,
                         uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: timer_dumpregs
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
static void timer_dumpregs(struct lpc17_40_timer_s *priv,
                           const char *msg)
{
  pwminfo("%s:\n", msg);
  pwminfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
          timer_getreg(priv, LPC17_40_TMR_MR0_OFFSET),
          timer_getreg(priv, LPC17_40_TMR_MR1_OFFSET),
          timer_getreg(priv, LPC17_40_TMR_MR2_OFFSET),
          timer_getreg(priv, LPC17_40_TMR_MR3_OFFSET));
#if defined(CONFIG_LPC17_40_TMR0)
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      pwminfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
              timer_getreg(priv, LPC17_40_TMR_MR0_OFFSET),
              timer_getreg(priv, LPC17_40_TMR_MR1_OFFSET),
              timer_getreg(priv, LPC17_40_TMR_MR2_OFFSET),
              timer_getreg(priv, LPC17_40_TMR_MR3_OFFSET));
    }
  else
#endif
    {
      pwminfo("  DCR: %04x DMAR: %04x\n",
              timer_getreg(priv, LPC17_40_TMR_MR2_OFFSET),
              timer_getreg(priv, LPC17_40_TMR_MR3_OFFSET));
    }
}
#endif

/****************************************************************************
 * Name: timer_timer
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

static int timer_timer(struct lpc17_40_timer_s *priv,
                       const struct pwm_info_s *info)
{
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  putreg32(info->frequency, LPC17_40_TMR0_MR1);  /* Set TIMER0 MR1 = number of counts */
  putreg32(info->frequency, LPC17_40_TMR1_MR0);  /* Set TIMER1 MR0 = number of counts */

  putreg32(1, LPC17_40_TMR0_TCR);                /* Start timer0 */
  putreg32(1, LPC17_40_TMR1_TCR);                /* Start timer1 */

  leave_critical_section(flags);
  timer_dumpregs(priv, "After starting");
  return OK;
}

#ifdef XXXXX
/****************************************************************************
 * Name: timer_interrupt
 *
 * Description:
 *   Handle timer interrupts.
 *
 * Input Parameters:
 *   priv - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int timer_interrupt(struct lpc17_40_timer_s *priv)
{
  uint16_t regval;

  /* Verify that this is an update interrupt.  Nothing else is expected. */

  regval = timer_getreg(priv, STM32_ATIM_SR_OFFSET);
  DEBUGASSERT((regval & ATIM_SR_UIF) != 0);

  /* Clear the UIF interrupt bit */

  timer_putreg(priv, STM32_ATIM_SR_OFFSET, regval & ~ATIM_SR_UIF);

  /* Calculate the new count by subtracting the number of pulses
   * since the last interrupt.
   */

  return OK;
}

/****************************************************************************
 * Name: timer_tim1/8interrupt
 *
 * Description:
 *   Handle timer 1 and 8 interrupts.
 *
 * Input Parameters:
 *   Standard NuttX interrupt inputs
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int timer_tim1interrupt(int irq, void *context)
{
  return timer_interrupt(&g_pwm1dev);
}

#endif /* XXXXX */

/****************************************************************************
 * Name: timer_setup
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

static int timer_setup(struct pwm_lowerhalf_s *dev)
{
  struct lpc17_40_timer_s *priv = (struct lpc17_40_timer_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  /* Power on the timer peripherals */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCTIM0;
  regval |= SYSCON_PCONP_PCTIM1;
  regval |= SYSCON_PCONP_PCTIM2;
  regval |= SYSCON_PCONP_PCTIM3;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Select clock for the timer peripheral */

  regval  = getreg32(LPC17_40_SYSCON_PCLKSEL0);
  regval &= ~(0x3 << 2);
  regval |= (0x1 << 2);                   /* PCLK_MC peripheral clk=CCLK=12.5 MHz */
  regval &= ~(0x3 << 4);
  regval |= (0x1 << 4);                   /* PCLK_MC peripheral clk=CCLK=12.5 MHz */
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL0);
  regval  = getreg32(LPC17_40_SYSCON_PCLKSEL1);
  regval &= ~(0x3 << 12);
  regval |= (0x1 << 12);                  /* PCLK_MC peripheral clk=CCLK=12.5 MHz */
  regval &= ~(0x3 << 14);
  regval |= (0x1 << 14);                  /* PCLK_MC peripheral clk=CCLK=12.5 MHz */
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL1);
  priv->pclk = (0x1 << 12) | (0x1 << 4);

  putreg32(1000, LPC17_40_TMR0_MR1);         /* Set TIMER0 MR1 = number of counts */

  putreg32(1, LPC17_40_TMR0_PR);             /* Prescaler count frequency: Fpclk/1 */
  putreg32(~(0x3 << 0), LPC17_40_TMR0_CCR);  /* Prescaler count frequency: Fpclk/1 */
  putreg32(~(0x3 << 0), LPC17_40_TMR0_CTCR); /* Prescaler count frequency: Fpclk/1 */
  putreg32((2 << 3), LPC17_40_TMR0_MCR);     /* Reset on match register MR1 */

  /* Output bit toggle on external match event External match on MR1, Toggle
   * external bit
   */

  putreg32(((1 << 1) | (3 << 6)), LPC17_40_TMR0_EMR);
  putreg32((1 << 0), LPC17_40_TMR0_TCR);     /* Start timer0 */

  /* Configure the output pins GPIO3.26 */

  lpc17_40_configgpio(GPIO_MAT0p1_2);

  putreg32(500, LPC17_40_TMR1_MR0);          /* Set TIMER1 MR0 = number of counts */

  putreg32(1, LPC17_40_TMR1_PR);             /* Prescaler count frequency:Fpclk/1 */
  putreg32(~(0x3 << 0), LPC17_40_TMR1_CCR);  /* Prescaler count frequency:Fpclk/1 */
  putreg32(~(0x3 << 0), LPC17_40_TMR1_CTCR); /* Prescaler count frequency:Fpclk/1 */
  putreg32((2 << 0), LPC17_40_TMR1_MCR);     /* Reset on match register MR0 */

  /*  putreg32(((1 << 0 )| (3 << 4)), LPC17_40_TMR1_EMR);
   * Output bit toggle on external match event MAT0
   */

  putreg32((1 << 0), LPC17_40_TMR1_TCR);     /* Start timer1 */

  /* configure the output pins GPIO3.26 */

  /* lpc17_40_configgpio(GPIO_MAT0p1_2); */

  leave_critical_section(flags);
  pwm_dumpgpio(priv->pincfg, "TIMER setup");
  return OK;
}

/****************************************************************************
 * Name: timer_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - A reference to the lower half TIMER driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int timer_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct lpc17_40_timer_s *priv = (struct lpc17_40_timer_s *)dev;
  uint32_t pincfg;

  pwminfo("TIM%d pincfg: %08x\n", priv->timid, priv->pincfg);

  /* Make sure that the output has been stopped */

  return OK;
}

/****************************************************************************
 * Name: timer_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev - A reference to the lower half TIMER driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int timer_start(struct pwm_lowerhalf_s *dev,
                       const struct pwm_info_s *info)
{
  struct lpc17_40_timer_s *priv = (struct lpc17_40_timer_s *)dev;
  return timer_timer(priv, info);
}

/****************************************************************************
 * Name: timer_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input Parameters:
 *   dev - A reference to the lower half TIMER driver state structure
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

static int timer_stop(struct pwm_lowerhalf_s *dev)
{
  struct lpc17_40_timer_s *priv = (struct lpc17_40_timer_s *)dev;
  uint32_t resetbit;
  uint32_t regaddr;
  uint32_t regval;
  irqstate_t flags;

  pwminfo("TIM%d\n", priv->timid);

  /* Disable interrupts momentary to stop any ongoing timer processing and
   * to prevent any concurrent access to the reset register.
   */

  flags = enter_critical_section();

  /* Disable further interrupts and stop the timer */

  /* Determine which timer to reset */

  switch (priv->timid)
    {
#ifdef CONFIG_LPC17_40_TMR0
      case 1:
        break;
#endif
    }

  /* Reset the timer - stopping the output and putting the timer back
   * into a state where timer_start() can be called.
   */

  leave_critical_section(flags);

  pwminfo("regaddr: %08x resetbit: %08x\n", regaddr, resetbit);
  timer_dumpregs(priv, "After stop");
  return OK;
}

/****************************************************************************
 * Name: timer_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - A reference to the lower half TIMER driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int timer_ioctl(struct pwm_lowerhalf_s *dev,
                       int cmd, unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  struct lpc17_40_timer_s *priv = (struct lpc17_40_timer_s *)dev;

  /* There are no platform-specific ioctl commands */

  pwminfo("TIM%d\n", priv->timid);
#endif
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_timerinitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level TIMER driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family but is somewhere in
 *     the range of {1,..,14}.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half TIMER driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *lpc17_40_timerinitialize(int timer)
{
  struct lpc17_40_timer_s *lower;

  pwminfo("TIM%d\n", timer);

  switch (timer)
    {
#ifdef CONFIG_LPC17_40_TMR0
      case 0:
        lower = &g_pwm1dev;

        /* Attach but disable the TIM1 update interrupt */

        break;
#endif

      default:
        pwmerr("ERROR: No such timer configured\n");
        return NULL;
    }

  return (struct pwm_lowerhalf_s *)lower;
}

#endif /* CONFIG_LPC17_40_TIMn_TIMER, n = 1,...,14 */
