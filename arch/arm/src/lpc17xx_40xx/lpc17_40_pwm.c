/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_pwm.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_pwm.h"
#include "hardware/lpc176x_pinconfig.h"
#include "lpc17_40_gpio.h"
#include "lpc176x_gpio.h"

/* This module then only compiles if there is at least one enabled timer
 * intended for use with the PWM upper half driver.
 */

#if defined(CONFIG_LPC17_40_PWM1)

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


#define LER0_EN            (1 << 0)
#define LER1_EN            (1 << 1)
#define LER2_EN            (1 << 2)
#define LER3_EN            (1 << 3)
#define LER4_EN            (1 << 4)
#define LER5_EN            (1 << 5)
#define LER6_EN            (1 << 6)
#define PWMENA1            (1 << 9)
#define PWMENA2            (1 << 10)
#define PWMENA3            (1 << 11)
#define PWMENA4            (1 << 12)
#define PWMENA5            (1 << 13)
#define PWMENA6            (1 << 14)
#define TCR_CNT_EN         (0x00000001)
#define TCR_RESET          (0x00000002)
#define TCR_PWM_EN         (0x00000008)

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwm_dumpgpio(p,m) stm32_dumpgpio(p,m)
#else
#  define pwm_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure represents the state of one PWM timer */

struct lpc17_40_pwmtimer_s
{
  FAR const struct pwm_ops_s *ops;     /* PWM operations */
  uint8_t                     timid;   /* Timer ID {0,...,7} */
  uint8_t                     channel; /* Timer output channel: {1,..4} */
  uint8_t                     timtype; /* See the TIMTYPE_* definitions */
  uint32_t                    base;    /* The base address of the timer */
  uint32_t                    pincfg;  /* Output pin configuration */
  uint32_t                    pclk;    /* The frequency of the peripheral clock
                                        * that drives the timer module. */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/
/* Register access */

static uint32_t pwm_getreg(struct lpc17_40_pwmtimer_s *priv, int offset);
static void pwm_putreg(struct lpc17_40_pwmtimer_s *priv, int offset, uint32_t value);

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct lpc17_40_pwmtimer_s *priv, FAR const char *msg);
#else
#  define pwm_dumpregs(priv,msg)
#endif

/* Timer management */

static int pwm_timer(FAR struct lpc17_40_pwmtimer_s *priv,
                     FAR const struct pwm_info_s *info);

/* PWM driver methods */

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);

static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info);

static int pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* This is the list of lower half PWM driver methods used by the upper half driver */

static const struct pwm_ops_s g_pwmops =
{
  .setup      = pwm_setup,
  .shutdown   = pwm_shutdown,
  .start      = pwm_start,
  .stop       = pwm_stop,
  .ioctl      = pwm_ioctl,
};

#ifdef CONFIG_LPC17_40_PWM1
static struct lpc17_40_pwmtimer_s g_pwm1dev =
{
  .ops        = &g_pwmops,
  .timid      = 1,
  .channel    = CONFIG_LPC17_40_PWM1_PIN,
  .timtype    = TIMTYPE_TIM1,
  .base       = LPC17_40_PWM1_BASE,
  .pincfg     = GPIO_PWM1p1_1,
  .pclk       = (0x1 << 12),
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

static uint32_t pwm_getreg(struct lpc17_40_pwmtimer_s *priv, int offset)
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

static void pwm_putreg(struct lpc17_40_pwmtimer_s *priv, int offset, uint32_t value)
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
static void pwm_dumpregs(struct lpc17_40_pwmtimer_s *priv, FAR const char *msg)
{
  pwminfo("%s:\n", msg);
  pwminfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
          pwm_getreg(priv, LPC17_40_PWM_MR0_OFFSET),
          pwm_getreg(priv, LPC17_40_PWM_MR1_OFFSET),
          pwm_getreg(priv, LPC17_40_PWM_MR2_OFFSET),
          pwm_getreg(priv, LPC17_40_PWM_MR3_OFFSET));
#if defined(CONFIG_LPC17_40_PWM1)
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      pwminfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
              pwm_getreg(priv, LPC17_40_PWM_MR0_OFFSET),
              pwm_getreg(priv, LPC17_40_PWM_MR1_OFFSET),
              pwm_getreg(priv, LPC17_40_PWM_MR2_OFFSET),
              pwm_getreg(priv, LPC17_40_PWM_MR3_OFFSET));
    }
  else
#endif
    {
      pwminfo("  DCR: %04x DMAR: %04x\n",
              pwm_getreg(priv, LPC17_40_PWM_MR2_OFFSET),
              pwm_getreg(priv, LPC17_40_PWM_MR3_OFFSET));
    }
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

static int pwm_timer(FAR struct lpc17_40_pwmtimer_s *priv,
                     FAR const struct pwm_info_s *info)
{
  irqstate_t flags;

  flags = enter_critical_section();

  putreg32(info->frequency, LPC17_40_PWM1_MR0);         /* Set PWMMR0 = number of counts */
  putreg32(info->duty, LPC17_40_PWM1_MR1);              /* Set PWM cycle */

  putreg32(LER0_EN | LER3_EN, LPC17_40_PWM1_LER);       /* Load Shadow register contents */
  putreg32(PWMENA1, LPC17_40_PWM1_PCR);                 /* Enable PWM outputs */
  putreg32(TCR_CNT_EN | TCR_PWM_EN, LPC17_40_PWM1_TCR); /* Enable PWM Timer */

  leave_critical_section(flags);
  pwm_dumpregs(priv, "After starting");
  return OK;
}

#ifdef XXXXX
/****************************************************************************
 * Name: pwm_interrupt
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

static int pwm_interrupt(struct lpc17_40_pwmtimer_s *priv)
{
  uint16_t regval;

  /* Verify that this is an update interrupt.  Nothing else is expected. */

  regval = pwm_getreg(priv, STM32_ATIM_SR_OFFSET);
  DEBUGASSERT((regval & ATIM_SR_UIF) != 0);

  /* Clear the UIF interrupt bit */

  pwm_putreg(priv, STM32_ATIM_SR_OFFSET, regval & ~ATIM_SR_UIF);

  /* Calculate the new count by subtracting the number of pulses
   * since the last interrupt.
   */

  return OK;
}

/****************************************************************************
 * Name: pwm_tim1/8interrupt
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

static int pwm_tim1interrupt(int irq, void *context, FAR void *arg)
{
  return pwm_interrupt(&g_pwm1dev);
}

/****************************************************************************
 * Name: pwm_set_apb_clock
 *
 * Description:
 *   Enable or disable APB clock for the timer peripheral
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static void pwm_set_apb_clock(FAR struct lpc17_40_pwmtimer_s *priv, bool on)
{
  uint32_t en_bit;
  uint32_t regaddr;

  /* Determine which timer to configure */

  switch (priv->timid)
    {
#ifdef CONFIG_LPC17_40_PWM1
      case 1:
        regaddr  = STM32_RCC_APB2ENR;
        en_bit   = RCC_APB2ENR_TIM1EN;
        break;
#endif
    }

  /* Enable/disable APB 1/2 clock for timer */

  if (on)
    {
      modifyreg32(regaddr, 0, en_bit);
    }
  else
    {
      modifyreg32(regaddr, en_bit, 0);
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

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct lpc17_40_pwmtimer_s *priv = (FAR struct lpc17_40_pwmtimer_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  /* Power on the pwm peripheral */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCPWM1;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Select clock for the pwm peripheral */

  regval  = getreg32(LPC17_40_SYSCON_PCLKSEL0);
  regval &= ~(0x3 << 12);            /* PCLK_MC peripheral clk = CCLK = 12.5 MHz */
  regval |= (0x1 << 12);             /* PCLK_MC peripheral clk = CCLK = 12.5 MHz */
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL0);
  priv->pclk = (0x1 << 12);

  /* Configure the output pin */

  lpc17_40_configgpio(GPIO_PWM1p1_1);

  putreg32(1, LPC17_40_PWM1_PR);        /* Prescaler count frequency: Fpclk/1 */
  putreg32(1 << 1, LPC17_40_PWM1_MCR);  /* Reset on match register MR0 */

  leave_critical_section(flags);
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

static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct lpc17_40_pwmtimer_s *priv = (FAR struct lpc17_40_pwmtimer_s *)dev;
  uint32_t pincfg;

  pwminfo("TIM%d pincfg: %08x\n", priv->timid, priv->pincfg);

  /* Make sure that the output has been stopped */

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

static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info)
{
  FAR struct lpc17_40_pwmtimer_s *priv = (FAR struct lpc17_40_pwmtimer_s *)dev;
  return pwm_timer(priv, info);
}

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

static int pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct lpc17_40_pwmtimer_s *priv = (FAR struct lpc17_40_pwmtimer_s *)dev;
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
#ifdef CONFIG_LPC17_40_PWM1
      case 1:
        break;
#endif
    }

  /* Reset the timer - stopping the output and putting the timer back
   * into a state where pwm_start() can be called.
   */

  leave_critical_section(flags);

  pwminfo("regaddr: %08x resetbit: %08x\n", regaddr, resetbit);
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

static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  FAR struct lpc17_40_pwmtimer_s *priv = (FAR struct lpc17_40_pwmtimer_s *)dev;

  /* There are no platform-specific ioctl commands */

  pwminfo("TIM%d\n", priv->timid);
#endif
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_pwminitialize
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

FAR struct pwm_lowerhalf_s *lpc17_40_pwminitialize(int timer)
{
  FAR struct lpc17_40_pwmtimer_s *lower;

  pwminfo("TIM%d\n", timer);

  switch (timer)
    {
#ifdef CONFIG_LPC17_40_PWM1
      case 0:
        lower = &g_pwm1dev;

        /* Attach but disable the TIM1 update interrupt */

        break;
#endif

      default:
        pwmerr("ERROR: No such timer configured\n");
        return NULL;
    }

  return (FAR struct pwm_lowerhalf_s *)lower;
}

#endif /* CONFIG_LPC17_40_TIMn_PWM, n = 1,...,14 */
