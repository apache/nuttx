/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_mcpwm.c
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
#include "lpc17_40_pwm.h"
#include "hardware/lpc176x_pinconfig.h"
#include "lpc17_40_gpio.h"
#include "lpc176x_gpio.h"

/* This module then only compiles if there is at least one enabled timer
 * intended for use with the PWM upper half driver.
 */

#if defined(CONFIG_LPC17_40_MCPWM)

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
#  define pwm_dumpgpio(p,m) stm32_dumpgpio(p,m)
#else
#  define pwm_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one PWM timer */

struct lpc17_40_mcpwmtimer_s
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

static uint32_t mcpwm_getreg(struct lpc17_40_mcpwmtimer_s *priv, int offset);
static void mcpwm_putreg(struct lpc17_40_mcpwmtimer_s *priv,
                         int offset, uint32_t value);

#ifdef CONFIG_DEBUG_PWM_INFO
static void mcpwm_dumpregs(struct lpc17_40_mcpwmtimer_s *priv,
                           FAR const char *msg);
#else
#  define mcpwm_dumpregs(priv,msg)
#endif

/* Timer management */

static int mcpwm_timer(FAR struct lpc17_40_mcpwmtimer_s *priv,
                     FAR const struct pwm_info_s *info);

/* PWM driver methods */

static int mcpwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int mcpwm_shutdown(FAR struct pwm_lowerhalf_s *dev);

static int mcpwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info);

static int mcpwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int mcpwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver
 */

static const struct pwm_ops_s g_pwmops =
{
  .setup      = mcpwm_setup,
  .shutdown   = mcpwm_shutdown,
  .start      = mcpwm_start,
  .stop       = mcpwm_stop,
  .ioctl      = mcpwm_ioctl,
};

#ifdef CONFIG_LPC17_40_MCPWM
static struct lpc17_40_mcpwmtimer_s g_pwm1dev =
{
  .ops        = &g_pwmops,
  .timid      = 1,
  .channel    = CONFIG_LPC17_40_MCPWM1_PIN,
  .timtype    = TIMTYPE_TIM1,
  .base       = LPC17_40_MCPWM_BASE,
  .pincfg     = GPIO_MCPWM_MCOA0,
  .pclk       = (1 << 12),
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcpwm_getreg
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

static uint32_t mcpwm_getreg(struct lpc17_40_mcpwmtimer_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: mcpwm_putreg
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

static void mcpwm_putreg(struct lpc17_40_mcpwmtimer_s *priv,
                         int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: mcpwm_dumpregs
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
static void mcpwm_dumpregs(FAR struct lpc17_40_mcpwmtimer_s *priv,
                           FAR const char *msg)
{
  pwminfo("%s:\n", msg);
  pwminfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
          mcpwm_getreg(priv, LPC17_40_PWM_MR0_OFFSET),
          mcpwm_getreg(priv, LPC17_40_PWM_MR1_OFFSET),
          mcpwm_getreg(priv, LPC17_40_PWM_MR2_OFFSET),
          mcpwm_getreg(priv, LPC17_40_PWM_MR3_OFFSET));
#if defined(CONFIG_LPC17_40_MCPWM)
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      pwminfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
              mcpwm_getreg(priv, LPC17_40_PWM_MR0_OFFSET),
              mcpwm_getreg(priv, LPC17_40_PWM_MR1_OFFSET),
              mcpwm_getreg(priv, LPC17_40_PWM_MR2_OFFSET),
              mcpwm_getreg(priv, LPC17_40_PWM_MR3_OFFSET));
    }
  else
#endif
    {
      pwminfo("  DCR: %04x DMAR: %04x\n",
              mcpwm_getreg(priv, LPC17_40_PWM_MR2_OFFSET),
              mcpwm_getreg(priv, LPC17_40_PWM_MR3_OFFSET));
    }
}
#endif

/****************************************************************************
 * Name: mcpwm_timer
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

static int mcpwm_timer(FAR struct lpc17_40_mcpwmtimer_s *priv,
                       FAR const struct pwm_info_s *info)
{
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  putreg32(info->frequency, LPC17_40_MCPWM_LIM0); /* Set PWMMR0 = number of counts */
  putreg32(info->duty, LPC17_40_MCPWM_MAT0);      /* Set PWM cycle */

  leave_critical_section(flags);
  mcpwm_dumpregs(priv, "After starting");
  return OK;
}

#ifdef XXXXX
/****************************************************************************
 * Name: mcpwm_interrupt
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

static int mcpwm_interrupt(struct lpc17_40_mcpwmtimer_s *priv)
{
  uint16_t regval;

  /* Verify that this is an update interrupt.  Nothing else is expected. */

  regval = mcpwm_getreg(priv, STM32_ATIM_SR_OFFSET);
  DEBUGASSERT((regval & ATIM_SR_UIF) != 0);

  /* Clear the UIF interrupt bit */

  mcpwm_putreg(priv, STM32_ATIM_SR_OFFSET, regval & ~ATIM_SR_UIF);

  /* Calculate the new count by subtracting the number of pulses
   * since the last interrupt.
   */

  return OK;
}

/****************************************************************************
 * Name: mcpwm_tim1/8interrupt
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

static int mcpwm_tim1interrupt(int irq, void *context)
{
  return mcpwm_interrupt(&g_pwm1dev);
}

/****************************************************************************
 * Name: mcpwm_set_apb_clock
 *
 * Description:
 *   Enable or disable APB clock for the timer peripheral
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static void mcpwm_set_apb_clock(FAR struct lpc17_40_mcpwmtimer_s *priv,
                                bool on)
{
  uint32_t en_bit;
  uint32_t regaddr;

  /* Determine which timer to configure */

  switch (priv->timid)
    {
#ifdef CONFIG_LPC17_40_MCPWM
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
 * Name: mcpwm_setup
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

static int mcpwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct lpc17_40_mcpwmtimer_s *priv =
                                   (FAR struct lpc17_40_mcpwmtimer_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  /* Power on the mcpwm peripheral */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCMCPWM;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Select clock for the mcpwm peripheral */

  regval  = getreg32(LPC17_40_SYSCON_PCLKSEL1);
  regval &= ~(0x3 << 30);
  regval |= (0x2 << 30);                      /* PCLK_MC peripheral clk = CCLK/2 = 50 MHz */
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL1);
  priv->pclk = (0x1 << 12) | (0x1 << 4);

  putreg32((1 << 15), LPC17_40_MCPWM_INTENCLR);  /* Disable MCABORT pin interrupt */
  putreg32((1 << 0),  LPC17_40_MCPWM_INTENCLR);  /* Disable ILIM0 interrupt */
  putreg32((1 << 1),  LPC17_40_MCPWM_INTENCLR);  /* Disable IMAT0 interrupt */
  putreg32((1 << 2),  LPC17_40_MCPWM_INTENCLR);  /* Disable ICAP0 interrupt */
  putreg32((1 << 4),  LPC17_40_MCPWM_INTENCLR);  /* Disable ILIM1 interrupt */
  putreg32((1 << 5),  LPC17_40_MCPWM_INTENCLR);  /* Disable IMAT1 interrupt */
  putreg32((1 << 6),  LPC17_40_MCPWM_INTENCLR);  /* Disable ICAP1 interrupt */
  putreg32((1 << 8),  LPC17_40_MCPWM_INTENCLR);  /* Disable ILIM2 interrupt */
  putreg32((1 << 9),  LPC17_40_MCPWM_INTENCLR);  /* Disable IMAT2 interrupt */
  putreg32((1 << 10), LPC17_40_MCPWM_INTENCLR);  /* Disable ICAP2 interrupt */

  putreg32((0xffffffff), LPC17_40_MCPWM_CAPCLR); /* Clear all event capture */

  /* Configure the output pins */

  lpc17_40_configgpio(GPIO_MCPWM_MCOA0);
  lpc17_40_configgpio(GPIO_MCPWM_MCOB0);

  /* Program the timing registers */

  putreg32((1 << 0),  LPC17_40_MCPWM_CONCLR);    /* Stop MCPWM timer0 */
  putreg32((1 << 8),  LPC17_40_MCPWM_CONCLR);    /* Stop MCPWM timer1 */
  putreg32((1 << 16), LPC17_40_MCPWM_CONCLR);    /* Stop MCPWM timer2 */

  putreg32((1 << 30), LPC17_40_MCPWM_CONCLR);    /* MCPWM not in AC mode */

  putreg32(1000, LPC17_40_MCPWM_TC0);            /* Count frequency: Fpclk/1000 = 50 MHz/1000 = 50 KHz */
  putreg32(400, LPC17_40_MCPWM_LIM0);            /* Set the starting duty cycle to 0.25 */
  putreg32(0, LPC17_40_MCPWM_MAT0);              /* Reset the timer */

  putreg32(100000, LPC17_40_MCPWM_TC1);          /* Count frequency:Fpclk/100000 = 50 MHz/100000 = 500 Hz */
  putreg32(50000, LPC17_40_MCPWM_LIM1);          /* Set the starting duty cycle to 0.5 */
  putreg32(0, LPC17_40_MCPWM_MAT1);              /* Reset the timer */

  putreg32(1000, LPC17_40_MCPWM_TC2);            /* Count frequency:Fpclk/1000 = 50 MHz/1000 = 50 KHz */
  putreg32(400, LPC17_40_MCPWM_LIM2);            /* Set the starting duty cycle to 0.25 */
  putreg32(0, LPC17_40_MCPWM_MAT2);              /* Reset the timer */

  putreg32((1 << 2),  LPC17_40_MCPWM_CONCLR);    /* Channel 0 polarity set to default */
  putreg32((1 << 10), LPC17_40_MCPWM_CONCLR);    /* Channel 1 polarity set to default */
  putreg32((1 << 18), LPC17_40_MCPWM_CONCLR);    /* Channel 2 polarity set to default */

  putreg32((1 << 3),  LPC17_40_MCPWM_CONCLR);    /* Channel 0 dead time disabled */
  putreg32((1 << 11), LPC17_40_MCPWM_CONCLR);    /* Channel 1 dead time disabled */
  putreg32((1 << 19), LPC17_40_MCPWM_CONCLR);    /* Channel 2 dead time disabled */

  putreg32((1 << 1),  LPC17_40_MCPWM_CONCLR);    /* Channel 0 edge aligned */
  putreg32((1 << 9),  LPC17_40_MCPWM_CONCLR);    /* Channel 1 edge aligned */
  putreg32((1 << 17), LPC17_40_MCPWM_CONCLR);    /* Channel 2 edge aligned */

  putreg32((0xffffffff), LPC17_40_MCPWM_CNTCONCLR); /* All channels in counter mode on PCLK */

  putreg32((1 << 0), LPC17_40_MCPWM_CONSET);     /* Start MCPWM timer0 */

  leave_critical_section(flags);
  pwm_dumpgpio(priv->pincfg, "PWM setup");
  return OK;
}

/****************************************************************************
 * Name: mcpwm_shutdown
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

static int mcpwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct lpc17_40_mcpwmtimer_s *priv =
                                     (FAR struct lpc17_40_mcpwmtimer_s *)dev;
  uint32_t pincfg;

  pwminfo("TIM%d pincfg: %08x\n", priv->timid, priv->pincfg);

  /* Make sure that the output has been stopped */

  return OK;
}

/****************************************************************************
 * Name: mcpwm_start
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

static int mcpwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info)
{
  FAR struct lpc17_40_mcpwmtimer_s *priv =
                                   (FAR struct lpc17_40_mcpwmtimer_s *)dev;
  return mcpwm_timer(priv, info);
}

/****************************************************************************
 * Name: mcpwm_stop
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

static int mcpwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct lpc17_40_mcpwmtimer_s *priv =
                                   (FAR struct lpc17_40_mcpwmtimer_s *)dev;
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
#ifdef CONFIG_LPC17_40_MCPWM
      case 1:
        break;
#endif
    }

  /* Reset the timer - stopping the output and putting the timer back
   * into a state where mcpwm_start() can be called.
   */

  leave_critical_section(flags);

  pwminfo("regaddr: %08x resetbit: %08x\n", regaddr, resetbit);
  mcpwm_dumpregs(priv, "After stop");
  return OK;
}

/****************************************************************************
 * Name: mcpwm_ioctl
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

static int mcpwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                       int cmd, unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  FAR struct lpc17_40_mcpwmtimer_s *priv =
                                   (FAR struct lpc17_40_mcpwmtimer_s *)dev;

  /* There are no platform-specific ioctl commands */

  pwminfo("TIM%d\n", priv->timid);
#endif
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_mcpwminitialize
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

FAR struct pwm_lowerhalf_s *lpc17_40_mcpwminitialize(int timer)
{
  FAR struct lpc17_40_mcpwmtimer_s *lower;

  pwminfo("TIM%d\n", timer);

  switch (timer)
    {
#ifdef CONFIG_LPC17_40_MCPWM
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
