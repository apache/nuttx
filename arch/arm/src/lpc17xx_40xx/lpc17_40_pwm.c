/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_pwm.c
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
#  define pwm_dumpgpio(p,m) lpc17_40_dumpgpio(p,m)
#else
#  define pwm_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one PWM timer */

struct lpc17_40_pwmchan_s
{
  uint8_t                     channel; /* Timer output channel */
  uint32_t                    pincfg;  /* Output pin configuration */
};

struct lpc17_40_pwmtimer_s
{
  const struct pwm_ops_s    *ops;        /* PWM operations */
  struct lpc17_40_pwmchan_s *channels;   /* Channels configuration */
  uint8_t                   timid;       /* Timer ID {0,...,7} */
  uint8_t                   chan_num;    /* Number of configured channels */
  uint8_t                   timtype;     /* See the TIMTYPE_* definitions */
  uint32_t                  base;        /* The base address of the timer */
  uint32_t                  pclk;        /* The frequency of the peripheral clock
                                          * that drives the timer module. */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Register access */

static uint32_t pwm_getreg(struct lpc17_40_pwmtimer_s *priv, int offset);
static void pwm_putreg(struct lpc17_40_pwmtimer_s *priv,
                       int offset, uint32_t value);

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct lpc17_40_pwmtimer_s *priv,
                         const char *msg);
#else
#  define pwm_dumpregs(priv,msg)
#endif

/* Timer management */

static int pwm_timer(struct lpc17_40_pwmtimer_s *priv,
                     const struct pwm_info_s *info);

/* PWM driver methods */

static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);

static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);

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

#ifdef CONFIG_LPC17_40_PWM1

static struct lpc17_40_pwmchan_s g_pwm1channels[] =
{
  /* PWM1 has 6 channels */

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL1
  {
    .channel = 1,
    .pincfg  = GPIO_PWM1p1,
  },
#endif
#ifdef CONFIG_LPC17_40_PWM1_CHANNEL2
  {
    .channel = 2,
    .pincfg  = GPIO_PWM1p2,
  },
#endif
#ifdef CONFIG_LPC17_40_PWM1_CHANNEL3
  {
    .channel = 3,
    .pincfg  = GPIO_PWM1p3,
  },
#endif
#ifdef CONFIG_LPC17_40_PWM1_CHANNEL4
  {
    .channel = 4,
    .pincfg  = GPIO_PWM1p5,
  },
#endif
#ifdef CONFIG_LPC17_40_PWM1_CHANNEL5
  {
    .channel = 5,
    .pincfg  = GPIO_PWM1p5,
  },
#endif
#ifdef CONFIG_LPC17_40_PWM1_CHANNEL6
  {
    .channel = 6,
    .pincfg  = GPIO_PWM1p6,
  },
#endif
};

static struct lpc17_40_pwmtimer_s g_pwm1dev =
{
  .ops        = &g_pwmops,
  .timid      = 1,
  .chan_num   = LPC17_40_PWM1_NCHANNELS,
  .channels   = g_pwm1channels,
  .timtype    = TIMTYPE_TIM1,
  .base       = LPC17_40_PWM1_BASE,
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

static void pwm_putreg(struct lpc17_40_pwmtimer_s *priv,
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
static void pwm_dumpregs(struct lpc17_40_pwmtimer_s *priv,
                         const char *msg)
{
  pwminfo("%s:\n", msg);
  pwminfo("  IR: %04x TCR:  %04x TC:  %04x PR:  %04x PC:  %04x\n",
          pwm_getreg(priv, LPC17_40_PWM_IR_OFFSET),
          pwm_getreg(priv, LPC17_40_PWM_TCR_OFFSET),
          pwm_getreg(priv, LPC17_40_PWM_TC_OFFSET),
          pwm_getreg(priv, LPC17_40_PWM_PR_OFFSET),
          pwm_getreg(priv, LPC17_40_PWM_PC_OFFSET));
  pwminfo("  MCR: %04x\n",
          pwm_getreg(priv, LPC17_40_PWM_MCR_OFFSET));
#ifdef CONFIG_PWM_MULTICHAN
  pwminfo("  0: %08x 1: %08x 2: %08x 3: %08x\n",
          pwm_getreg(priv, LPC17_40_PWM_MR0_OFFSET),
          pwm_getreg(priv, LPC17_40_PWM_MR1_OFFSET),
          pwm_getreg(priv, LPC17_40_PWM_MR2_OFFSET),
          pwm_getreg(priv, LPC17_40_PWM_MR3_OFFSET));
  pwminfo("  4: %08x 5: %08x 6: %08x\n",
          pwm_getreg(priv, LPC17_40_PWM_MR4_OFFSET),
          pwm_getreg(priv, LPC17_40_PWM_MR5_OFFSET),
          pwm_getreg(priv, LPC17_40_PWM_MR6_OFFSET));
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

static int pwm_timer(struct lpc17_40_pwmtimer_s *priv,
                     const struct pwm_info_s *info)
{
  irqstate_t flags;
  uint32_t i;
  int ret = OK;
  uint32_t lerval = LER0_EN;
  uint32_t pcrval = 0;
  uint32_t mr0_freq;

  flags = enter_critical_section();

  mr0_freq = 1.f / info->frequency * LPC17_40_PWM_CLOCK / 10;

  putreg32(mr0_freq, LPC17_40_PWM1_MR0);         /* Set PWMMR0 = number of counts */

#ifndef CONFIG_PWM_MULTICHAN
  putreg32(info->duty, LPC17_40_PWM1_MR1);              /* Set PWM cycle */
#else
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
     switch (priv->channels[i].channel)
       {
#ifdef CONFIG_LPC17_40_PWM1_CHANNEL1
         case 1:
             putreg32(ub16mulub16(info->channels[i].duty, mr0_freq),
                      LPC17_40_PWM1_MR1); /* Set PWM cycle */
             break;
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL2
         case 2:
             putreg32(ub16mulub16(info->channels[i].duty, mr0_freq),
                      LPC17_40_PWM1_MR2); /* Set PWM cycle */
             break;
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL3
         case 3:
             putreg32(ub16mulub16(info->channels[i].duty, mr0_freq),
                      LPC17_40_PWM1_MR3); /* Set PWM cycle */
             break;
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL4
         case 4:
             putreg32(ub16mulub16(info->channels[i].duty, mr0_freq),
                      LPC17_40_PWM1_MR4); /* Set PWM cycle */
             break;
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL5
         case 5:
             putreg32(ub16mulub16(info->channels[i].duty, mr0_freq),
                      LPC17_40_PWM1_MR5); /* Set PWM cycle */
             break;
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL6
         case 6:
             putreg32(ub16mulub16(info->channels[i].duty, mr0_freq),
                      LPC17_40_PWM1_MR6); /* Set PWM cycle */
             break;
#endif

         default:
           {
             pwmerr("ERROR: Channel %d does not exist\n",
                    priv->channels[i].channel);
             ret = -EINVAL;
           }
       }
    }
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL1
  pcrval |= PWMENA1;
  lerval |= LER1_EN;
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL2
  pcrval |= PWMENA2;
  lerval |= LER2_EN;
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL3
  pcrval |= PWMENA3;
  lerval |= LER3_EN;
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL4
  pcrval |= PWMENA4;
  lerval |= LER4_EN;
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL5
  pcrval |= PWMENA5;
  lerval |= LER5_EN;
#endif

#ifdef CONFIG_LPC17_40_PWM1_CHANNEL6
  pcrval |= PWMENA6;
  lerval |= LER6_EN;
#endif

  putreg32(lerval, LPC17_40_PWM1_LER);       /* Load Shadow register contents */
  putreg32(pcrval, LPC17_40_PWM1_PCR);       /* Enable PWM outputs */

  putreg32(TCR_CNT_EN | TCR_PWM_EN, LPC17_40_PWM1_TCR); /* Enable PWM Timer */

  leave_critical_section(flags);
  pwm_dumpregs(priv, "After starting");

  return ret;
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

static int pwm_tim1interrupt(int irq, void *context, void *arg)
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

static void pwm_set_apb_clock(struct lpc17_40_pwmtimer_s *priv, bool on)
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

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct lpc17_40_pwmtimer_s *priv =
                             (struct lpc17_40_pwmtimer_s *)dev;
  irqstate_t flags;
  uint32_t regval;
  int i;

  flags = enter_critical_section();

  /* Power on the pwm peripheral */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCPWM1;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Select clock for the pwm peripheral */

  regval  = getreg32(LPC17_40_SYSCON_PCLKSEL0);
  regval &= ~SYSCON_PCLKSEL0_PWM1_MASK;                          /* PCLK_MC peripheral clk = CCLK = LPC17_40_CCLK */
  regval |= (SYSCON_PCLKSEL_CCLK << SYSCON_PCLKSEL0_PWM1_SHIFT); /* PCLK_MC peripheral clk = CCLK = LPC17_40_CCLK */
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL0);
  priv->pclk = (0x1 << 12);

  /* Configure the output pin */

  for (i = 0; i < priv->chan_num; i++)
    {
      lpc17_40_configgpio(priv->channels[i].pincfg);
    }

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

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct lpc17_40_pwmtimer_s *priv =
                              (struct lpc17_40_pwmtimer_s *)dev;
  uint32_t regval;
  uint32_t i;

  /* Configure the output pin to be output and low */

  for (i = 0; i < priv->chan_num; i++)
    {
      regval = priv->channels[i].pincfg;
      regval &= (GPIO_PORT_MASK | GPIO_PIN_MASK);
      regval |= (GPIO_OUTPUT | GPIO_VALUE_ZERO);

      lpc17_40_configgpio(regval);
    }

  /* Power off the pwm peripheral */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval &= ~SYSCON_PCONP_PCPWM1;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

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

static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct lpc17_40_pwmtimer_s *priv =
                          (struct lpc17_40_pwmtimer_s *)dev;
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

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct lpc17_40_pwmtimer_s *priv =
                            (struct lpc17_40_pwmtimer_s *)dev;
  uint32_t resetbit;
  uint32_t regaddr;
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

  pwminfo("regaddr: %08lx resetbit: %08lx\n", regaddr, resetbit);
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
  struct lpc17_40_pwmtimer_s *priv =
                                 (struct lpc17_40_pwmtimer_s *)dev;

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

struct pwm_lowerhalf_s *lpc17_40_pwminitialize(int timer)
{
  struct lpc17_40_pwmtimer_s *lower;

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
        pwmerr("ERROR: No such timer configured or pin defined\n");
        return NULL;
    }

    /* Ensure PWM has been shutdown */

    pwm_shutdown((struct pwm_lowerhalf_s *)lower);

  return (struct pwm_lowerhalf_s *)lower;
}

#endif /* CONFIG_LPC17_40_TIMn_PWM, n = 1,...,14 */
