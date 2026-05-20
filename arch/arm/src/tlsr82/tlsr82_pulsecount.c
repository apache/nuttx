/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_pulsecount.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <inttypes.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/pulsecount.h>
#include <arch/board/board.h>

#include "arm_internal.h"

#include "chip.h"
#include "tlsr82_pwm.h"
#include "tlsr82_pulsecount.h"
#include "tlsr82_gpio.h"
#include "tlsr82_gpio_cfg.h"

#include "hardware/tlsr82_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The PWM peripheral clock source is the system clock. */

#define PWM_SRC_CLK_HZ        (CONFIG_TLSR82_CPU_CLK_MHZ * 1000000)

/* The default clkdiv = 9, default PWM clock = PWM_SRC_CLK_HZ / 10. */

#define PWM_INIT_CLKDIV       9

/* Make sure the max frequency has at least 1% duty accuracy. */

#define PWM_MAX_FREQ          (g_pulsecountclk / 100)
#define PWM_MIN_FREQ          (g_pulsecountclk / UINT16_MAX + 1)

/* PWM peripheral max pulse count number, bit 0 ~ 13. */

#define PWM_MAX_COUNT         (0x3fff)

/* PWM simple operation definition */

#define PWM_INT_PEND(id)      BM_IS_SET(PWM_IRQ_STA_REG, 1 << ((id) + 2))
#define PWM_INT_CLEAR(id)     BM_CLR(PWM_IRQ_STA_REG, 1 << ((id) + 2))
#define PWM_INT_ENABLE(id)    BM_SET(PWM_IRQ_CTRL_REG, 1 << ((id) + 2))
#define PWM_INT_DISABLE(id)   BM_CLR(PWM_IRQ_CTRL_REG, 1 << ((id) + 2))

#define PWM0_PNUM_INT_PEND()      BM_IS_SET(PWM_IRQ_STA_REG, 1 << 0)
#define PWM0_PNUM_INT_CLEAR()     BM_CLR(PWM_IRQ_STA_REG, 1 << 0)
#define PWM0_PNUM_INT_ENABLE()    BM_SET(PWM_IRQ_CTRL_REG, 1 << 0)
#define PWM0_PNUM_INT_ISENABLE()  BM_IS_SET(PWM_IRQ_CTRL_REG, 1 << 0)
#define PWM0_PNUM_INT_DISABLE()   BM_CLR(PWM_IRQ_CTRL_REG, 1 << 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one pulsecount timer */

struct tlsr82_pulsecounttimer_s
{
  uint32_t pincfg;               /* PWM peripheral pin config */
  uint8_t id;                    /* PWM peripheral channel id */
  bool invert;                   /* Pulsecount output is inverted or not */
  bool started;                  /* Pulsecount output has started or not */
  uint8_t irq;                   /* Timer update IRQ */
  uint32_t count;                /* Remaining pulse count */
  uint32_t frequency;            /* Current frequency setting */
  uint16_t max;
  uint16_t cmp;
  void *handle;                  /* Handle used for upper-half callback */
};

struct tlsr82_pulsecount_s
{
  const struct pulsecount_ops_s *ops;
  struct tlsr82_pulsecounttimer_s *timer;
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Register access */

#ifdef CONFIG_DEBUG_TIMER_INFO
static void pulsecount_dumpregs(const char *msg);
#else
#  define pulsecount_dumpregs(msg)
#endif

/* Timer management */

static void pulsecount_enable(struct tlsr82_pulsecounttimer_s *priv,
                              bool enable);
static int pulsecount_cfg_check(struct tlsr82_pulsecounttimer_s *priv);
static int pulsecount_config(struct tlsr82_pulsecounttimer_s *priv,
                             const struct pulsecount_info_s *info);

static int pulsecount_interrupt(int irq, void *context, void *arg);

/* Pulsecount timer methods */

static int pulsecount_timer_setup(struct tlsr82_pulsecounttimer_s *priv);
static int pulsecount_timer_shutdown(struct tlsr82_pulsecounttimer_s *priv);

static int pulsecount_timer_stop(struct tlsr82_pulsecounttimer_s *priv);
static int pulsecount_timer_ioctl(struct tlsr82_pulsecounttimer_s *priv,
                                  int cmd, unsigned long arg);

/* Pulsecount driver methods */

static int pulsecount_setup(struct pulsecount_lowerhalf_s *dev);
static int pulsecount_shutdown(struct pulsecount_lowerhalf_s *dev);
static int pulsecount_start(struct pulsecount_lowerhalf_s *dev,
                            const struct pulsecount_info_s *info,
                            void *handle);
static int pulsecount_stop(struct pulsecount_lowerhalf_s *dev);
static int pulsecount_ioctl(struct pulsecount_lowerhalf_s *dev,
                            int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_pulsecountclk;

static struct tlsr82_pulsecounttimer_s g_pulsecount0dev =
{
  .pincfg      = BOARD_PWM0_PIN,
  .id          = 0,
  .invert      = false,
  .started     = false,
  .irq         = NR_SW_PWM_IRQ,
};

static const struct pulsecount_ops_s g_pulsecountops =
{
  .setup       = pulsecount_setup,
  .shutdown    = pulsecount_shutdown,
  .start       = pulsecount_start,
  .stop        = pulsecount_stop,
  .ioctl       = pulsecount_ioctl,
};

static struct tlsr82_pulsecount_s g_pulsecount0lower =
{
  .ops = &g_pulsecountops,
  .timer = &g_pulsecount0dev,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pulsecount_dumpregs
 *
 * Description:
 *   Dump all PWM peripheral registers.
 *
 * Input Parameters:
 *   msg - message.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_TIMER_INFO
static void pulsecount_dumpregs(const char *msg)
{
  int i;
  _info("%s:\n", msg);
  _info("PWM_CLKDIV_REG   : 0x%x\n", PWM_CLKDIV_REG);
  _info("PWM_ENABLE0_REG  : 0x%x\n", PWM_ENABLE0_REG);
  _info("PWM_MODE0_REG    : 0x%x\n", PWM_MODE0_REG);
  _info("PWM_ENABLE_REG   : 0x%x\n", PWM_ENABLE_REG);
  _info("PWM_INVERT_REG   : 0x%x\n", PWM_INVERT_REG);
  _info("PWM_N_INVERT_REG : 0x%x\n", PWM_N_INVERT_REG);
  _info("PWM_POL_REG      : 0x%x\n", PWM_POL_REG);

  for (i = 0; i < 5; i++)
    {
      _info("PWM_CYCLE_REG(%d): 0x%lx\n", i, PWM_CYCLE_REG(i));
      _info("PWM_CMP_REG(%d)  : 0x%x\n", i, PWM_CMP_REG(i));
      _info("PWM_MAX_REG(%d)  : 0x%x\n", i, PWM_MAX_REG(i));
    }

  _info("PWM_PLUSE_NUM_REG : 0x%x\n", PWM_PLUSE_NUM_REG);
  _info("PWM_IRQ_CTRL_REG : 0x%x\n", PWM_IRQ_CTRL_REG);
  _info("PWM_IRQ_STA_REG  : 0x%x\n", PWM_IRQ_STA_REG);
}
#endif

/****************************************************************************
 * Name: pulsecount_config
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   priv - A reference to the lower half pulsecount driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pulsecount_config(struct tlsr82_pulsecounttimer_s *priv,
                             const struct pulsecount_info_s *info)
{
  /* Calculated values */

  uint16_t max;
  uint16_t cmp;

  if (priv == NULL || info == NULL)
    {
      _err("priv or info is null, priv=%p info=%p\n", priv, info);
      return -EINVAL;
    }

  _info("PWM%d invert: %d high: %" PRIu32 " ns low: %" PRIu32 " ns\n",
          (int)priv->id, (int)priv->invert, info->high_ns,
          info->low_ns);

  if (pulsecount_frequency(info) == 0)
    {
      _err("ERROR: pulsecount period is too long, high: %" PRIu32
           " ns low: %" PRIu32 " ns\n", info->high_ns, info->low_ns);
      return -EINVAL;
    }

  if (pulsecount_frequency(info) > PWM_MAX_FREQ ||
      pulsecount_frequency(info) < PWM_MIN_FREQ)
    {
      _err("ERROR: pulsecount frequency out of range, "
           "freq: %ld, max: %ld, min: %ld\n",
           pulsecount_frequency(info), PWM_MAX_FREQ, PWM_MIN_FREQ);
      return -EINVAL;
    }

  /* Calculate the MAX and CMP register value
   * period = max_reg / g_pulsecountclk
   * freq   = g_pulsecountclk / max_reg
   */

  max = (uint16_t)((g_pulsecountclk / pulsecount_frequency(info)));

  cmp = (uint16_t)(((uint64_t)max *
                    (uint64_t)pulsecount_duty(info)) >> 16);

  _info("PWM%d PCLK: %lu freq: %lu duty: %lu max: %u cmp: %u\n",
        priv->id, g_pulsecountclk, pulsecount_frequency(info),
        pulsecount_duty(info), max, cmp);

  priv->max = max;
  priv->cmp = cmp;

  PWM_MAX_REG(priv->id) = priv->max;
  PWM_CMP_REG(priv->id) = priv->cmp;

  if (priv->count > 0)
    {
      PWM_MODE0_REG = PWM_MODE0_COUNT;
      PWM_PLUSE_NUM_REG = priv->count;
      PWM0_PNUM_INT_ENABLE();
      up_enable_irq(priv->irq);
    }
  else
    {
      PWM_MODE0_REG = PWM_MODE0_NORMAL;
    }

  pulsecount_enable(priv, true);

  return OK;
}

/****************************************************************************
 * Name: pulsecount_interrupt
 *
 * Description:
 *   Handle timer interrupts.
 *
 * Input Parameters:
 *   priv - A reference to the lower half pulsecount driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pulsecount_interrupt(int irq, void *context, void *arg)
{
  struct tlsr82_pulsecounttimer_s *priv =
    (struct tlsr82_pulsecounttimer_s *)arg;

  if (PWM0_PNUM_INT_PEND() && PWM0_PNUM_INT_ISENABLE())
    {
      /* Disable first interrupts, stop and reset the timer */

      pulsecount_enable(priv, false);

      /* Disable PWMn interrupt and clear interrupt flag */

      PWM_INT_DISABLE(priv->id);
      PWM_INT_CLEAR(priv->id);

      /* Then perform the callback into the upper half driver */

      pulsecount_expired(priv->handle);

      priv->handle = NULL;
    }

  return OK;
}

/****************************************************************************
 * Name: pulsecount_enable
 *
 * Description:
 *   Enable or disable pulse output.
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
 *   en  - Enable clock if 'en' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static void pulsecount_enable(struct tlsr82_pulsecounttimer_s *priv, bool en)
{
  int id = (int)priv->id;

  if (en)
    {
      if (priv->started)
        {
          return;
        }

      if (id == 0)
        {
          BM_SET(PWM_ENABLE0_REG, 1);
        }
      else
        {
          BM_SET(PWM_ENABLE_REG, 1 << id);
        }

      priv->started = true;
    }
  else
    {
      if (!priv->started)
        {
          return;
        }

      if (id == 0)
        {
          BM_CLR(PWM_ENABLE0_REG, 1);
        }
      else
        {
          BM_CLR(PWM_ENABLE_REG, 1 << id);
        }

      priv->started = false;
    }
}

/****************************************************************************
 * Name: pulsecount_cfg_check
 *
 * Description:
 *   This method is called when the driver initialize. This function will
 *   Check the pincfg.  If pincfg is not valid or the current pin can not be
 *   used as a PWM peripheral output, this function will call PANIC().
 *
 * Input Parameters:
 *   dev    - A reference to the lower half pulsecount driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 *
 ****************************************************************************/

static int pulsecount_cfg_check(struct tlsr82_pulsecounttimer_s *priv)
{
  uint32_t pincfg = priv->pincfg;

  if (pincfg == GPIO_INVLD_CFG)
    {
      _err("pulsecount pincfg is not valid, pincfg=0x%lx\n", pincfg);
      PANIC();
      return -EINVAL;
    }

  if (tlsr82_gpio_cfg_check(pincfg, TLSR82_MUX_PWM) != 0)
    {
      _err("pincfg=0x%lx does not support pulsecount mux\n", pincfg);
      PANIC();
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: pulsecount_timer_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pulsecount_timer_setup(struct tlsr82_pulsecounttimer_s *priv)
{
  uint32_t pincfg;
  int ret = OK;

  pulsecount_dumpregs("Initially");

  /* Configure the pulsecount output pins, but do not start the timer yet */

  pincfg = priv->pincfg;

  _info("pincfg: %08lx\n", pincfg);

  /* Set the complementary output inversion before GPIO configuration.  This
   * keeps the output state stable while the GPIO is configured.
   */

  if (priv->invert)
    {
      BM_SET(PWM_N_INVERT_REG, 1 << priv->id);
    }

  if (priv->id == 0)
    {
      ret = irq_attach(priv->irq, pulsecount_interrupt, priv);
      if (ret < 0)
        {
          _err("ERROR: PWM0 pulsecount interrupt attach failed, ret=%d\n",
               ret);
          return ret;
        }
    }

  tlsr82_gpioconfig(pincfg);

  return ret;
}

/****************************************************************************
 * Name: pulsecount_timer_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pulsecount_timer_shutdown(struct tlsr82_pulsecounttimer_s *priv)
{
  /* Make sure that the output has been stopped */

  pulsecount_timer_stop(priv);

  /* Then put the GPIO pins back to the default state */

  tlsr82_gpiounconfig(priv->pincfg);

  return OK;
}

/****************************************************************************
 * Name: pulsecount_timer_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
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

static int pulsecount_timer_stop(struct tlsr82_pulsecounttimer_s *priv)
{
  irqstate_t flags;

  _info("PWM%u\n", priv->id);

  /* Disable interrupts momentary to stop any ongoing timer processing and
   * to prevent any concurrent access to the reset register.
   */

  flags = enter_critical_section();

  /* Disable PWMn output */

  pulsecount_enable(priv, false);

  /* Disable PWMn interrupt and clear interrupt flag */

  PWM_INT_DISABLE(priv->id);
  PWM_INT_CLEAR(priv->id);

  /* Disable and clear PWM0 count interrupt flag */

  if (priv->count > 0)
    {
      priv->count = 0;
      up_disable_irq(priv->irq);
      PWM0_PNUM_INT_DISABLE();
      PWM0_PNUM_INT_CLEAR();
    }

  leave_critical_section(flags);

  pulsecount_dumpregs("After stop");

  return OK;
}

/****************************************************************************
 * Name: pulsecount_timer_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pulsecount_timer_ioctl(struct tlsr82_pulsecounttimer_s *priv,
                                  int cmd, unsigned long arg)
{
#ifdef CONFIG_DEBUG_TIMER_INFO
  /* There are no platform-specific ioctl commands */

  _info("PWM%d\n", priv->id);
#endif
  return -ENOTTY;
}

static void tlsr82_pulsecount_hwinitialize(void)
{
  static bool pulsecount_initialized = false;

  if (!pulsecount_initialized)
    {
      pulsecount_initialized = true;

      /* Disable all the output */

      BM_CLR(PWM_ENABLE0_REG, 0xff);
      BM_CLR(PWM_ENABLE_REG, 0xff);

      /* Configure the PWM peripheral clock. */

      g_pulsecountclk = PWM_SRC_CLK_HZ / (PWM_INIT_CLKDIV + 1);
      PWM_CLKDIV_REG = PWM_INIT_CLKDIV;

      /* Disable and clear all PWM peripheral interrupts. */

      BM_CLR(PWM_IRQ_CTRL_REG, 0xff);
      BM_SET(PWM_IRQ_STA_REG, 0xff);

      /* Enable the shared PWM peripheral interrupt. */

      up_enable_irq(NR_SW_PWM_IRQ);
    }
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int pulsecount_setup(struct pulsecount_lowerhalf_s *dev)
{
  struct tlsr82_pulsecount_s *pulse = (struct tlsr82_pulsecount_s *)dev;
  return pulsecount_timer_setup(pulse->timer);
}

static int pulsecount_shutdown(struct pulsecount_lowerhalf_s *dev)
{
  struct tlsr82_pulsecount_s *pulse = (struct tlsr82_pulsecount_s *)dev;
  return pulsecount_timer_shutdown(pulse->timer);
}

static int pulsecount_start(struct pulsecount_lowerhalf_s *dev,
                            const struct pulsecount_info_s *info,
                            void *handle)
{
  struct tlsr82_pulsecount_s *pulse = (struct tlsr82_pulsecount_s *)dev;
  struct tlsr82_pulsecounttimer_s *priv = pulse->timer;
  int ret;

  if (priv->id != 0)
    {
      _err("ERROR: PWM%d cannot support pulse count: %" PRIu32 "\n",
           priv->id, info->count);
      return -EPERM;
    }

  if (info->count == 0 || info->count > PWM_MAX_COUNT)
    {
      _err("ERROR: PWM0 count out of range, count: %" PRIu32
           ", max: %d\n", info->count, PWM_MAX_COUNT);
      return -EINVAL;
    }

  priv->count = info->count;
  priv->handle = handle;

  ret = pulsecount_config(priv, info);
  if (ret == OK)
    {
      priv->frequency = pulsecount_frequency(info);
    }

  return ret;
}

static int pulsecount_stop(struct pulsecount_lowerhalf_s *dev)
{
  struct tlsr82_pulsecount_s *pulse = (struct tlsr82_pulsecount_s *)dev;
  return pulsecount_timer_stop(pulse->timer);
}

static int pulsecount_ioctl(struct pulsecount_lowerhalf_s *dev,
                            int cmd, unsigned long arg)
{
  struct tlsr82_pulsecount_s *pulse = (struct tlsr82_pulsecount_s *)dev;
  return pulsecount_timer_ioctl(pulse->timer, cmd, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tlsr82_pulsecountinitialize(const char *devpath, int minor)
{
  struct tlsr82_pulsecount_s *lower;
  int ret;

  tlsr82_pulsecount_hwinitialize();

  if (minor != 0)
    {
      _err("ERROR: PWM%d cannot support pulse count\n", minor);
      return -EINVAL;
    }

  lower = &g_pulsecount0lower;

  ret = pulsecount_cfg_check(lower->timer);
  if (ret < 0)
    {
      return ret;
    }

  ret = pulsecount_register(devpath,
                            (struct pulsecount_lowerhalf_s *)lower);
  if (ret < 0)
    {
      _err("%s has existed\n", devpath);
      return ret;
    }

  pulsecount_dumpregs("tlsr82_pulsecountinitialize");
  return ret;
}
