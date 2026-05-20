/****************************************************************************
 * arch/arm/src/tiva/common/tiva_pulsecount.c
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

#include <stdio.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/timers/pulsecount.h>

#include "arm_internal.h"
#include "tiva_gpio.h"
#include "tiva_pulsecount.h"
#include "tiva_enablepwr.h"
#include "tiva_enableclks.h"

#include "hardware/tiva_pwm.h"
#include "hardware/tiva_pinmap.h"
#include "hardware/tiva_memorymap.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

uint32_t g_pulsecount_pinset[] =
{
  GPIO_M0_PWM0,
  GPIO_M0_PWM1,
  GPIO_M0_PWM2,
  GPIO_M0_PWM3,
  GPIO_M0_PWM4,
  GPIO_M0_PWM5,
  GPIO_M0_PWM6,
  GPIO_M0_PWM7,
};

struct tiva_pulsecount_chan_s
{
  const struct pulsecount_ops_s *ops;
  uint8_t controller_id;
  uintptr_t controller_base;
  uint8_t generator_id;
  uintptr_t generator_base;
  uint8_t channel_id;
  bool inited;
  uint8_t irq;
  uint32_t count;
  uint32_t cur_count;
  void *handle;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN0
static int tiva_pulsecount_gen0_interrupt(int irq,
                                   void *context, void *arg);
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN2
static int tiva_pulsecount_gen1_interrupt(int irq,
                                   void *context, void *arg);
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN4
static int tiva_pulsecount_gen2_interrupt(int irq,
                                   void *context, void *arg);
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN6
static int tiva_pulsecount_gen3_interrupt(int irq,
                                   void *context, void *arg);
#endif

#if defined(CONFIG_TIVA_PULSECOUNT0_CHAN0) || defined(CONFIG_TIVA_PULSECOUNT0_CHAN2) || \
    defined(CONFIG_TIVA_PULSECOUNT0_CHAN4) || defined(CONFIG_TIVA_PULSECOUNT0_CHAN6)
static int tiva_pulsecount_interrupt(struct tiva_pulsecount_chan_s *chan);
#endif

static inline void
tiva_pulsecount_putreg(struct tiva_pulsecount_chan_s *chan,
                       unsigned int offset, uint32_t regval);
static inline uint32_t
tiva_pulsecount_getreg(struct tiva_pulsecount_chan_s *chan,
                       unsigned int offset);
static inline int tiva_pulsecount_timer(struct tiva_pulsecount_chan_s *chan,
                                 const struct pulsecount_info_s *info);

static int tiva_pulsecount_setup(struct pulsecount_lowerhalf_s *dev);
static int tiva_pulsecount_shutdown(struct pulsecount_lowerhalf_s *dev);
static int tiva_pulsecount_start(struct pulsecount_lowerhalf_s *dev,
                          const struct pulsecount_info_s *info,
                          void *handle);
static int tiva_pulsecount_stop(struct pulsecount_lowerhalf_s *dev);
static int tiva_pulsecount_ioctl(struct pulsecount_lowerhalf_s *dev,
                          int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_pulsecount_freq = 1875000;
static uint32_t g_pulsecount_counter = (1 << 16);

static const struct pulsecount_ops_s g_pulsecount_ops =
{
  .setup    = tiva_pulsecount_setup,
  .shutdown = tiva_pulsecount_shutdown,
  .start    = tiva_pulsecount_start,
  .stop     = tiva_pulsecount_stop,
  .ioctl    = tiva_pulsecount_ioctl,
};

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN0
static struct tiva_pulsecount_chan_s g_pulsecount_chan0 =
{
  .ops             = &g_pulsecount_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 0,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMN_BASE +
                     TIVA_PWMN_INTERVAL * 0,
  .channel_id      = 0,
  .inited          = false,
  .irq             = TIVA_IRQ_PWM0_GEN0,
  .count           = 0,
  .cur_count       = 0,
  .handle          = NULL,
};
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN1
static struct tiva_pulsecount_chan_s g_pulsecount_chan1 =
{
  .ops             = &g_pulsecount_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 0,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMN_BASE +
                     TIVA_PWMN_INTERVAL * 0,
  .channel_id      = 1,
  .inited          = false,
  .irq             = TIVA_IRQ_PWM0_GEN0,
  .count           = 0,
  .cur_count       = 0,
  .handle          = NULL,
};
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN2
static struct tiva_pulsecount_chan_s g_pulsecount_chan2 =
{
  .ops             = &g_pulsecount_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 1,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMN_BASE +
                     TIVA_PWMN_INTERVAL * 1,
  .channel_id      = 2,
  .inited          = false,
  .irq             = TIVA_IRQ_PWM0_GEN1,
  .count           = 0,
  .cur_count       = 0,
  .handle          = NULL,
};
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN3
static struct tiva_pulsecount_chan_s g_pulsecount_chan3 =
{
  .ops             = &g_pulsecount_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 1,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMN_BASE +
                     TIVA_PWMN_INTERVAL * 1,
  .channel_id      = 3,
  .inited          = false,
  .irq             = TIVA_IRQ_PWM0_GEN1,
  .count           = 0,
  .cur_count       = 0,
  .handle          = NULL,
};
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN4
static struct tiva_pulsecount_chan_s g_pulsecount_chan4 =
{
  .ops             = &g_pulsecount_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 2,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMN_BASE +
                     TIVA_PWMN_INTERVAL * 2,
  .channel_id      = 4,
  .inited          = false,
  .irq             = TIVA_IRQ_PWM0_GEN2,
  .count           = 0,
  .cur_count       = 0,
  .handle          = NULL,
};
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN5
static struct tiva_pulsecount_chan_s g_pulsecount_chan5 =
{
  .ops             = &g_pulsecount_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 2,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMN_BASE +
                     TIVA_PWMN_INTERVAL * 2,
  .channel_id      = 5,
  .inited          = false,
  .irq             = TIVA_IRQ_PWM0_GEN2,
  .count           = 0,
  .cur_count       = 0,
  .handle          = NULL,
};
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN6
static struct tiva_pulsecount_chan_s g_pulsecount_chan6 =
{
  .ops             = &g_pulsecount_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 3,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMN_BASE +
                     TIVA_PWMN_INTERVAL * 3,
  .channel_id      = 6,
  .inited          = false,
  .irq             = TIVA_IRQ_PWM0_GEN3,
  .count           = 0,
  .cur_count       = 0,
  .handle          = NULL,
};
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN7
static struct tiva_pulsecount_chan_s g_pulsecount_chan7 =
{
  .ops             = &g_pulsecount_ops,
  .controller_id   = 0,
  .controller_base = TIVA_PWM0_BASE,
  .generator_id    = 3,
  .generator_base  = TIVA_PWM0_BASE + TIVA_PWMN_BASE +
                     TIVA_PWMN_INTERVAL * 3,
  .channel_id      = 7,
  .inited          = false,
  .irq             = TIVA_IRQ_PWM0_GEN3,
  .count           = 0,
  .cur_count       = 0,
  .handle          = NULL,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_pulsecount_gen[n]_interrupt
 *
 * Description:
 *   Pulse count interrupt handlers for PWM[n]
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN0
static int tiva_pulsecount_gen0_interrupt(int irq, void *context, void *arg)
{
  return tiva_pulsecount_interrupt(&g_pulsecount_chan0);
}
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN2
static int tiva_pulsecount_gen1_interrupt(int irq, void *context, void *arg)
{
  return tiva_pulsecount_interrupt(&g_pulsecount_chan2);
}
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN4
static int tiva_pulsecount_gen2_interrupt(int irq, void *context, void *arg)
{
  return tiva_pulsecount_interrupt(&g_pulsecount_chan4);
}
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN6
static int tiva_pulsecount_gen3_interrupt(int irq, void *context, void *arg)
{
  return tiva_pulsecount_interrupt(&g_pulsecount_chan6);
}
#endif

/****************************************************************************
 * Name: tiva_pulsecount_interrupt
 *
 * Description:
 *   Common pulse count interrupt handler.
 *
 ****************************************************************************/

#if defined(CONFIG_TIVA_PULSECOUNT0_CHAN0) || defined(CONFIG_TIVA_PULSECOUNT0_CHAN2) || \
    defined(CONFIG_TIVA_PULSECOUNT0_CHAN4) || defined(CONFIG_TIVA_PULSECOUNT0_CHAN6)
static int tiva_pulsecount_interrupt(struct tiva_pulsecount_chan_s *chan)
{
  /* Clear interrupt */

  tiva_pulsecount_putreg(chan, TIVA_PWMN_ISC_OFFSET, INT_SET << INTCMPAD);

  /* Count down current pulse count */

  chan->cur_count--;

  /* Disable generator and reload current pulse count */

  if (chan->cur_count == 0)
    {
      tiva_pulsecount_putreg(chan, TIVA_PWMN_CTL_OFFSET,
                      CTL_DISABLE << TIVA_PWMN_CTL_ENABLE);
      chan->cur_count = chan->count;
      pulsecount_expired(chan->handle);
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: tiva_pulsecount_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t
tiva_pulsecount_getreg(struct tiva_pulsecount_chan_s *chan,
                       unsigned int offset)
{
  uintptr_t regaddr = chan->generator_base + offset;
  return getreg32(regaddr);
}

/****************************************************************************
 * Name: tiva_pulsecount_putreg
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void
tiva_pulsecount_putreg(struct tiva_pulsecount_chan_s *chan,
                       unsigned int offset, uint32_t regval)
{
  uintptr_t regaddr = chan->generator_base + offset;
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: tiva_pulsecount_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   will be configured and initialized the device so that it is ready for
 *   use.  It will not, however, output pulses until the start method is
 *   called.
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_pulsecount_setup(struct pulsecount_lowerhalf_s *dev)
{
  struct tiva_pulsecount_chan_s *chan = (struct tiva_pulsecount_chan_s *)dev;
  _info("setup pulsecount for channel %d\n", chan->channel_id);

  /* Enable GPIO port, GPIO pin type and GPIO alternate function (refer to
   * TM4C1294NCPDT 23.4.2-4)
   */

  int ret = tiva_configgpio(g_pulsecount_pinset[chan->channel_id]);
  if (ret < 0)
    {
      _err("ERROR: tiva_configgpio failed (%x)\n",
             g_pulsecount_pinset[chan->channel_id]);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: tiva_pulsecount_shutdown
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

static int tiva_pulsecount_shutdown(struct pulsecount_lowerhalf_s *dev)
{
  struct tiva_pulsecount_chan_s *chan = (struct tiva_pulsecount_chan_s *)dev;
  _info("shutdown pulsecount for channel %d\n", chan->channel_id);

  /* Remove unused-variable warning */

  UNUSED(chan);

  /* Ensure the PWM channel has been stopped */

  tiva_pulsecount_stop(dev);

  return OK;
}

/****************************************************************************
 * Name: tiva_pulsecount_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev  - A reference to the lower half pulsecount driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *   handle - This is the handle that was provided to the lower-half
 *            start() method.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tiva_pulsecount_start(struct pulsecount_lowerhalf_s *dev,
                          const struct pulsecount_info_s *info,
                          void *handle)
{
  struct tiva_pulsecount_chan_s *chan = (struct tiva_pulsecount_chan_s *)dev;
  _info("start pulsecount for channel %d\n", chan->channel_id);

  /* Save the handle */

  chan->handle = handle;

  /* Load pulse count and current pulse count
   *
   * Workaround:
   *   Count should be add 1 for the first time
   */

  chan->count = info->count;
  chan->cur_count = info->count;

  if (!chan->inited)
    {
      chan->count++;
      chan->cur_count++;
      chan->inited = true;
    }

  /* Count 0 means to generate indefinite number of pulses */

  if (info->count == 0)
    {
      pulsecount_expired(chan->handle);

      /* Disable interrupt */

      uint32_t enable = getreg32(chan->controller_base +
                                 TIVA_PWM_INTEN_OFFSET);
      enable &= ~(INT_ENABLE << chan->generator_id);
      putreg32(enable, chan->controller_base + TIVA_PWM_INTEN_OFFSET);
    }
  else
    {
      /* Enable interrupt */

      uint32_t enable = getreg32(chan->controller_base +
                                 TIVA_PWM_INTEN_OFFSET);
      enable |= (INT_ENABLE << chan->generator_id);
      putreg32(enable, chan->controller_base + TIVA_PWM_INTEN_OFFSET);
    }

  /* Start the timer */

  return tiva_pulsecount_timer(chan, info);
}

/****************************************************************************
 * Name: tiva_pulsecount_timer
 *
 * Description:
 *   Configure PWM registers and start pulsecount
 *
 * Input Parameters:
 *   dev  - A reference to the lower half pulsecount driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static inline int tiva_pulsecount_timer(struct tiva_pulsecount_chan_s *chan,
                                 const struct pulsecount_info_s *info)
{
  uint16_t duty = pulsecount_duty(info);
  uint32_t frequency = pulsecount_frequency(info);

  _info("> high = %" PRIu32 " ns\n", info->high_ns);
  _info("> low = %" PRIu32 " ns\n", info->low_ns);
  _info("> frequency = %" PRIu32 "\n", frequency);
  _info("> duty = %u\n", duty);

  /* Configure PWM countdown mode (refer to TM4C1294NCPDT 23.4.6) */

  tiva_pulsecount_putreg(chan, TIVA_PWMN_CTL_OFFSET, 0);

  if (chan->channel_id % 2 == 0)
    {
      tiva_pulsecount_putreg(chan, TIVA_PWMN_GENA_OFFSET,
                      GENX_LOW << TIVA_PWMN_GENX_ACTCMPAD |
                      GENX_HIGH << TIVA_PWMN_GENX_ACTLOAD);
    }
  else
    {
      tiva_pulsecount_putreg(chan, TIVA_PWMN_GENB_OFFSET,
                      GENX_LOW << TIVA_PWMN_GENX_ACTCMPBD |
                      GENX_HIGH << TIVA_PWMN_GENX_ACTLOAD);
    }

  /* Set the PWM period (refer to TM4C1294NCPDT 23.4.7) */

  uint32_t pulsecount_min_freq =
    (uint32_t)(g_pulsecount_freq / g_pulsecount_counter) + 1;
  uint32_t pulsecount_max_freq = g_pulsecount_freq;
  uint32_t load = (uint32_t)(g_pulsecount_freq / frequency);

  _info("> load = %u (%08x)\n", load, load);

  if (load >= g_pulsecount_counter || load < 1)
    {
      _err("ERROR: frequency should be in [%d, %d] Hz\n",
              pulsecount_min_freq, pulsecount_max_freq);
      return -ERANGE;
    }

  tiva_pulsecount_putreg(chan, TIVA_PWMN_LOAD_OFFSET, load - 1);

  /* Configure PWM duty (refer to TM4C1294NCPDT 23.4.8-9)
   *
   * Workaround:
   *   When comp equals to load, the signal is never pulled down,
   *   so let comp equals to (comp-1)
   */

  uint32_t comp =
    (uint32_t)((1 - (float)duty / g_pulsecount_counter) * load);
  comp = (duty == 0) ? (comp - 1) : (comp);
  _info("> comp = %u (%08x)\n", comp, comp);

  if (chan->channel_id % 2 == 0)
    {
      tiva_pulsecount_putreg(chan, TIVA_PWMN_CMPA_OFFSET, comp - 1);
    }
  else
    {
      tiva_pulsecount_putreg(chan, TIVA_PWMN_CMPB_OFFSET, comp - 1);
    }

  /* Enable the PWM generator (refer to TM4C1294NCPDT 23.4.10) */

  tiva_pulsecount_putreg(chan,
                  TIVA_PWMN_CTL_OFFSET,
                  CTL_ENABLE << TIVA_PWMN_CTL_ENABLE);

  /* Enable PWM channel (refer to TM4C1294NCPDT 23.4.11) */

  uint32_t enable = getreg32(chan->controller_base + TIVA_PWM_ENABLE_OFFSET);
  enable |= (1 << chan->channel_id);
  putreg32(enable, chan->controller_base + TIVA_PWM_ENABLE_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: tiva_pulsecount_stop
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

static int tiva_pulsecount_stop(struct pulsecount_lowerhalf_s *dev)
{
  struct tiva_pulsecount_chan_s *chan = (struct tiva_pulsecount_chan_s *)dev;
  _info("stop pulsecount for channel %d\n", chan->channel_id);

  /* Disable PWM channel */

  uint32_t value = getreg32(chan->controller_base + TIVA_PWM_ENABLE_OFFSET);
  value &= ~(1 << chan->channel_id);
  putreg32(value, chan->controller_base + TIVA_PWM_ENABLE_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: tiva_pulsecount_ioctl
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

static int tiva_pulsecount_ioctl(struct pulsecount_lowerhalf_s *dev, int cmd,
                          unsigned long arg)
{
  struct tiva_pulsecount_chan_s *chan = (struct tiva_pulsecount_chan_s *)dev;
  _info("ioctl pulsecount for channel %d\n", chan->channel_id);

  /* Remove unused-variable warning */

  UNUSED(chan);

  /* There are no platform-specific ioctl commands */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_pulsecount_initialize
 *
 * Description:
 *   Initialize one channel for use with the upper-level pulsecount driver.
 *
 * Input Parameters:
 *   channel - A number identifying the pulsecount channel to use.
 *
 * Returned Value:
 *   On success, a pointer to the Tiva lower half pulsecount driver is
 *   returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pulsecount_lowerhalf_s *tiva_pulsecount_initialize(int channel)
{
  ASSERT(channel >= 0 && channel <= 7);
  struct tiva_pulsecount_chan_s *chan;

  switch (channel)
    {
#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN0
    case 0:
      chan = &g_pulsecount_chan0;
      break;
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN1
    case 1:
      chan = &g_pulsecount_chan1;
      break;
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN2
    case 2:
      chan = &g_pulsecount_chan2;
      break;
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN3
    case 3:
      chan = &g_pulsecount_chan3;
      break;
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN4
    case 4:
      chan = &g_pulsecount_chan4;
      break;
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN5
    case 5:
      chan = &g_pulsecount_chan5;
      break;
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN6
    case 6:
      chan = &g_pulsecount_chan6;
      break;
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN7
    case 7:
      chan = &g_pulsecount_chan7;
      break;
#endif

    default:
      _err("ERROR: invalid channel %d\n", channel);
      return NULL;
    }

  _info("channel %d:\n", channel);
  _info("> channel_id = %d\n", chan->channel_id);
  _info("> controller_id = %d\n", chan->controller_id);
  _info("> controller_base = %08x\n", chan->controller_base);
  _info("> generator_id = %d\n", chan->generator_id);
  _info("> generator_base = %08x\n", chan->generator_base);

  /* Enable PWM controller (refer to TM4C1294NCPDT 23.4.1) */

  ASSERT(chan->controller_id == 0);
  tiva_pwm_enablepwr(chan->controller_id);
  tiva_pwm_enableclk(chan->controller_id);

  /* Configure PWM Clock Configuration (refer to TM4C1294NCPDT 23.4.5)
   *
   * On TM4C1294NCPDT, configure the PWM clock source as 1.875MHz (the system
   * clock 120MHz divided by 64)
   *
   * Keep the existing fixed divider/load selection in this split.
   */

  putreg32(CC_USEPWM << TIVA_PWM_CC_USEPWM |
           CC_PWMDIV_64 << TIVA_PWM_CC_PWMDIV,
           chan->controller_base + TIVA_PWM_CC);

  /* Enable interrupt INTCMPAD mode */

  tiva_pulsecount_putreg(chan, TIVA_PWMN_INTEN_OFFSET, INT_SET << INTCMPAD);

  /* Attach IRQ handler and enable interrupt */

  switch (chan->channel_id)
    {
#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN0
      case 0:
        irq_attach(chan->irq, tiva_pulsecount_gen0_interrupt, NULL);
        up_enable_irq(chan->irq);
        break;
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN2
      case 2:
        irq_attach(chan->irq, tiva_pulsecount_gen1_interrupt, NULL);
        up_enable_irq(chan->irq);
        break;
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN4
      case 4:
        irq_attach(chan->irq, tiva_pulsecount_gen2_interrupt, NULL);
        up_enable_irq(chan->irq);
        break;
#endif

#ifdef CONFIG_TIVA_PULSECOUNT0_CHAN6
      case 6:
        irq_attach(chan->irq, tiva_pulsecount_gen3_interrupt, NULL);
        up_enable_irq(chan->irq);
        break;
#endif
    }

  return (struct pulsecount_lowerhalf_s *)chan;
}
