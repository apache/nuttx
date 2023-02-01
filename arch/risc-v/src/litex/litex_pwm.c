/****************************************************************************
 * arch/risc-v/src/litex/litex_pwm.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "litex_pwm.h"
#include "litex_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_PWM_PULSECOUNT
#error PWM puslecount not supported for Litex.
#endif
#ifdef CONFIG_PWM_MULTICHAN
#error PWM multichannel not supported for Litex. 
#endif

/* Control register offsets from peripheral base address */

#define PWM_ENABLE_REG_OFFSET       0 /* Enable register */
#define PWM_WIDTH_REG_OFFSET        4 /* Pulse width control register */
#define PWM_PERIOD_REG_OFFSET       8 /* Period register*/

/* Enable register bit definitions */

#define PWM_ENABLE_SET_BIT          1 /* Bit used to enable/disable PWM */

/* The minimum period required for *sane* operation. This ensures that
 * setting the duty cycle actually makes sense. However, it does limit
 * the maximum PWM frequency.
 */

#define PWM_MINIMUM_PERIOD 10

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct litex_pwm_s
{
  const struct pwm_ops_s *ops;  /* PWM operations */
  uint32_t base;                /* Base address of PWM register */
  uint32_t frequency;           /* The current frequency  */
  uint32_t duty;                /* The current duty cycle */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* PWM driver methods needed by lower half driver operations */

static int litex_pwm_setup(struct pwm_lowerhalf_s *dev);
static int litex_pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int litex_pwm_start(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info);
static int litex_pwm_stop(struct pwm_lowerhalf_s *dev);
static int litex_pwm_ioctl(struct pwm_lowerhalf_s *dev,
                           int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver.
 */

static const struct pwm_ops_s g_litex_pwmops =
{
  .setup       = litex_pwm_setup,
  .shutdown    = litex_pwm_shutdown,
  .start       = litex_pwm_start,
  .stop        = litex_pwm_stop,
  .ioctl       = litex_pwm_ioctl,
};

/* Data structure containing the operations and base address for all enabled
 * peripherals.
 */

struct litex_pwm_s g_litex_pwm_inst[] =
{ [0 ... LITEX_PWM_MAX]
  {
    .ops = &g_litex_pwmops,
    .base = 0,
    .frequency = 0,
    .duty = 0
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: litex_pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - A pointer to the lower half instance to operate on.
 *
 * Returned Value:
 *   OK on success. A negated error number is returned on failure.
 *
 ****************************************************************************/

static int litex_pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct litex_pwm_s *priv = (struct litex_pwm_s *)dev;
  int ret  = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv->base);

  /* Just make sure that the device is not going to output anything */

  putreg32(0, priv->base + PWM_ENABLE_REG_OFFSET);

  return ret;
}

/****************************************************************************
 * Name: litex_pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - A pointer to the lower half instance to operate on.
 *
 * Returned Value:
 *   OK on success. A negated error number is returned on failure.
 *
 ****************************************************************************/

static int litex_pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct litex_pwm_s *priv = (struct litex_pwm_s *)dev;
  int                 ret  = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv->base);

  /* Disable PWM output */

  putreg32(0, priv->base + PWM_ENABLE_REG_OFFSET);

  return ret;
}

/****************************************************************************
 * Name: litex_pwm_start
 *
 * Description:
 *   (Re-)initialize the PWM and start the pulsed output
 *
 * Input Parameters:
 *   dev  - A pointer to the lower half instance to operate on.
 *   info - Structure containing the desired PWM characteristics.
 *
 * Returned Value:
 *   OK on success. A negated error number is returned on failure.
 *
 ****************************************************************************/

static int litex_pwm_start(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info)
{
  struct litex_pwm_s *priv = (struct litex_pwm_s *)dev;
  int ret = OK;
  uint32_t sysclk_freq;
  uint32_t period;
  uint32_t duty;
  bool update_frequency;
  bool update_duty;
  const uint32_t max_in_duty = 65536;
  const uint32_t min_in_duty = 1;

  update_frequency = priv->frequency != info->frequency;
  update_duty = update_frequency | (priv->duty != info->duty);

  DEBUGASSERT(dev);
  DEBUGASSERT(priv->base);

  if (update_frequency)
    {
      if (info->frequency == 0)
        {
          pwmwarn("Cannot set PMW to a frequency of 0Hz\n");
          return -EPERM;
        }

      /* Calculate the period for the required frequency */

      sysclk_freq = litex_get_hfclk();

      period = sysclk_freq / info->frequency;
      if (period < PWM_MINIMUM_PERIOD)
        {
          pwmwarn("Frequency %luHz too high for sysclk %luHz\n",
              info->frequency, sysclk_freq);
          return -EPERM;
        }

      priv->frequency = info->frequency;

      putreg32(period, priv->base + PWM_PERIOD_REG_OFFSET);
      pwminfo("Update PWM period to %lu\n", period);
    }

  if (update_duty)
    {
      /* Map the duty cycle compare to the period */

      /* The period may have already been calculated when adjusting the
       * frequency. However, if the frequency doesn't change, it will be
       * set to an undefined value. Just fetch it each time from hardware.
       */

      period = getreg32(priv->base + PWM_PERIOD_REG_OFFSET);
      duty = period * (info->duty / (float)(max_in_duty - min_in_duty));
      priv->duty = info->duty;

      putreg32(duty, priv->base + PWM_WIDTH_REG_OFFSET);
      pwminfo("Update PWM duty to %lu\n", duty);
    }

  putreg32(PWM_ENABLE_SET_BIT, priv->base + PWM_ENABLE_REG_OFFSET);
  return ret;
}

/****************************************************************************
 * Name: litex_pwm_stop
 *
 * Description:
 *   Stop the PWM
 *
 * Input Parameters:
 *   dev  - A pointer to the lower half instance to operate on.
 *
 * Returned Value:
 *   OK on success. A negated error number is returned on failure.
 *
 ****************************************************************************/

static int litex_pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct litex_pwm_s *priv = (struct litex_pwm_s *)dev;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv->base);

  putreg32(0, priv->base + PWM_ENABLE_REG_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: litex_pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands.
 *   Not implemented for Litex.
 *
 * Input Parameters:
 *   dev  - A pointer to the lower half instance to operate on.
 *   cmd  - IO control command.
 *   arg  - IO control command argument.
 *
 * Returned Value:
 *   OK on success. A negated error number is returned on failure.
 *
 ****************************************************************************/

static int litex_pwm_ioctl(struct pwm_lowerhalf_s *dev,
                           int cmd, unsigned long arg)
{
  /* There are no platform-specific ioctl commands */

  UNUSED(dev);
  UNUSED(cmd);
  UNUSED(arg);

  return -ENOTTY;
}

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * Name: litex_pwminitialize
 *
 * Description:
 *   Initialize one PWM channel
 *
 * Input Parameters:
 *   pwm - A number identifying the pwm instance.
 *
 * Returned Value:
 *   On success, a pointer to the Litex lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *litex_pwminitialize(int pwm)
{
  struct litex_pwm_s *lower = NULL;
  if (pwm >= LITEX_PWM_MAX)
    {
      return NULL;
    }

  lower = &g_litex_pwm_inst[pwm];
  lower->base = LITEX_PWM_BASE + (LITEX_PWM_OFFSET * pwm);

  return (struct pwm_lowerhalf_s *)lower;
}
