/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_pwm.c
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
#include <nuttx/timers/pwm.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "chip.h"
#include "arm_internal.h"
#include "cxd56_pinconfig.h"
#include "cxd56_clock.h"
#include "cxd56_pwm.h"

#if defined(CONFIG_CXD56_PWM0) || defined(CONFIG_CXD56_PWM1) || \
    defined(CONFIG_CXD56_PWM2) || defined(CONFIG_CXD56_PWM3)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PWM_REG_BASE        (0x04195600)
#define PWM_PHASE_REG_BASE  (0x04195630)

#define PWM_REG(ch) \
  ( \
    (pwm_reg_t*)(PWM_REG_BASE + (sizeof(pwm_reg_t) * (ch))) \
  )

#define PWM_PHASE_REG(ch) \
  ( \
    (pwm_phase_reg_t*) \
    (PWM_PHASE_REG_BASE + (sizeof(pwm_phase_reg_t) * (ch))) \
  )

#define PWM_PARAM_OFFPERIOD_SHIFT   (16)
#define PWM_PHASE_PRESCALE_SHIFT    (16)

#ifndef itemsof
#  define itemsof(array) (sizeof(array)/sizeof(array[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one PWM channel */

struct cxd56_pwm_chan_s
{
  const struct pwm_ops_s *ops;     /* PWM operations */
  uint8_t ch;                      /* PWM channel: {0..3} */
  uint8_t prescale;                /* prescale (reserved) */
};

typedef struct
{
  volatile uint32_t PARAM;
  volatile uint32_t EN;
  volatile uint32_t UPDATE;
} pwm_reg_t;

typedef struct
{
  volatile uint32_t PHASE;
} pwm_phase_reg_t;

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

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

#ifdef CONFIG_CXD56_PWM0
static struct cxd56_pwm_chan_s g_pwm_ch0 =
{
  .ops        = &g_pwmops,
  .ch         = CXD56_PWM_CH0,
  .prescale   = 0,
};
#endif

#ifdef CONFIG_CXD56_PWM1
static struct cxd56_pwm_chan_s g_pwm_ch1 =
{
  .ops        = &g_pwmops,
  .ch         = CXD56_PWM_CH1,
  .prescale   = 0,
};
#endif

#ifdef CONFIG_CXD56_PWM2
static struct cxd56_pwm_chan_s g_pwm_ch2 =
{
  .ops        = &g_pwmops,
  .ch         = CXD56_PWM_CH2,
  .prescale   = 0,
};
#endif

#ifdef CONFIG_CXD56_PWM3
static struct cxd56_pwm_chan_s g_pwm_ch3 =
{
  .ops        = &g_pwmops,
  .ch         = CXD56_PWM_CH3,
  .prescale   = 0,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_pin_config
 *
 * Description:
 *   Configure PWM pin
 *
 * Input Parameters:
 *   channel - pwm channel number.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int pwm_pin_config(uint32_t channel)
{
  int ret = 0;
  uint32_t pingroupa[] = PINCONFS_PWMA;
  uint32_t pingroupb[] = PINCONFS_PWMB;

  if ((channel == CXD56_PWM_CH0) || (channel == CXD56_PWM_CH1))
    {
      ret = cxd56_pin_configs(pingroupa, itemsof(pingroupa));
    }
  else
    {
      ret = cxd56_pin_configs(pingroupb, itemsof(pingroupb));
    }

  return ret;
}

/****************************************************************************
 * Name: convert_freq2period
 *
 * Description:
 *   Convert frequency and duty to period and offperiod of param register.
 *
 * Input Parameters:
 *   freq   - pwm frequency [Hz]
 *   duty   - duty
 *
 * Output Parameters:
 *   param  - set value of PWM_PARAM register
 *   phase  - set value of PWM_PHASE register
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int convert_freq2period(uint32_t freq, ub16_t duty, uint32_t *param,
                               uint32_t *phase)
{
  DEBUGASSERT(param);
  DEBUGASSERT(phase);

  uint32_t pwmfreq = 0;
  uint32_t period = 0;
  uint32_t offperiod = 0;
  uint32_t prescale = 0;

  /* Get frequency of pwm base clock */

  pwmfreq = cxd56_get_pwm_baseclock();
  if (pwmfreq == 0)
    {
      pwmerr("Unknown pwm frequency\n");
      return -1;
    }

  /* check frequency range */

  if ((freq > ((pwmfreq + 1) >> 1)) || (freq <= 0))
    {
      pwmerr("Frequency out of range. %" PRId32
             " [Effective range:%d - %" PRId32 "]\n",
             freq, 1, (pwmfreq + 1) >> 1);
      return -1;
    }

  /* check duty range */

  if ((duty < 0x00000001) || (duty > 0x0000ffff))
    {
      pwmerr("Duty out of range. %" PRId32 "\n", duty);
      return -1;
    }

  /* calcurate prescale */

  if ((freq << 8) < (pwmfreq >> 8))
    {
      for (prescale = 1; prescale <= 8; prescale++)
        {
          if (freq > ((pwmfreq >> prescale) / 65535))
            {
              break;
            }
        }
    }

  /* calculate period and offperiod */

  if (prescale > 0)
    {
      period = (((pwmfreq * 10) >> prescale) / freq + 5) / 10;
    }
  else
    {
      period = (pwmfreq * 10 / freq - 5) / 10;
    }

  if (period > 0xffff)
    {
      period = 0xffff;
    }

  if (prescale > 0)
    {
      offperiod = ((0x10000 - duty) * period + (1 << (16 - prescale))) >> 16;
      if (offperiod < 2)
        {
          pwmerr("Duty out of range. %" PRId32 "\n", duty);
          return -1;
        }
    }
  else
    {
      offperiod = ((0x10000 - duty) * (period + 1) + 0x8000) >> 16;
    }

  if (period < offperiod)
    {
      offperiod = period;
    }

  pwminfo("Cycle = %" PRId32 ", Low = %" PRId32
          ", High = %" PRId32 ", Clock = %" PRId32 " Hz\n",
          (prescale) ? (period << prescale) : period + 1,
          (prescale) ? (offperiod << prescale) - 1 : offperiod,
          (prescale) ? (period << prescale) - (offperiod << prescale) + 1
                     : period + 1 - offperiod, pwmfreq);
  pwminfo("period/off/on = 0x%04" PRIx32 "/0x%04" PRIx32
          "/0x%04" PRIx32 ", prescale = %" PRId32 "\n",
          period, offperiod, period - offperiod, prescale);

  *param = (period & 0xffff) |
           ((offperiod & 0xffff) << PWM_PARAM_OFFPERIOD_SHIFT);
  *phase = prescale << PWM_PHASE_PRESCALE_SHIFT;

  return OK;
}

/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct cxd56_pwm_chan_s *priv = (struct cxd56_pwm_chan_s *)dev;
  int ret;

  ret = pwm_pin_config(priv->ch);
  if (ret < 0)
    {
      pwmerr("Failed to pinconf() channel: %d\n", priv->ch);
      return -EINVAL;
    }

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
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct cxd56_pwm_chan_s *priv = (struct cxd56_pwm_chan_s *)dev;
  uint32_t param;
  uint32_t phase;
  int ret;

  if (info->duty <= 0)
    {
      /* Output low level if duty cycle is almost 0% */

      PWM_REG(priv->ch)->EN = 0x0;
    }
  else if (info->duty >= 65536)
    {
      /* Output high level if duty cycle is almost 100% */

      PWM_REG(priv->ch)->PARAM = 1;
      PWM_REG(priv->ch)->EN = 0x1;
    }
  else
    {
      ret = convert_freq2period(info->frequency, info->duty, &param, &phase);
      if (ret < 0)
        {
          return -EINVAL;
        }

      if (PWM_REG(priv->ch)->EN & 1)
        {
          /* Change duty cycle dynamically if already running */

          PWM_REG(priv->ch)->PARAM = param;
          return OK;
        }

      PWM_REG(priv->ch)->EN = 0x0;
      PWM_REG(priv->ch)->PARAM = param;
      PWM_PHASE_REG(priv->ch)->PHASE = phase;

      PWM_REG(priv->ch)->EN = 0x1;
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct cxd56_pwm_chan_s *priv = (struct cxd56_pwm_chan_s *)dev;

  PWM_REG(priv->ch)->EN = 0x0;

  return OK;
}

/****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_pwminitialize
 *
 * Description:
 *   Initialize PWM channel for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   channel - pwm channel number.
 *
 * Returned Value:
 *   On success, a pointer to the CXD56 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *cxd56_pwminitialize(uint32_t channel)
{
  struct cxd56_pwm_chan_s *pwmch;

  switch (channel)
    {
#ifdef CONFIG_CXD56_PWM0
      case CXD56_PWM_CH0:
        pwmch = &g_pwm_ch0;
        break;
#endif
#ifdef CONFIG_CXD56_PWM1
      case CXD56_PWM_CH1:
        pwmch = &g_pwm_ch1;
        break;
#endif
#ifdef CONFIG_CXD56_PWM2
      case CXD56_PWM_CH2:
        pwmch = &g_pwm_ch2;
        break;
#endif
#ifdef CONFIG_CXD56_PWM3
      case CXD56_PWM_CH3:
        pwmch = &g_pwm_ch3;
        break;
#endif
      default:
        pwmerr("Illeagal channel number:%" PRId32 "\n", channel);
        return NULL;
    }

  return (struct pwm_lowerhalf_s *)pwmch;
}

#endif
