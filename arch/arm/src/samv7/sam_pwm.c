/****************************************************************************
 * arch/arm/src/samv7/sam_pwm.c
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
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/timers/pwm.h>

#include "arm_internal.h"
#include "chip.h"
#include "sam_config.h"
#include "sam_pwm.h"
#include "sam_periphclks.h"
#include "sam_gpio.h"
#include "hardware/sam_pwm.h"
#include "hardware/sam_pinmap.h"
#include "hardware/sam_pio.h"

#include <arch/board/board.h>

#include <sys/time.h>

#ifdef CONFIG_SAMV7_PWM

#ifdef CONFIG_PWM_NCHANNELS
# define PWM_NCHANNELS CONFIG_PWM_NCHANNELS
#else
# define PWM_NCHANNELS 1
#endif

#define CHANNEL_OFFSET 0x20
#define CLK_FREQ  BOARD_MCK_FREQUENCY
#define PWM_RES 65535

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_pwm_channel_s
{
  uint8_t channel;            /* Number of PWM module */
  bool used;                  /* True if the module is used */
  uint32_t pin;               /* PWM output pin */
};

struct sam_pwm_s
{
  const struct pwm_ops_s *ops;    /* PWM operations */
  FAR const struct sam_pwm_channel_s *channels;
  uint8_t channels_num;           /* Number of channels */
  uint32_t frequency;             /* PWM frequency */
  uint32_t base;                  /* Base address of peripheral register */
};

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

static const struct pwm_ops_s g_pwmops =
{
  .setup      = pwm_setup,
  .shutdown   = pwm_shutdown,
  .start      = pwm_start,
  .stop       = pwm_stop,
  .ioctl      = pwm_ioctl,
};

#ifdef CONFIG_SAMV7_PWM0

static struct sam_pwm_channel_s g_pwm0_channels[] =
{
#ifdef CONFIG_SAMV7_PWM0_CH0
  {
    .channel = 0,
    .used    = true,
    .pin     = GPIO_PWMC0_H0,
  },
#endif
#ifdef CONFIG_SAMV7_PWM0_CH1
  {
    .channel = 1,
    .used    = true,
    .pin     = GPIO_PWMC0_H1,
  },
#endif
#ifdef CONFIG_SAMV7_PWM0_CH2
  {
    .channel = 2,
    .used    = true,
    .pin     = GPIO_PWMC0_H2,
  },
#endif
#ifdef CONFIG_SAMV7_PWM0_CH3
  {
    .channel = 3,
    .used    = true,
    .pin     = GPIO_PWMC0_H3,
  },
#endif
};

static struct sam_pwm_s g_pwm0 =
{
  .ops = &g_pwmops,
  .channels = g_pwm0_channels,
  .channels_num = 4,
  .frequency = 0,
  .base = SAM_PWM0_BASE,
};
#endif /* CONFIG_SAMV7_PWM0 */

#ifdef CONFIG_SAMV7_PWM1

static struct sam_pwm_channel_s g_pwm1_channels[] =
{
#ifdef CONFIG_SAMV7_PWM1_CH0
  {
    .channel = 0,
    .used    = true,
    .pin     = GPIO_PWMC1_H0
  },
#endif
#ifdef CONFIG_SAMV7_PWM1_CH1
  {
    .channel = 1,
    .used    = true,
    .pin     = GPIO_PWMC1_H1
  },
#endif
#ifdef CONFIG_SAMV7_PWM1_CH2
  {
    .channel = 2,
    .used    = true,
    .pin     = GPIO_PWMC1_H2
  },
#endif
#ifdef CONFIG_SAMV7_PWM1_CH3
  {
    .channel = 3,
    .used    = true,
    .pin     = GPIO_PWMC1_H3
  },
#endif
}; /* CONFIG_SAMV7_PWM1 */

static struct sam_pwm_s g_pwm1 =
{
  .ops = &g_pwmops,
  .channels = g_pwm1_channels,
  .channels_num = 4,
  .frequency = 0,
  .base = SAM_PWM1_BASE,
};

#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void pwm_putreg(FAR struct sam_pwm_s *priv, uint32_t offset,
                       uint32_t value);
static uint32_t pwm_getreg(FAR struct sam_pwm_s *priv, uint32_t offset);

/* Helper functions */

static int pwm_set_output(FAR struct pwm_lowerhalf_s *dev, uint8_t channel,
                           ub16_t duty);
static int pwm_change_freq(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info, uint8_t channel);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void pwm_putreg(FAR struct sam_pwm_s *priv, uint32_t offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static uint32_t pwm_getreg(FAR struct sam_pwm_s *priv, uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: pwm_change_freq
 *
 * Description:
 *   Set timer frequency and change registers value to respect that
 *   frequency.
 *
 * Input Parameters:
 *   dev  - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_change_freq(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info, uint8_t channel)
{
  FAR struct sam_pwm_s *priv = (FAR struct sam_pwm_s *)dev;
#ifdef CONFIG_PWM_MULTICHAN
  uint8_t shift = info->channels[channel].channel - 1;
#else
  uint8_t shift = priv->channels[0].channel;
#endif
  uint32_t regval;
  uint32_t olddiv = pwm_getreg(priv, SAMV7_PWM_CPRDX
                               + (shift * CHANNEL_OFFSET));
  uint32_t newdiv = (uint32_t)((float)CLK_FREQ / info->frequency + 0.5f);
  uint32_t prescale = 0;

  while (newdiv > PWM_RES && prescale < 11)
    {
      newdiv = newdiv >> 1;
      prescale = prescale + 1;
    }

  if (newdiv > PWM_RES)
    {
      newdiv = PWM_RES;
    }
  else if (newdiv < 2)
    {
      newdiv = 2;
    }

  regval = pwm_getreg(priv, SAMV7_PWM_CMRX + (shift * CHANNEL_OFFSET));
  regval |= CMR_CPRE_SEL(prescale);
  pwm_putreg(priv, SAMV7_PWM_CMRX + (shift * CHANNEL_OFFSET), regval);

  pwm_putreg(priv, SAMV7_PWM_CPRDUPDX + (shift * CHANNEL_OFFSET),
             newdiv - 1);

  regval = pwm_getreg(priv, SAMV7_PWM_CDTYX + (shift * CHANNEL_OFFSET));
  regval = regval * newdiv / olddiv;
  pwm_putreg(priv, SAMV7_PWM_CDTYUPDX + (shift * CHANNEL_OFFSET),
             regval);

  return OK;
}

/****************************************************************************
 * Name: pwm_set_output
 *
 * Description:
 *   Set duty cycle and enable PWM output.
 *
 * Input Parameters:
 *   dev     - A reference to the lower half PWM driver state structure
 *   channel - Channel to by updated
 *   duty    - New duty
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_set_output(FAR struct pwm_lowerhalf_s *dev, uint8_t channel,
                           ub16_t duty)
{
  FAR struct sam_pwm_s *priv = (FAR struct sam_pwm_s *)dev;
  uint16_t period;
  uint16_t width;
  uint16_t regval;
  double duty_pct;
  uint8_t shift = channel;  /* Shift submodle offset addresses */

  /* Get the period value */

  period = pwm_getreg(priv, SAMV7_PWM_CPRDX + (shift * CHANNEL_OFFSET));

  /* Compute PWM width (count value to set PWM low) */

  duty_pct = (duty / 65536.0) * 100;
  width = (uint16_t)(((uint16_t)duty_pct * period) / 100);

  /* Update duty cycle */

  pwm_putreg(priv, SAMV7_PWM_CDTYUPDX + (shift * CHANNEL_OFFSET), width);

  regval = CHID_SEL(1 << shift);
  pwm_putreg(priv, SAMV7_PWM_ENA, regval);

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
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct sam_pwm_s *priv = (FAR struct sam_pwm_s *)dev;
  uint32_t pin = 0;
  uint32_t regval;

  /* Unlock User Interface */

  regval = WPCR_WPRCMD_SEL(0) | WPCR_WPRG_MASK | WPCR_WPKEY_SEL(0x50574d);
  pwm_putreg(priv, SAMV7_PWM_WPCR, regval);

  /* Disable all channels */

  regval = CHID_SEL(CHID_MASK);
  pwm_putreg(priv, SAMV7_PWM_DIS, regval);

  for (int i = 0; i < priv->channels_num; i++)
    {
      /* Configure the channel only if is set to be used */

      if (priv->channels[i].used != 1)
        {
          continue;
        }

      pin = priv->channels[i].pin;

      if (pin != 0)
        {
          sam_configgpio(pin);
        }

      sam_configgpio(pin);

      regval = CMR_CPOL | CMR_DPOLI;
      pwm_putreg(priv, SAMV7_PWM_CMRX + (i * CHANNEL_OFFSET), regval);

      /* Set duty cycle register */

      pwm_putreg(priv, SAMV7_PWM_CDTYX + (i * CHANNEL_OFFSET), 0);

      /* Set period register with default period */

      regval = 0x82b8;
      pwm_putreg(priv, SAMV7_PWM_CPRDX + (i * CHANNEL_OFFSET), regval);

      /* Reset Dead Time Register */

      pwm_putreg(priv, SAMV7_PWM_DTX + (i * CHANNEL_OFFSET), 0);

      /* Fault protection registers */

      pwm_putreg(priv, SAMV7_PWM_FPV1, 0);
      pwm_putreg(priv, SAMV7_PWM_FPV2, 0);
      pwm_putreg(priv, SAMV7_PWM_FPE, 0);

      /* Enable the channel */

      regval = CHID_SEL(1 << i);
      pwm_putreg(priv, SAMV7_PWM_ENA, regval);

      /* Set synchronous output */

      regval = SCM_SYNC_SEL(1 << i);
      pwm_putreg(priv, SAMV7_PWM_SCM, regval);
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
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct sam_pwm_s *priv = (FAR struct sam_pwm_s *)dev;
  uint32_t regval;

  /* Disable all channels and interrupts */

  regval = CHID_SEL(CHID_MASK);
  pwm_putreg(priv, SAMV7_PWM_DIS, regval);

  regval = IR1_CHID_SEL(CHID_MASK);
  pwm_putreg(priv, SAMV7_PWM_IDR1, regval);

  for (int i = 0; i < priv->channels_num; i++)
    {
      /* Skip modules that are not used */

      if (priv->channels[i].used != 1)
        {
          continue;
        }

      /* Reset period register */

      pwm_putreg(priv, SAMV7_PWM_CPRDX + (i * CHANNEL_OFFSET), 0);

      /* Reset duty cycle register */

      pwm_putreg(priv, SAMV7_PWM_CDTYX + (i * CHANNEL_OFFSET), 0);

      /* Reset Dead Time Register */

      pwm_putreg(priv, SAMV7_PWM_DTX + (i * CHANNEL_OFFSET), 0);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev  - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info)
{
  FAR struct sam_pwm_s *priv = (FAR struct sam_pwm_s *)dev;
  int ret = OK;

  /* Change frequency only if it is needed */

  if (info->frequency != priv->frequency)
    {
      for (int i = 0; i < PWM_NCHANNELS; i++)
        {
#ifdef CONFIG_PWM_MULTICHAN
          /* Break the loop if all following channels are not configured */

          if (info->channels[i].channel == -1)
            {
              break;
            }

          /* Configure the module freq only if is set to be used */

          if (info->channels[i].channel != 0)
            {
              ret = pwm_change_freq(dev, info, i);
            }
#else
          ret = pwm_change_freq(dev, info, i);
#endif
        }

      /* Save current frequency */

      if (ret == OK)
        {
          priv->frequency = info->frequency;
        }
    }

#ifdef CONFIG_PWM_MULTICHAN
  for (int i = 0; ret == OK && i < PWM_NCHANNELS; i++)
    {
      /* Break the loop if all following channels are not configured */

      if (info->channels[i].channel == -1)
        {
          break;
        }

      /* Enable PWM output for each channel */

      if (info->channels[i].channel != 0)
        {
          ret = pwm_set_output(dev, info->channels[i].channel - 1,
                                    info->channels[i].duty);
        }
    }
#else
  /* Enable PWM output just for first channel */

  ret = pwm_set_output(dev, priv->channels[0].channel, info->duty);

#endif /* CONFIG_PWM_MULTICHAN */

  /* Set sychnronous outputs */

  pwm_putreg(priv, SAMV7_PWM_SCUC, SCUC_UPDULOCK);

  return ret;
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
  FAR struct sam_pwm_s *priv = (FAR struct sam_pwm_s *)dev;
  uint8_t shift;
  uint32_t regval;

#ifdef CONFIG_PWM_MULTICHAN
  for (int i = 0; i < priv->channels_num; i++)
    {
      /* Skip settings if channel is not configured */

      if (!priv->channels[i].used)
        {
          continue;
        }

      shift = priv->channels[i].channel - 1;

      regval = CHID_SEL(1 << shift);
      pwm_putreg(priv, SAMV7_PWM_DIS, regval);
    }

#else
    shift = priv->channels[0].channel;

    regval = CHID_SEL(1 << shift);
    pwm_putreg(priv, SAMV7_PWM_DIS, regval);

#endif /* CONFIG_PWM_MULTICHAN */

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

static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sam_pwminitialize
 *
 * Description:
 *   Initialize the PWM channel for use with the upper level PWM driver.
 *
 * Input Parameters:
 *   channel - a number identifying the PWM channel.
 *
 * Returned Value:
 *   A pointer to the lower half PWM driver is returned on success,
 *   NULL on failure.
 *
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *sam_pwminitialize(int pwm)
{
  FAR struct sam_pwm_s *priv;

  pwminfo("Initializing pwm %d\n", pwm);

  switch (pwm)
  {
#ifdef CONFIG_SAMV7_PWM0
    case 0:
      sam_pwm0_enableclk();
      priv = &g_pwm0;
      break;
#endif
#ifdef CONFIG_SAMV7_PWM1
    case 1:
      sam_pwm1_enableclk();
      priv = &g_pwm1;
      break;
#endif
    default:
      pwmerr("ERROR: PWM number invalid or not configured %d\n", pwm);
      return NULL;
  }

  return (FAR struct pwm_lowerhalf_s *)priv;
}
#endif /* CONFIG_SAMV7_PWM */
