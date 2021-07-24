/****************************************************************************
 * arch/arm/src/nrf52/nrf52_pwm.c
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

#include "arm_internal.h"
#include "arm_arch.h"

#include "nrf52_gpio.h"
#include "nrf52_pwm.h"

#include "hardware/nrf52_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default PWM polarity */

#define PWM_POLARITY_DEFAULT (PWM_DECODER_POL_FALLING)

/* Sequence 0 length */

#define PWM_SEQ0_LEN         (4)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52_pwm_s
{
  FAR const struct pwm_ops_s *ops;       /* PWM operations */
  uint32_t                    base;      /* Base address of PWM register */
  uint32_t                    frequency; /* Current frequency setting */
  uint32_t                    cntrtop;   /* Counter top */
  uint32_t                    ch0_pin;   /* Channel 1 pin */
  uint32_t                    ch1_pin;   /* Channel 2 pin */
  uint32_t                    ch2_pin;   /* Channel 3 pin */
  uint32_t                    ch3_pin;   /* Channel 4 pin */

  /* Sequence 0 */

  uint16_t                    seq0[PWM_SEQ0_LEN];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* PWM Register access */

static inline void nrf52_pwm_putreg(FAR struct nrf52_pwm_s *priv,
                                    uint32_t offset,
                                    uint32_t value);
static inline uint32_t nrf52_pwm_getreg(FAR struct nrf52_pwm_s *priv,
                                        uint32_t offset);

/* PWM helpers */

static int nrf52_pwm_configure(FAR struct nrf52_pwm_s *priv);
static int nrf52_pwm_duty(FAR struct nrf52_pwm_s *priv, uint8_t chan,
                          ub16_t duty);
static int nrf52_pwm_freq(FAR struct nrf52_pwm_s *priv, uint32_t freq);

/* PWM driver methods */

static int nrf52_pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int nrf52_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);
#ifdef CONFIG_PWM_PULSECOUNT
static int nrf52_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                           FAR const struct pwm_info_s *info,
                           FAR void *handle);
#else
static int nrf52_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                           FAR const struct pwm_info_s *info);
#endif
static int nrf52_pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int nrf52_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                           int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver.
 */

static const struct pwm_ops_s g_nrf52_pwmops =
{
  .setup       = nrf52_pwm_setup,
  .shutdown    = nrf52_pwm_shutdown,
  .start       = nrf52_pwm_start,
  .stop        = nrf52_pwm_stop,
  .ioctl       = nrf52_pwm_ioctl,
};

#ifdef CONFIG_NRF52_PWM0
/* PWM 0 */

struct nrf52_pwm_s g_nrf52_pwm0 =
{
  .ops     = &g_nrf52_pwmops,
  .base    = NRF52_PWM0_BASE,
#ifdef CONFIG_NRF52_PWM0_CH0
  .ch0_pin = NRF52_PWM0_CH0_PIN,
#endif
#ifdef CONFIG_NRF52_PWM0_CH1
  .ch1_pin = NRF52_PWM0_CH1_PIN,
#endif
#ifdef CONFIG_NRF52_PWM0_CH2
  .ch2_pin = NRF52_PWM0_CH2_PIN,
#endif
#ifdef CONFIG_NRF52_PWM0_CH3
  .ch3_pin = NRF52_PWM0_CH3_PIN,
#endif
};
#endif

#ifdef CONFIG_NRF52_PWM1
/* PWM 1 */

struct nrf52_pwm_s g_nrf52_pwm1 =
{
  .ops     = &g_nrf52_pwmops,
  .base    = NRF52_PWM1_BASE,
#ifdef CONFIG_NRF52_PWM1_CH0
  .ch0_pin = NRF52_PWM1_CH0_PIN,
#endif
#ifdef CONFIG_NRF52_PWM1_CH1
  .ch1_pin = NRF52_PWM1_CH1_PIN,
#endif
#ifdef CONFIG_NRF52_PWM1_CH2
  .ch2_pin = NRF52_PWM1_CH2_PIN,
#endif
#ifdef CONFIG_NRF52_PWM1_CH3
  .ch3_pin = NRF52_PWM1_CH3_PIN,
#endif
};
#endif

#ifdef CONFIG_NRF52_PWM2
/* PWM 2 */

struct nrf52_pwm_s g_nrf52_pwm2 =
{
  .ops     = &g_nrf52_pwmops,
  .base    = NRF52_PWM2_BASE,
#ifdef CONFIG_NRF52_PWM2_CH0
  .ch0_pin = NRF52_PWM2_CH0_PIN,
#endif
#ifdef CONFIG_NRF52_PWM2_CH1
  .ch1_pin = NRF52_PWM2_CH1_PIN,
#endif
#ifdef CONFIG_NRF52_PWM2_CH2
  .ch2_pin = NRF52_PWM2_CH2_PIN,
#endif
#ifdef CONFIG_NRF52_PWM2_CH3
  .ch3_pin = NRF52_PWM2_CH3_PIN,
#endif
};
#endif

#ifdef CONFIG_NRF52_PWM3
/* PWM 3 */

struct nrf52_pwm_s g_nrf52_pwm3 =
{
  .ops     = &g_nrf52_pwmops,
  .base    = NRF52_PWM3_BASE,
#ifdef CONFIG_NRF52_PWM3_CH0
  .ch0_pin = NRF52_PWM3_CH0_PIN,
#endif
#ifdef CONFIG_NRF52_PWM3_CH1
  .ch1_pin = NRF52_PWM3_CH1_PIN,
#endif
#ifdef CONFIG_NRF52_PWM3_CH2
  .ch2_pin = NRF52_PWM3_CH2_PIN,
#endif
#ifdef CONFIG_NRF52_PWM3_CH3
  .ch3_pin = NRF52_PWM3_CH3_PIN,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_pwm_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void nrf52_pwm_putreg(FAR struct nrf52_pwm_s *priv,
                                    uint32_t offset,
                                    uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: nrf52_pwm_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t nrf52_pwm_getreg(FAR struct nrf52_pwm_s *priv,
                                        uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: nrf52_pwm_configure
 *
 * Description:
 *   Configure PWM
 *
 ****************************************************************************/

static int nrf52_pwm_configure(FAR struct nrf52_pwm_s *priv)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(priv);

  /* Configure PWM mode */

  nrf52_pwm_putreg(priv, NRF52_PWM_MODE_OFFSET, PWM_MODE_UP);

  /* Configure decoder  */

  regval = PWM_DECODER_LOAD_INDIVIDUAL | PWM_DECODER_MODE_REFRESH;
  nrf52_pwm_putreg(priv, NRF52_PWM_DECODER_OFFSET, regval);

  /* Configure sequence 0 */

  regval = (uint32_t)priv->seq0;
  nrf52_pwm_putreg(priv, NRF52_PWM_SEQ0PTR_OFFSET, regval);

  regval = PWM_SEQ0_LEN;
  nrf52_pwm_putreg(priv, NRF52_PWM_SEQ0CNT_OFFSET, regval);

  regval = 0;
  nrf52_pwm_putreg(priv, NRF52_PWM_SEQ0REFRESH_OFFSET, regval);

  return ret;
}

/****************************************************************************
 * Name: nrf52_pwm_duty
 *
 * Description:
 *   Configure PWM duty
 *
 ****************************************************************************/

static int nrf52_pwm_duty(FAR struct nrf52_pwm_s *priv, uint8_t chan,
                          ub16_t duty)
{
  uint16_t compare = 0;

  DEBUGASSERT(priv);

  pwminfo("PWM channel: %d duty: %" PRId32 "\n", chan, duty);

  /* Get compare
   *
   * duty cycle = compare / reload (fractional value)
   */

  compare = b16toi(duty * priv->cntrtop + b16HALF);

  /* Configure channel sequence */

  priv->seq0[chan] = PWM_POLARITY_DEFAULT | compare;

  pwminfo("seq0[%d]: %d %d\n", chan, compare, priv->seq0[chan]);

  return OK;
}

/****************************************************************************
 * Name: nrf52_pwm_freq
 *
 * Description:
 *   Configure PWM frequency
 *
 ****************************************************************************/

static int nrf52_pwm_freq(FAR struct nrf52_pwm_s *priv, uint32_t freq)
{
  uint32_t regval    = 0;
  uint32_t pwm_clk   = 0;
  uint32_t top       = 0;
  uint32_t prescaler = 0;
  uint64_t tmp       = 0;
  int      ret       = OK;

  DEBUGASSERT(priv);

  /* Get best prescaler */

  tmp = PWM_COUNTERTOP_MASK * freq;

  if (tmp >= 16000000)
    {
      pwm_clk   = 16000000;
      prescaler = PWM_PRESCALER_16MHZ;
    }
  else if (tmp >= 8000000)
    {
      pwm_clk   = 8000000;
      prescaler = PWM_PRESCALER_8MHZ;
    }
  else if (tmp >= 4000000)
    {
      pwm_clk   = 4000000;
      prescaler = PWM_PRESCALER_4MHZ;
    }
  else if (tmp >= 2000000)
    {
      pwm_clk   = 2000000;
      prescaler = PWM_PRESCALER_2MHZ;
    }
  else if (tmp >= 1000000)
    {
      pwm_clk   = 1000000;
      prescaler = PWM_PRESCALER_1MHZ;
    }
  else if (tmp >= 500000)
    {
      pwm_clk   = 500000;
      prescaler = PWM_PRESCALER_500KHZ;
    }
  else if (tmp >= 250000)
    {
      pwm_clk   = 250000;
      prescaler = PWM_PRESCALER_250KHZ;
    }
  else
    {
      pwm_clk   = 125000;
      prescaler = PWM_PRESCALER_125KHZ;
    }

  /* Configure prescaler */

  nrf52_pwm_putreg(priv, NRF52_PWM_PRESCALER_OFFSET, prescaler);

  /* Get counter max */

  top = pwm_clk / freq;
  if (top < 2)
    {
      top = 1;
    }
  else if (top > PWM_COUNTERTOP_MASK)
    {
      top = PWM_COUNTERTOP_MASK;
    }
  else
    {
      top = top - 1;
    }

  /* Configure counter max */

  regval = top;
  nrf52_pwm_putreg(priv, NRF52_PWM_COUNTERTOP_OFFSET, regval);

  priv->cntrtop = top;

  pwminfo("PWM frequency: %" PRId32 " pwm_clk: %" PRId32
          " pwm_prescaler: %" PRId32 " top: %" PRId32 "\n",
          freq, pwm_clk, prescaler, top);

  return ret;
}

/****************************************************************************
 * Name: nrf52_pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 ****************************************************************************/

static int nrf52_pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct nrf52_pwm_s *priv = (FAR struct nrf52_pwm_s *)dev;
  int                     ret  = OK;
  uint32_t                regval = 0;
  uint32_t                pin = 0;
  uint32_t                port = 0;

  DEBUGASSERT(dev);

  /* Configure channels */

  if (priv->ch0_pin != 0)
    {
      nrf52_gpio_config(priv->ch0_pin);

      pin  = GPIO_PIN_DECODE(priv->ch0_pin);
      port = GPIO_PORT_DECODE(priv->ch0_pin);

      regval = (port << PWM_PSEL_PORT_SHIFT);
      regval |= (pin << PWM_PSEL_PIN_SHIFT);

      nrf52_pwm_putreg(priv, NRF52_PWM_PSEL0_OFFSET, regval);
    }

  if (priv->ch1_pin != 0)
    {
      nrf52_gpio_config(priv->ch1_pin);

      pin  = GPIO_PIN_DECODE(priv->ch1_pin);
      port = GPIO_PORT_DECODE(priv->ch1_pin);

      regval = (port << PWM_PSEL_PORT_SHIFT);
      regval |= (pin << PWM_PSEL_PIN_SHIFT);

      nrf52_pwm_putreg(priv, NRF52_PWM_PSEL1_OFFSET, regval);
    }

  if (priv->ch2_pin != 0)
    {
      nrf52_gpio_config(priv->ch2_pin);

      pin  = GPIO_PIN_DECODE(priv->ch2_pin);
      port = GPIO_PORT_DECODE(priv->ch2_pin);

      regval = (port << PWM_PSEL_PORT_SHIFT);
      regval |= (pin << PWM_PSEL_PIN_SHIFT);

      nrf52_pwm_putreg(priv, NRF52_PWM_PSEL2_OFFSET, regval);
    }

  if (priv->ch3_pin != 0)
    {
      nrf52_gpio_config(priv->ch3_pin);

      pin  = GPIO_PIN_DECODE(priv->ch3_pin);
      port = GPIO_PORT_DECODE(priv->ch3_pin);

      regval = (port << PWM_PSEL_PORT_SHIFT);
      regval |= (pin << PWM_PSEL_PIN_SHIFT);

      nrf52_pwm_putreg(priv, NRF52_PWM_PSEL3_OFFSET, regval);
    }

  /* Configure PWM */

  ret = nrf52_pwm_configure(priv);
  if (ret < 0)
    {
      pwmerr("ERROR: nrf52_pwm_configure failed %d\n", ret);
      goto errout;
    }

  /* Enable PWM */

  nrf52_pwm_putreg(priv, NRF52_PWM_ENABLE_OFFSET, 1);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 ****************************************************************************/

static int nrf52_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct nrf52_pwm_s *priv = (FAR struct nrf52_pwm_s *)dev;
  int                     ret  = OK;

  DEBUGASSERT(dev);

  /* Disable PWM */

  nrf52_pwm_putreg(priv, NRF52_PWM_ENABLE_OFFSET, 0);

  return ret;
}

/****************************************************************************
 * Name: nrf52_pwm_start
 *
 * Description:
 *   (Re-)initialize the PWM and start the pulsed output
 *
 ****************************************************************************/

#ifdef CONFIG_PWM_PULSECOUNT
static int nrf52_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                           FAR const struct pwm_info_s *info,
                           FAR void *handle)
{
#error Not supported
}
#else
static int nrf52_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                           FAR const struct pwm_info_s *info)
{
  FAR struct nrf52_pwm_s *priv = (FAR struct nrf52_pwm_s *)dev;
  int                     ret  = OK;
#ifdef CONFIG_PWM_MULTICHAN
  int                     i    = 0;
#endif

  DEBUGASSERT(dev);

  /* If frequency has not changed we just update duty */

  if (info->frequency != priv->frequency)
    {
      /* Update frequency */

      ret = nrf52_pwm_freq(priv, info->frequency);

      if (ret == OK)
        {
          priv->frequency = info->frequency;
        }
    }

#ifdef CONFIG_PWM_MULTICHAN
      for (i = 0; ret == OK && i < CONFIG_PWM_NCHANNELS; i++)
        {
          /* Break the loop if all following channels are not configured */

          if (info->channels[i].channel == -1)
            {
              break;
            }

          /* Set output if channel configured */

          if (info->channels[i].channel != 0)
            {
              ret = nrf52_pwm_duty(priv,
                                   (info->channels[i].channel - 1),
                                   info->channels[i].duty);
            }
        }

#else
      ret = nrf52_pwm_duty(dev,
                           (info->channels[0].channel - 1),
                           info->duty);
#endif /* CONFIG_PWM_MULTICHAN */

  /* Start sequence 0 */

  nrf52_pwm_putreg(priv, NRF52_PWM_TASKS_SEQSTART0_OFFSET, 1);

  /* Wait for sequence started */

  while (nrf52_pwm_getreg(priv, NRF52_PWM_EVENTS_SEQSTARTED0_OFFSET) != 1);

  return ret;
}
#endif

/****************************************************************************
 * Name: nrf52_pwm_stop
 *
 * Description:
 *   Stop the PWM
 *
 ****************************************************************************/

static int nrf52_pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct nrf52_pwm_s *priv = (FAR struct nrf52_pwm_s *)dev;

  DEBUGASSERT(dev);

  /* Stop PWM */

  nrf52_pwm_putreg(priv, NRF52_PWM_TASKS_STOP_OFFSET, 1);

  /* Wait for PWM stopped */

  while (nrf52_pwm_getreg(priv, NRF52_PWM_EVENTS_STOPPED_OFFSET) != 1);

  return OK;
}

/****************************************************************************
 * Name: nrf52_pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ****************************************************************************/

static int nrf52_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                           int cmd, unsigned long arg)
{
  FAR struct nrf52_pwm_s *priv = (FAR struct nrf52_pwm_s *)dev;

  DEBUGASSERT(dev);

  /* There are no platform-specific ioctl commands */

  UNUSED(priv);

  return -ENOTTY;
}

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   pwm - A number identifying the pwm instance.
 *
 * Returned Value:
 *   On success, a pointer to the NRF52 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *nrf52_pwminitialize(int pwm)
{
  struct nrf52_pwm_s *lower = NULL;

  pwminfo("Initialize PWM%u\n", pwm);

  switch (pwm)
    {
#ifdef CONFIG_NRF52_PWM0
      case 0:
        {
          lower = &g_nrf52_pwm0;
          break;
        }
#endif

#ifdef CONFIG_NRF52_PWM1
      case 1:
        {
          lower = &g_nrf52_pwm1;
          break;
        }
#endif

#ifdef CONFIG_NRF52_PWM2
      case 2:
        {
          lower = &g_nrf52_pwm2;
          break;
        }
#endif

#ifdef CONFIG_NRF52_PWM3
      case 3:
        {
          lower = &g_nrf52_pwm3;
          break;
        }
#endif

      default:
        {
          pwmerr("ERROR: No such PWM device %d\n", pwm);
          lower = NULL;
          goto errout;
        }
    }

errout:
  return (struct pwm_lowerhalf_s *)lower;
}
