/****************************************************************************
 * arch/risc-v/src/bl602/bl602_pwm_lowerhalf.c
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
#include "bl602_gpio.h"
#include "bl602_pwm_lowerhalf.h"
#include "hardware/bl602_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL_PWM_FREQ_DEFAULT       (1000)
#define BL_PWM_STOP_TIMEOUT_COUNT (160*1000)

#define BL_PWM_XTAL_CLK           (40000000)
#define BL_PWM_BUS_BCLK           (80000000)

#ifdef CONFIG_PWM_NCHANNELS
#  if (CONFIG_PWM_NCHANNELS > 5)
#    error "The maximum number of BL602 PWM channels is 5!"
#  endif
#endif

#ifdef CONFIG_BL602_PWM0_BUS_BCLK_SRC
#define BL_PWM_CLK BL_PWM_BUS_BCLK
#else
#define BL_PWM_CLK BL_PWM_XTAL_CLK
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bl602_pwm_ch_cfgs
{
    int ch;                                    /* PWM channel */
    int clk;                                   /* PWM Clock */
    int stop_mode;                             /* PWM Stop Mode */
    int pol;                                   /* PWM mode type */
    uint16_t clk_div;                          /* PWM clkDiv num */
    uint16_t period;                           /* PWM period set */
    uint16_t threshold1;                       /* PWM threshold1 num */
    uint16_t threshold2;                       /* PWM threshold2 num */
    uint16_t int_pulse_cnt;                    /* PWM interrupt pulse count */
};

struct bl602_pwm_s
{
  const struct pwm_ops_s *ops;         /* PWM operations */
  uint32_t                chan_pin[5]; /* Channel pin */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* PWM driver methods */

static int bl602_pwm_setup(struct pwm_lowerhalf_s *dev);
static int bl602_pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int bl602_pwm_start(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info);
static int bl602_pwm_stop(struct pwm_lowerhalf_s *dev);
static int bl602_pwm_ioctl(struct pwm_lowerhalf_s *dev,
                           int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver.
 */

static const struct pwm_ops_s g_bl602_pwmops =
{
  .setup       = bl602_pwm_setup,
  .shutdown    = bl602_pwm_shutdown,
  .start       = bl602_pwm_start,
  .stop        = bl602_pwm_stop,
  .ioctl       = bl602_pwm_ioctl,
};

struct bl602_pwm_s g_bl602_pwm0 =
{
  .ops         = &g_bl602_pwmops,
#ifdef BOARD_PWM_CH0_PIN
  .chan_pin[0] = BOARD_PWM_CH0_PIN,
#endif
#ifdef BOARD_PWM_CH1_PIN
  .chan_pin[1] = BOARD_PWM_CH1_PIN,
#endif
#ifdef BOARD_PWM_CH2_PIN
  .chan_pin[2] = BOARD_PWM_CH2_PIN,
#endif
#ifdef BOARD_PWM_CH3_PIN
  .chan_pin[3] = BOARD_PWM_CH3_PIN,
#endif
#ifdef BOARD_PWM_CH4_PIN
  .chan_pin[4] = BOARD_PWM_CH4_PIN,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void pwm_channel_disable(uint8_t ch)
{
  modifyreg32(BL602_PWM_N_CONFIG(ch), 0, CONFIG_PWM_STOP_EN);
  modifyreg32(BL602_PWM_N_CONFIG(ch), INTERRUPT_PWM_INT_ENABLE, 0);
}

static void pwm_channel_enable(uint8_t ch)
{
  modifyreg32(BL602_PWM_N_CONFIG(ch), CONFIG_PWM_STOP_EN, 0);
}

static void pwm_channel_init(uint8_t ch, struct bl602_pwm_ch_cfgs *ch_cfg)
{
  uint32_t timeout_cnt = BL_PWM_STOP_TIMEOUT_COUNT;

  /* Config pwm clock and polarity */

  modifyreg32(BL602_PWM_N_CONFIG(ch), 0, CONFIG_PWM_STOP_EN);

  while (!(getreg32(BL602_PWM_N_CONFIG(ch)) & CONFIG_PWM_STS_TOP))
    {
      timeout_cnt--;
      if (timeout_cnt == 0)
        {
          return;
        }
    }

  modifyreg32(BL602_PWM_N_CONFIG(ch),
              CONFIG_REG_CLK_SEL_MASK & (~ch_cfg->clk),
              CONFIG_REG_CLK_SEL_MASK & ch_cfg->clk);

  modifyreg32(BL602_PWM_N_CONFIG(ch),
              CONFIG_PWM_OUT_INV,
              ch_cfg->pol ? CONFIG_PWM_OUT_INV : 0);

  modifyreg32(BL602_PWM_N_CONFIG(ch),
              CONFIG_PWM_STOP_MODE,
              ch_cfg->stop_mode ? CONFIG_PWM_STOP_MODE : 0);

  /* Config pwm division */

  putreg32(ch_cfg->clk_div, BL602_PWM_N_CLKDIV(ch));

  /* Config pwm period and duty */

  putreg32(ch_cfg->threshold1, BL602_PWM_N_THRE1(ch));
  putreg32(ch_cfg->threshold2, BL602_PWM_N_THRE2(ch));
  putreg32(ch_cfg->period, BL602_PWM_N_PERIOD(ch));

  /* Config interrupt pulse count */

  modifyreg32(BL602_PWM_N_INTERRUPT(ch), 0,
    (ch_cfg->int_pulse_cnt & PERIOD_PWM_PERIOD_MASK));
}

static int32_t pwm_init(uint8_t id, uint32_t freq)
{
  struct bl602_pwm_ch_cfgs pwm_cfg =
    {
      .ch = BL602_PWM_CH0,
#ifdef CONFIG_BL602_PWM0_BUS_BCLK_SRC
      .clk = BL602_PWM_CLK_BCLK,
#else
      .clk = BL602_PWM_CLK_XCLK,
#endif
      .stop_mode = BL602_PWM_STOP_ABRUPT,
      .pol = BL602_PWM_POL_NORMAL,
      .clk_div = 0,
      .period = 100,
      .threshold1 = 0,
      .threshold2 = 0,
      .int_pulse_cnt = 0,
    };

  pwm_cfg.period = BL_PWM_CLK / freq;
  pwm_cfg.ch = id;

  pwm_channel_disable(id);
  pwm_channel_init(id, &pwm_cfg);

  return 0;
}

/****************************************************************************
 * Name: bl602_pwm_duty
 *
 * Description:
 *   Configure PWM duty
 *
 ****************************************************************************/

static int bl602_pwm_duty(struct bl602_pwm_s *priv, uint8_t chan,
                          ub16_t duty)
{
  uint16_t period;
  uint16_t threshold1;
  uint16_t threshold2;
  uint32_t tmp_val;

  /* get pwm period and duty */

  tmp_val = getreg32(BL602_PWM_N_THRE1(chan));
  threshold1 = tmp_val & THRE1_PWM_THRE1_MASK;

  tmp_val = getreg32(BL602_PWM_N_THRE2(chan));
  threshold2 = tmp_val & THRE2_PWM_THRE2_MASK;

  tmp_val = getreg32(BL602_PWM_N_PERIOD(chan));
  period = tmp_val & PERIOD_PWM_PERIOD_MASK;

  threshold1 = 0;
  threshold2 = (uint16_t)((uint32_t)period * duty / 65535);

  putreg32(threshold1, BL602_PWM_N_THRE1(chan));
  putreg32(threshold2, BL602_PWM_N_THRE2(chan));

  return OK;
}

/****************************************************************************
 * Name: bl602_pwm_freq
 *
 * Description:
 *   Configure PWM frequency
 *
 ****************************************************************************/

static int bl602_pwm_freq(struct bl602_pwm_s *priv, uint8_t chan,
                          uint32_t freq)
{
  uint16_t period = BL_PWM_CLK / freq;
  uint16_t threshold1 = 0;
  uint16_t threshold2 = 0;

  pwm_channel_disable(chan);

  /* Config pwm period and duty */

  putreg32(threshold1, BL602_PWM_N_THRE1(chan));
  putreg32(threshold2, BL602_PWM_N_THRE2(chan));
  putreg32(period, BL602_PWM_N_PERIOD(chan));

  pwm_channel_enable(chan);

  return OK;
}

/****************************************************************************
 * Name: bl602_pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 ****************************************************************************/

static int bl602_pwm_setup(struct pwm_lowerhalf_s *dev)
{
  int i;
  int ret = OK;
  struct bl602_pwm_s *priv = (struct bl602_pwm_s *)dev;

  UNUSED(i);

#ifdef CONFIG_PWM_NCHANNELS
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      bl602_configgpio(priv->chan_pin[i]);
      pwm_init(i, BL_PWM_FREQ_DEFAULT);
    }
#else
  bl602_configgpio(priv->chan_pin[0]);
  pwm_init(0, BL_PWM_FREQ_DEFAULT);
#endif

  return ret;
}

/****************************************************************************
 * Name: bl602_pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 ****************************************************************************/

static int bl602_pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  int i;
  int ret = OK;
  struct bl602_pwm_s *priv = (struct bl602_pwm_s *)dev;

  UNUSED(i);

#ifdef CONFIG_PWM_NCHANNELS
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      pwm_channel_disable(priv->chan_pin[i]);
      bl602_gpio_deinit(i);
    }
#else
  pwm_channel_disable(priv->chan_pin[0]);
  bl602_gpio_deinit(0);
#endif

  return ret;
}

/****************************************************************************
 * Name: bl602_pwm_start
 *
 * Description:
 *   (Re-)initialize the PWM and start the pulsed output
 *
 ****************************************************************************/

static int bl602_pwm_start(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info)
{
  struct bl602_pwm_s *priv = (struct bl602_pwm_s *)dev;
  int                 ret  = OK;
  int                 i;

  UNUSED(i);

#ifdef CONFIG_PWM_NCHANNELS
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      int8_t chan = info->channels[i].channel;

      /* Break the loop if all following channels are not configured */

      if (chan == -1)
        {
          break;
        }

      bl602_pwm_freq(priv, chan, info->frequency);
      bl602_pwm_duty(priv, chan, info->channels[i].duty);
      pwm_channel_enable(chan);
    }
#else
  bl602_pwm_freq(priv, 0, info->frequency);
  bl602_pwm_duty(priv, 0, info->duty);
  pwm_channel_enable(0);
#endif

  return ret;
}

/****************************************************************************
 * Name: bl602_pwm_stop
 *
 * Description:
 *   Stop the PWM
 *
 ****************************************************************************/

static int bl602_pwm_stop(struct pwm_lowerhalf_s *dev)
{
  int i;
  struct bl602_pwm_s *priv = (struct bl602_pwm_s *)dev;

  UNUSED(priv);
  UNUSED(i);

#ifdef CONFIG_PWM_NCHANNELS
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      pwm_channel_disable(i);
    }
#else
  pwm_channel_disable(0);
#endif

  return OK;
}

/****************************************************************************
 * Name: bl602_pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ****************************************************************************/

static int bl602_pwm_ioctl(struct pwm_lowerhalf_s *dev,
                           int cmd, unsigned long arg)
{
  struct bl602_pwm_s *priv = (struct bl602_pwm_s *)dev;

  DEBUGASSERT(dev);

  /* There are no platform-specific ioctl commands */

  UNUSED(priv);

  return -ENOTTY;
}

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   pwm - A number identifying the pwm instance.
 *
 * Returned Value:
 *   On success, a pointer to the BL602 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *bl602_pwminitialize(int pwm)
{
  struct bl602_pwm_s *lower = NULL;

  if (pwm > 0)
    {
      pwminfo("Initialize PWM%u failed\n", pwm);
      return NULL;
    }

  pwminfo("Initialize PWM%u\n", pwm);

  lower = &g_bl602_pwm0;

  return (struct pwm_lowerhalf_s *)lower;
}

