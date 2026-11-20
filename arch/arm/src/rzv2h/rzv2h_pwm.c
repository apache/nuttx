/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_pwm.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <fixedmath.h>

#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/debug.h>

#include <arch/board/board.h>

#include "r_gpt.h"
#include "rzv2h_gpio.h"
#include "rzv2h_pwm.h"
#include "rzv2h_fsp_err.h"

#ifdef CONFIG_RZV2H_GPT_PWM

/* Each GPT timer supports only 1 output at a time */

#if CONFIG_PWM_NCHANNELS > 1
#  error "Each GPT timer supports only 1 PWM output; CONFIG_PWM_NCHANNELS must be 1"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Per-channel private data */

struct rzv2h_gpt_pwm_priv_s
{
  const struct pwm_ops_s *ops;      /* PWM operations */
  uint8_t                 channel;  /* GPT channel 0-15 */
  gpt_instance_ctrl_t     ctrl;     /* GPT instance control block */
  timer_cfg_t             cfg;      /* Timer configuration */
  gpt_extended_cfg_t      ext_cfg;  /* GPT extended configuration */
  gpio_pinset_t           pin_cfg;  /* GTIOC output pin */
  bool                   is_gtioca; /* true = GTIOCA, false = GTIOCB */
  uint32_t               frequency; /* Current PWM frequency */
  bool                    is_setup; /* true after setup() completed */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* PWM driver methods */

static int rzv2h_gpt_pwm_setup(struct pwm_lowerhalf_s *dev);
static int rzv2h_gpt_pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int rzv2h_gpt_pwm_start(struct pwm_lowerhalf_s *dev,
                                const struct pwm_info_s *info);
static int rzv2h_gpt_pwm_stop(struct pwm_lowerhalf_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pwm_ops_s g_pwmops =
{
  .setup    = rzv2h_gpt_pwm_setup,
  .shutdown = rzv2h_gpt_pwm_shutdown,
  .start    = rzv2h_gpt_pwm_start,
  .stop     = rzv2h_gpt_pwm_stop,
  .ioctl    = NULL,
};

/* Per-channel private data instances */

#ifdef CONFIG_RZV2H_GPT0_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt0_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 0,
  .pin_cfg   = BOARD_GPT0_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT0_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT0,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT1_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt1_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 1,
  .pin_cfg   = BOARD_GPT1_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT1_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT1,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT2_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt2_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 2,
  .pin_cfg   = BOARD_GPT2_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT2_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT2,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT3_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt3_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 3,
  .pin_cfg   = BOARD_GPT3_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT3_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT3,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT4_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt4_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 4,
  .pin_cfg   = BOARD_GPT4_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT4_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT4,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT5_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt5_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 5,
  .pin_cfg   = BOARD_GPT5_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT5_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT5,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT6_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt6_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 6,
  .pin_cfg   = BOARD_GPT6_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT6_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT6,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT7_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt7_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 7,
  .pin_cfg   = BOARD_GPT7_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT7_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT7,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT8_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt8_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 8,
  .pin_cfg   = BOARD_GPT8_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT8_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT10,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT9_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt9_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 9,
  .pin_cfg   = BOARD_GPT9_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT9_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT11,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT10_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt10_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 10,
  .pin_cfg   = BOARD_GPT10_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT10_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT12,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT11_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt11_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 11,
  .pin_cfg   = BOARD_GPT11_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT11_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT13,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT12_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt12_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 12,
  .pin_cfg   = BOARD_GPT12_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT12_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT14,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT13_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt13_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 13,
  .pin_cfg   = BOARD_GPT13_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT13_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT15,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT14_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt14_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 14,
  .pin_cfg   = BOARD_GPT14_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT14_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT16,
  },
};
#endif

#ifdef CONFIG_RZV2H_GPT15_PWM
static struct rzv2h_gpt_pwm_priv_s g_gpt15_pwm_priv =
{
  .ops       = &g_pwmops,
  .channel   = 15,
  .pin_cfg   = BOARD_GPT15_GTIOC_GPIO,
  .is_gtioca = BOARD_GPT15_USE_GTIOCA,
  .ext_cfg   =
  {
    .p_reg   = (void *)R_GPT17,
  },
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_gpt_pwm_setup
 *
 * Description:
 *   This method is called when the PWM device is opened (first open).
 *   It configures the pin for the GPT output function and
 *   initializes the GPT hardware via FSP R_GPT_Open().  GTIOC output is
 *   enabled via ext_cfg settings during R_GPT_Open(); no separate
 *   R_GPT_OutputEnable() call is needed.  Pulse output is not started
 *   here; that happens in start().
 *
 * Input Parameters:
 *   dev - Pointer to the lower-half PWM driver instance.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure:
 *   - Error from rzv2h_configgpio() if pin configuration fails.
 *   - rzv2h_fsp_err_to_errno() mapping if R_GPT_Open() fails.
 *
 ****************************************************************************/

static int rzv2h_gpt_pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct rzv2h_gpt_pwm_priv_s *priv = (struct rzv2h_gpt_pwm_priv_s *)dev;
  fsp_err_t err;

  /* Guard against re-opening an already-open GPT channel */

  if (priv->is_setup)
    {
      return OK;
    }

  pwminfo("GPT%u: setup\n", priv->channel);

  /* Configure the output pin */

  int ret;

  pwminfo("GPT%u: %s\n",
          priv->channel,
          priv->is_gtioca ? "GTIOCA" : "GTIOCB");

  ret = rzv2h_configgpio(priv->pin_cfg);
  if (ret < 0)
    {
      pwmerr("ERROR: rzv2h_configgpio failed for GPT%u: %d\n",
              priv->channel, ret);
      return ret;
    }

  /* Configure the timer for PWM mode */

  priv->cfg.mode              = TIMER_MODE_PWM;
  priv->cfg.period_counts     = R_FSP_SystemClockHzGet(
                                   BSP_FEATURE_GPT_CLOCK_SOURCE) /
                                CONFIG_RZV2H_GPT_PWM_DEFAULT_FREQUENCY;
  priv->cfg.source_div        = TIMER_SOURCE_DIV_1;
  priv->cfg.duty_cycle_counts = 0;
  priv->cfg.channel           = priv->channel;
  priv->cfg.cycle_end_ipl     = 0;
  priv->cfg.cycle_end_irq     = (IRQn_Type)0;
  priv->cfg.p_callback        = NULL;
  priv->cfg.p_context         = NULL;
  priv->cfg.p_extend          = &priv->ext_cfg;

  /* Configure output pin */

  if (priv->is_gtioca)
    {
      priv->ext_cfg.gtioca.output_enabled = true;
      priv->ext_cfg.gtioca.stop_level     = GPT_PIN_LEVEL_LOW;
      priv->ext_cfg.gtiocb.output_enabled = false;
      priv->ext_cfg.gtiocb.stop_level     = GPT_PIN_LEVEL_LOW;
    }
  else
    {
      priv->ext_cfg.gtioca.output_enabled = false;
      priv->ext_cfg.gtioca.stop_level     = GPT_PIN_LEVEL_LOW;
      priv->ext_cfg.gtiocb.output_enabled = true;
      priv->ext_cfg.gtiocb.stop_level     = GPT_PIN_LEVEL_LOW;
    }

  /* No event sources */

  priv->ext_cfg.start_source     = GPT_SOURCE_NONE;
  priv->ext_cfg.stop_source      = GPT_SOURCE_NONE;
  priv->ext_cfg.clear_source     = GPT_SOURCE_NONE;
  priv->ext_cfg.capture_a_source = GPT_SOURCE_NONE;
  priv->ext_cfg.capture_b_source = GPT_SOURCE_NONE;
  priv->ext_cfg.count_up_source  = GPT_SOURCE_NONE;
  priv->ext_cfg.count_down_source = GPT_SOURCE_NONE;

  /* No capture filter */

  priv->ext_cfg.capture_filter_gtioca = GPT_CAPTURE_FILTER_NONE;
  priv->ext_cfg.capture_filter_gtiocb = GPT_CAPTURE_FILTER_NONE;

  /* No interrupts for basic PWM */

  priv->ext_cfg.capture_a_ipl = 0;
  priv->ext_cfg.capture_b_ipl = 0;
  priv->ext_cfg.dead_time_ipl = 0;
  priv->ext_cfg.capture_a_irq = (IRQn_Type)0;
  priv->ext_cfg.capture_b_irq = (IRQn_Type)0;
  priv->ext_cfg.dead_time_irq = (IRQn_Type)0;

  /* No advanced PWM features */

  priv->ext_cfg.p_pwm_cfg = NULL;

  /* No custom GTIOR settings */

  memset(&priv->ext_cfg.gtior_setting, 0,
         sizeof(priv->ext_cfg.gtior_setting));

  /* Open the GPT channel */

  err = R_GPT_Open(&priv->ctrl, &priv->cfg);
  if (err != FSP_SUCCESS)
    {
      pwmerr("ERROR: R_GPT_Open failed for GPT%u: %d\n",
             priv->channel, (int)err);
      return rzv2h_fsp_err_to_errno(err);
    }

  priv->frequency = CONFIG_RZV2H_GPT_PWM_DEFAULT_FREQUENCY;
  priv->is_setup  = true;

  return OK;
}

/****************************************************************************
 * Name: rzv2h_gpt_pwm_shutdown
 *
 * Description:
 *   This method is called when the PWM device is closed (last close).
 *   It stops the timer, releases the GPT hardware via R_GPT_Close(),
 *   and reconfigures the output pin to GPIO output low.
 *
 * Input Parameters:
 *   dev - Pointer to the lower-half PWM driver instance.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure:
 *   - rzv2h_fsp_err_to_errno() mapping if R_GPT_Close() fails.
 *   - Error from rzv2h_configgpio() if pin reconfiguration fails.
 *
 ****************************************************************************/

static int rzv2h_gpt_pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct rzv2h_gpt_pwm_priv_s *priv = (struct rzv2h_gpt_pwm_priv_s *)dev;
  gpio_pinset_t gpio_cfg;
  fsp_err_t err;
  int ret;

  pwmerr("GPT%u: shutdown\n", priv->channel);

  err = R_GPT_Close(&priv->ctrl);
  if (err != FSP_SUCCESS)
    {
      pwmerr("ERROR: R_GPT_Close failed for GPT%u: %d\n",
             priv->channel, (int)err);
      return rzv2h_fsp_err_to_errno(err);
    }

  priv->frequency = 0;
  priv->is_setup  = false;

  /* Reconfigure the PWM pin to GPIO output, drive low so the pin
   * does not float or hold its last PWM level after shutdown.
   */

  gpio_cfg.port_pin = priv->pin_cfg.port_pin;
  gpio_cfg.cfg = IOPORT_CFG_PORT_DIRECTION_OUTPUT |
                 IOPORT_CFG_PORT_OUTPUT_LOW;

  ret = rzv2h_configgpio(gpio_cfg);
  if (ret < 0)
    {
      pwmerr("ERROR: rzv2h_configgpio failed during shutdown "
             "for GPT%u: %d\n",
             priv->channel, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: rzv2h_gpt_pwm_start
 *
 * Description:
 *   Start the PWM output with the given parameters.
 *   Queries the GPT clock at runtime via R_GPT_InfoGet(), computes
 *   period and duty cycle in timer counts.
 *
 * Input Parameters:
 *   dev  - Pointer to the lower-half PWM driver instance.
 *   info - PWM configuration including frequency and duty cycle.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure:
 *   - rzv2h_fsp_err_to_errno() mapping if any FSP GPT API call fails.
 *
 ****************************************************************************/

static int rzv2h_gpt_pwm_start(struct pwm_lowerhalf_s *dev,
                                const struct pwm_info_s *info)
{
  struct rzv2h_gpt_pwm_priv_s *priv = (struct rzv2h_gpt_pwm_priv_s *)dev;
  fsp_err_t err;
  timer_info_t gpt_info;
  uint32_t period_counts;
  uint32_t duty_counts;

  DEBUGASSERT(priv != NULL && info != NULL);
  DEBUGASSERT(info->frequency > 0);

  /* Calculate period in timer counts:
   *   period_counts = gpt_clock_frequency / frequency
   */

  err = R_GPT_InfoGet(&priv->ctrl, &gpt_info);
  if (err != FSP_SUCCESS)
    {
      pwmerr("ERROR: R_GPT_InfoGet failed: %d\n", (int)err);
      return rzv2h_fsp_err_to_errno(err);
    }

  period_counts = gpt_info.clock_frequency / info->frequency;

  /* Calculate duty cycle in timer counts:
   * duty_counts = (duty * period_counts + b16HALF) >> 16;
   * duty is ub16_t (0x0000 = 0%, 0x10000 = 100%); apps use 0xffff
   * for 100%.  b16HALF rounds to nearest.  uint64_t avoids overflow.
   * Snap duty >= 0xffff to period_counts for FSP 100% output mode.
   */

  if (info->channels[0].duty >= 0xffff)
    {
      duty_counts = period_counts;
    }
  else
    {
      duty_counts = (uint32_t)(((uint64_t)info->channels[0].duty *
                                (uint64_t)period_counts + b16HALF) >> 16);
    }

  pwminfo("GPT%u: freq=%" PRIu32 " duty=%08" PRIx32
          " period=%" PRIu32 " duty_cnt=%" PRIu32 "\n",
          priv->channel, info->frequency, info->channels[0].duty,
          period_counts, duty_counts);

  /* Stop the timer before reconfiguring */

  err = R_GPT_Stop(&priv->ctrl);
  if (err != FSP_SUCCESS)
    {
      pwmerr("ERROR: R_GPT_Stop failed: %d\n", (int)err);
      return rzv2h_fsp_err_to_errno(err);
    }

  if (info->frequency != priv->frequency)
    {
      err = R_GPT_PeriodSet(&priv->ctrl, period_counts);
      if (err != FSP_SUCCESS)
        {
          pwmerr("ERROR: R_GPT_PeriodSet failed: %d\n", (int)err);
          return rzv2h_fsp_err_to_errno(err);
        }

      priv->frequency = info->frequency;
    }

  /* Update duty cycle on the configured pin */

  gpt_io_pin_t io_pin = priv->is_gtioca
                        ? GPT_IO_PIN_GTIOCA
                        : GPT_IO_PIN_GTIOCB;

  err = R_GPT_DutyCycleSet(&priv->ctrl, duty_counts, io_pin);
  if (err != FSP_SUCCESS)
    {
      pwmerr("ERROR: R_GPT_DutyCycleSet failed: %d\n", (int)err);
      return rzv2h_fsp_err_to_errno(err);
    }

  /* Pre-load counter to peak so first compare match fires
   * immediately after start, producing correct first cycle.
   */

  err = R_GPT_CounterSet(&priv->ctrl, period_counts - 1);
  if (err != FSP_SUCCESS)
    {
      pwmerr("ERROR: R_GPT_CounterSet failed: %d\n", (int)err);
      return rzv2h_fsp_err_to_errno(err);
    }

  /* Start the timer */

  err = R_GPT_Start(&priv->ctrl);
  if (err != FSP_SUCCESS)
    {
      pwmerr("ERROR: R_GPT_Start failed: %d\n", (int)err);
      return rzv2h_fsp_err_to_errno(err);
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_gpt_pwm_stop
 *
 * Description:
 *   Stop the GPT timer.
 *
 * Input Parameters:
 *   dev - Pointer to the lower-half PWM driver instance.
 *
 * Returned Value:
 *   OK on success; rzv2h_fsp_err_to_errno() mapping if R_GPT_Stop() fails.
 *
 ****************************************************************************/

static int rzv2h_gpt_pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct rzv2h_gpt_pwm_priv_s *priv = (struct rzv2h_gpt_pwm_priv_s *)dev;
  fsp_err_t err;

  pwminfo("GPT%u: stop\n", priv->channel);

  err = R_GPT_Stop(&priv->ctrl);
  if (err != FSP_SUCCESS)
    {
      pwmerr("ERROR: R_GPT_Stop failed: %d\n", (int)err);
      return rzv2h_fsp_err_to_errno(err);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_gpt_pwminitialize
 *
 * Description:
 *   Return the lower-half PWM driver instance for one GPT channel.
 *   Hardware initialization is deferred until the first open()
 *   of the associated /dev/pwmN device.
 *
 * Input Parameters:
 *   channel - GPT channel number (0-15)
 *
 * Returned Value:
 *   On success, a pointer to the lower-half PWM driver is returned.
 *   NULL if the channel is not enabled in Kconfig or is out of range.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *rzv2h_gpt_pwminitialize(uint8_t channel)
{
  struct rzv2h_gpt_pwm_priv_s *priv = NULL;

  /* Select the private data for this channel */

  switch (channel)
    {
#ifdef CONFIG_RZV2H_GPT0_PWM
      case 0:
        priv = &g_gpt0_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT1_PWM
      case 1:
        priv = &g_gpt1_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT2_PWM
      case 2:
        priv = &g_gpt2_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT3_PWM
      case 3:
        priv = &g_gpt3_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT4_PWM
      case 4:
        priv = &g_gpt4_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT5_PWM
      case 5:
        priv = &g_gpt5_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT6_PWM
      case 6:
        priv = &g_gpt6_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT7_PWM
      case 7:
        priv = &g_gpt7_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT8_PWM
      case 8:
        priv = &g_gpt8_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT9_PWM
      case 9:
        priv = &g_gpt9_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT10_PWM
      case 10:
        priv = &g_gpt10_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT11_PWM
      case 11:
        priv = &g_gpt11_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT12_PWM
      case 12:
        priv = &g_gpt12_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT13_PWM
      case 13:
        priv = &g_gpt13_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT14_PWM
      case 14:
        priv = &g_gpt14_pwm_priv;
        break;
#endif
#ifdef CONFIG_RZV2H_GPT15_PWM
      case 15:
        priv = &g_gpt15_pwm_priv;
        break;
#endif
      default:
        pwmerr("ERROR: No such GPT channel: %u\n", channel);
        return NULL;
    }

  return (struct pwm_lowerhalf_s *)priv;
}

/****************************************************************************
 * Name: rzv2h_pwm_setup
 *
 * Description:
 *   Initialize PWM and register PWM devices for all enabled GPT channels.
 *   Each enabled channel is registered as /dev/pwmN.  Hardware
 *   initialization is deferred until the first open() via the
 *   upper-half PWM driver.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure:
 *   - -ENODEV if rzv2h_gpt_pwminitialize() returns NULL.
 *   - Negated errno from pwm_register() on failure.
 *
 ****************************************************************************/

int rzv2h_pwm_setup(void)
{
  struct pwm_lowerhalf_s *pwm;
  int ret;

#ifdef CONFIG_RZV2H_GPT0_PWM
  pwm = rzv2h_gpt_pwminitialize(0);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT0 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm0", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm0) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT1_PWM
  pwm = rzv2h_gpt_pwminitialize(1);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT1 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm1", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm1) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT2_PWM
  pwm = rzv2h_gpt_pwminitialize(2);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT2 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm2", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm2) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT3_PWM
  pwm = rzv2h_gpt_pwminitialize(3);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT3 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm3", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm3) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT4_PWM
  pwm = rzv2h_gpt_pwminitialize(4);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT4 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm4", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm4) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT5_PWM
  pwm = rzv2h_gpt_pwminitialize(5);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT5 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm5", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm5) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT6_PWM
  pwm = rzv2h_gpt_pwminitialize(6);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT6 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm6", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm6) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT7_PWM
  pwm = rzv2h_gpt_pwminitialize(7);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT7 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm7", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm7) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT8_PWM
  pwm = rzv2h_gpt_pwminitialize(8);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT8 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm8", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm8) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT9_PWM
  pwm = rzv2h_gpt_pwminitialize(9);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT9 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm9", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm9) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT10_PWM
  pwm = rzv2h_gpt_pwminitialize(10);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT10 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm10", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm10) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT11_PWM
  pwm = rzv2h_gpt_pwminitialize(11);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT11 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm11", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm11) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT12_PWM
  pwm = rzv2h_gpt_pwminitialize(12);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT12 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm12", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm12) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT13_PWM
  pwm = rzv2h_gpt_pwminitialize(13);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT13 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm13", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm13) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT14_PWM
  pwm = rzv2h_gpt_pwminitialize(14);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT14 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm14", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm14) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_GPT15_PWM
  pwm = rzv2h_gpt_pwminitialize(15);
  if (pwm == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init GPT15 for PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm15", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register(/dev/pwm15) failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}

#endif /* CONFIG_RZV2H_GPT_PWM */
