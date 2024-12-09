/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_ledc.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

#include "esp_gpio.h"
#include "esp_ledc.h"
#include "riscv_internal.h"

#include <arch/chip/gpio_sig_map.h>
#include "esp_private/periph_ctrl.h"
#include "hal/ledc_hal.h"
#include "hal/ledc_types.h"
#include "soc/soc_caps.h"
#include "clk_ctrl_os.h"
#include "esp_clk_tree.h"
#include "esp_attr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LEDC_TIMER_DIV_NUM_MAX    (0x3FFFF)

#define LEDC_IS_DIV_INVALID(div)  ((div) <= LEDC_LL_FRACTIONAL_MAX || \
                                   (div) > LEDC_TIMER_DIV_NUM_MAX)

/* Precision degree only affects RC_FAST, other clock sources' frequences are
 * fixed values. For targets that do not support RC_FAST calibration, can
 * only use its approximate value.
 */

#if SOC_CLK_RC_FAST_SUPPORT_CALIBRATION
#  define LEDC_CLK_SRC_FREQ_PRECISION ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED
#else
#  define LEDC_CLK_SRC_FREQ_PRECISION ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX
#endif

/* All chips have 4 internal timers */

#define LEDC_TIMERS               (4)

/* If PWM multi-channel is disabled, then only one channel is supported per
 * timer. Thus, the number of channels is the same as the number of timers.
 * Note that we only support the maximum of 6 PWM channels.
 */

#if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS > 1
#  define LEDC_CHANNELS           SOC_LEDC_CHANNEL_NUM
#else
#  define LEDC_CHANNELS           LEDC_TIMERS
#endif

/* LEDC timer0 channels and offset */

#ifdef CONFIG_ESPRESSIF_LEDC_TIMER0
#  if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS > 1
#    define LEDC_TIM0_CHANS       CONFIG_ESPRESSIF_LEDC_TIMER0_CHANNELS
#  else
#    define LEDC_TIM0_CHANS       (1)
#  endif
#  define LEDC_TIM0_CHANS_OFF     (0)
#endif

/* LEDC timer1 channels and offset */

#ifdef CONFIG_ESPRESSIF_LEDC_TIMER1
#  if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS > 1
#    define LEDC_TIM1_CHANS       CONFIG_ESPRESSIF_LEDC_TIMER1_CHANNELS
#  else
#    define LEDC_TIM1_CHANS       (1)
#  endif
#  define LEDC_TIM1_CHANS_OFF     (LEDC_TIM0_CHANS_OFF + LEDC_TIM0_CHANS)
#endif

/* LEDC timer2 channels and offset */

#ifdef CONFIG_ESPRESSIF_LEDC_TIMER2
#  if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS > 1
#    define LEDC_TIM2_CHANS       CONFIG_ESPRESSIF_LEDC_TIMER2_CHANNELS
#  else
#    define LEDC_TIM2_CHANS       (1)
#  endif
#  define LEDC_TIM2_CHANS_OFF     (LEDC_TIM1_CHANS_OFF + LEDC_TIM1_CHANS)
#endif

/* LEDC timer3 channels and offset */

#ifdef CONFIG_ESPRESSIF_LEDC_TIMER3
#  if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS > 1
#    define LEDC_TIM3_CHANS       CONFIG_ESPRESSIF_LEDC_TIMER3_CHANNELS
#  else
#    define LEDC_TIM3_CHANS       (1)
#  endif
#  define LEDC_TIM3_CHANS_OFF     (LEDC_TIM2_CHANS_OFF + LEDC_TIM2_CHANS)
#endif

/* Unititialized LEDC timer clock */

#define LEDC_SLOW_CLK_UNINIT      (-1)

/* Clock not found */

#define LEDC_CLK_NOT_FOUND        (0)

/* LEDC keep config */

#define LEDC_VAL_NO_CHANGE        (-1)

/* LEDC Timer default frequency */

#define LEDC_DEFAULT_FREQ         (1000)

/* Check max LEDC channels number */

#if CONFIG_ESPRESSIF_LEDC_TIMER0_CHANNELS + \
    CONFIG_ESPRESSIF_LEDC_TIMER1_CHANNELS + \
    CONFIG_ESPRESSIF_LEDC_TIMER2_CHANNELS + \
    CONFIG_ESPRESSIF_LEDC_TIMER3_CHANNELS > LEDC_CHANNELS
#  error "Too many LEDC channels. The maximum number of channels used " \
         "by all timers is 6 when multi-channel PWM is enabled and " \
         "4 when multi-channel PWM is disabled."
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* LEDC timer channel configuration */

struct esp_ledc_chan_s
{
  const uint8_t num;                    /* PWM channel ID */
  const uint8_t pin;                    /* PWM channel GPIO pin number */

  uint32_t duty;                        /* PWM channel current duty */
};

/* This structure represents the state of one LEDC timer */

struct esp_ledc_s
{
  const struct pwm_ops_s *ops;          /* PWM operations */

  const uint8_t timer;                  /* Timer ID */

  const uint8_t channels;               /* Timer channels number */
  struct esp_ledc_chan_s *chans;        /* Timer channels pointer */

  ledc_timer_bit_t duty_resolution;     /* Timer duty resolution */
  ledc_clk_cfg_t clk_cfg;               /* Timer clock configuration */
  uint32_t frequency;                   /* Timer current frequency */
  uint32_t reload;                      /* Timer current reload */
};

struct ledc_obj_s
{
  ledc_hal_context_t ledc_hal;               /* LEDC hal context */
  ledc_slow_clk_sel_t glb_clk;               /* LEDC global clock selection */
  bool timer_is_stopped[LEDC_TIMER_MAX];     /* Indicates whether each timer has been stopped */
  bool glb_clk_is_acquired[LEDC_TIMER_MAX];  /* Tracks whether the global clock is being acquired by each timer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if 0 /* Should be added when sleep is supported */
extern void esp_sleep_periph_use_8m(bool use_or_not);
#endif

static bool ledc_ctx_create(void);
static bool ledc_slow_clk_calibrate(void);
static uint32_t ledc_calculate_divisor(uint32_t src_clk_freq,
                                       int freq_hz,
                                       uint32_t precision);

static uint32_t ledc_auto_global_clk_div(int freq_hz,
                                         uint32_t precision,
                                         ledc_slow_clk_sel_t *clk_target);

static uint32_t ledc_auto_clk_divisor(int freq_hz,
                                      uint32_t precision,
                                      ledc_clk_src_t *clk_source,
                                      ledc_slow_clk_sel_t *clk_target);

static int ledc_timer_set(ledc_timer_t timer_sel,
                          uint32_t clock_divider,
                          uint32_t duty_resolution,
                          ledc_clk_src_t clk_src);

static int ledc_set_timer_div(ledc_timer_t timer_num,
                              ledc_clk_cfg_t clk_cfg,
                              int freq_hz,
                              int duty_resolution);

static int ledc_timer_resume(ledc_timer_t timer_sel);
static int ledc_timer_rst(ledc_timer_t timer_sel);
static int ledc_timer_pause(ledc_timer_t timer_sel);
static int setup_timer(struct esp_ledc_s *priv);
static int ledc_duty_config(ledc_channel_t channel,
                            int hpoint_val,
                            int duty_val,
                            ledc_duty_direction_t duty_direction,
                            uint32_t duty_num,
                            uint32_t duty_cycle,
                            uint32_t duty_scale);

static int ledc_set_duty_with_hpoint(ledc_channel_t channel,
                                     uint32_t duty,
                                     uint32_t hpoint);

static int ledc_channel_output_enable(ledc_channel_t channel);
static int ledc_channel_output_disable(ledc_channel_t channel);
static int ledc_update_duty(ledc_channel_t channel);
static uint32_t ledc_duty_bin_conversion(uint16_t nuttx_duty,
                                         uint32_t duty_resolution);

static int ledc_bind_channel_timer(ledc_channel_t channel,
                                   ledc_timer_t timer_sel);

static void setup_channel(struct esp_ledc_s *priv, int cn);
static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);

static int pwm_stop(struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* LEDC context */

static struct ledc_obj_s *p_ledc_obj = NULL;

/* LEDC available timer global clocks */

static const ledc_slow_clk_sel_t s_glb_clks[] = LEDC_LL_GLOBAL_CLOCKS;

/* Current RC_FAST frequency */

static uint32_t s_ledc_slow_clk_rc_fast_freq = 0;

/* LEDC PWM operations */

static const struct pwm_ops_s g_pwmops =
{
  .setup       = pwm_setup,
  .shutdown    = pwm_shutdown,
  .start       = pwm_start,
  .stop        = pwm_stop,
  .ioctl       = pwm_ioctl
};

/* LEDC channels table */

static struct esp_ledc_chan_s g_ledc_chans[LEDC_CHANNELS] =
{
  {
    .num       = 0,
    .pin       = CONFIG_ESPRESSIF_LEDC_CHANNEL0_PIN
  },
  {
    .num       = 1,
    .pin       = CONFIG_ESPRESSIF_LEDC_CHANNEL1_PIN
  },
  {
    .num       = 2,
    .pin       = CONFIG_ESPRESSIF_LEDC_CHANNEL2_PIN
  },
  {
    .num       = 3,
    .pin       = CONFIG_ESPRESSIF_LEDC_CHANNEL3_PIN
  },
#if LEDC_CHANNELS > 4
  {
    .num       = 4,
    .pin       = CONFIG_ESPRESSIF_LEDC_CHANNEL4_PIN
  },
  {
    .num       = 5,
    .pin       = CONFIG_ESPRESSIF_LEDC_CHANNEL5_PIN
  }
#endif
};

/* LEDC timer0 private data */

#ifdef CONFIG_ESPRESSIF_LEDC_TIMER0

#  if CONFIG_ESPRESSIF_LEDC_TIMER0_RESOLUTION > SOC_LEDC_TIMER_BIT_WIDTH
#    error "Invalid PWM Timer 0 resolution."
#  endif

static struct esp_ledc_s g_pwm0dev =
{
  .ops             = &g_pwmops,
  .timer           = 0,
  .channels        = LEDC_TIM0_CHANS,
  .chans           = &g_ledc_chans[LEDC_TIM0_CHANS_OFF],
  .duty_resolution = CONFIG_ESPRESSIF_LEDC_TIMER0_RESOLUTION,
  .clk_cfg         = LEDC_AUTO_CLK,
  .frequency       = LEDC_DEFAULT_FREQ
};
#endif /* CONFIG_ESPRESSIF_LEDC_TIMER0 */

/* LEDC timer1 private data */

#ifdef CONFIG_ESPRESSIF_LEDC_TIMER1

#  if CONFIG_ESPRESSIF_LEDC_TIMER1_RESOLUTION > SOC_LEDC_TIMER_BIT_WIDTH
#    error "Invalid PWM Timer 1 resolution."
#  endif

static struct esp_ledc_s g_pwm1dev =
{
  .ops             = &g_pwmops,
  .timer           = 1,
  .channels        = LEDC_TIM1_CHANS,
  .chans           = &g_ledc_chans[LEDC_TIM1_CHANS_OFF],
  .duty_resolution = CONFIG_ESPRESSIF_LEDC_TIMER1_RESOLUTION,
  .clk_cfg         = LEDC_AUTO_CLK,
  .frequency       = LEDC_DEFAULT_FREQ
};
#endif /* CONFIG_ESPRESSIF_LEDC_TIMER1 */

/* LEDC timer2 private data */

#ifdef CONFIG_ESPRESSIF_LEDC_TIMER2

#  if CONFIG_ESPRESSIF_LEDC_TIMER2_RESOLUTION > SOC_LEDC_TIMER_BIT_WIDTH
#    error "Invalid PWM Timer 2 resolution."
#  endif

static struct esp_ledc_s g_pwm2dev =
{
  .ops             = &g_pwmops,
  .timer           = 2,
  .channels        = LEDC_TIM2_CHANS,
  .chans           = &g_ledc_chans[LEDC_TIM2_CHANS_OFF],
  .duty_resolution = CONFIG_ESPRESSIF_LEDC_TIMER2_RESOLUTION,
  .clk_cfg         = LEDC_AUTO_CLK,
  .frequency       = LEDC_DEFAULT_FREQ
};
#endif /* CONFIG_ESPRESSIF_LEDC_TIMER2 */

/* LEDC timer3 private data */

#ifdef CONFIG_ESPRESSIF_LEDC_TIMER3

#  if CONFIG_ESPRESSIF_LEDC_TIMER3_RESOLUTION > SOC_LEDC_TIMER_BIT_WIDTH
#    error "Invalid PWM Timer 3 resolution."
#  endif

static struct esp_ledc_s g_pwm3dev =
{
  .ops             = &g_pwmops,
  .timer           = 3,
  .channels        = LEDC_TIM3_CHANS,
  .chans           = &g_ledc_chans[LEDC_TIM3_CHANS_OFF],
  .duty_resolution = CONFIG_ESPRESSIF_LEDC_TIMER3_RESOLUTION,
  .clk_cfg         = LEDC_AUTO_CLK,
  .frequency       = LEDC_DEFAULT_FREQ
};
#endif /* CONFIG_ESPRESSIF_LEDC_TIMER3 */

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: ledc_ctx_create
 *
 * Description:
 *   Create LEDC context.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   True if context was created, false otherwise.
 *
 ****************************************************************************/

static bool ledc_ctx_create(void)
{
  bool new_ctx = false;
  irqstate_t flags;

  flags = enter_critical_section();

  if (!p_ledc_obj)
    {
      struct ledc_obj_s *ledc_new_mode_obj;
      ledc_new_mode_obj = (struct ledc_obj_s *)
        kmm_calloc(1, sizeof(struct ledc_obj_s));
      if (ledc_new_mode_obj)
        {
          new_ctx = true;

          /* Only ESP32 supports High Speed mode */

          ledc_hal_init(&(ledc_new_mode_obj->ledc_hal), LEDC_LOW_SPEED_MODE);
          ledc_new_mode_obj->glb_clk = LEDC_SLOW_CLK_UNINIT;
          p_ledc_obj = ledc_new_mode_obj;
          periph_module_enable(PERIPH_LEDC_MODULE);
      }
  }

  leave_critical_section(flags);

  return new_ctx;
}

/****************************************************************************
 * Name: ledc_slow_clk_calibrate
 *
 * Description:
 *   Calibrate RC_FAST clock.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   True if calibration was successful, false otherwise.
 *
 ****************************************************************************/

static bool ledc_slow_clk_calibrate(void)
{
  if (periph_rtc_dig_clk8m_enable())
    {
      s_ledc_slow_clk_rc_fast_freq = periph_rtc_dig_clk8m_get_freq();
#if !SOC_CLK_RC_FAST_SUPPORT_CALIBRATION
      pwminfo("Calibration cannot be performed."
              "Approximate RC_FAST_CLK : %"PRIu32" Hz\n",
              s_ledc_slow_clk_rc_fast_freq);
#else
      pwminfo("Calibrate RC_FAST_CLK : %"PRIu32" Hz",
              s_ledc_slow_clk_rc_fast_freq);
#endif
      return true;
    }

  pwmerr("Calibrate RC_FAST_CLK failed\n");
  return false;
}

/****************************************************************************
 * Name: ledc_calculate_divisor
 *
 * Description:
 *   Calculate the divisor to achieve the desired frequency.
 *
 * Input Parameters:
 *   src_clk_freq - The source clock frequency.
 *   freq_hz      - The desired frequency.
 *   precision    - The granularity of the clock.
 *
 * Returned Value:
 *   The divisor parameter to use to achieve the frequency requested.
 *
 ****************************************************************************/

static inline uint32_t ledc_calculate_divisor(uint32_t src_clk_freq,
                                              int freq_hz,
                                              uint32_t precision)
{
  /* In order to find the right divisor, we need to divide the source clock
   * frequency by the desired frequency. However, two things to note here:
   * - The lowest LEDC_LL_FRACTIONAL_BITS bits of the result are the
   *   FRACTIONAL part. The higher bits represent the integer part, this is
   *   why we need to right shift the source frequency.
   * - The `precision` parameter represents the granularity of the clock. It
   *   **must** be a power of 2. It means that the resulted divisor is
   *   a multiplier of `precision`.
   *
   * Let's take a concrete example, we need to generate a 5KHz clock out of
   * a 80MHz clock (APB).
   * If the precision is 1024 (10 bits), the resulted multiplier is:
   * (80000000 << 8) / (5000 * 1024) = 4000 (0xfa0)
   * Let's ignore the fractional part to simplify the explanation, so we get
   * a result of 15 (0xf).
   * This can be interpreted as: every 15 "precision" ticks, the resulted
   * clock will go high, where one precision tick is made out of 1024 source
   * clock ticks.
   * Thus, every `15 * 1024` source clock ticks, the resulted clock will go
   * high.
   *
   * NOTE: We are also going to round up the value when necessary, thanks to:
   * (freq_hz * precision) / 2
   */

  return (((uint64_t)src_clk_freq << LEDC_LL_FRACTIONAL_BITS) + \
          ((freq_hz * precision) / 2)) / (freq_hz * precision);
}

/****************************************************************************
 * Name: ledc_auto_global_clk_div
 *
 * Description:
 *   Try to find the clock with its divisor giving the frequency requested
 *   by the caller.
 *
 * Input Parameters:
 *   freq_hz    - Frequency to achieve;
 *   precision  - Precision of the frequency to achieve;
 *   clk_target - Clock target to use.
 *
 * Returned Value:
 *   The divisor parameter to use to achieve the frequency requested.
 *
 ****************************************************************************/

static uint32_t ledc_auto_global_clk_div(int freq_hz,
                                         uint32_t precision,
                                         ledc_slow_clk_sel_t *clk_target)
{
  uint32_t ret = LEDC_CLK_NOT_FOUND;
  uint32_t clk_freq = 0;

  /* This function will go through all the following clock sources to look
   * for a valid divisor which generates the requested frequency.
   */

  for (int i = 0; i < nitems(s_glb_clks); i++)
    {
      /* Before calculating the divisor, we need to have the RC_FAST
       * frequency. If it hasn't been measured yet, try calibrating
       * it now.
       */

      if (s_glb_clks[i] == LEDC_SLOW_CLK_RC_FAST &&
          s_ledc_slow_clk_rc_fast_freq == 0 &&
          !ledc_slow_clk_calibrate())
        {
          pwminfo("Unable to retrieve RC_FAST clock frequency, skipping it");
          continue;
        }

      esp_clk_tree_src_get_freq_hz((soc_module_clk_t)s_glb_clks[i],
                                   LEDC_CLK_SRC_FREQ_PRECISION,
                                   &clk_freq);

      uint32_t div_param = ledc_calculate_divisor(clk_freq,
                                                  freq_hz,
                                                  precision);

      /* If the divisor is valid, we can return this value. */

      if (!LEDC_IS_DIV_INVALID(div_param))
        {
          *clk_target = s_glb_clks[i];
          ret = div_param;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ledc_auto_clk_divisor
 *
 * Description:
 *   Try to find the clock with its divisor giving the frequency requested
 *   by the caller.
 *
 * Input Parameters:
 *   freq_hz    - Frequency to achieve;
 *   precision  - Precision of the frequency to achieve;
 *   clk_source - Clock source to use;
 *   clk_target - Clock target to use.
 *
 * Returned Value:
 *   The divisor parameter to use to achieve the frequency requested.
 *
 ****************************************************************************/

static uint32_t ledc_auto_clk_divisor(int freq_hz,
                                      uint32_t precision,
                                      ledc_clk_src_t *clk_source,
                                      ledc_slow_clk_sel_t *clk_target)
{
  uint32_t ret = LEDC_CLK_NOT_FOUND;

  uint32_t div_param_global = ledc_auto_global_clk_div(freq_hz,
                                                       precision,
                                                       clk_target);

  if (div_param_global != LEDC_CLK_NOT_FOUND)
    {
      pwminfo("Found a global clock source for frequency %d Hz "
              "and precision 0x%"PRIx32"\n", freq_hz, precision);
      *clk_source = LEDC_SCLK;
      ret = div_param_global;
    }
  else
    {
      pwminfo("Could not find a global clock source for frequency %d Hz "
              "and precision 0x%"PRIx32"\n", freq_hz, precision);
    }

  return ret;
}

/****************************************************************************
 * Name: ledc_timer_set
 *
 * Description:
 *   Set LEDC timer.
 *
 * Input Parameters:
 *   timer_sel      - Select the timer to be set;
 *   clock_divider  - The divider of the clock source;
 *   duty_resolution- The duty resolution of the timer;
 *   clk_src        - The clock source of the timer;
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int ledc_timer_set(ledc_timer_t timer_sel,
                          uint32_t clock_divider,
                          uint32_t duty_resolution,
                          ledc_clk_src_t clk_src)
{
  DEBUGASSERT(timer_sel < LEDC_TIMER_MAX);

  if (p_ledc_obj == NULL)
    {
      pwmerr("ERROR: LEDC not initialized\n");
      return -ENODEV;
    }

  irqstate_t flags;

  flags = enter_critical_section();

  ledc_hal_set_clock_divider(&(p_ledc_obj->ledc_hal),
                             timer_sel,
                             clock_divider);

  ledc_hal_set_duty_resolution(&(p_ledc_obj->ledc_hal),
                               timer_sel,
                               duty_resolution);

  ledc_hal_ls_timer_update(&(p_ledc_obj->ledc_hal), timer_sel);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: ledc_set_timer_div
 *
 * Description:
 *   Set LEDC timer divider.
 *
 * Input Parameters:
 *   timer_num - LEDC timer number;
 *   clk_cfg - LEDC timer clock source;
 *   freq_hz - LEDC timer frequency;
 *   duty_resolution - LEDC timer duty resolution.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int ledc_set_timer_div(ledc_timer_t timer_num,
                              ledc_clk_cfg_t clk_cfg,
                              int freq_hz,
                              int duty_resolution)
{
  irqstate_t flags;
  uint32_t div_param = 0;
  int i;
  const uint32_t precision = (0x1 << duty_resolution);

  /* The clock sources are not initialized on purpose. To produce compiler
   * warning if used but the selector functions don't set them properly.
   */

  /* Timer-specific mux. Set to timer-specific clock or LEDC_SCLK if a global
   * clock is used.
   */

  ledc_clk_src_t timer_clk_src;

  /* Global clock mux. Should be set when LEDC_SCLK is used in
   * LOW_SPEED_MODE. Otherwise left uninitialized.
   */

  ledc_slow_clk_sel_t glb_clk = LEDC_SLOW_CLK_UNINIT;

  if (clk_cfg == LEDC_AUTO_CLK)
    {
      /* User hasn't specified the speed, we should try to guess it. */

      pwminfo("Using auto clock source selection\n");

      div_param = ledc_auto_clk_divisor(freq_hz,
                                        precision,
                                        &timer_clk_src,
                                        &glb_clk);
    }
  else if (clk_cfg == LEDC_USE_RC_FAST_CLK)
    {
      pwminfo("Using RC_FAST clock source\n");

      /* Before calculating the divisor, we need to have the RC_FAST
       * frequency. If it hasn't been measured yet, try calibrating
       * it now.
       */

      if (s_ledc_slow_clk_rc_fast_freq == 0 &&
          ledc_slow_clk_calibrate() == false)
        {
          goto error;
        }

      /* Set the global clock source */

      timer_clk_src = LEDC_SCLK;
      glb_clk =  LEDC_SLOW_CLK_RC_FAST;

      /* We have the RC_FAST clock frequency now. */

      div_param = ledc_calculate_divisor(s_ledc_slow_clk_rc_fast_freq,
                                         freq_hz,
                                         precision);

      if (LEDC_IS_DIV_INVALID(div_param))
        {
          div_param = LEDC_CLK_NOT_FOUND;
        }
    }
  else
    {
      timer_clk_src = LEDC_SCLK;
      glb_clk = (ledc_slow_clk_sel_t)clk_cfg;

      uint32_t src_clk_freq = 0;
      esp_clk_tree_src_get_freq_hz((soc_module_clk_t)clk_cfg,
                                    LEDC_CLK_SRC_FREQ_PRECISION,
                                    &src_clk_freq);

      div_param = ledc_calculate_divisor(src_clk_freq, freq_hz, precision);

      if (LEDC_IS_DIV_INVALID(div_param))
        {
          div_param = LEDC_CLK_NOT_FOUND;
        }
    }

  if (div_param == LEDC_CLK_NOT_FOUND)
    {
      goto error;
    }

  pwminfo("Using clock source %d (in slow mode). Divisor: 0x%"PRIx32,
          timer_clk_src,
          div_param);

  /* The following block configures the global clock.
   * Arriving here, variable glb_clk must have been assigned to one of the
   * ledc_slow_clk_sel_t enum values
   */

  ASSERT(timer_clk_src == LEDC_SCLK);
  ASSERT(glb_clk != LEDC_SLOW_CLK_UNINIT);

  flags = enter_critical_section();

  if (p_ledc_obj->glb_clk != LEDC_SLOW_CLK_UNINIT &&
      p_ledc_obj->glb_clk != glb_clk)
    {
      for (i = 0; i < LEDC_TIMER_MAX; i++)
        {
          if (i != timer_num && p_ledc_obj->glb_clk_is_acquired[i])
            {
              leave_critical_section(flags);
              pwmerr("Timer clock conflict. Already is %d but attempt to %d",
                     p_ledc_obj->glb_clk,
                     glb_clk);
            }
        }
    }

  if (timer_num == LEDC_TIMER_MAX - 1 &&
      p_ledc_obj->glb_clk_is_acquired[timer_num - 1])
    {
      return -EINVAL;
    }

  p_ledc_obj->glb_clk_is_acquired[timer_num] = true;
  if (p_ledc_obj->glb_clk != glb_clk)
    {
      p_ledc_obj->glb_clk = glb_clk;
      ledc_hal_set_slow_clk_sel(&(p_ledc_obj->ledc_hal), glb_clk);
    }

  leave_critical_section(flags);

  pwminfo("In slow speed mode. Global clock: %d", glb_clk);

  /* Keep ESP_PD_DOMAIN_RC_FAST on during light sleep */

#if 0 /* Should be added when sleep is supported */
#ifndef CONFIG_ESPRESSIF_ESP32H2 /* TODO: Remove when H2 light sleep is supported */
  esp_sleep_periph_use_8m(glb_clk == LEDC_SLOW_CLK_RC_FAST);
#endif
#endif

  /* The divisor is correct, we can write in the hardware. */

  ledc_timer_set(timer_num, div_param, duty_resolution, timer_clk_src);
  return OK;

error:
  pwmerr("Requested frequency and duty resolution can not be achieved. "
         "Try reducing the frequency or timer resolution.\n");

  return -EINVAL;
}

/****************************************************************************
 * Name: ledc_timer_resume
 *
 * Description:
 *   Resume a LEDC timer.
 *
 * Input Parameters:
 *   timer_sel - LEDC timer id.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int ledc_timer_resume(ledc_timer_t timer_sel)
{
  DEBUGASSERT(timer_sel < LEDC_TIMER_MAX);

  if (p_ledc_obj == NULL)
    {
      pwmerr("ERROR: LEDC not initialized\n");
      return -ENODEV;
    }

  irqstate_t flags;

  flags = enter_critical_section();

  p_ledc_obj->timer_is_stopped[timer_sel] = false;
  ledc_hal_timer_resume(&(p_ledc_obj->ledc_hal), timer_sel);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ledc_timer_rst
 *
 * Description:
 *   Reset a LEDC timer.
 *
 * Input Parameters:
 *   timer_sel - LEDC timer id.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int ledc_timer_rst(ledc_timer_t timer_sel)
{
  DEBUGASSERT(timer_sel < LEDC_TIMER_MAX);

  if (p_ledc_obj == NULL)
    {
      pwmerr("ERROR: LEDC not initialized\n");
      return -ENODEV;
    }

  irqstate_t flags;

  flags = enter_critical_section();

  ledc_hal_timer_rst(&(p_ledc_obj->ledc_hal), timer_sel);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ledc_timer_pause
 *
 * Description:
 *   Pause a LEDC timer.
 *
 * Input Parameters:
 *   timer_sel - LEDC timer id.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int ledc_timer_pause(ledc_timer_t timer_sel)
{
  DEBUGASSERT(timer_sel < LEDC_TIMER_MAX);

  if (p_ledc_obj == NULL)
    {
      pwmerr("ERROR: LEDC not initialized\n");
      return -ENODEV;
    }

  irqstate_t flags;

  flags = enter_critical_section();

  p_ledc_obj->timer_is_stopped[timer_sel] = true;
  ledc_hal_timer_pause(&(p_ledc_obj->ledc_hal), timer_sel);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: setup_timer
 *
 * Description:
 *   Setup LEDC timer frequency and reload.
 *
 * Input Parameters:
 *   priv - A reference to the LEDC timer state structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int setup_timer(struct esp_ledc_s *priv)
{
  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->timer < LEDC_TIMER_MAX);
  DEBUGASSERT(priv->frequency > 0);
  DEBUGASSERT(priv->duty_resolution < LEDC_TIMER_BIT_MAX &&
              priv->duty_resolution > 0);

  int ret;

  if (!ledc_ctx_create() && !p_ledc_obj)
    {
      pwmerr("ERROR: No memory for LEDC context\n");
      PANIC();
    }

  ret = ledc_set_timer_div(priv->timer,
                           priv->clk_cfg,
                           priv->frequency,
                           priv->duty_resolution);

  if (ret == OK)
    {
      ledc_timer_pause(priv->timer);
      ledc_timer_rst(priv->timer);
    }

  return ret;
}

/****************************************************************************
 * Name: ledc_duty_config
 *
 * Description:
 *   Set the duty cycle of a PWM channel.
 *
 * Input Parameters:
 *   channel - LEDC channel index.
 *   hpoint_val - LEDC channel hpoint value.
 *   duty_val - LEDC channel duty value.
 *   duty_direction - LEDC channel duty direction.
 *   duty_num - LEDC channel duty number.
 *   duty_cycle - LEDC channel duty cycle.
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

static IRAM_ATTR int ledc_duty_config(ledc_channel_t channel,
                                      int hpoint_val,
                                      int duty_val,
                                      ledc_duty_direction_t duty_direction,
                                      uint32_t duty_num,
                                      uint32_t duty_cycle,
                                      uint32_t duty_scale)
{
  if (hpoint_val >= 0)
    {
      ledc_hal_set_hpoint(&(p_ledc_obj->ledc_hal), channel, hpoint_val);
    }

  if (duty_val >= 0)
    {
      ledc_hal_set_duty_int_part(&(p_ledc_obj->ledc_hal), channel, duty_val);
    }

  ledc_hal_set_duty_direction(&(p_ledc_obj->ledc_hal),
                              channel,
                              duty_direction);

  ledc_hal_set_duty_num(&(p_ledc_obj->ledc_hal), channel, duty_num);
  ledc_hal_set_duty_cycle(&(p_ledc_obj->ledc_hal), channel, duty_cycle);
  ledc_hal_set_duty_scale(&(p_ledc_obj->ledc_hal), channel, duty_scale);

#if SOC_LEDC_GAMMA_CURVE_FADE_SUPPORTED
  ledc_hal_set_duty_range_wr_addr(&(p_ledc_obj->ledc_hal), channel, 0);
  ledc_hal_set_range_number(&(p_ledc_obj->ledc_hal), channel, 1);
#endif
  return OK;
}

/****************************************************************************
 * Name: ledc_set_duty_with_hpoint
 *
 * Description:
 *   Set the duty cycle of a PWM channel with a given hpoint value.
 *
 * Input Parameters:
 *   channel - LEDC channel index.
 *   duty - Duty cycle value.
 *   hpoint - Hpoint value.
 *
 * Returned Value:
 *   Ok on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int ledc_set_duty_with_hpoint(ledc_channel_t channel,
                                     uint32_t duty,
                                     uint32_t hpoint)
{
  DEBUGASSERT(channel < LEDC_CHANNEL_MAX);
  DEBUGASSERT(hpoint <= LEDC_LL_HPOINT_VAL_MAX);

  if (p_ledc_obj == NULL)
    {
      pwmerr("ERROR: LEDC not initialized\n");
      return -ENODEV;
    }

  irqstate_t flags;

  flags = enter_critical_section();

  ledc_duty_config(channel, hpoint, duty, LEDC_DUTY_DIR_INCREASE, 1, 1, 0);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ledc_channel_output_enable
 *
 * Description:
 *   Enable LEDC channel output.
 *
 * Input Parameters:
 *   channel - LEDC channel index.
 *
 * Returned Value:
 *   Ok on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int ledc_channel_output_enable(ledc_channel_t channel)
{
  DEBUGASSERT(channel < LEDC_CHANNEL_MAX);

  if (p_ledc_obj == NULL)
    {
      pwmerr("ERROR: LEDC not initialized\n");
      return -ENODEV;
    }

  ledc_hal_set_sig_out_en(&(p_ledc_obj->ledc_hal), channel, true);
  ledc_hal_set_duty_start(&(p_ledc_obj->ledc_hal), channel, true);

  return OK;
}

/****************************************************************************
 * Name: ledc_channel_output_disable
 *
 * Description:
 *   Disable LEDC channel output.
 *
 * Input Parameters:
 *   channel - LEDC channel index.
 *
 * Returned Value:
 *   Ok on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int ledc_channel_output_disable(ledc_channel_t channel)
{
  DEBUGASSERT(channel < LEDC_CHANNEL_MAX);

  if (p_ledc_obj == NULL)
    {
      pwmerr("ERROR: LEDC not initialized\n");
      return -ENODEV;
    }

  irqstate_t flags;

  flags = enter_critical_section();

  ledc_hal_set_idle_level(&(p_ledc_obj->ledc_hal), channel, 0);
  ledc_hal_set_sig_out_en(&(p_ledc_obj->ledc_hal), channel, false);
  ledc_hal_set_duty_start(&(p_ledc_obj->ledc_hal), channel, false);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ledc_update_duty
 *
 * Description:
 *   Update the duty cycle of a PWM channel.
 *
 * Input Parameters:
 *   channel - LEDC channel index.
 *
 * Returned Value:
 *   Ok on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int ledc_update_duty(ledc_channel_t channel)
{
  DEBUGASSERT(channel < LEDC_CHANNEL_MAX);

  if (p_ledc_obj == NULL)
    {
      pwmerr("ERROR: LEDC not initialized\n");
      return -ENODEV;
    }

  irqstate_t flags;

  ledc_channel_output_enable(channel);

  flags = enter_critical_section();
  ledc_hal_ls_channel_update(&(p_ledc_obj->ledc_hal), channel);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ledc_duty_bin_conversion
 *
 * Description:
 *   Truncate or expand the duty cycle value to fit the duty resolution.
 *
 * Input Parameters:
 *   nuttx_duty - Duty cycle value from nuttx.
 *   duty_resolution - Duty resolution of the channel.
 *
 * Returned Value:
 *   Truncated or expanded duty cycle value.
 *
 ****************************************************************************/

static uint32_t ledc_duty_bin_conversion(uint16_t nuttx_duty,
                                         uint32_t duty_resolution)
{
  /* Calculate the maximum value based on the duty resolution */

  uint32_t max_value = (1 << duty_resolution) - 1;

  /* Map the value to the specified range */

  uint32_t mapped_value = (nuttx_duty * max_value) / UINT16_MAX;

  return mapped_value;
}

/****************************************************************************
 * Name: ledc_bind_channel_timer
 *
 * Description:
 *   Bind a LEDC channel to a LEDC timer.
 *
 * Input Parameters:
 *   channel - LEDC channel index.
 *   timer_sel - LEDC timer index.
 *
 * Returned Value:
 *   Ok on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

static int ledc_bind_channel_timer(ledc_channel_t channel,
                                   ledc_timer_t timer_sel)
{
  DEBUGASSERT(timer_sel < LEDC_TIMER_MAX);

  if (p_ledc_obj == NULL)
    {
      pwmerr("ERROR: LEDC not initialized\n");
      return -ENODEV;
    }

  irqstate_t flags;

  flags = enter_critical_section();

  ledc_hal_bind_channel_timer(&(p_ledc_obj->ledc_hal), channel, timer_sel);
  ledc_hal_ls_channel_update(&(p_ledc_obj->ledc_hal), channel);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: setup_channel
 *
 * Description:
 *   Setup LEDC timer channel duty.
 *
 * Input Parameters:
 *   priv - A reference to the LEDC timer state structure
 *   cn   - Timer channel number
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void setup_channel(struct esp_ledc_s *priv, int cn)
{
  irqstate_t flags;
  bool new_ctx_created;
  struct esp_ledc_chan_s *chan = &priv->chans[cn];

  new_ctx_created = ledc_ctx_create();
  if (!new_ctx_created && !p_ledc_obj)
    {
      pwmerr("ERROR: No memory for LEDC context\n");
      PANIC();
    }
#ifndef CONFIG_ESPRESSIF_ESP32H2
  /* On such targets, the default ledc core(global) clock does not connect to
   * any clock source. Setting channel configurations and updating bits
   * before core clock is enabled could lead to an error.
   * Therefore, we should connect the core clock to a real clock source to
   * enable it before any ledc register operation happens.
   * It can be switched to the other desired clock sources to meet the output
   * PWM frequency requirements later at timer configuration.
   * So we consider the glb_clk still as LEDC_SLOW_CLK_UNINIT.
   */

  else if (new_ctx_created)
    {
      flags = enter_critical_section();

      if (p_ledc_obj->glb_clk == LEDC_SLOW_CLK_UNINIT)
        {
          ledc_hal_set_slow_clk_sel(&(p_ledc_obj->ledc_hal),
                                    LEDC_LL_GLOBAL_CLK_DEFAULT);
        }

      leave_critical_section(flags);
    }
#endif

  ledc_set_duty_with_hpoint(chan->num,
                            chan->duty,
                            CONFIG_ESPRESSIF_LEDC_HPOINT);

  ledc_bind_channel_timer(chan->num, priv->timer);

  flags = enter_critical_section();
  ledc_hal_set_fade_end_intr(&(p_ledc_obj->ledc_hal), chan->num, false);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened. The lower half driver
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

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct esp_ledc_s *priv = (struct esp_ledc_s *)dev;
  int ret;

  pwminfo("Initializing PWM Timer %d with default frequency of %d Hz\n",
          priv->timer,
          LEDC_DEFAULT_FREQ);

  ret = setup_timer(priv);

  if (ret != OK)
    {
      pwmerr("ERROR: Failed to setup timer\n");
      return ret;
    }

  /* Setup channel GPIO pins */

  for (int i = 0; i < priv->channels; i++)
    {
      setup_channel(priv, i);

      pwminfo("Channel %d mapped to pin %d\n", priv->chans[i].num,
              priv->chans[i].pin);

      esp_configgpio(priv->chans[i].pin, OUTPUT | PULLUP);
      esp_gpio_matrix_out(priv->chans[i].pin,
                          LEDC_LS_SIG_OUT0_IDX + priv->chans[i].num,
                          0, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed. The lower half driver
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
  struct esp_ledc_s *priv = (struct esp_ledc_s *)dev;
#ifdef CONFIG_PWM_NCHANNELS
  int channels = MIN(priv->channels, CONFIG_PWM_NCHANNELS);
#else
  int channels = 1;
#endif

  pwminfo("Shutting down PWM Timer %d\n", priv->timer);

  /* Stop timer */

  pwm_stop(dev);

  /* Clear timer and channel configuration */

  priv->frequency = LEDC_DEFAULT_FREQ;

  for (int i = 0; i < channels; i++)
    {
      priv->chans[i].duty = 0;
    }

  if (p_ledc_obj != NULL)
    {
      periph_module_disable(PERIPH_LEDC_MODULE);
      kmm_free(p_ledc_obj);
      p_ledc_obj = NULL;
      s_ledc_slow_clk_rc_fast_freq = 0;
    }
  else
    {
      pwmerr("ERROR: LEDC not initialized\n");
      return -ENODEV;
    }

  return 0;
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

static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  uint32_t duty;
  int ret = OK;
  irqstate_t flags;
  struct esp_ledc_s *priv = (struct esp_ledc_s *)dev;
#ifdef CONFIG_PWM_NCHANNELS
  int channels = MIN(priv->channels, CONFIG_PWM_NCHANNELS);
#else
  int channels = 1;
#endif

  /* Update timer with given PWM timer frequency */

  if (priv->frequency != info->frequency)
    {
      pwminfo("Setting Timer %" PRIu8
              " to frequency %" PRIu32
              " Hz using %" PRIu8 " bits of resolution\n",
              priv->timer,
              info->frequency,
              priv->duty_resolution);

      ret |= ledc_set_timer_div(priv->timer,
                                priv->clk_cfg,
                                info->frequency,
                                priv->duty_resolution);

      ret |= ledc_timer_rst(priv->timer);

      if (ret != OK)
        {
          pwmerr("ERROR: Failed to set timer configuration\n");
          return ret;
        }

      pwminfo("Timer %d successfully configured\n", priv->timer);
      priv->frequency = info->frequency;
    }

  /* Update timer with given PWM channel duty */

  for (int i = 0; i < channels; i++)
    {
#ifdef CONFIG_PWM_NCHANNELS
      duty = ledc_duty_bin_conversion(info->channels[i].duty,
                                      priv->duty_resolution);
#else
      duty = ledc_duty_bin_conversion(info[i].duty,
                                      priv->duty_resolution);
#endif
      if (priv->chans[i].duty != duty)
        {
          uint32_t max_value = (1 << priv->duty_resolution) - 1;
          pwminfo("Setting PWM channel %" PRIu8
                  " to duty cycle %" PRIu32 "(%0.4f)\n",
                  priv->chans[i].num,
                  duty,
                  (float)duty / max_value);

          flags = enter_critical_section();

          ret |= ledc_duty_config(priv->chans[i].num,
                                  LEDC_VAL_NO_CHANGE,
                                  duty,
                                  LEDC_DUTY_DIR_INCREASE,
                                  1,
                                  1,
                                  0);

          leave_critical_section(flags);
          ret |= ledc_update_duty(priv->chans[i].num);

          if (ret != OK)
            {
              pwmerr("ERROR: Failed to set channel configuration\n");
              return ret;
            }

          pwminfo("Channel %d successfully configured\n",
                  priv->chans[i].num);

          priv->chans[i].duty = duty;
        }

      ledc_channel_output_enable(priv->chans[i].num);
    }

  ledc_timer_resume(priv->timer);

  return 0;
}

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  int i;
  int channel;
  struct esp_ledc_s *priv = (struct esp_ledc_s *)dev;

  pwminfo("Stopping PWM Timer %d\n", priv->timer);

  for (i = 0; i < priv->channels; i++)
    {
      ledc_channel_output_disable(priv->chans[i].num);
    }

  ledc_timer_pause(priv->timer);
  ledc_timer_rst(priv->timer);

  return 0;
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

static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  struct esp_ledc_s *priv = (struct esp_ledc_s *)dev;

  pwminfo("PWM Timer %d\n", priv->timer);
#endif

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_ledc_init
 *
 * Description:
 *   Initialize one LEDC timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.
 *
 * Returned Value:
 *   On success, a pointer to the ESP32-C3 LEDC lower half PWM driver is
 *   returned. NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *esp_ledc_init(int timer)
{
  struct esp_ledc_s *lower = NULL;

  pwminfo("PWM Timer %u initialized\n", timer);

  switch (timer)
    {
#ifdef CONFIG_ESPRESSIF_LEDC_TIMER0
      case 0:
        {
          lower = &g_pwm0dev;
          break;
        }
#endif

#ifdef CONFIG_ESPRESSIF_LEDC_TIMER1
      case 1:
        {
          lower = &g_pwm1dev;
          break;
        }
#endif

#ifdef CONFIG_ESPRESSIF_LEDC_TIMER2
      case 2:
        {
          lower = &g_pwm2dev;
          break;
        }
#endif

#ifdef CONFIG_ESPRESSIF_LEDC_TIMER3
      case 3:
        {
          lower = &g_pwm3dev;
          break;
        }
#endif

      default:
        {
          pwmerr("ERROR: No such timer configured %d\n", timer);
          lower = NULL;
          break;
        }
    }

  return (struct pwm_lowerhalf_s *)lower;
}
