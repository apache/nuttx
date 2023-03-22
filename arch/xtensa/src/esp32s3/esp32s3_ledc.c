/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_ledc.c
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

#include <sys/param.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include "esp32s3_clockconfig.h"
#include "esp32s3_gpio.h"
#include "esp32s3_ledc.h"

#include "xtensa.h"
#include "hardware/esp32s3_ledc.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_gpio_sigmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDC total timers */

#define LEDC_TIMERS               (4)

/* LEDC total channels */

#if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS > 1
#  define LEDC_CHANNELS           (8)
#else
#  define LEDC_CHANNELS           (4)
#endif

/* LEDC timer0 channels and offset */

#ifdef CONFIG_ESP32S3_LEDC_TIM0
#  if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS > 1
#    define LEDC_TIM0_CHANS       CONFIG_ESP32S3_LEDC_TIM0_CHANNELS
#  else
#    define LEDC_TIM0_CHANS       (1)
#  endif
#    define LEDC_TIM0_CHANS_OFF   (0)
#endif

/* LEDC timer1 channels and offset */

#ifdef CONFIG_ESP32S3_LEDC_TIM1
#  if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS > 1
#    define LEDC_TIM1_CHANS       CONFIG_ESP32S3_LEDC_TIM1_CHANNELS
#  else
#    define LEDC_TIM1_CHANS       (1)
#  endif
#  define LEDC_TIM1_CHANS_OFF     (LEDC_TIM0_CHANS_OFF + LEDC_TIM0_CHANS)
#endif

/* LEDC timer2 channels and offset */

#ifdef CONFIG_ESP32S3_LEDC_TIM2
#  if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS > 1
#    define LEDC_TIM2_CHANS       CONFIG_ESP32S3_LEDC_TIM2_CHANNELS
#  else
#    define LEDC_TIM2_CHANS       (1)
#  endif

#  define LEDC_TIM2_CHANS_OFF     (LEDC_TIM1_CHANS_OFF + LEDC_TIM1_CHANS)
#endif

/* LEDC timer3 channels and offset */

#ifdef CONFIG_ESP32S3_LEDC_TIM3
#  if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS > 1
#    define LEDC_TIM3_CHANS       CONFIG_ESP32S3_LEDC_TIM3_CHANNELS
#  else
#    define LEDC_TIM3_CHANS       (1)
#  endif

#  define LEDC_TIM3_CHANS_OFF     (LEDC_TIM2_CHANS_OFF + LEDC_TIM2_CHANS)
#endif

/* LEDC clock resource */

#define LEDC_CLK_RES              (1)         /* APB clock */

/* LEDC timer max reload */

#define LEDC_RELOAD_MAX           (16384)    /* 2^14 */

/* LEDC timer max clock divider parameter */

#define LEDC_CLKDIV_MAX           (262144)    /* 2^18 */

/* LEDC timer registers mapping */

#define LEDC_TIMER_REG(r, n)      ((r) + (n) * (LEDC_TIMER1_CONF_REG - \
                                                LEDC_TIMER0_CONF_REG))

/* LEDC timer channel registers mapping */

#define setbits(bs, a)            modifyreg32(a, 0, bs)
#define resetbits(bs, a)          modifyreg32(a, bs, 0)

#define LEDC_CHAN_REG(r, n)       ((r) + (n) * (LEDC_CH1_CONF0_REG - \
                                                LEDC_CH0_CONF0_REG))

#define SET_TIMER_BITS(t, r, b)   setbits(b, LEDC_TIMER_REG(r, (t)->num));
#define SET_TIMER_REG(t, r, v)    putreg32(v, LEDC_TIMER_REG(r, (t)->num));

#define SET_CHAN_BITS(c, r, b)    setbits(b, LEDC_CHAN_REG(r, (c)->num));
#define SET_CHAN_REG(c, r, v)     putreg32(v, LEDC_CHAN_REG(r, (c)->num));

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* LEDC timer channel configuration */

struct esp32s3_ledc_chan_s
{
  const uint8_t num;                    /* Timer channel ID */
  const uint8_t pin;                    /* Timer channel GPIO pin number */
  uint16_t duty;                        /* Timer channel current duty */
};

/* This structure represents the state of one LEDC timer */

struct esp32s3_ledc_s
{
  const struct pwm_ops_s *ops;          /* PWM operations */

  const uint8_t num;                    /* Timer ID */

  const uint8_t channels;               /* Timer channels number */
  struct esp32s3_ledc_chan_s *chans;    /* Timer channels pointer */

  uint32_t frequency;                   /* Timer current frequency */
  uint32_t reload;                      /* Timer current reload */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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

static struct esp32s3_ledc_chan_s g_ledc_chans[LEDC_CHANNELS] =
{
  {
    .num       = 0,
    .pin       = CONFIG_ESP32S3_LEDC_CHANNEL0_PIN
  },

  {
    .num       = 1,
    .pin       = CONFIG_ESP32S3_LEDC_CHANNEL1_PIN
  },

  {
    .num       = 2,
    .pin       = CONFIG_ESP32S3_LEDC_CHANNEL2_PIN
  },

  {
    .num       = 3,
    .pin       = CONFIG_ESP32S3_LEDC_CHANNEL3_PIN
  },

#if LEDC_CHANNELS > 4
  {
    .num       = 4,
    .pin       = CONFIG_ESP32S3_LEDC_CHANNEL4_PIN
  },

  {
    .num       = 5,
    .pin       = CONFIG_ESP32S3_LEDC_CHANNEL5_PIN
  },

  {
    .num       = 6,
    .pin       = CONFIG_ESP32S3_LEDC_CHANNEL6_PIN
  },

  {
    .num       = 7,
    .pin       = CONFIG_ESP32S3_LEDC_CHANNEL7_PIN
  }
#endif
};

/* LEDC timer0 private data */

#ifdef CONFIG_ESP32S3_LEDC_TIM0
static struct esp32s3_ledc_s g_pwm0dev =
{
  .ops         = &g_pwmops,
  .num         = 0,
  .channels    = LEDC_TIM0_CHANS,
  .chans       = &g_ledc_chans[LEDC_TIM0_CHANS_OFF]
};
#endif /* CONFIG_ESP32S3_LEDC_TIM0 */

/* LEDC timer1 private data */

#ifdef CONFIG_ESP32S3_LEDC_TIM1
static struct esp32s3_ledc_s g_pwm1dev =
{
  .ops         = &g_pwmops,
  .num         = 1,
  .channels    = LEDC_TIM1_CHANS,
  .chans       = &g_ledc_chans[LEDC_TIM1_CHANS_OFF]
};
#endif /* CONFIG_ESP32S3_LEDC_TIM1 */

/* LEDC timer2 private data */

#ifdef CONFIG_ESP32S3_LEDC_TIM2
static struct esp32s3_ledc_s g_pwm2dev =
{
  .ops         = &g_pwmops,
  .num         = 2,
  .channels    = LEDC_TIM2_CHANS,
  .chans       = &g_ledc_chans[LEDC_TIM2_CHANS_OFF]
};
#endif /* CONFIG_ESP32S3_LEDC_TIM2 */

/* LEDC timer3 private data */

#ifdef CONFIG_ESP32S3_LEDC_TIM3
static struct esp32s3_ledc_s g_pwm3dev =
{
  .ops         = &g_pwmops,
  .num         = 3,
  .channels    = LEDC_TIM3_CHANS,
  .chans       = &g_ledc_chans[LEDC_TIM3_CHANS_OFF]
};
#endif /* CONFIG_ESP32S3_LEDC_TIM3 */

/* Clock reference count */

static uint32_t g_clk_ref;

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: ledc_enable_clk
 *
 * Description:
 *   Enable LEDC clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void ledc_enable_clk(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (g_clk_ref == 0)
    {
      setbits(SYSTEM_LEDC_CLK_EN, SYSTEM_PERIP_CLK_EN0_REG);
      resetbits(SYSTEM_LEDC_RST, SYSTEM_PERIP_RST_EN0_REG);

      putreg32(LEDC_CLK_RES, LEDC_CONF_REG);
      putreg32(LEDC_CLK_EN, LEDC_CONF_REG);

      pwminfo("Enable ledc clock\n");
    }

  g_clk_ref++;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ledc_disable_clk
 *
 * Description:
 *   Disable LEDC clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void ledc_disable_clk(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  g_clk_ref--;

  if (g_clk_ref == 0)
    {
      pwminfo("Disable ledc clock\n");

      setbits(SYSTEM_LEDC_RST, SYSTEM_PERIP_RST_EN0_REG);
      resetbits(SYSTEM_LEDC_CLK_EN, SYSTEM_PERIP_CLK_EN0_REG);
    }

  leave_critical_section(flags);
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
 *   None
 *
 ****************************************************************************/

static void setup_timer(struct esp32s3_ledc_s *priv)
{
  irqstate_t flags;
  uint32_t regval;
  uint32_t reload;
  uint32_t prescaler;
  uint32_t shift = 1;
  uint64_t pwmclk = esp_clk_apb_freq();

  /* Reset timer */

  SET_TIMER_BITS(priv, LEDC_TIMER0_CONF_REG, LEDC_TIMER0_RST);

  /* Calculate optimal values for the timer prescaler and for the timer
   * modulo register.  If 'frequency' is the desired frequency, then
   *
   *   tpmclk = pwmclk / presc
   *   frequency = tpmclk / reload
   *
   * ==>
   *
   *   reload = pwmclk / presc / frequency
   *
   * In ESP32S3, there are 3 clock resources for PWM:
   *
   *   1. APB clock (80 MHz)
   *   2. RTC clock (8 MHz)
   *   3. XTAL_CLK
   *
   * We mostly use APB clock generally.
   *
   * There are many solutions to this, but the best solution will be the one
   * that has the largest reload value and the smallest prescaler value.
   * That is the solution that should give us the most accuracy in the timer
   * control.  Subject to:
   *
   *   2 <= presc  <= 2^18(262144)
   *   1 <= clkdiv <= 2^10
   *
   * clkdiv has 8-bit decimal precision, so
   * clkdiv = pwmclk * 256 / 16384 / frequency would be optimal.
   *
   * Example:
   *
   *  pwmclk    = 80 MHz
   *  frequency = 100 Hz
   *
   *  presc     = 80,000,000 * 256 / 16,384 / 100
   *            = 12,500
   *  timclk    = 80,000,000 / (12,500 / 256)
   *            = 1,638,400
   *  counter   = 1,638,400 / 100
   *            = 16,384
   *            = 2^14
   *  shift     = 14
   */

  reload = (pwmclk * 256 / priv->frequency + LEDC_CLKDIV_MAX) /
           LEDC_CLKDIV_MAX;
  if (reload == 0)
    {
      reload = 1;
    }
  else if (reload > LEDC_RELOAD_MAX)
    {
      reload = LEDC_RELOAD_MAX;
    }

  for (int c = 2; c <= LEDC_RELOAD_MAX; c *= 2)
    {
      if (c * 2 > reload)
        {
          reload = c;
          break;
        }

      shift++;
    }

  prescaler = pwmclk * 256 / reload / priv->frequency;

  pwminfo("PWM timer%" PRIu8 " frequency=%0.4f reload=%" PRIu32 " shift=%"
          PRIu32 " prescaler=%0.4f\n",
          priv->num, (float)pwmclk / reload / ((float)prescaler / 256),
          reload, shift, (float)prescaler / 256);

  /* Store reload for channel duty */

  priv->reload = reload;

  flags = enter_critical_section();

  /* Set timer clock divide and reload */

  regval = (shift << LEDC_TIMER0_DUTY_RES_S) |
           (prescaler << LEDC_CLK_DIV_TIMER0_S);
  SET_TIMER_REG(priv, LEDC_TIMER0_CONF_REG, regval);

  /* Setup to timer to use APB clock (80MHz) */

  SET_TIMER_BITS(priv, LEDC_TIMER0_CONF_REG, LEDC_TICK_SEL_TIMER0);

  /* Update clock divide and reload to hardware */

  SET_TIMER_BITS(priv, LEDC_TIMER0_CONF_REG, LEDC_TIMER0_PARA_UP);

  leave_critical_section(flags);
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

static void setup_channel(struct esp32s3_ledc_s *priv, int cn)
{
  irqstate_t flags;
  uint32_t regval;
  struct esp32s3_ledc_chan_s *chan = &priv->chans[cn];

  /* Duty cycle:
   *
   * duty cycle = duty / 65536 * reload (fractional value)
   */

  regval = b16toi(chan->duty * priv->reload + b16HALF);

  pwminfo("channel=%" PRIu8 " duty=%" PRIu16 "(%0.4f) regval=%" PRIu32
          " reload=%" PRIu32 "\n",
          chan->num, chan->duty, (float)chan->duty / UINT16_MAX,
          regval, priv->reload);

  flags = enter_critical_section();

  /* Reset config 0 & 1 registers */

  SET_CHAN_REG(chan, LEDC_CH0_CONF0_REG, 0);
  SET_CHAN_REG(chan, LEDC_CH0_CONF1_REG, 0);

  /* Set pulse phase 0 */

  SET_CHAN_REG(chan, LEDC_CH0_HPOINT_REG, 0);

  /* Duty register uses bits [18:4]  */

  SET_CHAN_REG(chan, LEDC_CH0_DUTY_REG, regval << 4);

  /* Start GPIO output  */

  SET_CHAN_BITS(chan, LEDC_CH0_CONF0_REG, LEDC_SIG_OUT_EN_CH0);

  /* Start Duty counter  */

  SET_CHAN_BITS(chan, LEDC_CH0_CONF1_REG, LEDC_DUTY_START_CH0);

  /* Update duty and phase to hardware */

  SET_CHAN_BITS(chan, LEDC_CH0_CONF0_REG, LEDC_PARA_UP_CH0);

  leave_critical_section(flags);
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

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct esp32s3_ledc_s *priv = (struct esp32s3_ledc_s *)dev;

  pwminfo("PWM timer%u\n", priv->num);

  ledc_enable_clk();

  /* Setup channel GPIO pins */

  for (int i = 0; i < priv->channels; i++)
    {
      pwminfo("channel%d --> pin%d\n", priv->chans[i].num,
              priv->chans[i].pin);

      esp32s3_configgpio(priv->chans[i].pin, OUTPUT | PULLUP);
      esp32s3_gpio_matrix_out(priv->chans[i].pin,
                              LEDC_LS_SIG_OUT0_IDX + priv->chans[i].num,
                              0, 0);
    }

  return 0;
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
  struct esp32s3_ledc_s *priv = (struct esp32s3_ledc_s *)dev;
#ifdef CONFIG_PWM_NCHANNELS
  int channels = MIN(priv->channels, CONFIG_PWM_NCHANNELS);
#else
  int channels = 1;
#endif

  /* Stop timer */

  pwm_stop(dev);

  /* Clear timer and channel configuration */

  priv->frequency = 0;
  priv->reload    = 0;
  for (int i = 0; i < channels; i++)
    {
      priv->chans[i].duty = 0;
    }

  ledc_disable_clk();

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
  struct esp32s3_ledc_s *priv = (struct esp32s3_ledc_s *)dev;
#ifdef CONFIG_PWM_NCHANNELS
  int channels = MIN(priv->channels, CONFIG_PWM_NCHANNELS);
#else
  int channels = 1;
#endif

  pwminfo("PWM timer%d\n", priv->num);

  /* Update timer with given PWM timer frequency */

  if (priv->frequency != info->frequency)
    {
      priv->frequency = info->frequency;
      setup_timer(priv);
    }

  /* Update timer with given PWM channel duty */

  for (int i = 0; i < channels; i++)
    {
#ifdef CONFIG_PWM_NCHANNELS
      if (priv->chans[i].duty != info->channels[i].duty)
#else
      if (priv->chans[i].duty != info[i].duty)
#endif
        {
#ifdef CONFIG_PWM_NCHANNELS
          priv->chans[i].duty = info->channels[i].duty;
#else
          priv->chans[i].duty = info[i].duty;
#endif
          setup_channel(priv, i);
        }
    }

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
  irqstate_t flags;
  struct esp32s3_ledc_s *priv = (struct esp32s3_ledc_s *)dev;

  pwminfo("PWM timer%d\n", priv->num);

  flags = enter_critical_section();

  /* Stop timer */

  SET_TIMER_BITS(priv, LEDC_TIMER0_CONF_REG, LEDC_TIMER0_PAUSE);

  /* Reset timer */

  SET_TIMER_BITS(priv, LEDC_TIMER0_CONF_REG, LEDC_TIMER0_RST);

  leave_critical_section(flags);
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
  struct esp32s3_ledc_s *priv = (struct esp32s3_ledc_s *)dev;

  pwminfo("PWM timer%d\n", priv->num);
#endif

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_ledc_init
 *
 * Description:
 *   Initialize one LEDC timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.
 *
 * Returned Value:
 *   On success, a pointer to the ESP32S3 LEDC lower half PWM driver is
 *   returned. NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *esp32s3_ledc_init(int timer)
{
  struct esp32s3_ledc_s *lower = NULL;

  pwminfo("TIM%u\n", timer);

  switch (timer)
    {
#ifdef CONFIG_ESP32S3_LEDC_TIM0
      case 0:
        lower = &g_pwm0dev;
        break;
#endif

#ifdef CONFIG_ESP32S3_LEDC_TIM1
      case 1:
        lower = &g_pwm1dev;
        break;
#endif

#ifdef CONFIG_ESP32S3_LEDC_TIM2
      case 2:
        lower = &g_pwm2dev;
        break;
#endif

#ifdef CONFIG_ESP32S3_LEDC_TIM3
      case 3:
        lower = &g_pwm3dev;
        break;
#endif

      default:
        pwmerr("ERROR: No such timer configured %d\n", timer);
        lower = NULL;
        break;
    }

  return (struct pwm_lowerhalf_s *)lower;
}
