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
#  define PWM_NCHANNELS  CONFIG_PWM_NCHANNELS
#else
#  define PWM_NCHANNELS  1
#endif

#define CHANNEL_OFFSET   0x20
#define COMP_OFFSET      0x10
#define FAULT_SEL_OFFSET 0x8
#define CLK_FREQ         BOARD_MCK_FREQUENCY
#define PWM_RES          65535
#define COMP_UNITS_NUM   8

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_pwm_channel_s
{
  uint8_t channel;            /* Number of PWM module */
  gpio_pinset_t pin_h;        /* PWM H output pin */
  gpio_pinset_t pin_l;        /* PWM L output pin */
};

struct sam_pwm_comparison_s
{
  int comp_vals[COMP_UNITS_NUM];
  int event0;
  int event1;
};

struct sam_pwm_fault_s
{
  uint8_t source;                 /* Source of fault input */
  uint8_t polarity;
  gpio_pinset_t gpio_0;           /* GPIO 1 fault input */
  gpio_pinset_t gpio_1;           /* GPIO 2 fault input */
  gpio_pinset_t gpio_2;           /* GPIO 3 fault input */
};

struct sam_pwm_s
{
  const struct pwm_ops_s *ops;    /* PWM operations */
  const struct sam_pwm_channel_s *channels;
  const struct sam_pwm_comparison_s *comparison;
  const struct sam_pwm_fault_s *fault;
  uint8_t channels_num;           /* Number of channels */
  uintptr_t base;                 /* Base address of peripheral register */
};

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
    .pin_h   = GPIO_PWMC0_H0,
#ifdef CONFIG_SAMV7_PWM0_CH0_COMP
    .pin_l   = GPIO_PWMC0_L0,
#else
    .pin_l   = 0,
#endif
  },
#endif
#ifdef CONFIG_SAMV7_PWM0_CH1
  {
    .channel = 1,
    .pin_h   = GPIO_PWMC0_H1,
#ifdef CONFIG_SAMV7_PWM0_CH1_COMP
    .pin_l   = GPIO_PWMC0_L1,
#else
    .pin_l   = 0,
#endif
  },
#endif
#ifdef CONFIG_SAMV7_PWM0_CH2
  {
    .channel = 2,
    .pin_h   = GPIO_PWMC0_H2,
#ifdef CONFIG_SAMV7_PWM0_CH2_COMP
    .pin_l   = GPIO_PWMC0_L2,
#else
    .pin_l   = 0,
#endif
  },
#endif
#ifdef CONFIG_SAMV7_PWM0_CH3
  {
    .channel = 3,
    .pin_h   = GPIO_PWMC0_H3,
#ifdef CONFIG_SAMV7_PWM0_CH3_COMP
    .pin_l   = GPIO_PWMC0_L3,
#else
    .pin_l   = 0,
#endif
  },
#endif
};

static struct sam_pwm_comparison_s g_pwm0_comparison =
{
  .comp_vals =
    {
      CONFIG_SAMV7_PWM0_TRIG0, CONFIG_SAMV7_PWM0_TRIG1,
      CONFIG_SAMV7_PWM0_TRIG2, CONFIG_SAMV7_PWM0_TRIG3,
      CONFIG_SAMV7_PWM0_TRIG4, CONFIG_SAMV7_PWM0_TRIG5,
      CONFIG_SAMV7_PWM0_TRIG6, CONFIG_SAMV7_PWM0_TRIG7
    },
#ifdef CONFIG_SAMV7_PWM0_EVENT0
  .event0    = 1,
#else
  .event0    = 0,
#endif
#ifdef CONFIG_SAMV7_PWM0_EVENT1
  .event1    = 1,
#else
  .event1    = 0,
#endif
};

static struct sam_pwm_fault_s g_pwm0_fault =
{
  .source = PWM0_FAULTS,
  .polarity = PWM0_POL,
  .gpio_0 = GPIO_PWMC0_FI0,
  .gpio_1 = GPIO_PWMC0_FI1,
  .gpio_2 = GPIO_PWMC0_FI2,
};

static struct sam_pwm_s g_pwm0 =
{
  .ops = &g_pwmops,
  .channels = g_pwm0_channels,
  .comparison = &g_pwm0_comparison,
  .fault = &g_pwm0_fault,
  .channels_num = PWM0_NCHANNELS,
  .base = SAM_PWM0_BASE,
};
#endif /* CONFIG_SAMV7_PWM0 */

#ifdef CONFIG_SAMV7_PWM1

static struct sam_pwm_channel_s g_pwm1_channels[] =
{
#ifdef CONFIG_SAMV7_PWM1_CH0
  {
    .channel = 0,
    .pin_h   = GPIO_PWMC1_H0,
#ifdef CONFIG_SAMV7_PWM1_CH0_COMP
    .pin_l   = GPIO_PWMC1_L0,
#else
    .pin_l   = 0,
#endif
  },
#endif
#ifdef CONFIG_SAMV7_PWM1_CH1
  {
    .channel = 1,
    .pin_h   = GPIO_PWMC1_H1,
#ifdef CONFIG_SAMV7_PWM1_CH1_COMP
    .pin_l   = GPIO_PWMC1_L1,
#else
    .pin_l   = 0,
#endif
  },
#endif
#ifdef CONFIG_SAMV7_PWM1_CH2
  {
    .channel = 2,
    .pin_h   = GPIO_PWMC1_H2,
#ifdef CONFIG_SAMV7_PWM1_CH2_COMP
    .pin_l   = GPIO_PWMC1_L2,
#else
    .pin_l   = 0,
#endif
  },
#endif
#ifdef CONFIG_SAMV7_PWM1_CH3
  {
    .channel = 3,
    .pin_h   = GPIO_PWMC1_H3,
#ifdef CONFIG_SAMV7_PWM1_CH3_COMP
    .pin_l   = GPIO_PWMC1_L3,
#else
    .pin_l   = 0,
#endif
  },
#endif
}; /* CONFIG_SAMV7_PWM1 */

static struct sam_pwm_comparison_s g_pwm1_comparison =
{
  .comp_vals =
    {
      CONFIG_SAMV7_PWM1_TRIG0, CONFIG_SAMV7_PWM1_TRIG1,
      CONFIG_SAMV7_PWM1_TRIG2, CONFIG_SAMV7_PWM1_TRIG3,
      CONFIG_SAMV7_PWM1_TRIG4, CONFIG_SAMV7_PWM1_TRIG5,
      CONFIG_SAMV7_PWM1_TRIG6, CONFIG_SAMV7_PWM1_TRIG7
    },
#ifdef CONFIG_SAMV7_PWM0_EVENT0
  .event0    = 1,
#else
  .event0    = 0,
#endif
#ifdef CONFIG_SAMV7_PWM0_EVENT1
  .event1    = 1,
#else
  .event1    = 0,
#endif
};

static struct sam_pwm_fault_s g_pwm1_fault =
{
  .source = PWM1_FAULTS,
  .polarity = PWM1_POL,
  .gpio_0 = GPIO_PWMC1_FI0,
  .gpio_1 = GPIO_PWMC1_FI1,
  .gpio_2 = GPIO_PWMC1_FI2,
};

static struct sam_pwm_s g_pwm1 =
{
  .ops = &g_pwmops,
  .channels = g_pwm1_channels,
  .comparison = &g_pwm1_comparison,
  .fault = &g_pwm1_fault,
  .channels_num = PWM1_NCHANNELS,
  .base = SAM_PWM1_BASE,
};

#endif
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void pwm_putreg(struct sam_pwm_s *priv, uint32_t offset,
                       uint32_t value);
static uint32_t pwm_getreg(struct sam_pwm_s *priv, uint32_t offset);

/* Helper functions */

static void pwm_set_output(struct pwm_lowerhalf_s *dev, uint8_t channel,
                           ub16_t duty);
static void pwm_set_freq(struct pwm_lowerhalf_s *dev, uint8_t channel,
                         uint32_t frequency);
static void pwm_set_comparison(struct pwm_lowerhalf_s *dev);
#ifdef CONFIG_PWM_DEADTIME
static void pwm_set_deadtime(struct pwm_lowerhalf_s *dev, uint8_t channel,
                             ub16_t dead_time_a, ub16_t dead_time_b,
                             ub16_t duty);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void pwm_putreg(struct sam_pwm_s *priv, uint32_t offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static uint32_t pwm_getreg(struct sam_pwm_s *priv, uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: pwm_set_freq
 *
 * Description:
 *   Set timer frequency and change registers value to respect that
 *   frequency.
 *
 * Input Parameters:
 *   dev       - A reference to the lower half PWM driver state structure
 *   channel   - Channel to by updated
 *   frequency - New frequency
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pwm_set_freq(struct pwm_lowerhalf_s *dev, uint8_t channel,
                         uint32_t frequency)
{
  struct sam_pwm_s *priv = (struct sam_pwm_s *)dev;
  uint32_t regval;
  uint32_t newdiv = (CLK_FREQ + (frequency / 2)) / frequency - 1;
  uint32_t prescale = 0;

  while (newdiv > PWM_RES && prescale < 11)
    {
      newdiv = newdiv >> 1;
      prescale++;
    }

  if (newdiv > PWM_RES)
    {
      newdiv = PWM_RES;
    }
  else if (newdiv < 2)
    {
      newdiv = 2;
    }

  regval = pwm_getreg(priv, SAMV7_PWM_CMRX + (channel * CHANNEL_OFFSET));
  regval &= ~CMR_CPRE_MASK;
  regval |= CMR_CPRE_SEL(prescale);

  if (pwm_getreg(priv, SAMV7_PWM_SR) & CHID_SEL(1 << channel))
    {
      pwm_putreg(priv, SAMV7_PWM_CMUPDX + (channel * CHANNEL_OFFSET),
                 regval);

      pwm_putreg(priv, SAMV7_PWM_CPRDUPDX + (channel * CHANNEL_OFFSET),
                 CPRD_CPRD_SEL(newdiv));
    }
  else
    {
      pwm_putreg(priv, SAMV7_PWM_CMRX + (channel * CHANNEL_OFFSET),
                 regval);

      pwm_putreg(priv, SAMV7_PWM_CPRDX + (channel * CHANNEL_OFFSET),
                 CPRD_CPRD_SEL(newdiv));
    }
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
 *   None
 *
 ****************************************************************************/

static void pwm_set_output(struct pwm_lowerhalf_s *dev, uint8_t channel,
                           ub16_t duty)
{
  struct sam_pwm_s *priv = (struct sam_pwm_s *)dev;
  uint16_t period;
  uint16_t width;
  uint16_t regval;

  /* Get the period value */

  period = pwm_getreg(priv, SAMV7_PWM_CPRDX + (channel * CHANNEL_OFFSET));

  /* Compute PWM width (count value to set PWM low) */

  width = b16toi(duty * period + b16HALF);

  /* Update duty cycle */

  if (pwm_getreg(priv, SAMV7_PWM_SR) & CHID_SEL(1 << channel))
    {
      pwm_putreg(priv, SAMV7_PWM_CDTYUPDX + (channel * CHANNEL_OFFSET),
                 width);
    }
  else
    {
      pwm_putreg(priv, SAMV7_PWM_CDTYX + (channel * CHANNEL_OFFSET),
                 width);
    }

  /* Enable the channel */

  regval = CHID_SEL(1 << channel);
  pwm_putreg(priv, SAMV7_PWM_ENA, regval);
}

/****************************************************************************
 * Name: pwm_set_comparison
 *
 * Description:
 *   Set comparison units.
 *
 * Input Parameters:
 *   dev     - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pwm_set_comparison(struct pwm_lowerhalf_s *dev)
{
  struct sam_pwm_s *priv = (struct sam_pwm_s *)dev;
  uint16_t period;
  uint16_t width;
  uint16_t comp_value;
  uint8_t comp_en;
  int i;

  /* Get the period value */

  period = pwm_getreg(priv, SAMV7_PWM_CPRDX);

  /* Compute PWM width (count value to set PWM low) */

  comp_en = 0;

  for (i = 0; i < COMP_UNITS_NUM; i++)
    {
      if (priv->comparison->comp_vals[i] == 0)
        {
          continue;
        }

      comp_value = b16divi(uitoub16(priv->comparison->comp_vals[i]), 100);

      width = b16toi(comp_value * period + b16HALF);

      /* Generate event line */

      if (pwm_getreg(priv, SAMV7_PWM_CMPMX + COMP_OFFSET * i) & CMPM_CEN)
        {
          /* Use update register if comparision unit is used */

          pwm_putreg(priv, SAMV7_PWM_CMPVUPDX + COMP_OFFSET * i, width);
          pwm_putreg(priv, SAMV7_PWM_CMPMUPDX + COMP_OFFSET * i, CMPM_CEN);
        }
      else
        {
          pwm_putreg(priv, SAMV7_PWM_CMPVX + COMP_OFFSET * i, width);
          pwm_putreg(priv, SAMV7_PWM_CMPMX + COMP_OFFSET * i, CMPM_CEN);
        }

      comp_en += 1 << i;
    }

  if (priv->comparison->event0)
    {
      /* Enable output on Event Line 0 */

      pwm_putreg(priv, SAMV7_PWM_ELMR1, comp_en);
    }

  if (priv->comparison->event1)
    {
      /* Enable output on Event Line 1 */

      pwm_putreg(priv, SAMV7_PWM_ELMR2, comp_en);
    }
}

/****************************************************************************
 * Name: pwm_set_deadtime
 *
 * Description:
 *   Set deadtime generator values.
 *
 * Input Parameters:
 *   dev         - A reference to the lower half PWM driver state structure
 *   channel     - Channel to by updated
 *   dead_time_a - dead time value for output A
 *   dead_time_b - dead time value for output B
 *   duty        - channel duty cycle
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PWM_DEADTIME
static void pwm_set_deadtime(struct pwm_lowerhalf_s *dev, uint8_t channel,
                             ub16_t dead_time_a, ub16_t dead_time_b,
                             ub16_t duty)
{
  struct sam_pwm_s *priv = (struct sam_pwm_s *)dev;
  uint16_t period;
  uint16_t width_1;
  uint16_t width_2;
  uint16_t regval;

  /* Get the period value */

  period = pwm_getreg(priv, SAMV7_PWM_CPRDX + (channel * CHANNEL_OFFSET));

  /* Compute channel's duty cycle value. Dead time counter has only 12 bits
   * and not 16 as duty cycle or period counter. Therefore a 12 bits recount
   * is necessary to set the dead time value corresponding to selected
   * frequency. This expects the dead time value selected in the application
   * is moved left by 12 and devided by 100. For example:
   *      dead_time_a = (selected_dead_time_duty << 12) / 100
   * This aproach is the same as with duty cycle setup in the application
   * but with 12 bits.
   *
   * Also note that it might not be possible to get correct delay on lower
   * frequencies since dead time register has only 12 bits.
   */

  width_1 = (dead_time_a * period) >> 12;
  width_2 = (dead_time_b * period) >> 12;

  regval = b16toi(duty * period + b16HALF);

  /* It is required width_1 < (CORD - CDTY) and
   * width_2 < CDTY
   */

  if (width_1 > (period - regval))
    {
      pwmerr("ERROR: Dead Time value DTH has to be < period - duty! " \
             "Setting DTH to 0\n");
      width_1 = 0;
    }

  if (width_2 > regval)
    {
      pwmerr("ERROR: Dead Time value DTL has to be < duty! " \
             "Setting DTL to 0\n");
      width_2 = 0;
    }

  /* Update dead time value */

  if (pwm_getreg(priv, SAMV7_PWM_SR) & CHID_SEL(1 << channel))
    {
      pwm_putreg(priv, SAMV7_PWM_DTUPDX + (channel * CHANNEL_OFFSET),
                 DTUPD_DTHUPD_SEL(width_1) | DTUPD_DTLUPD_SEL(width_2));
    }
  else
    {
      pwm_putreg(priv, SAMV7_PWM_DTX + (channel * CHANNEL_OFFSET),
                 DT_DTH_SEL(width_1) | DT_DTL_SEL(width_2));
    }
}
#endif

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
  struct sam_pwm_s *priv = (struct sam_pwm_s *)dev;
  gpio_pinset_t pin_h = 0;
  gpio_pinset_t pin_l = 0;
  uint8_t channel;
  uint32_t regval;

  /* Unlock User Interface */

  regval = WPCR_WPRCMD_SEL(0) | WPCR_WPRG_MASK | WPCR_WPKEY_SEL(0x50574d);
  pwm_putreg(priv, SAMV7_PWM_WPCR, regval);

  /* Disable all channels */

  regval = CHID_SEL(CHID_MASK);
  pwm_putreg(priv, SAMV7_PWM_DIS, regval);

  for (int i = 0; i < priv->channels_num; i++)
    {
      pin_h = priv->channels[i].pin_h;
      pin_l = priv->channels[i].pin_l;

      if (pin_h != 0)
        {
          sam_configgpio(pin_h);
        }

      if (pin_l != 0)
        {
          sam_configgpio(pin_l);
        }

      channel = priv->channels[i].channel;

      regval = CMR_DPOLI;
#ifdef CONFIG_PWM_DEADTIME
      regval |= CMR_DTE;
#endif

      pwm_putreg(priv, SAMV7_PWM_CMRX + (channel * CHANNEL_OFFSET), regval);

      /* Reset duty cycle register */

      pwm_putreg(priv, SAMV7_PWM_CDTYX + (channel * CHANNEL_OFFSET), 0);

      /* Reset period register */

      pwm_putreg(priv, SAMV7_PWM_CPRDX + (channel * CHANNEL_OFFSET), 0);

      /* Reset Dead Time Register */

      pwm_putreg(priv, SAMV7_PWM_DTX + (channel * CHANNEL_OFFSET), 0);

      /* Enable fault protection if configured. The protection has to
       * be enabled for every configured channel separately.
       */

      regval = pwm_getreg(priv, SAMV7_PWM_FPE);
      regval |= priv->fault->source << (FAULT_SEL_OFFSET * channel);
      pwm_putreg(priv, SAMV7_PWM_FPE, regval);
    }

  /* Configure fault GPIOs if used */

  if (priv->fault->source & 1)
    {
      sam_configgpio(priv->fault->gpio_0);
    }

  if (priv->fault->source & (1 << 1))
    {
      sam_configgpio(priv->fault->gpio_1);
    }

  if (priv->fault->source & (1 << 2))
    {
      sam_configgpio(priv->fault->gpio_2);
    }

  /* Set fault polarity. This has to be 1 for peripheral fault
   * generation (from ADC for example), GPIO generated fault
   * is set via configuration options.
   */

  regval = FMR_FPOL_SEL(priv->fault->polarity);
  pwm_putreg(priv, SAMV7_PWM_FMR, regval);

  /* Force both outputs to 0 if fault occurs */

  pwm_putreg(priv, SAMV7_PWM_FPV1, 0);
  pwm_putreg(priv, SAMV7_PWM_FPV2, 0);

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

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct sam_pwm_s *priv = (struct sam_pwm_s *)dev;
  uint32_t regval;

  /* Disable all channels and interrupts */

  regval = CHID_SEL(CHID_MASK);
  pwm_putreg(priv, SAMV7_PWM_DIS, regval);

  regval = IR1_CHID_SEL(CHID_MASK);
  pwm_putreg(priv, SAMV7_PWM_IDR1, regval);

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

static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct sam_pwm_s *priv = (struct sam_pwm_s *)dev;
#ifdef CONFIG_PWM_OVERWRITE
  uint32_t regval;
#endif

#ifdef CONFIG_PWM_MULTICHAN
      for (int i = 0; i < PWM_NCHANNELS; i++)
        {
          int8_t index = info->channels[i].channel;

          /* Break the loop if all following channels are not configured */

          if (index == -1)
            {
              break;
            }

          /* Configure the module freq only if is set to be used */

          if (index > 0 && (index - 1) < priv->channels_num)
            {
              /* Set the frequency and enable PWM output for each channel */

              pwm_set_freq(dev, priv->channels[index - 1].channel,
                           info->frequency);
#ifdef CONFIG_PWM_DEADTIME
              pwm_set_deadtime(dev, priv->channels[index - 1].channel,
                               info->channels[i].dead_time_a,
                               info->channels[i].dead_time_b,
                               info->channels[i].duty);
#endif
              pwm_set_output(dev, priv->channels[index - 1].channel,
                             info->channels[i].duty);
#ifdef CONFIG_PWM_OVERWRITE
              if (info->channels[i].ch_outp_ovrwr)
                {
                  regval = pwm_getreg(priv, SAMV7_PWM_OOV);
                  regval &= ~(info->channels[i].ch_outp_ovrwr_val
                      << priv->channels[i].channel);
                  pwm_putreg(priv, SAMV7_PWM_OOV, regval);

                  regval = (1 << priv->channels[i].channel);
                  pwm_putreg(priv, SAMV7_PWM_OSS, regval);
                }
              else
                {
                  /* Release overwrite of channel */

                  regval = (1 << priv->channels[i].channel);
                  pwm_putreg(priv, SAMV7_PWM_OSC, regval);
                }
#endif
            }
        }
#else
      /* Set the frequency and enable PWM output just for first channel */

      pwm_set_freq(dev, priv->channels[0].channel, info->frequency);
#ifdef CONFIG_PWM_DEADTIME
      pwm_set_deadtime(dev, priv->channels[index - 1].channel,
                       info->dead_time_a, info->dead_time_b);
#endif
      pwm_set_output(dev, priv->channels[0].channel, info->duty);
#endif

      pwm_set_comparison(dev);

  return OK;
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

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct sam_pwm_s *priv = (struct sam_pwm_s *)dev;
  uint32_t regval;

#ifdef CONFIG_PWM_MULTICHAN
  for (int i = 0; i < priv->channels_num; i++)
    {
      regval = CHID_SEL(1 << priv->channels[i].channel);
      pwm_putreg(priv, SAMV7_PWM_DIS, regval);
    }

#else
  regval = CHID_SEL(1 << priv->channels[0].channel);
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

static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
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

struct pwm_lowerhalf_s *sam_pwminitialize(int pwm)
{
  struct sam_pwm_s *priv;

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

  return (struct pwm_lowerhalf_s *)priv;
}
#endif /* CONFIG_SAMV7_PWM */
