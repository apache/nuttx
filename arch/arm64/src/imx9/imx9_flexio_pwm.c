/****************************************************************************
 * arch/arm64/src/imx9/imx9_flexio_pwm.c
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

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <time.h>
#include <inttypes.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/timers/pwm.h>

#include <arch/board/board.h>

#include "imx9_flexio_pwm.h"
#include "arm64_arch.h"
#include "imx9_ccm.h"
#include "imx9_iomuxc.h"
#include "hardware/imx9_ccm.h"
#include "hardware/imx9_pinmux.h"
#include "hardware/imx9_flexio.h"

#ifdef CONFIG_IMX9_FLEXIO_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PIN_FOR_TIMER(priv, timer) ((uint8_t)(priv->pins >> (timer * 8)))

#ifdef CONFIG_IMX9_FLEXIO1_PWM
#  if (CONFIG_PWM_NCHANNELS < CONFIG_IMX9_FLEXIO1_PWM_NCHANNELS)
#    error CONFIG_PWM_NCHANNELS < CONFIG_IMX9_FLEXIO1_PWM_NCHANNELS
#  endif
#endif

#ifdef CONFIG_IMX9_FLEXIO2_PWM
#  if (CONFIG_PWM_NCHANNELS < CONFIG_IMX9_FLEXIO2_PWM_NCHANNELS)
#    error CONFIG_PWM_NCHANNELS < CONFIG_IMX9_FLEXIO2_PWM_NCHANNELS
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one PWM timer */

struct imx9_pwmtimer_s
{
  const struct pwm_ops_s *ops; /* PWM operations */
  const flexio_pwm_id_t id;    /* PWM_FLEXIO1 or PWM_FLEXIO2 */
  const int nchannels;         /* Number of channels used */
  const uintptr_t base;        /* The base address of the FLEXIO */
  const uint64_t pins;         /* Mapping of timer outputs to flexio outputs */
  const int int_trigger;       /* Uses flex-io internal timer for frequency */
  int trigger_ch;              /* Trigger channel */
  unsigned frequency;          /* Current frequency setting */
  int period;                  /* PWM period in ticks of functional clock */
};

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
  .setup       = pwm_setup,
  .shutdown    = pwm_shutdown,
  .start       = pwm_start,
  .stop        = pwm_stop,
  .ioctl       = pwm_ioctl,
};

static struct imx9_pwmtimer_s g_pwmdev[] =
{
#ifdef CONFIG_IMX9_FLEXIO1_PWM
  {
    .ops         = &g_pwmops,
    .id          = PWM_FLEXIO1,
    .nchannels   = CONFIG_IMX9_FLEXIO1_PWM_NCHANNELS,
    .base        = IMX9_FLEXIO1_BASE,
    .pins        = CONFIG_IMX9_FLEXIO1_PWM_CHANNEL_PINS,
    .int_trigger = 1,
  },
#endif

#ifdef CONFIG_IMX9_FLEXIO2_PWM
  {
    .ops         = &g_pwmops,
    .id          = PWM_FLEXIO2,
    .nchannels   = CONFIG_IMX9_FLEXIO2_PWM_NCHANNELS,
    .base        = IMX9_FLEXIO2_BASE,
    .pins        = CONFIG_IMX9_FLEXIO2_PWM_CHANNEL_PINS,
    .int_trigger = 1,
  },
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: flexio_getreg
 *
 * Description:
 *   Read the value of a flex-io register.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static inline uint32_t flexio_getreg(struct imx9_pwmtimer_s *priv,
                                     int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: flexio_putreg
 *
 * Description:
 *   Read the value of an PWM timer register.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void flexio_putreg(struct imx9_pwmtimer_s *priv, int offset,
                                 uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: flexio_mux
 *
 * Description:
 *   Mux the flex-io output pins to pads. The macros FLEXIOn_PWMx_MUX
 *   need to be defined in the board.h file
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void flexio_mux(void)
{
#ifdef CONFIG_IMX9_FLEXIO1_PWM

#  ifdef FLEXIO1_PWM0_MUX
  imx9_iomux_configure(FLEXIO1_PWM0_MUX);
#  endif

#  ifdef FLEXIO1_PWM1_MUX
  imx9_iomux_configure(FLEXIO1_PWM1_MUX);
#  endif

#  ifdef FLEXIO1_PWM2_MUX
  imx9_iomux_configure(FLEXIO1_PWM2_MUX);
#  endif

#  ifdef FLEXIO1_PWM3_MUX
  imx9_iomux_configure(FLEXIO1_PWM3_MUX);
#  endif

#  ifdef FLEXIO1_PWM4_MUX
  imx9_iomux_configure(FLEXIO1_PWM4_MUX);
#  endif

#  ifdef FLEXIO1_PWM5_MUX
  imx9_iomux_configure(FLEXIO1_PWM5_MUX);
#  endif

#  ifdef FLEXIO1_PWM6_MUX
  imx9_iomux_configure(FLEXIO1_PWM6_MUX);
#  endif

#  ifdef FLEXIO1_PWM7_MUX
  imx9_iomux_configure(FLEXIO1_PWM7_MUX);
#  endif

#endif

#ifdef CONFIG_IMX9_FLEXIO2_PWM

#  ifdef FLEXIO2_PWM0_MUX
  imx9_iomux_configure(FLEXIO2_PWM0_MUX);
#  endif

#  ifdef FLEXIO2_PWM1_MUX
  imx9_iomux_configure(FLEXIO2_PWM1_MUX);
#  endif

#  ifdef FLEXIO2_PWM2_MUX
  imx9_iomux_configure(FLEXIO2_PWM2_MUX);
#  endif

#  ifdef FLEXIO2_PWM3_MUX
  imx9_iomux_configure(FLEXIO2_PWM3_MUX);
#  endif

#  ifdef FLEXIO2_PWM4_MUX
  imx9_iomux_configure(FLEXIO2_PWM4_MUX);
#  endif

#  ifdef FLEXIO2_PWM5_MUX
  imx9_iomux_configure(FLEXIO2_PWM5_MUX);
#  endif

#  ifdef FLEXIO2_PWM6_MUX
  imx9_iomux_configure(FLEXIO2_PWM6_MUX);
#  endif

#  ifdef FLEXIO2_PWM7_MUX
  imx9_iomux_configure(FLEXIO2_PWM7_MUX);
#  endif

#endif
}

/****************************************************************************
 * Name: pwm_init_trigger_timer
 *
 * Description:
 *   Initialize the timer trigger, generating the PWM frequency
 *
 * Input Parameters:
 *   priv    - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   OK on success, ERROR if the timer initialization fails
 *
 ****************************************************************************/

static int pwm_init_trigger_timer(struct imx9_pwmtimer_s *priv)
{
  uint32_t reg;
  int num_timers;

  /* For now, use the last available flexio timer to produce internal
   * trigger. This can be later expanded to use external trigger from
   * LPIT timer, if one more PWM channel is required
   */

  /* Get parameter register and number of supported timers */

  reg = flexio_getreg(priv, IMX9_FLEXIO_PARAM_OFFSET);
  num_timers = (reg & FLEXIO_PARAM_TIMER_MASK) >> FLEXIO_PARAM_TIMER_SHIFT;

  num_timers--;
  if (num_timers < priv->nchannels)
    {
      pwmerr("PWM%d max channels %d\n", priv->id, num_timers);
      return ERROR;
    }

  priv->trigger_ch = num_timers;

  return OK;
}

/****************************************************************************
 * Name: pwm_select_func_clock
 *
 * Description:
 *   Select best suitable functional clock for the flexio
 *
 * Input Parameters:
 *   priv    - A reference to the lower half PWM driver state structure
 *   freq    - The requested PWM frequency for this flexio block
 *
 * Returned Value:
 *   Zero on success, negated error value on failure
 *
 ****************************************************************************/

static int pwm_select_func_clock(struct imx9_pwmtimer_s *priv, int freq)
{
  const int max_div = 24000000 / 65536 + 1; /* for 1 Hz */
  int div;
  uint32_t period;

  if (freq == 0)
    {
      priv->period = 0;
      return 0;
    }

  /* Use the 24MHz OSC clock, and find the best divider to get as much
   * resolution as possible using 16 bit timer
   */

  for (div = 1; div < max_div; div++)
    {
      period = 24000000 / div / freq;
      if (period < 65536)
        {
          priv->period = period;
          break;
        }
    }

  imx9_ccm_configure_root_clock(CCM_CR_FLEXIO1 + priv->id, OSC_24M, div);

  /* Enable peripheral clock */

  imx9_ccm_gate_on(CCM_LPCG_FLEXIO1 + priv->id , true);

  return 0;
}

/****************************************************************************
 * Name: pwm_update_frequency
 *
 * Description:
 *   Initialize the timer trigger, generating the PWM freuency
 *
 * Input Parameters:
 *   priv    - A reference to the lower half PWM driver state structure
 *   freq    - The requested PWM frequency for this flexio block
 *
 * Returned Value:
 *   Zero on success, negated error value on failure
 *
 ****************************************************************************/

static int pwm_update_frequency(struct imx9_pwmtimer_s *priv, int freq)
{
  int ret = pwm_select_func_clock(priv, freq);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure the timer to produce internal trigger. The following
   * setting produces 50/50 pulse where duty cycle is defined by
   * TIMCMP:
   *
   * TIMCFG:
   *   TIMOUT = 0    Timer output is logic one when enabled and not
   *                 affected by timer reset
   *   TIMDEC = 0    Decrement counter on FLEXIO clock
   *   TIMRST = 0x0  Timer never reset
   *   TIMDIS = 0x0  Timer never disabled
   *   TIMENA = 0x0  Timer always enabled
   * TIMCTL:
   *   TIMOD  = 0x3  Single 16-bit counter
   *   PINCFG = 0x0  Output pin disabled
   * TIMCMP: frequency / 2
   */

  flexio_putreg(priv, IMX9_FLEXIO_TIMCMP_OFFSET(priv->trigger_ch),
                priv->period / 2);

  /* Enable / disable timer */

  flexio_putreg(priv, IMX9_FLEXIO_TIMCTL_OFFSET(priv->trigger_ch),
                freq > 0 ? FLEXIO_TIMCTL_TIMOD(0x3) :
                FLEXIO_TIMCTL_TIMOD(0x0));

  return ret;
}

/****************************************************************************
 * Name: pwm_update_duty
 *
 * Description:
 *   Change the channel duty cycle.
 *
 * Input Parameters:
 *   priv    - A reference to the lower half PWM driver state structure
 *   channel - Channel to by updated
 *   duty    - New duty cycle as fraction of 65536
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_update_duty(struct imx9_pwmtimer_s *priv, int pwm_ch,
                           ub16_t duty16)
{
  uint32_t edge = ub16toi(duty16 * priv->period + b16HALF);
  int timer = pwm_ch - 1; /* map pwm ch 1 to timer 0 etc.. */
  uint32_t regval;

  if (pwm_ch == 0 || pwm_ch > priv->nchannels)
    {
      pwmerr("ERROR: PWM%d has no such channel: %u\n", priv->id, pwm_ch);
      return -EINVAL;
    }

  /* Now configure the flexio timers in 16-bit counter mode */

  /* Timers 0-6:
   * TIMCFG:
   *  TIMOUT = 0  Timer output is 1 when enabled and not affected by reset
   *  TIMDEC = 0    Decrement counter on FLEXIO clock
   *  TIMRST = 0x0  Timer never reset
   *  TIMDIS = 0x2  Timer disabled on counter 0
   *  TIMENA = 0x6  Timer enabled on Trigger rising edge
   *  TIMCTL:
   *  TIMOD = 0x3  single 16-bit counter
   *  TRGSEL = 4 * trg_ch + 3  timer "trg_ch" trigger output
   *  TRGSRC = 1  internal trigger
   *  PINCFG = 0x3 Timer pin output
   *  PINSEL = timer number + mux conf
   *  TIMCMP: duty cycle
   */

  /* If this is the first time configuring the PWMs, configure the
   * timer fully, otherwise just update the duty cycle
   */

  flexio_putreg(priv, IMX9_FLEXIO_TIMCMP_OFFSET(timer), edge);

  if (priv->frequency == 0)
    {
      flexio_putreg(priv, IMX9_FLEXIO_TIMCFG_OFFSET(timer),
                    FLEXIO_TIMCFG_TIMDIS(0x2) |
                    FLEXIO_TIMCFG_TIMENA(0x6));

      /* When initially configuring PINCFG=11b, FLEXIO may briefly drive the
       * pin low. To avoid this, configure PINCFG=10b along with the rest of
       * the control register and then perform a subsequent write to set
       * PINCFG=11b
       */

      regval = (FLEXIO_TIMCTL_TIMOD(0x3) |
                FLEXIO_TIMCTL_TRGSEL(4 * priv->trigger_ch + 3) |
                FLEXIO_TIMCTL_TRGSRC(priv->int_trigger) |
                FLEXIO_TIMCTL_PINSEL(PIN_FOR_TIMER(priv, timer)));

      flexio_putreg(priv, IMX9_FLEXIO_TIMCTL_OFFSET(timer),
                    regval | FLEXIO_TIMCTL_PINCFG(0x2));

      flexio_putreg(priv, IMX9_FLEXIO_TIMCTL_OFFSET(timer),
                    regval | FLEXIO_TIMCTL_PINCFG(0x3));
    }

  pwminfo("PWM%d channel %d, p: %d e: %" PRIu32 "\n", priv->id, pwm_ch,
          priv->period, edge);

  return 0;
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
 * Assumptions:
 *
 ****************************************************************************/

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
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
  /* Make sure that the output has been stopped */

  pwm_stop(dev);

  return OK;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct imx9_pwmtimer_s *priv = (struct imx9_pwmtimer_s *)dev;
  int ret = OK;
  int i;

  if (priv == NULL || info == NULL || info->frequency == 0)
    {
      return -EINVAL;
    }

  /* Set the frequency if not changed */

  if (info->frequency != priv->frequency)
    {
      ret = pwm_update_frequency(priv, info->frequency);
    }

  /* Handle channel specific setup */

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      if (ret != OK || info->channels[i].channel == -1)
        {
          break;
        }

      ret = pwm_update_duty(priv, info->channels[i].channel,
                            info->channels[i].duty);
    }

  if (ret == OK)
    {
      priv->frequency = info->frequency;
    }

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
 *   This function is called to stop the pulsed output at anytime.
 *
 ****************************************************************************/

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct imx9_pwmtimer_s *priv = (struct imx9_pwmtimer_s *)dev;
  int i;

  pwminfo("PWM%d stop\n", priv->id);

  /* Check that timer is valid */

  if (priv == NULL)
    {
      return -EINVAL;
    }

  /* Disable all the channels */

  for (i = 0; i < priv->nchannels; i++)
    {
      flexio_putreg(priv, IMX9_FLEXIO_TIMCTL_OFFSET(i),
                    FLEXIO_TIMCTL_PINCFG(0x2));
      flexio_putreg(priv, IMX9_FLEXIO_TIMCTL_OFFSET(i), 0);
    }

  /* Setting frequency to zero disables trigger clock */

  return pwm_update_frequency(priv, 0);
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
 * Name: imx9_flexio_pwm_init
 *
 * Description:
 *   Initialize flexio blocks to generate EPWM.
 *
 * Input Parameters:
 *   pwmid - A number identifying the pwm block. The number of valid
 *           IDs varies depending on the configuration.
 *
 * Returned Value:
 *   On success, a pointer to the lower half PWM driver is
 *   returned. NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *imx9_flexio_pwm_init(flexio_pwm_id_t pwmid)
{
  struct imx9_pwmtimer_s *lower = NULL;
  int i;

  for (i = 0; i < sizeof(g_pwmdev) / sizeof(struct imx9_pwmtimer_s); i++)
    {
      if (pwmid == g_pwmdev[i].id)
        {
          lower = &g_pwmdev[i];
          break;
        }
    }

  if (lower)
    {
      /* IO mux */

      flexio_mux();

      /* Reset FlexIO */

      flexio_putreg(lower, IMX9_FLEXIO_CTRL_OFFSET, FLEXIO_CTRL_SWRST(1));

      /* Enable FlexIO and de-assert reset */

      flexio_putreg(lower, IMX9_FLEXIO_CTRL_OFFSET, FLEXIO_CTRL_FLEXEN(1));

      /* Make sure that FlexIO is enabled and reset is cleared */

      while (flexio_getreg(lower, IMX9_FLEXIO_CTRL_OFFSET) !=
             FLEXIO_CTRL_FLEXEN_MASK);

      /* Initialize the trigger timer used for PWM period generation */

      if (pwm_init_trigger_timer(lower) != OK)
        {
          /* Disable FlexIO */

          flexio_putreg(lower, IMX9_FLEXIO_CTRL_OFFSET, 0);

          return NULL;
        }

      pwminfo("PWM%d at 0x%" PRIxPTR " configured\n", pwmid, lower->base);
    }
  else
    {
      pwmerr("ERROR: No such timer configured %d\n", pwmid);
    }

  return (struct pwm_lowerhalf_s *)lower;
}

#endif
