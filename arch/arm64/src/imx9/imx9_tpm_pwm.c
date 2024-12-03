/****************************************************************************
 * arch/arm64/src/imx9/imx9_tpm_pwm.c
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

#include "arm64_arch.h"
#include "imx9_tpm_pwm.h"
#include "imx9_iomuxc.h"
#include "imx9_ccm.h"

#include "hardware/imx9_iomuxc.h"
#include "hardware/imx9_pinmux.h"
#include "hardware/imx9_ccm.h"

#ifdef CONFIG_IMX9_TPM_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CHMUX(priv, x) ((((priv)->chmux) >> (x)) & 0xff)

/* This is a temporary shortcut to configure TPM1 and TPM3
 * frequencies correctly; they don't have an own root clock
 *
 * TODO: Determine the frequencies in a more proper way
 */

#define AON_CLK_FREQ       133333333
#define WAKEUP_CLK_FREQ    133333333
#define OSC_24_CLK_FREQ    24000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one PWM timer */

struct imx9_pwmtimer_s
{
  const struct pwm_ops_s *ops; /* PWM operations */
  const tpm_pwm_id_t id;       /* PWM_TPM1...PWM_TPM6 */
  const uintptr_t base;        /* The base address of the TPM */
  const int n_channels;        /* Number of channels used for TPM block */
  const uint32_t chmux;        /* Additional muxing of TPM outputs */
  const int clk;               /* Input clock frequency for the TPM */
  uint32_t period;
  unsigned frequency;          /* Current frequency setting */
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
#ifdef CONFIG_IMX9_TPM1_PWM
  {
    .ops         = &g_pwmops,
    .id          = PWM_TPM1,
    .base        = IMX9_TPM1_BASE,
    .n_channels  = CONFIG_IMX9_TPM1_PWM_NCHANNELS,
    .chmux       = CONFIG_IMX9_TPM1_PWM_CHMUX,
    .clk         = AON_CLK_FREQ,
  },
#endif

#ifdef CONFIG_IMX9_TPM2_PWM
  {
    .ops         = &g_pwmops,
    .id          = PWM_TPM2,
    .base        = IMX9_TPM2_BASE,
    .n_channels  = CONFIG_IMX9_TPM2_PWM_NCHANNELS,
    .chmux       = CONFIG_IMX9_TPM2_PWM_CHMUX,
    .clk         = OSC_24_CLK_FREQ,
  },
#endif

#ifdef CONFIG_IMX9_TPM3_PWM
  {
    .ops         = &g_pwmops,
    .id          = PWM_TPM3,
    .base        = IMX9_TPM3_BASE,
    .n_channels  = CONFIG_IMX9_TPM3_PWM_NCHANNELS,
    .chmux       = CONFIG_IMX9_TPM3_PWM_CHMUX,
    .clk         = WAKEUP_CLK_FREQ,
  },
#endif

#ifdef CONFIG_IMX9_TPM4_PWM
  {
    .ops         = &g_pwmops,
    .id          = PWM_TPM4,
    .base        = IMX9_TPM4_BASE,
    .n_channels  = CONFIG_IMX9_TPM4_PWM_NCHANNELS,
    .chmux       = CONFIG_IMX9_TPM4_PWM_CHMUX,
    .clk         = OSC_24_CLK_FREQ,
  },
#endif

#ifdef CONFIG_IMX9_TPM5_PWM
  {
    .ops         = &g_pwmops,
    .id          = PWM_TPM5,
    .base        = IMX9_TPM5_BASE,
    .n_channels  = CONFIG_IMX9_TPM5_PWM_NCHANNELS,
    .chmux       = CONFIG_IMX9_TPM5_PWM_CHMUX,
    .clk         = OSC_24_CLK_FREQ,
  },
#endif

#ifdef CONFIG_IMX9_TPM6_PWM
  {
    .ops         = &g_pwmops,
    .id          = PWM_TPM6,
    .base        = IMX9_TPM6_BASE,
    .n_channels  = CONFIG_IMX9_TPM6_PWM_NCHANNELS,
    .chmux       = CONFIG_IMX9_TPM6_PWM_CHMUX,
    .clk         = OSC_24_CLK_FREQ,
  },
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_getreg
 *
 * Description:
 *   Read the value of an PWM timer register.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static inline uint32_t pwm_getreg(struct imx9_pwmtimer_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: pwm_putreg
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

static inline void pwm_putreg(struct imx9_pwmtimer_s *priv, int offset,
                              uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: tpm_mux
 *
 * Description:
 *   Mux the tpm output pins to pads. The macros TPMn_PWMx_MUX
 *   need to be defined in the board.h file
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tpm_mux(void)
{
#ifdef CONFIG_IMX9_TPM1_PWM
#  ifdef TPM1_PWM0_MUX
  imx9_iomux_configure(TPM1_PWM0_MUX);
#  endif

#  ifdef TPM1_PWM1_MUX
  imx9_iomux_configure(TPM1_PWM1_MUX);
#  endif

#  ifdef TPM1_PWM2_MUX
  imx9_iomux_configure(TPM1_PWM2_MUX);
#  endif

#  ifdef TPM1_PWM3_MUX
  imx9_iomux_configure(TPM1_PWM3_MUX);
#  endif
#endif

#ifdef CONFIG_IMX9_TPM2_PWM
#  ifdef TPM2_PWM0_MUX
  imx9_iomux_configure(TPM2_PWM0_MUX);
#  endif

#  ifdef TPM2_PWM1_MUX
  imx9_iomux_configure(TPM2_PWM1_MUX);
#  endif

#  ifdef TPM2_PWM2_MUX
  imx9_iomux_configure(TPM2_PWM2_MUX);
#  endif

#  ifdef TPM2_PWM3_MUX
  imx9_iomux_configure(TPM2_PWM3_MUX);
#  endif
#endif

#ifdef CONFIG_IMX9_TPM3_PWM
#  ifdef TPM3_PWM0_MUX
  imx9_iomux_configure(TPM3_PWM0_MUX);
#  endif

#  ifdef TPM3_PWM1_MUX
  imx9_iomux_configure(TPM3_PWM1_MUX);
#  endif

#  ifdef TPM3_PWM2_MUX
  imx9_iomux_configure(TPM3_PWM2_MUX);
#  endif

#  ifdef TPM3_PWM3_MUX
  imx9_iomux_configure(TPM3_PWM3_MUX);
#  endif
#endif

#ifdef CONFIG_IMX9_TPM4_PWM
#  ifdef TPM4_PWM0_MUX
  imx9_iomux_configure(TPM4_PWM0_MUX);
#  endif

#  ifdef TPM4_PWM1_MUX
  imx9_iomux_configure(TPM4_PWM1_MUX);
#  endif

#  ifdef TPM4_PWM2_MUX
  imx9_iomux_configure(TPM4_PWM2_MUX);
#  endif

#  ifdef TPM4_PWM3_MUX
  imx9_iomux_configure(TPM4_PWM3_MUX);
#  endif
#endif

#ifdef CONFIG_IMX9_TPM5_PWM
#  ifdef TPM5_PWM0_MUX
  imx9_iomux_configure(TPM5_PWM0_MUX);
#  endif

#  ifdef TPM5_PWM1_MUX
  imx9_iomux_configure(TPM5_PWM1_MUX);
#  endif

#  ifdef TPM5_PWM2_MUX
  imx9_iomux_configure(TPM5_PWM2_MUX);
#  endif

#  ifdef TPM5_PWM3_MUX
  imx9_iomux_configure(TPM5_PWM3_MUX);
#  endif
#endif

#ifdef CONFIG_IMX9_TPM6_PWM
#  ifdef TPM6_PWM0_MUX
  imx9_iomux_configure(TPM6_PWM0_MUX);
#  endif

#  ifdef TPM6_PWM1_MUX
  imx9_iomux_configure(TPM6_PWM1_MUX);
#  endif

#  ifdef TPM6_PWM2_MUX
  imx9_iomux_configure(TPM6_PWM2_MUX);
#  endif

#  ifdef TPM6_PWM3_MUX
  imx9_iomux_configure(TPM6_PWM3_MUX);
#  endif
#endif
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
  int i;

  if (freq != priv->frequency)
    {
      /* Start PWM on all channels if it was previously stopped */

      if (freq > 0 && priv->frequency == 0)
        {
          /* Set the EPWM mode for all the configured channels:
           *
           * High-true pulses (clear output on counter match, set output
           * on counter reload, clear output when counter first enabled
           * or paused)
           * SC[CPWMS] = 0, MSnB:MSnA = 1:0 ELSnB:ELSnA = 0
           */

          for (i = 0; i < priv->n_channels; i++)
            {
              pwm_putreg(priv, IMX9_TPM_CXSC_OFFSET(CHMUX(priv, i)),
                         TPM_CXSC_MSB_MASK);
            }
        }

      priv->period = freq > 0 ? priv->clk / freq : 0;

      pwminfo("PWM%d frequency: %" PRIu32" period: %" PRIu32 "\n", priv->id,
              freq, priv->period);

      /* Set the period */

      pwm_putreg(priv, IMX9_TPM_MOD_OFFSET, priv->period);

      priv->frequency = freq;
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_update_duty
 *
 * Description:
 *   Change the channel duty cycle.
 *
 * Input Parameters:
 *   priv    - A reference to the lower half PWM driver state structure
 *   period  - PWM pulse width in timer ticks
 *   channel - Channel to by updated
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_update_duty(struct imx9_pwmtimer_s *priv, int pwm_ch,
                           ub16_t duty16)
{
  uint64_t duty = duty16 & 0xffffffff;
  uint32_t edge = (duty * priv->period + 0x8000) >> 16;
  int timer = pwm_ch - 1;

  if (pwm_ch == 0 || timer > priv->n_channels)
    {
      pwmerr("ERROR: PWM%d has no such channel: %d\n", priv->id, timer);
      return -EINVAL;
    }

  pwminfo("PWM%d channel %d, p: %d e: %" PRIu32 "\n", priv->id,
          CHMUX(priv, timer), priv->period, edge);

  pwm_putreg(priv, IMX9_TPM_CXV_OFFSET(CHMUX(priv, timer)), edge);

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

  if (pwm_update_frequency(priv, info->frequency) == OK)
    {
      /* Handle channel specific setup */

      for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
        {
          if (ret != OK || info->channels[i].channel == -1)
            {
              break;
            }

          pwm_update_duty(priv, info->channels[i].channel,
                          info->channels[i].duty);
        }
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

  /* Disable the channels */

  for (i = 0; i < priv->n_channels; i++)
    {
      pwm_putreg(priv, IMX9_TPM_CXSC_OFFSET(CHMUX(priv, i)), 0);
    }

  /* Set frequency to 0 */

  pwm_update_frequency(priv, 0);

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
 * Name: imx9_tpm_pwm_init
 *
 * Description:
 *   Initialize tpm blocks to generate EPWM.
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

struct pwm_lowerhalf_s *imx9_tpm_pwm_init(tpm_pwm_id_t pwmid)
{
  struct imx9_pwmtimer_s *lower = NULL;
  int i;

  for (i = 0; i <  sizeof(g_pwmdev) / sizeof(g_pwmdev[0]); i++)
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

      tpm_mux();

      /* Reset TPM */

      pwm_putreg(lower, IMX9_TPM_GLOBAL_OFFSET, TPM_GLOBAL_RST_MASK);
      pwm_putreg(lower, IMX9_TPM_GLOBAL_OFFSET, 0);
      while (pwm_getreg(lower, IMX9_TPM_GLOBAL_OFFSET) != 0);

      /* TPM 1 is always clocked by AON bus and TPM3 by WAKEUP bus */

      if (pwmid != PWM_TPM1 && pwmid != PWM_TPM3)
        {
          /* 24 MHz source clock */

          imx9_ccm_configure_root_clock(CCM_CR_TPM1 + pwmid, OSC_24M, 1);

          /* Enable peripheral clock */

          imx9_ccm_gate_on(CCM_LPCG_TPM1 + pwmid, true);
        }

      /* Set status and control:
       * CMOD = 1 (increment on every clock)
       * PS = 0   (clock divider 1)
       */

      pwm_putreg(lower, IMX9_TPM_SC_OFFSET, 1 << TPM_SC_CMOD_SHIFT);

      pwminfo("PWM%d at %" PRIxPTR " configured\n", pwmid, lower->base);
    }
  else
    {
      pwmerr("ERROR: No such timer configured %d\n", pwmid);
    }

  return (struct pwm_lowerhalf_s *)lower;
}

#endif
