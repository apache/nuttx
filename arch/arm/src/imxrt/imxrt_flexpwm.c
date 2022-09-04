/****************************************************************************
 * arch/arm/src/imxrt/imxrt_flexpwm.c
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

#include <nuttx/timers/pwm.h>

#include "arm_internal.h"
#include "chip.h"
#include "imxrt_config.h"
#include "imxrt_flexpwm.h"
#include "imxrt_periphclks.h"
#include "imxrt_xbar.h"
#include "hardware/imxrt_flexpwm.h"
#include "hardware/imxrt_pinmux.h"
#include "hardware/imxrt_ccm.h"

#include <arch/board/board.h>

#include <sys/time.h>

#ifdef CONFIG_IMXRT_FLEXPWM

#ifdef CONFIG_PWM_NCHANNELS
# define PWM_NCHANNELS CONFIG_PWM_NCHANNELS
#else
# define PWM_NCHANNELS 1
#endif

#define MODULE_OFFSET 0x60
#define CLK_FREQ  150000000
#define PWM_RES 65535

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_flexpwm_out_s
{
  bool used;
  uint32_t pin;                   /* Output pin */
};

struct imxrt_flexpwm_module_s
{
  uint8_t module;                   /* Number of PWM module */
  bool used;                        /* True if the module is used */
  bool trig_en;
  struct imxrt_flexpwm_out_s out_a; /* PWM output */
  struct imxrt_flexpwm_out_s out_b; /* PWM output */
  bool complementary;               /* True if outputs are complementary */
  uint32_t irq;                     /* Combined interrupt */
  uint16_t ext_sync;
  int16_t sync_src;
};

struct imxrt_flexpwm_s
{
  const struct pwm_ops_s *ops;    /* PWM operations */
  struct imxrt_flexpwm_module_s *modules;
  uint8_t modules_num;            /* Number of modules */
  uint32_t frequency;             /* PWM frequency */
  uint32_t base;                  /* Base address of peripheral register */
};

/* PWM driver methods */

static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);
static int pwm_stop(struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg);

/* Helper functions */

static int pwm_set_output(struct pwm_lowerhalf_s *dev, uint8_t channel,
                          ub16_t duty);
static int pwm_change_freq(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info,
                           uint8_t channel);

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

#ifdef CONFIG_IMXRT_FLEXPWM1

static struct imxrt_flexpwm_module_s g_pwm1_modules[] =
{
  /* FlexPWM1 has 4 submodules with 2 outputs for each */

#ifdef CONFIG_IMXRT_FLEXPWM1_MOD1
  {
    .module = 1,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM1_MOD1_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM1_MOD1_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM1_MOD1_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM1_MOD1_B
    },
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM1_MOD1_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM1_EXT_SYNC0_SEL_OFFSET,
  },
#endif
#ifdef CONFIG_IMXRT_FLEXPWM1_MOD2
  {
    .module = 2,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM1_MOD2_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM1_MOD2_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM1_MOD2_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM1_MOD2_B
    },
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM1_MOD2_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM1_EXT_SYNC1_SEL_OFFSET,
  },
#endif
#ifdef CONFIG_IMXRT_FLEXPWM1_MOD3
  {
    .module = 3,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM1_MOD3_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM1_MOD3_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM1_MOD3_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM1_MOD3_B
    },
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM1_MOD3_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM1_EXT_SYNC2_SEL_OFFSET,
  },
#endif
#ifdef CONFIG_IMXRT_FLEXPWM1_MOD4
  {
    .module = 4,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM1_MOD4_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM1_MOD4_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM1_MOD4_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM1_MOD4_B
    },
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM1_MOD4_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM1_EXT_SYNC3_SEL_OFFSET,
  },
#endif
};

static struct imxrt_flexpwm_s g_pwm1 =
{
  .ops = &g_pwmops,
  .modules = g_pwm1_modules,
  .modules_num = FLEXPWM1_NMODULES,
  .frequency = 0,
  .base = IMXRT_FLEXPWM1_BASE,
};
#endif /* CONFIG_IMXRT_FLEXPWM1 */

#ifdef CONFIG_IMXRT_FLEXPWM2

static struct imxrt_flexpwm_module_s g_pwm2_modules[] =
{
  /* FlexPWM2 has 4 submodules with 2 outputs for each */

#ifdef CONFIG_IMXRT_FLEXPWM2_MOD1
  {
    .module = 1,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM2_MOD1_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM2_MOD1_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM2_MOD1_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM2_MOD1_B
    },
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM2_MOD1_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM2_EXT_SYNC0_SEL_OFFSET,
  },
#endif
#ifdef CONFIG_IMXRT_FLEXPWM2_MOD2
  {
    .module = 2,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM2_MOD2_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM2_MOD2_A,
    },
#ifdef CONFIG_IMXRT_FLEXPWM2_MOD2_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM2_MOD2_B
    }
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM2_MOD2_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM2_EXT_SYNC1_SEL_OFFSET,
  },
#endif
#ifdef CONFIG_IMXRT_FLEXPWM2_MOD3
  {
    .module = 3,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM2_MOD3_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM2_MOD3_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM2_MOD3_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM2_MOD3_B
    },
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM2_MOD3_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM2_EXT_SYNC2_SEL_OFFSET,
  },
#endif
#ifdef CONFIG_IMXRT_FLEXPWM2_MOD4
  {
    .module = 4,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM2_MOD4_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM2_MOD4_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM2_MOD4_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM2_MOD4_B
    },
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM2_MOD4_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM2_EXT_SYNC3_SEL_OFFSET,
  }
#endif
};

static struct imxrt_flexpwm_s g_pwm2 =
{
  .ops = &g_pwmops,
  .modules = g_pwm2_modules,
  .modules_num = FLEXPWM2_NMODULES,
  .frequency = 0,
  .base = IMXRT_FLEXPWM2_BASE,
};
#endif /* CONFIG_IMXRT_FLEXPWM2 */

#ifdef CONFIG_IMXRT_FLEXPWM3

static struct imxrt_flexpwm_module_s g_pwm3_modules[] =
{
  /* FlexPWM3 has 4 submodules with 2 outputs for each */

#ifdef CONFIG_IMXRT_FLEXPWM3_MOD1
  {
    .module = 1,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM3_MOD1_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM3_MOD1_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM3_MOD1_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM3_MOD1_B
    },
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM3_MOD1_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM3_EXT_SYNC0_SEL_OFFSET,
  },
#endif
#ifdef CONFIG_IMXRT_FLEXPWM3_MOD2
  {
    .module = 2,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM3_MOD2_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM3_MOD2_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM3_MOD2_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM3_MOD2_B
    },
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM3_MOD2_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM3_EXT_SYNC1_SEL_OFFSET,
  },
#endif
#ifdef CONFIG_IMXRT_FLEXPWM3_MOD3
  {
    .module = 3,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM3_MOD3_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM3_MOD3_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM3_MOD3_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM3_MOD3_B
    },
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM3_MOD3_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM3_EXT_SYNC2_SEL_OFFSET,
  },
#endif
#ifdef CONFIG_IMXRT_FLEXPWM3_MOD4
  {
    .module = 4,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM3_MOD4_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM3_MOD4_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM3_MOD4_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM3_MOD4_B
    },
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM3_MOD4_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM3_EXT_SYNC3_SEL_OFFSET,
  },
#endif
};

static struct imxrt_flexpwm_s g_pwm3 =
{
  .ops = &g_pwmops,
  .modules = g_pwm3_modules,
  .modules_num = FLEXPWM3_NMODULES,
  .frequency = 0,
  .base = IMXRT_FLEXPWM3_BASE,
};
#endif /* CONFIG_IMXRT_FLEXPWM3 */

#ifdef CONFIG_IMXRT_FLEXPWM4

static struct imxrt_flexpwm_module_s g_pwm4_modules[] =
{
  /* FlexPWM4 has 4 submodules with 2 outputs for each */

#ifdef CONFIG_IMXRT_FLEXPWM4_MOD1
  {
    .module = 1,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM4_MOD1_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM4_MOD1_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM4_MOD1_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM4_MOD1_B
    }
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM4_MOD1_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM4_EXT_SYNC0_SEL_OFFSET,
  },
#endif
#ifdef CONFIG_IMXRT_FLEXPWM4_MOD2
  {
    .module = 2,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM4_MOD2_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM4_MOD2_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM4_MOD2_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM4_MOD2_B
    }
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM4_MOD2_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM4_EXT_SYNC1_SEL_OFFSET,
  },
#endif
#ifdef CONFIG_IMXRT_FLEXPWM4_MOD3
  {
    .module = 3,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM4_MOD3_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM4_MOD3_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM4_MOD3_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM4_MOD3_B
    }
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM4_MOD3_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM4_EXT_SYNC3_SEL_OFFSET,
  },
#endif
#ifdef CONFIG_IMXRT_FLEXPWM4_MOD4
  {
    .module = 4,
    .used = true,
#ifdef CONFIG_IMXRT_FLEXPWM4_MOD4_TRIG
    .trig_en = true,
#else
    .trig_en = false,
#endif
    .out_a =
    {
      .used = true,
      .pin = GPIO_FLEXPWM4_MOD4_A
    },
#ifdef CONFIG_IMXRT_FLEXPWM4_MOD3_COMP
    .out_b =
    {
      .used = true,
      .pin = GPIO_FLEXPWM4_MOD4_B
    }
    .complementary = true,
#else
    .complementary = false,
#endif
    .sync_src = IMXRT_XBARA1(XBAR_INPUT,
                             CONFIG_IMXRT_FLEXPWM4_MOD4_SYNC_SRC),
    .ext_sync = IMXRT_XBARA1_OUT_FLEXPWM4_EXT_SYNC4_SEL_OFFSET,
  },
#endif
};

static struct imxrt_flexpwm_s g_pwm4 =
{
  .ops = &g_pwmops,
  .modules = g_pwm4_modules,
  .modules_num = FLEXPWM4_NMODULES,
  .frequency = 0,
  .base = IMXRT_FLEXPWM4_BASE,
};
#endif /* CONFIG_IMXRT_FLEXPWM4 */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

static int pwm_change_freq(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info,
                           uint8_t channel)
{
  struct imxrt_flexpwm_s *priv = (struct imxrt_flexpwm_s *)dev;
#ifdef CONFIG_PWM_MULTICHAN
  uint8_t shift = info->channels[channel].channel - 1;
#else
  uint8_t shift = priv->modules[0].module - 1;
#endif
  uint16_t regval;
  uint16_t olddiv = getreg16(priv->base + IMXRT_FLEXPWM_SM0VAL1_OFFSET
                                        + MODULE_OFFSET * shift);
  uint32_t newdiv = (CLK_FREQ + (info->frequency / 2)) / info->frequency;
  uint16_t prescale = 0;

  while (newdiv > PWM_RES && prescale < 7)
    {
      newdiv = (newdiv + 1) >> 1;
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

  regval = getreg16(priv->base + IMXRT_FLEXPWM_MCTRL_OFFSET);
  regval |= MCTRL_CLDOK(1 << shift);
  putreg16(regval, priv->base + IMXRT_FLEXPWM_MCTRL_OFFSET);

  regval = SMCTRL_FULL | SMCTRL_PRSC(prescale);
  putreg16(regval, priv->base + IMXRT_FLEXPWM_SM0CTRL_OFFSET
                              + MODULE_OFFSET * shift);

  putreg16(newdiv - 1, priv->base + IMXRT_FLEXPWM_SM0VAL1_OFFSET
                              + MODULE_OFFSET * shift);

  /* Update VAL0, VAL3 and VAL5 registers */

  regval = getreg16(priv->base + IMXRT_FLEXPWM_SM0VAL0_OFFSET
                               + MODULE_OFFSET * shift);
  regval = regval * newdiv / olddiv;
  putreg16(regval, priv->base + IMXRT_FLEXPWM_SM0VAL0_OFFSET
                              + MODULE_OFFSET * shift);

  regval = getreg16(priv->base + IMXRT_FLEXPWM_SM0VAL3_OFFSET
                               + MODULE_OFFSET * shift);
  regval = regval * newdiv / olddiv;
  putreg16(regval, priv->base + IMXRT_FLEXPWM_SM0VAL3_OFFSET
                              + MODULE_OFFSET * shift);

  regval = getreg16(priv->base + IMXRT_FLEXPWM_SM0VAL5_OFFSET
                               + MODULE_OFFSET * shift);
  regval = regval * newdiv / olddiv;
  putreg16(regval, priv->base + IMXRT_FLEXPWM_SM0VAL5_OFFSET
                              + MODULE_OFFSET * shift);

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

static int pwm_set_output(struct pwm_lowerhalf_s *dev, uint8_t channel,
                          ub16_t duty)
{
  struct imxrt_flexpwm_s *priv = (struct imxrt_flexpwm_s *)dev;
  uint16_t period;
  uint16_t width;
  uint16_t regval;
  uint8_t shift = channel - 1;  /* Shift submodle offset addresses */

  /* Get the period value */

  period = getreg16(priv->base + IMXRT_FLEXPWM_SM0VAL1_OFFSET
                               + MODULE_OFFSET * shift);

  /* Compute PWM width (count value to set PWM low) */

  width = b16toi(duty * period + b16HALF);

  /* Clear corresponding MCTRL[LDOK] bit  */

  regval = getreg16(priv->base + IMXRT_FLEXPWM_MCTRL_OFFSET);
  regval |= MCTRL_CLDOK(1 << shift);
  putreg16(regval, priv->base + IMXRT_FLEXPWM_MCTRL_OFFSET);

  /* Write width to value register 3 and enable output A */

  putreg16(width, priv->base + IMXRT_FLEXPWM_SM0VAL3_OFFSET
                             + MODULE_OFFSET * shift);

  regval = getreg16(priv->base + IMXRT_FLEXPWM_OUTEN_OFFSET);
  regval |= OUTEN_PWMA_EN(1 << shift);
  putreg16(regval, priv->base + IMXRT_FLEXPWM_OUTEN_OFFSET);

  /* Enable output B if complementary option is turn on */

  if (priv->modules[shift].complementary)
    {
      regval = getreg16(priv->base + IMXRT_FLEXPWM_OUTEN_OFFSET);
      regval |= OUTEN_PWMB_EN(1 << shift);
      putreg16(regval, priv->base + IMXRT_FLEXPWM_OUTEN_OFFSET);
    }

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

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct imxrt_flexpwm_s *priv = (struct imxrt_flexpwm_s *)dev;
  uint32_t pin = 0;
  uint16_t regval;
  uint8_t shift;
  int ret;

  putreg16(FCTRL0_FLVL(15), priv->base + IMXRT_FLEXPWM_FCTRL0_OFFSET);
  putreg16(0x000f, priv->base + IMXRT_FLEXPWM_FSTS0_OFFSET);
  putreg16(0, priv->base + IMXRT_FLEXPWM_FFILT0_OFFSET);

  for (int i = 0; i < priv->modules_num; i++)
    {
      /* Configure the module only if is set to be used */

      if (priv->modules[i].used != 1)
        {
          continue;
        }

      if (priv->modules[i].out_a.used)
        {
          pin = priv->modules[i].out_a.pin;
          if (pin != 0)
            {
              imxrt_config_gpio(pin);
            }
        }

      /* Configure PIN_B if complementary option is turn on */

      if (priv->modules[i].complementary)
        {
          if (priv->modules[i].out_b.used)
            {
              pin = priv->modules[i].out_b.pin;
              if (pin != 0)
                {
                  imxrt_config_gpio(pin);
                }
            }
        }

      shift = priv->modules[i].module - 1;

      regval = getreg16(priv->base + IMXRT_FLEXPWM_MCTRL_OFFSET);
      regval |= MCTRL_CLDOK(1 << shift);
      putreg16(regval, priv->base + IMXRT_FLEXPWM_MCTRL_OFFSET);

      /* Set control registers 1 and 2 */

      regval = 0;

      if (!priv->modules[i].complementary)
        {
          /* Enable independent PWM_A and PWM_B output */

          regval |= SMCTRL2_INDEP;
        }

      putreg16(regval, priv->base + IMXRT_FLEXPWM_SM0CTRL2_OFFSET
                                  + MODULE_OFFSET * shift);

      regval = SMCTRL_FULL;     /* Enable full read cycle reload */
      putreg16(regval, priv->base + IMXRT_FLEXPWM_SM0CTRL_OFFSET
                                  + MODULE_OFFSET * shift);

      /* Set output control register */

      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0OCTRL_OFFSET
                             + MODULE_OFFSET * shift);

      /* Set deadtime count register 0 */

      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0DTCNT0_OFFSET
                             + MODULE_OFFSET * shift);

      /* Set initial count register */

      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0INIT_OFFSET
                             + MODULE_OFFSET * shift);

      /* Set fraction value registers */

      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0FRACVAL1_OFFSET
                             + MODULE_OFFSET * shift);
      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0FRACVAL2_OFFSET
                             + MODULE_OFFSET * shift);
      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0FRACVAL3_OFFSET
                             + MODULE_OFFSET * shift);
      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0FRACVAL4_OFFSET
                             + MODULE_OFFSET * shift);
      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0FRACVAL5_OFFSET
                             + MODULE_OFFSET * shift);

      /* Set value registers */

      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0VAL0_OFFSET
                             + MODULE_OFFSET * shift);
      putreg16(0x82b8, priv->base + IMXRT_FLEXPWM_SM0VAL1_OFFSET
                                  + MODULE_OFFSET * shift);
      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0VAL2_OFFSET
                             + MODULE_OFFSET * shift);
      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0VAL3_OFFSET
                             + MODULE_OFFSET * shift);
      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0VAL4_OFFSET
                             + MODULE_OFFSET * shift);
      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0VAL5_OFFSET
                             + MODULE_OFFSET * shift);

      /* Is this PWM set to synchronized by external signal (trigger)? */

      if (priv->modules[i].sync_src != -1)
        {
          regval = getreg16(priv->base + IMXRT_FLEXPWM_SM0CTRL2_OFFSET
                                       + MODULE_OFFSET * shift);
          regval |= SMCTRL2_INIT_SEL_EXT_SYNC | SMCTRL2_FORCE_SEL_EXT_SYNC |
                    SMCTRL2_FRCEN;
          putreg16(regval, priv->base + IMXRT_FLEXPWM_SM0CTRL2_OFFSET
                                      + MODULE_OFFSET * shift);

          /* Connect corresponding XBARs */

          ret = imxrt_xbar_connect(priv->modules[i].ext_sync,
                                   priv->modules[i].sync_src);
          if (ret < 0)
            {
              pwmerr("ERROR: imxrt_xbar_connect failed: %d\n", ret);
            }
        }

      if (priv->modules[i].trig_en)
        {
          /* Use duty cycle register for trigger generation */

          regval = getreg16(priv->base + IMXRT_FLEXPWM_SM0TCTRL_OFFSET
                                       + MODULE_OFFSET * shift);
          regval |= SMT_OUT_TRIG_EN_VAL1;
          putreg16(regval, priv->base + IMXRT_FLEXPWM_SM0TCTRL_OFFSET
                                      + MODULE_OFFSET * shift);
        }

      regval = getreg16(priv->base + IMXRT_FLEXPWM_MCTRL_OFFSET);
      regval |= MCTRL_LDOK(1 << shift);
      putreg16(regval, priv->base + IMXRT_FLEXPWM_MCTRL_OFFSET);

      regval = getreg16(priv->base + IMXRT_FLEXPWM_MCTRL_OFFSET);
      regval |= MCTRL_RUN(1 << shift);
      putreg16(regval, priv->base + IMXRT_FLEXPWM_MCTRL_OFFSET);
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

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct imxrt_flexpwm_s *priv = (struct imxrt_flexpwm_s *)dev;

  for (int i = 0; i < priv->modules_num; i++)
    {
      /* Skip modules that are not used */

      if (priv->modules[i].used != 1)
        {
          continue;
        }

      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0CTRL_OFFSET
                                  + MODULE_OFFSET * i);

      /* Reset fractional value registers */

      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0VAL0_OFFSET
                             + MODULE_OFFSET * i);
      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0VAL1_OFFSET
                                  + MODULE_OFFSET * i);
      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0VAL2_OFFSET
                             + MODULE_OFFSET * i);
      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0VAL3_OFFSET
                             + MODULE_OFFSET * i);
      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0VAL4_OFFSET
                             + MODULE_OFFSET * i);
      putreg16(0, priv->base + IMXRT_FLEXPWM_SM0VAL5_OFFSET
                             + MODULE_OFFSET * i);
    }

  /* Stop run */

  putreg16(0, priv->base + IMXRT_FLEXPWM_MCTRL_OFFSET);

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
  struct imxrt_flexpwm_s *priv = (struct imxrt_flexpwm_s *)dev;
  int ret = OK;
  uint8_t ldok_map = 0;

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
          ret = pwm_set_output(dev, info->channels[i].channel,
                                    info->channels[i].duty);

          /* Remember the channel number in bitmap */

          ldok_map |= 1 << (info->channels[i].channel - 1);
        }
    }
#else
  /* Enable PWM output just for first channel */

  ret = pwm_set_output(dev, priv->modules[0].module, info->duty);

  /* Remember the channel number in bitmap */

  ldok_map = 1 << (priv->modules[0].module - 1);
#endif /* CONFIG_PWM_MULTICHAN */

  /* Set Load Okay bits */

  uint16_t regval = getreg16(priv->base + IMXRT_FLEXPWM_MCTRL_OFFSET);
  regval |= MCTRL_LDOK(ldok_map);
  putreg16(regval, priv->base + IMXRT_FLEXPWM_MCTRL_OFFSET);

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

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct imxrt_flexpwm_s *priv = (struct imxrt_flexpwm_s *)dev;
  uint8_t shift;
  uint16_t regval;

#ifdef CONFIG_PWM_MULTICHAN
  for (int i = 0; i < priv->modules_num; i++)
    {
      /* Skip settings if channel is not configured */

      if (!priv->modules[i].used)
        {
          continue;
        }

      shift = priv->modules[i].module - 1;

      regval = OUTEN_PWMA_EN(0 << shift);
      putreg16(regval, priv->base + IMXRT_FLEXPWM_OUTEN_OFFSET);

      regval = OUTEN_PWMB_EN(0 << shift);
      putreg16(regval, priv->base + IMXRT_FLEXPWM_OUTEN_OFFSET);
    }
#else
    shift = priv->modules[0].module - 1;

    regval = OUTEN_PWMA_EN(0 << shift);
    putreg16(regval, priv->base + IMXRT_FLEXPWM_OUTEN_OFFSET);

    regval = OUTEN_PWMB_EN(0 << shift);
    putreg16(regval, priv->base + IMXRT_FLEXPWM_OUTEN_OFFSET);

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
 * Function: imxrt_pwminitialize
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

struct pwm_lowerhalf_s *imxrt_pwminitialize(int pwm)
{
  struct imxrt_flexpwm_s *priv;

  pwminfo("Initializing pwm %d\n", pwm);

  switch (pwm)
  {
#ifdef CONFIG_IMXRT_FLEXPWM1
    case 1:
      imxrt_clockall_pwm1();
      priv = &g_pwm1;
      break;
#endif
#ifdef CONFIG_IMXRT_FLEXPWM2
    case 2:
      imxrt_clockall_pwm2();
      priv = &g_pwm2;
      break;
#endif
#ifdef CONFIG_IMXRT_FLEXPWM3
    case 3:
      imxrt_clockall_pwm3();
      priv = &g_pwm3;
      break;
#endif
#ifdef CONFIG_IMXRT_FLEXPWM4
    case 4:
      imxrt_clockall_pwm4();
      priv = &g_pwm4;
      break;
#endif
    default:
      pwmerr("ERROR: PWM number invalid or not configured %d\n", pwm);
      return NULL;
  }

  return (struct pwm_lowerhalf_s *)priv;
}
#endif /* CONFIG_IMXRT_FLEXPWM */
