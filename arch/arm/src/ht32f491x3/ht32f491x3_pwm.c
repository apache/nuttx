/****************************************************************************
 * arch/arm/src/ht32f491x3/ht32f491x3_pwm.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with this
 * work for additional information regarding copyright ownership.  The ASF
 * licenses this file to you under the Apache License, Version 2.0 (the
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
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ht32f491x3_gpio.h"
#include "ht32f491x3_pwm.h"

#include "hardware/ht32f491x3_crm.h"
#include "hardware/ht32f491x3_tmr.h"

#if defined(CONFIG_HT32F491X3_TMR1_PWM)  || defined(CONFIG_HT32F491X3_TMR2_PWM) || \
    defined(CONFIG_HT32F491X3_TMR3_PWM)  || defined(CONFIG_HT32F491X3_TMR4_PWM) || \
    defined(CONFIG_HT32F491X3_TMR9_PWM)  || defined(CONFIG_HT32F491X3_TMR10_PWM) || \
    defined(CONFIG_HT32F491X3_TMR11_PWM) || defined(CONFIG_HT32F491X3_TMR12_PWM) || \
    defined(CONFIG_HT32F491X3_TMR13_PWM) || defined(CONFIG_HT32F491X3_TMR14_PWM)
#  define HAVE_HT32F491X3_PWM 1
#endif

#if defined(CONFIG_PWM) && defined(HAVE_HT32F491X3_PWM)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ht32f491x3_pwmcfg_s
{
  uint8_t   timid;
  uint8_t   maxchannel;
  bool      advanced;
  uintptr_t base;
  uintptr_t clkreg;
  uint32_t  clkbit;
  uintptr_t rstreg;
  uint32_t  rstbit;
};

struct ht32f491x3_pwmtimer_s
{
  const struct pwm_ops_s            *ops;
  FAR const struct ht32f491x3_pwmcfg_s *cfg;
  uint32_t                           gpio_clken;
  uintptr_t                          gpio_base;
  uint8_t                            channel;
  uint8_t                            pin;
  uint8_t                            af;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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
  .setup    = pwm_setup,
  .shutdown = pwm_shutdown,
  .start    = pwm_start,
  .stop     = pwm_stop,
  .ioctl    = pwm_ioctl,
};

/* TMR6 and TMR7 are basic timers and do not provide PWM output channels. */

#define HT32_PWM_DEFINE(_timid, _maxchannel, _advanced, _base, _clkreg,       \
                        _clkbit, _rstreg, _rstbit)                             \
  static const struct ht32f491x3_pwmcfg_s g_pwm##_timid##cfg =                \
  {                                                                            \
    .timid      = _timid,                                                      \
    .maxchannel = _maxchannel,                                                 \
    .advanced   = _advanced,                                                   \
    .base       = _base,                                                       \
    .clkreg     = _clkreg,                                                     \
    .clkbit     = _clkbit,                                                     \
    .rstreg     = _rstreg,                                                     \
    .rstbit     = _rstbit,                                                     \
  };                                                                           \
                                                                               \
  static struct ht32f491x3_pwmtimer_s g_pwm##_timid##dev =                    \
  {                                                                            \
    .ops        = &g_pwmops,                                                   \
    .cfg        = &g_pwm##_timid##cfg,                                         \
    .channel    = CONFIG_HT32F491X3_TMR##_timid##_CHANNEL,                    \
    .gpio_clken = BOARD_TMR##_timid##_PWM_GPIO_CLKEN,                         \
    .gpio_base  = BOARD_TMR##_timid##_PWM_GPIO_BASE,                          \
    .pin        = BOARD_TMR##_timid##_PWM_GPIO_PIN,                           \
    .af         = BOARD_TMR##_timid##_PWM_GPIO_AF,                            \
  }

#ifdef CONFIG_HT32F491X3_TMR1_PWM
HT32_PWM_DEFINE(1, 4, true, HT32_TMR1_BASE, HT32_CRM_APB2EN,
                HT32_CRM_APB2EN_TMR1EN, HT32_CRM_APB2RST,
                HT32_CRM_APB2RST_TMR1RST);
#endif

#ifdef CONFIG_HT32F491X3_TMR2_PWM
HT32_PWM_DEFINE(2, 4, false, HT32_TMR2_BASE, HT32_CRM_APB1EN,
                HT32_CRM_APB1EN_TMR2EN, HT32_CRM_APB1RST,
                HT32_CRM_APB1RST_TMR2RST);
#endif

#ifdef CONFIG_HT32F491X3_TMR3_PWM
HT32_PWM_DEFINE(3, 4, false, HT32_TMR3_BASE, HT32_CRM_APB1EN,
                HT32_CRM_APB1EN_TMR3EN, HT32_CRM_APB1RST,
                HT32_CRM_APB1RST_TMR3RST);
#endif

#ifdef CONFIG_HT32F491X3_TMR4_PWM
HT32_PWM_DEFINE(4, 4, false, HT32_TMR4_BASE, HT32_CRM_APB1EN,
                HT32_CRM_APB1EN_TMR4EN, HT32_CRM_APB1RST,
                HT32_CRM_APB1RST_TMR4RST);
#endif

#ifdef CONFIG_HT32F491X3_TMR9_PWM
HT32_PWM_DEFINE(9, 2, false, HT32_TMR9_BASE, HT32_CRM_APB2EN,
                HT32_CRM_APB2EN_TMR9EN, HT32_CRM_APB2RST,
                HT32_CRM_APB2RST_TMR9RST);
#endif

#ifdef CONFIG_HT32F491X3_TMR10_PWM
HT32_PWM_DEFINE(10, 1, false, HT32_TMR10_BASE, HT32_CRM_APB2EN,
                HT32_CRM_APB2EN_TMR10EN, HT32_CRM_APB2RST,
                HT32_CRM_APB2RST_TMR10RST);
#endif

#ifdef CONFIG_HT32F491X3_TMR11_PWM
HT32_PWM_DEFINE(11, 1, false, HT32_TMR11_BASE, HT32_CRM_APB2EN,
                HT32_CRM_APB2EN_TMR11EN, HT32_CRM_APB2RST,
                HT32_CRM_APB2RST_TMR11RST);
#endif

#ifdef CONFIG_HT32F491X3_TMR12_PWM
HT32_PWM_DEFINE(12, 2, false, HT32_TMR12_BASE, HT32_CRM_APB1EN,
                HT32_CRM_APB1EN_TMR12EN, HT32_CRM_APB1RST,
                HT32_CRM_APB1RST_TMR12RST);
#endif

#ifdef CONFIG_HT32F491X3_TMR13_PWM
HT32_PWM_DEFINE(13, 1, false, HT32_TMR13_BASE, HT32_CRM_APB1EN,
                HT32_CRM_APB1EN_TMR13EN, HT32_CRM_APB1RST,
                HT32_CRM_APB1RST_TMR13RST);
#endif

#ifdef CONFIG_HT32F491X3_TMR14_PWM
HT32_PWM_DEFINE(14, 1, false, HT32_TMR14_BASE, HT32_CRM_APB1EN,
                HT32_CRM_APB1EN_TMR14EN, HT32_CRM_APB1RST,
                HT32_CRM_APB1RST_TMR14RST);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t pwm_getreg(FAR struct ht32f491x3_pwmtimer_s *priv,
                                  unsigned int offset)
{
  DEBUGASSERT(priv != NULL && priv->cfg != NULL);
  return getreg32(priv->cfg->base + offset);
}

static inline void pwm_putreg(FAR struct ht32f491x3_pwmtimer_s *priv,
                              unsigned int offset, uint32_t value)
{
  DEBUGASSERT(priv != NULL && priv->cfg != NULL);
  putreg32(value, priv->cfg->base + offset);
}

static inline unsigned int pwm_cm_offset(uint8_t channel)
{
  return channel <= 2 ? HT32_TMR_CM1_OFFSET : HT32_TMR_CM2_OFFSET;
}

static inline unsigned int pwm_cm_slot(uint8_t channel)
{
  return (channel - 1u) & 1u;
}

static inline uint32_t pwm_cm_mask(uint8_t channel)
{
  unsigned int slot = pwm_cm_slot(channel);

  return HT32_TMR_CM_CAPTURE_SEL_MASK(slot) |
         HT32_TMR_CM_OUTPUT_BUFFER(slot) |
         HT32_TMR_CM_OUTPUT_MODE_MASK(slot);
}

static inline uint32_t pwm_cm_value(uint8_t channel)
{
  unsigned int slot = pwm_cm_slot(channel);

  return HT32_TMR_CM_OUTPUT_BUFFER(slot) |
         HT32_TMR_CM_OUTPUT_MODE(slot, HT32_TMR_OUTPUT_CONTROL_PWM_A);
}

static void ht32f491x3_pwm_enableclk(FAR struct ht32f491x3_pwmtimer_s *priv,
                                     bool enable)
{
  DEBUGASSERT(priv != NULL && priv->cfg != NULL);

  modifyreg32(priv->cfg->clkreg,
              priv->cfg->clkbit,
              enable ? priv->cfg->clkbit : 0);
}

static void ht32f491x3_pwm_reset(FAR struct ht32f491x3_pwmtimer_s *priv)
{
  DEBUGASSERT(priv != NULL && priv->cfg != NULL);

  modifyreg32(priv->cfg->rstreg, 0, priv->cfg->rstbit);
  modifyreg32(priv->cfg->rstreg, priv->cfg->rstbit, 0);
}

static void ht32f491x3_pwm_gpioconfig(FAR struct ht32f491x3_pwmtimer_s *priv,
                                      bool enable)
{
  modifyreg32(HT32_CRM_AHBEN1, 0, priv->gpio_clken);

  ht32f491x3_gpioconfig(priv->gpio_base, priv->pin,
                        enable ? HT32_GPIO_MODE_ALTFN :
                                 HT32_GPIO_MODE_INPUT,
                        false,
                        enable ? HT32_GPIO_DRIVE_HIGH :
                                 HT32_GPIO_DRIVE_LOW,
                        HT32_GPIO_PULL_NONE,
                        enable ? priv->af : 0);
}

static uint32_t ht32f491x3_pwm_timclk(FAR struct ht32f491x3_pwmtimer_s *priv)
{
  uint32_t regval = getreg32(HT32_CRM_CFG);
  uint32_t pclk;
  bool doubled;

  DEBUGASSERT(priv != NULL && priv->cfg != NULL);

  if (priv->cfg->clkreg == HT32_CRM_APB2EN)
    {
      pclk = CONFIG_HT32F491X3_PCLK2_FREQUENCY;
      doubled = (regval & HT32_CRM_CFG_APB2DIV_MASK) !=
                HT32_CRM_CFG_APB2DIV_1;
    }
  else
    {
      pclk = CONFIG_HT32F491X3_PCLK1_FREQUENCY;
      doubled = (regval & HT32_CRM_CFG_APB1DIV_MASK) !=
                HT32_CRM_CFG_APB1DIV_1;
    }

  return doubled ? pclk * 2u : pclk;
}

static int pwm_timer(FAR struct ht32f491x3_pwmtimer_s *priv,
                     FAR const struct pwm_info_s *info)
{
  uint64_t timclk;
  uint64_t cycles;
  uint32_t prescaler;
  uint32_t reload;
  uint32_t pulse;
  uint32_t regval;

  DEBUGASSERT(priv != NULL && info != NULL);

  if (info->frequency == 0)
    {
      return -EINVAL;
    }

  timclk = ht32f491x3_pwm_timclk(priv);
  cycles = (timclk + (info->frequency / 2u)) / info->frequency;

  if (cycles == 0)
    {
      cycles = 1;
    }

  prescaler = (cycles + 65535u) / 65536u;
  if (prescaler == 0)
    {
      prescaler = 1;
    }

  if (prescaler > 65536u)
    {
      pwmerr("ERROR: PWM frequency %" PRIu32 "Hz is out of range\n",
             info->frequency);
      return -ERANGE;
    }

  reload = (uint32_t)((cycles + (prescaler / 2u)) / prescaler);
  if (reload < 2u)
    {
      reload = 2u;
    }
  else if (reload > 65536u)
    {
      reload = 65536u;
    }

  reload -= 1u;
  pulse = (uint32_t)(((uint64_t)reload * info->duty + 0x8000ull) >> 16);
  if (pulse > reload)
    {
      pulse = reload;
    }

  ht32f491x3_pwm_enableclk(priv, true);
  ht32f491x3_pwm_reset(priv);
  ht32f491x3_pwm_gpioconfig(priv, true);

  pwm_putreg(priv, HT32_TMR_CTRL1_OFFSET, HT32_TMR_CTRL1_COUNTUP);
  pwm_putreg(priv, HT32_TMR_CTRL2_OFFSET, 0);
  pwm_putreg(priv, HT32_TMR_CCTRL_OFFSET, 0);
  pwm_putreg(priv, HT32_TMR_CVAL_OFFSET, 0);
  pwm_putreg(priv, HT32_TMR_DIV_OFFSET, prescaler - 1u);
  pwm_putreg(priv, HT32_TMR_PR_OFFSET, reload);
  pwm_putreg(priv, HT32_TMR_CCR_OFFSET(priv->channel), pulse);
  pwm_putreg(priv, HT32_TMR_SWEVT_OFFSET, HT32_TMR_SWEVT_OVFSWTR);

  regval = pwm_getreg(priv, pwm_cm_offset(priv->channel));
  regval &= ~pwm_cm_mask(priv->channel);
  regval |= pwm_cm_value(priv->channel);
  pwm_putreg(priv, pwm_cm_offset(priv->channel), regval);

  regval = pwm_getreg(priv, HT32_TMR_CCTRL_OFFSET);
  regval &= ~(HT32_TMR_CCTRL_EN(priv->channel) |
              HT32_TMR_CCTRL_POL(priv->channel));
  regval |= HT32_TMR_CCTRL_EN(priv->channel);
  pwm_putreg(priv, HT32_TMR_CCTRL_OFFSET, regval);

  if (priv->cfg->advanced)
    {
      modifyreg32(priv->cfg->base + HT32_TMR_BRK_OFFSET, 0,
                  HT32_TMR_BRK_OEN);
    }

  modifyreg32(priv->cfg->base + HT32_TMR_CTRL1_OFFSET, 0,
              HT32_TMR_CTRL1_PRBEN | HT32_TMR_CTRL1_TMREN);

  return OK;
}

/****************************************************************************
 * PWM driver methods
 ****************************************************************************/

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct ht32f491x3_pwmtimer_s *priv =
    (FAR struct ht32f491x3_pwmtimer_s *)dev;

  ht32f491x3_pwm_enableclk(priv, true);
  ht32f491x3_pwm_gpioconfig(priv, true);
  return OK;
}

static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct ht32f491x3_pwmtimer_s *priv =
    (FAR struct ht32f491x3_pwmtimer_s *)dev;

  pwm_stop(dev);
  ht32f491x3_pwm_gpioconfig(priv, false);
  ht32f491x3_pwm_enableclk(priv, false);
  return OK;
}

static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info)
{
  FAR struct ht32f491x3_pwmtimer_s *priv =
    (FAR struct ht32f491x3_pwmtimer_s *)dev;

  return pwm_timer(priv, info);
}

static int pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct ht32f491x3_pwmtimer_s *priv =
    (FAR struct ht32f491x3_pwmtimer_s *)dev;
  irqstate_t flags;

  flags = enter_critical_section();
  modifyreg32(priv->cfg->base + HT32_TMR_CTRL1_OFFSET,
              HT32_TMR_CTRL1_TMREN, 0);
  modifyreg32(priv->cfg->base + HT32_TMR_CCTRL_OFFSET,
              HT32_TMR_CCTRL_EN(priv->channel), 0);
  leave_critical_section(flags);

  return OK;
}

static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg)
{
  (void)dev;
  (void)cmd;
  (void)arg;
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct pwm_lowerhalf_s *ht32f491x3_pwminitialize(int timer)
{
  FAR struct ht32f491x3_pwmtimer_s *lower = NULL;

  pwminfo("Initialize PWM timer %d\n", timer);

  switch (timer)
    {
#ifdef CONFIG_HT32F491X3_TMR1_PWM
      case 1:
        {
          lower = &g_pwm1dev;
          break;
        }
#endif

#ifdef CONFIG_HT32F491X3_TMR2_PWM
      case 2:
        {
          lower = &g_pwm2dev;
          break;
        }
#endif

#ifdef CONFIG_HT32F491X3_TMR3_PWM
      case 3:
        {
          lower = &g_pwm3dev;
          break;
        }
#endif

#ifdef CONFIG_HT32F491X3_TMR4_PWM
      case 4:
        {
          lower = &g_pwm4dev;
          break;
        }
#endif

#ifdef CONFIG_HT32F491X3_TMR9_PWM
      case 9:
        {
          lower = &g_pwm9dev;
          break;
        }
#endif

#ifdef CONFIG_HT32F491X3_TMR10_PWM
      case 10:
        {
          lower = &g_pwm10dev;
          break;
        }
#endif

#ifdef CONFIG_HT32F491X3_TMR11_PWM
      case 11:
        {
          lower = &g_pwm11dev;
          break;
        }
#endif

#ifdef CONFIG_HT32F491X3_TMR12_PWM
      case 12:
        {
          lower = &g_pwm12dev;
          break;
        }
#endif

#ifdef CONFIG_HT32F491X3_TMR13_PWM
      case 13:
        {
          lower = &g_pwm13dev;
          break;
        }
#endif

#ifdef CONFIG_HT32F491X3_TMR14_PWM
      case 14:
        {
          lower = &g_pwm14dev;
          break;
        }
#endif

      default:
        {
          pwmerr("ERROR: No such PWM timer %d\n", timer);
          return NULL;
        }
    }

  DEBUGASSERT(lower != NULL && lower->cfg != NULL);

  if (lower->channel == 0 || lower->channel > lower->cfg->maxchannel)
    {
      pwmerr("ERROR: PWM timer %d does not support channel %" PRIu8 "\n",
             timer, lower->channel);
      return NULL;
    }

  return (struct pwm_lowerhalf_s *)lower;
}

#endif /* CONFIG_PWM && HAVE_HT32F491X3_PWM */
