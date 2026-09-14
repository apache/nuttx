/****************************************************************************
 * arch/arm/src/am67/am67_ecap.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/timers/pwm.h>

#include "arm_internal.h"
#include "am67_pinmux.h"
#include "am67_ecap.h"
#include "am67_ecap_hw.h"

#if defined(CONFIG_AM67_ECAP0) || defined(CONFIG_AM67_ECAP1) || \
    defined(CONFIG_AM67_ECAP2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* eCAP FICLK = MAIN_SYSCLK0/4 = half the EPWM FICLK (measured 250 MHz).
 * TRM Table 4-344.
 */

#define AM67_ECAP_VBUSCLK_HZ            125000000u

/* APRD is 32-bit; the few-tick floor leaves room for the duty compare. */

#define AM67_ECAP_MIN_TICKS             2u
#define AM67_ECAP_MAX_TICKS             0xffffffffu

/* MAIN_CTRL_MMR partition 1 lock (kick) - the same unlock EPWM performs in
 * am67_epwm_init, so CTRL_MMR-side writes can land.
 */

#define AM67_MAIN_CTRL_MMR_BASE         0x00100000
#define AM67_CTRL_MMR_LOCK1_KICK0       0x5008
#define AM67_CTRL_MMR_LOCK1_KICK1       0x500c
#define AM67_CTRL_MMR_KICK0_UNLOCK_KEY  0x68ef3490
#define AM67_CTRL_MMR_KICK0_UNLOCKED    (1u << 0)
#define AM67_CTRL_MMR_KICK1_UNLOCK_KEY  0xd172bc5a

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Lower-half state; ops must stay first so this casts to and from
 * struct pwm_lowerhalf_s.
 */

struct am67_ecap_s
{
  const struct pwm_ops_s *ops;
  uint32_t base;
  uint32_t frequency;       /* Cached running frequency (0 = stopped) */
  uint32_t period;          /* Cached APRD+1 ticks, for the duty math */
  uint8_t pinmux_id;        /* Module number for pad muxing */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* PWM driver methods */

static int am67_ecap_setup(struct pwm_lowerhalf_s *dev);
static int am67_ecap_shutdown(struct pwm_lowerhalf_s *dev);
static int am67_ecap_start(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info);
static int am67_ecap_stop(struct pwm_lowerhalf_s *dev);
static int am67_ecap_ioctl(struct pwm_lowerhalf_s *dev,
                           int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pwm_ops_s g_am67_ecapops =
{
  .setup       = am67_ecap_setup,
  .shutdown    = am67_ecap_shutdown,
  .start       = am67_ecap_start,
  .stop        = am67_ecap_stop,
  .ioctl       = am67_ecap_ioctl,
};

#ifdef CONFIG_AM67_ECAP0
static struct am67_ecap_s g_am67_ecap0 =
{
  .ops        = &g_am67_ecapops,
  .base       = AM67_ECAP0_BASE,
  .pinmux_id  = 0,
};
#endif

#ifdef CONFIG_AM67_ECAP1
static struct am67_ecap_s g_am67_ecap1 =
{
  .ops        = &g_am67_ecapops,
  .base       = AM67_ECAP1_BASE,
  .pinmux_id  = 1,
};
#endif

#ifdef CONFIG_AM67_ECAP2
static struct am67_ecap_s g_am67_ecap2 =
{
  .ops        = &g_am67_ecapops,
  .base       = AM67_ECAP2_BASE,
  .pinmux_id  = 2,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_ecap_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset.
 *
 ****************************************************************************/

static inline uint32_t am67_ecap_getreg(uint32_t base, uint32_t offset)
{
  return getreg32(base + offset);
}

/****************************************************************************
 * Name: am67_ecap_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset.
 *
 ****************************************************************************/

static inline void am67_ecap_putreg(uint32_t base, uint32_t offset,
                                    uint32_t value)
{
  putreg32(value, base + offset);
}

/****************************************************************************
 * Name: am67_ecap_enable_register_write
 *
 * Description:
 *   Unlock MAIN_CTRL_MMR partition 1 (kick lock), mirroring am67_epwm_init.
 *   The KICK0 status bit is read back to confirm the partition is unlocked.
 *
 * Returned Value:
 *   Zero (OK) on success; -EIO if the partition is still locked.
 *
 ****************************************************************************/

static int am67_ecap_enable_register_write(void)
{
  uint32_t regval = am67_ecap_getreg(AM67_MAIN_CTRL_MMR_BASE,
                                     AM67_CTRL_MMR_LOCK1_KICK0);

  if ((regval & AM67_CTRL_MMR_KICK0_UNLOCKED) == 0u)
    {
      am67_ecap_putreg(AM67_MAIN_CTRL_MMR_BASE, AM67_CTRL_MMR_LOCK1_KICK0,
                       AM67_CTRL_MMR_KICK0_UNLOCK_KEY);
      am67_ecap_putreg(AM67_MAIN_CTRL_MMR_BASE, AM67_CTRL_MMR_LOCK1_KICK1,
                       AM67_CTRL_MMR_KICK1_UNLOCK_KEY);
    }

  regval = am67_ecap_getreg(AM67_MAIN_CTRL_MMR_BASE,
                            AM67_CTRL_MMR_LOCK1_KICK0);

  if ((regval & AM67_CTRL_MMR_KICK0_UNLOCKED) == 0u)
    {
      pwmerr("ERROR: Could not unlock CTRL_MMR partition 1\n");
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: am67_ecap_reset_counter
 *
 * Description:
 *   Zero the time-base counter (TSCTR).
 *
 ****************************************************************************/

static void am67_ecap_reset_counter(uint32_t base)
{
  am67_ecap_putreg(base, AM67_ECAP_TSCNT_OFFSET, 0u);
}

/****************************************************************************
 * Name: am67_ecap_config_apwm
 *
 * Description:
 *   Compose ECCTL for frozen APWM output (start() ignites the counter).
 *   Full write - this function owns every policy field.
 *
 ****************************************************************************/

static void am67_ecap_config_apwm(uint32_t base)
{
  uint32_t regval;

  regval  = AM67_ECAP_ECCTL_CAP_APWM;      /* capture -> APWM mode */
  regval |= AM67_ECAP_ECCTL_SYNCO_SEL_DIS; /* no sync-out */

  /* Left 0: APWMPOL (active hi), TSCNTSTP, SYNCI_EN, CONT_ONESHT */

  am67_ecap_putreg(base, AM67_ECAP_ECCTL_OFFSET, regval);
}

/****************************************************************************
 * Name: am67_ecap_counter_run
 *
 * Description:
 *   Ignition: set TSCNTSTP so the counter counts up.
 *
 ****************************************************************************/

static void am67_ecap_counter_run(uint32_t base)
{
  uint32_t regval = am67_ecap_getreg(base, AM67_ECAP_ECCTL_OFFSET);

  regval |= AM67_ECAP_ECCTL_TSCNTSTP;
  am67_ecap_putreg(base, AM67_ECAP_ECCTL_OFFSET, regval);
}

/****************************************************************************
 * Name: am67_ecap_counter_freeze
 *
 * Description:
 *   Freeze the counter (clear TSCNTSTP).
 *
 ****************************************************************************/

static void am67_ecap_counter_freeze(uint32_t base)
{
  uint32_t regval = am67_ecap_getreg(base, AM67_ECAP_ECCTL_OFFSET);

  regval &= ~AM67_ECAP_ECCTL_TSCNTSTP;
  am67_ecap_putreg(base, AM67_ECAP_ECCTL_OFFSET, regval);
}

/****************************************************************************
 * Name: am67_ecap_calculate_period
 *
 * Description:
 *   Map a frequency to the 32-bit APWM period (FICLK / frequency).
 *
 * Input Parameters:
 *   frequency - requested output frequency in Hz.
 *   period    - out: period in ticks (== APRD + 1).
 *
 * Returned Value:
 *   OK on success; -ERANGE if the frequency cannot be produced.
 *
 ****************************************************************************/

static int am67_ecap_calculate_period(uint32_t frequency, uint32_t *period)
{
  uint32_t ticks;

  if (frequency == 0u)
    {
      return -ERANGE;
    }

  ticks = AM67_ECAP_VBUSCLK_HZ / frequency;

  if (ticks < AM67_ECAP_MIN_TICKS || ticks > AM67_ECAP_MAX_TICKS)
    {
      return -ERANGE;
    }

  *period = ticks;
  return OK;
}

/****************************************************************************
 * Name: am67_ecap_write_immediate
 *
 * Description:
 *   Load period/compare into the ACTIVE CAP1/CAP2.  In APWM mode this also
 *   copies into the shadows, so the change is immediate - use on (re)start.
 *
 * Input Parameters:
 *   base    - eCAP module base address.
 *   period  - APRD value (period in ticks minus 1).
 *   compare - ACMP value (on-time in ticks, active high).
 *
 ****************************************************************************/

static void am67_ecap_write_immediate(uint32_t base, uint32_t period,
                                      uint32_t compare)
{
  am67_ecap_putreg(base, AM67_ECAP_CAP1_OFFSET, period);
  am67_ecap_putreg(base, AM67_ECAP_CAP2_OFFSET, compare);
}

/****************************************************************************
 * Name: am67_ecap_write_shadow
 *
 * Description:
 *   Load period/compare into the SHADOW CAP3/CAP4.  The hardware transfers
 *   them to CAP1/CAP2 at the next CTR = PRD event, so a run-time update
 *   lands on a period boundary without chopping the pulse.
 *
 * Input Parameters:
 *   base    - eCAP module base address.
 *   period  - APRD shadow value (period in ticks minus 1).
 *   compare - ACMP shadow value (on-time in ticks, active high).
 *
 ****************************************************************************/

static void am67_ecap_write_shadow(uint32_t base, uint32_t period,
                                   uint32_t compare)
{
  am67_ecap_putreg(base, AM67_ECAP_CAP3_OFFSET, period);
  am67_ecap_putreg(base, AM67_ECAP_CAP4_OFFSET, compare);
}

/****************************************************************************
 * Name: am67_ecap_park
 *
 * Description:
 *   Drive the output low via 0% duty (CMP = 0) with the counter left
 *   running, so a compare event lands the pin low within one period.  eCAP
 *   has no output force, so freezing instead could hold an arbitrary level.
 *
 ****************************************************************************/

static void am67_ecap_park(struct am67_ecap_s *priv)
{
  am67_ecap_putreg(priv->base, AM67_ECAP_CAP2_OFFSET, 0u);
}

/****************************************************************************
 * Name: am67_ecap_setup
 *
 * Description:
 *   First-open configuration: configure the output pad.  No output until
 *   start().
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

static int am67_ecap_setup(struct pwm_lowerhalf_s *dev)
{
  struct am67_ecap_s *priv = (struct am67_ecap_s *)dev;

  am67_ecap_pinmux_init(priv->pinmux_id);

  return OK;
}

/****************************************************************************
 * Name: am67_ecap_shutdown
 *
 * Description:
 *   Last close: park the output low via stop().  The module stays in APWM
 *   mode so the pin keeps being driven low (capture mode tri-states the
 *   pad); there is no clock to gate.
 *
 * Returned Value:
 *   Zero (OK) always.
 *
 ****************************************************************************/

static int am67_ecap_shutdown(struct pwm_lowerhalf_s *dev)
{
  return am67_ecap_stop(dev);
}

/****************************************************************************
 * Name: am67_ecap_start
 *
 * Description:
 *   Start or update the single APWM output (channel 1).  A frequency change
 *   reprograms the period and compare immediately and re-ignites the
 *   counter; a duty-only change shadow-loads so it lands on a period
 *   boundary.  A bad request leaves any running wave untouched.
 *
 * Returned Value:
 *   Zero (OK) on success; -EINVAL for a bad channel or duty; -ERANGE if the
 *   frequency cannot be produced.
 *
 ****************************************************************************/

static int am67_ecap_start(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info)
{
  struct am67_ecap_s *priv = (struct am67_ecap_s *)dev;
  ub16_t duty = 0;
  bool found = false;
  bool freq_changed;
  uint32_t period;
  uint32_t compare;
  int8_t ch;
  int i;

  /* config_apwm() clears TSCNTSTP (stops the counter) and reset_counter()
   * zeroes it, so both run only on a frequency change below - a duty-only
   * start() must not disturb the running counter.
   */

  /* Validate the channel array (only channel 1 exists) and read its duty. */

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      ch = info->channels[i].channel;

      if (ch == -1)
        {
          break;
        }

      if (ch == 0)
        {
          continue;
        }

      if (ch != 1)
        {
          /* This eCAP APWM exposes only channel 1.  The shared pwm
           * example is built for 2 channels (EPWM has A+B), so ignore
           * channel entries that aren't ours instead of failing START.
           */

          continue;
        }

      if (found)
        {
          pwmerr("ERROR: Channel 1 requested twice\n");
          return -EINVAL;
        }

      if (info->channels[i].duty > 0xffff)
        {
          pwmerr("ERROR: Duty out of range on channel %d\n", ch);
          return -EINVAL;
        }

      duty  = info->channels[i].duty;
      found = true;
    }

  if (!found)
    {
      return OK;
    }

  freq_changed = (info->frequency != priv->frequency);

  if (freq_changed)
    {
      int ret = am67_ecap_calculate_period(info->frequency, &period);

      if (ret < 0)
        {
          pwmerr("ERROR: Cannot produce %" PRIu32 " Hz\n", info->frequency);
          return ret;
        }
    }
  else
    {
      period = priv->period;
    }

  /* period is 32-bit, so duty * period overflows 32 bits - use 64-bit. */

  compare = (uint32_t)(((uint64_t)duty * period) >> 16);

  if (freq_changed)
    {
      am67_ecap_config_apwm(priv->base);
      am67_ecap_counter_freeze(priv->base);
      am67_ecap_write_immediate(priv->base, period - 1u, compare);
      am67_ecap_reset_counter(priv->base);
      am67_ecap_counter_run(priv->base);

      priv->frequency = info->frequency;
      priv->period    = period;
    }
  else
    {
      am67_ecap_write_shadow(priv->base, period - 1u, compare);
    }

  return OK;
}

/****************************************************************************
 * Name: am67_ecap_stop
 *
 * Description:
 *   Park the output low and clear the frequency cache so the next start()
 *   reconfigures.  The counter stays running - that keeps the parked level
 *   deterministic.
 *
 * Returned Value:
 *   Zero (OK) always.
 *
 ****************************************************************************/

static int am67_ecap_stop(struct pwm_lowerhalf_s *dev)
{
  struct am67_ecap_s *priv = (struct am67_ecap_s *)dev;

  am67_ecap_park(priv);
  priv->frequency = 0;

  return OK;
}

/****************************************************************************
 * Name: am67_ecap_ioctl
 *
 * Description:
 *   No platform-specific ioctl commands are supported.
 *
 * Returned Value:
 *   -ENOTTY always.
 *
 ****************************************************************************/

static int am67_ecap_ioctl(struct pwm_lowerhalf_s *dev,
                           int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_ecap_init
 *
 * Description:
 *   Boot-time preparation: unlock MAIN_CTRL_MMR partition 1, mirroring
 *   am67_epwm_init.  All eCAP register access is still deferred to setup()
 *   on open.  Must run before pwm_register().
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

int am67_ecap_init(void)
{
  return am67_ecap_enable_register_write();
}

/****************************************************************************
 * Name: am67_ecapinitialize
 *
 * Description:
 *   Return the lower-half instance for the given module so the board bringup
 *   can bind it with pwm_register().  No hardware is touched.
 *
 * Input Parameters:
 *   ecap - eCAP module number: 0, 1 or 2.
 *
 * Returned Value:
 *   Lower-half pointer on success; NULL on an unsupported or unconfigured
 *   module.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *am67_ecapinitialize(int ecap)
{
  switch (ecap)
    {
#ifdef CONFIG_AM67_ECAP0
      case 0:
        pwminfo("Initialize eCAP%d (APWM)\n", ecap);
        return (struct pwm_lowerhalf_s *)&g_am67_ecap0;
#endif

#ifdef CONFIG_AM67_ECAP1
      case 1:
        pwminfo("Initialize eCAP%d (APWM)\n", ecap);
        return (struct pwm_lowerhalf_s *)&g_am67_ecap1;
#endif

#ifdef CONFIG_AM67_ECAP2
      case 2:
        pwminfo("Initialize eCAP%d (APWM)\n", ecap);
        return (struct pwm_lowerhalf_s *)&g_am67_ecap2;
#endif

      default:
        pwmerr("ERROR: No such eCAP module: %d\n", ecap);
        return NULL;
    }
}

#endif /* CONFIG_AM67_ECAP0 || CONFIG_AM67_ECAP1 || CONFIG_AM67_ECAP2 */
