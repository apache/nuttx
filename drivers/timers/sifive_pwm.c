/****************************************************************************
 * drivers/timers/sifive_pwm.c
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
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/timers/sifive_pwm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets of the SiFive PWM block ("sifive,pwm0"). */

#define SIFIVE_PWM_CFG        0x00    /* Configuration                     */
#define SIFIVE_PWM_COUNT      0x08    /* Counter (31-bit, self-increments) */
#define SIFIVE_PWM_S          0x10    /* Scaled counter, compared to CMPx  */
#define SIFIVE_PWM_CMP(n)     (0x20 + ((n) << 2))

/* PWMCFG bit fields (only the ones this driver programs). */

#define PWM_CFG_SCALE_MASK    0x0000000fu  /* [3:0]  counter prescaler     */
#define PWM_CFG_ZEROCMP       (1u << 9)    /* PWMCMP0 acts as the period   */
#define PWM_CFG_ENALWAYS      (1u << 12)   /* Counter runs continuously    */
#define PWM_CFG_ENONESHOT     (1u << 13)   /* Counter runs once            */

/* PWMCMPx is 16 bits wide, so the counter wraps after PWM_CMP_FULL scaled
 * ticks when ZEROCMP is left clear (the mode used here).
 */

#define PWM_CMP_WIDTH         16
#define PWM_CMP_FULL          (1u << PWM_CMP_WIDTH)   /* 65536             */
#define PWM_CMP_MASK          (PWM_CMP_FULL - 1)      /* 0xffff            */

#define PWM_CFG_SCALE_MAX     PWM_CFG_SCALE_MASK

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sifive_pwm_lowerhalf_s
{
  /* The PWM upper half expects this to be the first member. */

  FAR const struct pwm_ops_s *ops;

  uintptr_t base;        /* MMIO register base                            */
  uint32_t  freq_hz;     /* Reference clock feeding the counter           */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sifive_pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int sifive_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);
static int sifive_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                            FAR const struct pwm_info_s *info);
static int sifive_pwm_stop(FAR struct pwm_lowerhalf_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pwm_ops_s g_pwmops =
{
  sifive_pwm_setup,     /* setup    */
  sifive_pwm_shutdown,  /* shutdown */
  sifive_pwm_start,     /* start    */
  sifive_pwm_stop       /* stop     */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Direct MMIO accessors.  Kept local so the driver is architecture neutral
 * (armv8-a / armv8-r / RISC-V) and does not depend on any chip-specific
 * getreg macro, matching drivers/timers/sp804.h and drivers/ioexpander.
 */

static inline uint32_t sifive_pwm_getreg(uintptr_t base, unsigned int off)
{
  return *(FAR volatile uint32_t *)(base + off);
}

static inline void sifive_pwm_putreg(uintptr_t base, unsigned int off,
                                     uint32_t val)
{
  *(FAR volatile uint32_t *)(base + off) = val;
}

/****************************************************************************
 * Name: sifive_pwm_scale
 *
 * Description:
 *   Pick the PWMCFG.SCALE prescaler whose 65536-tick period is closest to
 *   (but not shorter than) the requested pulse period.  With SCALE = s the
 *   output frequency is freq_hz / (1 << (16 + s)), so the achievable set is
 *   quantised to powers of two - the mainline Linux pwm-sifive driver
 *   approximates the period the same way.
 *
 * Input Parameters:
 *   freq_hz   - Reference clock in Hz
 *   frequency - Requested output frequency in Hz
 *
 * Returned Value:
 *   SCALE field value in [0, 15].
 *
 ****************************************************************************/

static uint32_t sifive_pwm_scale(uint32_t freq_hz, uint32_t frequency)
{
  uint32_t ticks;
  uint32_t scale;

  /* Counter ticks needed for one output period. */

  ticks = freq_hz / frequency;

  /* SCALE shifts the counter right before the compare, so one period is
   * (1 << (16 + SCALE)) reference ticks.  Find the smallest SCALE whose
   * period covers the request.
   */

  for (scale = 0; scale < PWM_CFG_SCALE_MAX; scale++)
    {
      if ((PWM_CMP_FULL << scale) >= ticks)
        {
          break;
        }
    }

  return scale;
}

/****************************************************************************
 * Name: sifive_pwm_duty2cmp
 *
 * Description:
 *   Convert an upper-half duty (b16 fraction, 0..0xffff for 0..100%) into a
 *   PWMCMPx compare value.  The output is high while PWMS < PWMCMPx counts
 *   down from the wrap point, so the device reports
 *   duty = 1 - PWMCMPx / 65536: the compare value is the complement of the
 *   requested duty.  This is the same inverted-polarity mapping the QEMU
 *   model publishes on "pwm/<instance>/out/<ch>" and that mainline Linux
 *   describes as PWM_POLARITY_INVERSED.
 *
 ****************************************************************************/

static uint32_t sifive_pwm_duty2cmp(ub16_t duty)
{
  uint32_t cmp;

  if (duty >= PWM_CMP_FULL)
    {
      /* 100% duty: the compare never matches within a period. */

      return 0;
    }

  cmp = PWM_CMP_FULL - duty;

  return cmp > PWM_CMP_MASK ? PWM_CMP_MASK : cmp;
}

/****************************************************************************
 * Name: sifive_pwm_idle
 *
 * Description:
 *   Drive every channel to its idle (0% duty) compare value.  Writing the
 *   compare registers is what makes the QEMU model re-publish the duty, so
 *   this also updates any external subscriber.
 *
 ****************************************************************************/

static void sifive_pwm_idle(FAR struct sifive_pwm_lowerhalf_s *priv)
{
  int i;

  for (i = 0; i < SIFIVE_PWM_NCHANNELS; i++)
    {
      sifive_pwm_putreg(priv->base, SIFIVE_PWM_CMP(i), PWM_CMP_MASK);
    }
}

/****************************************************************************
 * Name: sifive_pwm_setup
 *
 * Description:
 *   Called when /dev/pwmN is first opened.  Put the counter in a known,
 *   stopped state with all outputs idle; pulses only start from start().
 *
 ****************************************************************************/

static int sifive_pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct sifive_pwm_lowerhalf_s *priv =
    (FAR struct sifive_pwm_lowerhalf_s *)dev;

  sifive_pwm_putreg(priv->base, SIFIVE_PWM_CFG, 0);
  sifive_pwm_putreg(priv->base, SIFIVE_PWM_COUNT, 0);
  sifive_pwm_idle(priv);

  pwminfo("base=0x%" PRIxPTR " clk=%" PRIu32 " Hz\n",
          priv->base, priv->freq_hz);

  return OK;
}

/****************************************************************************
 * Name: sifive_pwm_shutdown
 *
 * Description:
 *   Called when the last reference to /dev/pwmN is closed.  Stop the
 *   counter and leave the outputs idle.
 *
 ****************************************************************************/

static int sifive_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  return sifive_pwm_stop(dev);
}

/****************************************************************************
 * Name: sifive_pwm_start
 *
 * Description:
 *   Program the prescaler from info->frequency and one compare register per
 *   requested channel, then let the counter run.  Channel numbers are
 *   one-based (channel 1 drives PWMCMP0); a zero entry is skipped and a
 *   negative entry ends the list.
 *
 ****************************************************************************/

static int sifive_pwm_start(FAR struct pwm_lowerhalf_s *dev,
                            FAR const struct pwm_info_s *info)
{
  FAR struct sifive_pwm_lowerhalf_s *priv =
    (FAR struct sifive_pwm_lowerhalf_s *)dev;
  uint32_t frequency;
  uint32_t scale;
  int i;

#ifdef CONFIG_PWM_FREQUENCY_FIXED
  frequency = b32toi(info->frequency);
#else
  frequency = info->frequency;
#endif

  if (frequency == 0 || frequency > priv->freq_hz)
    {
      pwmerr("ERROR: frequency %" PRIu32 " Hz out of range (clk %" PRIu32
             " Hz)\n", frequency, priv->freq_hz);
      return -EINVAL;
    }

  scale = sifive_pwm_scale(priv->freq_hz, frequency);

  /* Restart the period from a known point, then enable free running. */

  sifive_pwm_putreg(priv->base, SIFIVE_PWM_CFG, 0);
  sifive_pwm_putreg(priv->base, SIFIVE_PWM_COUNT, 0);

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      int8_t channel = info->channels[i].channel;
      uint32_t cmp;

      if (channel < 0)
        {
          /* End of the channel list. */

          break;
        }

      if (channel == 0)
        {
          /* Unused entry. */

          continue;
        }

      if (channel > SIFIVE_PWM_NCHANNELS)
        {
          pwmerr("ERROR: channel %d exceeds %d\n",
                 channel, SIFIVE_PWM_NCHANNELS);
          return -EINVAL;
        }

      cmp = sifive_pwm_duty2cmp(info->channels[i].duty);
      sifive_pwm_putreg(priv->base, SIFIVE_PWM_CMP(channel - 1), cmp);

      pwminfo("channel %d: duty=0x%08" PRIx32 " -> PWMCMP%d=0x%04" PRIx32
              "\n", channel, (uint32_t)info->channels[i].duty,
              channel - 1, cmp);
    }

  sifive_pwm_putreg(priv->base, SIFIVE_PWM_CFG,
                    (scale & PWM_CFG_SCALE_MASK) | PWM_CFG_ENALWAYS);

  pwminfo("frequency=%" PRIu32 " Hz scale=%" PRIu32 " (actual %" PRIu32
          " Hz)\n", frequency, scale,
          priv->freq_hz >> (PWM_CMP_WIDTH + scale));

  return OK;
}

/****************************************************************************
 * Name: sifive_pwm_stop
 *
 * Description:
 *   Stop the counter and park every output at 0% duty.
 *
 ****************************************************************************/

static int sifive_pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct sifive_pwm_lowerhalf_s *priv =
    (FAR struct sifive_pwm_lowerhalf_s *)dev;

  /* Park the outputs first so the published duty reflects the stop, then
   * take the enable bits away.
   */

  sifive_pwm_idle(priv);
  sifive_pwm_putreg(priv->base, SIFIVE_PWM_CFG, 0);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sifive_pwm_initialize
 *
 * Description:
 *   See include/nuttx/timers/sifive_pwm.h.
 *
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *sifive_pwm_initialize(uintptr_t base,
                                                  uint32_t freq_hz)
{
  FAR struct sifive_pwm_lowerhalf_s *priv;

  if (base == 0)
    {
      return NULL;
    }

  /* One state block per controller, allocated so that any number of
   * controllers can be registered.
   */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->ops     = &g_pwmops;
  priv->base    = base;
  priv->freq_hz = freq_hz != 0 ? freq_hz : SIFIVE_PWM_DEFAULT_FREQ;

  return (FAR struct pwm_lowerhalf_s *)priv;
}

/****************************************************************************
 * Name: sifive_pwm_uninitialize
 *
 * Description:
 *   See include/nuttx/timers/sifive_pwm.h.
 *
 ****************************************************************************/

void sifive_pwm_uninitialize(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct sifive_pwm_lowerhalf_s *priv =
    (FAR struct sifive_pwm_lowerhalf_s *)dev;

  if (priv != NULL)
    {
      kmm_free(priv);
    }
}
