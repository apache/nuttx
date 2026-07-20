/****************************************************************************
 * arch/arm/src/common/ameba/ameba_pwm.c
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

/* NuttX PWM lower half for the Realtek Ameba PWM timer.  The chip has a
 * single PWM-capable timer (TIM8 on amebadplus) whose one time base feeds
 * several compare channels (CCR0..CCR7).  Because the time base is shared,
 * every channel runs at the same frequency and each carries its own duty
 * cycle; the driver therefore exposes one /dev/pwm0 device that programs the
 * frequency once and walks info->channels[] for the per-channel duties (the
 * CONFIG_PWM_NCHANNELS multi-channel form of the upper half).
 *
 * The timer is driven through the SDK fwlib RTIM API.  The time-base setup
 * (RTIM_TimeBaseStructInit/RTIM_TimeBaseInit/RTIM_Cmd) resolves to the
 * on-chip ROM symbol table (those are _LONG_CALL_ entries), while the
 * capture/compare and period helpers (RTIM_CCStructInit/RTIM_CCxInit/
 * RTIM_CCRxSet/RTIM_CCxCmd/RTIM_ChangePeriod/RTIM_PrescalerConfig) are
 * compiled from the fwlib RAM source ameba_tim.c and linked in.  Everything
 * runs in task context (RTIM_CCxInit busy-waits on the update-generated
 * flag), never from an ISR.
 *
 * The frequency divider chain is f_out = CLKFREQ / ((PSC + 1) * (ARR + 1))
 * with 16-bit PSC and ARR; the driver picks the smallest prescaler that lets
 * the auto-reload value fit 16 bits, maximising duty resolution.  The duty
 * is the upper half's ub16_t fraction: CCR = duty * (ARR + 1) with the SDK
 * convention CCR = 0 -> 0 %, CCR >= ARR -> 100 %.
 *
 * The chip-specific wiring (timer index, channel count, register base, input
 * clock, clock masks and pad-mux base) lives in the per-chip
 * ameba_pwm_chip.h.  To keep the vendor headers out of the NuttX include
 * world, the few fwlib symbols and the two RTIM init-struct layouts used
 * here are declared locally rather than pulled in from <ameba_pwmtimer.h>.
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <debug.h>
#include <inttypes.h>

#include <nuttx/kmalloc.h>
#include <nuttx/timers/pwm.h>

#include "ameba_pwm.h"
#include "ameba_pwm_chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The timer index, channel count, register base, input clock, peripheral-
 * clock masks, IRQ and pad-mux base come from the per-chip ameba_pwm_chip.h.
 * Everything below is common to every current Ameba chip.
 */

/* Second/third argument to fwlib "state" style APIs. */

#define AMEBA_DISABLE               0x0
#define AMEBA_ENABLE                0x1

/* fwlib RTIM capture/compare field values (from ameba_pwmtimer.h). */

#define AMEBA_TIM_CCMODE_PWM        0x00000000  /* TIM_CCMode_PWM          */
#define AMEBA_TIM_CCPOL_HIGH        0x00000000  /* TIM_CCPolarity_High     */
#define AMEBA_TIM_CCPOL_LOW         0x04000000  /* TIM_CCPolarity_Low      */
#define AMEBA_TIM_OCPRELOAD_ENABLE  0x02000000  /* TIM_OCPreload_Enable    */
#define AMEBA_TIM_CCX_ENABLE        0x01000000  /* TIM_CCx_Enable          */
#define AMEBA_TIM_CCX_DISABLE       0x00000000  /* TIM_CCx_Disable         */
#define AMEBA_TIM_PSCRELOAD_UPDATE  0x00000000  /* TIM_PSCReloadMode_Update */

/* PSC and ARR are 16-bit on the PWM timer. */

#define AMEBA_TIM_MAX16             0xffff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Layout-compatible mirror of the fwlib RTIM_TimeBaseInitTypeDef (same field
 * order and types); passed by address to RTIM_TimeBaseStructInit()/
 * RTIM_TimeBaseInit().
 */

struct ameba_tim_timebase_s
{
  uint32_t prescaler;          /* TIM_Prescaler    */
  uint32_t period;             /* TIM_Period       */
  uint32_t updateevent;        /* TIM_UpdateEvent  */
  uint32_t updatesource;       /* TIM_UpdateSource */
  uint32_t arrprotection;      /* TIM_ARRProtection */
  uint8_t  idx;                /* TIM_Idx          */
  uint32_t securetimer;        /* TIM_SecureTimer  */
};

/* Layout-compatible mirror of the fwlib TIM_CCInitTypeDef (same field
 * order); passed by address to RTIM_CCStructInit()/RTIM_CCxInit().
 */

struct ameba_tim_cc_s
{
  uint32_t ccmode;             /* TIM_CCMode       */
  uint32_t ccpolarity;         /* TIM_CCPolarity   */
  uint32_t ocprotection;       /* TIM_OCProtection */
  uint32_t ocpulse;            /* TIM_OCPulse      */
  uint32_t icpulsemode;        /* TIM_ICPulseMode  */
};

struct ameba_pwm_dev_s
{
  const struct pwm_ops_s *ops; /* PWM lower half (must be first) */
  uintptr_t base;              /* Non-secure PWM timer register base */
  uint32_t  periph;            /* APBPeriph function mask (RCC arg 1) */
  uint32_t  clk;               /* APBPeriph clock mask (RCC arg 2) */
  uint32_t  clkfreq;           /* Timer input clock (Hz) */
  uint32_t  frequency;         /* Currently programmed frequency (Hz) */
  uint32_t  arr;               /* Currently programmed auto-reload value */
  uint32_t  psc;               /* Currently programmed prescaler */
  uint8_t   idx;               /* Timer index (TIM_Idx) */
  uint8_t   nchan;             /* Wired hardware channel count */
  uint8_t   chanen;            /* Bitmask of channels enabled since start */
  bool      running;           /* Time base is running */

  /* Output pad per hardware channel, AMEBA_PWM_PIN_NC when unused. */

  uint8_t   pins[AMEBA_PWM_NCHAN];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SDK fwlib RTIM/pin/clock API (ROM for the time base, RAM ameba_tim.c for
 * the compare/period helpers).  The RTIM_TypeDef pointer is passed as the
 * raw register base cast to void *.
 */

extern void RCC_PeriphClockCmd(uint32_t periph, uint32_t clock,
                               uint8_t newstate);
extern void Pinmux_Config(uint8_t pin, uint32_t func);
extern void RTIM_TimeBaseStructInit(struct ameba_tim_timebase_s *init);
extern void RTIM_TimeBaseInit(void *timx,
                              struct ameba_tim_timebase_s *init,
                              int irqnum, void *usercb, uint32_t cbdata);
extern void RTIM_PrescalerConfig(void *timx, uint32_t prescaler,
                                 uint32_t reloadmode);
extern void RTIM_ChangePeriod(void *timx, uint32_t autoreload);
extern void RTIM_Cmd(void *timx, uint32_t newstate);
extern void RTIM_CCStructInit(struct ameba_tim_cc_s *init);
extern void RTIM_CCxInit(void *timx, struct ameba_tim_cc_s *init,
                         uint16_t channel);
extern void RTIM_CCRxSet(void *timx, uint32_t compare, uint16_t channel);
extern void RTIM_CCxCmd(void *timx, uint16_t channel, uint32_t state);

/* PWM lower-half operations. */

static int ameba_pwm_setup(struct pwm_lowerhalf_s *dev);
static int ameba_pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int ameba_pwm_start(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info);
static int ameba_pwm_stop(struct pwm_lowerhalf_s *dev);
static int ameba_pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pwm_ops_s g_ameba_pwm_ops =
{
  .setup    = ameba_pwm_setup,
  .shutdown = ameba_pwm_shutdown,
  .start    = ameba_pwm_start,
  .stop     = ameba_pwm_stop,
  .ioctl    = ameba_pwm_ioctl,
};

/* Crossbar pad-mux function code per channel, from the chip header.  Indexed
 * by channel number minus one; the driver only indexes it, never computes a
 * code, so a chip with non-contiguous codes is handled by its header alone.
 */

static const uint8_t g_pwm_fids[AMEBA_PWM_NCHAN] = AMEBA_PWM_PINMUX_FIDS;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_pwm_timebase
 *
 * Description:
 *   Pick the smallest prescaler whose auto-reload value fits 16 bits for the
 *   requested frequency, so f_out = clkfreq / ((PSC + 1) * (ARR + 1)) is as
 *   close as possible while keeping the most duty resolution.
 *
 ****************************************************************************/

static void ameba_pwm_timebase(uint32_t clkfreq, uint32_t frequency,
                               uint32_t *psc, uint32_t *arr)
{
  uint32_t ticks;
  uint32_t p;
  uint32_t a;

  /* ticks = (PSC + 1) * (ARR + 1), the total divider needed. */

  ticks = clkfreq / frequency;
  if (ticks == 0)
    {
      ticks = 1;
    }

  /* Smallest prescaler that brings ARR within the 16-bit range. */

  p = (ticks - 1) >> 16;
  if (p > AMEBA_TIM_MAX16)
    {
      p = AMEBA_TIM_MAX16;
    }

  a = ticks / (p + 1);
  if (a == 0)
    {
      a = 1;
    }

  a -= 1;
  if (a > AMEBA_TIM_MAX16)
    {
      a = AMEBA_TIM_MAX16;
    }

  *psc = p;
  *arr = a;
}

/****************************************************************************
 * Name: ameba_pwm_duty2ccr
 *
 * Description:
 *   Convert the upper half's ub16_t duty fraction to a compare value:
 *   CCR = duty * (ARR + 1), clamped so CCR = 0 gives 0 % and CCR = ARR gives
 *   100 % (the SDK treats CCR >= period as full scale).
 *
 ****************************************************************************/

static uint32_t ameba_pwm_duty2ccr(ub16_t duty, uint32_t arr)
{
  uint32_t ccr = ((uint64_t)duty * (arr + 1)) >> 16;

  if (ccr > arr)
    {
      ccr = arr;
    }

  return ccr;
}

/****************************************************************************
 * Name: ameba_pwm_setperiod
 *
 * Description:
 *   Program the shared time base for the requested frequency, creating it on
 *   the first start and hot-updating the prescaler/period afterwards.
 *
 ****************************************************************************/

static void ameba_pwm_setperiod(struct ameba_pwm_dev_s *priv,
                                uint32_t frequency)
{
  struct ameba_tim_timebase_s tb;
  uint32_t psc;
  uint32_t arr;

  ameba_pwm_timebase(priv->clkfreq, frequency, &psc, &arr);

  if (!priv->running)
    {
      memset(&tb, 0, sizeof(tb));
      RTIM_TimeBaseStructInit(&tb);

      tb.prescaler     = psc;
      tb.period        = arr;
      tb.idx           = priv->idx;
      tb.arrprotection = AMEBA_ENABLE;

      RTIM_TimeBaseInit((void *)priv->base, &tb, AMEBA_PWM_IRQ, NULL,
                        (uint32_t)(uintptr_t)&tb);
      RTIM_PrescalerConfig((void *)priv->base, psc,
                           AMEBA_TIM_PSCRELOAD_UPDATE);
    }
  else if (frequency != priv->frequency)
    {
      /* Hot update: the auto-reload protection defers both changes to the
       * next update event so the running output does not glitch.
       */

      RTIM_PrescalerConfig((void *)priv->base, psc,
                           AMEBA_TIM_PSCRELOAD_UPDATE);
      RTIM_ChangePeriod((void *)priv->base, arr);
    }

  priv->psc       = psc;
  priv->arr       = arr;
  priv->frequency = frequency;
}

/****************************************************************************
 * Name: ameba_pwm_setchannel
 *
 * Description:
 *   Program one compare channel for its duty cycle.  A channel that has not
 *   run since the last stop is fully initialised (PWM mode, active-high,
 *   preload) and enabled; an already-running channel just has its compare
 *   value updated (deferred to the next update event by the preload).
 *
 ****************************************************************************/

static void ameba_pwm_setchannel(struct ameba_pwm_dev_s *priv,
                                 uint8_t hwch, uint32_t ccr)
{
  struct ameba_tim_cc_s cc;

  if ((priv->chanen & (1 << hwch)) == 0)
    {
      memset(&cc, 0, sizeof(cc));
      RTIM_CCStructInit(&cc);

      cc.ccmode       = AMEBA_TIM_CCMODE_PWM;
      cc.ccpolarity   = AMEBA_TIM_CCPOL_HIGH;
      cc.ocprotection = AMEBA_TIM_OCPRELOAD_ENABLE;
      cc.ocpulse      = ccr;

      RTIM_CCxInit((void *)priv->base, &cc, hwch);
      RTIM_CCxCmd((void *)priv->base, hwch, AMEBA_TIM_CCX_ENABLE);
      priv->chanen |= (1 << hwch);
    }
  else
    {
      RTIM_CCRxSet((void *)priv->base, ccr, hwch);
    }
}

/****************************************************************************
 * Name: ameba_pwm_setup
 ****************************************************************************/

static int ameba_pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct ameba_pwm_dev_s *priv = (struct ameba_pwm_dev_s *)dev;

  /* The peripheral clock and pad mux are latched at registration; nothing to
   * do until the first start().
   */

  UNUSED(priv);
  return OK;
}

/****************************************************************************
 * Name: ameba_pwm_shutdown
 ****************************************************************************/

static int ameba_pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  return ameba_pwm_stop(dev);
}

/****************************************************************************
 * Name: ameba_pwm_start
 *
 * Description:
 *   Program the shared frequency and every requested channel's duty, then
 *   run the time base.  info->channels[] carries 1-based channel numbers;
 *   entries that are out of range or map to an unwired pad are skipped.
 *
 ****************************************************************************/

static int ameba_pwm_start(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info)
{
  struct ameba_pwm_dev_s *priv = (struct ameba_pwm_dev_s *)dev;
  int i;

  if (info->frequency == 0 || info->frequency > priv->clkfreq)
    {
      pwmerr("ERROR: frequency %" PRIu32 " out of range\n",
             info->frequency);
      return -ERANGE;
    }

  ameba_pwm_setperiod(priv, info->frequency);

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      int8_t   ch = info->channels[i].channel;
      uint8_t  hwch;
      uint32_t ccr;

      if (ch < 1 || ch > priv->nchan)
        {
          continue;
        }

      hwch = (uint8_t)(ch - 1);
      if (priv->pins[hwch] == AMEBA_PWM_PIN_NC)
        {
          pwmwarn("WARNING: channel %d has no pad wired\n", ch);
          continue;
        }

      ccr = ameba_pwm_duty2ccr(info->channels[i].duty, priv->arr);
      ameba_pwm_setchannel(priv, hwch, ccr);

      pwminfo("channel %d: duty=%08" PRIx32 " ccr=%" PRIu32 "/%" PRIu32 "\n",
              ch, (uint32_t)info->channels[i].duty, ccr, priv->arr);
    }

  if (!priv->running)
    {
      RTIM_Cmd((void *)priv->base, AMEBA_ENABLE);
      priv->running = true;
    }

  return OK;
}

/****************************************************************************
 * Name: ameba_pwm_stop
 ****************************************************************************/

static int ameba_pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct ameba_pwm_dev_s *priv = (struct ameba_pwm_dev_s *)dev;
  int i;

  for (i = 0; i < priv->nchan; i++)
    {
      if (priv->chanen & (1 << i))
        {
          RTIM_CCxCmd((void *)priv->base, i, AMEBA_TIM_CCX_DISABLE);
        }
    }

  RTIM_Cmd((void *)priv->base, AMEBA_DISABLE);

  priv->chanen    = 0;
  priv->running   = false;
  priv->frequency = 0;
  return OK;
}

/****************************************************************************
 * Name: ameba_pwm_ioctl
 ****************************************************************************/

static int ameba_pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                           unsigned long arg)
{
  UNUSED(dev);
  UNUSED(cmd);
  UNUSED(arg);
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_pwm_register
 *
 * Description:
 *   See ameba_pwm.h.
 *
 ****************************************************************************/

int ameba_pwm_register(const char *path, const uint8_t *pins,
                       unsigned int npins)
{
  struct ameba_pwm_dev_s *priv;
  unsigned int i;
  int ret;

  priv = kmm_zalloc(sizeof(struct ameba_pwm_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->ops     = &g_ameba_pwm_ops;
  priv->base    = AMEBA_PWM_BASE;
  priv->periph  = AMEBA_PWM_APBPERIPH;
  priv->clk     = AMEBA_PWM_APBPERIPH_CLK;
  priv->clkfreq = AMEBA_PWM_CLKFREQ;
  priv->idx     = AMEBA_PWM_TIMER_IDX;

  if (npins > AMEBA_PWM_NCHAN)
    {
      npins = AMEBA_PWM_NCHAN;
    }

  priv->nchan = (uint8_t)npins;
  memset(priv->pins, AMEBA_PWM_PIN_NC, sizeof(priv->pins));

  /* Gate the PWM peripheral clock and route each wired channel to its pad
   * through the crossbar.  The per-channel function code comes from the
   * chip header table (g_pwm_fids), never computed, so a chip whose codes
   * are not contiguous (a single shared code, or grouped per timer) is
   * expressed by its header alone without touching this driver.
   */

  RCC_PeriphClockCmd(priv->periph, priv->clk, AMEBA_ENABLE);

  for (i = 0; i < npins; i++)
    {
      priv->pins[i] = pins[i];
      if (pins[i] != AMEBA_PWM_PIN_NC)
        {
          Pinmux_Config(pins[i], g_pwm_fids[i]);
        }
    }

  ret = pwm_register(path, (struct pwm_lowerhalf_s *)priv);
  if (ret < 0)
    {
      _err("ERROR: pwm_register(%s) failed: %d\n", path, ret);
      kmm_free(priv);
      return ret;
    }

  return OK;
}
