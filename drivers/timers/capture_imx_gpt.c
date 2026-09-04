/****************************************************************************
 * drivers/timers/capture_imx_gpt.c
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

#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/capture.h>
#include <nuttx/timers/imx_gpt.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int imx_gpt_cap_start(FAR struct cap_lowerhalf_s *lower);
static int imx_gpt_cap_stop(FAR struct cap_lowerhalf_s *lower);
static int imx_gpt_cap_getduty(FAR struct cap_lowerhalf_s *lower,
             FAR uint8_t *duty);
static int imx_gpt_cap_getfreq(FAR struct cap_lowerhalf_s *lower,
             FAR uint32_t *freq);
static int imx_gpt_cap_getedges(FAR struct cap_lowerhalf_s *lower,
             FAR uint32_t *edges);
#ifdef CONFIG_CAPTURE_NOTIFY
static int imx_gpt_cap_bind(FAR struct cap_lowerhalf_s *lower,
             enum cap_type_e type, capture_notify_t cb, FAR void *priv);
static int imx_gpt_cap_unbind(FAR struct cap_lowerhalf_s *lower);
#endif
static int imx_gpt_cap_start_capture(FAR struct cap_lowerhalf_s *lower);
static int imx_gpt_cap_stop_capture(FAR struct cap_lowerhalf_s *lower);
static int imx_gpt_cap_get_input_state(FAR struct cap_lowerhalf_s *lower,
             FAR uint8_t *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct cap_ops_s g_imx_gpt_cap_ops =
{
  .start           = imx_gpt_cap_start,
  .stop            = imx_gpt_cap_stop,
  .getduty         = imx_gpt_cap_getduty,
  .getfreq         = imx_gpt_cap_getfreq,
  .getedges        = imx_gpt_cap_getedges,
#ifdef CONFIG_CAPTURE_NOTIFY
  .bind            = imx_gpt_cap_bind,
  .unbind          = imx_gpt_cap_unbind,
#endif
  .start_capture   = imx_gpt_cap_start_capture,
  .stop_capture    = imx_gpt_cap_stop_capture,
  .get_input_state = imx_gpt_cap_get_input_state,
  .ioctl           = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_gpt_cap_enable
 *
 * Description:
 *   Reset the per-channel measurement state, program the input-capture
 *   mode to both edges and enable this channel's capture interrupt.
 *   Must be called with dev->lock held.
 *
 ****************************************************************************/

static void imx_gpt_cap_enable(FAR struct imx_gpt_icu_s *priv)
{
  FAR struct imx_gpt_dev_s *dev = priv->dev;

  priv->running   = true;
  priv->have_rise = false;
  priv->level     = 0;
  priv->period    = 0;
  priv->high_time = 0;
  priv->edges     = 0;

  /* Capture on both edges so frequency and duty can be derived. */

  imx_gpt_modreg(dev, IMX_GPT_CR, GPT_CR_IM_MASK(priv->ch),
                 GPT_IM_BOTH << GPT_CR_IM_SHIFT(priv->ch));
  imx_gpt_modreg(dev, IMX_GPT_IR, 0, GPT_IR_IF(priv->ch));
}

/****************************************************************************
 * Name: imx_gpt_cap_disable
 *
 * Description:
 *   Disable this channel's capture interrupt and input-capture mode.
 *   Must be called with dev->lock held.
 *
 ****************************************************************************/

static void imx_gpt_cap_disable(FAR struct imx_gpt_icu_s *priv)
{
  FAR struct imx_gpt_dev_s *dev = priv->dev;

  priv->running = false;
  imx_gpt_modreg(dev, IMX_GPT_IR, GPT_IR_IF(priv->ch), 0);
  imx_gpt_modreg(dev, IMX_GPT_CR, GPT_CR_IM_MASK(priv->ch),
                 GPT_IM_DISABLE << GPT_CR_IM_SHIFT(priv->ch));
}

/****************************************************************************
 * Name: imx_gpt_cap_start / imx_gpt_cap_stop
 ****************************************************************************/

static int imx_gpt_cap_start(FAR struct cap_lowerhalf_s *lower)
{
  FAR struct imx_gpt_icu_s *priv = (FAR struct imx_gpt_icu_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->dev->lock);
  imx_gpt_cap_enable(priv);
  spin_unlock_irqrestore(&priv->dev->lock, flags);
  return OK;
}

static int imx_gpt_cap_stop(FAR struct cap_lowerhalf_s *lower)
{
  FAR struct imx_gpt_icu_s *priv = (FAR struct imx_gpt_icu_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->dev->lock);
  imx_gpt_cap_disable(priv);
  spin_unlock_irqrestore(&priv->dev->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: imx_gpt_cap_getduty / getfreq / getedges
 ****************************************************************************/

static int imx_gpt_cap_getduty(FAR struct cap_lowerhalf_s *lower,
                               FAR uint8_t *duty)
{
  FAR struct imx_gpt_icu_s *priv = (FAR struct imx_gpt_icu_s *)lower;
  irqstate_t flags;
  uint32_t period;
  uint32_t high;

  flags = spin_lock_irqsave(&priv->dev->lock);
  period = priv->period;
  high   = priv->high_time;
  spin_unlock_irqrestore(&priv->dev->lock, flags);

  *duty = (period != 0) ? (uint8_t)(((uint64_t)high * 100) / period) : 0;
  return OK;
}

static int imx_gpt_cap_getfreq(FAR struct cap_lowerhalf_s *lower,
                               FAR uint32_t *freq)
{
  FAR struct imx_gpt_icu_s *priv = (FAR struct imx_gpt_icu_s *)lower;
  irqstate_t flags;
  uint32_t period;

  flags = spin_lock_irqsave(&priv->dev->lock);
  period = priv->period;
  spin_unlock_irqrestore(&priv->dev->lock, flags);

  /* Signal frequency = tick clock / ticks-per-period. */

  *freq = (period != 0) ? (priv->dev->freq / period) : 0;
  return OK;
}

static int imx_gpt_cap_getedges(FAR struct cap_lowerhalf_s *lower,
                                FAR uint32_t *edges)
{
  FAR struct imx_gpt_icu_s *priv = (FAR struct imx_gpt_icu_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->dev->lock);
  *edges = priv->edges;
  spin_unlock_irqrestore(&priv->dev->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: imx_gpt_cap_bind / imx_gpt_cap_unbind
 ****************************************************************************/

#ifdef CONFIG_CAPTURE_NOTIFY
static int imx_gpt_cap_bind(FAR struct cap_lowerhalf_s *lower,
                            enum cap_type_e type, capture_notify_t cb,
                            FAR void *priv_arg)
{
  FAR struct imx_gpt_icu_s *priv = (FAR struct imx_gpt_icu_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->dev->lock);

  priv->cb   = cb;
  priv->priv = priv_arg;
  priv->type = type;

  /* Ensure edge capture is active so the callback can fire even if start()
   * has not been issued separately.
   */

  if (!priv->running)
    {
      imx_gpt_cap_enable(priv);
    }

  spin_unlock_irqrestore(&priv->dev->lock, flags);
  return OK;
}

static int imx_gpt_cap_unbind(FAR struct cap_lowerhalf_s *lower)
{
  FAR struct imx_gpt_icu_s *priv = (FAR struct imx_gpt_icu_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->dev->lock);
  priv->cb   = NULL;
  priv->priv = NULL;
  spin_unlock_irqrestore(&priv->dev->lock, flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: imx_gpt_cap_start_capture / stop_capture / get_input_state
 ****************************************************************************/

static int imx_gpt_cap_start_capture(FAR struct cap_lowerhalf_s *lower)
{
  return imx_gpt_cap_start(lower);
}

static int imx_gpt_cap_stop_capture(FAR struct cap_lowerhalf_s *lower)
{
  return imx_gpt_cap_stop(lower);
}

static int imx_gpt_cap_get_input_state(FAR struct cap_lowerhalf_s *lower,
                                       FAR uint8_t *state)
{
  FAR struct imx_gpt_icu_s *priv = (FAR struct imx_gpt_icu_s *)lower;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->dev->lock);
  *state = (priv->level != 0) ? CAP_STATE_ACTIVE : CAP_STATE_IDLE;
  spin_unlock_irqrestore(&priv->dev->lock, flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_gpt_icu_on_edge
 *
 * Description:
 *   Edge-capture interrupt handler for one ICU channel.  Invoked from the
 *   shared core ISR with dev->lock already held.  Reads the latched capture
 *   register, updates the period / high-time / edge-count derivations and
 *   invokes the notify callback subject to the configured edge filter.
 *
 ****************************************************************************/

void imx_gpt_icu_on_edge(FAR struct imx_gpt_icu_s *icu)
{
  FAR struct imx_gpt_dev_s *dev = icu->dev;
  uint32_t t;
  uint8_t newlevel;

  /* The capture register holds the counter value latched at the edge. */

  t = imx_gpt_getreg(dev, IMX_GPT_ICR(icu->ch));

  /* Both-edge capture: infer direction by toggling the tracked level,
   * starting from the idle (low) assumption made at enable time.
   */

  newlevel = icu->level ? 0 : 1;

  if (newlevel != 0)
    {
      /* Rising edge: close the previous period (32-bit wrap-safe delta). */

      if (icu->have_rise)
        {
          icu->period = t - icu->last_rise;
        }

      icu->last_rise = t;
      icu->have_rise = true;
    }
  else
    {
      /* Falling edge: high-level duration within the current period. */

      if (icu->have_rise)
        {
          icu->high_time = t - icu->last_rise;
        }
    }

  icu->level = newlevel;
  icu->edges++;

#ifdef CONFIG_CAPTURE_NOTIFY
  if (icu->cb != NULL &&
      (icu->type == CAP_TYPE_BOTH ||
       (icu->type == CAP_TYPE_RISING && newlevel != 0) ||
       (icu->type == CAP_TYPE_FALLING && newlevel == 0)))
    {
      icu->cb(&icu->lower, icu->priv);
    }
#endif
}

/****************************************************************************
 * Name: imx_gpt_icu_register
 *
 * Description:
 *   Allocate, initialise and register one ICU channel as an /dev/capN
 *   character device.  Called by the shared core during initialisation.
 *
 ****************************************************************************/

FAR struct imx_gpt_icu_s *
imx_gpt_icu_register(FAR struct imx_gpt_dev_s *dev, int ch,
                     FAR const char *devpath)
{
  FAR struct imx_gpt_icu_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->lower.ops = &g_imx_gpt_cap_ops;
  priv->dev       = dev;
  priv->ch        = (uint8_t)ch;

  ret = cap_register(devpath, &priv->lower);
  if (ret < 0)
    {
      tmrerr("ERROR: cap_register(%s) failed: %d\n", devpath, ret);
      kmm_free(priv);
      return NULL;
    }

  return priv;
}
