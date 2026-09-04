/****************************************************************************
 * drivers/timers/ocu_imx_gpt.c
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
#include <nuttx/timers/ocu.h>
#include <nuttx/timers/imx_gpt.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  imx_gpt_ocu_setup(FAR struct ocu_lowerhalf_s *lower);
static int  imx_gpt_ocu_shutdown(FAR struct ocu_lowerhalf_s *lower);
static int  imx_gpt_ocu_start(FAR struct ocu_lowerhalf_s *lower);
static int  imx_gpt_ocu_stop(FAR struct ocu_lowerhalf_s *lower);
static int  imx_gpt_ocu_getstatus(FAR struct ocu_lowerhalf_s *lower,
              FAR struct ocu_status_s *status);
static int  imx_gpt_ocu_getcounter(FAR struct ocu_lowerhalf_s *lower,
              FAR uint32_t *counter);
static int  imx_gpt_ocu_setabsthreshold(FAR struct ocu_lowerhalf_s *lower,
              FAR struct ocu_abs_threshold_s *t);
static int  imx_gpt_ocu_setrelthreshold(FAR struct ocu_lowerhalf_s *lower,
              FAR struct ocu_rel_threshold_s *t);
static int  imx_gpt_ocu_setadvancethreshold(
              FAR struct ocu_lowerhalf_s *lower,
              FAR struct ocu_advance_threshold_s *t);
static void imx_gpt_ocu_setcallback(FAR struct ocu_lowerhalf_s *lower,
              CODE ocu_callback_t callback, FAR void *arg);
static int  imx_gpt_ocu_setpinstate(FAR struct ocu_lowerhalf_s *lower,
              enum ocu_pinstate_e state);
static int  imx_gpt_ocu_setpinaction(FAR struct ocu_lowerhalf_s *lower,
              enum ocu_pinaction_e action);
static int  imx_gpt_ocu_notification(FAR struct ocu_lowerhalf_s *lower,
              bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ocu_ops_s g_imx_gpt_ocu_ops =
{
  .setup               = imx_gpt_ocu_setup,
  .shutdown            = imx_gpt_ocu_shutdown,
  .start               = imx_gpt_ocu_start,
  .stop                = imx_gpt_ocu_stop,
  .getstatus           = imx_gpt_ocu_getstatus,
  .getcounter          = imx_gpt_ocu_getcounter,
  .setabsthreshold     = imx_gpt_ocu_setabsthreshold,
  .setrelthreshold     = imx_gpt_ocu_setrelthreshold,
  .setadvancethreshold = imx_gpt_ocu_setadvancethreshold,
  .setcallback         = imx_gpt_ocu_setcallback,
  .setpinstate         = imx_gpt_ocu_setpinstate,
  .setpinaction        = imx_gpt_ocu_setpinaction,
  .notification        = imx_gpt_ocu_notification,
  .ioctl               = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_gpt_ocu_om_from_action
 *
 * Description:
 *   Map a generic OCU pin action to the GPT OMx field encoding.
 *
 ****************************************************************************/

static uint32_t imx_gpt_ocu_om_from_action(enum ocu_pinaction_e action)
{
  switch (action)
    {
      case OCU_PINACTION_TOGGLE:
        return GPT_OM_TOGGLE;

      case OCU_PINACTION_SET_LOW:
        return GPT_OM_CLEAR;

      case OCU_PINACTION_SET_HIGH:
        return GPT_OM_SET;

      case OCU_PINACTION_DISABLE:
      default:
        return GPT_OM_DISCONNECT;
    }
}

/****************************************************************************
 * Name: imx_gpt_ocu_update_ie
 *
 * Description:
 *   Recompute and apply the compare-match interrupt enable for this
 *   channel.  The interrupt is wanted only while the channel is active and
 *   either a callback is registered or user notification is enabled.
 *   Must be called with dev->lock held.
 *
 ****************************************************************************/

static void imx_gpt_ocu_update_ie(FAR struct imx_gpt_ocu_s *priv)
{
  FAR struct imx_gpt_dev_s *dev = priv->dev;
  bool want = priv->active && (priv->notify_en || priv->callback != NULL);

  if (want)
    {
      imx_gpt_modreg(dev, IMX_GPT_IR, 0, GPT_IR_OF(priv->ch));
    }
  else
    {
      imx_gpt_modreg(dev, IMX_GPT_IR, GPT_IR_OF(priv->ch), 0);
    }
}

/****************************************************************************
 * Name: imx_gpt_ocu_setup / imx_gpt_ocu_shutdown
 ****************************************************************************/

static int imx_gpt_ocu_setup(FAR struct ocu_lowerhalf_s *lower)
{
  /* The shared core already attached the interrupt; nothing to do. */

  return OK;
}

static int imx_gpt_ocu_shutdown(FAR struct ocu_lowerhalf_s *lower)
{
  return imx_gpt_ocu_stop(lower);
}

/****************************************************************************
 * Name: imx_gpt_ocu_start / imx_gpt_ocu_stop
 ****************************************************************************/

static int imx_gpt_ocu_start(FAR struct ocu_lowerhalf_s *lower)
{
  FAR struct imx_gpt_ocu_s *priv = (FAR struct imx_gpt_ocu_s *)lower;
  FAR struct imx_gpt_dev_s *dev = priv->dev;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);

  /* Enable the configured pin action; the free-running counter keeps
   * running independently.
   */

  imx_gpt_modreg(dev, IMX_GPT_CR, GPT_CR_OM_MASK(priv->ch),
                 imx_gpt_ocu_om_from_action(priv->action) <<
                 GPT_CR_OM_SHIFT(priv->ch));

  priv->active = true;
  imx_gpt_ocu_update_ie(priv);

  spin_unlock_irqrestore(&dev->lock, flags);
  return OK;
}

static int imx_gpt_ocu_stop(FAR struct ocu_lowerhalf_s *lower)
{
  FAR struct imx_gpt_ocu_s *priv = (FAR struct imx_gpt_ocu_s *)lower;
  FAR struct imx_gpt_dev_s *dev = priv->dev;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);

  priv->active = false;

  /* Disconnect the pin and disable this channel's interrupt.  The shared
   * counter is deliberately left running.
   */

  imx_gpt_modreg(dev, IMX_GPT_CR, GPT_CR_OM_MASK(priv->ch),
                 GPT_OM_DISCONNECT << GPT_CR_OM_SHIFT(priv->ch));
  imx_gpt_ocu_update_ie(priv);

  spin_unlock_irqrestore(&dev->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: imx_gpt_ocu_getstatus / imx_gpt_ocu_getcounter
 ****************************************************************************/

static int imx_gpt_ocu_getstatus(FAR struct ocu_lowerhalf_s *lower,
                                 FAR struct ocu_status_s *status)
{
  FAR struct imx_gpt_ocu_s *priv = (FAR struct imx_gpt_ocu_s *)lower;
  FAR struct imx_gpt_dev_s *dev = priv->dev;

  /* The upper-half owns ACTIVE / NOTIF_EN; the lower-half reports the
   * hardware capability (an output pin is always present per channel) and
   * the hardware-specific counter / threshold values.
   */

  status->flags      = OCUFLAGS_PIN_AVAIL;
  status->counter    = imx_gpt_getreg(dev, IMX_GPT_CNT);
  status->threshold  = priv->last_threshold;
  status->maxcounter = GPT_TIMER_MAX;
  return OK;
}

static int imx_gpt_ocu_getcounter(FAR struct ocu_lowerhalf_s *lower,
                                  FAR uint32_t *counter)
{
  FAR struct imx_gpt_ocu_s *priv = (FAR struct imx_gpt_ocu_s *)lower;

  *counter = imx_gpt_getreg(priv->dev, IMX_GPT_CNT);
  return OK;
}

/****************************************************************************
 * Name: imx_gpt_ocu_setabsthreshold / setrelthreshold / setadvancethreshold
 ****************************************************************************/

static int imx_gpt_ocu_setabsthreshold(FAR struct ocu_lowerhalf_s *lower,
                                       FAR struct ocu_abs_threshold_s *t)
{
  FAR struct imx_gpt_ocu_s *priv = (FAR struct imx_gpt_ocu_s *)lower;
  FAR struct imx_gpt_dev_s *dev = priv->dev;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);

  imx_gpt_putreg(dev, IMX_GPT_OCR(priv->ch), t->absolute);
  priv->last_threshold = t->absolute;

  spin_unlock_irqrestore(&dev->lock, flags);

  /* A free-running up-counter reaches the target within the current
   * reference interval when the target is at or ahead of the reference.
   */

  t->result = (t->absolute >= t->reference) ?
              OCU_THRESHOLD_IN_REF_INTERVAL :
              OCU_THRESHOLD_OUT_REF_INTERVAL;
  return OK;
}

static int imx_gpt_ocu_setrelthreshold(FAR struct ocu_lowerhalf_s *lower,
                                       FAR struct ocu_rel_threshold_s *t)
{
  FAR struct imx_gpt_ocu_s *priv = (FAR struct imx_gpt_ocu_s *)lower;
  FAR struct imx_gpt_dev_s *dev = priv->dev;
  irqstate_t flags;
  uint32_t counter;
  uint32_t threshold;

  flags = spin_lock_irqsave(&dev->lock);

  counter   = imx_gpt_getreg(dev, IMX_GPT_CNT);
  threshold = counter + t->relative;
  imx_gpt_putreg(dev, IMX_GPT_OCR(priv->ch), threshold);
  priv->last_threshold = threshold;

  spin_unlock_irqrestore(&dev->lock, flags);

  /* If the addition wrapped past the 32-bit maximum the match falls into
   * the next reference interval.
   */

  t->result = (threshold >= counter) ?
              OCU_THRESHOLD_IN_REF_INTERVAL :
              OCU_THRESHOLD_OUT_REF_INTERVAL;
  return OK;
}

static int imx_gpt_ocu_setadvancethreshold(FAR struct ocu_lowerhalf_s *lower,
                                       FAR struct ocu_advance_threshold_s *t)
{
  FAR struct imx_gpt_ocu_s *priv = (FAR struct imx_gpt_ocu_s *)lower;
  FAR struct imx_gpt_dev_s *dev = priv->dev;
  irqstate_t flags;
  uint32_t old_threshold;
  uint32_t threshold;

  flags = spin_lock_irqsave(&dev->lock);

  old_threshold = priv->last_threshold;
  threshold     = old_threshold + t->step;
  imx_gpt_putreg(dev, IMX_GPT_OCR(priv->ch), threshold);
  priv->last_threshold = threshold;

  spin_unlock_irqrestore(&dev->lock, flags);

  /* A wrap past the 32-bit maximum pushes the match into the next
   * reference interval.
   */

  t->result = (threshold >= old_threshold) ?
              OCU_THRESHOLD_IN_REF_INTERVAL :
              OCU_THRESHOLD_OUT_REF_INTERVAL;
  return OK;
}

/****************************************************************************
 * Name: imx_gpt_ocu_setcallback
 ****************************************************************************/

static void imx_gpt_ocu_setcallback(FAR struct ocu_lowerhalf_s *lower,
                                    CODE ocu_callback_t callback,
                                    FAR void *arg)
{
  FAR struct imx_gpt_ocu_s *priv = (FAR struct imx_gpt_ocu_s *)lower;
  FAR struct imx_gpt_dev_s *dev = priv->dev;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);

  priv->callback = callback;
  priv->cbarg    = arg;
  imx_gpt_ocu_update_ie(priv);

  spin_unlock_irqrestore(&dev->lock, flags);
}

/****************************************************************************
 * Name: imx_gpt_ocu_setpinstate / imx_gpt_ocu_setpinaction
 ****************************************************************************/

static int imx_gpt_ocu_setpinstate(FAR struct ocu_lowerhalf_s *lower,
                                   enum ocu_pinstate_e state)
{
  FAR struct imx_gpt_ocu_s *priv = (FAR struct imx_gpt_ocu_s *)lower;
  FAR struct imx_gpt_dev_s *dev = priv->dev;
  irqstate_t flags;
  uint32_t om;

  /* The upper-half only permits this while the channel is stopped.  Drive
   * the pin to the requested level by programming the matching output mode
   * and issuing a forced compare (FOx self-clears after acting).
   */

  om = (state == OCU_PINSTATE_HIGH) ? GPT_OM_SET : GPT_OM_CLEAR;

  flags = spin_lock_irqsave(&dev->lock);

  imx_gpt_modreg(dev, IMX_GPT_CR, GPT_CR_OM_MASK(priv->ch),
                 om << GPT_CR_OM_SHIFT(priv->ch));
  imx_gpt_modreg(dev, IMX_GPT_CR, 0, GPT_CR_FO(priv->ch));

  /* Restore the disconnected state so the forced level is not re-driven on
   * the next compare match while stopped.
   */

  imx_gpt_modreg(dev, IMX_GPT_CR, GPT_CR_OM_MASK(priv->ch),
                 GPT_OM_DISCONNECT << GPT_CR_OM_SHIFT(priv->ch));

  spin_unlock_irqrestore(&dev->lock, flags);
  return OK;
}

static int imx_gpt_ocu_setpinaction(FAR struct ocu_lowerhalf_s *lower,
                                    enum ocu_pinaction_e action)
{
  FAR struct imx_gpt_ocu_s *priv = (FAR struct imx_gpt_ocu_s *)lower;
  FAR struct imx_gpt_dev_s *dev = priv->dev;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);

  priv->action = action;

  /* Apply immediately if the channel is already running. */

  if (priv->active)
    {
      imx_gpt_modreg(dev, IMX_GPT_CR, GPT_CR_OM_MASK(priv->ch),
                     imx_gpt_ocu_om_from_action(action) <<
                     GPT_CR_OM_SHIFT(priv->ch));
    }

  spin_unlock_irqrestore(&dev->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: imx_gpt_ocu_notification
 ****************************************************************************/

static int imx_gpt_ocu_notification(FAR struct ocu_lowerhalf_s *lower,
                                    bool enable)
{
  FAR struct imx_gpt_ocu_s *priv = (FAR struct imx_gpt_ocu_s *)lower;
  FAR struct imx_gpt_dev_s *dev = priv->dev;
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);

  priv->notify_en = enable;
  imx_gpt_ocu_update_ie(priv);

  spin_unlock_irqrestore(&dev->lock, flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_gpt_ocu_on_match
 *
 * Description:
 *   Compare-match interrupt handler for one OCU channel.  Invoked from the
 *   shared core ISR with dev->lock already held.  Runs the registered
 *   callback and, if it asks to reload, re-arms the compare register with
 *   the returned absolute threshold.
 *
 ****************************************************************************/

void imx_gpt_ocu_on_match(FAR struct imx_gpt_ocu_s *ocu)
{
  uint32_t next = ocu->last_threshold;

  if (ocu->callback != NULL)
    {
      if (ocu->callback(&next, ocu->cbarg))
        {
          imx_gpt_putreg(ocu->dev, IMX_GPT_OCR(ocu->ch), next);
          ocu->last_threshold = next;
        }
    }
}

/****************************************************************************
 * Name: imx_gpt_ocu_register
 *
 * Description:
 *   Allocate, initialise and register one OCU channel as an /dev/ocuN
 *   character device.  Called by the shared core during initialisation.
 *
 ****************************************************************************/

FAR struct imx_gpt_ocu_s *
imx_gpt_ocu_register(FAR struct imx_gpt_dev_s *dev, int ch,
                     FAR const char *devpath)
{
  FAR struct imx_gpt_ocu_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->lower.ops      = &g_imx_gpt_ocu_ops;
  priv->dev            = dev;
  priv->ch             = (uint8_t)ch;
  priv->action         = OCU_PINACTION_TOGGLE;
  priv->last_threshold = GPT_TIMER_MAX;

  ret = ocu_register(devpath, &priv->lower);
  if (ret < 0)
    {
      tmrerr("ERROR: ocu_register(%s) failed: %d\n", devpath, ret);
      kmm_free(priv);
      return NULL;
    }

  return priv;
}
