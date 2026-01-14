/****************************************************************************
 * drivers/timers/fake_capture.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>
#include <nuttx/timers/capture.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAPTURE_DUTY_DEFAULT 50
#define CAPTURE_FREQ_DEFAULT 10

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fake_capture_s
{
  struct cap_lowerhalf_s lower;   /* Lower half capture driver */
  spinlock_t             lock;    /* For mutually exclusive access */
  struct wdog_s          wdog;    /* For timing capture events */
  bool                   high;    /* Current edge state */
  capture_notify_t       cb;      /* Capture event callback function pointer */
  FAR void              *priv;    /* Pointer to private data */
  enum cap_type_e        type;    /* Edge type for the capture channel */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int fake_capture_start(FAR struct cap_lowerhalf_s *lower);
static int fake_capture_stop(FAR struct cap_lowerhalf_s *lower);
static int fake_capture_getduty(FAR struct cap_lowerhalf_s *lower,
                                FAR uint8_t *duty);
static int fake_capture_getfreq(FAR struct cap_lowerhalf_s *lower,
                                FAR uint32_t *freq);
static int fake_capture_getedges(FAR struct cap_lowerhalf_s *lower,
                                 FAR uint32_t *edges);
static int fake_capture_bind(FAR struct cap_lowerhalf_s *lower,
                             enum cap_type_e type, capture_notify_t cb,
                             FAR void *priv);
static int fake_capture_unbind(FAR struct cap_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct cap_ops_s g_fake_cap_ops =
{
  .start     = fake_capture_start,
  .stop      = fake_capture_stop,
  .getduty   = fake_capture_getduty,
  .getfreq   = fake_capture_getfreq,
  .getedges  = fake_capture_getedges,
  .bind      = fake_capture_bind,
  .unbind    = fake_capture_unbind,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void fake_capture_isr(wdparm_t arg)
{
  FAR struct fake_capture_s *capture = (FAR struct fake_capture_s *)arg;
  irqstate_t flags;
  bool last;

  flags = spin_lock_irqsave(&capture->lock);

  last = capture->high;
  capture->high = !capture->high;

  if (capture->cb != NULL &&
      (capture->type == CAP_TYPE_BOTH ||
       (capture->type == CAP_TYPE_RISING && !last) ||
       (capture->type == CAP_TYPE_FALLING && last)))
    {
      capture->cb(&capture->lower, capture->priv);
    }

  spin_unlock_irqrestore(&capture->lock, flags);

  wd_start_next(&capture->wdog, MSEC2TICK(1000 / CAPTURE_FREQ_DEFAULT),
                fake_capture_isr, (wdparm_t)capture);
}

/****************************************************************************
 * Name: fake_capture_start
 *
 * Description:
 *   This function is a requirement of the upper-half driver. When called,
 *   enables the capture channel, interruption routine, sets the positive
 *   edge to trigger this interrupt and resets the frequency and duty
 *   values. The positive edge is always the first expected.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int fake_capture_start(FAR struct cap_lowerhalf_s *lower)
{
  FAR struct fake_capture_s *capture = (FAR struct fake_capture_s *)lower;

  wd_start(&capture->wdog, MSEC2TICK(1000 / CAPTURE_FREQ_DEFAULT),
           fake_capture_isr, (wdparm_t)capture);
  return OK;
}

/****************************************************************************
 * Name: fake_capture_stop
 *
 * Description:
 *   This function is a requirement of the upper-half driver. When called,
 *   disables the capture channel and the interrupt routine associated.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int fake_capture_stop(FAR struct cap_lowerhalf_s *lower)
{
  FAR struct fake_capture_s *capture = (FAR struct fake_capture_s *)lower;

  wd_cancel(&capture->wdog);
  return OK;
}

/****************************************************************************
 * Name: fake_capture_getduty
 *
 * Description:
 *   This function is a requirement of the upper-half driver. Returns
 *   the last calculated duty cycle value.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *   duty  - uint8_t pointer where the duty cycle value is written.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int fake_capture_getduty(FAR struct cap_lowerhalf_s *lower,
                                FAR uint8_t *duty)
{
  *duty = CAPTURE_DUTY_DEFAULT;

  return OK;
}

/****************************************************************************
 * Name: fake_capture_getfreq
 *
 * Description:
 *   This function is a requirement of the upper-half driver. Returns
 *   the last calculated frequency value.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *   duty  - uint8_t pointer where the frequency value is written.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int fake_capture_getfreq(FAR struct cap_lowerhalf_s *lower,
                                FAR uint32_t *freq)
{
  *freq = CAPTURE_FREQ_DEFAULT;

  return OK;
}

/****************************************************************************
 * Name: fake_capture_getedges
 *
 * Description:
 *   This function is a requirement of the upper-half driver. Returns
 *   the current edge type configured for the capture channel.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *   type  - Pointer to the edge type variable to be updated.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int fake_capture_getedges(FAR struct cap_lowerhalf_s *lower,
                                 FAR uint32_t *edges)
{
  FAR struct fake_capture_s *capture = (FAR struct fake_capture_s *)lower;
  irqstate_t flags = spin_lock_irqsave(&capture->lock);

  *edges = capture->high;

  spin_unlock_irqrestore(&capture->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: fake_capture_bind
 *
 * Description:
 *   This function is used to bind a upper-half provided callback that will
 *   be invoked upon capture edge events.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *   cb    - The callback function pointer.
 *   priv  - The private argument to be passed to the callback function.
 *   edge  - The edge type that will trigger the callback.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int fake_capture_bind(FAR struct cap_lowerhalf_s *lower,
                             enum cap_type_e type, capture_notify_t cb,
                             FAR void *priv)
{
  FAR struct fake_capture_s *capture = (FAR struct fake_capture_s *)lower;
  irqstate_t flags = spin_lock_irqsave(&capture->lock);

  capture->cb = cb;
  capture->priv = priv;
  capture->type = type;

  spin_unlock_irqrestore(&capture->lock, flags);
  return 0;
}

/****************************************************************************
 * Name: fake_capture_unbind
 *
 * Description:
 *   This function is used to un-bind a previously bound upper-half provided
 *   callback that was to be invoked upon capture edge events.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int fake_capture_unbind(FAR struct cap_lowerhalf_s *lower)
{
  FAR struct fake_capture_s *capture = (FAR struct fake_capture_s *)lower;
  irqstate_t flags = spin_lock_irqsave(&capture->lock);

  capture->cb = NULL;
  capture->priv = NULL;

  spin_unlock_irqrestore(&capture->lock, flags);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fake_capture_initialize
 *
 * Description:
 *   This function is called by board-specific logic to initialize
 *   fake capture.
 *
 * Input Parameters:
 *   channel - The capture channel number to initialize.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.  The following
 *   possible error values may be returned (most are returned by
 *   register_driver()):
 *
 ****************************************************************************/

int fake_capture_initialize(int channels)
{
  char path[32];
  int ret = 0;
  int i;

  for (i = 0; i < channels; i++)
    {
      FAR struct fake_capture_s *priv;

      priv = kmm_zalloc(sizeof(*priv));
      DEBUGASSERT(priv);

      spin_lock_init(&priv->lock);
      priv->lower.ops = &g_fake_cap_ops;
      snprintf(path, sizeof(path), "/dev/fake_capture%d", i);
      ret = cap_register(path, &priv->lower);
      if (ret < 0)
        {
          cperr("Failed to register capture:%d\n", ret);
          kmm_free(priv);
          break;
        }
    }

  return ret;
}
