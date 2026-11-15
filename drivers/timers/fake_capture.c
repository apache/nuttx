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

/* The simulated signal is a square wave whose edges happen at fixed times
 * of perf_gettime(), used as the "true" time.  Its edges are time stamped
 * with a simulated free running counter that runs
 * CONFIG_FAKE_CAPTURE_ERROR_PPB faster than its nominal frequency, like a
 * real crystal would.  As with a hardware input capture, the counter value
 * is latched exactly at the edge (plus a configurable jitter) and reported
 * later from a watchdog, so the latency of the watchdog does not affect
 * the time stamps.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <errno.h>

#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>
#include <nuttx/timers/capture.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAPTURE_DUTY_DEFAULT 50
#define CAPTURE_FREQ_DEFAULT 10

/* Actual frequency of the simulated counter, in mHz, integer Hz and mHz
 * remainder.
 */

#define CAPTURE_COUNT_MHZ \
  ((int64_t)CONFIG_FAKE_CAPTURE_COUNTER_FREQ * 1000 + \
   (int64_t)CONFIG_FAKE_CAPTURE_COUNTER_FREQ * \
   CONFIG_FAKE_CAPTURE_ERROR_PPB / 1000000)

#define CAPTURE_COUNT_HZ     (CAPTURE_COUNT_MHZ / 1000)
#define CAPTURE_COUNT_MILLI  (CAPTURE_COUNT_MHZ % 1000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fake_capture_s
{
  struct cap_lowerhalf_s lower;   /* Lower half capture driver */
  spinlock_t             lock;    /* For mutually exclusive access */
  struct wdog_s          wdog;    /* For timing capture events */
  capture_notify_t       cb;      /* Capture event callback function pointer */
  FAR void              *priv;    /* Pointer to private data */
  enum cap_type_e        type;    /* Edge type for the capture channel */
  uint32_t               freq;    /* Frequency of the signal (Hz) */
  uint8_t                duty;    /* Duty cycle of the signal (%) */
  int64_t                period;  /* Period of the signal (ns) */
  int64_t                width;   /* High time of the signal (ns) */
  int64_t                edge;    /* Number of the last edge */
  uint32_t               edges;   /* Number of edges since start */
  uint32_t               capture; /* Counter latched by the last edge */
  uint32_t               rand;    /* Jitter pseudo random state */
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
static int fake_capture_getcount(FAR struct cap_lowerhalf_s *lower,
                                 FAR uint32_t *count);
static int fake_capture_getcapture(FAR struct cap_lowerhalf_s *lower,
                                   FAR uint32_t *count);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct cap_ops_s g_fake_cap_ops =
{
  .start      = fake_capture_start,
  .stop       = fake_capture_stop,
  .getduty    = fake_capture_getduty,
  .getfreq    = fake_capture_getfreq,
  .getedges   = fake_capture_getedges,
  .bind       = fake_capture_bind,
  .unbind     = fake_capture_unbind,
  .getcount   = fake_capture_getcount,
  .getcapture = fake_capture_getcapture,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fake_capture_now
 *
 * Description:
 *   The "true" time, in nanoseconds.
 *
 ****************************************************************************/

static inline int64_t fake_capture_now(void)
{
  struct timespec ts;

  perf_convert(perf_gettime(), &ts);
  return (int64_t)ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
}

/****************************************************************************
 * Name: fake_capture_count
 *
 * Description:
 *   The value of the simulated counter at a given true time.
 *
 ****************************************************************************/

static inline int64_t fake_capture_count(int64_t ns)
{
  int64_t sec = ns / NSEC_PER_SEC;
  int64_t nsec = ns % NSEC_PER_SEC;

  return sec * CAPTURE_COUNT_HZ + sec * CAPTURE_COUNT_MILLI / 1000 +
         nsec * CAPTURE_COUNT_HZ / NSEC_PER_SEC;
}

/****************************************************************************
 * Name: fake_capture_jitter
 *
 * Description:
 *   Uniform jitter in [-JITTER, +JITTER] ns, converted to counts.
 *
 ****************************************************************************/

static int64_t fake_capture_jitter(FAR struct fake_capture_s *capture)
{
#if CONFIG_FAKE_CAPTURE_JITTER_NS > 0
  int64_t ns;

  capture->rand = capture->rand * 1664525u + 1013904223u;
  ns = (capture->rand >> 8) % (2 * CONFIG_FAKE_CAPTURE_JITTER_NS + 1);
  ns -= CONFIG_FAKE_CAPTURE_JITTER_NS;
  return ns * CAPTURE_COUNT_HZ / NSEC_PER_SEC;
#else
  UNUSED(capture);
  return 0;
#endif
}

/****************************************************************************
 * Name: fake_capture_edge
 *
 * Description:
 *   The number of the last edge at or before a true time.  The even edges
 *   are rising and the odd edges are falling.
 *
 ****************************************************************************/

static inline int64_t fake_capture_edge(FAR struct fake_capture_s *capture,
                                        int64_t ns)
{
  int64_t period = ns / capture->period;

  return 2 * period + (ns - period * capture->period >= capture->width);
}

/****************************************************************************
 * Name: fake_capture_time
 *
 * Description:
 *   The true time of an edge, in nanoseconds.
 *
 ****************************************************************************/

static inline int64_t fake_capture_time(FAR struct fake_capture_s *capture,
                                        int64_t edge)
{
  return edge / 2 * capture->period + (edge & 1 ? capture->width : 0);
}

/****************************************************************************
 * Name: fake_capture_schedule
 *
 * Description:
 *   Start the watchdog for the edge that follows the last one.  One more
 *   tick makes sure that the watchdog does not expire before the edge.
 *
 ****************************************************************************/

static void fake_capture_isr(wdparm_t arg);

static void fake_capture_schedule(FAR struct fake_capture_s *capture,
                                  int64_t now)
{
  int64_t delay = fake_capture_time(capture, capture->edge + 1) - now;

  wd_start(&capture->wdog, NSEC2TICK(delay) + 1, fake_capture_isr,
           (wdparm_t)capture);
}

static void fake_capture_isr(wdparm_t arg)
{
  FAR struct fake_capture_s *capture = (FAR struct fake_capture_s *)arg;
  int64_t now = fake_capture_now();
  int64_t edge = fake_capture_edge(capture, now);
  irqstate_t flags;
  bool rising;

  flags = spin_lock_irqsave(&capture->lock);

  if (edge != capture->edge)
    {
      /* Latch the counter at the time of the edge, as the hardware does */

      capture->edges  += edge - capture->edge;
      capture->edge    = edge;
      capture->capture = fake_capture_count(fake_capture_time(capture,
                                                              edge)) +
                         fake_capture_jitter(capture);

      rising = (edge & 1) == 0;
      if (capture->cb != NULL &&
          (capture->type == CAP_TYPE_BOTH ||
           (capture->type == CAP_TYPE_RISING && rising) ||
           (capture->type == CAP_TYPE_FALLING && !rising)))
        {
          capture->cb(&capture->lower, capture->priv);
        }
    }

  spin_unlock_irqrestore(&capture->lock, flags);

  fake_capture_schedule(capture, now);
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
  int64_t now = fake_capture_now();
  irqstate_t flags;

  flags = spin_lock_irqsave(&capture->lock);

  capture->edge  = fake_capture_edge(capture, now);
  capture->edges = 0;

  spin_unlock_irqrestore(&capture->lock, flags);

  fake_capture_schedule(capture, now);
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
  FAR struct fake_capture_s *capture = (FAR struct fake_capture_s *)lower;

  *duty = capture->duty;
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
 *   freq  - uint32_t pointer where the frequency value is written.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int fake_capture_getfreq(FAR struct cap_lowerhalf_s *lower,
                                FAR uint32_t *freq)
{
  FAR struct fake_capture_s *capture = (FAR struct fake_capture_s *)lower;

  *freq = capture->freq;
  return OK;
}

/****************************************************************************
 * Name: fake_capture_getedges
 *
 * Description:
 *   This function is a requirement of the upper-half driver. Returns
 *   the number of edges since the capture was started.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *   edges - uint32_t pointer where the number of edges is written.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int fake_capture_getedges(FAR struct cap_lowerhalf_s *lower,
                                 FAR uint32_t *edges)
{
  FAR struct fake_capture_s *capture = (FAR struct fake_capture_s *)lower;

  *edges = capture->edges;
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
 *   type  - The edge type that will trigger the callback.
 *   cb    - The callback function pointer.
 *   priv  - The private argument to be passed to the callback function.
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
  return OK;
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
  return OK;
}

/****************************************************************************
 * Name: fake_capture_getcount
 *
 * Description:
 *   Return the current value of the simulated counter.
 *
 ****************************************************************************/

static int fake_capture_getcount(FAR struct cap_lowerhalf_s *lower,
                                 FAR uint32_t *count)
{
  *count = fake_capture_count(fake_capture_now());
  return OK;
}

/****************************************************************************
 * Name: fake_capture_getcapture
 *
 * Description:
 *   Return the value of the simulated counter latched by the last edge.
 *   It is called from the notification callback, with the lock held.
 *
 ****************************************************************************/

static int fake_capture_getcapture(FAR struct cap_lowerhalf_s *lower,
                                   FAR uint32_t *count)
{
  FAR struct fake_capture_s *capture = (FAR struct fake_capture_s *)lower;

  *count = capture->capture;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fake_capture_create
 *
 * Description:
 *   Create a fake capture lower half, without registering it.
 *
 * Input Parameters:
 *   freq - The frequency of the simulated signal, in Hz.
 *   duty - The duty cycle of the simulated signal, in percent.
 *
 * Returned Value:
 *   The new lower half on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct cap_lowerhalf_s *fake_capture_create(uint32_t freq, uint8_t duty)
{
  FAR struct fake_capture_s *capture;

  if (freq == 0 || freq > NSEC_PER_SEC / 2 || duty == 0 || duty >= 100)
    {
      return NULL;
    }

  capture = kmm_zalloc(sizeof(*capture));
  if (capture == NULL)
    {
      return NULL;
    }

  spin_lock_init(&capture->lock);
  capture->lower.ops = &g_fake_cap_ops;
  capture->freq      = freq;
  capture->duty      = duty;
  capture->period    = NSEC_PER_SEC / freq;
  capture->width     = capture->period * duty / 100;
  capture->rand      = 0x2545f491;

  return &capture->lower;
}

/****************************************************************************
 * Name: fake_capture_initialize
 *
 * Description:
 *   This function is called by board-specific logic to initialize
 *   fake capture.
 *
 * Input Parameters:
 *   channels - The number of capture channels to initialize.
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
      FAR struct cap_lowerhalf_s *lower;

      lower = fake_capture_create(CAPTURE_FREQ_DEFAULT,
                                  CAPTURE_DUTY_DEFAULT);
      if (lower == NULL)
        {
          return -ENOMEM;
        }

      snprintf(path, sizeof(path), "/dev/fake_capture%d", i);
      ret = cap_register(path, lower);
      if (ret < 0)
        {
          cperr("Failed to register capture:%d\n", ret);
          kmm_free(lower);
          break;
        }
    }

  return ret;
}
