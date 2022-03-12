/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_freerun.c
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
#include <sys/types.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>

#include "esp32s3_clockconfig.h"
#include "esp32s3_freerun.h"
#include "esp32s3_gpio.h"

#ifdef CONFIG_ESP32S3_FREERUN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_TIMERS          2

/* Definition for maximum resolution value in microseconds.
 * Calculated according to the following algorithm:
 *   (PREmax * USEC_PER_SEC) / CLKmin
 * where:
 *   PREmax == 65536, maximum supported divider in HW
 *   CLKmin == 80 MHz, lowest clock frequency (from APB)
 */

#define MAX_US_RESOLUTION   819

#define TIMER_WIDTH         54  /* ESP32-S3 timer has 54-bit counter */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_lock;      /* Device specific lock */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: freerun_handler
 *
 * Description:
 *   Timer interrupt callback. When the freerun timer counter overflows,
 *   this interrupt will occur. We will just increment an overflow counter.
 *
 * Input Parameters:
 *   irq           - CPU interrupt index.
 *   context       - Context data from the ISR.
 *   arg           - Opaque pointer to the internal driver state structure.
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

#ifndef CONFIG_CLOCK_TIMEKEEPING
static int freerun_handler(int irq, void *context, void *arg)
{
  struct esp32s3_freerun_s *freerun = (struct esp32s3_freerun_s *) arg;

  DEBUGASSERT(freerun != NULL);

  freerun->overflow++;

  ESP32S3_TIM_SETALRM(freerun->tch, true); /* Re-enables the alarm */
  ESP32S3_TIM_ACKINT(freerun->tch);        /* Clear the Interrupt */

  return OK;
}
#endif /* CONFIG_CLOCK_TIMEKEEPING */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_freerun_initialize
 *
 * Description:
 *   Initialize the freerun timer wrapper.
 *
 * Input Parameters:
 *   freerun    - Caller allocated instance of the freerun state structure.
 *   chan       - Timer counter channel to be used.
 *   resolution - The required resolution of the timer in units of
 *                microseconds. NOTE that the range is restricted to the
 *                range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32s3_freerun_initialize(struct esp32s3_freerun_s *freerun, int chan,
                               uint16_t resolution)
{
  uint16_t pre;
  int ret = OK;

  tmrinfo("chan=%d resolution=%d usecs\n", chan, resolution);

  DEBUGASSERT(freerun != NULL);
  DEBUGASSERT(chan >= 0);
  DEBUGASSERT(chan < MAX_TIMERS);
  DEBUGASSERT(resolution > 0);

  /* We can't have a resolution bigger than this.
   * The ESP32-S3 prescaler doesn't support.
   * max resolution = (max prescaler * USEC_PER_SEC) / esp_clk_apb_freq()
   */

  DEBUGASSERT(resolution <= MAX_US_RESOLUTION);

  freerun->tch = esp32s3_tim_init(chan);
  if (freerun->tch == NULL)
    {
      tmrerr("ERROR: Failed to allocate TIM %d\n", chan);
      ret = -EBUSY;
    }
  else
    {
      /* Initialize the remaining fields in the state structure. */

      freerun->chan        = chan;
      freerun->resolution  = resolution;
      freerun->max_timeout = (UINT64_C(1) << (TIMER_WIDTH - 1));

      /* Ensure timer is disabled.
       * Change the prescaler divider with the timer enabled can lead to
       * unpredictable results.
       */

      ESP32S3_TIM_STOP(freerun->tch);

      /* Configure clock source */

      ESP32S3_TIM_CLK_SRC(freerun->tch, ESP32S3_TIM_APB_CLK);

      /* Calculate the suitable prescaler for a period for the requested
       * resolution.
       */

      pre = esp_clk_apb_freq() * resolution / USEC_PER_SEC;

      tmrinfo("pre=%" PRIu16 " clk=%d\n", pre, esp_clk_apb_freq());

      /* Configure TIMER prescaler */

      ESP32S3_TIM_SETPRE(freerun->tch, pre);

      /* Configure TIMER mode */

      ESP32S3_TIM_SETMODE(freerun->tch, ESP32S3_TIM_MODE_UP);

      /* Clear TIMER counter value */

      ESP32S3_TIM_CLEAR(freerun->tch);

      /* Set the maximum timeout */

      ESP32S3_TIM_SETALRVL(freerun->tch, freerun->max_timeout);

#ifndef CONFIG_CLOCK_TIMEKEEPING

      /* Set the interrupt */

      freerun->overflow = 0;

      /* Enable autoreload */

      ESP32S3_TIM_SETARLD(freerun->tch, true);

      /* Enable TIMER alarm */

      ESP32S3_TIM_SETALRM(freerun->tch, true);

      /* Clear Interrupt Bits Status */

      ESP32S3_TIM_ACKINT(freerun->tch);

      /* Register the handler */

        {
          irqstate_t flags = spin_lock_irqsave(&g_lock);
          ret = ESP32S3_TIM_SETISR(freerun->tch, freerun_handler, freerun);
          spin_unlock_irqrestore(&g_lock, flags);
        }

      if (ret == OK)
        {
          ESP32S3_TIM_ENABLEINT(freerun->tch);
        }

#endif
      /* Finally, start the TIMER */

      ESP32S3_TIM_START(freerun->tch);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32s3_freerun_counter
 *
 * Description:
 *   Read the counter register of the free-running timer.
 *
 * Input Parameters:
 *   freerun - Caller allocated instance of the freerun state structure.
 *             This structure must have been previously initialized via a
 *             call to esp32s3_freerun_initialize().
 *   ts      - The location in which to return the time from the
 *             free-running timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_CLOCK_TIMEKEEPING

int esp32s3_freerun_counter(struct esp32s3_freerun_s *freerun,
                            struct timespec *ts)
{
  uint64_t usec;
  uint64_t counter;
  uint64_t verify;
  uint32_t overflow;
  uint32_t sec;
  int pending;
  irqstate_t flags;

  DEBUGASSERT(freerun != NULL);
  DEBUGASSERT(ts != NULL);
  DEBUGASSERT(freerun->tch != NULL);

  /* Temporarily disable the overflow counter. */

  flags    = spin_lock_irqsave(&g_lock);

  overflow = freerun->overflow;
  ESP32S3_TIM_GETCTR(freerun->tch, &counter);
  pending  = ESP32S3_TIM_CHECKINT(freerun->tch);
  ESP32S3_TIM_GETCTR(freerun->tch, &verify);

  /* If an interrupt was pending before we re-enabled interrupts, then the
   * overflow needs to be incremented.
   */

  if (pending)
    {
      ESP32S3_TIM_ACKINT(freerun->tch);

      /* Increment the overflow count and use the value of the guaranteed to
       * be AFTER the overflow occurred.
       */

      overflow++;
      counter = verify;

      /* Update freerun overflow counter. */

      freerun->overflow = overflow;
    }

  spin_unlock_irqrestore(&g_lock, flags);

  tmrinfo("counter=%" PRIu64 " (%" PRIu64 ") overflow=%" PRIu32
          ", pending=%i\n",
          counter, verify, overflow, pending);

  usec = (uint64_t)(((overflow * freerun->max_timeout) + counter)
          * freerun->resolution);

  /* And return the value of the timer */

  sec         = (uint32_t)(usec / USEC_PER_SEC);
  ts->tv_sec  = sec;
  ts->tv_nsec = (usec - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

  tmrinfo("   usec=%" PRIu64 " ts=(%lu, %lu)\n",
          usec, (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);

  return OK;
}

#endif /* CONFIG_CLOCK_TIMEKEEPING */

/****************************************************************************
 * Name: esp32s3_freerun_uninitialize
 *
 * Description:
 *   Stop the free-running timer and release all resources that it uses.
 *
 * Input Parameters:
 *   freerun - Caller allocated instance of the freerun state structure.
 *             This structure must have been previously initialized via a
 *             call to esp32s3_freerun_initialize().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32s3_freerun_uninitialize(struct esp32s3_freerun_s *freerun)
{
  int ret;

  DEBUGASSERT(freerun != NULL);
  DEBUGASSERT(freerun->tch != NULL);

  tmrinfo("chan=%d\n", freerun->chan);

  /* Stop timer */

  ESP32S3_TIM_STOP(freerun->tch);

  /* Disable timer interrupt */

  ESP32S3_TIM_DISABLEINT(freerun->tch);

  /* Detach handler */

  ret = ESP32S3_TIM_SETISR(freerun->tch, NULL, NULL);

  /* Free the timer */

  esp32s3_tim_deinit(freerun->tch);
  freerun->tch = NULL;

  return ret;
}

#endif /* CONFIG_ESP32S3_FREERUN */
