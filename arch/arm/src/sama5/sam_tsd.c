/****************************************************************************
 * arch/arm/src/sama5/sam_tsd.c
 *
 *   Copyright (C) 2013, 2016-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * The Atmel sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2011, Atmel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* References:
 *
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/input/touchscreen.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/sam_adc.h"
#include "sam_adc.h"
#include "sam_tsd.h"

#if defined(CONFIG_SAMA5_ADC) && defined(CONFIG_SAMA5_TSD)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/input[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/input%d"
#define DEV_NAMELEN         16

/* Poll the pen position while the pen is down at this rate (50MS): */

#define TSD_WDOG_DELAY      MSEC2TICK(50)

/* This is a value for the threshold that guantees a big difference on the
 * first pendown (but can't overflow).
 */

#define INVALID_THRESHOLD 0x1000

/* Data read bit definitions */

#ifdef CONFIG_SAMA5_TSD_4WIRE
#  define TSD_ALLREADY      (ADC_INT_XRDY | ADC_INT_YRDY | ADC_INT_PRDY)
#else
#  define TSD_ALLREADY      (ADC_INT_XRDY | ADC_INT_YRDY)
#endif

/* Pen sample state bit sets */

#ifdef CONFIG_SAMA5_TSD_4WIRE
#  define TSD_PENUP_VALID   (TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID | \
                             TOUCH_PRESSURE_VALID)
#  define TSD_PENUP_INVALID (TOUCH_UP | TOUCH_ID_VALID)
#  define TSD_PENDOWN       (TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID | \
                             TOUCH_PRESSURE_VALID)
#  define TSD_PENMOVE       (TOUCH_MOVE | TOUCH_ID_VALID | TOUCH_POS_VALID | \
                             TOUCH_PRESSURE_VALID)
#else
#  define TSD_PENUP_VALID   (TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID)
#  define TSD_PENUP_INVALID (TOUCH_UP | TOUCH_ID_VALID)
#  define TSD_PENDOWN       (TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID)
#  define TSD_PENMOVE       (TOUCH_MOVE | TOUCH_ID_VALID | TOUCH_POS_VALID)
#endif

/* Ever-present MIN and MAX macros */

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This describes the state of one contact */

enum sam_contact_e
{
  CONTACT_NONE = 0,             /* No contact */
  CONTACT_DOWN,                 /* First contact */
  CONTACT_MOVE,                 /* Same contact, possibly different position */
  CONTACT_UP,                   /* Contact lost */
};

/* This structure describes the results of one touchscreen sample */

struct sam_sample_s
{
  uint8_t id;                   /* Sampled touch point ID */
  uint8_t contact;              /* Contact state (see enum sam_contact_e) */
  bool valid;                   /* True: x,y,p contain valid, sampled data */
  uint16_t x;                   /* Measured X position */
  uint16_t y;                   /* Measured Y position */
#ifdef CONFIG_SAMA5_TSD_4WIRE
  uint16_t p;                   /* Measured pressure */
#endif
};

/* This structure describes the state of one touchscreen driver instance */

struct sam_tsd_s
{
  uint8_t nwaiters;             /* Number of threads waiting for touchscreen data */
  uint8_t id;                   /* Current touch point ID */
  uint8_t valid;                /* Data ready bit set */
  uint8_t crefs;                /* The number of times the device has been opened */
  volatile bool penchange;      /* An unreported event is buffered */
  uint32_t threshx;             /* Thresholding X value */
  uint32_t threshy;             /* Thresholding Y value */
  sem_t waitsem;                /* Used to wait for the availability of data */

  struct sam_adc_s *adc;        /* ADC device handle */
  struct work_s work;           /* Supports the interrupt handling "bottom half" */
  struct sam_sample_s sample;   /* Last sampled touch point data */
  struct wdog_s wdog;           /* Poll the position while the pen is down */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_SAMA5_TSD_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt bottom half logic and data sampling */

static void sam_tsd_notify(struct sam_tsd_s *priv);
static int  sam_tsd_sample(struct sam_tsd_s *priv,
              struct sam_sample_s *sample);
static int  sam_tsd_waitsample(struct sam_tsd_s *priv,
              struct sam_sample_s *sample);
static void sam_tsd_bottomhalf(void *arg);
static int  sam_tsd_schedule(struct sam_tsd_s *priv);
static void sam_tsd_expiry(wdparm_t arg);

/* Character driver methods */

static int  sam_tsd_open(struct file *filep);
static int  sam_tsd_close(struct file *filep);
static ssize_t sam_tsd_read(struct file *filep, char *buffer, size_t len);
static int  sam_tsd_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  sam_tsd_poll(struct file *filep, struct pollfd *fds, bool setup);

/* Initialization and configuration */

static void sam_tsd_startuptime(struct sam_tsd_s *priv, uint32_t time);
static void sam_tsd_tracking(struct sam_tsd_s *priv, uint32_t time);
static void sam_tsd_trigperiod(struct sam_tsd_s *priv, uint32_t period);
static void sam_tsd_debounce(struct sam_tsd_s *priv, uint32_t time);
static void sam_tsd_initialize(struct sam_tsd_s *priv);
static void sam_tsd_uninitialize(struct sam_tsd_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_tsdops =
{
  sam_tsd_open,    /* open */
  sam_tsd_close,   /* close */
  sam_tsd_read,    /* read */
  NULL,            /* write */
  NULL,            /* seek */
  sam_tsd_ioctl,   /* ioctl */
  sam_tsd_poll     /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/* The driver state structure is pre-allocated. */

static struct sam_tsd_s g_tsd;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_tsd_notify
 ****************************************************************************/

static void sam_tsd_notify(struct sam_tsd_s *priv)
{
  /* If there are threads waiting on poll() for touchscreen data to become
   * available, then wake them up now.  NOTE: we wake up all waiting threads
   * because we do not know that they are going to do.  If they all try to
   * read the data, then some make end up blocking after all.
   */

  poll_notify(priv->fds, CONFIG_SAMA5_TSD_NPOLLWAITERS, POLLIN);

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the
       * touchscreen is no longer available.
       */

      nxsem_post(&priv->waitsem);
    }
}

/****************************************************************************
 * Name: sam_tsd_sample
 ****************************************************************************/

static int sam_tsd_sample(struct sam_tsd_s *priv,
                          struct sam_sample_s *sample)
{
  irqstate_t flags;
  int ret = -EAGAIN;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semaphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   */

  flags = enter_critical_section();

  /* Is there new touchscreen sample data available? */

  if (priv->penchange)
    {
      /* Yes.. the state has changed in some way.  Return a copy of the
       * sampled data.
       */

      memcpy(sample, &priv->sample, sizeof(struct sam_sample_s));

      /* Now manage state transitions */

      if (sample->contact == CONTACT_UP)
        {
          /* Next.. no contact.  Increment the ID so that next contact ID
           * will be unique.  X/Y positions are no longer valid.
           */

          priv->sample.contact = CONTACT_NONE;
          priv->sample.valid   = false;
          priv->id++;
        }
      else if (sample->contact == CONTACT_DOWN)
        {
          /* First report -- next report will be a movement */

          priv->sample.contact = CONTACT_MOVE;
        }

      priv->penchange = false;
      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: sam_tsd_waitsample
 ****************************************************************************/

static int sam_tsd_waitsample(struct sam_tsd_s *priv,
                              struct sam_sample_s *sample)
{
  irqstate_t flags;
  int ret = 0;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semaphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   *
   * In addition, we will also disable pre-emption to prevent other threads
   * from getting control while we muck with the semaphores.
   */

  sched_lock();
  flags = enter_critical_section();

  /* Now release the semaphore that manages mutually exclusive access to
   * the device structure.  This may cause other tasks to become ready to
   * run, but they cannot run yet because pre-emption is disabled.
   */

  sam_adc_unlock(priv->adc);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is available.
   */

  while (sam_tsd_sample(priv, sample) < 0)
    {
      /* Wait for a sample data */

      iinfo("Waiting..\n");
      priv->nwaiters++;
      ret = nxsem_wait(&priv->waitsem);
      priv->nwaiters--;

      if (ret < 0)
        {
          ierr("ERROR: nxsem_wait: %d\n", ret);
          goto errout;
        }
    }

  iinfo("Sampled\n");

  /* Re-acquire the semaphore that manages mutually exclusive access to
   * the device structure. We may have to wait here. But we have our sample.
   * Interrupts and pre-emption will be re-enabled while we wait.
   */

  sam_adc_lock(priv->adc);

errout:
  /* Then re-enable interrupts.  We might get interrupt here and there
   * could be a new sample.  But no new threads will run because we still
   * have pre-emption disabled.
   */

  leave_critical_section(flags);

  /* Restore pre-emption.  We might get suspended here but that is okay
   * because we already have our sample.  Note:  this means that if there
   * were two threads reading from the touchscreen ADC for some reason, the
   * data might be read out of order.
   */

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: sam_tsd_setaverage
 *
 * Description:
 *   The ADC hardware can filter the touchscreen samples by averaging.  The
 *   function selects (or de-selects) that filtering.
 *
 * Input Parameters:
 *   priv - The touchscreen private data structure
 *   tsav - The new (shifted) value of the TSAV field of ADC TSMR register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_tsd_setaverage(struct sam_tsd_s *priv, uint32_t tsav)
{
  uint32_t regval;
  uint32_t minfreq;
  uint32_t tsfreq;

  /* Get the current value of the TSMR register */

  regval = sam_adc_getreg(priv->adc, SAM_ADC_TSMR);

  /* Get the unshifted TSAVE value and compare it to the touchscreen
   * frequency
   *
   *   minfreq = 0: No filtering
   *   minfreq = 1: Averages 2 ADC conversions
   *   minfreq = 2: Averages 4 ADC conversions
   *   minfreq = 3: Averages 8 ADC conversions
   */

  minfreq = (tsav >> ADC_TSMR_TSAV_SHIFT);
  if (minfreq)
    {
      /* TSFREQ: Defines the Touchscreen Frequency compared to the Trigger
       * Frequency.  --> TSFREQ must be greater or equal to TSAV. <--
       */

      tsfreq = (regval & ADC_TSMR_TSFREQ_MASK) >> ADC_TSMR_TSFREQ_SHIFT;
      if (minfreq > tsfreq)
        {
          /* Set TSFREQ = TSAV */

          regval &= ~ADC_TSMR_TSFREQ_MASK;
          regval |=  ADC_TSMR_TSFREQ(minfreq);
        }
    }

  /* Save the new filter value */

  regval &= ~ADC_TSMR_TSAV_MASK;
  regval |= tsav;
  sam_adc_putreg(priv->adc, SAM_ADC_TSMR, regval);
}

/****************************************************************************
 * Name: sam_tsd_bottomhalf
 *
 * Description:
 *   This function executes on the worker thread.  It is scheduled by
 *   sam_tsd_interrupt whenever any interesting, enabled TSD event occurs.
 *   All TSD interrupts are disabled when this function runs.
 *   sam_tsd_bottomhalf will re-enable TSD interrupts when it completes
 *   processing all pending TSD events.
 *
 * Input Parameters:
 *   arg - The touchscreen private data structure cast to (void *)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_tsd_bottomhalf(void *arg)
{
  struct sam_tsd_s *priv = (struct sam_tsd_s *)arg;
  uint32_t pending;
  uint32_t ier;
  uint32_t regval;
  uint32_t xraw;
  uint32_t xscale;
  uint32_t x;
  uint32_t xdiff;
  uint32_t yraw;
  uint32_t yscale;
  uint32_t y;
  uint32_t ydiff;
  uint32_t z1;
  uint32_t z2;
  uint32_t pressr;
  uint32_t p;
  bool pendown;

  DEBUGASSERT(priv != NULL);

  /* Get the set of pending ADC interrupts and pen status */

  pending = sam_adc_getreg(priv->adc, SAM_ADC_ISR);

  /* Get exclusive access to the driver data structure */

  sam_adc_lock(priv->adc);

  /* Check the pen state */

  pendown = ((pending & ADC_SR_PENS) != 0);

  /* Handle the change from pen down to pen up */

  iinfo("pending: %08" PRIx32 " pendown: %d contact: %d\n",
        pending, pendown, priv->sample.contact);

  if (!pendown)
    {
      /* The pen is up.. reset thresholding variables. */

      priv->threshx = INVALID_THRESHOLD;
      priv->threshy = INVALID_THRESHOLD;

      /* We will enable only the ADC_INT_PEN interrupt on exit.  We don't
       * want to hear anything from the touchscreen until the next touch.
       */

      ier = ADC_INT_PEN;

      /* Ignore the interrupt if the pen was already up (CONTACT_NONE == pen
       * up and already reported; CONTACT_UP == pen up, but not reported)
       */

      if (priv->sample.contact == CONTACT_NONE ||
          priv->sample.contact == CONTACT_UP)

        {
          goto ignored;
        }

      /* The pen is up.  NOTE: We know from a previous test, that this is a
       * loss of contact condition.  This will be changed to CONTACT_NONE
       * after the loss of contact is sampled.
       */

       priv->sample.contact = CONTACT_UP;

      /* Stop periodic trigger & enable pen */

      sam_tsd_setaverage(priv, ADC_TSMR_TSAV_NOFILTER);
      sam_tsd_debounce(priv, BOARD_TSD_DEBOUNCE);

      regval  = sam_adc_getreg(priv->adc, SAM_ADC_TRGR);
      regval &= ~ADC_TRGR_TRGMOD_MASK;
      regval |= ADC_TRGR_TRGMOD_PEN;
      sam_adc_putreg(priv->adc, SAM_ADC_TRGR, regval);
    }

  /* It is a pen down event.  If the last loss-of-contact event has not been
   * processed yet, then we have to ignore the pen down event (or else it
   * will look like a drag event)
   */

  else if (priv->sample.contact == CONTACT_UP)
    {
      /* If we have not yet processed the last pen up event, then we
       * cannot handle this pen down event. We will have to discard it.  That
       * should be okay because we will set the timer to sample again
       * a little later.  NOTE that pen interrupts are not re-enabled in
       * this case; we rely on the timer expiry to get us going again.
       */

      wd_start(&priv->wdog, TSD_WDOG_DELAY,
               sam_tsd_expiry, (wdparm_t)priv);
      ier = 0;
      goto ignored;
    }
  else
    {
      /* The pen is down and the driver accepted the last sample values. */

      /* While the pen is down we want interrupts on all data ready and pen
       * release events.
       */

      ier = ADC_TSD_RELEASEINTS;

      /* Check if all of the date that we need is available.  If not, just
       * re-enable interrupts and wait until it is.
       */

      if ((pending & TSD_ALLREADY) != TSD_ALLREADY)
        {
          /* But don't enable interrupts for the data that we already have */

          ier &= ~(pending & TSD_ALLREADY);
          goto ignored;
        }

      /* Sample positional values.  Get raw X and Y position data */

      regval = sam_adc_getreg(priv->adc, SAM_ADC_XPOSR);
      xraw   = (regval & ADC_XPOSR_XPOS_MASK) >> ADC_XPOSR_XPOS_SHIFT;
      xscale = (regval & ADC_XPOSR_XSCALE_MASK) >> ADC_XPOSR_XSCALE_SHIFT;

      regval = sam_adc_getreg(priv->adc, SAM_ADC_YPOSR);
      yraw   = (regval & ADC_YPOSR_YPOS_MASK) >> ADC_YPOSR_YPOS_SHIFT;
      yscale = (regval & ADC_YPOSR_YSCALE_MASK) >> ADC_YPOSR_YSCALE_SHIFT;

#ifdef CONFIG_SAMA5_TSD_4WIRE
      /* Read the PRESSR register now, but don't do anything until we
       * decide if we are going to use this measurement.
       */

      pressr = sam_adc_getreg(priv->adc, SAM_ADC_PRESSR);
#endif
      /* Discard any bad readings.  This check may not be necessary. */

      if (xraw == 0 || xraw >= xscale || yraw == 0 || yraw > yscale)
        {
          iwarn("WARNING: Discarding: x %" PRId32 ":%" PRId32
                " y %" PRId32 ":%" PRId32 "\n",
                xraw, xscale,
                yraw, yscale);
          goto ignored;
        }

      /* Scale the X/Y measurements.  The scale value is the maximum
       * value that the sample can attain.  It should be close to 4095.
       * Scaling:
       *
       *   scaled = raw * 4095 / scale
       *          = ((raw << 12) - raw) / scale
       */

#ifdef CONFIG_SAMA5_TSD_SWAPXY
      x  = ((yraw << 12) - yraw) / yscale;
      y  = ((xraw << 12) - xraw) / xscale;
#else
      x  = ((xraw << 12) - xraw) / xscale;
      y  = ((yraw << 12) - yraw) / yscale;
#endif

      /* Perform a thresholding operation so that the results will be
       * more stable.  If the difference from the last sample is small,
       * then ignore the event. REVISIT:  Should a large change in
       * pressure also generate a event?
       */

      xdiff = x > priv->threshx ? (x - priv->threshx) : (priv->threshx - x);
      ydiff = y > priv->threshy ? (y - priv->threshy) : (priv->threshy - y);

      /* Continue to sample the position while the pen is down */

      wd_start(&priv->wdog, TSD_WDOG_DELAY,
               sam_tsd_expiry, (wdparm_t)priv);

      /* Check the thresholds.  Bail if (1) this is not the first
       * measurement and (2) there is no significant difference from
       * the last measurement.
       */

      if (priv->sample.contact == CONTACT_MOVE &&
          xdiff < CONFIG_SAMA5_TSD_THRESHX &&
          ydiff < CONFIG_SAMA5_TSD_THRESHY)
        {
          /* Little or no change in either direction ... don't report
           * anything.
           */

          goto ignored;
        }

      /* When we see a big difference, snap to the new x/y thresholds */

      priv->threshx  = x;
      priv->threshy  = y;

      /* Update the x/y position in the sample data */

      priv->sample.x = MIN(x, UINT16_MAX);
      priv->sample.y = MIN(y, UINT16_MAX);

#ifdef CONFIG_SAMA5_TSD_4WIRE
      /* Scale the pressure and update the pressure in the sample data.
       *
       * The method to measure the pressure (Rp) applied to the
       * touchscreen is based on the known resistance of the X-Panel
       * resistance (Rxp). Three conversions (Xpos, Z1, Z2) are
       * necessary to determine the value of Rp (Zaxis resistance).
       *
       *   Rp = Rxp * (Xraw / 1024) * [(Z2 / Z1) - 1]
       */

      z2 = (pressr & ADC_PRESSR_Z2_MASK) >> ADC_PRESSR_Z2_SHIFT;
      z1 = (pressr & ADC_PRESSR_Z1_MASK) >> ADC_PRESSR_Z1_SHIFT;
      p  = CONFIG_SAMA_TSD_RXP * xraw * (z2 - z1) / z1;

      priv->sample.p = MIN(p, UINT16_MAX);
#endif

      /* The X/Y positional data is now valid */

      priv->sample.valid = true;

      /* If this is the first (acknowledged) pen down report, then
       * report this as the first contact.  If contact == CONTACT_DOWN,
       * it will be set to set to CONTACT_MOVE after the contact is
       * first sampled.
       */

      if (priv->sample.contact != CONTACT_MOVE)
        {
          /* First contact.  Handle transitions from pen UP to pen DOWN */

          priv->sample.contact = CONTACT_DOWN;

          /* Configure for periodic trigger */

          sam_tsd_setaverage(priv, ADC_TSMR_TSAV_8CONV);
          sam_tsd_debounce(priv, 300);         /* 300ns */

          regval  = sam_adc_getreg(priv->adc, SAM_ADC_TRGR);
          regval &= ~ADC_TRGR_TRGMOD_MASK;
          regval |= ADC_TRGR_TRGMOD_PERIOD;
          sam_adc_putreg(priv->adc, SAM_ADC_TRGR, regval);
        }
    }

  /* Indicate the availability of new sample data for this ID */

  priv->sample.id = priv->id;
  priv->penchange = true;

  /* Notify any waiters that new touchscreen data is available */

  sam_tsd_notify(priv);

  /* Exit, re-enabling touchscreen interrupts */

ignored:

  /* Re-enable touchscreen interrupts as appropriate. */

  sam_adc_putreg(priv->adc, SAM_ADC_IER, ier);

  /* Release our lock on the state structure */

  sam_adc_unlock(priv->adc);
}

/****************************************************************************
 * Name: sam_tsd_schedule
 ****************************************************************************/

static int sam_tsd_schedule(struct sam_tsd_s *priv)
{
  int ret;

  /* Disable the watchdog timer.  It will be re-enabled in the worker thread
   * while the pen remains down.
   */

  wd_cancel(&priv->wdog);

  /* Disable further touchscreen interrupts.  Touchscreen interrupts will be
   * re-enabled after the worker thread executes.
   */

  sam_adc_putreg(priv->adc, SAM_ADC_IDR, ADC_TSD_ALLINTS);

  /* Transfer processing to the worker thread.  Since touchscreen ADC
   * interrupts are disabled while the work is pending, no special action
   * should be required to protected the work queue.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, sam_tsd_bottomhalf, priv, 0);
  if (ret != 0)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
    }

  return OK;
}

/****************************************************************************
 * Name: sam_tsd_expiry
 *
 * Description:
 *   While the pen is pressed, pen position is periodically polled via a
 *   watchdog timer.  This function handles that timer expiration.
 *
 ****************************************************************************/

static void sam_tsd_expiry(wdparm_t arg)
{
  struct sam_tsd_s *priv = (struct sam_tsd_s *)arg;

  /* Schedule touchscreen work */

  sam_tsd_schedule(priv);
}

/****************************************************************************
 * Name: sam_tsd_open
 ****************************************************************************/

static int sam_tsd_open(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct sam_tsd_s *priv = inode->i_private;
  uint8_t tmp;
  int ret;

  iinfo("crefs: %d\n", priv->crefs);

  /* Get exclusive access to the device structures */

  sam_adc_lock(priv->adc);

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = priv->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EAGAIN;
    }
  else
    {
      /* Save the new open count */

      priv->crefs = tmp;

      /* Initialize the touchscreen when it is first opened */

      if (tmp == 1)
        {
          sam_tsd_initialize(priv);
        }

      /* Successfully opened */

      ret = OK;
    }

  sam_adc_unlock(priv->adc);
  return ret;
}

/****************************************************************************
 * Name: sam_tsd_close
 ****************************************************************************/

static int sam_tsd_close(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct sam_tsd_s *priv = inode->i_private;

  iinfo("crefs: %d\n", priv->crefs);

  /* Get exclusive access to the ADC device */

  sam_adc_lock(priv->adc);

  /* Decrement the references to the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If this was the last reference to the driver, then un-initialize the
   * TSD now.
   */

  if (priv->crefs == 0)
    {
      sam_tsd_uninitialize(priv);
    }

  sam_adc_unlock(priv->adc);
  return OK;
}

/****************************************************************************
 * Name: sam_tsd_read
 ****************************************************************************/

static ssize_t sam_tsd_read(struct file *filep, char *buffer, size_t len)
{
  struct inode *inode;
  struct sam_tsd_s *priv;
  struct touch_sample_s *report;
  struct sam_sample_s sample;
  int ret;

  iinfo("buffer:%p len:%d\n", buffer, len);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (struct sam_tsd_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the touch data.
   */

  if (len < SIZEOF_TOUCH_SAMPLE_S(1))
    {
      /* We could provide logic to break up a touch report into segments and
       * handle smaller reads... but why?
       */

      ierr("ERROR: Unsupported read size: %d\n", len);
      return -ENOSYS;
    }

  /* Get exclusive access to the driver data structure */

  sam_adc_lock(priv->adc);

  /* Try to read sample data. */

  ret = sam_tsd_sample(priv, &sample);
  if (ret < 0)
    {
      /* Sample data is not available now.  We would ave to wait to get
       * receive sample data.  If the user has specified the O_NONBLOCK
       * option, then just return an error.
       */

      iinfo("Sample data is not available\n");
      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
        }

      /* Wait for sample data */

      ret = sam_tsd_waitsample(priv, &sample);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          ierr("ERROR: sam_tsd_waitsample: %d\n", ret);
          goto errout;
        }
    }

  /* In any event, we now have sampled touchscreen data that we can report
   * to the caller.
   */

  report = (struct touch_sample_s *)buffer;
  memset(report, 0, SIZEOF_TOUCH_SAMPLE_S(1));
  report->npoints           = 1;
  report->point[0].id       = sample.id;
  report->point[0].x        = sample.x;
  report->point[0].y        = sample.y;
#ifdef CONFIG_SAMA5_TSD_4WIRE
  report->point[0].pressure = sample.p;
#endif

  /* Report the appropriate flags */

  if (sample.contact == CONTACT_UP)
    {
      /* Pen is now up.  Is the positional data valid?  This is important
       * to know because the release will be sent to the window based on
       * its last positional data.
       */

      if (sample.valid)
        {
          report->point[0].flags  = TSD_PENUP_VALID;
        }
      else
        {
          report->point[0].flags  = TSD_PENUP_INVALID;
        }
    }
  else if (sample.contact == CONTACT_DOWN)
    {
      /* First contact */

      report->point[0].flags  = TSD_PENDOWN;
    }
  else /* if (sample->contact == CONTACT_MOVE) */
    {
      /* Movement of the same contact */

      report->point[0].flags  = TSD_PENMOVE;
    }

  iinfo("  id:      %d\n", report->point[0].id);
  iinfo("  flags:   %02x\n", report->point[0].flags);
  iinfo("  x:       %d\n", report->point[0].x);
  iinfo("  y:       %d\n", report->point[0].y);

  ret = SIZEOF_TOUCH_SAMPLE_S(1);

errout:
  sam_adc_unlock(priv->adc);
  iinfo("Returning: %d\n", ret);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: sam_tsd_ioctl
 ****************************************************************************/

static int sam_tsd_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode *inode;
  struct sam_tsd_s *priv;
  int ret;

  iinfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (struct sam_tsd_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  sam_adc_lock(priv->adc);

  /* Process the IOCTL by command */

  switch (cmd)
    {
      default:
        ret = -ENOTTY;
        break;
    }

  sam_adc_unlock(priv->adc);
  return ret;
}

/****************************************************************************
 * Name: sam_tsd_poll
 ****************************************************************************/

static int sam_tsd_poll(struct file *filep, struct pollfd *fds, bool setup)
{
  struct inode *inode;
  struct sam_tsd_s *priv;
  int ret = OK;
  int i;

  iinfo("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (struct sam_tsd_s *)inode->i_private;

  /* Get exclusive access to the ADC hardware */

  sam_adc_lock(priv->adc);

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_SAMA5_TSD_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_SAMA5_TSD_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->penchange)
        {
          sam_tsd_notify(priv);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  sam_adc_unlock(priv->adc);
  return ret;
}

/****************************************************************************
 * Initialization and Configuration
 ****************************************************************************/

/****************************************************************************
 * Name: sam_tsd_startuptime
 *
 * Description:
 *   Set the STARTUP field of the ADC MR register.
 *
 * Input Parameters:
 *   priv - A reference to the touchscreen device structure
 *   time - The new startup time in nanoseconds
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_tsd_startuptime(struct sam_tsd_s *priv, uint32_t time)
{
  uint32_t startup;
  uint32_t regval;

  /* Formula for STARTUP is:
   *
   * STARTUP = (time x ADCCLK) / (1000000) - 1
   *
   * Division multiplied by 10 for higher precision.
   */

  startup = (time * BOARD_ADCCLK_FREQUENCY) / 100000;

  if (startup % 10)
    {
      /* Handle partial values by not decrementing startup.  This is
       * basically a 'ceil' operation.
       */

      startup /= 10;
    }
  else
    {
      /* The final value needs to be decrement by one */

      startup /= 10;
      if (startup)
        {
          startup--;
        }
    }

  /* Then set the STARTUP value in the ADC MR register. */

  regval  = sam_adc_getreg(priv->adc, SAM_ADC_MR);
  regval &= ~ADC_MR_STARTUP_MASK;

  if (startup > 896)
    {
      regval = ADC_MR_STARTUP_960;
    }
  else if (startup > 832)
    {
      regval = ADC_MR_STARTUP_896;
    }
  else if (startup > 768)
    {
      regval = ADC_MR_STARTUP_832;
    }
  else if (startup > 704)
    {
      regval = ADC_MR_STARTUP_768;
    }
  else if (startup > 640)
    {
      regval = ADC_MR_STARTUP_704;
    }
  else if (startup > 576)
    {
      regval = ADC_MR_STARTUP_640;
    }
  else if (startup > 512)
    {
      regval = ADC_MR_STARTUP_576;
    }
  else if (startup > 112)
    {
      regval = ADC_MR_STARTUP_512;
    }
  else if (startup > 96)
    {
      regval = ADC_MR_STARTUP_112;
    }
  else if (startup > 80)
    {
      regval = ADC_MR_STARTUP_96;
    }
  else if (startup > 64)
    {
      regval = ADC_MR_STARTUP_80;
    }
  else if (startup > 24)
    {
      regval = ADC_MR_STARTUP_64;
    }
  else if (startup > 16)
    {
      regval = ADC_MR_STARTUP_24;
    }
  else if (startup > 8)
    {
      regval = ADC_MR_STARTUP_16;
    }
  else if (startup > 0)
    {
      regval = ADC_MR_STARTUP_8;
    }
  else
    {
      regval = ADC_MR_STARTUP_0;
    }

  sam_adc_putreg(priv->adc, SAM_ADC_MR, regval);
}

/****************************************************************************
 * Name: sam_tsd_tracking
 *
 * Description:
 *   Set the TRACKTIM field of the ADC MR register.
 *
 *     TrackingTime = (TRACKTIM + 1) * ADCClock periods.
 *     TRACKTIM     = (TrackingTime * ADCCLK) / (1000000000) - 1
 *
 * Input Parameters:
 *   priv - A reference to the touchscreen device structure
 *   time - The new tracking time in nanoseconds
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_tsd_tracking(struct sam_tsd_s *priv, uint32_t time)
{
  uint32_t tracktim;
  uint32_t regval;

  /* Formula for SHTIM is:
   *
   *  TRACKTIM     = (TrackingTime * ADCCLK) / (1000000000) - 1
   *
   * Since 1 billion is close to the maximum value for an integer, we first
   * divide ADCCLK by 1000 to avoid an overflow
   */

  tracktim = (time * (BOARD_ADCCLK_FREQUENCY / 1000)) / 100000;
  if (tracktim % 10)
    {
      /* Handle partial values by not decrementing tracktim.  This is
       * basically a 'ceil' operation.
       */

      tracktim /= 10;
    }
  else
    {
      /* The final value needs to be decrement by one */

      tracktim /= 10;
      if (tracktim)
        {
          tracktim--;
        }
    }

  /* Set the neew TRACKTIM field value int he ADC MR register */

  regval  = sam_adc_getreg(priv->adc, SAM_ADC_MR);
  regval &= ~ADC_MR_TRACKTIM_MASK;
  regval |= ADC_MR_TRACKTIM(tracktim);
  sam_adc_putreg(priv->adc, SAM_ADC_MR, regval);
}

/****************************************************************************
 * Name: sam_tsd_trigperiod
 *
 * Description:
 *   Set the TGPER field of the TRGR register in order to define a periodic
 *   trigger perioc.
 *
 *     Trigger Period = (TRGPER+1) / ADCCLK
 *
 * Input Parameters:
 *   priv - A reference to the touchscreen device structure
 *   time - The new trigger period in nanoseconds
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_tsd_trigperiod(struct sam_tsd_s *priv, uint32_t period)
{
  uint32_t trigper;
  uint32_t regval;
  uint32_t div;

  /* Divide trigger period avoid overflows.  Division by ten is awkard, but
   * appropriate here because times are specified in decimal with lots of
   * zeroes.
   */

  div = 100000000;
  while (period >= 10 && div >= 10)
    {
      period /= 10;
      div    /= 10;
    }

  /* Calculate and adjust the scaled trigger period:
   *
   *   Trigger Period = (TRGPER+1) / ADCCLK
   */

  trigper = (period * BOARD_ADCCLK_FREQUENCY) / div;
  if ((trigper % 10) != 0)
    {
      /* Handle partial values by not decrementing trigper.  This is
       * basically a 'ceil' operation.
       */

      trigper /= 10;
    }
  else
    {
      /* The final value needs to be decrement by one */

      trigper /= 10;
      if (trigper > 0)
        {
          trigper--;
        }
    }

  /* Set the calculated trigger period in the TRGR register */

  regval  = sam_adc_getreg(priv->adc, SAM_ADC_TRGR);
  regval &= ~ADC_TRGR_TRGPER_MASK;
  regval |=  ADC_TRGR_TRGPER(trigper);
  sam_adc_putreg(priv->adc, SAM_ADC_TRGR, regval);
}

/****************************************************************************
 * Name: sam_tsd_debounce
 *
 * Description:
 *   Set the Pen Detect Debouncing Period (PENBC) in the touchscreen mode
 *   register.
 *
 *     Debouncing period = 2 ** PENDBC ADCClock periods.
 *
 * Input Parameters:
 *   priv - A reference to the touchscreen device structure
 *   time - The new debounce time in nanoseconds
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_tsd_debounce(struct sam_tsd_s *priv, uint32_t time)
{
  uint32_t candidate;
  uint32_t target;
  uint32_t regval;
  uint32_t pendbc;
  uint32_t div;
  uint32_t clk;

  /* Make sure that a valid debounce time was provided */

  DEBUGASSERT(time > 0);

  /* Divide time and ADCCLK to avoid overflows.  Division by ten is awkard,
   * but appropriate here because times are specified in decimal with lots of
   * zeroes.
   */

  div = 1000000000;
  while (div > 1 && (time % 10) == 0)
    {
      time /= 10;
      div  /= 10;
    }

  clk = BOARD_ADCCLK_FREQUENCY;
  while (div > 1 && (clk % 10) == 0)
    {
      clk  /= 10;
      div  /= 10;
    }

  /* Compute PENDBC */

  target    = time * clk / div;
  candidate = 1;
  pendbc    = 0;

  while (candidate < target)
    {
      pendbc++;
      candidate *= 2;
    }

  DEBUGASSERT(pendbc > 0);

  /* Update the TSMR with the new PENBC value */

  regval = sam_adc_getreg(priv->adc, SAM_ADC_TSMR);
  regval &= ~ADC_TSMR_PENDBC_MASK;
  regval |= ADC_TSMR_PENDBC(pendbc);
  sam_adc_putreg(priv->adc, SAM_ADC_TSMR, regval);
}

/****************************************************************************
 * Name: sam_tsd_initialize
 *
 * Description:
 *   Initialize the touchscreen for normal operation.  This function is
 *   called from sam_tsd_open() the first time that the driver is opened.
 *
 * Input Parameters:
 *   priv - A reference to the touchscreen device structure
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static void sam_tsd_initialize(struct sam_tsd_s *priv)
{
  uint32_t regval;

  /* Disable touch trigger */

  regval  = sam_adc_getreg(priv->adc, SAM_ADC_TRGR);
  regval &= ~ADC_TRGR_TRGMOD_MASK;
  regval |= ADC_TRGR_TRGMOD_NOTRIG;
  sam_adc_putreg(priv->adc, SAM_ADC_TRGR, regval);

  /* Setup timing */

  sam_tsd_startuptime(priv, BOARD_TSD_STARTUP);
  sam_tsd_tracking(priv, BOARD_TSD_TRACKTIM);
  sam_tsd_trigperiod(priv, 20000000); /*  20ms */

  /* Setup the touchscreen mode */

  regval  = sam_adc_getreg(priv->adc, SAM_ADC_TSMR);
  regval &= ~ADC_TSMR_TSMODE_MASK;

#if defined(CONFIG_SAMA5_TSD_5WIRE)
  regval |= ADC_TSMR_TSMODE_5WIRE;
#elif defined(CONFIG_SAMA5_TSD_4WIRENPM)
  regval |= ADC_TSMR_TSMODE_4WIRENPM;
#else /* if defined(CONFIG_SAMA5_TSD_4WIRE) */
  regval |= ADC_TSMR_TSMODE_4WIRE;
#endif

  sam_adc_putreg(priv->adc, SAM_ADC_TSMR, regval);

  /* Disable averaging */

  sam_tsd_setaverage(priv, ADC_TSMR_TSAV_NOFILTER);

  /* Select 4-wire w/pressure, 4-wire w/o pressure, or 5 wire modes */

  regval  = sam_adc_getreg(priv->adc, SAM_ADC_TSMR);
  regval &= ~ADC_TSMR_TSMODE_MASK;

#if defined(CONFIG_SAMA5_TSD_5WIRE)
  regval |= ADC_TSMR_TSMODE_5WIRE;
#elif defined(CONFIG_SAMA5_TSD_4WIRENPM)
  regval |= ADC_TSMR_TSMODE_4WIRENPM;
#else /* if defined(CONFIG_SAMA5_TSD_4WIRE) */
  regval |= ADC_TSMR_TSMODE_4WIRE;
#endif

  /* Disable all TSD-related interrupts */

  sam_adc_putreg(priv->adc, SAM_ADC_IDR, ADC_TSD_ALLINTS);

  /* Clear any pending TSD interrupts */

  sam_adc_getreg(priv->adc, SAM_ADC_ISR);

  /* Read and discard any samples */

  sam_adc_getreg(priv->adc, SAM_ADC_XPOSR);
  sam_adc_getreg(priv->adc, SAM_ADC_YPOSR);
#ifdef CONFIG_SAMA5_TSD_4WIRE
  sam_adc_getreg(priv->adc, SAM_ADC_PRESSR);
#endif

  /* Enable pen contact detection */

  regval |= ADC_TSMR_PENDET;
  sam_adc_putreg(priv->adc, SAM_ADC_TSMR, regval);

  /* Set up pen debounce time */

  sam_tsd_debounce(priv, BOARD_TSD_DEBOUNCE);

  /* Configure pen interrupt generation */

  regval  = sam_adc_getreg(priv->adc, SAM_ADC_TRGR);
  regval &= ~ADC_TRGR_TRGMOD_MASK;
  regval |= ADC_TRGR_TRGMOD_PEN;
  sam_adc_putreg(priv->adc, SAM_ADC_TRGR, regval);

  sam_adc_putreg(priv->adc, SAM_ADC_IER, ADC_INT_PEN);
}

/****************************************************************************
 * Name: sam_tsd_uninitialize
 *
 * Description:
 *   Uninitialize the touchscreen.  This function is called from
 *   sam_tsd_close() when the final driver instance is closed.
 *
 * Input Parameters:
 *   priv - A reference to the touchscreen device structure
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static void sam_tsd_uninitialize(struct sam_tsd_s *priv)
{
  uint32_t regval;

  /* Disable the watchdog timer.  It will be re-enabled in the worker thread
   * while the pen remains down.
   */

  wd_cancel(&priv->wdog);

  /* Disable further touchscreen interrupts.  Touchscreen interrupts will be
   * re-enabled after the worker thread executes.
   */

  sam_adc_putreg(priv->adc, SAM_ADC_IDR, ADC_TSD_ALLINTS);

  /* Disable touch trigger */

  regval  = sam_adc_getreg(priv->adc, SAM_ADC_TRGR);
  regval &= ~ADC_TRGR_TRGMOD_MASK;
  regval |= ADC_TRGR_TRGMOD_NOTRIG;
  sam_adc_putreg(priv->adc, SAM_ADC_TRGR, regval);

  /* Disable the touchscreen mode */

  regval  = sam_adc_getreg(priv->adc, SAM_ADC_TSMR);
  regval &= ~ADC_TSMR_TSMODE_MASK;
  regval |= ADC_TSMR_TSMODE_NONE;
  sam_adc_putreg(priv->adc, SAM_ADC_TSMR, regval);

  /* No touch and no valid sample data */

  priv->sample.contact = CONTACT_NONE;
  priv->sample.valid = false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_tsd_register
 *
 * Description:
 *   Configure the SAMA5 touchscreen.  This will register the driver as
 *   /dev/inputN where N is the minor device number
 *
 * Input Parameters:
 *   adc   - An opaque reference to the ADC device structure
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int sam_tsd_register(struct sam_adc_s *adc, int minor)
{
  struct sam_tsd_s *priv = &g_tsd;
  char devname[DEV_NAMELEN];
  int ret;

  iinfo("minor: %d\n", minor);

  /* Debug-only sanity checks */

  DEBUGASSERT(adc && minor >= 0 && minor < 100);

  /* Initialize the touchscreen device driver instance */

  memset(priv, 0, sizeof(struct sam_tsd_s));
  priv->adc     = adc;               /* Save the ADC device handle */
  priv->threshx = INVALID_THRESHOLD; /* Initialize thresholding logic */
  priv->threshy = INVALID_THRESHOLD; /* Initialize thresholding logic */

  /* Initialize pen event wait semaphore.  This semaphore is used for
   * signaling and, hence, should not have priority inheritance enabled.
   */

  nxsem_init(&priv->waitsem, 0, 0);
  nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

  /* Register the device as an input device */

  snprintf(devname, sizeof(devname), DEV_FORMAT, minor);
  iinfo("Registering %s\n", devname);

  ret = register_driver(devname, &g_tsdops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: register_driver() failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* And return success.  The hardware will be initialized as soon as the
   * touchscreen driver is opened for the first time.
   */

  return OK;

errout_with_priv:
  nxsem_destroy(&priv->waitsem);
  return ret;
}

/****************************************************************************
 * Name: sam_tsd_interrupt
 *
 * Description:
 *   Handles ADC interrupts associated with touchscreen channels
 *
 * Input Parameters:
 *   pending - Current set of pending interrupts being handled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_tsd_interrupt(uint32_t pending)
{
  struct sam_tsd_s *priv = &g_tsd;
  int ret;

  /* Are there pending TSD interrupts? */

  if ((pending & ADC_TSD_ALLINTS) != 0)
    {
      /* Schedule sampling to occur by the interrupt bottom half on the
       * worker thread.
       */

      ret = sam_tsd_schedule(priv);
      if (ret < 0)
        {
          ierr("ERROR: sam_tsd_schedule failed: %d\n", ret);
        }
    }
}

#endif /* CONFIG_SAMA5_ADC && CONFIG_SAMA5_TSD */
