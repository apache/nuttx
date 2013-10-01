/****************************************************************************
 * arch/arm/src/sama5/sam_touchsreen.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <poll.h>
#include <wdog.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>

#include <nuttx/input/touchscreen.h>

#include "sam_touchscreen.h"

#if defined(CONFIG_SAMA5_ADC) && defined(CONFIG_SAMA5_TOUCHSCREEN)

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

#define TSD_WDOG_DELAY      ((50 + (MSEC_PER_TICK-1))/ MSEC_PER_TICK)

/* This is a value for the threshold that guantees a big difference on the
 * first pendown (but can't overflow).
 */

#define INVALID_THRESHOLD 0x1000

/* Touchscreen interrupt event sets
 *
 *   ADC_INT_XRDY           TS Measure XPOS Ready Interrupt
 *   ADC_INT_YRDY           TS Measure YPOS Ready Interrupt
 *   ADC_INT_PRDY           TS Measure Pressure Ready Interrupt
 *   ADC_INT_PEN            Pen Contact Interrupt
 *   ADC_INT_NOPEN          No Pen Contact Interrupt
 *   ADC_SR_PENS            Pen detect Status (Not an interrupt)
 */

#define ADC_TSD_CMNINTS     (ADC_INT_XRDY | ADC_INT_YRDY | ADC_INT_PRDY | ADC_INT_NOPEN)
#define ADC_TSD_ALLINTS     (ADC_TSD_CMNINTS | ADC_INT_PEN)
#define ADC_TSD_ALLSTATUS   (ADC_TSD_ALLINTS | ADC_INT_PENS)
#define ADC_TSD_RELEASEINTS ADC_TSD_CMNINTS

/* Data ready/pen status bit definitions */

#define TSD_XREADY          (1 << 0)  /* X value is ready */
#define TSD_YREADY          (1 << 1)  /* Y value is ready */
#define TSD_PREADY          (1 << 2)  /* Pressure value is ready */
#define TSD_PENDOWN         (1 << 3)  /* Pen is down */
#define TSD_ALLREADY        (TSD_XREADY | TSD_YREADY | TSD_PREADY)

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
  bool valid;                   /* True: x,y contain valid, sampled data */
  uint16_t x;                   /* Measured X position */
  uint16_t y;                   /* Measured Y position */
};

/* This structure describes the state of one touchscreen driver instance */

struct sam_tsd_s
{
  uint8_t nwaiters;             /* Number of threads waiting for touchscreen data */
  uint8_t id;                   /* Current touch point ID */
  uint8_t status;               /* Data ready/pen status bit set */
  volatile bool penchange;      /* An unreported event is buffered */
  uint16_t threshx;             /* Thresholding X value */
  uint16_t threshy;             /* Thresholding Y value */
  volatile uint32_t pending;    /* Pending interrupt set set */
  sem_t devsem;                 /* Manages exclusive access to this structure */
  sem_t waitsem;                /* Used to wait for the availability of data */

  struct adc_dev_s *dev;        /* ADC device handle */
  struct work_s work;           /* Supports the interrupt handling "bottom half" */
  struct sam_sample_s sample;   /* Last sampled touch point data */
  WDOG_ID wdog;                 /* Poll the position while the pen is down */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_SAMA5_TSD_NPOLLWAITERS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Interrupt bottom half logic and data sampling */

static void sam_notify(struct sam_tsd_s *priv);
static int  sam_sample(struct sam_tsd_s *priv, struct sam_sample_s *sample);
static int  sam_waitsample(struct sam_tsd_s *priv,
              struct sam_sample_s *sample);
static void sam_bottomhalf(void *arg);
static int  sam_schedule(struct sam_tsd_s *priv, uint32_t pending);
static void sam_wdog(int argc, uint32_t arg1, ...);

/* Character driver methods */

static int  sam_open(struct file *filep);
static int  sam_close(struct file *filep);
static ssize_t sam_read(struct file *filep, char *buffer, size_t len);
static int  sam_ioctl(struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int  sam_poll(struct file *filep, struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_tsdops =
{
  sam_open,    /* open */
  sam_close,   /* close */
  sam_read,    /* read */
  0,           /* write */
  0,           /* seek */
  sam_ioctl    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , sam_poll   /* poll */
#endif
};

/* The driver state structure is pre-allocated. */

static struct sam_tsd_s g_tsd;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_notify
 ****************************************************************************/

static void sam_notify(struct sam_tsd_s *priv)
{
#ifndef CONFIG_DISABLE_POLL
  int i;
#endif

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the touchscreen
       * is no longer available.
       */

      sem_post(&priv->waitsem);
    }

  /* If there are threads waiting on poll() for touchscreen data to become available,
   * then wake them up now.  NOTE: we wake up all waiting threads because we
   * do not know that they are going to do.  If they all try to read the data,
   * then some make end up blocking after all.
   */

#ifndef CONFIG_DISABLE_POLL
  for (i = 0; i < CONFIG_SAMA5_TSD_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          ivdbg("Report events: %02x\n", fds->revents);
          sem_post(fds->sem);
        }
    }
#endif
}

/****************************************************************************
 * Name: sam_sample
 ****************************************************************************/

static int sam_sample(struct sam_tsd_s *priv, struct sam_sample_s *sample)
{
  irqstate_t flags;
  int ret = -EAGAIN;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semaphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   */

  flags = irqsave();

  /* Is there new touchscreen sample data available? */

  if (priv->penchange)
    {
      /* Yes.. the state has changed in some way.  Return a copy of the
       * sampled data.
       */

      memcpy(sample, &priv->sample, sizeof(struct sam_sample_s ));

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

  irqrestore(flags);
  return ret;
}

/****************************************************************************
 * Name: sam_waitsample
 ****************************************************************************/

static int sam_waitsample(struct sam_tsd_s *priv, struct sam_sample_s *sample)
{
  irqstate_t flags;
  int ret;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semaphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   *
   * In addition, we will also disable pre-emption to prevent other threads
   * from getting control while we muck with the semaphores.
   */

  sched_lock();
  flags = irqsave();

  /* Now release the semaphore that manages mutually exclusive access to
   * the device structure.  This may cause other tasks to become ready to
   * run, but they cannot run yet because pre-emption is disabled.
   */

  sem_post(&priv->devsem);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is available.
   */

  while (sam_sample(priv, sample) < 0)
    {
      /* Wait for a sample data */

      ivdbg("Waiting..\n");
      priv->nwaiters++;
      ret = sem_wait(&priv->waitsem);
      priv->nwaiters--;

      if (ret < 0)
        {
          /* If we are awakened by a signal, then we need to return
           * the failure now.
           */

          idbg("sem_wait: %d\n", errno);
          DEBUGASSERT(errno == EINTR);
          ret = -EINTR;
          goto errout;
        }
    }

  ivdbg("Sampled\n");

   /* Re-acquire the semaphore that manages mutually exclusive access to
   * the device structure.  We may have to wait here.  But we have our sample.
   * Interrupts and pre-emption will be re-enabled while we wait.
   */

  ret = sem_wait(&priv->devsem);

errout:
  /* Then re-enable interrupts.  We might get interrupt here and there
   * could be a new sample.  But no new threads will run because we still
   * have pre-emption disabled.
   */

  irqrestore(flags);

  /* Restore pre-emption.  We might get suspended here but that is okay
   * because we already have our sample.  Note:  this means that if there
   * were two threads reading from the touchscreen ADC for some reason, the
   * data might be read out of order.
   */

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: sam_bottomhalf
 *
 * Description:
 *   This function executes on the worker thread.  It is scheduled by
 *   sam_tsd_interrupt whenever any interesting, enabled TSD event occurs.
 *   All TSD interrupts are disabled when this function runs.  sam_bottomhalf
 *   will re-enable TSD interrupts when it completes processing all pending
 *   TSD events.
 *
 * Input Parameters
 *   arg - The touchscreen private data structure cast to (void *)
 * 
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_bottomhalf(void *arg)
{
  struct sam_tsd_s *priv = (struct sam_tsd_s *)arg;
  uint32_t pending;
  uint32_t regval;
  uint16_t x;
  uint16_t y;
  uint16_t xdiff;
  uint16_t ydiff;
  bool pendown;
  int ret;

  ASSERT(priv != NULL);

  /* Disable the watchdog timer.  This is safe because it is started only
   * by this function and this function is serialized on the worker thread.
   */

  wd_cancel(priv->wdog);

  /* Get the set of unmasked, pending ADC interrupts */

  pending = priv->pending;

  /* Get exclusive access to the driver data structure */

  do
    {
      ret = sem_wait(&priv->devsem);

      /* This should only fail if the wait was canceled by an signal
       * (and the worker thread will receive a lot of signals).
       */

      DEBUGASSERT(ret == OK || errno == EINTR);
    }
  while (ret < 0);

  /* Check for pen up or down by reading the PENIRQ GPIO. */
#warning Missing logic

  /* Handle the change from pen down to pen up */

  if (!pendown)
    {
      /* The pen is up.. reset thresholding variables. */

      priv->threshx = INVALID_THRESHOLD;
      priv->threshy = INVALID_THRESHOLD;

      /* Ignore the interrupt if the pen was already up (CONTACT_NONE == pen up
       * and already reported; CONTACT_UP == pen up, but not reported)
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
    }

  /* It is a pen down event.  If the last loss-of-contact event has not been
   * processed yet, then we have to ignore the pen down event (or else it will
   * look like a drag event)
   */

  else if (priv->sample.contact == CONTACT_UP)
    {
      /* If we have not yet processed the last pen up event, then we
       * cannot handle this pen down event. We will have to discard it.  That
       * should be okay because we will set the timer to to sample again
       * later.
       */

       wd_start(priv->wdog, TSD_WDOG_DELAY, sam_wdog, 1, (uint32_t)priv);
       goto ignored;
    }
  else
    {
      /* Handle pen down events.  First, sample positional values. */
#warning Missing logic

      /* Perform a thresholding operation so that the results will be more stable.
       * If the difference from the last sample is small, then ignore the event.
       * REVISIT:  Should a large change in pressure also generate a event?
       */

      xdiff = x > priv->threshx ? (x - priv->threshx) : (priv->threshx - x);
      ydiff = y > priv->threshy ? (y - priv->threshy) : (priv->threshy - y);

      /* Continue to sample the position while the pen is down */

      wd_start(priv->wdog, TSD_WDOG_DELAY, sam_wdog, 1, (uint32_t)priv);

      /* Check the thresholds.  Bail if there is no significant difference */

      if (xdiff < CONFIG_SAMA5_TSD_THRESHX && ydiff < CONFIG_SAMA5_TSD_THRESHY)
        {
          /* Little or no change in either direction ... don't report anything. */

          goto ignored;
        }

      /* When we see a big difference, snap to the new x/y thresholds */

      priv->threshx       = x;
      priv->threshy       = y;

      /* Update the x/y position in the sample data */

      priv->sample.x      = priv->threshx;
      priv->sample.y      = priv->threshy;

      /* The X/Y positional data is now valid */

      priv->sample.valid = true;

      /* If this is the first (acknowledged) pen down report, then report
       * this as the first contact.  If contact == CONTACT_DOWN, it will be
       * set to set to CONTACT_MOVE after the contact is first sampled.
       */

      if (priv->sample.contact != CONTACT_MOVE)
        {
          /* First contact */

          priv->sample.contact = CONTACT_DOWN;
        }
    }

  /* Indicate the availability of new sample data for this ID */

  priv->sample.id = priv->id;
  priv->penchange = true;

  /* Notify any waiters that new touchscreen data is available */

  sam_notify(priv);

  /* Exit, re-enabling touchscreen interrupts */

ignored:
  /* Re-enable touchscreen interrupts. */

  sam_adc_putreg32(priv->dev, SAM_ADC_IER, ADC_TSD_ALLINTS);

  /* Release our lock on the state structure */

  sem_post(&priv->devsem);
}

/****************************************************************************
 * Name: sam_schedule
 ****************************************************************************/

static int sam_schedule(struct sam_tsd_s *priv, uint32_t pending)
{
  int ret;

  /* Disable the watchdog timer.  It will be re-enabled in the worker thread
   * while the pen remains down.
   */

  wd_cancel(priv->wdog);

  /* Disable further touchscreen interrupts.  Touchscreen interrupts will be
   * re-enabled after the worker thread executes.
   */

  sam_adc_putreg32(priv->dev, SAM_ADC_IDR, ADC_TSD_ALLINTS);

  /* Save the set of pending interrupts for the bottom half (in case any
   * were cleared by reading the ISR.
   */

  priv->pending = pending.

  /* Transfer processing to the worker thread.  Since touchscreen ADC interrupts are
   * disabled while the work is pending, no special action should be required
   * to protected the work queue.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, sam_bottomhalf, priv, 0);
  if (ret != 0)
    {
      illdbg("Failed to queue work: %d\n", ret);
    }

  return OK;
}

/****************************************************************************
 * Name: sam_wdog
 *
 * Description:
 *   While the pen is pressed, pen position is periodically polled via a
 *   watchdog timer.  This function handles that timer expiration.
 *
 ****************************************************************************/

static void sam_wdog(int argc, uint32_t arg1, ...)
{
  struct sam_tsd_s *priv = (struct sam_tsd_s *)((uintptr_t)arg1);
  uint32_t pending;

  /* There should be no pending TSD interrupts, but we need to get the PENS
   * status bit as a minimum.
   */

  pending =  sam_adc_getreg(priv, SAM_ADC_ISR) & ADC_TSD_ALLSTATUS;
  (void)sam_schedule(priv, pending);
}

/****************************************************************************
 * Name: sam_open
 ****************************************************************************/

static int sam_open(struct file *filep)
{
  ivdbg("Opening\n");
  return OK;
}

/****************************************************************************
 * Name: sam_close
 ****************************************************************************/

static int sam_close(struct file *filep)
{
  ivdbg("Closing\n");
  return OK;
}

/****************************************************************************
 * Name: sam_read
 ****************************************************************************/

static ssize_t sam_read(struct file *filep, char *buffer, size_t len)
{
  struct inode *inode;
  struct sam_tsd_s *priv;
  struct touch_sample_s *report;
  struct sam_sample_s sample;
  int ret;

  ivdbg("buffer:%p len:%d\n", buffer, len);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (struct sam_tsd_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the touch data.
   */

  if (len < SIZEOF_TOUCH_SAMPLE_S(1))
    {
      /* We could provide logic to break up a touch report into segments and
       * handle smaller reads... but why?
       */

      idbg("Unsupported read size: %d\n", len);
      return -ENOSYS;
    }

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      idbg("sem_wait: %d\n", errno);
      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Try to read sample data. */

  ret = sam_sample(priv, &sample);
  if (ret < 0)
    {
      /* Sample data is not available now.  We would ave to wait to get
       * receive sample data.  If the user has specified the O_NONBLOCK
       * option, then just return an error.
       */

      ivdbg("Sample data is not available\n");
      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
       }

      /* Wait for sample data */

      ret = sam_waitsample(priv, &sample);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          idbg("sam_waitsample: %d\n", ret);
          goto errout;
        }
    }

  /* In any event, we now have sampled touchscreen data that we can report
   * to the caller.
   */

  report = (struct touch_sample_s *)buffer;
  memset(report, 0, SIZEOF_TOUCH_SAMPLE_S(1));
  report->npoints      = 1;
  report->point[0].id  = sample.id;
  report->point[0].x   = sample.x;
  report->point[0].y   = sample.y;

  /* Report the appropriate flags */

  if (sample.contact == CONTACT_UP)
    {
       /* Pen is now up.  Is the positional data valid?  This is important to
        * know because the release will be sent to the window based on its
        * last positional data.
        */

      if (sample.valid)
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID;
        }
      else
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
        }
    }
  else if (sample.contact == CONTACT_DOWN)
    {
      /* First contact */

      report->point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID;
    }
  else /* if (sample->contact == CONTACT_MOVE) */
    {
      /* Movement of the same contact */

      report->point[0].flags  = TOUCH_MOVE | TOUCH_ID_VALID | TOUCH_POS_VALID;
    }

  ivdbg("  id:      %d\n", report->point[0].id);
  ivdbg("  flags:   %02x\n", report->point[0].flags);
  ivdbg("  x:       %d\n", report->point[0].x);
  ivdbg("  y:       %d\n", report->point[0].y);

  ret = SIZEOF_TOUCH_SAMPLE_S(1);

errout:
  sem_post(&priv->devsem);
  ivdbg("Returning: %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name:sam_ioctl
 ****************************************************************************/

static int sam_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode *inode;
  struct sam_tsd_s *priv;
  int ret;

  ivdbg("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (struct sam_tsd_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      default:
        ret = -ENOTTY;
        break;
    }

  sem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: sam_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int sam_poll(struct file *filep, struct pollfd *fds, bool setup)
{
  struct inode *inode;
  struct sam_tsd_s *priv;
  int ret = OK;
  int i;

  ivdbg("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (struct sam_tsd_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

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
          sam_notify(priv);
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
  sem_post(&priv->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int sam_tsd_register(struct adc_dev_s *dev, int minor)
{
  struct sam_tsd_s *priv = &g_tsd;
  char devname[DEV_NAMELEN];
  int ret;

  ivdbg("minor: %d\n", minor);

  /* Debug-only sanity checks */

  DEBUGASSERT(dev && minor >= 0 && minor < 100);

  /* Initialize the touchscreen device driver instance */

  memset(priv, 0, sizeof(struct sam_tsd_s));
  priv->dev     = dev;               /* Save the ADC device handle */
  priv->wdog    = wd_create();       /* Create a watchdog timer */
  priv->threshx = INVALID_THRESHOLD; /* Initialize thresholding logic */
  priv->threshy = INVALID_THRESHOLD; /* Initialize thresholding logic */

  sem_init(&priv->devsem,  0, 1);    /* Initialize device structure semaphore */
  sem_init(&priv->waitsem, 0, 0);    /* Initialize pen event wait semaphore */

  /* Make sure that interrupts are disabled and clear any pending interrupts */
#warning Missing logic

  /* Attach the interrupt handler */
#warning Missing logic

  /* Enable the PEN IRQ */
#warning Missing logic

  /* Register the device as an input device */

  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, minor);
  ivdbg("Registering %s\n", devname);

  ret = register_driver(devname, &g_tsdops, 0666, priv);
  if (ret < 0)
    {
      idbg("register_driver() failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* Schedule work to perform the initial sampling and to set the data
   * availability conditions.
   */

  ret = work_queue(HPWORK, &priv->work, sam_bottomhalf, priv, 0);
  if (ret != 0)
    {
      idbg("Failed to queue work: %d\n", ret);
      goto errout_with_priv;
    }

  /* And return success */

  return OK;

errout_with_priv:
  sem_destroy(&priv->devsem);
  sem_destroy(&priv->waitsem);
  return ret;
}

/****************************************************************************
 * Name: sam_tsd_interrupt
 *
 * Description:
 *   Handles ADC interrupts associated with touchscreen channels
 *
 * Input parmeters:
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
      /* Disable further touchscreen interrupts */

      sam_adc_putreg32(priv->dev, SAM_ADC_IDR, ADC_TSD_ALLINTS);

      /* Save the set of pending interrupts for the bottom half (in case any
       * were cleared by reading the ISR.
       */

      priv->pending = pending & ADC_TSD_ALLSTATUS;

      /* Schedule sampling to occur by the interrupt bottom half on the
       * worker thread.
       */

      ret = sam_schedule(priv, pending);
      if (ret < 0)
        {
          idbg("ERROR: sam_schedule failed: %d\n", ret);
        }
    }

  return ret;
}

#endif /* CONFIG_SAMA5_ADC && CONFIG_SAMA5_TOUCHSCREEN */
