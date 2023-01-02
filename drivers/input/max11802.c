/****************************************************************************
 * drivers/input/max11802.c
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

/* References:
 *   "Low-Power, Ultra-Small Resistive Touch-Screen Controllers
 *    with I2C/SPI Interface" Maxim IC, Rev 3, 10/2010
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/random.h>

#include <nuttx/input/touchscreen.h>
#include <nuttx/input/max11802.h>

#include "max11802.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is a value for the threshold that guarantees a big difference on the
 * first pendown (but can't overflow).
 */

#define INVALID_THRESHOLD 0x1000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level SPI helpers */

static void max11802_lock(FAR struct spi_dev_s *spi);
static void max11802_unlock(FAR struct spi_dev_s *spi);
static uint16_t max11802_sendcmd(FAR struct max11802_dev_s *priv,
                                 uint8_t cmd, int *tags);

/* Interrupts and data sampling */

static void max11802_notify(FAR struct max11802_dev_s *priv);
static int max11802_sample(FAR struct max11802_dev_s *priv,
                           FAR struct max11802_sample_s *sample);
static int max11802_waitsample(FAR struct max11802_dev_s *priv,
                               FAR struct max11802_sample_s *sample);
static void max11802_worker(FAR void *arg);
static int max11802_interrupt(int irq, FAR void *context, FAR void *arg);

/* Character driver methods */

static int max11802_open(FAR struct file *filep);
static int max11802_close(FAR struct file *filep);
static ssize_t max11802_read(FAR struct file *filep, FAR char *buffer,
                             size_t len);
static int max11802_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
static int max11802_poll(FAR struct file *filep, struct pollfd *fds,
                         bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations max11802_fops =
{
  max11802_open,    /* open */
  max11802_close,   /* close */
  max11802_read,    /* read */
  NULL,             /* write */
  NULL,             /* seek */
  max11802_ioctl,   /* ioctl */
  NULL,             /* truncate */
  NULL,             /* mmap */
  max11802_poll     /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL            /* unlink */
#endif
};

/* If only a single MAX11802 device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

#ifndef CONFIG_MAX11802_MULTIPLE
static struct max11802_dev_s g_max11802;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct max11802_dev_s *g_max11802list;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max11802_lock
 *
 * Description:
 *   Lock the SPI bus and re-configure as necessary.  This function must be
 *   to assure: (1) exclusive access to the SPI bus, and (2) to assure that
 *   the shared bus is properly configured for the touchscreen controller.
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void max11802_lock(FAR struct spi_dev_s *spi)
{
  /* Lock the SPI bus because there are multiple devices competing for the
   * SPI bus
   */

  SPI_LOCK(spi, true);

  /* We have the lock.  Now make sure that the SPI bus is configured for the
   * MAX11802 (it might have gotten configured for a different device while
   * unlocked)
   */

  SPI_SETMODE(spi, CONFIG_MAX11802_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_MAX11802_FREQUENCY);
}

/****************************************************************************
 * Name: max11802_unlock
 *
 * Description:
 *   Un-lock the SPI bus after each transfer, possibly losing the current
 *   configuration if we are sharing the SPI bus with other devices
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void max11802_unlock(FAR struct spi_dev_s *spi)
{
  /* Relinquish the SPI bus. */

  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: max11802_sendcmd
 ****************************************************************************/

static uint16_t max11802_sendcmd(FAR struct max11802_dev_s *priv,
                                 uint8_t cmd, int *tags)
{
  uint8_t  buffer[2];
  uint16_t result;

  /* Select the MAX11802 */

  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), true);

  /* Send the command */

  SPI_SEND(priv->spi, cmd);

  /* Read the data */

  SPI_RECVBLOCK(priv->spi, buffer, 2);
  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), false);

  result = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
  *tags = result & 0xf;
  result >>= 4; /* Get rid of tags */

  iinfo("cmd:%02x response:%04x\n", cmd, result);
  return result;
}

/****************************************************************************
 * Name: max11802_notify
 ****************************************************************************/

static void max11802_notify(FAR struct max11802_dev_s *priv)
{
  /* If there are threads waiting on poll() for MAX11802 data to become
   * available, then wake them up now.  NOTE: we wake up all waiting
   * threads because we do not know that they are going to do.  If they
   * all try to read the data, then some make end up blocking after all.
   */

  poll_notify(priv->fds, CONFIG_MAX11802_NPOLLWAITERS, POLLIN);

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the sample
       * is no longer available.
       */

      nxsem_post(&priv->waitsem);
    }
}

/****************************************************************************
 * Name: max11802_sample
 ****************************************************************************/

static int max11802_sample(FAR struct max11802_dev_s *priv,
                          FAR struct max11802_sample_s *sample)
{
  irqstate_t flags;
  int ret = -EAGAIN;

  /* Interrupts must be disabled when this is called to (1) prevent posting
   * of semaphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   */

  flags = enter_critical_section();

  /* Is there new MAX11802 sample data available? */

  if (priv->penchange)
    {
      /* Yes.. the state has changed in some way.  Return a copy of the
       * sampled data.
       */

      memcpy(sample, &priv->sample, sizeof(struct max11802_sample_s));

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
 * Name: max11802_waitsample
 ****************************************************************************/

static int max11802_waitsample(FAR struct max11802_dev_s *priv,
                               FAR struct max11802_sample_s *sample)
{
  irqstate_t flags;
  int ret;

  /* Interrupts must be disabled when this is called to (1) prevent posting
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

  nxmutex_unlock(&priv->devlock);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is available.
   */

  while (max11802_sample(priv, sample) < 0)
    {
      /* Wait for a change in the MAX11802 state */

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
   * the device structure.  We may have to wait here.  But we have our
   * sample.  Interrupts and pre-emption will be re-enabled while we wait.
   */

  ret = nxmutex_lock(&priv->devlock);

errout:
  /* Then re-enable interrupts.  We might get interrupt here and there
   * could be a new sample.  But no new threads will run because we still
   * have pre-emption disabled.
   */

  leave_critical_section(flags);

  /* Restore pre-emption.  We might get suspended here but that is okay
   * because we already have our sample.  Note:  this means that if there
   * were two threads reading from the MAX11802 for some reason, the data
   * might be read out of order.
   */

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: max11802_schedule
 ****************************************************************************/

static int max11802_schedule(FAR struct max11802_dev_s *priv)
{
  FAR struct max11802_config_s *config;
  int                           ret;

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Disable further interrupts.  MAX11802 interrupts will be re-enabled
   * after the worker thread executes.
   */

  config->enable(config, false);

  /* Disable the watchdog timer.  It will be re-enabled in the worker thread
   * while the pen remains down.
   */

  wd_cancel(&priv->wdog);

  /* Transfer processing to the worker thread.  Since MAX11802 interrupts are
   * disabled while the work is pending, no special action should be required
   * to protected the work queue.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, max11802_worker, priv, 0);
  if (ret != 0)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
    }

  return OK;
}

/****************************************************************************
 * Name: max11802_wdog
 ****************************************************************************/

static void max11802_wdog(wdparm_t arg)
{
  FAR struct max11802_dev_s *priv =
    (FAR struct max11802_dev_s *)arg;

  max11802_schedule(priv);
}

/****************************************************************************
 * Name: max11802_worker
 ****************************************************************************/

static void max11802_worker(FAR void *arg)
{
  FAR struct max11802_dev_s    *priv = (FAR struct max11802_dev_s *)arg;
  FAR struct max11802_config_s *config;
  uint16_t                      x;
  uint16_t                      y;
  uint16_t                      xdiff;
  uint16_t                      ydiff;
  bool                          pendown;
  int                           tags;
  int                           tags2;

  DEBUGASSERT(priv != NULL);

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Disable the watchdog timer.  This is safe because it is started only
   * by this function and this function is serialized on the worker thread.
   */

  wd_cancel(&priv->wdog);

  /* Lock the SPI bus so that we have exclusive access */

  max11802_lock(priv->spi);

  /* Start coordinate measurement */

  max11802_sendcmd(priv, MAX11802_CMD_MEASUREXY, &tags);

  /* Get exclusive access to the driver data structure */

  nxmutex_lock(&priv->devlock);

  /* Check for pen up or down by reading the PENIRQ GPIO. */

  pendown = config->pendown(config);

  /* Handle the change from pen down to pen up */

  if (pendown)
    {
      iinfo("\nPD\n");
    }
  else
    {
      iinfo("\nPU\n");
    }

  if (!pendown)
    {
      /* The pen is up.. reset thresholding variables. */

      priv->threshx = INVALID_THRESHOLD;
      priv->threshy = INVALID_THRESHOLD;

      /* Ignore the interrupt if the pen was already up (CONTACT_NONE ==
       * pen up and already reported; CONTACT_UP == pen up, but not
       * reported).
       */

      iinfo("\nPC%d\n", priv->sample.contact);

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
   * processed yet, then we have to ignore the pen down event (or else it
   * will look like a drag event)
   */

  else if (priv->sample.contact == CONTACT_UP)
    {
      /* If we have not yet processed the last pen up event, then we
       * cannot handle this pen down event. We will have to discard it.
       * That should be okay because we will set the timer to sample
       * again later.
       */

      iinfo("Previous pen up event still in buffer\n");
      max11802_notify(priv);
      wd_start(&priv->wdog, MAX11802_WDOG_DELAY,
               max11802_wdog, (wdparm_t)priv);
      goto ignored;
    }
  else
    {
      /* Wait for data ready
       *
       * Note: MAX11802 signals the readiness of the results using
       * the lowest 4 bits of the result. However these are the
       * last bits to be read out of the device. It appears that
       * the hardware value can change in the middle of the readout,
       * causing the upper bits to be still invalid even though lower
       * bits indicate valid result.
       *
       * We work around this by reading the registers once more after
       * the tags indicate they are ready.
       */

      int readycount = 0;
      do
        {
#ifdef CONFIG_MAX11802_SWAPXY
          x = max11802_sendcmd(priv, MAX11802_CMD_YPOSITION, &tags);
          y = max11802_sendcmd(priv, MAX11802_CMD_XPOSITION, &tags2);
#else
          x = max11802_sendcmd(priv, MAX11802_CMD_XPOSITION, &tags);
          y = max11802_sendcmd(priv, MAX11802_CMD_YPOSITION, &tags2);
#endif
          if (tags != 0xf && tags2 != 0xf)
            {
              readycount++;
            }
        }
      while (readycount < 2);

      add_ui_randomness((x << 16) | y);

      /* Continue to sample the position while the pen is down */

      wd_start(&priv->wdog, MAX11802_WDOG_DELAY,
               max11802_wdog, (wdparm_t)priv);

      /* Check if data is valid */

      if ((tags & 0x03) != 0)
        {
          iinfo("Touch ended before measurement\n");
          goto ignored;
        }

      /* Perform a thresholding operation so that the results will be more
       * stable.  If the difference from the last sample is small, then
       * ignore the event.
       *
       * REVISIT:  Should a large change in pressure also generate a event?
       */

      xdiff = x > priv->threshx ? (x - priv->threshx) : (priv->threshx - x);
      ydiff = y > priv->threshy ? (y - priv->threshy) : (priv->threshy - y);

      /* Check the thresholds.  Bail if there is no significant
       * difference.
       */

      if (xdiff < CONFIG_MAX11802_THRESHX && ydiff < CONFIG_MAX11802_THRESHY)
        {
          /* Little or no change in either direction ... don't report
           * anything.
           */

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

  /* Notify any waiters that new MAX11802 data is available */

  max11802_notify(priv);

ignored:
  config->enable(config, true);

  /* Release our lock on the state structure and unlock the SPI bus */

  nxmutex_unlock(&priv->devlock);
  max11802_unlock(priv->spi);
}

/****************************************************************************
 * Name: max11802_interrupt
 ****************************************************************************/

static int max11802_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct max11802_dev_s    *priv;
  FAR struct max11802_config_s *config;
  int                           ret;

  /* Which MAX11802 device caused the interrupt? */

#ifndef CONFIG_MAX11802_MULTIPLE
  priv = &g_max11802;
#else
  for (priv = g_max11802list;
       priv && priv->configs->irq != irq;
       priv = priv->flink);

  DEBUGASSERT(priv != NULL);
#endif

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Schedule sampling to occur on the worker thread */

  ret = max11802_schedule(priv);

  /* Clear any pending interrupts and return success */

  config->clear(config);
  return ret;
}

/****************************************************************************
 * Name: max11802_open
 ****************************************************************************/

static int max11802_open(FAR struct file *filep)
{
#ifdef CONFIG_MAX11802_REFCNT
  FAR struct inode          *inode;
  FAR struct max11802_dev_s *priv;
  uint8_t                   tmp;
  int                       ret;

  iinfo("Opening\n");

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct max11802_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the reference count */

  tmp = priv->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_lock;
    }

  /* When the reference increments to 1, this is the first open event
   * on the driver.. and an opportunity to do any one-time initialization.
   */

  /* Save the new open count on success */

  priv->crefs = tmp;

errout_with_lock:
  nxmutex_unlock(&priv->devlock);
  return ret;
#else
  iinfo("Opening\n");
  return OK;
#endif
}

/****************************************************************************
 * Name: max11802_close
 ****************************************************************************/

static int max11802_close(FAR struct file *filep)
{
#ifdef CONFIG_MAX11802_REFCNT
  FAR struct inode          *inode;
  FAR struct max11802_dev_s *priv;
  int                       ret;

  iinfo("Closing\n");
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct max11802_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the reference count unless it would decrement a negative
   * value.  When the count decrements to zero, there are no further
   * open references to the driver.
   */

  if (priv->crefs >= 1)
    {
      priv->crefs--;
    }

  nxmutex_unlock(&priv->devlock);
#endif
  iinfo("Closing\n");
  return OK;
}

/****************************************************************************
 * Name: max11802_read
 ****************************************************************************/

static ssize_t max11802_read(FAR struct file *filep, FAR char *buffer,
                             size_t len)
{
  FAR struct inode          *inode;
  FAR struct max11802_dev_s *priv;
  FAR struct touch_sample_s *report;
  struct max11802_sample_s   sample;
  int                        ret;

  iinfo("buffer:%p len:%d\n", buffer, len);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct max11802_dev_s *)inode->i_private;

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

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait: %d\n", ret);
      return ret;
    }

  /* Try to read sample data. */

  ret = max11802_sample(priv, &sample);
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

      ret = max11802_waitsample(priv, &sample);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          ierr("ERROR: max11802_waitsample: %d\n", ret);
          goto errout;
        }
    }

  /* In any event, we now have sampled MAX11802 data that we can report
   * to the caller.
   */

  report = (FAR struct touch_sample_s *)buffer;
  memset(report, 0, SIZEOF_TOUCH_SAMPLE_S(1));
  report->npoints            = 1;
  report->point[0].id        = sample.id;
  report->point[0].x         = sample.x;
  report->point[0].y         = sample.y;

  /* Report the appropriate flags */

  if (sample.contact == CONTACT_UP)
    {
      /* Pen is now up.  Is the positional data valid?  This is important to
       * know because the release will be sent to the window based on its
       * last positional data.
       */

      if (sample.valid)
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID |
                                    TOUCH_POS_VALID;
        }
      else
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
        }
    }
  else if (sample.contact == CONTACT_DOWN)
    {
      /* First contact */

      report->point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID |
                                TOUCH_POS_VALID;
    }
  else /* if (sample->contact == CONTACT_MOVE) */
    {
      /* Movement of the same contact */

      report->point[0].flags  = TOUCH_MOVE | TOUCH_ID_VALID |
                                TOUCH_POS_VALID;
    }

  iinfo("  id:      %d\n", report->point[0].id);
  iinfo("  flags:   %02x\n", report->point[0].flags);
  iinfo("  x:       %d\n", report->point[0].x);
  iinfo("  y:       %d\n", report->point[0].y);

  ret = SIZEOF_TOUCH_SAMPLE_S(1);

errout:
  nxmutex_unlock(&priv->devlock);
  iinfo("Returning: %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: max11802_ioctl
 ****************************************************************************/

static int max11802_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode          *inode;
  FAR struct max11802_dev_s *priv;
  int                       ret;

  iinfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct max11802_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      case TSIOC_SETFREQUENCY:  /* arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(priv->config != NULL && ptr != NULL);
          priv->config->frequency = SPI_SETFREQUENCY(priv->spi, *ptr);
        }
        break;

      case TSIOC_GETFREQUENCY:  /* arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(priv->config != NULL && ptr != NULL);
          *ptr = priv->config->frequency;
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: max11802_poll
 ****************************************************************************/

static int max11802_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode;
  FAR struct max11802_dev_s *priv;
  int ret;
  int i;

  iinfo("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct max11802_dev_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
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

      for (i = 0; i < CONFIG_MAX11802_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_MAX11802_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->penchange)
        {
          max11802_notify(priv);
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
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max11802_register
 *
 * Description:
 *   Configure the MAX11802 to use the provided SPI device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   dev     - An SPI driver instance
 *   config  - Persistent board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int max11802_register(FAR struct spi_dev_s *spi,
                      FAR struct max11802_config_s *config, int minor)
{
  FAR struct max11802_dev_s *priv;
  char devname[DEV_NAMELEN];
#ifdef CONFIG_MAX11802_MULTIPLE
  irqstate_t flags;
#endif
  int ret;

  iinfo("spi: %p minor: %d\n", spi, minor);

  /* Debug-only sanity checks */

  DEBUGASSERT(spi != NULL && config != NULL && minor >= 0 && minor < 100);

  /* Create and initialize a MAX11802 device driver instance */

#ifndef CONFIG_MAX11802_MULTIPLE
  priv = &g_max11802;
#else
  priv = (FAR struct max11802_dev_s *)
    kmm_malloc(sizeof(struct max11802_dev_s));
  if (!priv)
    {
      ierr("ERROR: kmm_malloc(%d) failed\n", sizeof(struct max11802_dev_s));
      return -ENOMEM;
    }
#endif

  /* Initialize the MAX11802 device driver instance */

  memset(priv, 0, sizeof(struct max11802_dev_s));
  priv->spi     = spi;               /* Save the SPI device handle */
  priv->config  = config;            /* Save the board configuration */
  priv->threshx = INVALID_THRESHOLD; /* Initialize thresholding logic */
  priv->threshy = INVALID_THRESHOLD; /* Initialize thresholding logic */

  /* Initialize mutex & semaphores */

  nxmutex_init(&priv->devlock);     /* Initialize device structure mutex */
  nxsem_init(&priv->waitsem, 0, 0); /* Initialize pen event wait semaphore */

  /* Make sure that interrupts are disabled */

  config->clear(config);
  config->enable(config, false);

  /* Attach the interrupt handler */

  ret = config->attach(config, max11802_interrupt);
  if (ret < 0)
    {
      ierr("ERROR: Failed to attach interrupt\n");
      goto errout_with_priv;
    }

  iinfo("Mode: %d Bits: 8 Frequency: %d\n",
        CONFIG_MAX11802_SPIMODE, CONFIG_MAX11802_FREQUENCY);

  /* Lock the SPI bus so that we have exclusive access */

  max11802_lock(spi);

  /* Configure MAX11802 registers */

  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), true);
  SPI_SEND(priv->spi, MAX11802_CMD_MODE_WR);
  SPI_SEND(priv->spi, MAX11802_MODE);
  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), false);

  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), true);
  SPI_SEND(priv->spi, MAX11802_CMD_AVG_WR);
  SPI_SEND(priv->spi, MAX11802_AVG);
  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), false);

  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), true);
  SPI_SEND(priv->spi, MAX11802_CMD_SAMPLE_WR);
  SPI_SEND(priv->spi, MAX11802_SAMPLE);
  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), false);

  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), true);
  SPI_SEND(priv->spi, MAX11802_CMD_TIMING_WR);
  SPI_SEND(priv->spi, MAX11802_TIMING);
  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), false);

  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), true);
  SPI_SEND(priv->spi, MAX11802_CMD_DELAY_WR);
  SPI_SEND(priv->spi, MAX11802_DELAY);
  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), false);

  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), true);
  SPI_SEND(priv->spi, MAX11802_CMD_PULL_WR);
  SPI_SEND(priv->spi, MAX11802_PULL);
  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), false);

  /* Test that the device access was successful. */

  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), true);
  SPI_SEND(priv->spi, MAX11802_CMD_MODE_RD);
  ret = SPI_SEND(priv->spi, 0);
  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), false);

  /* Unlock the bus */

  max11802_unlock(spi);

  if (ret != MAX11802_MODE)
    {
      ierr("ERROR: max11802 mode readback failed: %02x\n", ret);
      goto errout_with_priv;
    }

  /* Register the device as an input device */

  snprintf(devname, sizeof(devname), DEV_FORMAT, minor);
  iinfo("Registering %s\n", devname);

  ret = register_driver(devname, &max11802_fops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: register_driver() failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* If multiple MAX11802 devices are supported, then we will need to add
   * this new instance to a list of device instances so that it can be
   * found by the interrupt handler based on the received IRQ number.
   */

#ifdef CONFIG_MAX11802_MULTIPLE
  flags          = enter_critical_section();
  priv->flink    = g_max11802list;
  g_max11802list = priv;
  leave_critical_section(flags);
#endif

  /* Schedule work to perform the initial sampling and to set the data
   * availability conditions.
   */

  ret = work_queue(HPWORK, &priv->work, max11802_worker, priv, 0);
  if (ret != 0)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
      goto errout_with_priv;
    }

  /* And return success (?) */

  return OK;

errout_with_priv:
  nxmutex_destroy(&priv->devlock);
  nxsem_destroy(&priv->waitsem);
#ifdef CONFIG_MAX11802_MULTIPLE
  kmm_free(priv);
#endif
  return ret;
}
