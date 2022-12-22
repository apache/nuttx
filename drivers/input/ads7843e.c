/****************************************************************************
 * drivers/input/ads7843e.c
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
 *   "Touch Screen Controller, ADS7843," Burr-Brown Products from Texas
 *    Instruments, SBAS090B, September 2000, Revised May 2002"
 *
 * See also:
 *   "Low Voltage I/O Touch Screen Controller, TSC2046," Burr-Brown Products
 *    from Texas Instruments, SBAS265F, October 2002, Revised August 2007.
 *
 *   "XPT2046 Data Sheet," Shenzhen XPTek Technology Co., Ltd, 2007
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
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wqueue.h>
#include <nuttx/random.h>

#include <nuttx/input/touchscreen.h>
#include <nuttx/input/ads7843e.h>

#include "ads7843e.h"

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

static void ads7843e_lock(FAR struct spi_dev_s *spi);
static void ads7843e_unlock(FAR struct spi_dev_s *spi);

static uint16_t ads7843e_sendcmd(FAR struct ads7843e_dev_s *priv,
                                 uint8_t cmd);

/* Interrupts and data sampling */

static void ads7843e_notify(FAR struct ads7843e_dev_s *priv);
static int  ads7843e_sample(FAR struct ads7843e_dev_s *priv,
                            FAR struct ads7843e_sample_s *sample);
static int  ads7843e_waitsample(FAR struct ads7843e_dev_s *priv,
                                FAR struct ads7843e_sample_s *sample);
static void ads7843e_worker(FAR void *arg);
static int  ads7843e_interrupt(int irq, FAR void *context, FAR void *arg);

/* Character driver methods */

static int  ads7843e_open(FAR struct file *filep);
static int  ads7843e_close(FAR struct file *filep);
static ssize_t ads7843e_read(FAR struct file *filep, FAR char *buffer,
                             size_t len);
static int  ads7843e_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
static int  ads7843e_poll(FAR struct file *filep, struct pollfd *fds,
                          bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations ads7843e_fops =
{
  ads7843e_open,    /* open */
  ads7843e_close,   /* close */
  ads7843e_read,    /* read */
  NULL,             /* write */
  NULL,             /* seek */
  ads7843e_ioctl,   /* ioctl */
  NULL,             /* truncate */
  ads7843e_poll     /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL            /* unlink */
#endif
};

/* If only a single ADS7843E device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

#ifndef CONFIG_ADS7843E_MULTIPLE
static struct ads7843e_dev_s g_ads7843e;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct ads7843e_dev_s *g_ads7843elist;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads7843e_lock
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

static void ads7843e_lock(FAR struct spi_dev_s *spi)
{
  /* Lock the SPI bus because there are multiple devices competing for the
   * SPI bus
   */

  SPI_LOCK(spi, true);

  /* We have the lock.  Now make sure that the SPI bus is configured for the
   * ADS7843 (it might have gotten configured for a different device while
   * unlocked)
   */

  SPI_SELECT(spi, SPIDEV_TOUCHSCREEN(0), true);
  SPI_SETMODE(spi, CONFIG_ADS7843E_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_ADS7843E_FREQUENCY);
  SPI_SELECT(spi, SPIDEV_TOUCHSCREEN(0), false);
}

/****************************************************************************
 * Name: ads7843e_unlock
 *
 * Description:
 *   Un-lock the SPI bus after each transfer,  possibly losing the current
 *   configuration if we are sharing the bus with other devices.
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

static void ads7843e_unlock(FAR struct spi_dev_s *spi)
{
  /* Relinquish the SPI bus. */

  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: ads7843e_sendcmd
 *
 * Description:
 *   The command/data sequences is as follows:
 *
 *            DCLK
 *              1  2   3   4   5    6   7   8         1   2  3  4  ...
 *              S  A2  A1  A0 MODE SER PD1 PD0
 *                                 DFR
 *   START      CCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
 *   CMD
 *   Acquisition                    AAAAAAAAAAA
 *   TIME
 *   BUSY                                     BBBBBBBB
 *   Reported
 *   12-bit                                           DDDDDDDDDDDD...
 *   response
 *
 *   The BUSY output is high impedance when /CS is high.  BUSY goes low when
 *   /CS goes low (within 200ns).  BUSY goes high on the falling edge of the
 *   8th clock (within 200ns); BUSY goes low again after the falling edge of
 *   first clock of the 12-bit data read, at the leading edge of the MS bit
 *   11 of the 12-bit data response.
 *
 *   The acquisition time is 3 clock cycles and so should be complete at the
 *   end of the command transfer.  Other places say that this time is
 *   nominally 2 microseconds.
 *
 *   So what good is this BUSY?  Many boards do not even bother to bring it
 *   to the MCU.  Busy will stick high until we read the data so you cannot
 *   wait on it before reading.
 *
 ****************************************************************************/

static uint16_t ads7843e_sendcmd(FAR struct ads7843e_dev_s *priv,
                                 uint8_t cmd)
{
  uint8_t  buffer[2];
  uint16_t result;

  /* Select the ADS7843E */

  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), true);

  /* Send the command */

  SPI_SEND(priv->spi, cmd);

  /* Wait a tiny amount to make sure that the acquisition time is complete */

  up_udelay(3); /* 3 microseconds */

  /* Read the 12-bit data (LS 4 bits will be padded with zero) */

  SPI_RECVBLOCK(priv->spi, buffer, 2);
  SPI_SELECT(priv->spi, SPIDEV_TOUCHSCREEN(0), false);

  result = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
  result = result >> 4;

  iinfo("cmd:%02x response:%04x\n", cmd, result);
  return result;
}

/****************************************************************************
 * Name: ads7843e_notify
 ****************************************************************************/

static void ads7843e_notify(FAR struct ads7843e_dev_s *priv)
{
  /* If there are threads waiting on poll() for ADS7843E data to become
   * available, then wake them up now.  NOTE: we wake up all waiting threads
   * because we do not know that they are going to do.  If they all try to
   * read the data, then some make end up blocking after all.
   */

  poll_notify(priv->fds, CONFIG_ADS7843E_NPOLLWAITERS, POLLIN);

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the ADS7843E
       * is no longer available.
       */

      nxsem_post(&priv->waitsem);
    }
}

/****************************************************************************
 * Name: ads7843e_sample
 ****************************************************************************/

static int ads7843e_sample(FAR struct ads7843e_dev_s *priv,
                           FAR struct ads7843e_sample_s *sample)
{
  irqstate_t flags;
  int ret = -EAGAIN;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semaphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   */

  flags = enter_critical_section();

  /* Is there new ADS7843E sample data available? */

  if (priv->penchange)
    {
      /* Yes.. the state has changed in some way.  Return a copy of the
       * sampled data.
       */

      memcpy(sample, &priv->sample, sizeof(struct ads7843e_sample_s));

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
 * Name: ads7843e_waitsample
 ****************************************************************************/

static int ads7843e_waitsample(FAR struct ads7843e_dev_s *priv,
                               FAR struct ads7843e_sample_s *sample)
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
  flags = enter_critical_section();

  /* Now release the semaphore that manages mutually exclusive access to
   * the device structure.  This may cause other tasks to become ready to
   * run, but they cannot run yet because pre-emption is disabled.
   */

  nxmutex_unlock(&priv->devlock);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is available.
   */

  while (ads7843e_sample(priv, sample) < 0)
    {
      /* Wait for a change in the ADS7843E state */

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
   * were two threads reading from the ADS7843E for some reason, the data
   * might be read out of order.
   */

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: ads7843e_schedule
 ****************************************************************************/

static int ads7843e_schedule(FAR struct ads7843e_dev_s *priv)
{
  FAR struct ads7843e_config_s *config;
  int                           ret;

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Disable further interrupts.  ADS7843E interrupts will be re-enabled
   * after the worker thread executes.
   */

  config->enable(config, false);

  /* Disable the watchdog timer.  It will be re-enabled in the worker thread
   * while the pen remains down.
   */

  wd_cancel(&priv->wdog);

  /* Transfer processing to the worker thread.  Since ADS7843E interrupts are
   * disabled while the work is pending, no special action should be required
   * to protected the work queue.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, ads7843e_worker, priv, 0);
  if (ret != 0)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
    }

  return OK;
}

/****************************************************************************
 * Name: ads7843e_wdog
 ****************************************************************************/

static void ads7843e_wdog(wdparm_t arg)
{
  FAR struct ads7843e_dev_s *priv =
    (FAR struct ads7843e_dev_s *)arg;

  ads7843e_schedule(priv);
}

/****************************************************************************
 * Name: ads7843e_worker
 ****************************************************************************/

static void ads7843e_worker(FAR void *arg)
{
  FAR struct ads7843e_dev_s    *priv = (FAR struct ads7843e_dev_s *)arg;
  FAR struct ads7843e_config_s *config;
  uint16_t                      x;
  uint16_t                      y;
  uint16_t                      xdiff;
  uint16_t                      ydiff;
  bool                          pendown;

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

  ads7843e_lock(priv->spi);

  /* Get exclusive access to the driver data structure */

  nxmutex_lock(&priv->devlock);

  /* Check for pen up or down by reading the PENIRQ GPIO. */

  pendown = config->pendown(config);

  /* Handle the change from pen down to pen up */

  if (!pendown)
    {
      /* The pen is up.. reset thresholding variables. */

      priv->threshx = INVALID_THRESHOLD;
      priv->threshy = INVALID_THRESHOLD;

      /* Ignore the interrupt if the pen was already up (CONTACT_NONE ==
       * pen up and already reported; CONTACT_UP == pen up, but not
       * reported)
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
   * processed yet, then we have to ignore the pen down event (or else it
   * will look like a drag event)
   */

  else if (priv->sample.contact == CONTACT_UP)
    {
      /* If we have not yet processed the last pen up event, then we
       * cannot handle this pen down event. We will have to discard it.  That
       * should be okay because we will set the timer to sample again
       * later.
       */

      wd_start(&priv->wdog, ADS7843E_WDOG_DELAY,
               ads7843e_wdog, (wdparm_t)priv);
      goto ignored;
    }
  else
    {
      /* Handle pen down events.  First, sample positional values. NOTE:
       * that these commands have the side-effect of disabling the PENIRQ.
       */

#ifdef CONFIG_ADS7843E_SWAPXY
      x = ads7843e_sendcmd(priv, ADS7843_CMD_YPOSITION);
      y = ads7843e_sendcmd(priv, ADS7843_CMD_XPOSITION);
#else
      x = ads7843e_sendcmd(priv, ADS7843_CMD_XPOSITION);
      y = ads7843e_sendcmd(priv, ADS7843_CMD_YPOSITION);
#endif

      add_ui_randomness((x << 16) | y);

      /* Perform a thresholding operation so that the results will be more
       * stable.  If the difference from the last sample is small, then
       * ignore the event.  REVISIT:  Should a large change in pressure also
       * generate a event?
       */

      xdiff = x > priv->threshx ?
        (x - priv->threshx) :
        (priv->threshx - x);
      ydiff = y > priv->threshy ?
        (y - priv->threshy) :
        (priv->threshy - y);

      /* Continue to sample the position while the pen is down */

      wd_start(&priv->wdog, ADS7843E_WDOG_DELAY,
               ads7843e_wdog, (wdparm_t)priv);

      /* Check the thresholds.  Bail if there is no significant difference */

      if (xdiff < CONFIG_ADS7843E_THRESHX && ydiff < CONFIG_ADS7843E_THRESHY)
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

  /* Notify any waiters that new ADS7843E data is available */

  ads7843e_notify(priv);

  /* Exit, re-enabling ADS7843E interrupts */

ignored:

  /* Re-enable the PENIRQ interrupt at the ADS7843E */

  ads7843e_sendcmd(priv, ADS7843_CMD_ENABPENIRQ);

  /* Re-enable the PENIRQ interrupt at the MCU's interrupt controller */

  config->enable(config, true);

  /* Release our lock on the state structure and unlock the SPI bus */

  nxmutex_unlock(&priv->devlock);
  ads7843e_unlock(priv->spi);
}

/****************************************************************************
 * Name: ads7843e_interrupt
 ****************************************************************************/

static int ads7843e_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ads7843e_dev_s    *priv;
  FAR struct ads7843e_config_s *config;
  int                           ret;

  /* Which ADS7843E device caused the interrupt? */

#ifndef CONFIG_ADS7843E_MULTIPLE
  priv = &g_ads7843e;
#else
  for (priv = g_ads7843elist;
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

  ret = ads7843e_schedule(priv);

  /* Clear any pending interrupts and return success */

  config->clear(config);
  return ret;
}

/****************************************************************************
 * Name: ads7843e_open
 ****************************************************************************/

static int ads7843e_open(FAR struct file *filep)
{
#ifdef CONFIG_ADS7843E_REFCNT
  FAR struct inode          *inode;
  FAR struct ads7843e_dev_s *priv;
  uint8_t                   tmp;
  int                       ret;

  iinfo("Opening\n");

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ads7843e_dev_s *)inode->i_private;

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
 * Name: ads7843e_close
 ****************************************************************************/

static int ads7843e_close(FAR struct file *filep)
{
#ifdef CONFIG_ADS7843E_REFCNT
  FAR struct inode          *inode;
  FAR struct ads7843e_dev_s *priv;
  int                       ret;

  iinfo("Closing\n");
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ads7843e_dev_s *)inode->i_private;

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
 * Name: ads7843e_read
 ****************************************************************************/

static ssize_t ads7843e_read(FAR struct file *filep, FAR char *buffer,
                             size_t len)
{
  FAR struct inode          *inode;
  FAR struct ads7843e_dev_s *priv;
  FAR struct touch_sample_s *report;
  struct ads7843e_sample_s   sample;
  int                        ret;

  iinfo("buffer:%p len:%d\n", buffer, len);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ads7843e_dev_s *)inode->i_private;

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

  ret = ads7843e_sample(priv, &sample);
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

      ret = ads7843e_waitsample(priv, &sample);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          ierr("ERROR: ads7843e_waitsample: %d\n", ret);
          goto errout;
        }
    }

  /* In any event, we now have sampled ADS7843E data that we can report
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
 * Name: ads7843e_ioctl
 ****************************************************************************/

static int ads7843e_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode          *inode;
  FAR struct ads7843e_dev_s *priv;
  int                       ret;

  iinfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ads7843e_dev_s *)inode->i_private;

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
 * Name: ads7843e_poll
 ****************************************************************************/

static int ads7843e_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  FAR struct inode *inode;
  FAR struct ads7843e_dev_s *priv;
  int ret;
  int i;

  iinfo("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ads7843e_dev_s *)inode->i_private;

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

      for (i = 0; i < CONFIG_ADS7843E_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_ADS7843E_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->penchange)
        {
          ads7843e_notify(priv);
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
 * Name: ads7843e_register
 *
 * Description:
 *   Configure the ADS7843E to use the provided SPI device instance.  This
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

int ads7843e_register(FAR struct spi_dev_s *spi,
                      FAR struct ads7843e_config_s *config, int minor)
{
  FAR struct ads7843e_dev_s *priv;
  char devname[DEV_NAMELEN];
#ifdef CONFIG_ADS7843E_MULTIPLE
  irqstate_t flags;
#endif
  int ret;

  iinfo("spi: %p minor: %d\n", spi, minor);

  /* Debug-only sanity checks */

  DEBUGASSERT(spi != NULL && config != NULL && minor >= 0 && minor < 100);

  /* Create and initialize a ADS7843E device driver instance */

#ifndef CONFIG_ADS7843E_MULTIPLE
  priv = &g_ads7843e;
#else
  priv = (FAR struct ads7843e_dev_s *)
    kmm_malloc(sizeof(struct ads7843e_dev_s));
  if (!priv)
    {
      ierr("ERROR: kmm_malloc(%d) failed\n", sizeof(struct ads7843e_dev_s));
      return -ENOMEM;
    }
#endif

  /* Initialize the ADS7843E device driver instance */

  memset(priv, 0, sizeof(struct ads7843e_dev_s));
  priv->spi     = spi;               /* Save the SPI device handle */
  priv->config  = config;            /* Save the board configuration */
  priv->threshx = INVALID_THRESHOLD; /* Initialize thresholding logic */
  priv->threshy = INVALID_THRESHOLD; /* Initialize thresholding logic */

  /* Initialize mutex & semaphores */

  nxmutex_init(&priv->devlock);      /* Initialize device structure mutex */
  nxsem_init(&priv->waitsem, 0, 0);  /* Initialize pen event wait semaphore */

  /* Make sure that interrupts are disabled */

  config->clear(config);
  config->enable(config, false);

  /* Attach the interrupt handler */

  ret = config->attach(config, ads7843e_interrupt);
  if (ret < 0)
    {
      ierr("ERROR: Failed to attach interrupt\n");
      goto errout_with_priv;
    }

  iinfo("Mode: %d Bits: 8 Frequency: %d\n",
        CONFIG_ADS7843E_SPIMODE, CONFIG_ADS7843E_FREQUENCY);

  /* Lock the SPI bus so that we have exclusive access */

  ads7843e_lock(spi);

  /* Enable the PEN IRQ */

  ads7843e_sendcmd(priv, ADS7843_CMD_ENABPENIRQ);

  /* Unlock the bus */

  ads7843e_unlock(spi);

  /* Register the device as an input device */

  snprintf(devname, sizeof(devname), DEV_FORMAT, minor);
  iinfo("Registering %s\n", devname);

  ret = register_driver(devname, &ads7843e_fops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: register_driver() failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* If multiple ADS7843E devices are supported, then we will need to add
   * this new instance to a list of device instances so that it can be
   * found by the interrupt handler based on the received IRQ number.
   */

#ifdef CONFIG_ADS7843E_MULTIPLE
  flags = enter_critical_section();
  priv->flink    = g_ads7843elist;
  g_ads7843elist = priv;
  leave_critical_section(flags);
#endif

  /* Schedule work to perform the initial sampling and to set the data
   * availability conditions.
   */

  ret = work_queue(HPWORK, &priv->work, ads7843e_worker, priv, 0);
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
#ifdef CONFIG_ADS7843E_MULTIPLE
  kmm_free(priv);
#endif
  return ret;
}
