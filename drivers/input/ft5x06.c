/****************************************************************************
 * drivers/input/ft5x06.c
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
 *   "FT5x06", FocalTech Systems Co., Ltd, D-FT5x06-1212-V4.0, Revised
 *   Dec. 18, 2012
 */

/* The FT5x06 Series ICs are single-chip capacitive touch panel controller
 * ICs with a built-in 8 bit Micro-controller unit (MCU).  They adopt the
 * mutual capacitance approach, which supports true multi-touch capability.
 * In conjunction with a mutual capacitive touch panel, the FT5x06 have
 * user-friendly input functions, which can be applied on many portable
 * devices, such as cellular phones, MIDs, netbook and notebook personal
 * computers.
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
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#include <nuttx/wdog.h>

#include <nuttx/input/touchscreen.h>
#include <nuttx/input/ft5x06.h>

#include "ft5x06.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/input[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT     "/dev/input%d"
#define DEV_NAMELEN    16

/* In polled mode, the polling rate will decrease when there is no touch
 * activity.  These definitions represent the maximum and the minimum
 * polling rates.
 */

#define POLL_MINDELAY  MSEC2TICK(50)
#define POLL_MAXDELAY  MSEC2TICK(200)
#define POLL_INCREMENT MSEC2TICK(10)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of one FT5x06 driver instance */

struct ft5x06_dev_s
{
  uint8_t crefs;                            /* Number of times the device
                                             * has been opened */
  uint8_t nwaiters;                         /* Number of threads waiting for
                                             * FT5x06 data */
  volatile bool valid;                      /* True:  New, valid touch data
                                             * in touchbuf[] */
#ifdef CONFIG_FT5X06_SINGLEPOINT
  uint8_t lastid;                           /* Last reported touch id */
  uint8_t lastevent;                        /* Last reported event */
  int16_t lastx;                            /* Last reported X position */
  int16_t lasty;                            /* Last reported Y position */
#endif
  mutex_t devlock;                          /* Manages exclusive access to
                                             * this structure */
  sem_t waitsem;                            /* Used to wait for the
                                             * availability of data */
  uint32_t frequency;                       /* Current I2C frequency */
#ifdef CONFIG_FT5X06_POLLMODE
  uint32_t delay;                           /* Current poll delay */
#endif

  FAR const struct ft5x06_config_s *config; /* Board configuration data */
  FAR struct i2c_master_s *i2c;             /* Saved I2C driver instance */
  struct work_s work;                       /* Supports the interrupt
                                             * handling "bottom half" */
#ifdef CONFIG_FT5X06_POLLMODE
  struct wdog_s polltimer;                  /* Poll timer */
#endif
  uint8_t touchbuf[FT5X06_TOUCH_DATA_LEN];  /* Raw touch data */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_FT5X06_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ft5x06_notify(FAR struct ft5x06_dev_s *priv);
static void ft5x06_data_worker(FAR void *arg);
#ifdef CONFIG_FT5X06_POLLMODE
static void ft5x06_poll_timeout(wdparm_t arg);
#else
static int  ft5x06_data_interrupt(int irq, FAR void *context, FAR void *arg);
#endif
static ssize_t ft5x06_sample(FAR struct ft5x06_dev_s *priv, FAR char *buffer,
                             size_t len);
static ssize_t ft5x06_waitsample(FAR struct ft5x06_dev_s *priv,
                                 FAR char *buffer, size_t len);
static int  ft5x06_bringup(FAR struct ft5x06_dev_s *priv);
static void ft5x06_shutdown(FAR struct ft5x06_dev_s *priv);

/* Character driver methods */

static int  ft5x06_open(FAR struct file *filep);
static int  ft5x06_close(FAR struct file *filep);
static ssize_t ft5x06_read(FAR struct file *filep, FAR char *buffer,
                           size_t len);
static int  ft5x06_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);
static int  ft5x06_poll(FAR struct file *filep, struct pollfd *fds,
                        bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_ft5x06_fops =
{
  ft5x06_open,    /* open */
  ft5x06_close,   /* close */
  ft5x06_read,    /* read */
  NULL,           /* write */
  NULL,           /* seek */
  ft5x06_ioctl,   /* ioctl */
  NULL,           /* mmap */
  NULL,           /* truncate */
  ft5x06_poll     /* poll */
};

/* Maps FT5x06 touch events into bit encoded representation used by NuttX */

static const uint8_t g_event_map[4] =
{
  (TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID), /* FT5X06_DOWN */
  (TOUCH_UP   | TOUCH_ID_VALID),                   /* FT5X06_UP */
  (TOUCH_MOVE | TOUCH_ID_VALID | TOUCH_POS_VALID), /* FT5X06_CONTACT */
  TOUCH_ID_VALID                                   /* FT5X06_INVALID */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ft5x06_notify
 ****************************************************************************/

static void ft5x06_notify(FAR struct ft5x06_dev_s *priv)
{
  /* If there are threads waiting on poll() for FT5x06 data to become
   * available, then wake them up now.  NOTE: we wake up all waiting threads
   * because we do not know that they are going to do.  If they all try to
   * read the data, then some make end up blocking after all.
   */

  poll_notify(priv->fds, CONFIG_FT5X06_NPOLLWAITERS, POLLIN);

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the FT5x06
       * is no longer available.
       */

      nxsem_post(&priv->waitsem);
    }
}

/****************************************************************************
 * Name: ft5x06_data_worker
 ****************************************************************************/

static void ft5x06_data_worker(FAR void *arg)
{
  FAR struct ft5x06_dev_s *priv = (FAR struct ft5x06_dev_s *)arg;
  FAR const struct ft5x06_config_s *config;
  FAR struct ft5x06_touch_data_s *sample;
  struct i2c_msg_s msg[2];
  uint8_t regaddr;
  int ret;

  /* Get a pointer the callbacks for convenience */

  DEBUGASSERT(priv != NULL && priv->config != NULL);
  config = priv->config;

  /* We need to have exclusive access to the touchbuf so that we do not
   * corrupt any read operation that is in place.
   */

  nxmutex_lock(&priv->devlock);

  /* Read touch data */

  /* Set up the address write operation */

  regaddr          = FT5X06_TOUCH_DATA_STARTREG;

  msg[0].frequency = priv->frequency;       /* I2C frequency */
  msg[0].addr      = config->address;       /* 7-bit address */
  msg[0].flags     = 0;                     /* Write transaction with START */
  msg[0].buffer    = &regaddr;              /* Send one byte of data (no STOP) */
  msg[0].length    = 1;

  /* Set up the data read operation.
   *
   * REVISIT:  If CONFIG_FT5X06_SINGLEPOINT is selected, could we not just
   * set the length for one sample?  Or is there some reason why we have to
   * read all of the points?
   */

  msg[1].frequency = priv->frequency;       /* I2C frequency */
  msg[1].addr      = config->address;       /* 7-bit address */
  msg[1].flags     = I2C_M_READ;            /* Read transaction with Re-START */
  msg[1].buffer    = priv->touchbuf;        /* Read all touch data */
  msg[1].length    = FT5X06_TOUCH_DATA_LEN;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret >= 0)
    {
      /* In polled mode, we may read invalid touch data.  If there is
       * no touch data, the FT5x06 returns all 0xff the very first time.
       * After that, it returns the same old stale data when there is
       * no touch data.
       */

      sample = (FAR struct ft5x06_touch_data_s *)priv->touchbuf;

      /* Notify waiters (only if we ready some valid data).
       *
       * REVISIT: For improved performance consider moving the duplicate
       * report and thresholding logic from ft5x06_sample() to here.  That
       * would save a context switch.
       */

      if (sample->tdstatus <= FT5X06_MAX_TOUCHES)
        {
          /* Notify any waiters that new FT5x06 data is available */

          priv->valid = true;
          ft5x06_notify(priv);
        }

#ifdef CONFIG_FT5X06_POLLMODE
      /* Update the poll rate */

      if (sample->tdstatus > 0 && sample->tdstatus <= FT5X06_MAX_TOUCHES)
        {
          /* Keep it at the minimum if touches are detected. */

          priv->delay = POLL_MINDELAY;
        }
      else if (priv->delay < POLL_MAXDELAY)
        {
          /* Otherwise, let the poll rate rise gradually up to the maximum
           * if there is no touch.
           */

          priv->delay += POLL_INCREMENT;
        }
#endif
    }

#ifdef CONFIG_FT5X06_POLLMODE
  /* Exit, re-starting the poll. */

  wd_start(&priv->polltimer, priv->delay,
           ft5x06_poll_timeout, (wdparm_t)priv);

#else
  /* Exit, re-enabling FT5x06 interrupts */

  config->enable(config, true);
#endif

  nxmutex_unlock(&priv->devlock);
}

/****************************************************************************
 * Name: ft5x06_poll_timeout
 ****************************************************************************/

#ifdef CONFIG_FT5X06_POLLMODE
static void ft5x06_poll_timeout(wdparm_t arg)
{
  FAR struct ft5x06_dev_s *priv = (FAR struct ft5x06_dev_s *)arg;
  int ret;

  /* Transfer processing to the worker thread.  Since FT5x06 poll timer is
   * disabled while the work is pending, no special action should be
   * required to protected the work queue.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, ft5x06_data_worker, priv, 0);
  if (ret != 0)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
    }
}
#endif

/****************************************************************************
 * Name: ft5x06_data_interrupt
 ****************************************************************************/

#ifndef CONFIG_FT5X06_POLLMODE
static int ft5x06_data_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct ft5x06_dev_s *priv = (FAR struct ft5x06_dev_s *)arg;
  FAR const struct ft5x06_config_s *config;
  int ret;

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Disable further interrupts */

  config->enable(config, false);

  /* Transfer processing to the worker thread.  Since FT5x06 interrupts are
   * disabled while the work is pending, no special action should be required
   * to protected the work queue.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, ft5x06_data_worker, priv, 0);
  if (ret != 0)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
    }

  /* Clear any pending interrupts and return success */

  config->clear(config);
  return OK;
}
#endif

/****************************************************************************
 * Name: ft5x06_sample
 ****************************************************************************/

#ifdef CONFIG_FT5X06_SINGLEPOINT
static ssize_t ft5x06_sample(FAR struct ft5x06_dev_s *priv, FAR char *buffer,
                             size_t len)
{
  FAR struct ft5x06_touch_data_s *raw;
  FAR struct ft5x06_touch_point_s *touch;
  FAR struct touch_sample_s *sample;
  FAR struct touch_point_s *point;
  int16_t x;
  int16_t y;
  uint8_t event;
  uint8_t id;

  if (!priv->valid)
    {
      return 0;  /* Nothing to read */
    }

  /* Raw data pointers (source) */

  raw = (FAR struct ft5x06_touch_data_s *)priv->touchbuf;
  touch = raw->touch;

  /* Get the reported X and Y positions */

  #ifdef CONFIG_FT5X06_SWAPXY
  y = TOUCH_POINT_GET_X(touch[0]);
  x = TOUCH_POINT_GET_Y(touch[0]);
#else
  x = TOUCH_POINT_GET_X(touch[0]);
  y = TOUCH_POINT_GET_Y(touch[0]);
#endif

  /* Get the touch point ID and event */

  event = TOUCH_POINT_GET_EVENT(touch[0]);
  id    = TOUCH_POINT_GET_ID(touch[0]);

  if (event == FT5X06_INVALID)
    {
      priv->lastevent = FT5X06_INVALID;
      goto reset_and_drop;
    }

  if (id == priv->lastid && event == priv->lastevent)
    {
      /* Same ID and event..  Is there positional data? */

      if (raw->tdstatus == 0 || event == FT5X06_UP)
        {
          /* No... no new touch data */

          goto reset_and_drop;
        }
      else
        {
          int16_t deltax;
          int16_t deltay;

          /* Compare the change in position from the last report. */

          deltax = (x - priv->lastx);
          if (deltax < 0)
            {
              deltax = -deltax;
            }

          if (deltax < CONFIG_FT5X06_THRESHX)
            {
              /* There as been no significant change in X, try Y */

              deltay = (y - priv->lasty);
              if (deltay < 0)
                {
                  deltay = -deltay;
                }

              if (deltax < CONFIG_FT5X06_THRESHX)
                {
                  /* Ignore... no significant change in Y either */

                  goto drop;
                }
            }
        }
    }

  priv->lastid      = id;
  priv->lastevent   = event;
  priv->lastx       = x;
  priv->lasty       = y;

  /* User data buffer points (sink) */

  /* Return the number of touches read */

  sample            = (FAR struct touch_sample_s *)buffer;
  sample->npoints   = 1;

  /* Decode and return the single touch point */

  point             = sample->point;
  point[0].id       = id;
  point[0].flags    = g_event_map[event];
  point[0].x        = x;
  point[0].y        = y;
  point[0].h        = 0;
  point[0].w        = 0;
  point[0].pressure = 0;

  priv->valid       = false;
  return SIZEOF_TOUCH_SAMPLE_S(1);

reset_and_drop:
  priv->lastx = 0;
  priv->lasty = 0;
drop:
  priv->valid = false;
  return 0;  /* No new touches read. */
}
#else
static ssize_t ft5x06_sample(FAR struct ft5x06_dev_s *priv, FAR char *buffer,
                             size_t len)
{
  FAR struct ft5x06_touch_data_s *raw;
  FAR struct ft5x06_touch_point_s *touch;
  FAR struct touch_sample_s *sample;
  FAR struct touch_point_s *point;
  unsigned int maxtouches;
  unsigned int ntouches;
  int i;

  maxtouches = (len - sizeof(int)) / sizeof(struct touch_point_s);
  DEBUGASSERT(maxtouches > 0);  /* Already verified */

  if (!priv->valid)
    {
      return 0;  /* Nothing to read */
    }

  /* Raw data pointers (source) */

  raw      = (FAR struct ft5x06_touch_data_s *)priv->touchbuf;
  touch    = raw->touch;

  /* Decode number of touches */

  ntouches = raw->tdstatus;
  DEBUGASSERT(ntouches <= FT5X06_MAX_TOUCHES);

  if (ntouches > maxtouches)
    {
      ntouches = maxtouches;
    }

  if (ntouches < 1)
    {
      priv->valid = false;
      return 0;  /* No touches read. */
    }

  /* User data buffer points (sink) */

  sample = (FAR struct touch_sample_s *)buffer;
  point  = sample->point;

  /* Return the number of touches read */

  sample->npoints = ntouches;

  /* Decode and return the touch points */

  for (i = 0; i < ntouches; i++)
    {
      int event         = TOUCH_POINT_GET_EVENT(touch[i]);

      point[i].id       = TOUCH_POINT_GET_ID(touch[i]);
      point[i].flags    = g_event_map[event];
#ifdef CONFIG_FT5X06_SWAPXY
      point[i].y        = TOUCH_POINT_GET_X(touch[i]);
      point[i].x        = TOUCH_POINT_GET_Y(touch[i]);
#else
      point[i].x        = TOUCH_POINT_GET_X(touch[i]);
      point[i].y        = TOUCH_POINT_GET_Y(touch[i]);
#endif
      point[i].h        = 0;
      point[i].w        = 0;
      point[i].pressure = 0;
    }

  priv->valid = false;
  return SIZEOF_TOUCH_SAMPLE_S(ntouches);
}
#endif /* CONFIG_FT5X06_SINGLEPOINT */

/****************************************************************************
 * Name: ft5x06_waitsample
 ****************************************************************************/

static ssize_t ft5x06_waitsample(FAR struct ft5x06_dev_s *priv,
                                 FAR char *buffer, size_t len)
{
  int ret;

  /* Disable pre-emption to prevent other threads from getting control while
   * we muck with the semaphores.
   */

  sched_lock();

  /* Now release the semaphore that manages mutually exclusive access to
   * the device structure.  This may cause other tasks to become ready to
   * run, but they cannot run yet because pre-emption is disabled.
   */

  nxmutex_unlock(&priv->devlock);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is available.
   */

  while (!priv->valid)
    {
      /* Increment the count of waiters */

      priv->nwaiters++;

      /* Wait for a change in the FT5x06 state */

      ret = nxsem_wait(&priv->waitsem);
      priv->nwaiters--;

      if (ret < 0)
        {
          ierr("ERROR: nxsem_wait failed: %d\n", ret);
          goto errout;
        }
    }

  /* Re-acquire the semaphore that manages mutually exclusive access to
   * the device structure.  We may have to wait here.  But we have our
   * sample.  Interrupts and pre-emption will be re-enabled while we wait.
   */

  ret = nxmutex_lock(&priv->devlock);
  if (ret >= 0)
    {
      /* Now sample the data.
       *
       * REVISIT:  Is it safe to assume that priv->valid will always be
       * true?  I think that sched_lock() would protect the setting.
       */

      ret = ft5x06_sample(priv, buffer, len);
    }

errout:
  /* Restore pre-emption.  We might get suspended here but that is okay
   * because we already have our sample.  Note:  this means that if there
   * were two threads reading from the FT5x06 for some reason, the data
   * might be read out of order.
   */

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: ft5x06_bringup
 ****************************************************************************/

static int ft5x06_bringup(FAR struct ft5x06_dev_s *priv)
{
  FAR const struct ft5x06_config_s *config;
  struct i2c_msg_s msg;
  uint8_t data[2];
  int ret;

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Set device mode to normal operation */

  data[0]       = FT5X06_TOUCH_MODE_REG;   /* Register address */
  data[1]       = FT5X06_DEV_MODE_WORKING; /* Normal mode */

  msg.frequency = priv->frequency;         /* I2C frequency */
  msg.addr      = config->address;         /* 7-bit address */
  msg.flags     = 0;                       /* Write transaction with START */
  msg.buffer    = data;                    /* Send two bytes followed by STOP */
  msg.length    = 2;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      return ret;
    }

#ifndef CONFIG_FT5X06_POLLMODE
  /* Enable FT5x06 interrupts */

  config->clear(config);
  config->enable(config, true);
#endif
  return OK;
}

/****************************************************************************
 * Name: ft5x06_shutdown
 ****************************************************************************/

static void ft5x06_shutdown(FAR struct ft5x06_dev_s *priv)
{
#ifdef CONFIG_FT5X06_POLLMODE
  /* Stop the poll timer */

  wd_cancel(&priv->polltimer);

#else
  FAR const struct ft5x06_config_s *config = priv->config;

  /* Make sure that the FT5x06 interrupt is disabled */

  config->clear(config);
  config->enable(config, false);
#endif
}

/****************************************************************************
 * Name: ft5x06_open
 ****************************************************************************/

static int ft5x06_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct ft5x06_dev_s *priv;
  uint8_t tmp;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ft5x06_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("ERROR: nxmutex_lock failed: %d\n", ret);
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
   * on the driver.. and the time when we must initialize the driver.
   */

  if (tmp == 1)
    {
      ret = ft5x06_bringup(priv);
      if (ret < 0)
        {
          ierr("ERROR: ft5x06_bringup failed: %d\n", ret);
          goto errout_with_lock;
        }
    }

  /* Save the new open count on success */

  priv->crefs = tmp;

errout_with_lock:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: ft5x06_close
 ****************************************************************************/

static int ft5x06_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct ft5x06_dev_s *priv;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ft5x06_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  /* Decrement the reference count unless it would decrement a negative
   * value.
   */

  if (priv->crefs >= 1)
    {
      priv->crefs--;
    }

  /* When the count decrements to zero, there are no further open references
   * to the driver and it can be uninitialized.
   */

  if (priv->crefs == 0)
    {
      ft5x06_shutdown(priv);
    }

  nxmutex_unlock(&priv->devlock);
  return OK;
}

/****************************************************************************
 * Name: ft5x06_read
 ****************************************************************************/

static ssize_t ft5x06_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct inode *inode;
  FAR struct ft5x06_dev_s *priv;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ft5x06_dev_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the touch data.
   */

  if (len < SIZEOF_TOUCH_SAMPLE_S(1))
    {
      /* We could provide logic to break up a touch report into segments and
       * handle smaller reads... but why?
       */

      return -ENOSYS;
    }

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  /* Try to read sample data. */

  ret = ft5x06_sample(priv, buffer, len);
  while (ret == 0)
    {
      /* Sample data is not available now.  We would have to wait to receive
       * sample data.  If the user has specified the O_NONBLOCK option, then
       * just return an error.
       */

      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
        }

      /* Wait for sample data */

      ret = ft5x06_waitsample(priv, buffer, len);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          goto errout;
        }
    }

  ret = SIZEOF_TOUCH_SAMPLE_S(1);

errout:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: ft5x06_ioctl
 ****************************************************************************/

static int ft5x06_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode        *inode;
  FAR struct ft5x06_dev_s *priv;
  int                      ret;

  iinfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ft5x06_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      case TSIOC_SETFREQUENCY:  /* arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(priv->config != NULL && ptr != NULL);
          priv->frequency = *ptr;
        }
        break;

      case TSIOC_GETFREQUENCY:  /* arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(priv->config != NULL && ptr != NULL);
          *ptr = priv->frequency;
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
 * Name: ft5x06_poll
 ****************************************************************************/

static int ft5x06_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  FAR struct inode        *inode;
  FAR struct ft5x06_dev_s *priv;
  int                      ret;
  int                      i;

  iinfo("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct ft5x06_dev_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ierr("ERROR: Missing POLLIN: revents: %08" PRIx32 "\n",
               fds->revents);
          ret = -EDEADLK;
          goto errout;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_FT5X06_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_FT5X06_NPOLLWAITERS)
        {
          ierr("ERROR: No available slot found: %d\n", i);
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->valid)
        {
          ft5x06_notify(priv);
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ft5x06_register
 *
 * Description:
 *   Configure the FT5x06 to use the provided I2C device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   dev     - An I2C driver instance
 *   config  - Persistent board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int ft5x06_register(FAR struct i2c_master_s *i2c,
                    FAR const struct ft5x06_config_s *config, int minor)
{
  FAR struct ft5x06_dev_s *priv;
  char devname[DEV_NAMELEN];
  int ret;

  iinfo("i2c: %p minor: %d\n", i2c, minor);

  /* Debug-only sanity checks */

  DEBUGASSERT(i2c != NULL && config != NULL && minor >= 0 && minor < 100);
#ifdef CONFIG_FT5X06_POLLMODE
  DEBUGASSERT(config->wakeup != NULL && config->nreset != NULL);
#else
  DEBUGASSERT(config->attach != NULL && config->enable != NULL &&
              config->clear  != NULL && config->wakeup != NULL &&
              config->nreset != NULL);
#endif

  /* Create and initialize a FT5x06 device driver instance */

  priv = (FAR struct ft5x06_dev_s *)kmm_zalloc(sizeof(struct ft5x06_dev_s));
  if (!priv)
    {
      ierr("ERROR: kmm_zalloc(%d) failed\n", sizeof(struct ft5x06_dev_s));
      return -ENOMEM;
    }

  /* Initialize the FT5x06 device driver instance */

  priv->i2c       = i2c;               /* Save the I2C device handle */
  priv->config    = config;            /* Save the board configuration */
  priv->frequency = config->frequency; /* Set the current I2C frequency */

  nxmutex_init(&priv->devlock);        /* Initialize device structure mutex */
  nxsem_init(&priv->waitsem, 0, 0);    /* Initialize pen event wait semaphore */

#ifdef CONFIG_FT5X06_POLLMODE
  /* Allocate a timer for polling the FT5x06 */

  priv->delay     = POLL_MAXDELAY;
#else
  /* Make sure that the FT5x06 interrupt interrupt is disabled */

  config->clear(config);
  config->enable(config, false);

  /* Attach the interrupt handler */

  ret = config->attach(config, ft5x06_data_interrupt,
                       priv);
  if (ret < 0)
    {
      ierr("ERROR: Failed to attach interrupt\n");
      goto errout_with_priv;
    }
#endif

  /* Register the device as an input device */

  snprintf(devname, sizeof(devname), DEV_FORMAT, minor);
  iinfo("Registering %s\n", devname);

  ret = register_driver(devname, &g_ft5x06_fops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: register_driver() failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* Schedule work to perform the initial sampling and to set the data
   * availability conditions.
   */

  ret = work_queue(HPWORK, &priv->work, ft5x06_data_worker, priv, 0);
  if (ret < 0)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
      goto errout_with_priv;
    }

  /* And return success */

  return OK;

errout_with_priv:
  nxmutex_destroy(&priv->devlock);
  nxsem_destroy(&priv->waitsem);
  kmm_free(priv);
  return ret;
}
