/****************************************************************************
 * drivers/input/stmpe811_tsc.c
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
 *   "STMPE811 S-Touch advanced resistive touchscreen controller with 8-bit
 *    GPIO expander," Doc ID 14489 Rev 6, CD00186725, STMicroelectronics"
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

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/wqueue.h>
#include <nuttx/random.h>

#include <nuttx/arch.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/stmpe811.h>

#include "stmpe811.h"

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_STMPE811) && !defined(CONFIG_STMPE811_TSC_DISABLE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IO_IT_0                  0x01
#define IO_IT_1                  0x02
#define IO_IT_2                  0x04
#define IO_IT_3                  0x08
#define IO_IT_4                  0x10
#define IO_IT_5                  0x20
#define IO_IT_6                  0x40
#define IO_IT_7                  0x80
#define ALL_IT                   0xFF
#define IOE_JOY_IT               (uint8_t)(IO_IT_3 | IO_IT_4 | IO_IT_5 | IO_IT_6 | IO_IT_7)
#define IOE_TS_IT                (uint8_t)(IO_IT_0 | IO_IT_1 | IO_IT_2)
#define IOE_INMEMS_IT            (uint8_t)(IO_IT_2 | IO_IT_3)

#define EDGE_FALLING             0x01
#define EDGE_RISING              0x02
/* <! The value of the maximal timeout for I2C waiting loops */
#define TIMEOUT_MAX              0x3000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Internal logic */

static void     stmpe811_notify(FAR struct stmpe811_dev_s *priv);
static int      stmpe811_sample(FAR struct stmpe811_dev_s *priv,
                  FAR struct stmpe811_sample_s *sample);
static inline int stmpe811_waitsample(FAR struct stmpe811_dev_s *priv,
                  FAR struct stmpe811_sample_s *sample);

/* Character driver methods */

static int      stmpe811_open(FAR struct file *filep);
static int      stmpe811_close(FAR struct file *filep);
static ssize_t  stmpe811_read(FAR struct file *filep, FAR char *buffer,
                  size_t len);
static int      stmpe811_ioctl(FAR struct file *filep, int cmd,
                  unsigned long arg);
static int      stmpe811_poll(FAR struct file *filep, struct pollfd *fds,
                  bool setup);

/* Initialization logic */

static inline void stmpe811_tscinitialize(FAR struct stmpe811_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_stmpe811fops =
{
  stmpe811_open,    /* open */
  stmpe811_close,   /* close */
  stmpe811_read,    /* read */
  NULL,             /* write */
  NULL,             /* seek */
  stmpe811_ioctl,   /* ioctl */
  stmpe811_poll     /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe811_notify
 *
 * Description:
 *   Notify any threads waiting on touchscreen data that data is now
 *   available for reading.
 *
 ****************************************************************************/

static void stmpe811_notify(FAR struct stmpe811_dev_s *priv)
{
  int i;

  /* If there are threads waiting on poll() for STMPE811 data to become
   * available, then wake them up now.  NOTE: we wake up all waiting threads
   * because we do not know that they are going to do.  If they all try to
   * read the data, then some make end up blocking after all.
   */

  for (i = 0; i < CONFIG_STMPE811_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          iinfo("Report events: %02x\n", fds->revents);
          nxsem_post(fds->sem);
        }
    }

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the STMPE811
       * is no longer available.
       */

      nxsem_post(&priv->waitsem);
    }
}

/****************************************************************************
 * Name: stmpe811_sample
 *
 * Description:
 *   Check if touchscreen sample data is available now and, if so, return
 *   the sample data.  This is part of the stmpe811_read logic.
 *
 * Assumption:
 *   Pre-emption is disable to prevent the worker thread from running.
 *   Otherwise, sampled data may continue to change.
 *
 ****************************************************************************/

static int stmpe811_sample(FAR struct stmpe811_dev_s *priv,
                           FAR struct stmpe811_sample_s *sample)
{
  int ret = -EAGAIN;

  /* Is there new STMPE811 sample data available? */

  if (priv->penchange)
    {
      /* Yes.. the state has changed in some way.  Return a copy of the
       * sampled data.
       */

      memcpy(sample, &priv->sample, sizeof(struct stmpe811_sample_s));

      /* Now manage state transitions */

      if (sample->contact == CONTACT_UP)
        {
          /* The sampling logic has detected pen-up in some condition other
           * than CONTACT_NONE.  Set the next state to CONTACT_NONE:  Further
           * pen-down reports will be ignored.  Increment the ID so that
           * next contact ID will be unique
           */

          priv->sample.contact = CONTACT_NONE;
          priv->sample.valid   = false;
          priv->id++;
        }
      else if (sample->contact == CONTACT_DOWN)
        {
          /* The sampling logic has detected pen-up in some condition other
           * than CONTACT_MOVE. Set the next state to CONTACT_MOVE: Further
           * samples collected while the pen is down will reported as
           * movement events.
           */

         priv->sample.contact = CONTACT_MOVE;
        }

      priv->penchange = false;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: stmpe811_waitsample
 *
 * Description:
 *   Wait for a sample to become available (this is really part of the
 *   stmpe811_read logic).
 *
 ****************************************************************************/

static inline int stmpe811_waitsample(FAR struct stmpe811_dev_s *priv,
                                      FAR struct stmpe811_sample_s *sample)
{
  int ret;

  /* Disable pre-emption to prevent the worker thread from running
   * asynchronously.
   */

  sched_lock();

  /* Now release the semaphore that manages mutually exclusive access to
   * the device structure.  This may cause other tasks to become ready to
   * run, but they cannot run yet because pre-emption is disabled.
   */

  nxsem_post(&priv->exclsem);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is available.
   */

  while (stmpe811_sample(priv, sample) < 0)
    {
      /* Wait for a change in the STMPE811 state */

      priv->nwaiters++;
      ret = nxsem_wait(&priv->waitsem);
      priv->nwaiters--;

      /* When we are re-awakened, pre-emption will again be disabled */

      if (ret < 0)
        {
          ierr("ERROR: nxsem_wait failed: %d\n", ret);
          goto errout;
        }
    }

  /* Re-acquire the semaphore that manages mutually exclusive access to
   * the device structure. We may have to wait here. But we have our sample.
   * Interrupts and pre-emption will be re-enabled while we wait.
   */

  ret = nxsem_wait(&priv->exclsem);

errout:
  /* Restore pre-emption.  We might get suspended here but that is okay
   * because we already have our sample.  Note:  this means that if there
   * were two threads reading from the STMPE811 for some reason, the data
   * might be read out of order.
   */

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: stmpe811_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int stmpe811_open(FAR struct file *filep)
{
#ifdef CONFIG_STMPE811_REFCNT
  FAR struct inode         *inode;
  FAR struct stmpe811_dev_s *priv;
  uint8_t                   tmp;
  int                       ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct stmpe811_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Increment the reference count */

  tmp = priv->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* When the reference increments to 1, this is the first open event
   * on the driver.. and an opportunity to do any one-time initialization.
   */

  /* Save the new open count on success */

  priv->crefs = tmp;

errout_with_sem:
  nxsem_post(&priv->exclsem);
  return ret;
#else
  return OK;
#endif
}

/****************************************************************************
 * Name: stmpe811_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int stmpe811_close(FAR struct file *filep)
{
#ifdef CONFIG_STMPE811_REFCNT
  FAR struct inode         *inode;
  FAR struct stmpe811_dev_s *priv;
  int                       ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct stmpe811_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait failed: %d\n", ret);
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

  nxsem_post(&priv->exclsem);
#endif
  return OK;
}

/****************************************************************************
 * Name: stmpe811_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t stmpe811_read(FAR struct file *filep,
                             FAR char *buffer, size_t len)
{
  FAR struct inode          *inode;
  FAR struct stmpe811_dev_s  *priv;
  FAR struct touch_sample_s *report;
  struct stmpe811_sample_s   sample;
  int                        ret;

  iinfo("len=%d\n", len);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct stmpe811_dev_s *)inode->i_private;

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

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Try to read sample data. */

  ret = stmpe811_sample(priv, &sample);
  if (ret < 0)
    {
      /* Sample data is not available now.  We would ave to wait to get
       * receive sample data.  If the user has specified the O_NONBLOCK
       * option, then just return an error.
       */

      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
        }

      /* Wait for sample data */

      ret = stmpe811_waitsample(priv, &sample);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          goto errout;
        }
    }

  /* In any event, we now have sampled STMPE811 data that we can report
   * to the caller.
   */

  report = (FAR struct touch_sample_s *)buffer;
  memset(report, 0, SIZEOF_TOUCH_SAMPLE_S(1));
  report->npoints            = 1;
  report->point[0].id        = sample.id;
  report->point[0].x         = sample.x;
  report->point[0].y         = sample.y;
  report->point[0].pressure  = sample.z;

  add_ui_randomness((sample.x << 16) ^ (sample.y << 8) ^ sample.z);

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
                                    TOUCH_POS_VALID | TOUCH_PRESSURE_VALID;
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
                                TOUCH_POS_VALID | TOUCH_PRESSURE_VALID;
    }
  else /* if (sample->contact == CONTACT_MOVE) */
    {
      /* Movement of the same contact */

      report->point[0].flags  = TOUCH_MOVE | TOUCH_ID_VALID |
                                TOUCH_POS_VALID | TOUCH_PRESSURE_VALID;
    }

  ret = SIZEOF_TOUCH_SAMPLE_S(1);

errout:
  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: stmpe811_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int stmpe811_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode          *inode;
  FAR struct stmpe811_dev_s *priv;
  int                        ret;

  iinfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct stmpe811_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      case TSIOC_SETFREQUENCY:  /* arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(priv->config != NULL && ptr != NULL);
          priv->config->frequency = *ptr;
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

  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: stmpe811_poll
 *
 * Description:
 *   Standard character driver poll method.
 *
 ****************************************************************************/

static int stmpe811_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  FAR struct inode          *inode;
  FAR struct stmpe811_dev_s *priv;
  int                        ret;
  int                        i;

  iinfo("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct stmpe811_dev_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      ierr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ierr("ERROR: Missing POLLIN: revents: %08x\n", fds->revents);
          ret = -EDEADLK;
          goto errout;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_STMPE811_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_STMPE811_NPOLLWAITERS)
        {
          ierr("ERROR: No available slot found: %d\n", i);
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->penchange)
        {
          stmpe811_notify(priv);
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
  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: stmpe811_timeoutworker
 *
 * Description:
 *   A timer has expired without receiving a pen up event.  Check again.
 *
 ****************************************************************************/

static void stmpe811_timeoutworker(FAR void *arg)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)arg;

  DEBUGASSERT(priv);

  /* Treat the timeout just like an interrupt occurred */

  stmpe811_tscworker(priv, stmpe811_getreg8(priv, STMPE811_INT_STA));
}

/****************************************************************************
 * Name: stmpe811_timeout
 *
 * Description:
 *   A timer has expired without receiving a pen up event.  Schedule work
 *   to check again.
 *
 ****************************************************************************/

static void stmpe811_timeout(wdparm_t arg)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)arg;
  int ret;

  /* Are we still stuck in the pen down state? */

  if (priv->sample.contact == CONTACT_DOWN ||
      priv->sample.contact == CONTACT_MOVE)
    {
      /* Yes... is the worker thread available?   If not, then apparently
       * we have work already pending?
       */

      if (work_available(&priv->timeout))
        {
          /* Yes.. Transfer processing to the worker thread.  Since STMPE811
           * interrupts are disabled while the work is pending, no special
           * action should be required to protect the work queue.
           */

          ret = work_queue(HPWORK, &priv->timeout,
                           stmpe811_timeoutworker, priv, 0);
          if (ret != 0)
            {
              ierr("ERROR: Failed to queue work: %d\n", ret);
            }
        }
    }
}

/****************************************************************************
 * Name: stmpe811_tscinitialize
 *
 * Description:
 *   Initialize the touchscreen controller.  This is really a part of the
 *   stmpe811_register logic,
 *
 ****************************************************************************/

static inline void stmpe811_tscinitialize(FAR struct stmpe811_dev_s *priv)
{
  uint8_t regval;

  iinfo("Initializing touchscreen controller\n");

  /* Enable TSC and ADC functions */

  regval = stmpe811_getreg8(priv, STMPE811_SYS_CTRL2);
  regval &= ~(SYS_CTRL2_TSC_OFF | SYS_CTRL2_ADC_OFF);
  stmpe811_putreg8(priv, STMPE811_SYS_CTRL2, regval);

  /* Enable the TSC global interrupts */

  regval  = stmpe811_getreg8(priv, STMPE811_INT_EN);
  regval |= (uint32_t)(INT_TOUCH_DET | INT_FIFO_TH | INT_FIFO_OFLOW);
  stmpe811_putreg8(priv, STMPE811_INT_EN, regval);

  /* Select Sample Time, bit number and ADC Reference */

  stmpe811_putreg8(priv, STMPE811_ADC_CTRL1, priv->config->ctrl1);

  /* Wait for 20 ms */

  up_mdelay(20);

  /* Select the ADC clock speed */

  stmpe811_putreg8(priv, STMPE811_ADC_CTRL2, priv->config->ctrl2);

  /* Select TSC pins in non-GPIO mode (AF=0) */

  regval  = stmpe811_getreg8(priv, STMPE811_GPIO_AF);
  regval &= ~(uint8_t)TSC_PIN_SET;
  stmpe811_putreg8(priv, STMPE811_GPIO_AF, regval);

  /* Select 2 nF filter capacitor */

  stmpe811_putreg8(priv, STMPE811_TSC_CFG, TSC_CFG_AVE_CTRL_4SAMPLES |
                   TSC_CFG_TOUCH_DELAY_500US | TSC_CFG_SETTLING_500US);

  /* Select single point reading */

  stmpe811_putreg8(priv, STMPE811_FIFO_TH, 1);

  /* Reset and clear the FIFO. */

  stmpe811_putreg8(priv, STMPE811_FIFO_STA, FIFO_STA_FIFO_RESET);
  stmpe811_putreg8(priv, STMPE811_FIFO_STA, 0);

  /* set the data format for Z value: 7 fractional part and 1 whole part */

  stmpe811_putreg8(priv, STMPE811_TSC_FRACTIONZ, 0x01);

  /* Set the driving capability of the device for TSC pins: 50mA */

  stmpe811_putreg8(priv, STMPE811_TSC_IDRIVE, TSC_IDRIVE_50MA);

  /* Enable the TSC.  Use no tracking index, touch-screen controller
   * operation mode (XYZ).
   */

  stmpe811_putreg8(priv, STMPE811_TSC_CTRL, TSC_CTRL_EN);

  /* Clear all the status pending bits */

  stmpe811_putreg8(priv, STMPE811_INT_STA, INT_ALL);
}

/****************************************************************************
 *  Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe811_register
 *
 * Description:
 *  Enable TSC functionality.  GPIO4-7 must be available.  This function
 *  will register the touchscreen driver as /dev/inputN where N is the minor
 *  device number
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_register
 *   minor     - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stmpe811_register(STMPE811_HANDLE handle, int minor)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  char devname[DEV_NAMELEN];
  int ret;

  iinfo("handle=%p minor=%d\n", handle, minor);
  DEBUGASSERT(priv);

  /* Get exclusive access to the device structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Make sure that the pins (4-7) need by the TSC are not already in use */

  if ((priv->inuse & TSC_PIN_SET) != 0)
    {
      ierr("ERROR: TSC pins is already in-use: %02x\n", priv->inuse);
      nxsem_post(&priv->exclsem);
      return -EBUSY;
    }

  /* Initialize the TS structure fields to their default values */

  priv->minor     = minor;
  priv->penchange = false;
  priv->threshx   = 0;
  priv->threshy   = 0;

  /* Register the character driver */

  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, minor);
  ret = register_driver(devname, &g_stmpe811fops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: Failed to register driver %s: %d\n", devname, ret);
      nxsem_post(&priv->exclsem);
      return ret;
    }

  /* Initialize the touchscreen controller */

  stmpe811_tscinitialize(priv);

  /* Inidicate that the touchscreen controller was successfully initialized */

  priv->inuse |= TSC_PIN_SET;                    /* Pins 4-7 are now in-use */
  priv->flags |= STMPE811_FLAGS_TSC_INITIALIZED; /* TSC function is initialized */
  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: stmpe811_tscworker
 *
 * Description:
 *   This function is called to handle a TSC interrupt.  It is not really
 *   an interrupt handle because it is called from the STMPE811 "bottom half"
 *   logic that runs on the worker thread.
 *
 ****************************************************************************/

void stmpe811_tscworker(FAR struct stmpe811_dev_s *priv, uint8_t intsta)
{
  uint16_t xdiff;    /* X difference used in thresholding */
  uint16_t ydiff;    /* Y difference used in thresholding */
  uint16_t x;        /* X position */
  uint16_t y;        /* Y position */
  bool     pendown;  /* true: pen is down */

  DEBUGASSERT(priv != NULL);

  /* Cancel the missing pen up timer */

  wd_cancel(&priv->wdog);

  /* Check for pen up or down from the TSC_STA bit in STMPE811_TSC_CTRL. */

  pendown = !!(stmpe811_getreg8(priv, STMPE811_TSC_CTRL) & TSC_CTRL_TSC_STA);

  /* Handle the change from pen down to pen up */

  if (!pendown)
    {
      /* The pen is up.. reset thresholding variables.  FIFOs will read zero
       * if there is no data available (hence the choice of (0,0))
       */

      priv->threshx = 0;
      priv->threshy = 0;

      /* Ignore the interrupt if the pen was already up (CONTACT_NONE == pen
       * up and already reported; CONTACT_UP == pen up, but not reported)
       */

      if (priv->sample.contact == CONTACT_NONE ||
          priv->sample.contact == CONTACT_UP)
        {
          goto ignored;
        }

      /* A pen-down to up transition has been detected.  CONTACT_UP indicates
       * the initial loss of contact.  The state will be changed to
       * CONTACT_NONE after the loss of contact is sampled.
       */

       priv->sample.contact = CONTACT_UP;
    }

  /* The pen is down... check for data in the FIFO */

  else if ((intsta & (INT_FIFO_TH | INT_FIFO_OFLOW)) != 0)
    {
      /* Read the next x and y positions from the FIFO. */

#ifdef CONFIG_STMPE811_SWAPXY
      x = stmpe811_getreg16(priv, STMPE811_TSC_DATAX);
      y = stmpe811_getreg16(priv, STMPE811_TSC_DATAY);
#else
      x = stmpe811_getreg16(priv, STMPE811_TSC_DATAY);
      y = stmpe811_getreg16(priv, STMPE811_TSC_DATAX);
#endif

      /* If we have not yet processed the last pen up event, then we
       * cannot handle this pen down event. We will have to discard it.  That
       * should be okay because there will be another FIFO event right behind
       * this one.  Other kinds of data overruns are not harmful.
       *
       * Hmm.. a better design might be to disable FIFO interrupts when we
       * detect pen up.  Then re-enable them when CONTACT_UP is reported.
       * That would save processing interrupts just to discard the data.
       */

      if (priv->sample.contact == CONTACT_UP)
        {
          /* We have not closed the loop on the last touch ... don't report
           * anything.
           */

          goto ignored;
        }

      /* Perform a thresholding operation so that the results will be more
       * stable. If the difference from the last sample is small, then ignore
       * the event. REVISIT:  Should a large change in pressure also generate
       * a event?
       */

      xdiff = x > priv->threshx ? (x - priv->threshx) : (priv->threshx - x);
      ydiff = y > priv->threshy ? (y - priv->threshy) : (priv->threshy - y);

      if (xdiff < CONFIG_STMPE811_THRESHX && ydiff < CONFIG_STMPE811_THRESHY)
        {
          /* Little or no change in either direction ...
           * don't report anything.
           */

          goto ignored;
        }

      /* When we see a big difference, snap to the new x/y thresholds */

      priv->threshx       = x;
      priv->threshy       = y;

      /* Update the x/y position in the sample data */

      priv->sample.x      = priv->threshx;
      priv->sample.y      = priv->threshy;

      /* Update the Z pressure index */

      priv->sample.z      = stmpe811_getreg8(priv, STMPE811_TSC_DATAZ);
      priv->sample.valid  = true;

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

  /* Pen down, but no data in FIFO */

  else
    {
      /* Ignore the interrupt... wait until there is data in the FIFO */

      goto ignored;
    }

  /* We get here if (1) we just went from a pen down to a pen up state OR (2)
   * We just get a measurement from the FIFO in a pen down state.  Indicate
   * the availability of new sample data for this ID.
   */

  priv->sample.id = priv->id;
  priv->penchange = true;

  /* Notify any waiters that new STMPE811 data is available */

  stmpe811_notify(priv);

  /* If we think that the pen is still down, the start/re-start the pen up
   * timer.
   */

ignored:
  if (priv->sample.contact == CONTACT_DOWN ||
      priv->sample.contact == CONTACT_MOVE)
    {
      wd_start(&priv->wdog, STMPE811_PENUP_TICKS,
               stmpe811_timeout, (wdparm_t)priv);
    }

  /*  Reset and clear all data in the FIFO */

  stmpe811_putreg8(priv, STMPE811_FIFO_STA, FIFO_STA_FIFO_RESET);
  stmpe811_putreg8(priv, STMPE811_FIFO_STA, 0);
}

#endif /* CONFIG_INPUT && CONFIG_INPUT_STMPE811 && !CONFIG_STMPE811_TSC_DISABLE */
