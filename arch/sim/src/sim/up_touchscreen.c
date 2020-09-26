/****************************************************************************
 * arch/sim/src/sim/up_touchscreen.c
 *
 *   Copyright (C) 2011-2012, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/nx/nx.h>

#include <nuttx/input/touchscreen.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SIM_TCNWAITERS
#  define CONFIG_SIM_TCNWAITERS 4
#endif

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/input[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/input%d"
#define DEV_NAMELEN  16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This describes the state of one contact */

enum up_contact_3
{
  CONTACT_NONE = 0,                    /* No contact */
  CONTACT_DOWN,                        /* First contact */
  CONTACT_MOVE,                        /* Same contact, possibly different position */
  CONTACT_UP,                          /* Contact lost */
};

/* This structure describes the results of one touchscreen sample */

struct up_sample_s
{
  uint8_t  id;                         /* Sampled touch point ID */
  uint8_t  contact;                    /* Contact state (see enum up_contact_e) */
  uint16_t x;                          /* Measured X position */
  uint16_t y;                          /* Measured Y position */
};

/* This structure describes the state of one touchscreen driver instance */

struct up_dev_s
{
  int eventloop;
  volatile uint8_t nwaiters;           /* Number of threads waiting for touchscreen data */
  uint8_t id;                          /* Current touch point ID */
  uint8_t minor;                       /* Minor device number */
  volatile bool penchange;             /* An unreported event is buffered */
  sem_t devsem;                        /* Manages exclusive access to this structure */
  sem_t waitsem;                       /* Used to wait for the availability of data */

  struct up_sample_s sample;           /* Last sampled touch point data */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_SIM_TCNWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void up_notify(FAR struct up_dev_s *priv);
static int up_sample(FAR struct up_dev_s *priv,
                     FAR struct up_sample_s *sample);
static int up_waitsample(FAR struct up_dev_s *priv,
                         FAR struct up_sample_s *sample);

/* Character driver methods */

static int up_open(FAR struct file *filep);
static int up_close(FAR struct file *filep);
static ssize_t up_read(FAR struct file *filep, FAR char *buffer, size_t len);
static int up_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int up_poll(FAR struct file *filep, struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations up_fops =
{
  up_open,    /* open */
  up_close,   /* close */
  up_read,    /* read */
  NULL,       /* write */
  NULL,       /* seek */
  up_ioctl,   /* ioctl */
  up_poll     /* poll */
};

/* Only one simulated touchscreen is supported so the driver state
 * structure may as well be pre-allocated.
 */

static struct up_dev_s g_simtouchscreen;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_notify
 ****************************************************************************/

static void up_notify(FAR struct up_dev_s *priv)
{
  int i;

  /* If there are threads waiting on poll() for touchscreen data to become
   * available, then wake them up now.  NOTE: we wake up all waiting threads
   * because we do not know what they are going to do.  If they all try to
   * read the data then some make end up blocking after all.
   */

  for (i = 0; i < CONFIG_SIM_TCNWAITERS; i++)
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

  iinfo("contact=%d nwaiters=%d\n", priv->sample.contact, priv->nwaiters);
  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because
       * the touchscreen is no longer available.
       */

      nxsem_post(&priv->waitsem);
    }
}

/****************************************************************************
 * Name: up_sample
 ****************************************************************************/

static int up_sample(FAR struct up_dev_s *priv,
                     FAR struct up_sample_s *sample)
{
  int ret = -EAGAIN;

  /* Is there new touchscreen sample data available? */

  iinfo("penchange=%d contact=%d id=%d\n",
        priv->penchange, sample->contact, priv->id);

  if (priv->penchange)
    {
      /* Yes.. the state has changed in some way.  Return a copy of the
       * sampled data.
       */

      memcpy(sample, &priv->sample, sizeof(struct up_sample_s));

      /* Now manage state transitions */

      if (sample->contact == CONTACT_UP)
        {
          /* Next.. no contract.  Increment the ID so that next contact ID
           * will be unique
           */

          priv->sample.contact = CONTACT_NONE;
          priv->id++;
        }
      else if (sample->contact == CONTACT_DOWN)
        {
          /* First report -- next report will be a movement */

          priv->sample.contact = CONTACT_MOVE;
        }

      priv->penchange = false;
      iinfo("penchange=%d contact=%d id=%d\n",
             priv->penchange, priv->sample.contact, priv->id);

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: up_waitsample
 ****************************************************************************/

static int up_waitsample(FAR struct up_dev_s *priv,
                         FAR struct up_sample_s *sample)
{
  irqstate_t flags;
  int ret;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semphores from interrupt handlers, and (2) to prevent sampled data
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

  nxsem_post(&priv->devsem);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is available.
   */

  while (up_sample(priv, sample) < 0)
    {
      /* Wait for a change in the touchscreen state */

      iinfo("Waiting...\n");
      priv->nwaiters++;
      ret = nxsem_wait(&priv->waitsem);
      priv->nwaiters--;
      iinfo("Awakened...\n");

      if (ret < 0)
        {
          goto errout;
        }
    }

  /* Re-acquire the semaphore that manages mutually exclusive access to the
   * device structure.  We may have to wait here.  But we have our sample.
   * Interrupts and pre-emption will be re-enabled while we wait.
   */

  ret = nxsem_wait(&priv->devsem);

errout:
  /* Then re-enable interrupts.  We might get interrupt here and there
   * could be a new sample.  But no new threads will run because we still
   * have pre-emption disabled.
   */

  leave_critical_section(flags);

  /* Restore pre-emption.  We might get suspended here but that is okay
   * because we already have our sample.  Note:  this means that if there
   * were two threads reading from the touchscreen for some reason, the data
   * might be read out of order.
   */

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: up_open
 ****************************************************************************/

static int up_open(FAR struct file *filep)
{
  iinfo("Opening...\n");
  return OK;
}

/****************************************************************************
 * Name: up_close
 ****************************************************************************/

static int up_close(FAR struct file *filep)
{
  iinfo("Closing...\n");
  return OK;
}

/****************************************************************************
 * Name: up_read
 ****************************************************************************/

static ssize_t up_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode          *inode;
  FAR struct up_dev_s       *priv;
  FAR struct touch_sample_s *report;
  struct up_sample_s         sample;
  int                        ret;

  iinfo("len=%d\n", len);

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct up_dev_s *)inode->i_private;

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

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Try to read sample data. */

  ret = up_sample(priv, &sample);
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

      ret = up_waitsample(priv, &sample);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          goto errout;
        }
    }

  /* In any event, we now have sampled touchscreen data that we can report
   * to the caller.
   */

  report = (FAR struct touch_sample_s *)buffer;
  memset(report, 0, SIZEOF_TOUCH_SAMPLE_S(1));
  report->npoints            = 1;
  report->point[0].id        = priv->id;
  report->point[0].x         = sample.x;
  report->point[0].y         = sample.y;
  report->point[0].h         = 1;
  report->point[0].w         = 1;
  report->point[0].pressure  = 42;

  /* Report the appropriate flags */

  if (sample.contact == CONTACT_UP)
    {
      /* Pen is now up */

      report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
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
  iinfo("Returning %d\n", ret);
  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: up_ioctl
 ****************************************************************************/

static int up_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode         *inode;
  FAR struct up_dev_s *priv;
  int                       ret;

  iinfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct up_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      default:
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: up_poll
 ****************************************************************************/

static int up_poll(FAR struct file *filep, FAR struct pollfd *fds,
                   bool setup)
{
  FAR struct inode    *inode;
  FAR struct up_dev_s *priv;
  int                  ret;
  int                  i;

  iinfo("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct up_dev_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->revents & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_SIM_TCNWAITERS; i++)
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

      if (i >= CONFIG_SIM_TCNWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->penchange)
        {
          up_notify(priv);
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
  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_tsc_initialize
 *
 * Description:
 *   Configure the simulated touchscreen.  This will register the driver as
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

int sim_tsc_initialize(int minor)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)&g_simtouchscreen;
  char devname[DEV_NAMELEN];
  int ret;

  iinfo("minor: %d\n", minor);

  /* Debug-only sanity checks */

  DEBUGASSERT(minor >= 0 && minor < 100);

  /* Initialize the touchscreen device driver instance */

  memset(priv, 0, sizeof(struct up_dev_s));

  /* Initialize semaphores */

  nxsem_init(&priv->devsem,  0, 1); /* Initialize device structure semaphore */
  nxsem_init(&priv->waitsem, 0, 0); /* Initialize pen event wait semaphore */

  /* The waitsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

  priv->minor = minor;

  /* Register the device as an input device */

  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, minor);
  iinfo("Registering %s\n", devname);

  ret = register_driver(devname, &up_fops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: register_driver() failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* Enable X11 event processing from the IDLE loop */

  priv->eventloop = 1;

  /* And return success */

  return OK;

errout_with_priv:
  nxsem_destroy(&priv->waitsem);
  nxsem_destroy(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: sim_tsc_uninitialize
 *
 * Description:
 *   Uninitialized the simulated touchscreen
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Return OK if success or negative value of the error.
 *
 ****************************************************************************/

int sim_tsc_uninitialize(void)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)&g_simtouchscreen;
  char devname[DEV_NAMELEN];
  int ret = OK;

  /* Get exclusive access */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Stop the event loop (Hmm.. the caller must be sure that there are no
   * open references to the touchscreen driver.  This might better be
   * done in close() using a reference count).
   */

  priv->eventloop = 0;

  /* Un-register the device */

  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->minor);
  iinfo("Un-registering %s\n", devname);

  ret = unregister_driver(devname);
  if (ret < 0)
    {
      ierr("ERROR: uregister_driver() failed: %d\n", ret);
    }

  /* Clean up any resources.  Ouch!  While we are holding the semaphore? */

  nxsem_destroy(&priv->waitsem);
  nxsem_destroy(&priv->devsem);

  return ret;
}

/****************************************************************************
 * Name: up_buttonevent
 ****************************************************************************/

void up_buttonevent(int x, int y, int buttons)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)&g_simtouchscreen;
  bool                 pendown;  /* true: pen is down */

  if (priv->eventloop == 0)
    {
      return;
    }

  iinfo("x=%d y=%d buttons=%02x\n", x, y, buttons);
  iinfo("contact=%d nwaiters=%d\n", priv->sample.contact, priv->nwaiters);

  /* Any button press will count as pendown. */

  pendown = (buttons != 0);

  /* Handle the change from pen down to pen up */

  if (!pendown)
    {
      /* Ignore the pend up if the pen was already up
       * (CONTACT_NONE == pen up and  already reported.
       *  CONTACT_UP == pen up, but not reported)
       */

      if (priv->sample.contact == CONTACT_NONE)
        {
          return;
        }

      /* Not yet reported */

      priv->sample.contact = CONTACT_UP;
    }
  else
    {
      /* Save the measurements */

      priv->sample.x = x;
      priv->sample.y = y;

      /* Note the availability of new measurements:
       * If this is the first (acknowledged) pen down report, then report
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

  up_notify(priv);
}
