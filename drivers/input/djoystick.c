/****************************************************************************
 * drivers/input/djoystick.c
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

/* This file provides a driver for a standard discrete joystick device.  A
 * discrete joystick refers to a joystick that could be implemented entirely
 * with GPIO input pins.  So up, down, left, and right are all discrete
 * values like buttons (as opposed to integer values like you might obtain
 * from an analog joystick).
 *
 * The discrete joystick driver exports a standard character driver
 * interface. By convention, the discrete joystick is registered as an input
 * device at /dev/djoyN where N uniquely identifies the driver instance.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/input/djoystick.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the state of one discrete joystick driver */

struct djoy_upperhalf_s
{
  /* Saved binding to the lower half discrete joystick driver */

  FAR const struct djoy_lowerhalf_s *du_lower;

  djoy_buttonset_t du_sample;  /* Last sampled button states */

  /* The following is a singly linked list of open references to the
   * joystick device.
   */

  FAR struct djoy_open_s *du_open;
};

/* This structure describes the state of one open joystick driver instance */

struct djoy_open_s
{
  /* Supports a singly linked list */

  FAR struct djoy_open_s *do_flink;

  /* Joystick event notification information */

  pid_t do_pid;
  struct djoy_notify_s do_notify;
  struct sigwork_s do_work;

  /* Poll event information */

  struct djoy_pollevents_s do_pollevents;

  /* The following is a list if poll structures of threads waiting for
   * driver events.
   */

  bool do_pollpending;
  FAR struct pollfd *do_fds[CONFIG_INPUT_DJOYSTICK_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sampling and Interrupt handling */

static void    djoy_enable(FAR struct djoy_upperhalf_s *priv);
static void    djoy_interrupt(FAR const struct djoy_lowerhalf_s *lower,
                              FAR void *arg);

/* Sampling */

static void    djoy_sample(FAR struct djoy_upperhalf_s *priv);

/* Character driver methods */

static int     djoy_open(FAR struct file *filep);
static int     djoy_close(FAR struct file *filep);
static ssize_t djoy_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static int     djoy_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
static int     djoy_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations djoy_fops =
{
  djoy_open,  /* open */
  djoy_close, /* close */
  djoy_read,  /* read */
  NULL,       /* write */
  NULL,       /* seek */
  djoy_ioctl, /* ioctl */
  NULL,       /* mmap */
  NULL,       /* truncate */
  djoy_poll   /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: djoy_enable
 ****************************************************************************/

static void djoy_enable(FAR struct djoy_upperhalf_s *priv)
{
  FAR const struct djoy_lowerhalf_s *lower;
  FAR struct djoy_open_s *opriv;
  djoy_buttonset_t press;
  djoy_buttonset_t release;

  DEBUGASSERT(priv);
  lower = priv->du_lower;
  DEBUGASSERT(lower);

  /* Visit each opened reference to the device */

  press   = 0;
  release = 0;

  for (opriv = priv->du_open; opriv; opriv = opriv->do_flink)
    {
      press   |= opriv->do_pollevents.dp_press;
      release |= opriv->do_pollevents.dp_release;

      /* OR in the signal events */

      press   |= opriv->do_notify.dn_press;
      release |= opriv->do_notify.dn_release;
    }

  /* Enable/disable button interrupts */

  DEBUGASSERT(lower->dl_enable);
  if (press != 0 || release != 0)
    {
      /* Enable interrupts with the new button set */

      lower->dl_enable(lower, press, release,
                       (djoy_interrupt_t)djoy_interrupt, priv);
    }
  else
    {
      /* Disable further interrupts */

      lower->dl_enable(lower, 0, 0, NULL, NULL);
    }
}

/****************************************************************************
 * Name: djoy_interrupt
 ****************************************************************************/

static void djoy_interrupt(FAR const struct djoy_lowerhalf_s *lower,
                           FAR void *arg)
{
  FAR struct djoy_upperhalf_s *priv = (FAR struct djoy_upperhalf_s *)arg;

  DEBUGASSERT(priv);

  /* Process the next sample */

  djoy_sample(priv);
}

/****************************************************************************
 * Name: djoy_sample
 ****************************************************************************/

static void djoy_sample(FAR struct djoy_upperhalf_s *priv)
{
  FAR const struct djoy_lowerhalf_s *lower;
  FAR struct djoy_open_s *opriv;
  djoy_buttonset_t sample;
  djoy_buttonset_t change;
  djoy_buttonset_t press;
  djoy_buttonset_t release;
  irqstate_t flags;

  DEBUGASSERT(priv);
  lower = priv->du_lower;
  DEBUGASSERT(lower);

  /* This routine is called both task level and interrupt level, so
   * interrupts must be disabled.
   */

  flags = enter_critical_section();

  /* Sample the new button state */

  DEBUGASSERT(lower->dl_sample);
  sample = lower->dl_sample(lower);

  add_ui_randomness(sample);

  /* Determine which buttons have been newly pressed and which have been
   * newly released.
   */

  change = sample ^ priv->du_sample;
  press  = change & sample;

  DEBUGASSERT(lower->dl_supported);
  release = change & (lower->dl_supported(lower) & ~sample);

  /* Visit each opened reference to the device */

  for (opriv = priv->du_open; opriv; opriv = opriv->do_flink)
    {
      /* Have any poll events occurred? */

      if ((press & opriv->do_pollevents.dp_press)     != 0 ||
          (release & opriv->do_pollevents.dp_release) != 0)
        {
          opriv->do_pollpending = true;

          /* Yes.. Notify all waiters */

          poll_notify(opriv->do_fds, CONFIG_INPUT_DJOYSTICK_NPOLLWAITERS,
                      POLLIN);
        }

      /* Have any signal events occurred? */

      if ((press & opriv->do_notify.dn_press)     != 0 ||
          (release & opriv->do_notify.dn_release) != 0)
        {
          /* Yes.. Signal the waiter */

          opriv->do_notify.dn_event.sigev_value.sival_int = sample;
          nxsig_notification(opriv->do_pid, &opriv->do_notify.dn_event,
                             SI_QUEUE, &opriv->do_work);
        }
    }

  priv->du_sample = sample;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: djoy_open
 ****************************************************************************/

static int djoy_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct djoy_upperhalf_s *priv;
  FAR struct djoy_open_s *opriv;
  FAR const struct djoy_lowerhalf_s *lower;
  djoy_buttonset_t supported;
  irqstate_t flags;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = (FAR struct djoy_upperhalf_s *)inode->i_private;

  /* Allocate a new open structure */

  opriv = (FAR struct djoy_open_s *)kmm_zalloc(sizeof(struct djoy_open_s));
  if (!opriv)
    {
      ierr("ERROR: Failed to allocate open structure\n");
      return -ENOMEM;
    }

  /* Initialize the open structure */

  lower = priv->du_lower;
  DEBUGASSERT(lower && lower->dl_supported);

  flags = enter_critical_section();

  supported = lower->dl_supported(lower);
  opriv->do_pollevents.dp_press   = supported;
  opriv->do_pollevents.dp_release = supported;

  /* Attach the open structure to the device */

  opriv->do_flink = priv->du_open;
  priv->du_open = opriv;

  /* Enable/disable interrupt handling */

  djoy_enable(priv);

  /* Attach the open structure to the file structure */

  filep->f_priv = (FAR void *)opriv;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: djoy_close
 ****************************************************************************/

static int djoy_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct djoy_upperhalf_s *priv;
  FAR struct djoy_open_s *opriv;
  FAR struct djoy_open_s *curr;
  FAR struct djoy_open_s *prev;
  irqstate_t flags;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct djoy_upperhalf_s *)inode->i_private;

  flags = enter_critical_section();

  /* Find the open structure in the list of open structures for the device */

  for (prev = NULL, curr = priv->du_open;
       curr && curr != opriv;
       prev = curr, curr = curr->do_flink);

  DEBUGASSERT(curr);
  if (!curr)
    {
      ierr("ERROR: Failed to find open entry\n");
      leave_critical_section(flags);
      return -ENOENT;
    }

  /* Remove the structure from the device */

  if (prev)
    {
      prev->do_flink = opriv->do_flink;
    }
  else
    {
      priv->du_open = opriv->do_flink;
    }

  /* Enable/disable interrupt handling */

  djoy_enable(priv);

  leave_critical_section(flags);

  /* Cancel any pending notification */

  nxsig_cancel_notification(&opriv->do_work);

  /* And free the open structure */

  kmm_free(opriv);
  return OK;
}

/****************************************************************************
 * Name: djoy_read
 ****************************************************************************/

static ssize_t djoy_read(FAR struct file *filep, FAR char *buffer,
                         size_t len)
{
  FAR struct inode *inode;
  FAR struct djoy_open_s *opriv;
  FAR struct djoy_upperhalf_s *priv;
  FAR const struct djoy_lowerhalf_s *lower;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct djoy_upperhalf_s *)inode->i_private;

  /* Make sure that the buffer is sufficiently large to hold at least one
   * complete sample.
   */

  if (len < sizeof(djoy_buttonset_t))
    {
      ierr("ERROR: buffer too small: %lu\n", (unsigned long)len);
      return -EINVAL;
    }

  /* Get exclusive access to the driver structure */

  flags = enter_critical_section();

  /* Read and return the current state of the joystick buttons */

  lower = priv->du_lower;
  DEBUGASSERT(lower && lower->dl_sample);
  priv->du_sample = lower->dl_sample(lower);
  *(FAR djoy_buttonset_t *)buffer = priv->du_sample;
  opriv->do_pollpending = false;
  ret = sizeof(djoy_buttonset_t);

  leave_critical_section(flags);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: djoy_ioctl
 ****************************************************************************/

static int djoy_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct djoy_upperhalf_s *priv;
  FAR struct djoy_open_s *opriv;
  FAR const struct djoy_lowerhalf_s *lower;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct djoy_upperhalf_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  flags = enter_critical_section();

  /* Handle the ioctl command */

  ret = -EINVAL;
  switch (cmd)
    {
    /* Command:     DJOYIOC_SUPPORTED
     * Description: Report the set of button events supported by the
     *              hardware;
     * Argument:    A pointer to writeable integer value in which to return
     *              the set of supported buttons.
     * Return:      Zero (OK) on success.  Minus one will be returned on
     *              failure with the errno value set appropriately.
     */

    case DJOYIOC_SUPPORTED:
      {
        FAR int *supported = (FAR int *)((uintptr_t)arg);

        if (supported)
          {
            lower = priv->du_lower;
            DEBUGASSERT(lower && lower->dl_supported);

            *supported = (int)lower->dl_supported(lower);
            ret = OK;
          }
      }
      break;

    /* Command:     DJOYIOC_POLLEVENTS
     * Description: Specify the set of button events that can cause a poll()
     *              to awaken.  The default is all button depressions and
     *              all button releases (all supported buttons);
     * Argument:    A read-only pointer to an instance of struct
     *              djoy_pollevents_s
     * Return:      Zero (OK) on success.  Minus one will be returned on
     *              failure with the errno value set appropriately.
     */

    case DJOYIOC_POLLEVENTS:
      {
        FAR struct djoy_pollevents_s *pollevents =
          (FAR struct djoy_pollevents_s *)((uintptr_t)arg);

        if (pollevents)
          {
            /* Save the poll events */

            opriv->do_pollevents.dp_press   = pollevents->dp_press;
            opriv->do_pollevents.dp_release = pollevents->dp_release;

            /* Enable/disable interrupt handling */

            djoy_enable(priv);
            ret = OK;
          }
      }
      break;

    /* Command:     DJOYIOC_REGISTER
     * Description: Register to receive a signal whenever there is a change
     *              in any of the joystick discrete inputs.  This feature,
     *              of course, depends upon interrupt GPIO support from the
     *              platform.
     * Argument:    A read-only pointer to an instance of struct
     *              djoy_notify_s
     * Return:      Zero (OK) on success.  Minus one will be returned on
     *              failure with the errno value set appropriately.
     */

    case DJOYIOC_REGISTER:
      {
        FAR struct djoy_notify_s *notify =
          (FAR struct djoy_notify_s *)((uintptr_t)arg);

        if (notify)
          {
            /* Save the notification events */

            opriv->do_notify.dn_press   = notify->dn_press;
            opriv->do_notify.dn_release = notify->dn_release;
            opriv->do_notify.dn_event   = notify->dn_event;
            opriv->do_pid               = nxsched_getpid();

            /* Enable/disable interrupt handling */

            djoy_enable(priv);
            ret = OK;
          }
      }
      break;

    default:
      ierr("ERROR: Unrecognized command: %d\n", cmd);
      ret = -ENOTTY;
      break;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: djoy_poll
 ****************************************************************************/

static int djoy_poll(FAR struct file *filep, FAR struct pollfd *fds,
                     bool setup)
{
  FAR struct inode *inode;
  FAR struct djoy_open_s *opriv;
  irqstate_t flags;
  int ret = OK;
  int i;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);

  /* Get exclusive access to the driver structure */

  flags = enter_critical_section();

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_INPUT_DJOYSTICK_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!opriv->do_fds[i])
            {
              /* Bind the poll structure and this slot */

              opriv->do_fds[i] = fds;
              fds->priv = &opriv->do_fds[i];

              if (opriv->do_pollpending)
                {
                  poll_notify(&fds, 1, POLLIN);
                }

              break;
            }
        }

      if (i >= CONFIG_INPUT_DJOYSTICK_NPOLLWAITERS)
        {
          ierr("ERROR: Too man poll waiters\n");
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

#ifdef CONFIG_DEBUG_FEATURES
      if (!slot)
        {
          ierr("ERROR: Poll slot not found\n");
          ret = -EIO;
          goto errout;
        }
#endif

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

errout:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: djoy_register
 *
 * Description:
 *   Bind the lower half discrete joystick driver to an instance of the
 *   upper half discrete joystick driver and register the composite character
 *   driver as the specific device.
 *
 * Input Parameters:
 *   devname - The name of the discrete joystick device to be registers.
 *     This should be a string of the form "/dev/djoyN" where N is the
 *     minor device number.
 *   lower - An instance of the platform-specific discrete joystick lower
 *     half driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int djoy_register(FAR const char *devname,
                  FAR const struct djoy_lowerhalf_s *lower)
{
  FAR struct djoy_upperhalf_s *priv;
  int ret;

  DEBUGASSERT(devname && lower);

  /* Allocate a new djoystick driver instance */

  priv = (FAR struct djoy_upperhalf_s *)
    kmm_zalloc(sizeof(struct djoy_upperhalf_s));
  if (!priv)
    {
      ierr("ERROR: Failed to allocate device structure\n");
      return -ENOMEM;
    }

  /* Make sure that all djoystick interrupts are disabled */

  DEBUGASSERT(lower->dl_enable);
  lower->dl_enable(lower, 0, 0, NULL, NULL);

  /* Initialize the new djoystick driver instance */

  priv->du_lower = lower;

  DEBUGASSERT(lower->dl_sample);
  priv->du_sample = lower->dl_sample(lower);

  /* And register the djoystick driver */

  ret = register_driver(devname, &djoy_fops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: register_driver failed: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
