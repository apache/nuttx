/****************************************************************************
 * drivers/input/ajoystick.c
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

/* This file provides a driver for a standard analog joystick device.  An
 * analog joystick refers to a joystick that provides X/Y positional data as
 * integer values such as might be provides by Analog-to-Digital Conversion
 * (ADC).  The analog positional data may also be accompanied by discrete
 * button data.
 *
 * The analog joystick driver exports a standard character driver
 * interface. By convention, the analog joystick is registered as an input
 * device at /dev/ajoyN where N uniquely identifies the driver instance.
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
#include <nuttx/input/ajoystick.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the state of one analog joystick driver */

struct ajoy_upperhalf_s
{
  /* Saved binding to the lower half analog joystick driver */

  FAR const struct ajoy_lowerhalf_s *au_lower;

  ajoy_buttonset_t au_sample;  /* Last sampled button states */

  /* The following is a singly linked list of open references to the
   * joystick device.
   */

  FAR struct ajoy_open_s *au_open;
};

/* This structure describes the state of one open joystick driver instance */

struct ajoy_open_s
{
  /* Supports a singly linked list */

  FAR struct ajoy_open_s *ao_flink;

  /* Joystick event notification information */

  pid_t ao_pid;
  struct ajoy_notify_s ao_notify;
  struct sigwork_s ao_work;

  /* Poll event information */

  struct ajoy_pollevents_s ao_pollevents;

  /* The following is a list if poll structures of threads waiting for
   * driver events.
   */

  bool ao_pollpending;
  FAR struct pollfd *ao_fds[CONFIG_INPUT_AJOYSTICK_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sampling and Interrupt handling */

static void    ajoy_enable(FAR struct ajoy_upperhalf_s *priv);
static void    ajoy_interrupt(FAR const struct ajoy_lowerhalf_s *lower,
                              FAR void *arg);

/* Sampling */

static void    ajoy_sample(FAR struct ajoy_upperhalf_s *priv);

/* Character driver methods */

static int     ajoy_open(FAR struct file *filep);
static int     ajoy_close(FAR struct file *filep);
static ssize_t ajoy_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static int     ajoy_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
static int     ajoy_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations ajoy_fops =
{
  ajoy_open,  /* open */
  ajoy_close, /* close */
  ajoy_read,  /* read */
  NULL,       /* write */
  NULL,       /* seek */
  ajoy_ioctl, /* ioctl */
  ajoy_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL      /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ajoy_enable
 ****************************************************************************/

static void ajoy_enable(FAR struct ajoy_upperhalf_s *priv)
{
  FAR const struct ajoy_lowerhalf_s *lower;
  FAR struct ajoy_open_s *opriv;
  ajoy_buttonset_t press;
  ajoy_buttonset_t release;

  DEBUGASSERT(priv);
  lower = priv->au_lower;
  DEBUGASSERT(lower);

  /* Visit each opened reference to the device */

  press   = 0;
  release = 0;

  for (opriv = priv->au_open; opriv; opriv = opriv->ao_flink)
    {
      press   |= opriv->ao_pollevents.ap_press;
      release |= opriv->ao_pollevents.ap_release;

      /* OR in the signal events */

      press   |= opriv->ao_notify.an_press;
      release |= opriv->ao_notify.an_release;
    }

  /* Enable/disable button interrupts */

  DEBUGASSERT(lower->al_enable);
  if (press != 0 || release != 0)
    {
      /* Enable interrupts with the new button set */

      lower->al_enable(lower, press, release,
                       (ajoy_handler_t)ajoy_interrupt, priv);
    }
  else
    {
      /* Disable further interrupts */

      lower->al_enable(lower, 0, 0, NULL, NULL);
    }
}

/****************************************************************************
 * Name: ajoy_interrupt
 ****************************************************************************/

static void ajoy_interrupt(FAR const struct ajoy_lowerhalf_s *lower,
                           FAR void *arg)
{
  FAR struct ajoy_upperhalf_s *priv = (FAR struct ajoy_upperhalf_s *)arg;

  DEBUGASSERT(priv);

  /* Process the next sample */

  ajoy_sample(priv);
}

/****************************************************************************
 * Name: ajoy_sample
 ****************************************************************************/

static void ajoy_sample(FAR struct ajoy_upperhalf_s *priv)
{
  FAR const struct ajoy_lowerhalf_s *lower;
  FAR struct ajoy_open_s *opriv;
  ajoy_buttonset_t sample;
  ajoy_buttonset_t change;
  ajoy_buttonset_t press;
  ajoy_buttonset_t release;
  irqstate_t flags;

  DEBUGASSERT(priv);
  lower = priv->au_lower;
  DEBUGASSERT(lower);

  /* This routine is called both task level and interrupt level, so
   * interrupts must be disabled.
   */

  flags = enter_critical_section();

  /* Sample the new button state */

  DEBUGASSERT(lower->al_buttons);
  sample = lower->al_buttons(lower);

  add_ui_randomness(sample);

  /* Determine which buttons have been newly pressed and which have been
   * newly released.
   */

  change = sample ^ priv->au_sample;
  press  = change & sample;

  DEBUGASSERT(lower->al_supported);
  release = change & (lower->al_supported(lower) & ~sample);

  /* Visit each opened reference to the device */

  for (opriv = priv->au_open; opriv; opriv = opriv->ao_flink)
    {
      /* Have any poll events occurred? */

      if ((press & opriv->ao_pollevents.ap_press)     != 0 ||
          (release & opriv->ao_pollevents.ap_release) != 0)
        {
          opriv->ao_pollpending = true;

          /* Yes.. Notify all waiters */

          poll_notify(opriv->ao_fds, CONFIG_INPUT_AJOYSTICK_NPOLLWAITERS,
                      POLLIN);
        }

      /* Have any signal events occurred? */

      if ((press & opriv->ao_notify.an_press)     != 0 ||
          (release & opriv->ao_notify.an_release) != 0)
        {
          /* Yes.. Signal the waiter */

          opriv->ao_notify.an_event.sigev_value.sival_int = sample;
          nxsig_notification(opriv->ao_pid, &opriv->ao_notify.an_event,
                             SI_QUEUE, &opriv->ao_work);
        }
    }

  priv->au_sample = sample;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ajoy_open
 ****************************************************************************/

static int ajoy_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct ajoy_upperhalf_s *priv;
  FAR struct ajoy_open_s *opriv;
  FAR const struct ajoy_lowerhalf_s *lower;
  ajoy_buttonset_t supported;
  irqstate_t flags;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = (FAR struct ajoy_upperhalf_s *)inode->i_private;

  /* Allocate a new open structure */

  opriv = (FAR struct ajoy_open_s *)kmm_zalloc(sizeof(struct ajoy_open_s));
  if (!opriv)
    {
      ierr("ERROR: Failed to allocate open structure\n");
      return -ENOMEM;
    }

  /* Initialize the open structure */

  lower = priv->au_lower;
  DEBUGASSERT(lower && lower->al_supported);

  flags = enter_critical_section();

  supported = lower->al_supported(lower);
  opriv->ao_pollevents.ap_press   = supported;
  opriv->ao_pollevents.ap_release = supported;

  /* Attach the open structure to the device */

  opriv->ao_flink = priv->au_open;
  priv->au_open = opriv;

  /* Enable/disable interrupt handling */

  ajoy_enable(priv);

  /* Attach the open structure to the file structure */

  filep->f_priv = (FAR void *)opriv;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ajoy_close
 ****************************************************************************/

static int ajoy_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct ajoy_upperhalf_s *priv;
  FAR struct ajoy_open_s *opriv;
  FAR struct ajoy_open_s *curr;
  FAR struct ajoy_open_s *prev;
  irqstate_t flags;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct ajoy_upperhalf_s *)inode->i_private;

  flags = enter_critical_section();

  /* Find the open structure in the list of open structures for the device */

  for (prev = NULL, curr = priv->au_open;
       curr && curr != opriv;
       prev = curr, curr = curr->ao_flink);

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
      prev->ao_flink = opriv->ao_flink;
    }
  else
    {
      priv->au_open = opriv->ao_flink;
    }

  /* Enable/disable interrupt handling */

  ajoy_enable(priv);

  leave_critical_section(flags);

  /* Cancel any pending notification */

  nxsig_cancel_notification(&opriv->ao_work);

  /* And free the open structure */

  kmm_free(opriv);
  return OK;
}

/****************************************************************************
 * Name: ajoy_read
 ****************************************************************************/

static ssize_t ajoy_read(FAR struct file *filep, FAR char *buffer,
                         size_t len)
{
  FAR struct inode *inode;
  FAR struct ajoy_open_s *opriv;
  FAR struct ajoy_upperhalf_s *priv;
  FAR const struct ajoy_lowerhalf_s *lower;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct ajoy_upperhalf_s *)inode->i_private;

  /* Make sure that the buffer is sufficiently large to hold at least one
   * complete sample.
   *
   * REVISIT:  Should also check buffer alignment.
   */

  if (len < sizeof(struct ajoy_sample_s))
    {
      ierr("ERROR: buffer too small: %lu\n", (unsigned long)len);
      return -EINVAL;
    }

  /* Get exclusive access to the driver structure */

  flags = enter_critical_section();

  /* Read and return the current state of the joystick buttons */

  lower = priv->au_lower;
  DEBUGASSERT(lower && lower->al_sample);
  ret = lower->al_sample(lower, (FAR struct ajoy_sample_s *)buffer);
  if (ret >= 0)
    {
      opriv->ao_pollpending = false;
      ret = sizeof(struct ajoy_sample_s);
    }

  leave_critical_section(flags);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: ajoy_ioctl
 ****************************************************************************/

static int ajoy_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct ajoy_upperhalf_s *priv;
  FAR struct ajoy_open_s *opriv;
  FAR const struct ajoy_lowerhalf_s *lower;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct ajoy_upperhalf_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  flags = enter_critical_section();

  /* Handle the ioctl command */

  ret = -EINVAL;
  switch (cmd)
    {
    /* Command:     AJOYIOC_SUPPORTED
     * Description: Report the set of button events supported by the
     *              hardware;
     * Argument:    A pointer to writeable integer value in which to return
     *              the set of supported buttons.
     * Return:      Zero (OK) on success.  Minus one will be returned on
     *              failure with the errno value set appropriately.
     */

    case AJOYIOC_SUPPORTED:
      {
        FAR int *supported = (FAR int *)((uintptr_t)arg);

        if (supported)
          {
            lower = priv->au_lower;
            DEBUGASSERT(lower && lower->al_supported);

            *supported = (int)lower->al_supported(lower);
            ret = OK;
          }
      }
      break;

    /* Command:     AJOYIOC_POLLEVENTS
     * Description: Specify the set of button events that can cause a poll()
     *              to awaken.  The default is all button depressions and
     *              all button releases (all supported buttons);
     * Argument:    A read-only pointer to an instance of struct
     *              ajoy_pollevents_s
     * Return:      Zero (OK) on success.  Minus one will be returned on
     *              failure with the errno value set appropriately.
     */

    case AJOYIOC_POLLEVENTS:
      {
        FAR struct ajoy_pollevents_s *pollevents =
          (FAR struct ajoy_pollevents_s *)((uintptr_t)arg);

        if (pollevents)
          {
            /* Save the poll events */

            opriv->ao_pollevents.ap_press   = pollevents->ap_press;
            opriv->ao_pollevents.ap_release = pollevents->ap_release;

            /* Enable/disable interrupt handling */

            ajoy_enable(priv);
            ret = OK;
          }
      }
      break;

    /* Command:     AJOYIOC_REGISTER
     * Description: Register to receive a signal whenever there is a change
     *              in any of the joystick discrete inputs.  This feature,
     *              of course, depends upon interrupt GPIO support from the
     *              platform.
     * Argument:    A read-only pointer to an instance of struct
     *              ajoy_notify_s
     * Return:      Zero (OK) on success.  Minus one will be returned on
     *              failure with the errno value set appropriately.
     */

    case AJOYIOC_REGISTER:
      {
        FAR struct ajoy_notify_s *notify =
          (FAR struct ajoy_notify_s *)((uintptr_t)arg);

        if (notify)
          {
            /* Save the notification events */

            opriv->ao_notify.an_press   = notify->an_press;
            opriv->ao_notify.an_release = notify->an_release;
            opriv->ao_notify.an_event   = notify->an_event;
            opriv->ao_pid               = getpid();

            /* Enable/disable interrupt handling */

            ajoy_enable(priv);
            ret = OK;
          }
      }
      break;

    default:
      ierr("ERROR: Unrecognized command: %ld\n", cmd);
      ret = -ENOTTY;
      break;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: ajoy_poll
 ****************************************************************************/

static int ajoy_poll(FAR struct file *filep, FAR struct pollfd *fds,
                     bool setup)
{
  FAR struct inode *inode;
  FAR struct ajoy_open_s *opriv;
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

      for (i = 0; i < CONFIG_INPUT_AJOYSTICK_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!opriv->ao_fds[i])
            {
              /* Bind the poll structure and this slot */

              opriv->ao_fds[i] = fds;
              fds->priv = &opriv->ao_fds[i];

              /* Report if the event is pending */

              if (opriv->ao_pollpending)
                {
                  poll_notify(&fds, 1, POLLIN);
                }

              break;
            }
        }

      if (i >= CONFIG_INPUT_AJOYSTICK_NPOLLWAITERS)
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
 * Name: ajoy_register
 *
 * Description:
 *   Bind the lower half analog joystick driver to an instance of the
 *   upper half analog joystick driver and register the composite character
 *   driver as the specific device.
 *
 * Input Parameters:
 *   devname - The name of the analog joystick device to be registers.
 *     This should be a string of the form "/dev/ajoyN" where N is the
 *     minor device number.
 *   lower - An instance of the platform-specific analog joystick lower
 *     half driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int ajoy_register(FAR const char *devname,
                  FAR const struct ajoy_lowerhalf_s *lower)
{
  FAR struct ajoy_upperhalf_s *priv;
  int ret;

  DEBUGASSERT(devname && lower);

  /* Allocate a new ajoystick driver instance */

  priv = (FAR struct ajoy_upperhalf_s *)
    kmm_zalloc(sizeof(struct ajoy_upperhalf_s));
  if (!priv)
    {
      ierr("ERROR: Failed to allocate device structure\n");
      return -ENOMEM;
    }

  /* Make sure that all ajoystick interrupts are disabled */

  DEBUGASSERT(lower->al_enable);
  lower->al_enable(lower, 0, 0, NULL, NULL);

  /* Initialize the new ajoystick driver instance */

  priv->au_lower = lower;

  DEBUGASSERT(lower->al_buttons);
  priv->au_sample = lower->al_buttons(lower);

  /* And register the ajoystick driver */

  ret = register_driver(devname, &ajoy_fops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: register_driver failed: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
