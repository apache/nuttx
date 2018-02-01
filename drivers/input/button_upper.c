/****************************************************************************
 * drivers/input/button_upper.c
 *
 *   Copyright (C) 2015-2017 Gregory Nutt. All rights reserved.
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

/* This file provides a driver for a button input devices.
 *
 * The buttons driver exports a standard character driver interface. By
 * convention, the button driver is registered as an input device at
 * /dev/btnN where N uniquely identifies the driver instance.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/input/buttons.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the state of one button driver */

struct btn_upperhalf_s
{
  /* Saved binding to the lower half button driver */

  FAR const struct btn_lowerhalf_s *bu_lower;

  btn_buttonset_t bu_enabled; /* Set of currently enabled button interrupts */
  btn_buttonset_t bu_sample;  /* Last sampled button states */
  sem_t bu_exclsem;           /* Supports exclusive access to the device */

  /* The following is a singly linked list of open references to the
   * button device.
   */

  FAR struct btn_open_s *bu_open;
};

/* This structure describes the state of one open button driver instance */

struct btn_open_s
{
  /* Supports a singly linked list */

  FAR struct btn_open_s *bo_flink;

  /* The following will be true if we are closing */

  volatile bool bo_closing;

#ifndef CONFIG_DISABLE_SIGNALS
  /* Button event notification information */

  pid_t bo_pid;
  struct btn_notify_s bo_notify;
#endif

#ifndef CONFIG_DISABLE_POLL
  /* Poll event information */

  struct btn_pollevents_s bo_pollevents;

  /* The following is a list if poll structures of threads waiting for
   * driver events.
   */

  FAR struct pollfd *bo_fds[CONFIG_BUTTONS_NPOLLWAITERS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphore helpers */

static inline int btn_takesem(sem_t *sem);
#define btn_givesem(s) nxsem_post(s);

/* Sampling and Interrupt handling */

#if !defined(CONFIG_DISABLE_POLL) || !defined(CONFIG_DISABLE_SIGNALS)
static void    btn_enable(FAR struct btn_upperhalf_s *priv);
static void    btn_interrupt(FAR const struct btn_lowerhalf_s *lower,
                             FAR void *arg);
#endif

/* Sampling */

static void    btn_sample(FAR struct btn_upperhalf_s *priv);

/* Character driver methods */

static int     btn_open(FAR struct file *filep);
static int     btn_close(FAR struct file *filep);
static ssize_t btn_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static int     btn_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int     btn_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations btn_fops =
{
  btn_open,  /* open */
  btn_close, /* close */
  btn_read,  /* read */
  NULL,      /* write */
  NULL,      /* seek */
  btn_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , btn_poll /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btn_takesem
 ****************************************************************************/

static inline int btn_takesem(sem_t *sem)
{
  int ret;

  /* Take a count from the semaphore, possibly waiting */

  ret = nxsem_wait(sem);

  /* The only case that an error should occur here is if the wait
   * was awakened by a signal
   */

  DEBUGASSERT(ret == OK || ret == -EINTR);
  return ret;
}

/****************************************************************************
 * Name: btn_enable
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_POLL) || !defined(CONFIG_DISABLE_SIGNALS)
static void btn_enable(FAR struct btn_upperhalf_s *priv)
{
  FAR const struct btn_lowerhalf_s *lower = priv->bu_lower;
  FAR struct btn_open_s *opriv;
  btn_buttonset_t press;
  btn_buttonset_t release;
  irqstate_t flags;

  DEBUGASSERT(priv && priv->bu_lower);
  lower = priv->bu_lower;

  /* This routine is called both task level and interrupt level, so
   * interrupts must be disabled.
   */

  flags = enter_critical_section();

  /* Visit each opened reference to the device */

  press   = 0;
  release = 0;

  for (opriv = priv->bu_open; opriv; opriv = opriv->bo_flink)
    {
#ifndef CONFIG_DISABLE_POLL
      /* OR in the poll event buttons */

      press   |= opriv->bo_pollevents.bp_press;
      release |= opriv->bo_pollevents.bp_release;
#endif

#ifndef CONFIG_DISABLE_SIGNALS
      /* OR in the signal events */

      press   |= opriv->bo_notify.bn_press;
      release |= opriv->bo_notify.bn_release;
#endif
    }

  /* Enable/disable button interrupts */

  DEBUGASSERT(lower->bl_enable);
  if (press != 0 || release != 0)
    {
      /* Enable interrupts with the new button set */

      lower->bl_enable(lower, press, release,
                       (btn_handler_t)btn_interrupt, priv);
    }
  else
    {
      /* Disable further interrupts */

      lower->bl_enable(lower, 0, 0, NULL, NULL);
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: btn_interrupt
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_POLL) || !defined(CONFIG_DISABLE_SIGNALS)
static void btn_interrupt(FAR const struct btn_lowerhalf_s *lower,
                          FAR void *arg)
{
  FAR struct btn_upperhalf_s *priv = (FAR struct btn_upperhalf_s *)arg;

  DEBUGASSERT(priv);

  /* Process the next sample */

  btn_sample(priv);
}
#endif

/****************************************************************************
 * Name: btn_sample
 ****************************************************************************/

static void btn_sample(FAR struct btn_upperhalf_s *priv)
{
  FAR const struct btn_lowerhalf_s *lower = priv->bu_lower;
  FAR struct btn_open_s *opriv;
  btn_buttonset_t sample;
#if !defined(CONFIG_DISABLE_POLL) || !defined(CONFIG_DISABLE_SIGNALS)
  btn_buttonset_t change;
  btn_buttonset_t press;
  btn_buttonset_t release;
#endif
  irqstate_t flags;
#ifndef CONFIG_DISABLE_POLL
  int i;
#endif

  DEBUGASSERT(priv && priv->bu_lower);
  lower = priv->bu_lower;

  /* This routine is called both task level and interrupt level, so
   * interrupts must be disabled.
   */

  flags = enter_critical_section();

  /* Sample the new button state */

  DEBUGASSERT(lower->bl_buttons);
  sample = lower->bl_buttons(lower);

  add_ui_randomness(sample);

#if !defined(CONFIG_DISABLE_POLL) || !defined(CONFIG_DISABLE_SIGNALS)
  /* Determine which buttons have been newly pressed and which have been
   * newly released.
   */

  change = sample ^ priv->bu_sample;
  press  = change & sample;

  DEBUGASSERT(lower->bl_supported);
  release = change & (lower->bl_supported(lower) & ~sample);

  /* Visit each opened reference to the device */

  for (opriv = priv->bu_open; opriv; opriv = opriv->bo_flink)
    {
#ifndef CONFIG_DISABLE_POLL
      /* Have any poll events occurred? */

      if ((press & opriv->bo_pollevents.bp_press)     != 0 ||
          (release & opriv->bo_pollevents.bp_release) != 0)
        {
          /* Yes.. Notify all waiters */

          for (i = 0; i < CONFIG_BUTTONS_NPOLLWAITERS; i++)
            {
              FAR struct pollfd *fds = opriv->bo_fds[i];
              if (fds)
                {
                  fds->revents |= (fds->events & POLLIN);
                  if (fds->revents != 0)
                    {
                      iinfo("Report events: %02x\n", fds->revents);
                      nxsem_post(fds->sem);
                    }
                }
            }
        }
#endif

#ifndef CONFIG_DISABLE_SIGNALS
      /* Have any signal events occurred? */

      if ((press & opriv->bo_notify.bn_press)     != 0 ||
          (release & opriv->bo_notify.bn_release) != 0)
        {
          /* Yes.. Signal the waiter */

#ifdef CONFIG_CAN_PASS_STRUCTS
          union sigval value;
          value.sival_int = (int)sample;
          (void)nxsig_queue(opriv->bo_pid, opriv->bo_notify.bn_signo,
                            value);
#else
          (void)nxsig_queue(opriv->bo_pid, opriv->bo_notify.dn.signo,
                            (FAR void *)sample);
#endif
        }
#endif
    }

  /* Enable/disable interrupt handling */

  btn_enable(priv);
#endif

  priv->bu_sample = sample;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: btn_open
 ****************************************************************************/

static int btn_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct btn_upperhalf_s *priv;
  FAR struct btn_open_s *opriv;
#ifndef CONFIG_DISABLE_POLL
  FAR const struct btn_lowerhalf_s *lower;
  btn_buttonset_t supported;
#endif
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = (FAR struct btn_upperhalf_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = btn_takesem(&priv->bu_exclsem);
  if (ret < 0)
    {
      ierr("ERROR: btn_takesem failed: %d\n", ret);
      return ret;
    }

  /* Allocate a new open structure */

  opriv = (FAR struct btn_open_s *)kmm_zalloc(sizeof(struct btn_open_s));
  if (!opriv)
    {
      ierr("ERROR: Failled to allocate open structure\n");
      ret = -ENOMEM;
      goto errout_with_sem;
    }

  /* Initialize the open structure */

#ifndef CONFIG_DISABLE_POLL
  lower = priv->bu_lower;
  DEBUGASSERT(lower && lower->bl_supported);
  supported = lower->bl_supported(lower);

  opriv->bo_pollevents.bp_press   = supported;
  opriv->bo_pollevents.bp_release = supported;
#endif

  /* Attach the open structure to the device */

  opriv->bo_flink = priv->bu_open;
  priv->bu_open = opriv;

  /* Attach the open structure to the file structure */

  filep->f_priv = (FAR void *)opriv;
  ret = OK;

  /* Enable/disable interrupt handling */

  btn_enable(priv);

errout_with_sem:
  btn_givesem(&priv->bu_exclsem);
  return ret;
}

/****************************************************************************
 * Name: btn_close
 ****************************************************************************/

static int btn_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct btn_upperhalf_s *priv;
  FAR struct btn_open_s *opriv;
  FAR struct btn_open_s *curr;
  FAR struct btn_open_s *prev;
  irqstate_t flags;
  bool closing;
  int ret;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct btn_upperhalf_s *)inode->i_private;

  /* Handle an improbable race conditions with the following atomic test
   * and set.
   *
   * This is actually a pretty feeble attempt to handle this.  The
   * improbable race condition occurs if two different threads try to
   * close the button driver at the same time.  The rule:  don't do
   * that!  It is feeble because we do not really enforce stale pointer
   * detection anyway.
   */

  flags = enter_critical_section();
  closing = opriv->bo_closing;
  opriv->bo_closing = true;
  leave_critical_section(flags);

  if (closing)
    {
      /* Another thread is doing the close */

      return OK;
    }

  /* Get exclusive access to the driver structure */

  ret = btn_takesem(&priv->bu_exclsem);
  if (ret < 0)
    {
      ierr("ERROR: btn_takesem failed: %d\n", ret);
      return ret;
    }

  /* Find the open structure in the list of open structures for the device */

  for (prev = NULL, curr = priv->bu_open;
       curr && curr != opriv;
       prev = curr, curr = curr->bo_flink);

  DEBUGASSERT(curr);
  if (!curr)
    {
      ierr("ERROR: Failed to find open entry\n");
      ret = -ENOENT;
      goto errout_with_exclsem;
    }

  /* Remove the structure from the device */

  if (prev)
    {
      prev->bo_flink = opriv->bo_flink;
    }
  else
    {
      priv->bu_open = opriv->bo_flink;
    }

  /* And free the open structure */

  kmm_free(opriv);

  /* Enable/disable interrupt handling */

  btn_enable(priv);
  ret = OK;

errout_with_exclsem:
  btn_givesem(&priv->bu_exclsem);
  return ret;
}

/****************************************************************************
 * Name: btn_read
 ****************************************************************************/

static ssize_t btn_read(FAR struct file *filep, FAR char *buffer,
                        size_t len)
{
  FAR struct inode *inode;
  FAR struct btn_upperhalf_s *priv;
  FAR const struct btn_lowerhalf_s *lower;
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct btn_upperhalf_s *)inode->i_private;

  /* Make sure that the buffer is sufficiently large to hold at least one
   * complete sample.
   *
   * REVISIT:  Should also check buffer alignment.
   */

  if (len < sizeof(btn_buttonset_t))
    {
      ierr("ERROR: buffer too small: %lu\n", (unsigned long)len);
      return -EINVAL;
    }

  /* Get exclusive access to the driver structure */

  ret = btn_takesem(&priv->bu_exclsem);
  if (ret < 0)
    {
      ierr("ERROR: btn_takesem failed: %d\n", ret);
      return ret;
    }

  /* Read and return the current state of the buttons */

  lower = priv->bu_lower;
  DEBUGASSERT(lower && lower->bl_buttons);
  *(FAR btn_buttonset_t *)buffer = lower->bl_buttons(lower);

  btn_givesem(&priv->bu_exclsem);
  return (ssize_t)sizeof(btn_buttonset_t);
}

/****************************************************************************
 * Name: btn_ioctl
 ****************************************************************************/

static int btn_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct btn_upperhalf_s *priv;
  FAR struct btn_open_s *opriv;
  FAR const struct btn_lowerhalf_s *lower;
  int ret;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct btn_upperhalf_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = btn_takesem(&priv->bu_exclsem);
  if (ret < 0)
    {
      ierr("ERROR: btn_takesem failed: %d\n", ret);
      return ret;
    }

  /* Handle the ioctl command */

  ret = -EINVAL;
  switch (cmd)
    {
    /* Command:     BTNIOC_SUPPORTED
     * Description: Report the set of button events supported by the hardware;
     * Argument:    A pointer to writeable integer value in which to return the
     *              set of supported buttons.
     * Return:      Zero (OK) on success.  Minus one will be returned on failure
     *              with the errno value set appropriately.
     */

    case BTNIOC_SUPPORTED:
      {
        FAR btn_buttonset_t *supported = (FAR btn_buttonset_t *)((uintptr_t)arg);

        if (supported)
          {
            lower = priv->bu_lower;
            DEBUGASSERT(lower && lower->bl_supported);

            *supported = lower->bl_supported(lower);
            ret = OK;
          }
      }
      break;

#ifndef CONFIG_DISABLE_POLL
    /* Command:     BTNIOC_POLLEVENTS
     * Description: Specify the set of button events that can cause a poll()
     *              to awaken.  The default is all button depressions and
     *              all button releases (all supported buttons);
     * Argument:    A read-only pointer to an instance of struct
     *              btn_pollevents_s
     * Return:      Zero (OK) on success.  Minus one will be returned on
     *              failure with the errno value set appropriately.
     */

    case BTNIOC_POLLEVENTS:
      {
        FAR struct btn_pollevents_s *pollevents =
          (FAR struct btn_pollevents_s *)((uintptr_t)arg);

        if (pollevents)
          {
            /* Save the poll events */

            opriv->bo_pollevents.bp_press   = pollevents->bp_press;
            opriv->bo_pollevents.bp_release = pollevents->bp_release;

            /* Enable/disable interrupt handling */

            btn_enable(priv);
            ret = OK;
          }
      }
      break;
#endif

#ifndef CONFIG_DISABLE_SIGNALS
    /* Command:     BTNIOC_REGISTER
     * Description: Register to receive a signal whenever there is a change
     *              in any of the discrete buttone inputs.  This feature,
     *              of course, depends upon interrupt GPIO support from the
     *              platform.
     * Argument:    A read-only pointer to an instance of struct
     *              btn_notify_s
     * Return:      Zero (OK) on success.  Minus one will be returned on
     *              failure with the errno value set appropriately.
     */

    case BTNIOC_REGISTER:
      {
        FAR struct btn_notify_s *notify =
          (FAR struct btn_notify_s *)((uintptr_t)arg);

        if (notify)
          {
            /* Save the notification events */

            opriv->bo_notify.bn_press   = notify->bn_press;
            opriv->bo_notify.bn_release = notify->bn_release;
            opriv->bo_notify.bn_signo   = notify->bn_signo;
            opriv->bo_pid               = getpid();

            /* Enable/disable interrupt handling */

            btn_enable(priv);
            ret = OK;
          }
      }
      break;
#endif

    default:
      ierr("ERROR: Unrecognized command: %ld\n", cmd);
      ret = -ENOTTY;
      break;
    }

  btn_givesem(&priv->bu_exclsem);
  return ret;
}

/****************************************************************************
 * Name: btn_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int btn_poll(FAR struct file *filep, FAR struct pollfd *fds,
                    bool setup)
{
  FAR struct inode *inode;
  FAR struct btn_upperhalf_s *priv;
  FAR struct btn_open_s *opriv;
  int ret;
  int i;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct btn_upperhalf_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = btn_takesem(&priv->bu_exclsem);
  if (ret < 0)
    {
      ierr("ERROR: btn_takesem failed: %d\n", ret);
      return ret;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_BUTTONS_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!opriv->bo_fds[i])
            {
              /* Bind the poll structure and this slot */

              opriv->bo_fds[i] = fds;
              fds->priv = &opriv->bo_fds[i];
              break;
            }
        }

      if (i >= CONFIG_BUTTONS_NPOLLWAITERS)
        {
          ierr("ERROR: Too many poll waiters\n");
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout_with_dusem;
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
          goto errout_with_dusem;
        }
#endif

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

errout_with_dusem:
  btn_enable(priv);
  btn_givesem(&priv->bu_exclsem);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btn_register
 *
 * Description:
 *   Bind the lower half button driver to an instance of the upper half
 *   button driver and register the composite character driver as the
 *   specified device.
 *
 * Input Parameters:
 *   devname - The name of the button device to be registered.
 *     This should be a string of the form "/dev/btnN" where N is the
 *     minor device number.
 *   lower - An instance of the platform-specific button lower half driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int btn_register(FAR const char *devname,
                 FAR const struct btn_lowerhalf_s *lower)

{
  FAR struct btn_upperhalf_s *priv;
  int ret;

  DEBUGASSERT(devname && lower);

  /* Allocate a new button driver instance */

  priv = (FAR struct btn_upperhalf_s *)
    kmm_zalloc(sizeof(struct btn_upperhalf_s));

  if (!priv)
    {
      ierr("ERROR: Failed to allocate device structure\n");
      return -ENOMEM;
    }

  /* Make sure that all button interrupts are disabled */

  DEBUGASSERT(lower->bl_enable);
  lower->bl_enable(lower, 0, 0, NULL, NULL);

  /* Initialize the new button driver instance */

  priv->bu_lower = lower;
  nxsem_init(&priv->bu_exclsem, 0, 1);

  DEBUGASSERT(lower->bl_buttons);
  priv->bu_sample = lower->bl_buttons(lower);

  /* And register the button driver */

  ret = register_driver(devname, &btn_fops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: register_driver failed: %d\n", ret);
      goto errout_with_priv;
    }

  return OK;

errout_with_priv:
  nxsem_destroy(&priv->bu_exclsem);
  kmm_free(priv);
  return ret;
}
