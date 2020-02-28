/****************************************************************************
 * drivers/sensors/zerocross.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/zerocross.h>

#include <nuttx/irq.h>

#ifdef CONFIG_SENSORS_ZEROCROSS

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct zc_upperhalf_s
{
  FAR struct zc_lowerhalf_s *lower;    /* lower-half state */
  sem_t                      exclsem;  /* Supports mutual exclusion */

  /* The following is a singly linked list of open references to the
   * zero cross device.
   */

  FAR struct zc_open_s *zu_open;
};

/* This structure describes the state of one open zero cross driver instance */

struct zc_open_s
{
  /* Supports a singly linked list */

  FAR struct zc_open_s *do_flink;

  /* The following will be true if we are closing */

  volatile bool do_closing;

  /* Zero cross event notification information */

  pid_t do_pid;
  struct sigevent do_event;
  struct sigwork_s do_work;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     zc_open(FAR struct file *filep);
static int     zc_close(FAR struct file *filep);
static ssize_t zc_read(FAR struct file *filep, FAR char *buffer, size_t
                 buflen);
static ssize_t zc_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     zc_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static void    zerocross_enable(FAR struct zc_upperhalf_s *priv);
static void    zerocross_interrupt(FAR const struct zc_lowerhalf_s *lower,
                 FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_zcops =
{
  zc_open,   /* open */
  zc_close,  /* close */
  zc_read,   /* read */
  zc_write,  /* write */
  NULL,      /* seek */
  zc_ioctl,  /* ioctl */
  NULL       /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , 0        /* unlink */
#endif
};

volatile int sample = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: zerocross_enable
 ****************************************************************************/

static void zerocross_enable(FAR struct zc_upperhalf_s *priv)
{
  FAR const struct zc_lowerhalf_s *lower;
  irqstate_t flags;

  DEBUGASSERT(priv && priv->lower);
  lower = priv->lower;

  /* This routine is called both task level and interrupt level, so
   * interrupts must be disabled.
   */

  flags = enter_critical_section();

  /* Enable interrupts */

  DEBUGASSERT(lower->zc_enable);

  /* Enable interrupts with the new button set */

  lower->zc_enable(lower, (zc_interrupt_t)zerocross_interrupt, priv);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: zerocross_interrupt
 ****************************************************************************/

static void zerocross_interrupt(FAR const struct zc_lowerhalf_s *lower,
                                FAR void *arg)
{
  FAR struct zc_upperhalf_s *priv = (FAR struct zc_upperhalf_s *)arg;
  FAR struct zc_open_s *opriv;
  irqstate_t flags;

  /* This routine is called both task level and interrupt level, so
   * interrupts must be disabled.
   */

  flags = enter_critical_section();

  /* Update sample value */

  sample++;

  /* Visit each opened reference and notify a zero cross event */

  for (opriv = priv->zu_open; opriv; opriv = opriv->do_flink)
    {
      /* Signal the waiter */

      opriv->do_event.sigev_value.sival_int = sample;
      nxsig_notification(opriv->do_pid, &opriv->do_event,
                         SI_QUEUE, &opriv->do_work);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: zc_open
 *
 * Description:
 *   This function is called whenever the PWM device is opened.
 *
 ****************************************************************************/

static int zc_open(FAR struct file *filep)
{
  FAR struct inode                *inode;
  FAR struct zc_upperhalf_s       *priv;
  FAR struct zc_open_s            *opriv;
  int ret;

  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = (FAR struct zc_upperhalf_s *)inode->i_private;

  /* Get exclusive access to the driver structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      snerr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Allocate a new open structure */

  opriv = (FAR struct zc_open_s *)kmm_zalloc(sizeof(struct zc_open_s));
  if (!opriv)
    {
      snerr("ERROR: Failed to allocate open structure\n");
      ret = -ENOMEM;
      goto errout_with_sem;
    }

  /* Attach the open structure to the device */

  opriv->do_flink = priv->zu_open;
  priv->zu_open = opriv;

  /* Attach the open structure to the file structure */

  filep->f_priv = (FAR void *)opriv;
  ret = OK;

errout_with_sem:
  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: zc_close
 *
 * Description:
 *   This function is called when the PWM device is closed.
 *
 ****************************************************************************/

static int zc_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct zc_upperhalf_s *priv;
  FAR struct zc_open_s *opriv;
  FAR struct zc_open_s *curr;
  FAR struct zc_open_s *prev;
  irqstate_t flags;
  bool closing;
  int ret;

  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = (FAR struct zc_upperhalf_s *)inode->i_private;

  /* Handle an improbable race conditions with the following atomic test
   * and set.
   *
   * This is actually a pretty feeble attempt to handle this.  The
   * improbable race condition occurs if two different threads try to
   * close the zero cross driver at the same time.  The rule:  don't do
   * that!  It is feeble because we do not really enforce stale pointer
   * detection anyway.
   */

  flags = enter_critical_section();
  closing = opriv->do_closing;
  opriv->do_closing = true;
  leave_critical_section(flags);

  if (closing)
    {
      /* Another thread is doing the close */

      return OK;
    }

  /* Get exclusive access to the driver structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      snerr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Find the open structure in the list of open structures for the device */

  for (prev = NULL, curr = priv->zu_open;
       curr && curr != opriv;
       prev = curr, curr = curr->do_flink);

  DEBUGASSERT(curr);
  if (!curr)
    {
      snerr("ERROR: Failed to find open entry\n");
      ret = -ENOENT;
      goto errout_with_exclsem;
    }

  /* Remove the structure from the device */

  if (prev)
    {
      prev->do_flink = opriv->do_flink;
    }
  else
    {
      priv->zu_open = opriv->do_flink;
    }

  /* Cancel any pending notification */

  nxsig_cancel_notification(&opriv->do_work);

  /* And free the open structure */

  kmm_free(opriv);

  /* Enable/disable interrupt handling */

  zerocross_enable(priv);
  ret = OK;

errout_with_exclsem:
  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: zc_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satsify the VFS layer.
 *
 ****************************************************************************/

static ssize_t zc_read(FAR struct file *filep, FAR char *buffer,
                       size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: zc_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satsify the VFS layer.
 *
 ****************************************************************************/

static ssize_t zc_write(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen)
{
  /* Return a failure */

  return -EPERM;
}

/****************************************************************************
 * Name: zc_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the PWM work is done.
 *
 ****************************************************************************/

static int zc_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode          *inode;
  FAR struct zc_upperhalf_s *priv;
  FAR struct zc_open_s      *opriv;
  int                        ret;

  sninfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep && filep->f_priv && filep->f_inode);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = (FAR struct zc_upperhalf_s *)inode->i_private;

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle built-in ioctl commands */

  ret = -EINVAL;
  switch (cmd)
    {
      /* Command:     ZCIOC_REGISTER
       * Description: Register to receive a signal whenever there is zero
       *              cross detection interrupt.
       * Argument:    A read-only pointer to an instance of struct
       *              zc_notify_s
       * Return:      Zero (OK) on success.  Minus one will be returned on
       *              failure with the errno value set appropriately.
       */

      case ZCIOC_REGISTER:
        {
          FAR struct sigevent *event =
            (FAR struct sigevent *)((uintptr_t)arg);

          if (event)
            {
              /* Save the notification events */

              opriv->do_event = *event;
              opriv->do_pid   = getpid();

              /* Enable/disable interrupt handling */

              zerocross_enable(priv);
              ret = OK;
            }
        }
        break;

      default:
        {
          snerr("ERROR: Unrecognized cmd: %d arg: %ld\n", cmd, arg);
          ret = -ENOTTY;
        }
        break;
    }

  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: zc_register
 *
 * Description:
 *   Register the Zero Cross character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/zc0"
 *   lower - An instance of the lower half interface
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.  The following
 *   possible error values may be returned (most are returned by
 *   register_driver()):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int zc_register(FAR const char *devname, FAR struct zc_lowerhalf_s *lower)
{
  FAR struct zc_upperhalf_s *priv;
  int ret;

  DEBUGASSERT(devname && lower);

  /* Allocate a new zero cross driver instance */

  priv = (FAR struct zc_upperhalf_s *)
    kmm_zalloc(sizeof(struct zc_upperhalf_s));

  if (!priv)
    {
      snerr("ERROR: Failed to allocate device structure\n");
      return -ENOMEM;
    }

  /* Make sure that zero cross interrupt is disabled */

  DEBUGASSERT(lower->zc_enable);
  lower->zc_enable(lower, NULL, NULL);

  /* Initialize the new zero cross driver instance */

  priv->lower = lower;
  nxsem_init(&priv->exclsem, 0, 1);

  /* And register the zero cross driver */

  ret = register_driver(devname, &g_zcops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: register_driver failed: %d\n", ret);
      nxsem_destroy(&priv->exclsem);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_SENSORS_ZEROCROSS */
