/****************************************************************************
 * drivers/timers/oneshot.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
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
#include <fcntl.h>
#include <semaphore.h>
#include <signal.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/timers/oneshot.h>

#ifdef CONFIG_ONESHOT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DISABLE_SIGNALS
#  error "This driver needs SIGNAL support, remove CONFIG_DISABLE_SIGNALS"
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct oneshot_dev_s
{
  FAR struct oneshot_lowerhalf_s *od_lower;    /* Lower-half driver state */
  sem_t od_exclsem;                            /* Supports mutual exclusion */

  /* Oneshot timer expiration notification information */

  uint8_t od_signo;                            /* Signal number for notification */
  pid_t od_pid;                                /* PID to be notified */
  FAR void *od_arg;                            /* Signal value argument */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     oneshot_open(FAR struct file *filep);
static int     oneshot_close(FAR struct file *filep);
static ssize_t oneshot_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t oneshot_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     oneshot_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);

static void    oneshot_callback(FAR struct oneshot_lowerhalf_s *lower,
                 FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_oneshot_ops =
{
  oneshot_open,  /* open */
  oneshot_close, /* close */
  oneshot_read,  /* read */
  oneshot_write, /* write */
  0,             /* seek */
  oneshot_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0            /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , 0            /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_callback
 ****************************************************************************/

static void oneshot_callback(FAR struct oneshot_lowerhalf_s *lower,
                             FAR void *arg)
{
  FAR struct oneshot_dev_s *priv = (FAR struct oneshot_dev_s *)arg;
#ifdef CONFIG_CAN_PASS_STRUCTS
  union sigval value;
#endif

  DEBUGASSERT(priv != NULL);

  /* Signal the waiter.. if there is one */

#ifdef CONFIG_CAN_PASS_STRUCTS
  value.sival_ptr = priv->od_arg;
  (void)nxsig_queue(priv->od_pid, priv->od_signo, value);
#else
  (void)nxsig_queue(priv->od_pid, priv->od_signo, priv->od_arg);
#endif
}

/************************************************************************************
 * Name: oneshot_open
 *
 * Description:
 *   This function is called whenever the PWM device is opened.
 *
 ************************************************************************************/

static int oneshot_open(FAR struct file *filep)
{
  tmrinfo("Opening...\n");
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  return OK;
}

/************************************************************************************
 * Name: oneshot_close
 *
 * Description:
 *   This function is called when the PWM device is closed.
 *
 ************************************************************************************/

static int oneshot_close(FAR struct file *filep)
{
  tmrinfo("Closing...\n");
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  return OK;
}

/************************************************************************************
 * Name: oneshot_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t oneshot_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  tmrinfo("buflen=%ld\n", (unsigned long)buflen);
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  return 0;
}

/************************************************************************************
 * Name: oneshot_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t oneshot_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  /* Return a failure */

  tmrinfo("buflen=%ld\n", (unsigned long)buflen);
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  return -EPERM;
}

/************************************************************************************
 * Name: oneshot_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the PWM work is done.
 *
 ************************************************************************************/

static int oneshot_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct oneshot_dev_s *priv;
  int ret;

  tmrinfo("cmd=%d arg=%08lx\n", cmd, (unsigned long)arg);

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  priv  = (FAR struct oneshot_dev_s *)inode->i_private;
  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->od_exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle oneshot timer ioctl commands */

  switch (cmd)
    {
      /* OSIOC_MAXDELAY - Return the maximum delay that can be supported
       *                  by this timer.
       *                  Argument: A referenct to a struct timespec in
       *                  which the maximum time will be returned.
       */

      case OSIOC_MAXDELAY:
        {
          FAR struct timespec *ts = (FAR struct timespec *)((uintptr_t)arg);
          DEBUGASSERT(ts != NULL);

          ret = ONESHOT_MAX_DELAY(priv->od_lower, ts);
        }
        break;

      /* OSIOC_START - Start the oneshot timer
       *               Argument: A reference to struct oneshot_start_s
       */

      case OSIOC_START:
        {
          FAR struct oneshot_start_s *start;
          pid_t pid;

          start = (FAR struct oneshot_start_s *)((uintptr_t)arg);
          DEBUGASSERT(start != NULL);

          /* Save signalling information */

          priv->od_signo = start->signo;
          priv->od_arg   = start->arg;

          pid = start->pid;
          if (pid == 0)
            {
              pid = getpid();
            }

          priv->od_pid = pid;

          /* Start the oneshot timer */

          ret = ONESHOT_START(priv->od_lower, oneshot_callback, priv,
                              &start->ts);
        }
        break;

      /* OSIOC_CANCEL - Stop the timer
       *                Argument: A reference to a struct timespec in
       *                which the time remaining will be returned.
       */

      case OSIOC_CANCEL:
        {
          FAR struct timespec *ts = (FAR struct timespec *)((uintptr_t)arg);

          /* Cancel the oneshot timer */

          ret = ONESHOT_CANCEL(priv->od_lower, ts);
        }
        break;

      /* OSIOC_CURRENT - Get the current time
       *                 Argument: A reference to a struct timespec in
       *                 which the current time will be returned.
       */

      case OSIOC_CURRENT:
        {
          FAR struct timespec *ts = (FAR struct timespec *)((uintptr_t)arg);

          /* Get the current time */

          ret = ONESHOT_CURRENT(priv->od_lower, ts);
        }
        break;

      default:
        {
          tmrerr("ERROR: Unrecognized cmd: %d arg: %ld\n", cmd, arg);
          ret = -ENOTTY;
        }
        break;
    }

  nxsem_post(&priv->od_exclsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_register
 *
 * Description:
 *   Register the oneshot device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/oneshot0"
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

int oneshot_register(FAR const char *devname,
                     FAR struct oneshot_lowerhalf_s *lower)
{
  FAR struct oneshot_dev_s *priv;
  int ret;

  sninfo("devname=%s lower=%p\n", devname, lower);
  DEBUGASSERT(devname != NULL && lower != NULL);

  /* Allocate a new oneshot timer driver instance */

  priv = (FAR struct oneshot_dev_s *)
    kmm_zalloc(sizeof(struct oneshot_dev_s));

  if (!priv)
    {
      snerr("ERROR: Failed to allocate device structure\n");
      return -ENOMEM;
    }

  /* Initialize the new oneshot timer driver instance */

  priv->od_lower = lower;
  nxsem_init(&priv->od_exclsem, 0, 1);

  /* And register the oneshot timer driver */

  ret = register_driver(devname, &g_oneshot_ops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: register_driver failed: %d\n", ret);
      nxsem_destroy(&priv->od_exclsem);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_ONESHOT */
