/****************************************************************************
 * drivers/timers/oneshot.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/timers/oneshot.h>

#ifdef CONFIG_ONESHOT

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct oneshot_dev_s
{
  FAR struct oneshot_lowerhalf_s *od_lower;    /* Lower-half driver state */
  mutex_t od_lock;                             /* Supports mutual exclusion */

  /* Oneshot timer expiration notification information */

  struct sigevent od_event;                    /* Signal info */
  struct sigwork_s od_work;                    /* Signal work */
  pid_t od_pid;                                /* PID to be notified */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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
  NULL,          /* open */
  NULL,          /* close */
  oneshot_read,  /* read */
  oneshot_write, /* write */
  NULL,          /* seek */
  oneshot_ioctl, /* ioctl */
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

  DEBUGASSERT(priv != NULL);

  /* Signal the waiter.. if there is one */

  nxsig_notification(priv->od_pid, &priv->od_event, SI_QUEUE,
                     &priv->od_work);
}

/****************************************************************************
 * Name: oneshot_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satsify the VFS layer.
 *
 ****************************************************************************/

static ssize_t oneshot_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  tmrinfo("buflen=%ld\n", (unsigned long)buflen);
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  return 0;
}

/****************************************************************************
 * Name: oneshot_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satsify the VFS layer.
 *
 ****************************************************************************/

static ssize_t oneshot_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  /* Return a failure */

  tmrinfo("buflen=%ld\n", (unsigned long)buflen);
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  return -EPERM;
}

/****************************************************************************
 * Name: oneshot_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the PWM work is done.
 *
 ****************************************************************************/

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

  ret = nxmutex_lock(&priv->od_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle oneshot timer ioctl commands */

  switch (cmd)
    {
      /* OSIOC_MAXDELAY - Return the maximum delay that can be supported
       *                  by this timer.
       *                  Argument: A reference to a struct timespec in
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

          /* Save signaling information */

          priv->od_event = start->event;

          pid = start->pid;
          if (pid == 0)
            {
              pid = nxsched_getpid();
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
          nxsig_cancel_notification(&priv->od_work);
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

  nxmutex_unlock(&priv->od_lock);
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

  tmrinfo("devname=%s lower=%p\n", devname, lower);
  DEBUGASSERT(devname != NULL && lower != NULL);

  /* Allocate a new oneshot timer driver instance */

  priv = (FAR struct oneshot_dev_s *)
    kmm_zalloc(sizeof(struct oneshot_dev_s));

  if (!priv)
    {
      tmrerr("ERROR: Failed to allocate device structure\n");
      return -ENOMEM;
    }

  /* Initialize the new oneshot timer driver instance */

  priv->od_lower = lower;
  nxmutex_init(&priv->od_lock);

  /* And register the oneshot timer driver */

  ret = register_driver(devname, &g_oneshot_ops, 0666, priv);
  if (ret < 0)
    {
      tmrerr("ERROR: register_driver failed: %d\n", ret);
      nxmutex_destroy(&priv->od_lock);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_ONESHOT */
