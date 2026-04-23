/****************************************************************************
 * drivers/timers/ocu.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/signal.h>
#include <nuttx/timers/ocu.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_TIMER_INFO
#  define ocuinfo  _info
#else
#  define ocuinfo  _none
#endif

#ifdef CONFIG_DEBUG_TIMER_ERROR
#  define ocuerr   _err
#else
#  define ocuerr   _none
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct ocu_upperhalf_s
{
  uint8_t                     crefs;    /* The number of times the device
                                         * has been opened */
  volatile bool               started;  /* True: channel is running */
  mutex_t                     lock;     /* Supports mutual exclusion */
  FAR struct ocu_lowerhalf_s *lower;    /* lower-half state */
  struct sigwork_s            work;     /* Signal work for deferred notify */
  struct ocu_notify_s         notify;   /* Notification configuration */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     ocu_open(FAR struct file *filep);
static int     ocu_close(FAR struct file *filep);
static ssize_t ocu_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t ocu_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen);
static int     ocu_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ocu_ops =
{
  ocu_open,   /* open */
  ocu_close,  /* close */
  ocu_read,   /* read */
  ocu_write,  /* write */
  NULL,       /* seek */
  ocu_ioctl,  /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ocu_notifier
 *
 * Description:
 *   Internal callback registered with the lower-half driver when user-space
 *   notification is enabled via OCUIOC_NOTIFICATION.  This function is
 *   called from ISR context on compare match and sends a signal to the
 *   registered process.
 *
 *   This follows the same pattern as timer_notifier() in timer.c:
 *   the upper-half registers this as the lower-half callback and translates
 *   the event into a user-space signal notification.
 *
 *   Kernel-internal drivers that need a direct callback should use the
 *   OCU_SETCALLBACK() macro to register directly with the lower-half,
 *   bypassing this upper-half entirely (same as TIMER_SETCALLBACK).
 *
 ****************************************************************************/

static bool ocu_notifier(FAR uint32_t *next_threshold, FAR void *arg)
{
  FAR struct ocu_upperhalf_s *upper = (FAR struct ocu_upperhalf_s *)arg;

  DEBUGASSERT(upper != NULL);

  /* Signal the waiter */

  nxsig_notification(upper->notify.pid, &upper->notify.event,
                     SI_QUEUE, &upper->work);

  /* Do not re-arm threshold from upper-half.  User-space must explicitly
   * set the next threshold via ioctl after receiving the signal.
   */

  return false;
}

/****************************************************************************
 * Name: ocu_open
 ****************************************************************************/

static int ocu_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct ocu_upperhalf_s *upper = inode->i_private;
  int                         ret   = OK;
  FAR struct ocu_lowerhalf_s *lower;
  uint8_t                     tmp;

  ocuinfo("crefs: %d\n", upper->crefs);

  nxmutex_lock(&upper->lock);

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      ret = -EMFILE;
      goto errout_with_lock;
    }

  /* If this is the first open, call setup */

  if (upper->crefs == 0)
    {
      lower = upper->lower;

      if (lower->ops->setup != NULL)
        {
          ret = lower->ops->setup(lower);
          if (ret < 0)
            {
              goto errout_with_lock;
            }
        }
    }

  upper->crefs = tmp;

errout_with_lock:
  nxmutex_unlock(&upper->lock);
  return ret;
}

/****************************************************************************
 * Name: ocu_close
 ****************************************************************************/

static int ocu_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct ocu_upperhalf_s *upper = inode->i_private;

  ocuinfo("crefs: %d\n", upper->crefs);

  nxmutex_lock(&upper->lock);

  if (upper->crefs > 1)
    {
      upper->crefs--;
    }
  else
    {
      FAR struct ocu_lowerhalf_s *lower = upper->lower;

      upper->crefs = 0;

      /* Stop the channel if still running */

      if (upper->started)
        {
          OCU_STOP(lower);
          upper->started = false;
        }

      /* Disable hardware notification, unregister callback and shutdown */

      OCU_NOTIFICATION(lower, false);
      OCU_SETCALLBACK(lower, NULL, NULL);

      if (lower->ops->shutdown != NULL)
        {
          lower->ops->shutdown(lower);
        }
    }

  nxmutex_unlock(&upper->lock);
  return OK;
}

/****************************************************************************
 * Name: ocu_read
 ****************************************************************************/

static ssize_t ocu_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: ocu_write
 ****************************************************************************/

static ssize_t ocu_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: ocu_ioctl
 ****************************************************************************/

static int ocu_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct ocu_upperhalf_s *upper;
  FAR struct ocu_lowerhalf_s *lower;
  int                         ret;

  ocuinfo("cmd: %d arg: %ld\n", cmd, arg);
  upper = inode->i_private;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  nxmutex_lock(&upper->lock);

  switch (cmd)
    {
      /* OCUIOC_START - Start the OCU channel */

      case OCUIOC_START:
        {
          ocuinfo("OCUIOC_START: started=%d\n", upper->started);

          if (upper->started)
            {
              ret = -EBUSY;
            }
          else
            {
              ret = OCU_START(lower);
              if (ret == OK)
                {
                  upper->started = true;
                }
            }
        }
        break;

      /* OCUIOC_STOP - Stop the OCU channel */

      case OCUIOC_STOP:
        {
          ocuinfo("OCUIOC_STOP: started=%d\n", upper->started);

          if (upper->started)
            {
              ret = OCU_STOP(lower);
              upper->started = false;
            }
          else
            {
              ret = OK;  /* Already stopped, not an error */
            }
        }
        break;

      /* OCUIOC_GETSTATUS - Get the current OCU channel status */

      case OCUIOC_GETSTATUS:
        {
          FAR struct ocu_status_s *status =
            (FAR struct ocu_status_s *)((uintptr_t)arg);
          DEBUGASSERT(status != NULL);

          /* Let the lower-half fill in the hardware/config-specific fields
           * (counter, threshold, maxcounter and the PIN_AVAIL flag).  It
           * zeroes status->flags, so the upper-half must OR in its own
           * flags afterwards.
           */

          ret = OCU_GETSTATUS(lower, status);
          if (ret == OK)
            {
              /* The running and notification states are tracked by the
               * upper-half, so it fills those flags here instead of the
               * lower-half duplicating them.
               */

              if (upper->started)
                {
                  status->flags |= OCUFLAGS_ACTIVE;
                }

              if (upper->notify.enable)
                {
                  status->flags |= OCUFLAGS_NOTIF_EN;
                }
            }
        }
        break;

      /* OCUIOC_GETCOUNTER - Read the current counter value */

      case OCUIOC_GETCOUNTER:
        {
          FAR uint32_t *counter = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(counter != NULL);
          ret = OCU_GETCOUNTER(lower, counter);
        }
        break;

      /* OCUIOC_SETABSTHRESHOLD - Set absolute compare threshold */

      case OCUIOC_SETABSTHRESHOLD:
        {
          FAR struct ocu_abs_threshold_s *threshold =
            (FAR struct ocu_abs_threshold_s *)((uintptr_t)arg);
          DEBUGASSERT(threshold != NULL);
          ret = OCU_SETABSTHRESHOLD(lower, threshold);
        }
        break;

      /* OCUIOC_SETRELTHRESHOLD - Set relative compare threshold */

      case OCUIOC_SETRELTHRESHOLD:
        {
          FAR struct ocu_rel_threshold_s *threshold =
            (FAR struct ocu_rel_threshold_s *)((uintptr_t)arg);
          DEBUGASSERT(threshold != NULL);
          ret = OCU_SETRELTHRESHOLD(lower, threshold);
        }
        break;

      /* OCUIOC_SETPINSTATE - Set the output pin level */

      case OCUIOC_SETPINSTATE:
        {
          enum ocu_pinstate_e state = (enum ocu_pinstate_e)arg;

          if (upper->started)
            {
              ret = -EBUSY;  /* Cannot set pin state while running */
            }
          else
            {
              ret = OCU_SETPINSTATE(lower, state);
            }
        }
        break;

      /* OCUIOC_SETPINACTION - Set the automatic pin action */

      case OCUIOC_SETPINACTION:
        {
          enum ocu_pinaction_e action = (enum ocu_pinaction_e)arg;
          ret = OCU_SETPINACTION(lower, action);
        }
        break;

      /* OCUIOC_SETADVANCETHRESHOLD - Advance threshold by step */

      case OCUIOC_SETADVANCETHRESHOLD:
        {
          FAR struct ocu_advance_threshold_s *threshold =
            (FAR struct ocu_advance_threshold_s *)((uintptr_t)arg);
          DEBUGASSERT(threshold != NULL);
          ret = OCU_SETADVANCETHRESHOLD(lower, threshold);
        }
        break;

      /* OCUIOC_NOTIFICATION - Enable/disable signal notification.
       *
       * When enabled, the upper-half registers ocu_notifier() as the
       * lower-half callback.  On compare match, the lower-half calls
       * ocu_notifier() from ISR context, which sends a signal to the
       * registered process.
       *
       * Kernel-internal callers that need a direct ISR callback should
       * use the OCU_SETCALLBACK() macro instead of this ioctl.
       */

      case OCUIOC_NOTIFICATION:
        {
          FAR struct ocu_notify_s *notify =
            (FAR struct ocu_notify_s *)((uintptr_t)arg);
          DEBUGASSERT(notify != NULL);

          if (notify->enable)
            {
              memcpy(&upper->notify, notify, sizeof(*notify));

              /* Register the notifier callback, then enable the hardware
               * compare-match notification.
               */

              OCU_SETCALLBACK(lower, ocu_notifier, upper);
              ret = OCU_NOTIFICATION(lower, true);
              if (ret < 0)
                {
                  /* Roll back the callback registration on failure */

                  OCU_SETCALLBACK(lower, NULL, NULL);
                  memset(&upper->notify, 0, sizeof(upper->notify));
                }
            }
          else
            {
              /* Disable the hardware notification, then unregister the
               * callback.
               */

              ret = OCU_NOTIFICATION(lower, false);
              OCU_SETCALLBACK(lower, NULL, NULL);
              memset(&upper->notify, 0, sizeof(upper->notify));
            }
        }
        break;

      /* Any unrecognized IOCTL commands are forwarded to the lower half */

      default:
        {
          ocuinfo("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
          if (lower->ops->ioctl != NULL)
            {
              ret = lower->ops->ioctl(lower, cmd, arg);
            }
          else
            {
              ret = -ENOTTY;
            }
        }
        break;
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ocu_register
 ****************************************************************************/

int ocu_register(FAR const char *devpath,
                 FAR struct ocu_lowerhalf_s *lower)
{
  FAR struct ocu_upperhalf_s *upper;

  DEBUGASSERT(devpath != NULL && lower != NULL && lower->ops != NULL);

  /* Allocate the upper-half data structure */

  upper = kmm_zalloc(sizeof(struct ocu_upperhalf_s));
  if (upper == NULL)
    {
      ocuerr("Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the OCU device structure */

  nxmutex_init(&upper->lock);
  upper->lower = lower;

  /* Register the OCU device */

  ocuinfo("Registering %s\n", devpath);
  return register_driver(devpath, &g_ocu_ops, 0600, upper);
}
