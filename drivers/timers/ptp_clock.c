/****************************************************************************
 * drivers/timers/ptp_clock.c
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

#include <sys/types.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/timers/ptp_clock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct ptp_upperhalf_s
{
  FAR struct ptp_lowerhalf_s *lower;    /* The handle of lower half driver */
  mutex_t                     lock;     /* Manages exclusive access to file operations */
  long                        max_adj;  /* The maximum frequency adjustment */
  long                        adj_freq; /* remembers the frequency adjustment */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t ptp_clock_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static int     ptp_clock_ioctl(FAR struct file *filep, int cmd,
                               unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ptp_clock_file_ops =
{
  NULL,            /* open */
  NULL,            /* close */
  ptp_clock_read,  /* read */
  NULL,            /* write */
  NULL,            /* seek */
  ptp_clock_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t ptp_clock_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  return 0;
}

static inline long ptp_clock_scaled_ppm_to_ppb(long ppm)
{
  /* The 'freq' field in the 'struct timex' is in parts per
   * million, but with a 16 bit binary fractional field.
   *
   * We want to calculate
   *
   *    ppb = scaled_ppm * 1000 / 2^16
   *
   * which simplifies to
   *
   *    ppb = scaled_ppm * 125 / 2^13
   */

  int64_t ppb = 1 + ppm;

  ppb *= 125;
  ppb >>= 13;
  return (long)ppb;
}

static int ptp_clock_adjtime(FAR struct ptp_lowerhalf_s *lower,
                             FAR struct timex *tx)
{
  FAR struct ptp_upperhalf_s *upper = lower->upper;
  int ret = -ENOTSUP;

  if (tx->modes & ADJ_SETOFFSET)
    {
      struct timespec ts;
      int64_t delta;

      ts.tv_sec  = tx->time.tv_sec;
      ts.tv_nsec = tx->time.tv_usec;
      if (!(tx->modes & ADJ_NANO))
        {
          ts.tv_nsec *= 1000;
        }

      if ((unsigned long)ts.tv_nsec >= NSEC_PER_SEC)
        {
          return -EINVAL;
        }

      delta = ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
      ret = lower->ops->adjtime(lower, delta);
    }
  else if ((tx->modes & ADJ_FREQUENCY) && lower->ops->adjfine != NULL)
    {
      long ppb = ptp_clock_scaled_ppm_to_ppb(tx->freq);

      if (ppb > upper->max_adj || ppb < -upper->max_adj)
        {
          return -ERANGE;
        }

      ret = lower->ops->adjfine(lower, ppb);
      upper->adj_freq = tx->freq;
    }
  else if ((tx->modes & ADJ_OFFSET) &&
           lower->ops->adjphase != NULL)
    {
      int32_t offset = tx->offset;

      if (!(tx->modes & ADJ_NANO))
        {
          offset *= NSEC_PER_USEC;
        }

      ret = lower->ops->adjphase(lower, offset);
    }
  else if (tx->modes == 0)
    {
      tx->freq = upper->adj_freq;
      ret = 0;
    }

  return ret;
}

static int ptp_clock_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct ptp_upperhalf_s *upper = filep->f_inode->i_private;
  FAR struct ptp_lowerhalf_s *lower = upper->lower;
  int ret = -ENOTTY;
  int i;

  nxmutex_lock(&upper->lock);
  switch (cmd)
    {
      case PTP_CLOCK_SETTIME:
        {
          ret = -ENOTSUP;
          if (lower->ops->settime)
            {
              ret = lower->ops->settime(lower,
                    (FAR const struct timespec *)(uintptr_t)arg);
            }
        }
        break;

      case PTP_CLOCK_GETTIME:
        {
          ret = -ENOTSUP;
          if (lower->ops->gettime)
            {
              ret = lower->ops->gettime(lower,
                    (FAR struct timespec *)(uintptr_t)arg, NULL);
            }
        }
        break;

      case PTP_CLOCK_GETRES:
        {
          ret = -ENOTSUP;
          if (lower->ops->getres)
            {
              ret = lower->ops->getres(lower,
                    (FAR struct timespec *)(uintptr_t)arg);
            }
        }
        break;

      case PTP_CLOCK_ADJTIME:
        {
          ret = ptp_clock_adjtime(lower, (FAR struct timex *)(uintptr_t)arg);
        }
        break;

      case PTP_CLOCK_GETCAPS:
      case PTP_CLOCK_GETCAPS2:
        {
          FAR struct ptp_clock_caps *caps = (FAR struct ptp_clock_caps *)
                                            (uintptr_t)arg;

          memset(caps, 0, sizeof(*caps));
          caps->max_adj            = upper->max_adj;
          caps->cross_timestamping = lower->ops->getcrosststamp != NULL;
          caps->adjust_phase       = lower->ops->adjphase != NULL;
          ret = OK;
        }
        break;

      case PTP_SYS_OFFSET_PRECISE:
      case PTP_SYS_OFFSET_PRECISE2:
        {
          FAR struct ptp_sys_offset_precise *preoff =
                (FAR struct ptp_sys_offset_precise *)(uintptr_t)arg;
          struct system_device_crosststamp xtstamp;

          if (!lower->ops->getcrosststamp)
            {
              ret = -ENOTSUP;
              break;
            }

          ret = lower->ops->getcrosststamp(lower, &xtstamp);
          if (ret != 0)
            {
              break;
            }

          memset(preoff, 0, sizeof(*preoff));
          preoff->device.sec = xtstamp.device.tv_sec;
          preoff->device.nsec = xtstamp.device.tv_nsec;
          preoff->sys_realtime.sec = xtstamp.realtime.tv_sec;
          preoff->sys_realtime.nsec = xtstamp.realtime.tv_nsec;
          preoff->sys_monoraw.sec = xtstamp.monoraw.tv_sec;
          preoff->sys_monoraw.nsec = xtstamp.monoraw.tv_nsec;
        }
        break;

      case PTP_SYS_OFFSET_EXTENDED:
      case PTP_SYS_OFFSET_EXTENDED2:
        {
          FAR struct ptp_sys_offset_extended *extoff =
            (FAR struct ptp_sys_offset_extended *)(uintptr_t)arg;
          struct ptp_system_timestamp sts;
          struct timespec ts;

          if (!lower->ops->gettime)
            {
              ret = -ENOTSUP;
              break;
            }

          if (extoff->n_samples > PTP_MAX_SAMPLES ||
              extoff->rsv[0] || extoff->rsv[1] || extoff->rsv[2])
            {
              ret = -EINVAL;
              break;
            }

          for (i = 0; i < extoff->n_samples; i++)
            {
              ret = lower->ops->gettime(lower, &ts, &sts);
              if (ret < 0)
                {
                  break;
                }

              extoff->ts[i][0].sec  = sts.pre_ts.tv_sec;
              extoff->ts[i][0].nsec = sts.pre_ts.tv_nsec;
              extoff->ts[i][1].sec  = ts.tv_sec;
              extoff->ts[i][1].nsec = ts.tv_nsec;
              extoff->ts[i][2].sec  = sts.post_ts.tv_sec;
              extoff->ts[i][2].nsec = sts.post_ts.tv_nsec;
            }
        }
        break;

      case PTP_SYS_OFFSET:
      case PTP_SYS_OFFSET2:
        {
          FAR struct ptp_sys_offset *sysoff =
            (FAR struct ptp_sys_offset *)(uintptr_t)arg;
          FAR struct ptp_clock_time *pct;
          struct timespec ts;

          if (lower->ops->gettime == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          if (sysoff->n_samples > PTP_MAX_SAMPLES)
            {
              ret = -EINVAL;
              break;
            }

          pct = &sysoff->ts[0];
          for (i = 0; i < sysoff->n_samples; i++)
            {
              nxclock_gettime(CLOCK_REALTIME, &ts);
              pct->sec  = ts.tv_sec;
              pct->nsec = ts.tv_nsec;
              pct++;
              ret = lower->ops->gettime(lower, &ts, NULL);
              if (ret < 0)
                {
                  break;
                }

              pct->sec  = ts.tv_sec;
              pct->nsec = ts.tv_nsec;
              pct++;
            }

          nxclock_gettime(CLOCK_REALTIME, &ts);
          pct->sec  = ts.tv_sec;
          pct->nsec = ts.tv_nsec;
        }
        break;

      default:
        {
          if (lower->ops->control)
            {
              ret = lower->ops->control(lower, cmd, arg);
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
 * Name: ptp_clockid_to_filep
 *
 * Description:
 *   Convert clockid to struct filep.
 *
 ****************************************************************************/

int ptp_clockid_to_filep(clockid_t clock_id, FAR struct file **filep)
{
  FAR const struct file_operations *ops;
  int ret;

  if ((clock_id & CLOCK_MASK) != CLOCK_FD)
    {
      return -EINVAL;
    }

  ret = clock_id >> CLOCK_SHIFT;
  if (ret >= 0)
    {
      ret = fs_getfilep(ret, filep);
    }

  if (ret < 0)
    {
      return ret;
    }

  ops = (*filep)->f_inode->u.i_ops;
  if (ops != &g_ptp_clock_file_ops)
    {
      fs_putfilep(*filep);
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: ptp_clock_register
 *
 * Description:
 *   This function binds an instance of a "lower half" ptp driver with the
 *   "upper half" ptp device and registers that device so that can be used
 *   by application code.
 *
 * Input Parameters:
 *   lower   - A pointer to an instance of lower half ptp driver. This
 *             instance is bound to the ptp driver and must persists as long
 *             as the driver persists.
 *   mxa_adj - The maximum frequency adjustment in parts per billion.
 *   devno   - The user specifies number of device. ex: /dev/ptpX.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int ptp_clock_register(FAR struct ptp_lowerhalf_s *lower, int32_t max_adj,
                       int devno)
{
  FAR struct ptp_upperhalf_s *upper;
  char path[16];
  int ret;

  DEBUGASSERT(lower != NULL);

  /* Allocate the upper-half data structure */

  upper = kmm_zalloc(sizeof(struct ptp_upperhalf_s));
  if (!upper)
    {
      ptperr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  upper->lower   = lower;
  upper->max_adj = max_adj;
  lower->upper   = upper;
  nxmutex_init(&upper->lock);

  snprintf(path, sizeof(path), "/dev/ptp%d", devno);
  ptpinfo("Registering %s\n", path);
  ret = register_driver(path, &g_ptp_clock_file_ops, 0666, upper);
  if (ret < 0)
    {
      nxmutex_destroy(&upper->lock);
      kmm_free(upper);
    }

  return ret;
}

/****************************************************************************
 * Name: ptp_clock_unregister
 *
 * Description:
 *   This function unregister character node and release all resource about
 *   upper half driver.
 *
 * Input Parameters:
 *   lower - A pointer to an instance of lower half ptp driver. This
 *           instance is bound to the ptp driver and must persists as long
 *           as the driver persists.
 *   devno - The user specifies which device of this type, from 0.
 ****************************************************************************/

void ptp_clock_unregister(FAR struct ptp_lowerhalf_s *lower, int devno)
{
  FAR struct ptp_upperhalf_s *upper = lower->upper;

  if (upper != NULL)
    {
      char path[16];

      snprintf(path, sizeof(path), "/dev/ptp%d", devno);
      unregister_driver(path);
      nxmutex_destroy(&upper->lock);
      kmm_free(upper);
    }
}
