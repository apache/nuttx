/****************************************************************************
 * drivers/rc/lirc_dev.c
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

#include <stdio.h>
#include <sys/param.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <poll.h>
#include <fcntl.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/mm/circbuf.h>
#include <nuttx/rc/lirc_dev.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVNAME_FMT        "/dev/lirc%d"
#define DEVNAME_MAX        32

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct lirc_upperhalf_s
{
  struct list_node             fh;           /* list of struct lirc_fh_s object */
  FAR struct lirc_lowerhalf_s *lower;        /* the handle of lower half driver */
  mutex_t                      lock;         /* Manages exclusive access to lowerhalf */
  bool                         gap;          /* true if we're in a gap */
  uint64_t                     gap_start;    /* time when gap starts */
  uint64_t                     gap_duration; /* duration of initial gap */
};

/* The structure describes an open lirc file */

struct lirc_fh_s
{
  struct list_node             node;         /* list of open file handles */
  FAR struct lirc_lowerhalf_s *lower;        /* the pointer to lirc_lowerhalf_s */
  struct circbuf_s             buffer;       /* buffer for incoming IR */
  FAR struct pollfd           *fd;           /* poll structures of threads waiting for driver events */
  sem_t                        waitsem;      /* sem of wait buffer for ready */
  int                          carrier_low;  /* when setting the carrier range, first the low end must be
                                              * set with an ioctl and then the high end with another ioctl
                                              */
  unsigned char                send_mode;    /* lirc mode for sending, LIRC_MODE_PULSE LIRC_MODE_SCANCODE */
  unsigned char                rec_mode;     /* lirc mode for receiving, LIRC_MODE_MODE2 LIRC_MODE_SCANCODE */
  bool            send_timeout_reports;      /* report timeouts in lirc raw IR. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lirc_open(FAR struct file *filep);
static int lirc_close(FAR struct file *filep);
static int lirc_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int lirc_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup);
static ssize_t lirc_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static ssize_t lirc_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lirc_fops =
{
  lirc_open,   /* open */
  lirc_close,  /* close */
  lirc_read,   /* read */
  lirc_write,  /* write */
  NULL,        /* seek */
  lirc_ioctl,  /* ioctl */
  NULL,        /* mmap */
  NULL,        /* truncate */
  lirc_poll    /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int lirc_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lirc_upperhalf_s *upper = inode->i_private;
  FAR struct lirc_lowerhalf_s *lower = upper->lower;
  FAR struct lirc_fh_s *fh;
  irqstate_t flags;
  int ret;

  fh = kmm_zalloc(sizeof(*fh));
  if (!fh)
    {
      return -ENOMEM;
    }

  switch (lower->ops->driver_type)
    {
      case LIRC_DRIVER_SCANCODE:
        fh->rec_mode = LIRC_MODE_SCANCODE;
        break;
      default:
        fh->rec_mode = LIRC_MODE_MODE2;
        break;
    }

  if (lower->ops->tx_scancode)
    {
      fh->send_mode = LIRC_MODE_SCANCODE;
    }
  else if (lower->ops->tx_ir)
    {
      fh->send_mode = LIRC_MODE_PULSE;
    }

  ret = circbuf_init(&fh->buffer, NULL, lower->buffer_bytes);
  if (ret < 0)
    {
      goto buffer_err;
    }

  nxsem_init(&fh->waitsem, 0, 0);

  fh->lower = lower;
  fh->send_timeout_reports = true;

  if (list_is_empty(&upper->fh))
    {
      ret = lower->ops->open(lower);
      if (ret < 0)
        {
          goto open_err;
        }
    }

  flags = enter_critical_section();
  list_add_tail(&upper->fh, &fh->node);
  leave_critical_section(flags);

  filep->f_priv = fh;
  return 0;

open_err:
  nxsem_destroy(&fh->waitsem);
  circbuf_uninit(&fh->buffer);
buffer_err:
  kmm_free(fh);
  return ret;
}

static int lirc_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lirc_upperhalf_s *upper = inode->i_private;
  FAR struct lirc_fh_s *fh = filep->f_priv;
  FAR struct lirc_lowerhalf_s *lower = fh->lower;
  irqstate_t flags;

  flags = enter_critical_section();
  list_delete(&fh->node);
  leave_critical_section(flags);

  nxsem_destroy(&fh->waitsem);
  circbuf_uninit(&fh->buffer);

  kmm_free(fh);
  if (list_is_empty(&upper->fh))
    {
      lower->ops->close(lower);
    }

  return 0;
}

static int lirc_poll(FAR struct file *filep,
                     FAR struct pollfd *fds, bool setup)
{
  FAR struct lirc_fh_s *fh = filep->f_priv;
  pollevent_t eventset = 0;
  irqstate_t flags;
  int ret = 0;

  flags = enter_critical_section();
  if (setup)
    {
      if (fh->fd)
        {
          ret = -EBUSY;
          goto errout;
        }

      fh->fd = fds;
      fds->priv = &fh->fd;

      if (!circbuf_is_empty(&fh->buffer))
        {
          eventset |= POLLIN | POLLRDNORM;
        }

      poll_notify(&fh->fd, 1, eventset);
    }
  else if (fds->priv != NULL)
    {
      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      if (!slot)
        {
          ret = -EIO;
          goto errout;
        }

      *slot = NULL;
      fds->priv = NULL;
    }

errout:
  leave_critical_section(flags);
  return ret;
}

static int lirc_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct lirc_fh_s *fh = filep->f_priv;
  FAR struct lirc_lowerhalf_s *lower = fh->lower;
  FAR struct lirc_upperhalf_s *upper = lower->priv;
  FAR unsigned int *val = (unsigned int *)(uintptr_t)arg;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case LIRC_GET_FEATURES:
        switch (lower->ops->driver_type)
          {
            case LIRC_DRIVER_SCANCODE:
              *val = LIRC_CAN_REC_SCANCODE;
              break;
            case LIRC_DRIVER_IR_RAW:
              *val = LIRC_CAN_REC_MODE2;
              break;
            default:
              *val = 0;
              break;
          }

        if (lower->rx_resolution)
          {
            *val |= LIRC_CAN_GET_REC_RESOLUTION;
          }

        if (lower->ops->tx_ir)
          {
            *val |= LIRC_CAN_SEND_PULSE;
          }

        if (lower->ops->tx_scancode)
          {
            *val |= LIRC_CAN_SEND_SCANCODE;
          }

        if (lower->ops->s_tx_mask)
          {
            *val |= LIRC_CAN_SET_TRANSMITTER_MASK;
          }

        if (lower->ops->s_tx_carrier)
          {
            *val |= LIRC_CAN_SET_SEND_CARRIER;
          }

        if (lower->ops->s_tx_duty_cycle)
          {
            *val |= LIRC_CAN_SET_SEND_DUTY_CYCLE;
          }

        if (lower->ops->s_rx_carrier_range)
          {
            *val |= LIRC_CAN_SET_REC_CARRIER |
                    LIRC_CAN_SET_REC_CARRIER_RANGE;
          }

        if (lower->ops->s_learning_mode)
          {
            *val |= LIRC_CAN_USE_WIDEBAND_RECEIVER;
          }

        if (lower->ops->s_carrier_report)
          {
            *val |= LIRC_CAN_MEASURE_CARRIER;
          }

        if (lower->max_timeout)
          {
            *val |= LIRC_CAN_SET_REC_TIMEOUT;
          }
        break;

      /* mode support */

      case LIRC_GET_REC_MODE:
        if (lower->ops->driver_type == LIRC_DRIVER_IR_RAW_TX)
          {
            ret = -ENOTTY;
          }
        else
          {
            *val = fh->rec_mode;
          }
        break;

      case LIRC_SET_REC_MODE:
        switch (lower->ops->driver_type)
          {
            case LIRC_DRIVER_IR_RAW_TX:
              ret = -ENOTTY;
              break;

            case LIRC_DRIVER_SCANCODE:
              if (arg != LIRC_MODE_SCANCODE)
                {
                  ret = -EINVAL;
                }
              break;

            case LIRC_DRIVER_IR_RAW:
              if (arg != LIRC_MODE_MODE2)
                {
                  ret = -EINVAL;
                }
              break;
          }

        if (ret >= 0)
          {
            fh->rec_mode = arg;
          }
        break;

      case LIRC_GET_SEND_MODE:
        if (!lower->ops->tx_ir && !lower->ops->tx_scancode)
          {
            ret = -ENOTTY;
          }
        else
          {
            *val = fh->send_mode;
          }
        break;

      case LIRC_SET_SEND_MODE:
        if ((arg == LIRC_MODE_PULSE && lower->ops->tx_ir) ||
            (arg == LIRC_MODE_SCANCODE && lower->ops->tx_scancode))
          {
            fh->send_mode = arg;
          }
        else
          {
            ret = -EINVAL;
          }
        break;

      /* TX settings */

      case LIRC_SET_TRANSMITTER_MASK:
        if (!lower->ops->s_tx_mask)
          {
            ret = -ENOTTY;
          }
        else
          {
            ret = lower->ops->s_tx_mask(lower, arg);
          }
        break;

      case LIRC_SET_SEND_CARRIER:
        if (!lower->ops->s_tx_carrier)
          {
            ret = -ENOTTY;
          }
        else
          {
            ret = lower->ops->s_tx_carrier(lower, arg);
          }
        break;

      case LIRC_SET_SEND_DUTY_CYCLE:
        if (!lower->ops->s_tx_duty_cycle)
          {
            ret = -ENOTTY;
          }
        else if (arg <= 0 || arg >= 100)
          {
            ret = -EINVAL;
          }
        else
          {
            ret = lower->ops->s_tx_duty_cycle(lower, arg);
          }
        break;

      /* RX settings */

      case LIRC_SET_REC_CARRIER:
        if (!lower->ops->s_rx_carrier_range)
          {
            ret = -ENOTTY;
          }
        else if (arg <= 0)
          {
            ret = -EINVAL;
          }
        else
          {
            ret = lower->ops->s_rx_carrier_range(lower,
                                                 fh->carrier_low, arg);
          }
        break;

      case LIRC_SET_REC_CARRIER_RANGE:
        if (!lower->ops->s_rx_carrier_range)
          {
            ret = -ENOTTY;
          }
        else if (arg <= 0)
          {
            ret = -EINVAL;
          }
        else
          {
            fh->carrier_low = arg;
          }
        break;

      case LIRC_GET_REC_RESOLUTION:
        if (!lower->rx_resolution)
          {
            ret = -ENOTTY;
          }
        else
          {
            *val = lower->rx_resolution;
          }
        break;

      case LIRC_SET_WIDEBAND_RECEIVER:
        if (!lower->ops->s_learning_mode)
          {
            ret = -ENOTTY;
          }
        else
          {
            ret = lower->ops->s_learning_mode(lower, !!arg);
          }
        break;

      case LIRC_SET_MEASURE_CARRIER_MODE:
        if (!lower->ops->s_carrier_report)
          {
            ret = -ENOTTY;
          }
        else
          {
            ret = lower->ops->s_carrier_report(lower, !!arg);
          }
        break;

      /* Generic timeout support */

      case LIRC_GET_MIN_TIMEOUT:
        if (!lower->min_timeout)
          {
            ret = -ENOTTY;
          }
        else
          {
            *val = lower->min_timeout;
          }
        break;

      case LIRC_GET_MAX_TIMEOUT:
        if (!lower->max_timeout)
          {
            ret = -ENOTTY;
          }
        else
          {
            *val = lower->max_timeout;
          }
        break;

      case LIRC_SET_REC_TIMEOUT:
        if (!lower->max_timeout)
          {
            ret = -ENOTTY;
          }
        else
          {
            if (arg < lower->min_timeout || arg > lower->max_timeout)
              {
                ret = -EINVAL;
              }
            else if (lower->ops->s_timeout)
              {
                ret = lower->ops->s_timeout(lower, arg);
              }
            else
              {
                lower->timeout = arg;
              }
          }
        break;

      case LIRC_GET_REC_TIMEOUT:
        if (!lower->timeout)
          {
            ret = -ENOTTY;
          }
        else
          {
            *val = lower->timeout;
          }
        break;

      case LIRC_SET_REC_TIMEOUT_REPORTS:
        if (lower->ops->driver_type != LIRC_DRIVER_IR_RAW)
          {
            ret = -ENOTTY;
          }
        else
          {
            fh->send_timeout_reports = !!arg;
          }
        break;

      default:
        ret = -ENOTTY;
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

static ssize_t lirc_write_pulse(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen)
{
  FAR struct lirc_fh_s *fh = filep->f_priv;
  FAR struct lirc_lowerhalf_s *lower = fh->lower;
  size_t count;

  if (buflen < sizeof(unsigned int) || buflen % sizeof(unsigned int))
    {
      return -EINVAL;
    }

  count = buflen / sizeof(unsigned int);
  if (count % 2 == 0)
    {
      return -EINVAL;
    }

  /* tx_ir need sleep some time to wait for thr actual IR signal
   * to be transmitted before returning
   */

  return lower->ops->tx_ir(lower, (FAR unsigned int *)buffer, count);
}

static ssize_t lirc_write_scancode(FAR struct file *filep,
                                   FAR const char *buffer, size_t buflen)
{
  FAR struct lirc_fh_s *fh = filep->f_priv;
  FAR struct lirc_lowerhalf_s *lower = fh->lower;

  if (buflen != sizeof(struct lirc_scancode))
    {
      return -EINVAL;
    }

  /* tx_scancode need sleep some time to wait for thr actual IR signal
   * to be transmitted before returning
   */

  return lower->ops->tx_scancode(lower,
                                (FAR struct lirc_scancode *)buffer);
}

static ssize_t lirc_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lirc_upperhalf_s *upper = inode->i_private;
  FAR struct lirc_fh_s *fh = filep->f_priv;
  ssize_t ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (fh->send_mode == LIRC_MODE_SCANCODE)
    {
      ret = lirc_write_scancode(filep, buffer, buflen);
    }
  else
    {
      ret = lirc_write_pulse(filep, buffer, buflen);
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

static ssize_t lirc_read_scancode(FAR struct file *filep, FAR char *buffer,
                                  size_t length)
{
  FAR struct lirc_fh_s *fh = filep->f_priv;
  irqstate_t flags;
  ssize_t ret;

  if (length < sizeof(struct lirc_scancode) ||
      length % sizeof(struct lirc_scancode))
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  do
    {
      if (circbuf_is_empty(&fh->buffer))
        {
          if (filep->f_oflags & O_NONBLOCK)
            {
              ret = -EAGAIN;
              goto err;
            }

          ret = nxsem_wait_uninterruptible(&fh->waitsem);
          if (ret < 0)
            {
              goto err;
            }
        }

      ret = circbuf_read(&fh->buffer, buffer, length);
    }
  while (ret <= 0);

err:
  leave_critical_section(flags);
  return ret;
}

static ssize_t lirc_read_mode2(FAR struct file *filep, FAR char *buffer,
                               size_t length)
{
  FAR struct lirc_fh_s *fh = filep->f_priv;
  irqstate_t flags;
  ssize_t ret = 0;

  if (length < sizeof(unsigned int) || length % sizeof(unsigned int))
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  do
    {
      if (circbuf_is_empty(&fh->buffer))
        {
          if (filep->f_oflags & O_NONBLOCK)
            {
              ret = -EAGAIN;
              goto err;
            }

          ret = nxsem_wait_uninterruptible(&fh->waitsem);
          if (ret < 0)
            {
              goto err;
            }
        }

      ret = circbuf_read(&fh->buffer, buffer, length);
    }
  while (ret <= 0);

err:
  leave_critical_section(flags);
  return ret;
}

static ssize_t lirc_read(FAR struct file *filep, FAR char *buffer,
                         size_t len)
{
  FAR struct lirc_fh_s *fh = filep->f_priv;

  if (fh->rec_mode == LIRC_MODE_MODE2)
    {
      return lirc_read_mode2(filep, buffer, len);
    }
  else /* LIRC_MODE_SCANCODE */
    {
      return lirc_read_scancode(filep, buffer, len);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lirc_register
 *
 * Description:
 *   This function binds an instance of a "lower half" lirc driver with the
 *   "upper half" RC device and registers that device so that can be used
 *   by application code.
 *
 *   We will register the chararter device. ex: /dev/lirc%d(0, 1, ...)
 *
 * Input Parameters:
 *   lower - A pointer to an instance of lower half lirc driver.
 *   devno - The user specifies device number, from 0. If the
 *           devno alerady exists, -EEXIST will be returned.
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int lirc_register(FAR struct lirc_lowerhalf_s *lower, int devno)
{
  FAR struct lirc_upperhalf_s *upper;
  char path[DEVNAME_MAX];
  int ret;

  DEBUGASSERT(lower != NULL);

  /* Allocate and init the upper-half data structure */

  upper = kmm_zalloc(sizeof(struct lirc_upperhalf_s));
  if (!upper)
    {
      snerr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  upper->lower = lower;
  list_initialize(&upper->fh);
  nxmutex_init(&upper->lock);
  lower->priv = upper;

  /* Register remote control character device */

  snprintf(path, DEVNAME_MAX, DEVNAME_FMT, devno);
  ret = register_driver(path, &g_lirc_fops, 0666, upper);
  if (ret < 0)
    {
      goto drv_err;
    }

  return ret;

drv_err:
  nxmutex_destroy(&upper->lock);
  kmm_free(upper);
  return ret;
}

/****************************************************************************
 * Name: lirc_unregister
 *
 * Description:
 *   This function unregister character node and release all resource about
 *   upper half driver.
 *
 * Input Parameters:
 *   lower - A pointer to an instance of lower half lirc driver.
 *   devno - The user specifies device number, from 0.
 ****************************************************************************/

void lirc_unregister(FAR struct lirc_lowerhalf_s *lower, int devno)
{
  FAR struct lirc_upperhalf_s *upper = lower->priv;
  char path[DEVNAME_MAX];

  nxmutex_destroy(&upper->lock);
  snprintf(path, DEVNAME_MAX, DEVNAME_FMT, devno);
  rcinfo("UnRegistering %s\n", path);
  unregister_driver(path);
  kmm_free(upper);
}

/****************************************************************************
 * Name: lirc_raw_event
 *
 * Description:
 *   Lirc lowerhalf driver sends IR data to lirc upperhalf buffer, to
 *   notify userspace to read IR data.
 *
 *   The type of data is struct lirc_raw_event_s.
 *
 * Input Parameters:
 *   lower  - A pointer to an instance of lower half lirc driver.
 *   ev     - The data of receiving from IR device
 ****************************************************************************/

void lirc_raw_event(FAR struct lirc_lowerhalf_s *lower,
                    struct lirc_raw_event_s ev)
{
  FAR struct lirc_upperhalf_s *upper = lower->priv;
  FAR struct list_node *node;
  FAR struct list_node *tmp;
  FAR struct lirc_fh_s *fh;
  unsigned int sample;
  irqstate_t flags;
  int semcount;
  int gap;

  /* Packet start */

  if (ev.reset)
    {
      /* Userspace expects a long space event before the start of
       * the signal to use as a sync.  This may be done with repeat
       * packets and normal samples.  But if a reset has been sent
       * then we assume that a long time has passed, so we send a
       * space with the maximum time value.
       */

      sample = LIRC_SPACE(LIRC_VALUE_MASK);
      rcinfo("delivering reset sync space to lirc_dev\n");
    }
  else if (ev.carrier_report)
    {
      /* Carrier reports */

      sample = LIRC_FREQUENCY(ev.carrier);
      rcinfo("carrier report (freq: %d)\n", sample);
    }
  else if (ev.timeout)
    {
      /* Packet end */

      if (upper->gap)
        {
          return;
        }

      upper->gap = true;
      upper->gap_start = lirc_get_timestamp() / 1000;
      upper->gap_duration = ev.duration;

      sample = LIRC_TIMEOUT(ev.duration);
      rcinfo("timeout report (duration: %d)\n", sample);
    }
  else
    {
      /* Normal sample */

      if (upper->gap)
        {
          upper->gap_duration += (lirc_get_timestamp() / 1000) -
                                 upper->gap_start;

          /* Cap by LIRC_VALUE_MASK */

          upper->gap_duration = MIN(upper->gap_duration, LIRC_VALUE_MASK);
          gap = LIRC_SPACE(upper->gap_duration);

          flags = enter_critical_section();
          list_for_every_safe(&upper->fh, node, tmp)
            {
              fh = (FAR struct lirc_fh_s *)node;
              if (circbuf_write(&fh->buffer, &gap, sizeof(int)) > 0)
                {
                  poll_notify(&fh->fd, 1, POLLIN | POLLRDNORM);
                  nxsem_get_value(&fh->waitsem, &semcount);
                  if (semcount < 1)
                    {
                      nxsem_post(&fh->waitsem);
                    }
                }

              upper->gap = false;
            }

          leave_critical_section(flags);
        }

      sample = ev.pulse ? LIRC_PULSE(ev.duration) : LIRC_SPACE(ev.duration);
      rcinfo("delivering %" PRIu32 "us %u to lirc\n",
             ev.duration, ev.pulse ? 1 : 0);
    }

  flags = enter_critical_section();
  list_for_every_safe(&upper->fh, node, tmp)
    {
      fh = (FAR struct lirc_fh_s *)node;
      if (LIRC_IS_TIMEOUT(sample) && !fh->send_timeout_reports)
        {
          continue;
        }

      if (circbuf_write(&fh->buffer, &sample, sizeof(unsigned int)) > 0)
        {
          poll_notify(&fh->fd, 1, POLLIN | POLLRDNORM);
          nxsem_get_value(&fh->waitsem, &semcount);
          if (semcount < 1)
            {
              nxsem_post(&fh->waitsem);
            }
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lirc_scancode_event
 *
 * Description:
 *   Lirc lowerhalf driver sends IR data to lirc upperhalf buffer, to
 *   notify userspace to read IR data.
 *
 *   The type of data is struct lirc_scancode.
 *
 * Input Parameters:
 *   lower  - A pointer to an instance of lower half lirc driver.
 *   lsc    - The data of receiving from IR device
 ****************************************************************************/

void lirc_scancode_event(FAR struct lirc_lowerhalf_s *lower,
                         FAR struct lirc_scancode *lsc)
{
  FAR struct lirc_upperhalf_s *upper = lower->priv;
  FAR struct list_node *node;
  FAR struct list_node *tmp;
  FAR struct lirc_fh_s *fh;
  irqstate_t flags;
  int semcount;

  lsc->timestamp = lirc_get_timestamp();

  flags = enter_critical_section();
  list_for_every_safe(&upper->fh, node, tmp)
    {
      fh = (FAR struct lirc_fh_s *)node;
      if (circbuf_write(&fh->buffer, lsc, sizeof(*lsc)) > 0)
        {
          poll_notify(&fh->fd, 1, POLLIN | POLLRDNORM);
          nxsem_get_value(&fh->waitsem, &semcount);
          if (semcount < 1)
            {
              nxsem_post(&fh->waitsem);
            }
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lirc_sample_event
 *
 * Description:
 *   Lirc lowerhalf driver sends raw IR data to lirc upperhalf buffer, to
 *   notify userspace to read IR data.
 *
 *   The type of data is a sequence of pulse and space codes, as a seriers
 *   of unsigned int values.
 *
 *   The upper 8 bits determine the packet type, and the lower 24 bits the
 *   payload.
 *
 * Input Parameters:
 *   lower  - A pointer to an instance of lower half lirc driver.
 *   sample - The data of receiving from IR device
 ****************************************************************************/

void lirc_sample_event(FAR struct lirc_lowerhalf_s *lower,
                       unsigned int sample)
{
  FAR struct lirc_upperhalf_s *upper = lower->priv;
  FAR struct list_node *node;
  FAR struct list_node *tmp;
  FAR struct lirc_fh_s *fh;
  irqstate_t flags;
  int semcount;

  flags = enter_critical_section();
  list_for_every_safe(&upper->fh, node, tmp)
    {
      fh = (FAR struct lirc_fh_s *)node;
      if (circbuf_write(&fh->buffer, &sample, sizeof(unsigned int)) > 0)
        {
          poll_notify(&fh->fd, 1, POLLIN | POLLRDNORM);
          nxsem_get_value(&fh->waitsem, &semcount);
          if (semcount < 1)
            {
              nxsem_post(&fh->waitsem);
            }
        }
    }

  leave_critical_section(flags);
}
