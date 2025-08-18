/****************************************************************************
 * audio/audio.c
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

#include <sys/param.h>
#include <sys/types.h>
#include <sys/stat.h>
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
#include <nuttx/mqueue.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/audio/audio.h>
#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>

#include <arch/irq.h>

#ifdef CONFIG_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing Audio */

#ifndef AUDIO_MAX_DEVICE_PATH
#  define AUDIO_MAX_DEVICE_PATH 32
#endif

#ifndef CONFIG_AUDIO_BUFFER_DEQUEUE_PRIO
#  define CONFIG_AUDIO_BUFFER_DEQUEUE_PRIO  1
#endif

#define AUDIO_ENQUEUE_THRESHOLD 2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct audio_openpriv_s
{
  unsigned long               head;
  int                         state;
  FAR struct pollfd           *fd;
  FAR struct audio_openpriv_s *flink;
  struct file                 *usermq; /* User mode app's message queue */
};

/* This structure describes the state of the upper half driver */

struct audio_upperhalf_s
{
  struct audio_info_s          info;     /* Record the last playing audio format */
  mutex_t                      lock;     /* Supports mutual exclusion */
  uint8_t                      nbuffers; /* Max ap buffers number */
  spinlock_t                   spinlock; /* Supports spin lock */
  uint8_t                      periods;  /* Ap buffers number */
  FAR struct ap_buffer_s       **apbs;   /* Ap buffers list */
  FAR struct audio_lowerhalf_s *dev;     /* lower-half state */
  FAR struct audio_openpriv_s  *head;    /* Appl private info list */
  struct audio_status_s        *status;  /* lowerhalf driver status */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      audio_open(FAR struct file *filep);
static int      audio_close(FAR struct file *filep);
static ssize_t  audio_read(FAR struct file *filep,
                           FAR char *buffer,
                           size_t buflen);
static ssize_t  audio_write(FAR struct file *filep,
                            FAR const char *buffer,
                            size_t buflen);
static int      audio_ioctl(FAR struct file *filep,
                            int cmd,
                            unsigned long arg);
static int      audio_mmap(FAR struct file *filep,
                           FAR struct mm_map_entry_s *map);
static int      audio_poll(FAR struct file *filep,
                           FAR struct pollfd *fds, bool setup);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static void     audio_callback(FAR void *priv,
                               uint16_t reason,
                               FAR struct ap_buffer_s *apb,
                               uint16_t status,
                               FAR void *session);
#else
static void     audio_callback(FAR void *priv,
                               uint16_t reason,
                               FAR struct ap_buffer_s *apb,
                               uint16_t status);
#endif /* CONFIG_AUDIO_MULTI_SESSION */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_audioops =
{
  audio_open,  /* open */
  audio_close, /* close */
  audio_read,  /* read */
  audio_write, /* write */
  NULL,        /* seek */
  audio_ioctl, /* ioctl */
  audio_mmap,  /* mmap */
  NULL,        /* truncate */
  audio_poll,  /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: audio_open
 *
 * Description:
 *   This function is called whenever the Audio device is opened.
 *
 ****************************************************************************/

static int audio_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct audio_openpriv_s *priv;
  irqstate_t flags;
  int ret;

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* First open, alloc status memory */

  if (upper->head == NULL)
    {
      upper->status = kumm_zalloc(sizeof(struct audio_status_s));
      if (!upper->status)
        {
          auderr("ERROR: Allocation status failed\n");
          ret = -ENOMEM;
          goto errout;
        }
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  flags = spin_lock_irqsave(&upper->spinlock);
  priv->flink = upper->head;
  filep->f_priv = priv;
  upper->head = priv;
  spin_unlock_irqrestore(&upper->spinlock, flags);
  ret = OK;

errout:
  nxmutex_unlock(&upper->lock);
  return ret;
}

/****************************************************************************
 * Name: audio_close
 *
 * Description:
 *   This function is called when the Audio device is closed.
 *
 ****************************************************************************/

static int audio_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct audio_openpriv_s *priv = filep->f_priv;
  FAR struct audio_openpriv_s *curr;
  FAR struct audio_openpriv_s *prev;
  irqstate_t flags;
  int ret;

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  flags = spin_lock_irqsave(&upper->spinlock);

  /* find and drop priv from list */

  for (prev = NULL, curr = upper->head; curr != NULL && curr != priv;
       prev = curr, curr = curr->flink);

  if (prev != NULL)
    {
      prev->flink = priv->flink;
    }
  else
    {
      upper->head = priv->flink;
    }

  spin_unlock_irqrestore(&upper->spinlock, flags);

  kmm_free(priv);

  /*  If the reference head decrement to NULL,
   *  then uninitialize the driver.
   */

  if (upper->head == NULL)
    {
      FAR struct audio_lowerhalf_s *lower = upper->dev;

      /* Disable the Audio device */

      DEBUGASSERT(lower->ops->shutdown != NULL);
      audinfo("calling shutdown\n");

      lower->ops->shutdown(lower);
      kumm_free(upper->status);
      upper->status = NULL;
    }

  ret = OK;
  nxmutex_unlock(&upper->lock);

  return ret;
}

/****************************************************************************
 * Name: audio_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t audio_read(FAR struct file *filep,
                          FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct audio_lowerhalf_s *lower = upper->dev;

  /* TODO: Should we check permissions here? */

  /* Audio read operations get passed directly to the lower-level */

  if (lower->ops->read != NULL)
    {
      return lower->ops->read(lower, buffer, buflen);
    }

  return 0;
}

/****************************************************************************
 * Name: audio_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t audio_write(FAR struct file *filep,
                           FAR const char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct audio_lowerhalf_s *lower = upper->dev;

  /* TODO: Should we check permissions here? */

  /* Audio write operations get passed directly to the lower-level */

  if (lower->ops->write != NULL)
    {
      return lower->ops->write(lower, buffer, buflen);
    }

  return 0;
}

/****************************************************************************
 * Name: audio_setstate
 *
 * Description:
 *   Update lower driver state
 *
 ****************************************************************************/

static inline void audio_setstate(FAR struct audio_upperhalf_s *upper,
                                  int state)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&upper->spinlock);
  upper->status->state = state;
  spin_unlock_irqrestore(&upper->spinlock, flags);
}

/****************************************************************************
 * Name: audio_getnstate
 *
 * Description:
 *   Get the applications next top state
 *
 ****************************************************************************/

static inline int audio_getnstate(FAR struct audio_upperhalf_s *upper,
                                  FAR struct audio_openpriv_s *cur)
{
  FAR struct audio_openpriv_s *priv;
  int nstate = AUDIO_STATE_OPEN;
  irqstate_t flags;

  flags = spin_lock_irqsave(&upper->spinlock);
  for (priv = upper->head; priv != NULL; priv = priv->flink)
    {
      if (priv != cur && priv->state > nstate)
        {
          nstate = priv->state;
        }
    }

  spin_unlock_irqrestore(&upper->spinlock, flags);

  return nstate;
}

/****************************************************************************
 * Name: audio_try_enqueue
 *
 * Description:
 *   Try enqueue audio buffer
 *
 ****************************************************************************/

static int audio_try_enqueue(FAR struct audio_upperhalf_s *upper)
{
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  FAR struct audio_openpriv_s *priv;
  int ret = OK;

  for (; ; )
    {
      uint32_t count = upper->status->head - upper->status->tail;
      bool ready = false;
      bool wait = false;

      for (priv = upper->head; priv != NULL; priv = priv->flink)
        {
          if (priv->state == AUDIO_STATE_OPEN ||
              priv->state == AUDIO_STATE_PAUSED)
            {
              continue;
            }
          else if (priv->head > upper->status->head)
            {
              ready = true;
            }
          else if (priv->head < upper->status->head ||
                   priv->head <= upper->status->tail)
            {
              priv->state = AUDIO_STATE_XRUN;
            }
          else
            {
              wait = true;
            }
        }

      if (!ready || (wait && count > AUDIO_ENQUEUE_THRESHOLD))
        {
          return OK;
        }

      ret = lower->ops->enqueuebuffer(
          lower, upper->apbs[upper->status->head % upper->periods]);
      if (ret < 0)
        {
          return ret;
        }

      upper->status->head++;
    }

  return ret;
}

/****************************************************************************
 * Name: audio_configure
 *
 * Description:
 *   Handle the AUDIOIOC_CONFIGURE ioctl command
 *
 ****************************************************************************/

static int audio_configure(FAR struct file *filep,
                           FAR const struct audio_caps_desc_s *cap_desc)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  FAR const struct audio_caps_s *caps = &cap_desc->caps;
  FAR struct audio_openpriv_s *priv = filep->f_priv;
  int ret = OK;

  DEBUGASSERT(lower->ops->configure != NULL);

  if (upper->status->state == AUDIO_STATE_OPEN ||
      (caps->ac_type != AUDIO_TYPE_INPUT &&
       caps->ac_type != AUDIO_TYPE_OUTPUT))
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      ret = lower->ops->configure(lower, cap_desc->session, caps);
#else
      ret = lower->ops->configure(lower, caps);
#endif
      if (ret < 0 || (caps->ac_type != AUDIO_TYPE_INPUT &&
                      caps->ac_type != AUDIO_TYPE_OUTPUT))
        {
          return ret;
        }

      /* INPUT/OUTPUT configure success here */

      audio_setstate(upper, AUDIO_STATE_PREPARED);
      upper->info.type = caps->ac_type;
      upper->info.format = caps->ac_subtype;
      upper->info.channels = caps->ac_channels;
      upper->info.subformat = caps->ac_format.b[0];
      upper->info.samplerate =
          caps->ac_controls.hw[0] | (caps->ac_controls.b[3] << 16);
      memcpy(&upper->info.codec, &caps->ac_codec, sizeof(caps->ac_codec));
    }

  priv->state = AUDIO_STATE_PREPARED;
  return ret;
}

/****************************************************************************
 * Name: audio_pause
 *
 * Description:
 *   Handle the AUDIOIOC_PAUSE ioctl command
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_pause(FAR struct file *filep, FAR void *session)
#else
static int audio_pause(FAR struct file *filep)
#endif
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  FAR struct audio_openpriv_s *priv = filep->f_priv;
  int nstate;
  int ret;

  DEBUGASSERT(upper != NULL && lower->ops->pause != NULL);

  nstate = audio_getnstate(upper, priv);
  if (upper->status->state == AUDIO_STATE_RUNNING &&
      nstate <= AUDIO_STATE_PAUSED)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      ret = lower->ops->pause(lower, session);
#else
      ret = lower->ops->pause(lower);
#endif
      if (ret != OK)
        {
          return ret;
        }

      audio_setstate(upper, AUDIO_STATE_PAUSED);
    }

  priv->state = AUDIO_STATE_PAUSED;
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: audio_resume
 *
 * Description:
 *   Handle the AUDIOIOC_resume ioctl command
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_resume(FAR struct file *filep, FAR void *session)
#else
static int audio_resume(FAR struct file *filep)
#endif
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  FAR struct audio_openpriv_s *priv = filep->f_priv;
  int ret;

  DEBUGASSERT(upper != NULL && lower->ops->resume != NULL);

  if (upper->status->state == AUDIO_STATE_PAUSED ||
      upper->status->state == AUDIO_STATE_XRUN)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      ret = lower->ops->resume(lower, session);
#else
      ret = lower->ops->resume(lower);
#endif
      if (ret != OK)
        {
          return ret;
        }

      audio_setstate(upper, AUDIO_STATE_RUNNING);
    }

  priv->state = AUDIO_STATE_RUNNING;
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: audio_start
 *
 * Description:
 *   Handle the AUDIOIOC_START ioctl command
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_start(FAR struct file *filep, FAR void *session)
#else
static int audio_start(FAR struct file *filep)
#endif
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  FAR struct audio_openpriv_s *priv = filep->f_priv;
  int ret;

  DEBUGASSERT(upper != NULL && lower->ops->start != NULL);

  if (upper->status->state == AUDIO_STATE_OPEN)
    {
      return -EPERM;
    }
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  else if (upper->status->state == AUDIO_STATE_PAUSED)
    {
      return audio_resume(filep);
    }
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

  /* Verify that the Audio is not already running */

  if (upper->status->state == AUDIO_STATE_PREPARED ||
      upper->status->state == AUDIO_STATE_XRUN)
    {
      /* Invoke the bottom half method to start the audio stream */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      ret = lower->ops->start(lower, session);
#else
      ret = lower->ops->start(lower);
#endif
      /* A return value of zero means that the audio stream was running
       * successfully.
       */

      if (ret != OK)
        {
          return ret;
        }

      audio_setstate(upper, AUDIO_STATE_RUNNING);
    }

  priv->state = AUDIO_STATE_RUNNING;
  return OK;
}

/****************************************************************************
 * Name: audio_stop
 *
 * Description:
 *   Handle the AUDIOIOC_STOP ioctl command
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_stop(FAR struct file *filep, FAR void *session)
#else
static int audio_stop(FAR struct file *filep)
#endif
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  FAR struct audio_openpriv_s *priv = filep->f_priv;
  int nstate;
  int ret;

  DEBUGASSERT(upper != NULL && lower->ops->stop != NULL);

  nstate = audio_getnstate(upper, priv);
  if ((upper->status->state == AUDIO_STATE_RUNNING ||
       upper->status->state == AUDIO_STATE_PAUSED) &&
      nstate == AUDIO_STATE_OPEN)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      ret = lower->ops->stop(lower, session);
#else
      ret = lower->ops->stop(lower);
#endif
      if (ret != OK)
        {
          return ret;
        }

      memset(&upper->info, 0, sizeof(upper->info));

      /* Audio_complete may have set state to AUDIO_STATE_OPEN */

      if (upper->status->state != AUDIO_STATE_OPEN)
        {
          audio_setstate(upper, AUDIO_STATE_DRAINING);
        }
    }
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  else if (nstate == AUDIO_STATE_PAUSED)
    {
      ret = audio_pause(filep);
      if (ret != OK)
        {
          return ret;
        }
    }
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

  priv->state = AUDIO_STATE_OPEN;
  return OK;
}

/****************************************************************************
 * Name: audio_freebuffer
 *
 * Description:
 *   Handle the AUDIOIOC_FREEBUFFER ioctl command
 *
 ****************************************************************************/

static int audio_freebuffer(FAR struct audio_upperhalf_s *upper,
                            FAR struct audio_buf_desc_s *bufdesc)
{
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  bool share = false;
  int ret;

  if (bufdesc->u.buffer == NULL)
    {
      bufdesc->u.buffer = upper->apbs[upper->periods - 1];
      share = true;
    }

  if (lower->ops->freebuffer)
    {
      ret = lower->ops->freebuffer(lower, bufdesc);
    }
  else
    {
      /* Perform a simple apb_free operation */

      DEBUGASSERT(bufdesc->u.buffer != NULL);
      apb_free(bufdesc->u.buffer);
      ret = sizeof(struct audio_buf_desc_s);
    }

  if (ret > 0 && share)
    {
      bufdesc->u.buffer = NULL;
      upper->periods--;
      upper->apbs[upper->periods] = NULL;
      if (upper->periods == 0)
        {
          kmm_free(upper->apbs);
          upper->apbs = NULL;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: audio_allocbuffer
 *
 * Description:
 *   Handle the AUDIOIOC_ALLOCBUFFER ioctl command
 *
 ****************************************************************************/

static int audio_allocbuffer(FAR struct audio_upperhalf_s *upper,
                             FAR struct audio_buf_desc_s *bufdesc)
{
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  FAR struct ap_buffer_s *apb;
  bool share = false;
  FAR void *newaddr;
  int ret;

  if (upper->periods >= upper->nbuffers)
    {
      return 0;
    }

  if (bufdesc->u.pbuffer == NULL)
    {
      bufdesc->u.pbuffer = &apb;
      share = true;
    }

  if (lower->ops->allocbuffer != NULL)
    {
      ret = lower->ops->allocbuffer(lower, bufdesc);
    }
  else
    {
      /* Perform a simple kumm_malloc operation assuming 1 session */

      ret = apb_alloc(bufdesc);
    }

  if (ret > 0 && share)
    {
      newaddr = kmm_realloc(upper->apbs,
                            (upper->periods + 1) * sizeof(*upper->apbs));
      if (newaddr == NULL)
        {
          audio_freebuffer(upper, bufdesc);
          return -ENOMEM;
        }

      upper->apbs = (FAR struct ap_buffer_s **)newaddr;
      upper->apbs[upper->periods] = apb;
      upper->periods++;
      bufdesc->u.pbuffer = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: audio_enqueuebuffer
 *
 * Description:
 *   Handle the AUDIOIOC_ENQUEUEBUFFER ioctl command
 *
 ****************************************************************************/

static int audio_enqueuebuffer(FAR struct file *filep,
                               FAR struct audio_buf_desc_s *bufdesc)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  FAR struct audio_openpriv_s *priv = filep->f_priv;
  irqstate_t flags;
  int ret = OK;

  DEBUGASSERT(lower->ops->enqueuebuffer != NULL);

  if (bufdesc->u.buffer)
    {
      ret = lower->ops->enqueuebuffer(lower, bufdesc->u.buffer);
      if (ret != OK)
        {
          return ret;
        }

      flags = spin_lock_irqsave(&upper->spinlock);
      upper->status->head++;
      spin_unlock_irqrestore(&upper->spinlock, flags);
    }
  else
    {
      flags = spin_lock_irqsave_nopreempt(&upper->spinlock);
      upper->apbs[priv->head % upper->periods]->nbytes =
          MAX(upper->apbs[priv->head % upper->periods]->nbytes,
              bufdesc->numbytes);
      priv->head++;
      ret = audio_try_enqueue(upper);
      spin_unlock_irqrestore_nopreempt(&upper->spinlock, flags);
    }

  return ret;
}

/****************************************************************************
 * Name: audio_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the Audio work is done.
 *
 ****************************************************************************/

static int audio_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  FAR struct audio_openpriv_s *priv = filep->f_priv;
  int ret;

  audinfo("cmd: %d arg: %ld\n", cmd, arg);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
      /* AUDIOIOC_GETCAPS - Get the audio device capabilities.
       *
       *   ioctl argument:  A pointer to the audio_caps_s structure.
       */

      case AUDIOIOC_GETCAPS:
        {
          FAR struct audio_caps_s *caps =
                     (FAR struct audio_caps_s *)((uintptr_t)arg);
          DEBUGASSERT(lower->ops->getcaps != NULL);

          audinfo("AUDIOIOC_GETCAPS: Device=%d\n", caps->ac_type);

          /* Call the lower-half driver capabilities handler */

          ret = lower->ops->getcaps(lower, caps->ac_type, caps);
        }
        break;

      case AUDIOIOC_CONFIGURE:
        {
          FAR const struct audio_caps_desc_s *caps =
            (FAR const struct audio_caps_desc_s *)((uintptr_t)arg);

          audinfo("AUDIOIOC_INITIALIZE: Device=%d\n", caps->caps.ac_type);

          /* Call the lower-half driver configure handler */

          ret = audio_configure(filep, caps);
        }
        break;

      case AUDIOIOC_SHUTDOWN:
        {
          DEBUGASSERT(lower->ops->shutdown != NULL);

          audinfo("AUDIOIOC_SHUTDOWN\n");

          /* Call the lower-half driver initialize handler */

          ret = lower->ops->shutdown(lower);
        }
        break;

      /* AUDIOIOC_START - Start the audio stream.
       *   The AUDIOIOC_SETCHARACTERISTICS
       *   command must have previously been sent.
       *
       *   ioctl argument:  Audio session
       */

      case AUDIOIOC_START:
        {
          audinfo("AUDIOIOC_START\n");

          /* Start the audio stream */

#ifdef CONFIG_AUDIO_MULTI_SESSION
          ret = audio_start(filep, (FAR void *)arg);
#else
          ret = audio_start(filep);
#endif
        }
        break;

      /* AUDIOIOC_STOP - Stop the audio stream.
       *
       *   ioctl argument:  Audio session
       */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
      case AUDIOIOC_STOP:
        {
          audinfo("AUDIOIOC_STOP\n");

#ifdef CONFIG_AUDIO_MULTI_SESSION
          ret = audio_stop(filep, (FAR void *)arg);
#else
          ret = audio_stop(filep);
#endif
        }
        break;
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

      /* AUDIOIOC_PAUSE - Pause the audio stream.
       *
       *   ioctl argument:  Audio session
       */

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME

      case AUDIOIOC_PAUSE:
        {
          audinfo("AUDIOIOC_PAUSE\n");

#ifdef CONFIG_AUDIO_MULTI_SESSION
          ret = audio_pause(filep, (FAR void *)arg);
#else
          ret = audio_pause(filep);
#endif
        }
        break;

      /* AUDIOIOC_RESUME - Resume the audio stream.
       *
       *   ioctl argument:  Audio session
       */

      case AUDIOIOC_RESUME:
        {
          audinfo("AUDIOIOC_RESUME\n");

#ifdef CONFIG_AUDIO_MULTI_SESSION
          ret = audio_resume(filep, (FAR void *)arg);
#else
          ret = audio_resume(filep);
#endif
        }
        break;

#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

      /* AUDIOIOC_ALLOCBUFFER - Allocate an audio buffer
       *
       *   ioctl argument:  pointer to an audio_buf_desc_s structure
       */

      case AUDIOIOC_ALLOCBUFFER:
        {
          audinfo("AUDIOIOC_ALLOCBUFFER\n");

          ret = audio_allocbuffer(upper, (FAR struct audio_buf_desc_s *)arg);
        }
        break;

      /* AUDIOIOC_FREEBUFFER - Free an audio buffer
       *
       *   ioctl argument:  pointer to an audio_buf_desc_s structure
       */

      case AUDIOIOC_FREEBUFFER:
        {
          audinfo("AUDIOIOC_FREEBUFFER\n");

          ret = audio_freebuffer(upper, (FAR struct audio_buf_desc_s *)arg);
        }
        break;

      /* AUDIOIOC_ENQUEUEBUFFER - Enqueue an audio buffer
       *
       *   ioctl argument:  pointer to an audio_buf_desc_s structure
       */

      case AUDIOIOC_ENQUEUEBUFFER:
        {
          audinfo("AUDIOIOC_ENQUEUEBUFFER\n");

          ret =
              audio_enqueuebuffer(filep, (FAR struct audio_buf_desc_s *)arg);
        }
        break;

      /* AUDIOIOC_REGISTERMQ - Register a client Message Queue
       *
       * TODO:  This needs to have multi session support.
       */

      case AUDIOIOC_REGISTERMQ:
        {
          audinfo("AUDIOIOC_REGISTERMQ\n");

          ret = file_get((mqd_t)arg, &priv->usermq);
        }
        break;

      /* AUDIOIOC_UNREGISTERMQ - Register a client Message Queue
       *
       * TODO:  This needs to have multi session support.
       */

      case AUDIOIOC_UNREGISTERMQ:
        {
          audinfo("AUDIOIOC_UNREGISTERMQ\n");

          if (priv->usermq != NULL)
            {
              file_put(priv->usermq);
              priv->usermq = NULL;
            }

          ret = OK;
        }
        break;

      /* AUDIOIOC_RESERVE - Reserve a session with the driver
       *
       *   ioctl argument - pointer to receive the session context
       */

      case AUDIOIOC_RESERVE:
        {
          audinfo("AUDIOIOC_RESERVE\n");
          DEBUGASSERT(lower->ops->reserve != NULL);

          if (upper->head->flink)
            {
              break;
            }

          /* Call lower-half to perform the reservation */

#ifdef CONFIG_AUDIO_MULTI_SESSION
          ret = lower->ops->reserve(lower, (FAR void **) arg);
#else
          ret = lower->ops->reserve(lower);
#endif
        }
        break;

      /* AUDIOIOC_RESERVE - Reserve a session with the driver
       *
       *   ioctl argument - pointer to receive the session context
       */

      case AUDIOIOC_RELEASE:
        {
          audinfo("AUDIOIOC_RELEASE\n");
          DEBUGASSERT(lower->ops->release != NULL);

          if (upper->head->flink)
            {
              break;
            }

          /* Call lower-half to perform the release */

#ifdef CONFIG_AUDIO_MULTI_SESSION
          ret = lower->ops->release(lower, (FAR void *) arg);
#else
          ret = lower->ops->release(lower);
#endif
        }
        break;

      /* AUDIOIOC_GETAUDIOINFO - Get the last playing audio format
       *
       *   ioctl argument - pointer to receive the audio info
       */

      case AUDIOIOC_GETAUDIOINFO:
        {
          if (lower->ops->ioctl == NULL ||
              lower->ops->ioctl(lower, AUDIOIOC_GETAUDIOINFO, arg) < 0)
            {
              memcpy((void *)arg, &upper->info, sizeof(struct audio_info_s));
            }

          ret = OK;
        }
        break;

      /* AUDIOIOC_GETSTATUS - Get lower driver state
       *
       *   ioctl argument - pointer to receive the state
       */

      case AUDIOIOC_GETSTATUS:
        {
          memcpy((void *)arg, upper->status, sizeof(struct audio_status_s));
          ret = OK;
        }
        break;

      /* AUDIOIOC_RESETSTATUS - Reset appl head
       *
       *   ioctl argument - pointer to receive the state
       */

      case AUDIOIOC_RESETSTATUS:
        {
          struct audio_buf_desc_s buf_desc;
          int target;

          buf_desc.numbytes = upper->apbs[0]->nmaxbytes;
          buf_desc.u.pbuffer = NULL;
          priv->head = upper->status->head;
          target = MAX(upper->status->head,
                       upper->status->tail + upper->periods - 1);

          if (priv->state == AUDIO_STATE_XRUN)
            {
              priv->state = AUDIO_STATE_RUNNING;
            }

          while (priv->head < target)
            {
              audio_enqueuebuffer(filep, &buf_desc);
            }
        }
        break;

      /* AUDIOIOC_SETBUFFERINFO - Set buffer information
       *
       *   ioctl argument - pointer to set the buffer information
       */

      case AUDIOIOC_SETBUFFERINFO:
        {
          audinfo("AUDIOIOC_SETBUFFERINFO\n");

          if (upper->periods == 0)
            {
              ret = lower->ops->ioctl(lower, AUDIOIOC_SETBUFFERINFO, arg);
            }
        }
        break;

      /* AUDIOIOC_GETBUFFERINFO - Get buffer information
       *
       *   ioctl argument - pointer to get the buffer information
       */

      case AUDIOIOC_GETBUFFERINFO:
        {
          audinfo("AUDIOIOC_GETBUFFERINFO\n");

          ret = lower->ops->ioctl(lower, AUDIOIOC_GETBUFFERINFO, arg);
          if (ret >= 0)
            {
              upper->nbuffers =
                  ((FAR struct ap_buffer_info_s *)arg)->nbuffers;
            }
        }
        break;

      /* Any unrecognized IOCTL commands might be
       * platform-specific ioctl commands
       */

      default:
        {
          audinfo("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
          DEBUGASSERT(lower->ops->ioctl != NULL);
          ret = lower->ops->ioctl(lower, cmd, arg);
        }
        break;
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

/****************************************************************************
 * Name: audio_munmap
 *
 * Description:
 *   The standard unmap method.
 *
 ****************************************************************************/

static int audio_munmap(FAR struct task_group_s *group,
                         FAR struct mm_map_entry_s *entry, FAR void *start,
                         size_t length)
{
  return mm_map_remove(get_group_mm(group), entry);
}

/****************************************************************************
 * Name: audio_mmap
 *
 * Description:
 *   The standard mmap method.
 *
 ****************************************************************************/

static int audio_mmap(FAR struct file *filep, FAR struct mm_map_entry_s *map)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  int ret;

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Return the address corresponding to the start of frame buffer. */

  if (upper->apbs != NULL && map->length == upper->apbs[0]->nmaxbytes)
    {
      ret = map->offset / upper->apbs[0]->nmaxbytes;

      DEBUGASSERT(ret < upper->periods);

      map->vaddr = (FAR char *)upper->apbs[ret]->samp;
      map->munmap = audio_munmap;
      ret = mm_map_add(get_current_mm(), map);
    }
  else if (map->length == sizeof(struct audio_status_s))
    {
      map->vaddr = (FAR char *)upper->status;
      map->munmap = audio_munmap;
      ret = mm_map_add(get_current_mm(), map);
    }
  else
    {
      ret = -EINVAL;
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

/****************************************************************************
 * Name: audio_poll
 *
 * Description:
 *   Wait for framebuffer to be writable.
 *
 ****************************************************************************/

static int audio_poll(FAR struct file *filep,
                      FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct audio_openpriv_s *priv = filep->f_priv;
  pollevent_t eventset;
  irqstate_t flags;

  DEBUGASSERT(upper != NULL);

  flags = spin_lock_irqsave_nopreempt(&upper->spinlock);

  if (setup)
    {
      priv->fd = fds;
      fds->priv = &priv->fd;

      if (priv->head - upper->status->tail != upper->periods)
        {
          eventset = POLLIN | POLLOUT;
          if ((long)(priv->head - upper->status->tail) <= 0)
            {
              eventset |= POLLERR;
            }

          poll_notify(&fds, 1, eventset);
        }
    }
  else if (fds->priv != NULL)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;
      *slot = NULL;
      fds->priv = NULL;
    }

  spin_unlock_irqrestore_nopreempt(&upper->spinlock, flags);
  return OK;
}

/****************************************************************************
 * Name: audio_sendmsg
 *
 * Description:
 *   Send message to the user if a message queue is registered
 *
 ****************************************************************************/

static inline void audio_sendmsg(FAR struct audio_upperhalf_s *upper,
                                 FAR struct audio_msg_s *msg)
{
  FAR struct audio_openpriv_s *priv;
  irqstate_t flags;

  flags = spin_lock_irqsave_nopreempt(&upper->spinlock);
  for (priv = upper->head; priv != NULL; priv = priv->flink)
    {
      if (priv->usermq != NULL)
        {
          file_mq_send(priv->usermq, (FAR const char *)msg, sizeof(*msg),
                       CONFIG_AUDIO_BUFFER_DEQUEUE_PRIO);
        }
    }

  spin_unlock_irqrestore_nopreempt(&upper->spinlock, flags);
}

/****************************************************************************
 * Name: audio_dequeuebuffer
 *
 * Description:
 *   Dequeues a previously enqueued Audio Pipeline Buffer.
 *
 *   1. The upper half driver calls the enqueuebuffer method, providing the
 *      lower half driver with the ab_buffer to process.
 *   2. The lower half driver's enqueuebuffer will either processes the
 *      buffer directly, or more likely add it to a queue for processing
 *      by a background thread or worker task.
 *   3. When the lower half driver has completed processing of the enqueued
 *      ab_buffer, it will call this routine to indicate processing of the
 *      buffer is complete.
 *   4. When this routine is called, it will check if any threads are waiting
 *      to enqueue additional buffers and "wake them up" for further
 *      processing.
 *
 * Input Parameters:
 *   handle - This is the handle that was provided to the lower-half
 *     start() method.
 *   apb - A pointer to the previously enqueued ap_buffer_s
 *   status - Status of the dequeue operation
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static inline void audio_dequeuebuffer(FAR struct audio_upperhalf_s *upper,
                    FAR struct ap_buffer_s *apb, uint16_t status,
                    FAR void *session)
#else
static inline void audio_dequeuebuffer(FAR struct audio_upperhalf_s *upper,
                    FAR struct ap_buffer_s *apb, uint16_t status)
#endif
{
  FAR struct audio_openpriv_s *priv;
  struct audio_msg_s    msg;
  pollevent_t eventset;
  irqstate_t flags;

  audinfo("Entry\n");

  if (upper->info.type == AUDIO_TYPE_OUTPUT)
    {
      apb->nbytes = 0;
      memset(apb->samp, 0, apb->nmaxbytes);
    }

  flags = spin_lock_irqsave_nopreempt(&upper->spinlock);
  upper->status->tail++;
  audio_try_enqueue(upper);
  for (priv = upper->head; priv != NULL; priv = priv->flink)
    {
      if (priv->fd > 0)
        {
          eventset = POLLIN | POLLOUT;
          if ((long)(priv->head - upper->status->tail) <= 0)
            {
              eventset |= POLLERR;
            }

          poll_notify(&priv->fd, 1, eventset);
        }
    }

  spin_unlock_irqrestore_nopreempt(&upper->spinlock, flags);

  /* Send a dequeue message to the user if a message queue is registered */

  msg.msg_id = AUDIO_MSG_DEQUEUE;
  msg.u.ptr = apb;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  msg.session = session;
#endif
  apb->flags |= AUDIO_APB_DEQUEUED;
  audio_sendmsg(upper, &msg);
}

/****************************************************************************
 * Name: audio_complete
 *
 * Description:
 *   Send an AUDIO_MSG_COMPLETE message to the client to indicate that the
 *   active playback has completed.  The lower-half driver initiates this
 *   call via its callback pointer to our upper-half driver.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static inline void audio_complete(FAR struct audio_upperhalf_s *upper,
                    FAR struct ap_buffer_s *apb, uint16_t status,
                    FAR void *session)
#else
static inline void audio_complete(FAR struct audio_upperhalf_s *upper,
                    FAR struct ap_buffer_s *apb, uint16_t status)
#endif
{
  struct audio_msg_s    msg;

  audinfo("Entry\n");

  if (upper->status != NULL)
    {
      audio_setstate(upper, AUDIO_STATE_OPEN);
    }

  /* Send a dequeue message to the user if a message queue is registered */

  msg.msg_id = AUDIO_MSG_COMPLETE;
  msg.u.ptr = NULL;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  msg.session = session;
#endif
  audio_sendmsg(upper, &msg);
}

/****************************************************************************
 * Name: audio_message
 *
 * Description:
 *   Send an custom message to the client to indicate that the
 *   active message has delivered.  The lower-half driver initiates this
 *   call via its callback pointer to our upper-half driver.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static inline void audio_message(FAR struct audio_upperhalf_s *upper,
                    FAR struct ap_buffer_s *apb, uint16_t status,
                    FAR void *session)
#else
static inline void audio_message(FAR struct audio_upperhalf_s *upper,
                    FAR struct ap_buffer_s *apb, uint16_t status)
#endif
{
  struct audio_msg_s *msg = (FAR struct audio_msg_s *)apb;

  /* Send a message to the user if a message queue is registered */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  msg->session = session;
#endif
  audio_sendmsg(upper, msg);
}

/****************************************************************************
 * Name: audio_ioerr
 *
 * Description:
 *   Send an AUDIO_MSG_IOERR message to the client to indicate that
 *   audio driver have io error.  The lower-half driver initiates this
 *   call via its callback pointer to our upper-half driver.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static inline void audio_ioerr(FAR struct audio_upperhalf_s *upper,
                    FAR struct ap_buffer_s *apb, uint16_t status,
                    FAR void *session)
#else
static inline void audio_ioerr(FAR struct audio_upperhalf_s *upper,
                    FAR struct ap_buffer_s *apb, uint16_t status)
#endif
{
  struct audio_msg_s msg;

  audinfo("Entry\n");

  /* Send io error message to the user if a message queue is registered */

  msg.msg_id = AUDIO_MSG_IOERR;
  msg.u.data = status;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  msg.session = session;
#endif
  audio_sendmsg(upper, &msg);
}

/****************************************************************************
 * Name: audio_underrun
 *
 * Description:
 *   Send an AUDIO_MSG_UNDERRUN message to the client to indicate that the
 *   active playback is underrun.  The lower-half driver initiates this
 *   call via its callback pointer to our upper-half driver.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static inline void audio_underrun(FAR struct audio_upperhalf_s *upper,
                    FAR struct ap_buffer_s *apb, uint16_t status,
                    FAR void *session)
#else
static inline void audio_underrun(FAR struct audio_upperhalf_s *upper,
                    FAR struct ap_buffer_s *apb, uint16_t status)
#endif
{
  struct audio_msg_s    msg;

  audinfo("Entry\n");

  /* Send a dequeue message to the user if a message queue is registered */

  msg.msg_id = AUDIO_MSG_UNDERRUN;
  msg.u.ptr = NULL;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  msg.session = session;
#endif
  audio_sendmsg(upper, &msg);
}

/****************************************************************************
 * Name: audio_callback
 *
 * Description:
 *   Provides a callback interface for lower-half drivers to call to the
 *   upper-half for buffer dequeueing, error reporting, etc.
 *
 * Input Parameters:
 *   priv - Private context data owned by the upper-half
 *   reason - The reason code for the callback
 *   apb - A pointer to the previously enqueued ap_buffer_s
 *   status - Status information associated with the callback
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static void audio_callback(FAR void *handle, uint16_t reason,
        FAR struct ap_buffer_s *apb, uint16_t status,
        FAR void *session)
#else
static void audio_callback(FAR void *handle, uint16_t reason,
        FAR struct ap_buffer_s *apb, uint16_t status)
#endif
{
  FAR struct audio_upperhalf_s *upper =
            (FAR struct audio_upperhalf_s *)handle;

  audinfo("Entry\n");

  /* Perform operation based on reason code */

  switch (reason)
    {
      case AUDIO_CALLBACK_DEQUEUE:
        {
          /* Call the dequeue routine */

#ifdef CONFIG_AUDIO_MULTI_SESSION
          audio_dequeuebuffer(upper, apb, status, session);
#else
          audio_dequeuebuffer(upper, apb, status);
#endif
          break;
        }

      /* Lower-half I/O error occurred */

      case AUDIO_CALLBACK_IOERR:
        {
#ifdef CONFIG_AUDIO_MULTI_SESSION
          audio_ioerr(upper, apb, status, session);
#else
          audio_ioerr(upper, apb, status);
#endif
        }
        break;

      /* Lower-half driver has completed a playback */

      case AUDIO_CALLBACK_COMPLETE:
        {
          /* Send a complete message to the user if a message queue
           * is registered
           */

#ifdef CONFIG_AUDIO_MULTI_SESSION
          audio_complete(upper, apb, status, session);
#else
          audio_complete(upper, apb, status);
#endif
        }
        break;

      case AUDIO_CALLBACK_MESSAGE:
        {
#ifdef CONFIG_AUDIO_MULTI_SESSION
          audio_message(upper, apb, status, session);
#else
          audio_message(upper, apb, status);
#endif
        }
        break;

      case AUDIO_CALLBACK_UNDERRUN:
        {
          /* send underrun status */
#ifdef CONFIG_AUDIO_MULTI_SESSION
          audio_underrun(upper, apb, status, session);
#else
          audio_underrun(upper, apb, status);
#endif
        }
        break;

      default:
        {
          auderr("ERROR: Unknown callback reason code %d\n", reason);
          break;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: audio_register
 *
 * Description:
 *   This function binds an instance of a "lower half" audio driver with the
 *   "upper half" Audio device and registers that device so that can be used
 *   by application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   reset state (as if the shutdown() method had already been called).
 *
 * Input Parameters:
 *   path - The full path to the driver to be registers in the NuttX pseudo-
 *     filesystem.  The recommended convention is to name Audio drivers
 *     based on the function they provide, such as "/dev/pcm0", "/dev/mp31",
 *     etc.
 *   dev - A pointer to an instance of lower half audio driver.
 *     This instance is bound to the Audio driver and must persists as long
 *     as the driver persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int audio_register(FAR const char *name, FAR struct audio_lowerhalf_s *dev)
{
  FAR struct audio_upperhalf_s *upper;
  char path[AUDIO_MAX_DEVICE_PATH];
  static bool dev_audio_created = false;
#ifndef CONFIG_AUDIO_CUSTOM_DEV_PATH
  FAR const char *devname = "/dev/audio";
#elif !defined(CONFIG_AUDIO_DEV_ROOT)
  FAR const char *devname = CONFIG_AUDIO_DEV_PATH;
  FAR const char *ptr;
  FAR char *pathptr;
#endif

  /* Allocate the upper-half data structure */

  upper = kmm_zalloc(sizeof(struct audio_upperhalf_s));
  if (!upper)
    {
      auderr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the Audio device structure
   * (it was already zeroed by kmm_zalloc())
   */

  nxmutex_init(&upper->lock);
  spin_lock_init(&upper->spinlock);
  upper->dev = dev;

#ifdef CONFIG_AUDIO_CUSTOM_DEV_PATH

#ifdef CONFIG_AUDIO_DEV_ROOT

  /* This is the simple case ... No need to make a directory */

  strlcpy(path, "/dev/", sizeof(path));
  strlcat(path, name, sizeof(path));

#else
  /* Ensure the path begins with "/dev" as we don't support placing device
   * anywhere but in the /dev directory
   */

  DEBUGASSERT(strncmp(devname, "/dev", 4) == 0);

  /* Create a /dev/audio directory. */

  if (!dev_audio_created)
    {
      /* Get path name after "/dev" */

      ptr = &devname[4];
      if (*ptr == '/')
        {
          ptr++;
        }

      strlcpy(path, "/dev/", sizeof(path));
      pathptr = &path[5];

      /* Do mkdir for each segment of the path */

      while (*ptr != '\0')
        {
          /* Build next path segment into path variable */

          while (*ptr != '/' && *ptr != '\0')
            {
              *pathptr++ = *ptr++;
            }

          *pathptr = '\0';

          /* Make this level of directory */

          mkdir(path, 0644);

          /* Check for another level */

          *pathptr++ = '/';
          if (*ptr == '/')
            {
              ptr++;
            }
        }

      /* Indicate we have created the audio dev path */

      dev_audio_created = true;
    }

  /* Now build the path for registration */

  strlcpy(path, devname, sizeof(path));
  if (devname[sizeof(devname)-1] != '/')
    {
      strlcat(path, "/", sizeof(path));
    }

  strlcat(path, name, sizeof(path));

#endif /* CONFIG_AUDIO_DEV_PATH=="/dev" */

#else  /* CONFIG_AUDIO_CUSTOM_DEV_PATH */

  /* Create a /dev/audio directory. */

  if (!dev_audio_created)
    {
      /* We don't check for error here because even if it fails, then
       * the register_driver call below will return an error condition
       * for us.
       */

      mkdir(devname, 0644);
      dev_audio_created = true;
    }

  /* Register the Audio device */

  strlcpy(path, devname, sizeof(path));
  strlcat(path, "/", sizeof(path));
  strlcat(path, name, sizeof(path));
#endif

  /* Give the lower-half a context to the upper half */

  dev->upper = audio_callback;
  dev->priv = upper;

  audinfo("Registering %s\n", path);
  return register_driver(path, &g_audioops, 0666, upper);
}

#endif /* CONFIG_AUDIO */
