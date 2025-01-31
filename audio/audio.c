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

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

typedef unsigned int  snd_pcm_format_t;
typedef unsigned long snd_pcm_uframes_t;

typedef enum
{
  STREAM_STATE_NONE,
  STREAM_STATE_PAUSED,
  STREAM_STATE_XRUN,
  STREAM_STATE_RUNNING,
} stream_state_t;

struct appl_s
{
  pid_t          pid;
  stream_state_t state;
  unsigned long  appl_ptr;
};

struct snd_pcm_mmap_status
{
  unsigned long read_head;
  unsigned long read_tail;
};

struct snd_pcm_dmix_share
{
  unsigned int refs;
  snd_pcm_format_t format;
  unsigned int channels;
  unsigned int sample_rate;
  snd_pcm_uframes_t period_frames;
  snd_pcm_uframes_t buffer_frames;
};

/* This structure describes the state of the upper half driver */

struct audio_upperhalf_s
{
  uint8_t           crefs;            /* The number of times the device has been opened */
  stream_state_t    state;            /* lowerhalf state */
  mutex_t           lock;             /* Supports mutual exclusion */
  FAR struct file   *usermq;          /* User mode app's message queue */
  struct dq_queue_s pendq;
  bool              waiting;

  int               periods;
  int               period_bytes;
  int               frame_bytes;

  FAR uint8_t       *mix_buffer;
  int               mix_buffer_size;

  struct snd_pcm_mmap_status     mmap_status;
  FAR struct snd_pcm_dmix_share  *share_info;

  FAR struct appl_s appls[CONFIG_AUDIO_MAX_APPS];
  FAR struct pollfd *fds[CONFIG_AUDIO_MAX_APPS];

  FAR struct audio_lowerhalf_s *dev;  /* lower-half state */
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
                           struct pollfd *fds, bool setup);
static int      audio_allocbuffer(FAR struct audio_upperhalf_s *upper,
                                  FAR struct audio_buf_desc_s * bufdesc);
static int      audio_freebuffer(FAR struct audio_upperhalf_s *upper,
                                 FAR struct audio_buf_desc_s * bufdesc);
static int      audio_enqueue_check(FAR struct audio_upperhalf_s *upper);
FAR static struct appl_s *
                audio_find_this_appl(FAR struct audio_upperhalf_s *upper);
FAR static struct appl_s *
                audio_find_empty_appl(FAR struct audio_upperhalf_s *upper);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      audio_start(FAR struct audio_upperhalf_s *upper,
                            FAR void *session);
static int      audio_stop(FAR struct audio_upperhalf_s *upper,
                           FAR void *session);
static int      audio_pause(FAR struct audio_upperhalf_s *upper,
                            FAR void *session);
static int      audio_resume(FAR struct audio_upperhalf_s *upper,
                             FAR void *session);
static void     audio_callback(FAR void *priv,
                               uint16_t reason,
                               FAR struct ap_buffer_s *apb,
                               uint16_t status,
                               FAR void *session);
#else
static int      audio_start(FAR struct audio_upperhalf_s *upper);
static int      audio_stop(FAR struct audio_upperhalf_s *upper);
static int      audio_pause(FAR struct audio_upperhalf_s *upper);
static int      audio_resume(FAR struct audio_upperhalf_s *upper);
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
  .open  = audio_open,
  .close = audio_close,
  .read  = audio_read,
  .write = audio_write,
  .ioctl = audio_ioctl,
  .mmap  = audio_mmap,
  .poll  = audio_poll,
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
  FAR struct appl_s *appl;
  uint8_t tmp;
  int ret;

  audinfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      goto errout;
    }

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_lock;
    }

  appl = audio_find_empty_appl(upper);
  if (appl == NULL)
    {
      ret = -EMFILE;
      goto errout_with_lock;
    }

  appl->pid = getpid();
  appl->state = STREAM_STATE_NONE;
  appl->appl_ptr = 0;

  /* Save the new open count on success */

  upper->crefs = tmp;
  ret = OK;

errout_with_lock:
  nxmutex_unlock(&upper->lock);

errout:
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
  FAR struct appl_s *appl;
  int ret;

  audinfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      goto errout;
    }

  appl = audio_find_this_appl(upper);
  if (appl != NULL)
    {
      appl->state = STREAM_STATE_NONE;
      appl->appl_ptr = 0;
      appl->pid = 0;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 1)
    {
      upper->crefs--;
    }
  else
    {
      FAR struct audio_lowerhalf_s *lower = upper->dev;

      /* There are no more references to the port */

      upper->crefs = 0;

      if (!upper->mix_buffer)
        {
          /* Disable the Audio device */

          DEBUGASSERT(lower->ops->shutdown != NULL);
          audinfo("calling shutdown\n");

          lower->ops->shutdown(lower);
          upper->usermq = NULL;
        }
    }

  ret = OK;
  nxmutex_unlock(&upper->lock);

errout:
  return ret;
}

/****************************************************************************
 * Name: audio_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satsify the VFS layer.
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
 *   A dummy write method.  This is provided only to satsify the VFS layer.
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
  FAR struct audio_buf_desc_s  *bufdesc;
  FAR struct ap_buffer_info_s *bufinfo;
  FAR struct appl_s *appl;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void *session;
#endif
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
          DEBUGASSERT(lower->ops->configure != NULL);

          audinfo("AUDIOIOC_INITIALIZE: Device=%d\n", caps->caps.ac_type);

          if (caps->caps.ac_type == AUDIO_TYPE_OUTPUT)
            {
              upper->frame_bytes =
                  caps->caps.ac_controls.b[2] / 8 * caps->caps.ac_channels;
            }

          /* Call the lower-half driver configure handler */

#ifdef CONFIG_AUDIO_MULTI_SESSION
          ret = lower->ops->configure(lower, caps->session, &caps->caps);
#else
          ret = lower->ops->configure(lower, &caps->caps);
#endif
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
          DEBUGASSERT(lower->ops->start != NULL);

          /* Start the audio stream */

#ifdef CONFIG_AUDIO_MULTI_SESSION
          session = (FAR void *) arg;
          ret = audio_start(upper, session);
#else
          ret = audio_start(upper);
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
          DEBUGASSERT(lower->ops->stop != NULL);

#ifdef CONFIG_AUDIO_MULTI_SESSION
          session = (FAR void *) arg;
          ret = audio_stop(upper, session);
#else
          ret = audio_stop(upper);
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
          DEBUGASSERT(lower->ops->pause != NULL);

#ifdef CONFIG_AUDIO_MULTI_SESSION
          session = (FAR void *) arg;
          ret = audio_pause(upper, session);
#else
          ret = audio_pause(upper);
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
          DEBUGASSERT(lower->ops->resume != NULL);

#ifdef CONFIG_AUDIO_MULTI_SESSION
          session = (FAR void *) arg;
          ret = audio_resume(upper, session);
#else
          ret = audio_resume(upper);
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

          bufdesc = (FAR struct audio_buf_desc_s *) arg;
          ret = audio_allocbuffer(upper, bufdesc);
        }
        break;

      /* AUDIOIOC_FREEBUFFER - Free an audio buffer
       *
       *   ioctl argument:  pointer to an audio_buf_desc_s structure
       */

      case AUDIOIOC_FREEBUFFER:
        {
          audinfo("AUDIOIOC_FREEBUFFER\n");

          bufdesc = (FAR struct audio_buf_desc_s *) arg;
          ret = audio_freebuffer(upper, bufdesc);
        }
        break;

      /* AUDIOIOC_ENQUEUEBUFFER - Enqueue an audio buffer
       *
       *   ioctl argument:  pointer to an audio_buf_desc_s structure
       */

      case AUDIOIOC_ENQUEUEBUFFER:
        {
          audinfo("AUDIOIOC_ENQUEUEBUFFER\n");

          DEBUGASSERT(lower->ops->enqueuebuffer != NULL);

          bufdesc = (FAR struct audio_buf_desc_s *) arg;
          ret = lower->ops->enqueuebuffer(lower, bufdesc->u.buffer);
        }
        break;

      /* AUDIOIOC_REGISTERMQ - Register a client Message Queue
       *
       * TODO:  This needs to have multi session support.
       */

      case AUDIOIOC_REGISTERMQ:
        {
          audinfo("AUDIOIOC_REGISTERMQ\n");

          ret = fs_getfilep((mqd_t)arg, &upper->usermq);
        }
        break;

      /* AUDIOIOC_UNREGISTERMQ - Register a client Message Queue
       *
       * TODO:  This needs to have multi session support.
       */

      case AUDIOIOC_UNREGISTERMQ:
        {
          audinfo("AUDIOIOC_UNREGISTERMQ\n");

          fs_putfilep(upper->usermq);
          upper->usermq = NULL;
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

          if (upper->crefs > 0)
            {
              ret = OK;
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

          if (upper->crefs > 1)
            {
              ret = OK;
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

      case AUDIOIOC_PTR_APPL:
        {
          ret = OK;
          appl = audio_find_this_appl(upper);
          if (!appl)
            {
              ret = -ENOENT;
              break;
            }

          /* reset */

          if (arg == 0)
            {
              appl->appl_ptr = upper->mmap_status.read_head;
              upper->waiting = true;

              ret = OK;
              audinfo("[%s %d],arg:%ld [pid:%d, %ld %ld %ld]", __func__,
                      __LINE__, (signed long)arg, appl->pid,
                      upper->mmap_status.read_tail,
                      upper->mmap_status.read_head, appl->appl_ptr);
              break;
            }

          appl->appl_ptr += arg;
          audinfo("[%s %d],arg:%ld [pid:%d, %ld %ld %ld]", __func__,
                  __LINE__, (signed long)arg, appl->pid,
                  upper->mmap_status.read_tail, upper->mmap_status.read_head,
                  appl->appl_ptr);
          if (appl->state == STREAM_STATE_XRUN)
            {
              appl->state = STREAM_STATE_RUNNING;
            }

          if (appl->state == STREAM_STATE_RUNNING)
            {
              ret = audio_enqueue_check(upper);
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

          if (ret == OK)
            {
              if (cmd == AUDIOIOC_GETBUFFERINFO &&
                  upper->mix_buffer_size == 0)
                {
                  bufinfo = (FAR struct ap_buffer_info_s *)arg;
                  upper->periods = bufinfo->nbuffers;
                  upper->period_bytes = bufinfo->buffer_size;
                  upper->mix_buffer_size =
                      upper->periods * upper->period_bytes;
                }
            }
        }
        break;
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

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
      goto errout;
    }

  /* Return the address corresponding to the start of frame buffer. */

  if (map->length == upper->mix_buffer_size)
    {
      if (!upper->mix_buffer)
        {
          upper->mix_buffer = kmm_zalloc(upper->mix_buffer_size);
          audinfo("[%s %d], mix_buffer:%p", __func__, __LINE__,
                  upper->mix_buffer);
          if (!upper->mix_buffer)
            {
              ret = -ENOMEM;
              goto errout_with_lock;
            }

          upper->mmap_status.read_head = 0;
          upper->mmap_status.read_tail = 0;
        }

      map->vaddr = (FAR char *)upper->mix_buffer + map->offset;
      map->munmap = audio_munmap;
      ret = mm_map_add(get_current_mm(), map);
    }
  else if (map->length == sizeof(struct snd_pcm_dmix_share))
    {
      if (!upper->share_info)
        {
          upper->share_info = kmm_zalloc(map->length);
          if (!upper->share_info)
            {
              ret = -ENOMEM;
              goto errout_with_lock;
            }
        }

      map->vaddr = (FAR char *)upper->share_info;
      map->munmap = audio_munmap;
      ret = mm_map_add(get_current_mm(), map);
    }
  else if (map->length == sizeof(struct snd_pcm_mmap_status))
    {
      map->vaddr = (FAR char *)&upper->mmap_status;
      map->munmap = audio_munmap;
      ret = mm_map_add(get_current_mm(), map);
    }
  else
    {
      ret = -EINVAL;
    }

errout_with_lock:
  nxmutex_unlock(&upper->lock);

errout:
  return ret;
}

/****************************************************************************
 * Name: audio_poll
 *
 * Description:
 *   Wait for framebuffer to be writable.
 *
 ****************************************************************************/

static int audio_poll(FAR struct file *filep, struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct pollfd **pollfds = NULL;
  FAR struct appl_s *appl;
  irqstate_t flags;
  int ret = OK;
  int i;

  DEBUGASSERT(upper != NULL);

  flags = enter_critical_section();

  if (setup)
    {
      for (i = 0; i < CONFIG_AUDIO_MAX_APPS; ++i)
        {
          if (!upper->fds[i])
            {
              pollfds = &upper->fds[i];
              break;
            }
        }

      if (pollfds == NULL)
        {
          ret = -EBUSY;
          goto errout;
        }

      *pollfds = fds;
      fds->priv = pollfds;

      appl = audio_find_this_appl(upper);
      if (!appl)
        {
          ret = -ENOENT;
          goto errout;
        }

      if (appl->appl_ptr == upper->mmap_status.read_tail)
        {
          audinfo("[%s %d], poll_notify, [pid:%d, %ld %ld %ld]", __func__,
                  __LINE__, appl->pid, upper->mmap_status.read_tail,
                  upper->mmap_status.read_head, appl->appl_ptr);
          poll_notify(&fds, 1, POLLOUT);
        }
    }
  else if (fds->priv != NULL)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;
      *slot = NULL;
      fds->priv = NULL;
    }

errout:
  leave_critical_section(flags);
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
  int ret;

  if (lower->ops->allocbuffer)
    {
      ret = lower->ops->allocbuffer(lower, bufdesc);
    }
  else if (upper->mix_buffer)
    {
      apb = kmm_zalloc(sizeof(struct ap_buffer_s));
      if (!apb)
        {
          return -ENOMEM;
        }

      apb->crefs = 1;
      apb->nmaxbytes = bufdesc->numbytes;
      apb->nbytes = 0;
      apb->flags = 0;
      apb->samp = NULL;
      nxmutex_init(&apb->lock);
      *bufdesc->u.pbuffer = apb;
      ret = sizeof(struct audio_buf_desc_s);
    }
  else
    {
      /* Perform a simple kumm_malloc operation assuming 1 session */

      ret = apb_alloc(bufdesc);
    }

  return ret;
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
  FAR struct ap_buffer_s *apb;
  int ret;

  if (lower->ops->freebuffer)
    {
      ret = lower->ops->freebuffer(lower, bufdesc);
    }
  else if (upper->mix_buffer)
    {
      apb = bufdesc->u.buffer;
      kmm_free(apb);
      ret = sizeof(struct audio_buf_desc_s);
    }
  else
    {
      /* Perform a simple apb_free operation */

      DEBUGASSERT(bufdesc->u.buffer != NULL);
      apb_free(bufdesc->u.buffer);
      ret = sizeof(struct audio_buf_desc_s);
    }

  return ret;
}

/****************************************************************************
 * Name: audio_enqueue_check
 *
 * Description:
 *   Enqueue audio buffer
 *
 ****************************************************************************/

static int audio_enqueue_check(FAR struct audio_upperhalf_s *upper)
{
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  FAR struct ap_buffer_s *apb;
  FAR struct appl_s *appl;
  int running_stream;
  int need_enqueue;
  int ret = OK;
  int offset;
  int count;
  int i;

  appl = audio_find_this_appl(upper);
  if (!appl)
    {
      return -ENOENT;
    }

  audinfo("[%s %d]enter [pid:%d, %ld %ld %ld]", __func__, __LINE__,
          appl->pid, upper->mmap_status.read_tail,
          upper->mmap_status.read_head, appl->appl_ptr);

check_again:
  running_stream = 0;
  need_enqueue = true;
  count = dq_count(&upper->pendq);
  if (count == 0)
    {
      return OK;
    }

  /* waiting new stream join */

  if (upper->waiting)
    {
      if (upper->periods - count >= 2)
        {
          return OK;
        }

      upper->waiting = false;
    }

  if (appl->appl_ptr <= upper->mmap_status.read_head)
    {
      return 0;
    }

  for (i = 0; i < CONFIG_AUDIO_MAX_APPS; i++)
    {
      if (upper->appls[i].pid == 0 ||
          upper->appls[i].state != STREAM_STATE_RUNNING)
        {
          continue;
        }

      running_stream++;

      /* upper don't have enough readable buffer */

      if (upper->appls[i].appl_ptr <= upper->mmap_status.read_head)
        {
          if (upper->appls[i].appl_ptr < upper->mmap_status.read_head ||
              upper->appls[i].appl_ptr <= upper->mmap_status.read_tail)
            {
              audwarn("[%s %d]xrun occurs [pid:%d, %ld %ld %ld]", __func__,
                      __LINE__, upper->appls[i].pid,
                      upper->mmap_status.read_tail,
                      upper->mmap_status.read_head,
                      upper->appls[i].appl_ptr);
              upper->appls[i].state = STREAM_STATE_XRUN;
            }

          /* lower have enough buffer, wait next time */

          if (upper->periods - count >= 2)
            {
              need_enqueue = false;
            }
          break;
        }
    }

  if (running_stream && need_enqueue)
    {
      apb = (FAR struct ap_buffer_s *)dq_remfirst(&upper->pendq);
      offset = upper->mmap_status.read_head % upper->periods *
               upper->period_bytes;

      if (lower->ops->allocbuffer)
        {
          memcpy(apb->samp, upper->mix_buffer + offset, upper->period_bytes);
          memset(upper->mix_buffer + offset, 0, upper->period_bytes);
        }
      else
        {
          apb->samp = upper->mix_buffer + offset;
        }

      apb->nmaxbytes = upper->period_bytes;
      apb->nbytes = apb->nmaxbytes;
      apb->curbyte = 0;

      upper->mmap_status.read_head++;

      audinfo("write data[%p:%d %d %d %d]\n", apb->samp, apb->samp[0],
              apb->samp[1], apb->samp[2], apb->samp[3]);

      ret = lower->ops->enqueuebuffer(lower, apb);
      goto check_again;
    }

  audinfo("[%s %d]leave, ret:%d", __func__, __LINE__, ret);
  return ret;
}

/****************************************************************************
 * Name: audio_find_this_appl
 *
 * Description:
 *   Find this application
 *
 ****************************************************************************/

FAR static struct appl_s *
audio_find_this_appl(FAR struct audio_upperhalf_s *upper)
{
  pid_t pid = getpid();
  int i;

  for (i = 0; i < CONFIG_AUDIO_MAX_APPS; i++)
    {
      if (upper->appls[i].pid == pid)
        {
          return &upper->appls[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: audio_find_empty_appl
 *
 * Description:
 *   Find empty application
 *
 ****************************************************************************/

FAR static struct appl_s *
audio_find_empty_appl(FAR struct audio_upperhalf_s *upper)
{
  int i;

  for (i = 0; i < CONFIG_AUDIO_MAX_APPS; i++)
    {
      if (upper->appls[i].pid == 0)
        {
          return &upper->appls[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: audio_start
 *
 * Description:
 *   Handle the AUDIOIOC_START ioctl command
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_start(FAR struct audio_upperhalf_s *upper,
                       FAR void *session)
#else
static int audio_start(FAR struct audio_upperhalf_s *upper)
#endif
{
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  struct audio_buf_desc_s buf_desc;
  FAR struct ap_buffer_s *apb;
  FAR struct appl_s *appl;
  int ret = OK;
  int i;

  DEBUGASSERT(upper != NULL && lower->ops->start != NULL);

  if (upper->mix_buffer)
    {
      if (upper->state == STREAM_STATE_PAUSED)
        {
          return audio_resume(upper);
        }

      appl = audio_find_this_appl(upper);
      if (appl != NULL)
        {
          appl->state = STREAM_STATE_RUNNING;
        }
    }

  /* Verify that the Audio is not already running */

  if (upper->state == STREAM_STATE_NONE)
    {
      if (upper->mix_buffer)
        {
          buf_desc.numbytes = upper->period_bytes;
          buf_desc.u.pbuffer = &apb;
          for (i = dq_count(&upper->pendq); i < upper->periods; i++)
            {
              ret = audio_allocbuffer(upper, &buf_desc);
              if (ret < 0)
                {
                  return ret;
                }

              dq_addlast(&apb->dq_entry, &upper->pendq);
            }

          audio_enqueue_check(upper);
        }

      /* Invoke the bottom half method to start the audio stream */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      ret = lower->ops->start(lower, session);
#else
      ret = lower->ops->start(lower);
#endif
      /* A return value of zero means that the audio stream was running
       * successfully.
       */

      if (ret == OK)
        {
          /* Indicate that the audio stream has running */

          upper->state = STREAM_STATE_RUNNING;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: audio_stop
 *
 * Description:
 *   Handle the AUDIOIOC_STOP ioctl command
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_stop(FAR struct audio_upperhalf_s *upper,
                      FAR void *session)
#else
static int audio_stop(FAR struct audio_upperhalf_s *upper)
#endif
{
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  stream_state_t app_state = STREAM_STATE_NONE;
  FAR struct appl_s *appl;
  int ret = OK;
  int i;

  DEBUGASSERT(upper != NULL && lower->ops->start != NULL);

  if (upper->mix_buffer)
    {
      appl = audio_find_this_appl(upper);
      if (appl != NULL)
        {
          appl->state = STREAM_STATE_NONE;
        }

      for (i = 0; i < CONFIG_AUDIO_MAX_APPS; i++)
        {
          if (upper->appls[i].pid != 0 && upper->appls[i].state > app_state)
            {
              app_state = upper->appls[i].state;
            }
        }

      if (app_state == STREAM_STATE_PAUSED)
        {
          return audio_pause(upper);
        }
    }

  if (upper->state != STREAM_STATE_NONE && app_state == STREAM_STATE_NONE)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      session = (FAR void *) arg;
      ret = lower->ops->stop(lower, session);
#else
      ret = lower->ops->stop(lower);
#endif
      upper->state = STREAM_STATE_NONE;
    }

  return ret;
}

/****************************************************************************
 * Name: audio_pause
 *
 * Description:
 *   Handle the AUDIOIOC_PAUSE ioctl command
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_pause(FAR struct audio_upperhalf_s *upper,
                       FAR void *session)
#else
static int audio_pause(FAR struct audio_upperhalf_s *upper)
#endif
{
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  FAR struct appl_s *appl;
  int need_pause = true;
  int ret = OK;
  int i;

  DEBUGASSERT(upper != NULL && lower->ops->start != NULL);

  if (upper->mix_buffer)
    {
      appl = audio_find_this_appl(upper);
      if (appl != NULL)
        {
          appl->state = STREAM_STATE_PAUSED;
        }

      for (i = 0; i < CONFIG_AUDIO_MAX_APPS; i++)
        {
          if (upper->appls[i].pid != 0 &&
              upper->appls[i].state > STREAM_STATE_PAUSED)
            {
              need_pause = false;
              break;
            }
        }
    }

  if (upper->state == STREAM_STATE_RUNNING && need_pause)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      session = (FAR void *) arg;
      ret = lower->ops->pause(lower, session);
#else
      ret = lower->ops->pause(lower);
#endif
      if (ret == OK)
        {
          upper->state = STREAM_STATE_PAUSED;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: audio_resume
 *
 * Description:
 *   Handle the AUDIOIOC_resume ioctl command
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_resume(FAR struct audio_upperhalf_s *upper,
                        FAR void *session)
#else
static int audio_resume(FAR struct audio_upperhalf_s *upper)
#endif
{
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  FAR struct appl_s *appl;
  int need_resume = true;
  int ret = OK;
  int i;

  DEBUGASSERT(upper != NULL && lower->ops->start != NULL);

  if (upper->mix_buffer)
    {
      for (i = 0; i < CONFIG_AUDIO_MAX_APPS; i++)
        {
          if (upper->appls[i].pid != 0 &&
              upper->appls[i].state > STREAM_STATE_PAUSED)
            {
              need_resume = false;
              break;
            }
        }

      appl = audio_find_this_appl(upper);
      if (appl != NULL)
        {
          appl->state = STREAM_STATE_RUNNING;
          upper->waiting = true;
        }
    }

  if (upper->state == STREAM_STATE_PAUSED && need_resume)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      session = (FAR void *) arg;
      ret = lower->ops->resume(lower, session);
#else
      ret = lower->ops->resume(lower);
#endif
      if (ret == OK)
        {
          upper->state = STREAM_STATE_RUNNING;
        }
    }

  return ret;
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
 *   apb - A pointer to the previsously enqueued ap_buffer_s
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
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  struct audio_msg_s msg;
  irqstate_t flags;

  audinfo("Entry\n");

  if (upper->mix_buffer)
    {
      upper->mmap_status.read_tail++;
      if (lower->ops->allocbuffer == NULL)
        {
          memset(apb->samp, 0, apb->nmaxbytes);
        }

      dq_addlast(&apb->dq_entry, &upper->pendq);

      flags = enter_critical_section();
      poll_notify(upper->fds, CONFIG_AUDIO_MAX_APPS, POLLOUT);
      leave_critical_section(flags);
    }

  /* Send a dequeue message to the user if a message queue is registered */

  if (upper->usermq != NULL)
    {
      msg.msg_id = AUDIO_MSG_DEQUEUE;
      msg.u.ptr = apb;
#ifdef CONFIG_AUDIO_MULTI_SESSION
      msg.session = session;
#endif
      apb->flags |= AUDIO_APB_DEQUEUED;
      file_mq_send(upper->usermq, (FAR const char *)&msg, sizeof(msg),
                   CONFIG_AUDIO_BUFFER_DEQUEUE_PRIO);
    }
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
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  struct audio_buf_desc_s buf_desc;
  struct audio_msg_s msg;

  audinfo("Entry\n");

  if (!nxmutex_is_locked(&upper->lock) && nxmutex_lock(&upper->lock) == OK)
    {
      if (upper->mix_buffer && upper->crefs == 0)
        {
          kmm_free(upper->share_info);
          upper->share_info = NULL;
          kmm_free(upper->mix_buffer);
          upper->mix_buffer = NULL;
          upper->mix_buffer_size = 0;

          while (!dq_empty(&upper->pendq))
            {
              buf_desc.u.buffer =
                  (FAR struct ap_buffer_s *)dq_remfirst(&upper->pendq);
              audio_freebuffer(upper, &buf_desc);
            }

          /* Disable the Audio device */

          DEBUGASSERT(lower->ops->shutdown != NULL);

          audinfo("calling shutdown\n");

          lower->ops->shutdown(lower);
        }

      nxmutex_unlock(&upper->lock);
    }

  /* Send a dequeue message to the user if a message queue is registered */

  if (upper->usermq != NULL)
    {
      msg.msg_id = AUDIO_MSG_COMPLETE;
      msg.u.ptr = NULL;
#ifdef CONFIG_AUDIO_MULTI_SESSION
      msg.session = session;
#endif
      file_mq_send(upper->usermq, (FAR const char *)&msg, sizeof(msg),
                   CONFIG_AUDIO_BUFFER_DEQUEUE_PRIO);
    }
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

  if (upper->usermq != NULL)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      msg->session = session;
#endif
      file_mq_send(upper->usermq, (FAR const char *)msg, sizeof(*msg),
                   CONFIG_AUDIO_BUFFER_DEQUEUE_PRIO);
    }
}

/****************************************************************************
 * Name: audio_ioerr
 *
 * Description:
 *   Send an AUDIO_MSG_IOERR message to the client to indicate that
 *   audio dirver have io error.  The lower-half driver initiates this
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

  if (upper->usermq != NULL)
    {
      msg.msg_id = AUDIO_MSG_IOERR;
      msg.u.data = status;
#ifdef CONFIG_AUDIO_MULTI_SESSION
      msg.session = session;
#endif
      file_mq_send(upper->usermq, (FAR const char *)&msg, sizeof(msg),
                   CONFIG_AUDIO_BUFFER_DEQUEUE_PRIO);
    }
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

  if (upper->usermq != NULL)
    {
      msg.msg_id = AUDIO_MSG_UNDERRUN;
      msg.u.ptr = NULL;
#ifdef CONFIG_AUDIO_MULTI_SESSION
      msg.session = session;
#endif
      file_mq_send(upper->usermq, (FAR const char *)&msg, sizeof(msg),
                   CONFIG_AUDIO_BUFFER_DEQUEUE_PRIO);
    }
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
 *   apb - A pointer to the previsously enqueued ap_buffer_s
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
  dq_init(&upper->pendq);
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
  if (devname[strlen(devname)-1] != '/')
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
