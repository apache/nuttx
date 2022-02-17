/****************************************************************************
 * audio/audio.c
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
#include <nuttx/semaphore.h>

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

/* This structure describes the state of the upper half driver */

struct audio_upperhalf_s
{
  uint8_t           crefs;            /* The number of times the device has been opened */
  volatile bool     started;          /* True: playback is active */
  sem_t             exclsem;          /* Supports mutual exclusion */
  FAR struct audio_lowerhalf_s *dev;  /* lower-half state */
  struct file      *usermq;           /* User mode app's message queue */
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
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      audio_start(FAR struct audio_upperhalf_s *upper,
                            FAR void *session);
static void     audio_callback(FAR void *priv,
                               uint16_t reason,
                               FAR struct ap_buffer_s *apb,
                               uint16_t status,
                               FAR void *session);
#else
static int      audio_start(FAR struct audio_upperhalf_s *upper);
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
  NULL         /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL       /* unlink */
#endif
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
  uint8_t tmp;
  int ret;

  audinfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -errno;
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
      goto errout_with_sem;
    }

  /* Save the new open count on success */

  upper->crefs = tmp;
  ret = OK;

errout_with_sem:
  nxsem_post(&upper->exclsem);

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
  int ret;

  audinfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -errno;
      goto errout;
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

      /* Disable the Audio device */

      DEBUGASSERT(lower->ops->shutdown != NULL);
      audinfo("calling shutdown\n");

      lower->ops->shutdown(lower);
      upper->usermq = NULL;
    }

  ret = OK;

  nxsem_post(&upper->exclsem);

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
  int ret = OK;

  DEBUGASSERT(upper != NULL && lower->ops->start != NULL);

  /* Verify that the Audio is not already running */

  if (!upper->started)
    {
      /* Invoke the bottom half method to start the audio stream */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      ret = lower->ops->start(lower, session);
#else
      ret = lower->ops->start(lower);
#endif

      /* A return value of zero means that the audio stream was started
       * successfully.
       */

      if (ret == OK)
        {
          /* Indicate that the audio stream has started */

          upper->started = true;
        }
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
  FAR struct audio_buf_desc_s  *bufdesc;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void *session;
#endif
  int ret;

  audinfo("cmd: %d arg: %ld\n", cmd, arg);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&upper->exclsem);
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

          if (upper->started)
            {
#ifdef CONFIG_AUDIO_MULTI_SESSION
              session = (FAR void *) arg;
              ret = lower->ops->stop(lower, session);
#else
              ret = lower->ops->stop(lower);
#endif
              upper->started = false;
            }
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

          if (upper->started)
            {
#ifdef CONFIG_AUDIO_MULTI_SESSION
              session = (FAR void *) arg;
              ret = lower->ops->pause(lower, session);
#else
              ret = lower->ops->pause(lower);
#endif
            }
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

          if (upper->started)
            {
#ifdef CONFIG_AUDIO_MULTI_SESSION
              session = (FAR void *) arg;
              ret = lower->ops->resume(lower, session);
#else
              ret = lower->ops->resume(lower);
#endif
            }
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
          if (lower->ops->allocbuffer)
            {
              ret = lower->ops->allocbuffer(lower, bufdesc);
            }
          else
            {
              /* Perform a simple kumm_malloc operation assuming 1 session */

              ret = apb_alloc(bufdesc);
            }
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

          /* Call lower-half to perform the release */

#ifdef CONFIG_AUDIO_MULTI_SESSION
          ret = lower->ops->release(lower, (FAR void *) arg);
#else
          ret = lower->ops->release(lower);
#endif
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

  nxsem_post(&upper->exclsem);
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
  struct audio_msg_s    msg;

  audinfo("Entry\n");

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
  struct audio_msg_s    msg;

  audinfo("Entry\n");

  /* Send a dequeue message to the user if a message queue is registered */

  upper->started = false;
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

  upper = (FAR struct audio_upperhalf_s *)kmm_zalloc(
                                           sizeof(struct audio_upperhalf_s));
  if (!upper)
    {
      auderr("ERROR: Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the Audio device structure
   * (it was already zeroed by kmm_zalloc())
   */

  nxsem_init(&upper->exclsem, 0, 1);
  upper->dev = dev;

#ifdef CONFIG_AUDIO_CUSTOM_DEV_PATH

#ifdef CONFIG_AUDIO_DEV_ROOT

  /* This is the simple case ... No need to make a directory */

  strcpy(path, "/dev/");
  strcat(path, name);

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

      strcpy(path, "/dev/");
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

  strcpy(path, devname);
  if (devname[sizeof(devname)-1] != '/')
    {
      strcat(path, "/");
    }

  strcat(path, name);

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

  memset(path, 0, AUDIO_MAX_DEVICE_PATH);
  strcpy(path, devname);
  strcat(path, "/");
  strncat(path, name, AUDIO_MAX_DEVICE_PATH - 11);
#endif

  /* Give the lower-half a context to the upper half */

  dev->upper = audio_callback;
  dev->priv = upper;

  audinfo("Registering %s\n", path);
  return register_driver(path, &g_audioops, 0666, upper);
}

#endif /* CONFIG_AUDIO */
