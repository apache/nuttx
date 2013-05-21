/****************************************************************************
 * audio/audio.c
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
 * Compilation Switches
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
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/audio/audio.h>

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

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct audio_upperhalf_s
{
  uint8_t           crefs;    /* The number of times the device has been opened */
  volatile bool     started;  /* True: pulsed output is being generated */
  sem_t             exclsem;  /* Supports mutual exclusion */
  struct audio_info_s info;     /* Pulsed output characteristics */
  FAR struct audio_lowerhalf_s *dev;  /* lower-half state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      audio_open(FAR struct file *filep);
static int      audio_close(FAR struct file *filep);
static ssize_t  audio_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t  audio_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int      audio_start(FAR struct audio_upperhalf_s *upper, unsigned int oflags);
static int      audio_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static void     audio_callback(FAR void *priv, uint16_t reason,
                    FAR struct ap_buffer_s *apb, uint16_t status);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_audioops =
{
  audio_open,  /* open */
  audio_close, /* close */
  audio_read,  /* read */
  audio_write, /* write */
  0,         /* seek */
  audio_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0        /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: audio_open
 *
 * Description:
 *   This function is called whenever the Audio device is opened.
 *
 ************************************************************************************/

static int audio_open(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  uint8_t                     tmp;
  int                         ret;

  audvdbg("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
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
  sem_post(&upper->exclsem);

errout:
  return ret;
}

/************************************************************************************
 * Name: audio_close
 *
 * Description:
 *   This function is called when the Audio device is closed.
 *
 ************************************************************************************/

static int audio_close(FAR struct file *filep)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  int                         ret;

  audvdbg("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
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
      audvdbg("calling shutdown: %d\n");

      lower->ops->shutdown(lower);
    }
  ret = OK;

//errout_with_sem:
  sem_post(&upper->exclsem);

errout:
  return ret;
}

/************************************************************************************
 * Name: audio_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t audio_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/************************************************************************************
 * Name: audio_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satsify the VFS layer.
 *
 ************************************************************************************/

static ssize_t audio_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  return 0;
}

/************************************************************************************
 * Name: audio_start
 *
 * Description:
 *   Handle the AUDIOIOC_START ioctl command
 *
 ************************************************************************************/

static int audio_start(FAR struct audio_upperhalf_s *upper, unsigned int oflags)
{
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  int ret = OK;

  DEBUGASSERT(upper != NULL && lower->ops->start != NULL);

  /* Verify that the Audio is not already running */

  if (!upper->started)
    {
      /* Invoke the bottom half method to start the pulse train */

      ret = lower->ops->start(lower);

      /* A return value of zero means that the pulse train was started
       * successfully.
       */

      if (ret == OK)
        {
          /* Indicate that the pulse train has started */

          upper->started = true;
        }
    }

  return ret;
}

/************************************************************************************
 * Name: audio_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the Audio work is done.
 *
 ************************************************************************************/

static int audio_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode           *inode = filep->f_inode;
  FAR struct audio_upperhalf_s *upper = inode->i_private;
  FAR struct audio_lowerhalf_s *lower = upper->dev;
  int                         ret;

  audvdbg("cmd: %d arg: %ld\n", cmd, arg);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
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
          FAR struct audio_caps_s *caps = (FAR struct audio_caps_s*)((uintptr_t)arg);
          DEBUGASSERT(lower->ops->getcaps != NULL);

          audvdbg("AUDIOIOC_GETCAPS: Device=%d", caps->ac_type);

          /* Call the lower-half driver capabilities handler */
          ret = lower->ops->getcaps(lower, caps->ac_type, caps);
        }
        break;

      case AUDIOIOC_CONFIGURE:
        {
          FAR const struct audio_caps_s *caps = (FAR const struct audio_caps_s*)((uintptr_t)arg);
          DEBUGASSERT(lower->ops->configure != NULL);

          audvdbg("AUDIOIOC_INITIALIZE: Device=%d", caps->ac_type);

          /* Call the lower-half driver configure handler */

          ret = lower->ops->configure(lower, caps, &audio_callback, upper);
        }
        break;

      case AUDIOIOC_SHUTDOWN:
        {
          DEBUGASSERT(lower->ops->shutdown != NULL);

          audvdbg("AUDIOIOC_SHUTDOWN\n");

          /* Call the lower-half driver initialize handler */
          ret = lower->ops->shutdown(lower);
        }
        break;

      /* AUDIOIOC_START - Start the pulsed output.  The AUDIOIOC_SETCHARACTERISTICS
       *   command must have previously been sent.
       *
       *   ioctl argument:  None
       */

      case AUDIOIOC_START:
        {
          audvdbg("AUDIOIOC_START\n");
          DEBUGASSERT(lower->ops->start != NULL);

          /* Start the pulse train */

          ret = audio_start(upper, filep->f_oflags);
        }
        break;

      /* AUDIOIOC_STOP - Stop the pulsed output.
       *
       *   ioctl argument:  None
       */

      case AUDIOIOC_STOP:
        {
          audvdbg("AUDIOIOC_STOP\n");
          DEBUGASSERT(lower->ops->stop != NULL);

          if (upper->started)
            {
              ret = lower->ops->stop(lower);
              upper->started = false;
            }
        }
        break;

      /* Any unrecognized IOCTL commands might be platform-specific ioctl commands */

      default:
        {
          audvdbg("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
          DEBUGASSERT(lower->ops->ioctl != NULL);
          ret = lower->ops->ioctl(lower, cmd, arg);
        }
        break;
    }

  sem_post(&upper->exclsem);
  return ret;
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
 * Input parameters:
 *   path - The full path to the driver to be registers in the NuttX pseudo-
 *     filesystem.  The recommended convention is to name Audio drivers
 *     based on the function they provide, such as "/dev/pcm0", "/dev/mp31",
 *     etc.
 *   dev - A pointer to an instance of lower half audio driver.  This instance
 *     is bound to the Audio driver and must persists as long as the driver
 *     persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int audio_register(FAR const char *name, FAR struct audio_lowerhalf_s *dev)
{
  FAR struct audio_upperhalf_s *upper;
  char  path[AUDIO_MAX_DEVICE_PATH];
  static bool dev_audio_created = false;
#ifndef CONFIG_AUDIO_CUSTOM_DEV_PATH
  const char* devname = "/dev/audio";
#elif !defined(CONFIG_AUDIO_DEV_ROOT)
  const char* devname = CONFIG_AUDIO_DEV_PATH;
  const char* ptr;
  char*       pathptr;
#endif

  /* Allocate the upper-half data structure */

  upper = (FAR struct audio_upperhalf_s *)kzalloc(sizeof(struct audio_upperhalf_s));
  if (!upper)
    {
      auddbg("Allocation failed\n");
      return -ENOMEM;
    }

  /* Initialize the Audio device structure (it was already zeroed by kzalloc()) */

  sem_init(&upper->exclsem, 0, 1);
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

  audvdbg("Registering %s\n", path);
  return register_driver(path, &g_audioops, 0666, upper);
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
 *      ab_buffer, it will call this routine to indicated processing of the
 *      buffer is complete.
 *   4. When this routine is called, it will check if any threads are waiting
 *      to enqueue additional buffers and "wake them up" for further
 *      processing.
 *
 * Input parameters:
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

static inline void audio_dequeuebuffer(FAR struct audio_upperhalf_s *upper,
                    FAR struct ap_buffer_s *apb, uint16_t status)
{
  audllvdbg("Entry\n");

  /* TODO:  Implement the logic */

}

/****************************************************************************
 * Name: audio_callback
 *
 * Description:
 *   Provides a callback interface for lower-half drivers to call to the
 *   upper-half for buffer dequeueing, error reporting, etc.
 *
 * Input parameters:
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

static void audio_callback(FAR void *handle, uint16_t reason,
        FAR struct ap_buffer_s *apb, uint16_t status)
{
  FAR struct audio_upperhalf_s *upper = (FAR struct audio_upperhalf_s *)handle;

  audllvdbg("Entry\n");

  /* Perform operation based on reason code */

  switch (reason)
    {
      case AUDIO_CALLBACK_DEQUEUE:
        {
          /* Call the dequeue routine */

          audio_dequeuebuffer(upper, apb, status);
          break;
        }

      case AUDIO_CALLBACK_IOERR:
        {
        }
        break;

      default:
        {
          auddbg("Unknown callback reason code %d\n", reason);
          break;
        }
    }
}

#endif /* CONFIG_AUDIO */
