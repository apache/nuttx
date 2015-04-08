/****************************************************************************
 * drivers/audio/audio_null.c
 *
 * A do-nothinig audio device driver to simplify testing of audio decoders.
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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
#include <sys/ioctl.h>

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/audio_null.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct null_dev_s
{
  struct audio_lowerhalf_s dev; /* Audio lower half (this device) */
  mqd_t         mq;             /* Message queue for receiving messages */
  char          mqname[16];     /* Our message queue name */
  pthread_t     threadid;       /* ID of our thread */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  volatile bool terminate;      /* True: request to terminate */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      null_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                  FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      null_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session, FAR const struct audio_caps_s *caps);
#else
static int      null_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR const struct audio_caps_s *caps);
#endif
static int      null_shutdown(FAR struct audio_lowerhalf_s *dev);
static void    *null_workerthread(pthread_addr_t pvarg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      null_start(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      null_start(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      null_stop(FAR struct audio_lowerhalf_s *dev,
                  FAR void* session);
#else
static int      null_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      null_pause(FAR struct audio_lowerhalf_s *dev,
                  FAR void* session);
static int      null_resume(FAR struct audio_lowerhalf_s *dev,
                  FAR void* session);
#else
static int      null_pause(FAR struct audio_lowerhalf_s *dev);
static int      null_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int      null_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                  FAR struct ap_buffer_s *apb);
static int      null_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                  FAR struct ap_buffer_s *apb);
static int      null_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                  unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      null_reserve(FAR struct audio_lowerhalf_s *dev,
                  FAR void **session);
#else
static int      null_reserve(FAR struct audio_lowerhalf_s *dev);
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      null_release(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      null_release(FAR struct audio_lowerhalf_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audioops =
{
  null_getcaps,       /* getcaps        */
  null_configure,     /* configure      */
  null_shutdown,      /* shutdown       */
  null_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  null_stop,          /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  null_pause,         /* pause          */
  null_resume,        /* resume         */
#endif
  NULL,               /* allocbuffer    */
  NULL,               /* freebuffer     */
  null_enqueuebuffer, /* enqueue_buffer */
  null_cancelbuffer,  /* cancel_buffer  */
  null_ioctl,         /* ioctl          */
  NULL,               /* read           */
  NULL,               /* write          */
  null_reserve,       /* reserve        */
  null_release        /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: null_getcaps
 *
 * Description: Get the audio device capabilities
 *
 ****************************************************************************/

static int null_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                          FAR struct audio_caps_s *caps)
{
  audvdbg("type=%d\n", type);

  /* Validate the structure */

  DEBUGASSERT(caps->ac_len >= sizeof(struct audio_caps_s));

  /* Fill in the caller's structure based on requested info */

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      /* Caller is querying for the types of units we support */

      case AUDIO_TYPE_QUERY:

        /* Provide our overall capabilities.  The interfacing software
         * must then call us back for specific info for each capability.
         */

        caps->ac_channels = 2;       /* Stereo output */

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:
              /* We don't decode any formats!  Only something above us in
               * the audio stream can perform decoding on our behalf.
               */

              /* The types of audio units we implement */

              caps->ac_controls.b[0] = AUDIO_TYPE_OUTPUT | AUDIO_TYPE_FEATURE |
                                       AUDIO_TYPE_PROCESSING;

              break;

            case AUDIO_FMT_MIDI:
              /* We only support Format 0 */

              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;

            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;
          }

        break;

      /* Provide capabilities of our OUTPUT unit */

      case AUDIO_TYPE_OUTPUT:

        caps->ac_channels = 2;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* Report the Sample rates we support */

              caps->ac_controls.b[0] = AUDIO_SAMP_RATE_8K | AUDIO_SAMP_RATE_11K |
                                       AUDIO_SAMP_RATE_16K | AUDIO_SAMP_RATE_22K |
                                       AUDIO_SAMP_RATE_32K | AUDIO_SAMP_RATE_44K |
                                       AUDIO_SAMP_RATE_48K;
              break;

            case AUDIO_FMT_MP3:
            case AUDIO_FMT_WMA:
            case AUDIO_FMT_PCM:
              break;

            default:
              break;
          }

        break;

      /* Provide capabilities of our FEATURE units */

      case AUDIO_TYPE_FEATURE:

        /* If the sub-type is UNDEF, then report the Feature Units we support */

        if (caps->ac_subtype == AUDIO_FU_UNDEF)
          {
            /* Fill in the ac_controls section with the Feature Units we have */

            caps->ac_controls.b[0] = AUDIO_FU_VOLUME | AUDIO_FU_BASS | AUDIO_FU_TREBLE;
            caps->ac_controls.b[1] = AUDIO_FU_BALANCE >> 8;
          }
        else
          {
            /* TODO:  Do we need to provide specific info for the Feature Units,
             * such as volume setting ranges, etc.?
             */
          }

        break;

      /* Provide capabilities of our PROCESSING unit */

      case AUDIO_TYPE_PROCESSING:

        switch (caps->ac_subtype)
          {
            case AUDIO_PU_UNDEF:

              /* Provide the type of Processing Units we support */

              caps->ac_controls.b[0] = AUDIO_PU_STEREO_EXTENDER;
              break;

            case AUDIO_PU_STEREO_EXTENDER:

              /* Provide capabilities of our Stereo Extender */

              caps->ac_controls.b[0] = AUDIO_STEXT_ENABLE | AUDIO_STEXT_WIDTH;
              break;

            default:

              /* Other types of processing uint we don't support */

              break;
          }

        break;

      /* All others we don't support */

      default:

        /* Zero out the fields to indicate no support */

        caps->ac_subtype = 0;
        caps->ac_channels = 0;

        break;
    }

  /* Return the length of the audio_caps_s struct for validation of
   * proper Audio device type.
   */

  audvdbg("Return %d\n", caps->ac_len);
  return caps->ac_len;
}

/****************************************************************************
 * Name: null_configure
 *
 * Description:
 *   Configure the audio device for the specified  mode of operation.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int null_configure(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session,
                          FAR const struct audio_caps_s *caps)
#else
static int null_configure(FAR struct audio_lowerhalf_s *dev,
                          FAR const struct audio_caps_s *caps)
#endif
{
  audvdbg("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
    case AUDIO_TYPE_FEATURE:
      audvdbg("  AUDIO_TYPE_FEATURE\n");

      /* Process based on Feature Unit */

      switch (caps->ac_format.hw)
        {
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
        case AUDIO_FU_VOLUME:
          audvdbg("    Volume: %d\n", caps->ac_controls.hw[0]);
          break;
#endif  /* CONFIG_AUDIO_EXCLUDE_VOLUME */

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
        case AUDIO_FU_BASS:
          audvdbg("    Bass: %d\n", caps->ac_controls.b[0]);
          break;

        case AUDIO_FU_TREBLE:
          audvdbg("    Treble: %d\n", caps->ac_controls.b[0]);
          break;
#endif  /* CONFIG_AUDIO_EXCLUDE_TONE */

        default:
          auddbg("    Unrecognized feature unit\n");
          break;
        }
      break;

    case AUDIO_TYPE_OUTPUT:
      audvdbg("  AUDIO_TYPE_OUTPUT:\n");
      audvdbg("    Number of channels: %u\n", caps->ac_channels);
      audvdbg("    Sample rate:        %u\n", caps->ac_controls.hw[0]);
      audvdbg("    Sample width:       %u\n", caps->ac_controls.b[2]);
      break;

    case AUDIO_TYPE_PROCESSING:
      audvdbg("  AUDIO_TYPE_PROCESSING:\n");
      break;
    }

  audvdbg("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: null_shutdown
 *
 * Description:
 *   Shutdown the driver and put it in the lowest power state possible.
 *
 ****************************************************************************/

static int null_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  audvdbg("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: null_workerthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 *
 ****************************************************************************/

static void *null_workerthread(pthread_addr_t pvarg)
{
  FAR struct null_dev_s *priv = (struct null_dev_s *) pvarg;
  struct audio_msg_s msg;
  int msglen;
  int prio;

  audvdbg("Entry\n");

  /* Loop as long as we are supposed to be running */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  while (!priv->terminate)
#else
  for (;;)
#endif
    {
      /* Wait for messages from our message queue */

      msglen = mq_receive(priv->mq, (FAR char *)&msg, sizeof(msg), &prio);

      /* Handle the case when we return with no message */

      if (msglen < sizeof(struct audio_msg_s))
        {
          auddbg("ERROR: Message too small: %d\n", msglen);
          continue;
        }

      /* Process the message */

      switch (msg.msgId)
        {
          case AUDIO_MSG_DATA_REQUEST:
            break;

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
          case AUDIO_MSG_STOP:
            priv->terminate = true;
            break;
#endif

          case AUDIO_MSG_ENQUEUE:
            break;

          case AUDIO_MSG_COMPLETE:
            break;

          default:
            auddbg("ERROR: Ignoring message ID %d\n", msg.msgId);
            break;
        }
    }

  /* Close the message queue */

  mq_close(priv->mq);
  mq_unlink(priv->mqname);
  priv->mq = NULL;

  /* Send an AUDIO_MSG_COMPLETE message to the client */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  audvdbg("Exit\n");
  return NULL;
}

/****************************************************************************
 * Name: null_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int null_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int null_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct null_dev_s *priv = (FAR struct null_dev_s *)dev;
  struct sched_param sparam;
  struct mq_attr attr;
  pthread_attr_t tattr;
  FAR void *value;
  int ret;

  audvdbg("Entry\n");

  /* Create a message queue for the worker thread */

  snprintf(priv->mqname, sizeof(priv->mqname), "/tmp/%X", priv);

  attr.mq_maxmsg  = 16;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  priv->mq = mq_open(priv->mqname, O_RDWR | O_CREAT, 0644, &attr);
  if (priv->mq == NULL)
    {
      /* Error creating message queue! */

      auddbg("ERROR: Couldn't allocate message queue\n");
      return -ENOMEM;
    }

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      audvdbg("Joining old thread\n");
      pthread_join(priv->threadid, &value);
    }

  /* Start our thread for sending data to the device */

  pthread_attr_init(&tattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  (void)pthread_attr_setschedparam(&tattr, &sparam);
  (void)pthread_attr_setstacksize(&tattr, CONFIG_AUDIO_NULL_WORKER_STACKSIZE);

  audvdbg("Starting worker thread\n");
  ret = pthread_create(&priv->threadid, &tattr, null_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auddbg("ERROR: pthread_create failed: %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "null audio");
      audvdbg("Created worker thread\n");
    }

  audvdbg("Return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: null_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int null_stop(FAR struct audio_lowerhalf_s *dev, FAR void* session)
#else
static int null_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct null_dev_s *priv = (FAR struct null_dev_s *)dev;
  struct audio_msg_s term_msg;
  FAR void *value;

  /* Send a message to stop all audio streaming */
  /* REVISIT: There should be a check to see if the worker thread is still
   * running.
   */

  term_msg.msgId = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  mq_send(priv->mq, (FAR const char *)&term_msg, sizeof(term_msg),
          CONFIG_AUDIO_NULL_MSG_PRIO);

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

  audvdbg("Return OK\n");
  return OK;
}
#endif

/****************************************************************************
 * Name: null_pause
 *
 * Description: Pauses the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int null_pause(FAR struct audio_lowerhalf_s *dev, FAR void* session)
#else
static int null_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  audvdbg("Return OK\n");
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: null_resume
 *
 * Description: Resumes the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int null_resume(FAR struct audio_lowerhalf_s *dev, FAR void* session)
#else
static int null_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  audvdbg("Return OK\n");
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: null_enqueuebuffer
 *
 * Description: Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 ****************************************************************************/

static int null_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  FAR struct null_dev_s *priv = (FAR struct null_dev_s *)dev;
  bool done;

  audvdbg("apb=%p curbyte=%d nbytes=%d\n", apb, apb->curbyte, apb->nbytes);

  /* Say that we consumed all of the data */

  apb->curbyte = apb->nbytes;

  /* Check if this was the last buffer in the stream */

  done = ((apb->flags & AUDIO_APB_FINAL) != 0);

  /* And return the buffer to the upper level */

  DEBUGASSERT(priv && apb && priv->dev.upper);

  /* The buffer belongs to to an upper level.  Just forward the event to
   * the next level up.
   */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif

  /* Say we are done playing if this was the last buffer in the stream */

  if (done)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif
    }

  audvdbg("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: null_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/

static int null_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                             FAR struct ap_buffer_s *apb)
{
  audvdbg("apb=%p curbyte=%d nbytes=%d, return OK\n",
          apb, apb->curbyte, apb->nbytes);

  return OK;
}

/****************************************************************************
 * Name: null_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/

static int null_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                        unsigned long arg)
{
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  FAR struct ap_buffer_info_s *bufinfo;
#endif

  audvdbg("cmd=%d arg=%ld\n");

  /* Deal with ioctls passed from the upper-half driver */

  switch (cmd)
    {
      /* Check for AUDIOIOC_HWRESET ioctl.  This ioctl is passed straight
       * through from the upper-half audio driver.
       */

      case AUDIOIOC_HWRESET:
        {
          audvdbg("AUDIOIOC_HWRESET:\n");
        }
        break;

       /* Report our preferred buffer size and quantity */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
      case AUDIOIOC_GETBUFFERINFO:
        {
          audvdbg("AUDIOIOC_GETBUFFERINFO:\n");
          bufinfo              = (FAR struct ap_buffer_info_s *) arg;
          bufinfo->buffer_size = CONFIG_AUDIO_NULL_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_AUDIO_NULL_NUM_BUFFERS;
        }
        break;
#endif

      default:
        break;
    }

  audvdbg("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: null_reserve
 *
 * Description: Reserves a session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int null_reserve(FAR struct audio_lowerhalf_s *dev,
                          FAR void **session)
#else
static int null_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  audvdbg("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: null_release
 *
 * Description: Releases the session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int null_release(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session)
#else
static int null_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct null_dev_s *priv = (FAR struct null_dev_s *)dev;
  void  *value;

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
      priv->threadid = 0;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: audio_null_initialize
 *
 * Description:
 *   Initialize the null audio device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   i2s     - An I2S driver instance
 *   lower   - Persistent board configuration data
 *
 * Returned Value:
 *   A new lower half audio interface for the NULL audio device is returned
 *   on success; NULL is returned on failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *audio_null_initialize(void)
{
  FAR struct null_dev_s *priv;

  /* Allocate the null audio device structure */

  priv = (FAR struct null_dev_s *)kmm_zalloc(sizeof(struct null_dev_s));
  if (priv)
    {
      /* Initialize the null audio device structure.  Since we used kmm_zalloc,
       * only the non-zero elements of the structure need to be initialized.
       */

      priv->dev.ops = &g_audioops;
      return &priv->dev;
    }

  auddbg("ERROR: Failed to allocate null audio device\n");
  return NULL;
}
