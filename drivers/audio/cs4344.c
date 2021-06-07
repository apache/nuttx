/****************************************************************************
 * drivers/audio/cs4344.c
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
#include <sys/ioctl.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <fixedmath.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/cs4344.h>

#include "cs4344.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  cs4344_takesem(FAR sem_t *sem);
static int  cs4344_forcetake(FAR sem_t *sem);
#define     cs4344_givesem(s) nxsem_post(s)

static void cs4344_setdatawidth(FAR struct cs4344_dev_s *priv);
static void cs4344_setbitrate(FAR struct cs4344_dev_s *priv);

/* Audio lower half methods (and close friends) */

static int  cs4344_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                            FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  cs4344_configure(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session,
                              FAR const struct audio_caps_s *caps);
#else
static int  cs4344_configure(FAR struct audio_lowerhalf_s *dev,
                              FAR const struct audio_caps_s *caps);
#endif
static int  cs4344_shutdown(FAR struct audio_lowerhalf_s *dev);
static void cs4344_senddone(FAR struct i2s_dev_s *i2s,
                             FAR struct ap_buffer_s *apb, FAR void *arg,
                             int result);
static void cs4344_returnbuffers(FAR struct cs4344_dev_s *priv);
static int  cs4344_sendbuffer(FAR struct cs4344_dev_s *priv);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  cs4344_start(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session);
#else
static int  cs4344_start(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  cs4344_stop(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session);
#else
static int  cs4344_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  cs4344_pause(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session);
static int  cs4344_resume(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session);
#else
static int  cs4344_pause(FAR struct audio_lowerhalf_s *dev);
static int  cs4344_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int  cs4344_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                  FAR struct ap_buffer_s *apb);
static int  cs4344_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb);
static int  cs4344_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                          unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  cs4344_reserve(FAR struct audio_lowerhalf_s *dev,
                           FAR void **session);
#else
static int  cs4344_reserve(FAR struct audio_lowerhalf_s *dev);
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  cs4344_release(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session);
#else
static int  cs4344_release(FAR struct audio_lowerhalf_s *dev);
#endif

static void *cs4344_workerthread(pthread_addr_t pvarg);

/* Initialization */

static void cs4344_reset(FAR struct cs4344_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audioops =
{
  cs4344_getcaps,       /* getcaps */
  cs4344_configure,     /* configure */
  cs4344_shutdown,      /* shutdown */
  cs4344_start,         /* start */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  cs4344_stop,          /* stop */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  cs4344_pause,         /* pause */
  cs4344_resume,        /* resume */
#endif
  NULL,                 /* allocbuffer */
  NULL,                 /* freebuffer */
  cs4344_enqueuebuffer, /* enqueue_buffer */
  cs4344_cancelbuffer,  /* cancel_buffer */
  cs4344_ioctl,         /* ioctl */
  NULL,                 /* read */
  NULL,                 /* write */
  cs4344_reserve,       /* reserve */
  cs4344_release        /* release */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs4344_takesem
 *
 * Description:
 *  Take a semaphore count, handling the nasty EINTR return if we are
 *  interrupted by a signal.
 *
 ****************************************************************************/

static int cs4344_takesem(FAR sem_t *sem)
{
  return nxsem_wait_uninterruptible(sem);
}

/****************************************************************************
 * Name: cs4344_forcetake
 *
 * Description:
 *   This is just another wrapper but this one continues even if the thread
 *   is canceled.  This must be done in certain conditions where were must
 *   continue in order to clean-up resources.
 *
 ****************************************************************************/

static int cs4344_forcetake(FAR sem_t *sem)
{
  int result;
  int ret = OK;

  do
    {
      result = nxsem_wait_uninterruptible(sem);

      /* The only expected error would -ECANCELED meaning that the
       * parent thread has been canceled.  We have to continue and
       * terminate the poll in this case.
       */

      DEBUGASSERT(result == OK || result == -ECANCELED);
      if (ret == OK && result < 0)
        {
          /* Remember the first failure */

          ret = result;
        }
    }
  while (result < 0);

  return ret;
}

/****************************************************************************
 * Name: cs4344_setdatawidth
 *
 * Description:
 *   Set the 8- or 16-bit data modes
 *
 ****************************************************************************/

static void cs4344_setdatawidth(FAR struct cs4344_dev_s *priv)
{
  if (priv->bpsamp == 16)
    {
      /* Reset default default setting */

      priv->i2s->ops->i2s_txdatawidth(priv->i2s, 16);
    }
  else
    {
      /* This should select 8-bit with no companding */

      priv->i2s->ops->i2s_txdatawidth(priv->i2s, 8);
    }
}

/****************************************************************************
 * Name: cs4344_setbitrate
 *
 ****************************************************************************/

static void cs4344_setbitrate(FAR struct cs4344_dev_s *priv)
{
  DEBUGASSERT(priv);

  priv->i2s->ops->i2s_txsamplerate(priv->i2s, priv->samprate);

  audinfo("sample rate=%u nchannels=%u bpsamp=%u\n",
          priv->samprate, priv->nchannels, priv->bpsamp);
}

/****************************************************************************
 * Name: cs4344_getcaps
 *
 * Description:
 *   Get the audio device capabilities
 *
 ****************************************************************************/

static int cs4344_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                           FAR struct audio_caps_s *caps)
{
  /* Validate the structure */

  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));
  audinfo("type=%d ac_type=%d\n", type, caps->ac_type);

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

              caps->ac_controls.b[0] =
                AUDIO_TYPE_OUTPUT | AUDIO_TYPE_FEATURE |
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

              caps->ac_controls.b[0] =
                AUDIO_SAMP_RATE_8K | AUDIO_SAMP_RATE_11K |
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

        /* If the sub-type is UNDEF, then report the Feature Units we
         * support.
         */

        if (caps->ac_subtype == AUDIO_FU_UNDEF)
          {
            /* Fill in the ac_controls section with the Feature Units we
             * have.
             */

            caps->ac_controls.b[0] = AUDIO_FU_VOLUME | AUDIO_FU_BASS |
                                     AUDIO_FU_TREBLE;
            caps->ac_controls.b[1] = AUDIO_FU_BALANCE >> 8;
          }
        else
          {
            /* TODO:  Do we need to provide specific info for the Feature
             * Units, such as volume setting ranges, etc.?
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

              caps->ac_controls.b[0] =
                AUDIO_STEXT_ENABLE | AUDIO_STEXT_WIDTH;
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

  return caps->ac_len;
}

/****************************************************************************
 * Name: cs4344_configure
 *
 * Description:
 *   Configure the audio device for the specified  mode of operation.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
cs4344_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session, FAR const struct audio_caps_s *caps)
#else
static int
cs4344_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct cs4344_dev_s *priv = (FAR struct cs4344_dev_s *)dev;
  int ret = OK;

  DEBUGASSERT(priv != NULL && caps != NULL);
  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
    case AUDIO_TYPE_FEATURE:
      audinfo("  AUDIO_TYPE_FEATURE\n");

      /* Process based on Feature Unit */

      switch (caps->ac_format.hw)
        {
        default:
          auderr("    ERROR: Unrecognized feature unit\n");
          ret = -ENOTTY;
          break;
        }
        break;

    case AUDIO_TYPE_OUTPUT:
      {
        audinfo("  AUDIO_TYPE_OUTPUT:\n");
        audinfo("    Number of channels: %u\n", caps->ac_channels);
        audinfo("    Sample rate:        %u\n", caps->ac_controls.hw[0]);
        audinfo("    Sample width:       %u\n", caps->ac_controls.b[2]);

        /* Verify that all of the requested values are supported */

        ret = -ERANGE;
        if (caps->ac_channels != 1 && caps->ac_channels != 2)
          {
            auderr("ERROR: Unsupported number of channels: %d\n",
                   caps->ac_channels);
            break;
          }

        if (caps->ac_controls.b[2] != 8 && caps->ac_controls.b[2] != 16)
          {
            auderr("ERROR: Unsupported bits per sample: %d\n",
                   caps->ac_controls.b[2]);
            break;
          }

        /* Save the current stream configuration */

        priv->samprate  = caps->ac_controls.hw[0];
        priv->nchannels = caps->ac_channels;
        priv->bpsamp    = caps->ac_controls.b[2];

        /* Reconfigure the FLL to support the resulting number or channels,
         * bits per sample, and bitrate.
         */

        cs4344_setdatawidth(priv);
        cs4344_setbitrate(priv);
        ret = OK;
      }
      break;

    case AUDIO_TYPE_PROCESSING:
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: cs4344_shutdown
 *
 * Description:
 *   Shutdown the CS4344 chip and put it in the lowest power state possible.
 *
 ****************************************************************************/

static int cs4344_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct cs4344_dev_s *priv = (FAR struct cs4344_dev_s *)dev;

  DEBUGASSERT(priv);

  /* Now issue a software reset. This puts all CS4344 registers back in
   * their default state.
   */

  cs4344_reset(priv);
  return OK;
}

/****************************************************************************
 * Name: cs4344_senddone
 *
 * Description:
 *   This is the I2S callback function that is invoked when the transfer
 *   completes.
 *
 ****************************************************************************/

static void
cs4344_senddone(FAR struct i2s_dev_s *i2s,
                 FAR struct ap_buffer_s *apb, FAR void *arg, int result)
{
  FAR struct cs4344_dev_s *priv = (FAR struct cs4344_dev_s *)arg;
  struct audio_msg_s msg;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(i2s && priv && priv->running && apb);
  audinfo("apb=%p inflight=%d result=%d\n", apb, priv->inflight, result);

  /* We do not place any restriction on the context in which this function
   * is called.  It may be called from an interrupt handler.  Therefore, the
   * doneq and in-flight values might be accessed from the interrupt level.
   * Not the best design.  But we will use interrupt controls to protect
   * against that possibility.
   */

  flags = enter_critical_section();

  /* Add the completed buffer to the end of our doneq.  We do not yet
   * decrement the reference count.
   */

  dq_addlast((FAR dq_entry_t *)apb, &priv->doneq);

  /* And decrement the number of buffers in-flight */

  DEBUGASSERT(priv->inflight > 0);
  priv->inflight--;

  /* Save the result of the transfer */

  /* REVISIT:  This can be overwritten */

  priv->result = result;
  leave_critical_section(flags);

  /* Now send a message to the worker thread, informing it that there are
   * buffers in the done queue that need to be cleaned up.
   */

  msg.msg_id = AUDIO_MSG_COMPLETE;
  ret = file_mq_send(&priv->mq, (FAR const char *)&msg, sizeof(msg),
                     CONFIG_CS4344_MSG_PRIO);
  if (ret < 0)
    {
      auderr("ERROR: file_mq_send failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: cs4344_returnbuffers
 *
 * Description:
 *   This function is called after the complete of one or more data
 *   transfers.  This function will empty the done queue and release our
 *   reference to each buffer.
 *
 ****************************************************************************/

static void cs4344_returnbuffers(FAR struct cs4344_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;

  /* The doneq and in-flight values might be accessed from the interrupt
   * level in some implementations.  Not the best design.  But we will
   * use interrupt controls to protect against that possibility.
   */

  flags = enter_critical_section();
  while (dq_peek(&priv->doneq) != NULL)
    {
      /* Take the next buffer from the queue of completed transfers */

      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->doneq);
      leave_critical_section(flags);

      audinfo("Returning: apb=%p curbyte=%d nbytes=%d flags=%04x\n",
              apb, apb->curbyte, apb->nbytes, apb->flags);

      /* Are we returning the final buffer in the stream? */

      if ((apb->flags & AUDIO_APB_FINAL) != 0)
        {
          /* Both the pending and the done queues should be empty and there
           * should be no buffers in-flight.
           */

          DEBUGASSERT(dq_empty(&priv->doneq) && dq_empty(&priv->pendq) &&
                      priv->inflight == 0);

          /* Set the terminating flag.  This will, eventually, cause the
           * worker thread to exit (if it is not already terminating).
           */

          audinfo("Terminating\n");
          priv->terminating = true;
        }

      /* Release our reference to the audio buffer */

      apb_free(apb);

      /* Send the buffer back up to the previous level. */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif
      flags = enter_critical_section();
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: cs4344_sendbuffer
 *
 * Description:
 *   Start the transfer an audio buffer to the CS4344 via I2S.  This
 *   will not wait for the transfer to complete but will return immediately.
 *   the wmd8904_senddone called will be invoked when the transfer
 *   completes, stimulating the worker thread to call this function again.
 *
 ****************************************************************************/

static int cs4344_sendbuffer(FAR struct cs4344_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;
  uint32_t timeout;
  int shift;
  int ret;

  /* Loop while there are audio buffers to be sent and we have few than
   * CONFIG_CS4344_INFLIGHT then "in-flight"
   *
   * The 'inflight' value might be modified from the interrupt level in some
   * implementations.  We will use interrupt controls to protect against
   * that possibility.
   *
   * The 'pendq', on the other hand, is protected via a semaphore.  Let's
   * hold the semaphore while we are busy here and disable the interrupts
   * only while accessing 'inflight'.
   */

  ret = cs4344_takesem(&priv->pendsem);
  if (ret < 0)
    {
      return ret;
    }

  while (priv->inflight < CONFIG_CS4344_INFLIGHT &&
         dq_peek(&priv->pendq) != NULL && !priv->paused)
    {
      /* Take next buffer from the queue of pending transfers */

      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq);
      audinfo("Sending apb=%p, size=%d inflight=%d\n",
              apb, apb->nbytes, priv->inflight);

      /* Increment the number of buffers in-flight before sending in order
       * to avoid a possible race condition.
       */

      flags = enter_critical_section();
      priv->inflight++;
      leave_critical_section(flags);

      /* Send the entire audio buffer via I2S.  What is a reasonable timeout
       * to use?  This would depend on the bit rate and size of the buffer.
       *
       * Samples in the buffer (samples):
       *   = buffer_size * 8 / bpsamp                           samples
       * Sample rate (samples/second):
       *   = samplerate * nchannels
       * Expected transfer time (seconds):
       *   = (buffer_size * 8) / bpsamp / samplerate / nchannels
       *
       * We will set the timeout about twice that.
       *
       * NOTES:
       * - The multiplier of 8 becomes 16000 for 2x and units of
       *   milliseconds.
       * - 16000 is a approximately 16384 (1 << 14), bpsamp is either
       *   (1 << 3) or (1 << 4), and nchannels is either (1 << 0) or
       *   (1 << 1).  So this can be simplifies to (milliseconds):
       *
       *   = (buffer_size << shift) / samplerate
       */

      shift  = (priv->bpsamp == 8) ? 14 - 3 : 14 - 4;
      shift -= (priv->nchannels > 1) ? 1 : 0;

      timeout = MSEC2TICK(((uint32_t)(apb->nbytes - apb->curbyte) << shift) /
                           (uint32_t)priv->samprate);

      ret = I2S_SEND(priv->i2s, apb, cs4344_senddone, priv, timeout);
      if (ret < 0)
        {
          auderr("ERROR: I2S_SEND failed: %d\n", ret);
          break;
        }
    }

  cs4344_givesem(&priv->pendsem);
  return ret;
}

/****************************************************************************
 * Name: cs4344_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int cs4344_start(FAR struct audio_lowerhalf_s *dev,
           FAR void *session)
#else
static int cs4344_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct cs4344_dev_s *priv = (FAR struct cs4344_dev_s *)dev;
  struct sched_param sparam;
  struct mq_attr attr;
  pthread_attr_t tattr;
  FAR void *value;
  int ret;

  audinfo("Entry\n");

  /* Create a message queue for the worker thread */

  snprintf(priv->mqname, sizeof(priv->mqname), "/tmp/%" PRIXPTR,
           (uintptr_t)priv);

  attr.mq_maxmsg  = 16;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  ret = file_mq_open(&priv->mq, priv->mqname,
                     O_RDWR | O_CREAT, 0644, &attr);
  if (ret < 0)
    {
      /* Error creating message queue! */

      auderr("ERROR: Couldn't allocate message queue\n");
      return ret;
    }

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      audinfo("Joining old thread\n");
      pthread_join(priv->threadid, &value);
    }

  /* Start our thread for sending data to the device */

  pthread_attr_init(&tattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  pthread_attr_setschedparam(&tattr, &sparam);
  pthread_attr_setstacksize(&tattr, CONFIG_CS4344_WORKER_STACKSIZE);

  audinfo("Starting worker thread\n");
  ret = pthread_create(&priv->threadid, &tattr, cs4344_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("ERROR: pthread_create failed: %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "cs4344");
      audinfo("Created worker thread\n");
    }

  return ret;
}

/****************************************************************************
 * Name: cs4344_stop
 *
 * Description:
 *   Stop the configured operation (audio streaming, volume disabled, etc.).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int cs4344_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#  else
static int cs4344_stop(FAR struct audio_lowerhalf_s *dev)
#  endif
{
  FAR struct cs4344_dev_s *priv = (FAR struct cs4344_dev_s *)dev;
  struct audio_msg_s term_msg;
  FAR void *value;

  /* Send a message to stop all audio streaming */

  term_msg.msg_id = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  file_mq_send(&priv->mq, (FAR const char *)&term_msg, sizeof(term_msg),
               CONFIG_CS4344_MSG_PRIO);

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

  return OK;
}
#endif

/****************************************************************************
 * Name: cs4344_pause
 *
 * Description:
 *   Pauses the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int cs4344_pause(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session)
#  else
static int cs4344_pause(FAR struct audio_lowerhalf_s *dev)
#  endif
{
  FAR struct cs4344_dev_s *priv = (FAR struct cs4344_dev_s *)dev;

  if (priv->running && !priv->paused)
    {
      /* Disable interrupts to prevent us from suppling any more data */

      priv->paused = true;
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: cs4344_resume
 *
 * Description:
 *   Resumes the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int cs4344_resume(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session)
#  else
static int cs4344_resume(FAR struct audio_lowerhalf_s *dev)
#  endif
{
  FAR struct cs4344_dev_s *priv = (FAR struct cs4344_dev_s *)dev;

  if (priv->running && priv->paused)
    {
      priv->paused = false;

      /* Enable interrupts to allow sampling data */

      cs4344_sendbuffer(priv);
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: cs4344_enqueuebuffer
 *
 * Description:
 *   Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 ****************************************************************************/

static int cs4344_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb)
{
  FAR struct cs4344_dev_s *priv = (FAR struct cs4344_dev_s *)dev;
  struct audio_msg_s term_msg;
  int ret;

  audinfo("Enqueueing: apb=%p curbyte=%d nbytes=%d flags=%04x\n",
          apb, apb->curbyte, apb->nbytes, apb->flags);

  ret = cs4344_takesem(&priv->pendsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Take a reference on the new audio buffer */

  apb_reference(apb);

  /* Add the new buffer to the tail of pending audio buffers */

  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;
  dq_addlast(&apb->dq_entry, &priv->pendq);
  cs4344_givesem(&priv->pendsem);

  /* Send a message to the worker thread indicating that a new buffer has
   * been enqueued.  If mq is NULL, then the playing has not yet started.
   * In that case we are just "priming the pump" and we don't need to send
   * any message.
   */

  ret = OK;
  if (priv->mq.f_inode != NULL)
    {
      term_msg.msg_id  = AUDIO_MSG_ENQUEUE;
      term_msg.u.data = 0;

      ret = file_mq_send(&priv->mq, (FAR const char *)&term_msg,
                         sizeof(term_msg), CONFIG_CS4344_MSG_PRIO);
      if (ret < 0)
        {
          auderr("ERROR: file_mq_send failed: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: cs4344_cancelbuffer
 *
 * Description:
 *   Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/

static int cs4344_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  audinfo("apb=%p\n", apb);
  return OK;
}

/****************************************************************************
 * Name: cs4344_ioctl
 *
 * Description:
 *   Perform a device ioctl
 *
 ****************************************************************************/

static int cs4344_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                         unsigned long arg)
{
  int ret = OK;
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  FAR struct ap_buffer_info_s *bufinfo;
#endif

  /* Deal with ioctls passed from the upper-half driver */

  switch (cmd)
    {
      /* Check for AUDIOIOC_HWRESET ioctl.  This ioctl is passed straight
       * through from the upper-half audio driver.
       */

      case AUDIOIOC_HWRESET:
        {
          audinfo("AUDIOIOC_HWRESET:\n");
        }
        break;

       /* Report our preferred buffer size and quantity */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
      case AUDIOIOC_GETBUFFERINFO:
        {
          audinfo("AUDIOIOC_GETBUFFERINFO:\n");
          bufinfo              = (FAR struct ap_buffer_info_s *)arg;
          bufinfo->buffer_size = CONFIG_CS4344_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_CS4344_NUM_BUFFERS;
        }
        break;
#endif

      default:
        ret = -ENOTTY;
        audinfo("Ignored\n");
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: cs4344_reserve
 *
 * Description:
 *   Reserves a session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
cs4344_reserve(FAR struct audio_lowerhalf_s *dev, FAR void **session)
#else
static int cs4344_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct cs4344_dev_s *priv = (FAR struct cs4344_dev_s *)dev;
  int ret = OK;

  /* Borrow the APBQ semaphore for thread sync */

  ret = cs4344_takesem(&priv->pendsem);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->reserved)
    {
      ret = -EBUSY;
    }
  else
    {
      /* Initialize the session context */

#ifdef CONFIG_AUDIO_MULTI_SESSION
     *session           = NULL;
#endif
      priv->inflight    = 0;
      priv->running     = false;
      priv->paused      = false;
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
      priv->terminating = false;
#endif
      priv->reserved    = true;
    }

  cs4344_givesem(&priv->pendsem);

  return ret;
}

/****************************************************************************
 * Name: cs4344_release
 *
 * Description:
 *   Releases the session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int cs4344_release(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session)
#else
static int cs4344_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct cs4344_dev_s *priv = (FAR struct cs4344_dev_s *)dev;
  FAR void *value;
  int ret;

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
      priv->threadid = 0;
    }

  /* Borrow the APBQ semaphore for thread sync */

  ret = cs4344_forcetake(&priv->pendsem);

  /* Really we should free any queued buffers here */

  priv->reserved = false;
  cs4344_givesem(&priv->pendsem);

  return ret;
}

/****************************************************************************
 * Name: cs4344_workerthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 *
 ****************************************************************************/

static void *cs4344_workerthread(pthread_addr_t pvarg)
{
  FAR struct cs4344_dev_s *priv = (struct cs4344_dev_s *)pvarg;
  struct audio_msg_s msg;
  FAR struct ap_buffer_s *apb;
  int msglen;
  unsigned int prio;

  audinfo("Entry\n");

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  priv->terminating = false;
#endif

/* Mark ourself as running and make sure that CS4344 interrupts are
 * enabled.
 */

  priv->running = true;

  /* Loop as long as we are supposed to be running and as long as we have
   * buffers in-flight.
   */

  while (priv->running || priv->inflight > 0)
    {
      /* Check if we have been asked to terminate.  We have to check if we
       * still have buffers in-flight.  If we do, then we can't stop until
       * birds come back to roost.
       */

      if (priv->terminating && priv->inflight <= 0)
        {
          /* We are IDLE.  Break out of the loop and exit. */

          break;
        }
      else
        {
          /* Check if we can send more audio buffers to the CS4344 */

          cs4344_sendbuffer(priv);
        }

      /* Wait for messages from our message queue */

      msglen = file_mq_receive(&priv->mq, (FAR char *)&msg,
                               sizeof(msg), &prio);

      /* Handle the case when we return with no message */

      if (msglen < sizeof(struct audio_msg_s))
        {
          auderr("ERROR: Message too small: %d\n", msglen);
          continue;
        }

      /* Process the message */

      switch (msg.msg_id)
        {
          /* The ISR has requested more data.  We will catch this case at
           * the top of the loop.
           */

          case AUDIO_MSG_DATA_REQUEST:
            audinfo("AUDIO_MSG_DATA_REQUEST\n");
            break;

          /* Stop the playback */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
          case AUDIO_MSG_STOP:

            /* Indicate that we are terminating */

            audinfo("AUDIO_MSG_STOP: Terminating\n");
            priv->terminating = true;
            break;
#endif

          /* We have a new buffer to send.  We will catch this case at
           * the top of the loop.
           */

          case AUDIO_MSG_ENQUEUE:
            audinfo("AUDIO_MSG_ENQUEUE\n");
            break;

          /* We will wake up from the I2S callback with this message */

          case AUDIO_MSG_COMPLETE:
            audinfo("AUDIO_MSG_COMPLETE\n");
            cs4344_returnbuffers(priv);
            break;

          default:
            auderr("ERROR: Ignoring message ID %d\n", msg.msg_id);
            break;
        }
    }

  /* Reset the CS4344 hardware */

  cs4344_reset(priv);

  /* Return any pending buffers in our pending queue */

  cs4344_forcetake(&priv->pendsem);
  while ((apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq)) != NULL)
    {
      /* Release our reference to the buffer */

      apb_free(apb);

      /* Send the buffer back up to the previous level. */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif
    }

  cs4344_givesem(&priv->pendsem);

  /* Return any pending buffers in our done queue */

  cs4344_returnbuffers(priv);

  /* Close the message queue */

  file_mq_close(&priv->mq);
  file_mq_unlink(priv->mqname);

  /* Send an AUDIO_MSG_COMPLETE message to the client */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  audinfo("Exit\n");
  return NULL;
}

/****************************************************************************
 * Name: cs4344_reset
 *
 * Description:
 *   Reset and re-initialize the CS4344
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void cs4344_reset(FAR struct cs4344_dev_s *priv)
{
  /* Put audio output back to its initial configuration */

  priv->samprate   = CS4344_DEFAULT_SAMPRATE;
  priv->nchannels  = CS4344_DEFAULT_NCHANNELS;
  priv->bpsamp     = CS4344_DEFAULT_BPSAMP;

  /* Configure the FLL and the LRCLK */

  cs4344_setbitrate(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs4344_initialize
 *
 * Description:
 *   Initialize the CS4344 device.
 *
 * Input Parameters:
 *   i2s     - An I2S driver instance
 *
 * Returned Value:
 *   A new lower half audio interface for the CS4344 device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *cs4344_initialize(FAR struct i2s_dev_s *i2s)
{
  FAR struct cs4344_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(i2s);

  /* Allocate a CS4344 device structure */

  priv = (FAR struct cs4344_dev_s *) kmm_zalloc(sizeof(struct cs4344_dev_s));
  if (priv)
    {
      /* Initialize the CS4344 device structure.  Since we used kmm_zalloc,
       * only the non-zero elements of the structure need to be initialized.
       */

      priv->dev.ops    = &g_audioops;
      priv->i2s        = i2s;

      nxsem_init(&priv->pendsem, 0, 1);
      dq_init(&priv->pendq);
      dq_init(&priv->doneq);

      /* Reset and reconfigure the CS4344 hardware */

      cs4344_reset(priv);
      return &priv->dev;
    }

  nxsem_destroy(&priv->pendsem);
  kmm_free(priv);
  return NULL;
}
