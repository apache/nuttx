/****************************************************************************
 * drivers/audio/wm8776.c
 *
 *   Copyright 2017, 2018 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *
 * Based on drivers/audio/wm8904.c
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
#include <fixedmath.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/wm8776.h>

#include "wm8776.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

static void     wm8776_writereg(FAR struct wm8776_dev_s *priv,
                  uint8_t regaddr, uint16_t regval);

static void     wm8776_takesem(sem_t *sem);
#define         wm8776_givesem(s) nxsem_post(s)

static int      wm8776_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                  FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8776_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session, FAR const struct audio_caps_s *caps);
#else
static int      wm8776_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR const struct audio_caps_s *caps);
#endif
static int      wm8776_shutdown(FAR struct audio_lowerhalf_s *dev);
static void     wm8776_senddone(FAR struct i2s_dev_s *i2s,
                  FAR struct ap_buffer_s *apb, FAR void *arg, int result);
static void     wm8776_returnbuffers(FAR struct wm8776_dev_s *priv);
static int      wm8776_sendbuffer(FAR struct wm8776_dev_s *priv);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8776_start(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      wm8776_start(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8776_stop(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      wm8776_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8776_pause(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
static int      wm8776_resume(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      wm8776_pause(FAR struct audio_lowerhalf_s *dev);
static int      wm8776_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int      wm8776_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                  FAR struct ap_buffer_s *apb);
static int      wm8776_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                  FAR struct ap_buffer_s *apb);
static int      wm8776_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                  unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8776_reserve(FAR struct audio_lowerhalf_s *dev,
                  FAR void **session);
#else
static int      wm8776_reserve(FAR struct audio_lowerhalf_s *dev);
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8776_release(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      wm8776_release(FAR struct audio_lowerhalf_s *dev);
#endif

static void    *wm8776_workerthread(pthread_addr_t pvarg);

/* Initialization */

static void     wm8776_audio_output(FAR struct wm8776_dev_s *priv);
#if 0 /* Not used */
static void     wm8776_audio_input(FAR struct wm8776_dev_s *priv);
#endif
static void     wm8776_hw_reset(FAR struct wm8776_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audioops =
{
  wm8776_getcaps,       /* getcaps        */
  wm8776_configure,     /* configure      */
  wm8776_shutdown,      /* shutdown       */
  wm8776_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  wm8776_stop,          /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  wm8776_pause,         /* pause          */
  wm8776_resume,        /* resume         */
#endif
  NULL,                 /* allocbuffer    */
  NULL,                 /* freebuffer     */
  wm8776_enqueuebuffer, /* enqueue_buffer */
  wm8776_cancelbuffer,  /* cancel_buffer  */
  wm8776_ioctl,         /* ioctl          */
  NULL,                 /* read           */
  NULL,                 /* write          */
  wm8776_reserve,       /* reserve        */
  wm8776_release        /* release        */
};

/************************************************************************************
 * Name: wm8776_writereg
 *
 * Description:
 *   Write the specified 16-bit register to the WM8776 device.
 *
 ************************************************************************************/

static void wm8776_writereg(FAR struct wm8776_dev_s *priv,
                            uint8_t regaddr,
                            uint16_t regval)
{
  struct i2c_config_s config;
  uint8_t data[2];
  int ret;

  /* Setup up the I2C configuration */

  config.frequency = priv->lower->frequency;
  config.address   = priv->lower->address;
  config.addrlen   = 7;

  /* Set up the data to write */

  data[0] = (regaddr << 1) + ((regval >> 8) & 0x1);
  data[1] = (regval & 0xff);

  ret = i2c_write(priv->i2c, &config, data, sizeof(data));
  if (ret < 0)
    {
      auderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }
}

/************************************************************************************
 * Name: wm8776_takesem
 *
 * Description:
 *  Take a semaphore count, handling the nasty EINTR return if we are interrupted
 *  by a signal.
 *
 ************************************************************************************/

static void wm8776_takesem(sem_t *sem)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(sem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/************************************************************************************
 * Name: wm8776_setvolume
 *
 * Description:
 *   Set the right and left volume values in the WM8776 device based on the current
 *   volume and balance settings.
 *
 ************************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static void wm8776_setvolume(FAR struct wm8776_dev_s *priv, uint16_t volume,
                             bool mute)
{
  uint16_t regval;
  uint16_t tmp_vol;

  /* TODO: balance */

  if (mute)
    {
      tmp_vol = 0;
    }
  else
    {
      tmp_vol = volume;
    }

  /* limit the max vol */

  if (tmp_vol > 0x69)
    {
      tmp_vol = 0x69; /* -10db */
    }

  regval = WM8776_UPDATE | WM8776_HPOUT_VOL(tmp_vol);

  wm8776_writereg(priv, WM8776_MASTER_ATT, regval);

  audinfo("volume=%d mute=%d tmp_vol=%d (regval=0x%x) \n",
          volume, mute, tmp_vol, regval);

  /* Remember the volume level and mute settings */

  priv->volume = volume;
  priv->mute   = mute;
}
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

/****************************************************************************
 * Name: wm8776_getcaps
 *
 * Description:
 *   Get the audio device capabilities
 *
 ****************************************************************************/

static int wm8776_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
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

              caps->ac_controls.b[0] = AUDIO_TYPE_OUTPUT | AUDIO_TYPE_FEATURE |
                                     AUDIO_TYPE_PROCESSING;

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

              caps->ac_controls.b[0] = AUDIO_SAMP_RATE_44K;
              break;

            case AUDIO_FMT_MP3:
            case AUDIO_FMT_WMA:
            case AUDIO_FMT_PCM:
            default:
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
 * Name: wm8776_configure
 *
 * Description:
 *   Configure the audio device for the specified  mode of operation.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8776_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session,
                            FAR const struct audio_caps_s *caps)
#else
static int wm8776_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct wm8776_dev_s *priv = (FAR struct wm8776_dev_s *)dev;
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
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
        case AUDIO_FU_VOLUME:
          {
            /* Set the volume */

            uint16_t volume = caps->ac_controls.hw[0];
            audinfo("    Volume: %d\n", volume);

            if (volume >= 0 && volume <= 1000)
              {
                /* Scale the volume setting to the range {0x2f .. 0x79} */

                wm8776_setvolume(priv, (0x4a * volume / 1000) + 0x2f, priv->mute);
              }
            else
              {
                ret = -EDOM;
              }
           }
          break;
#endif  /* CONFIG_AUDIO_EXCLUDE_VOLUME */

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

        /* TODO : channels, bits per sample, bitrate */

        ret = OK;
      }
      break;

    case AUDIO_TYPE_PROCESSING:
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: wm8776_shutdown
 *
 * Description:
 *   Shutdown the WM8776 chip and put it in the lowest power state possible.
 *
 ****************************************************************************/

static int wm8776_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct wm8776_dev_s *priv = (FAR struct wm8776_dev_s *)dev;

  DEBUGASSERT(priv);

  /* Now issue a software reset.  This puts all WM8776 registers back in
   * their default state.
   */

  wm8776_hw_reset(priv);
  return OK;
}

/****************************************************************************
 * Name: wm8776_senddone
 *
 * Description:
 *   This is the I2S callback function that is invoked when the transfer
 *   completes.
 *
 ****************************************************************************/

static void  wm8776_senddone(FAR struct i2s_dev_s *i2s,
                             FAR struct ap_buffer_s *apb, FAR void *arg,
                             int result)
{
  FAR struct wm8776_dev_s *priv = (FAR struct wm8776_dev_s *)arg;
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

  flags = spin_lock_irqsave();

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
  spin_unlock_irqrestore(flags);

  /* Now send a message to the worker thread, informing it that there are
   * buffers in the done queue that need to be cleaned up.
   */

  msg.msgId = AUDIO_MSG_COMPLETE;
  ret = mq_send(priv->mq, (FAR const char *)&msg, sizeof(msg),
                CONFIG_WM8776_MSG_PRIO);
  if (ret < 0)
    {
      auderr("ERROR: mq_send failed: %d\n", get_errno());
    }
}

/****************************************************************************
 * Name: wm8776_returnbuffers
 *
 * Description:
 *   This function is called after the complete of one or more data
 *   transfers.  This function will empty the done queue and release our
 *   reference to each buffer.
 *
 ****************************************************************************/

static void wm8776_returnbuffers(FAR struct wm8776_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;

  /* The doneq and in-flight values might be accessed from the interrupt
   * level in some implementations.  Not the best design.  But we will
   * use interrupt controls to protect against that possibility.
   */

  flags = spin_lock_irqsave();
  while (dq_peek(&priv->doneq) != NULL)
    {
      /* Take the next buffer from the queue of completed transfers */

      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->doneq);
      spin_unlock_irqrestore(flags);

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
      flags = spin_lock_irqsave();
    }

  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: wm8776_sendbuffer
 *
 * Description:
 *   Start the transfer an audio buffer to the WM8776 via I2S.  This
 *   will not wait for the transfer to complete but will return immediately.
 *   the wmd8776_senddone called will be invoked when the transfer
 *   completes, stimulating the worker thread to call this function again.
 *
 ****************************************************************************/

static int wm8776_sendbuffer(FAR struct wm8776_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;
  uint32_t timeout;
  int shift;
  int ret = OK;

  /* Loop while there are audio buffers to be sent and we have few than
   * CONFIG_WM8776_INFLIGHT then "in-flight"
   *
   * The 'inflight' value might be modified from the interrupt level in some
   * implementations.  We will use interrupt controls to protect against
   * that possibility.
   *
   * The 'pendq', on the other hand, is protected via a semaphore.  Let's
   * hold the semaphore while we are busy here and disable the interrupts
   * only while accessing 'inflight'.
   */

  wm8776_takesem(&priv->pendsem);
  while (priv->inflight < CONFIG_WM8776_INFLIGHT &&
         dq_peek(&priv->pendq) != NULL && !priv->paused)
    {
      /* Take next buffer from the queue of pending transfers */

      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq);
      audinfo("Sending apb=%p, size=%d inflight=%d\n",
              apb, apb->nbytes, priv->inflight);

      /* Increment the number of buffers in-flight before sending in order
       * to avoid a possible race condition.
       */

      flags = spin_lock_irqsave();
      priv->inflight++;
      spin_unlock_irqrestore(flags);

      shift  = (priv->bpsamp == 8) ? 14 - 3 : 14 - 4;
      shift -= (priv->nchannels > 1) ? 1 : 0;

      timeout = MSEC2TICK(((uint32_t)(apb->nbytes - apb->curbyte) << shift) /
                           (uint32_t)priv->samprate);

      ret = I2S_SEND(priv->i2s, apb, wm8776_senddone, priv, timeout);
      if (ret < 0)
        {
          auderr("ERROR: I2S_SEND failed: %d\n", ret);
          break;
        }
    }

  wm8776_givesem(&priv->pendsem);
  return ret;
}

/****************************************************************************
 * Name: wm8776_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8776_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int wm8776_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8776_dev_s *priv = (FAR struct wm8776_dev_s *)dev;
  struct sched_param sparam;
  struct mq_attr attr;
  pthread_attr_t tattr;
  FAR void *value;
  int ret;

  audinfo("Entry\n");

  /* Exit reduced power modes of operation */
  /* REVISIT */

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

      auderr("ERROR: Couldn't allocate message queue\n");
      return -ENOMEM;
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
  (void)pthread_attr_setschedparam(&tattr, &sparam);
  (void)pthread_attr_setstacksize(&tattr, CONFIG_WM8776_WORKER_STACKSIZE);

  audinfo("Starting worker thread\n");
  ret = pthread_create(&priv->threadid, &tattr, wm8776_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("ERROR: pthread_create failed: %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "wm8776");
      audinfo("Created worker thread\n");
    }

  return ret;
}

/****************************************************************************
 * Name: wm8776_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8776_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int wm8776_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8776_dev_s *priv = (FAR struct wm8776_dev_s *)dev;
  struct audio_msg_s term_msg;
  FAR void *value;

  /* Send a message to stop all audio streaming */

  term_msg.msgId = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  mq_send(priv->mq, (FAR const char *)&term_msg, sizeof(term_msg),
          CONFIG_WM8776_MSG_PRIO);

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

  /* Enter into a reduced power usage mode */
  /* REVISIT: */

  return OK;
}
#endif

/****************************************************************************
 * Name: wm8776_pause
 *
 * Description: Pauses the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8776_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int wm8776_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8776_dev_s *priv = (FAR struct wm8776_dev_s *)dev;

  if (priv->running && !priv->paused)
    {
      priv->paused = true;
      wm8776_setvolume(priv, priv->volume, true);
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: wm8776_resume
 *
 * Description: Resumes the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8776_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int wm8776_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8776_dev_s *priv = (FAR struct wm8776_dev_s *)dev;

  if (priv->running && priv->paused)
    {
      priv->paused = false;
      wm8776_setvolume(priv, priv->volume, false);
      wm8776_sendbuffer(priv);
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: wm8776_enqueuebuffer
 *
 * Description: Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 ****************************************************************************/

static int wm8776_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  FAR struct wm8776_dev_s *priv = (FAR struct wm8776_dev_s *)dev;
  struct audio_msg_s  term_msg;
  int ret;

  audinfo("Enqueueing: apb=%p curbyte=%d nbytes=%d flags=%04x\n",
          apb, apb->curbyte, apb->nbytes, apb->flags);

  /* Take a reference on the new audio buffer */

  apb_reference(apb);

  /* Add the new buffer to the tail of pending audio buffers */

  wm8776_takesem(&priv->pendsem);
  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;
  dq_addlast(&apb->dq_entry, &priv->pendq);
  wm8776_givesem(&priv->pendsem);

  /* Send a message to the worker thread indicating that a new buffer has been
   * enqueued.  If mq is NULL, then the playing has not yet started.  In that
   * case we are just "priming the pump" and we don't need to send any message.
   */

  ret = OK;
  if (priv->mq != NULL)
    {
      term_msg.msgId  = AUDIO_MSG_ENQUEUE;
      term_msg.u.data = 0;

      ret = mq_send(priv->mq, (FAR const char *)&term_msg, sizeof(term_msg),
                    CONFIG_WM8776_MSG_PRIO);
      if (ret < 0)
        {
          int errcode = get_errno();
          DEBUGASSERT(errcode > 0);

          auderr("ERROR: mq_send failed: %d\n", errcode);
          UNUSED(errcode);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: wm8776_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/

static int wm8776_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                               FAR struct ap_buffer_s *apb)
{
  audinfo("apb=%p\n", apb);
  return OK;
}

/****************************************************************************
 * Name: wm8776_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/

static int wm8776_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                        unsigned long arg)
{
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
          /* REVISIT:  Should we completely re-initialize the chip?   We
           * can't just issue a software reset; that would puts all WM8776
           * registers back in their default state.
           */

          audinfo("AUDIOIOC_HWRESET:\n");
        }
        break;

       /* Report our preferred buffer size and quantity */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
      case AUDIOIOC_GETBUFFERINFO:
        {
          audinfo("AUDIOIOC_GETBUFFERINFO:\n");
          bufinfo              = (FAR struct ap_buffer_info_s *) arg;
          bufinfo->buffer_size = CONFIG_WM8776_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_WM8776_NUM_BUFFERS;
        }
        break;
#endif

      default:
        audinfo("Ignored\n");
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: wm8776_reserve
 *
 * Description: Reserves a session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8776_reserve(FAR struct audio_lowerhalf_s *dev,
                          FAR void **session)
#else
static int wm8776_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8776_dev_s *priv = (FAR struct wm8776_dev_s *) dev;
  int   ret = OK;

  /* Borrow the APBQ semaphore for thread sync */

  wm8776_takesem(&priv->pendsem);
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

  wm8776_givesem(&priv->pendsem);

  return ret;
}

/****************************************************************************
 * Name: wm8776_release
 *
 * Description: Releases the session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8776_release(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session)
#else
static int wm8776_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8776_dev_s *priv = (FAR struct wm8776_dev_s *)dev;
  void  *value;

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
      priv->threadid = 0;
    }

  /* Borrow the APBQ semaphore for thread sync */

  wm8776_takesem(&priv->pendsem);

  /* Really we should free any queued buffers here */

  priv->reserved = false;
  wm8776_givesem(&priv->pendsem);

  return OK;
}

/****************************************************************************
 * Name: wm8776_audio_output
 *
 * Description:
 *   Initialize and configure the WM8776 device as an audio output device.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None.  No failures are detected.
 *
 ****************************************************************************/

static void wm8776_audio_output(FAR struct wm8776_dev_s *priv)
{
  wm8776_writereg(priv, WM8776_MASTER_ATT, WM8776_UPDATE | 0x58); /* -33db */
  wm8776_writereg(priv, WM8776_DAC_IF, 0x32);  /* 32bit, I2S, standard pol */

#ifdef CONFIG_WM8776_SWAP_HPOUT
  wm8776_writereg(priv, WM8776_DAC_CC, 0x62);  /* Swap HPOUT L/R */
#endif

  wm8776_writereg(priv, WM8776_MASTER_MODE, 0x00); /* slave mode, 128fs */
  wm8776_writereg(priv, WM8776_PWR_DOWN, 0x12);    /* AINPD, ADCPD  */
}

/****************************************************************************
 * Name: wm8776_hw_reset
 *
 * Description:
 *   Reset and re-initialize the WM8776
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wm8776_hw_reset(FAR struct wm8776_dev_s *priv)
{
  /* Put audio output back to its initial configuration */

  priv->samprate   = WM8776_DEFAULT_SAMPRATE;
  priv->nchannels  = WM8776_DEFAULT_NCHANNELS;
  priv->bpsamp     = WM8776_DEFAULT_BPSAMP;
#if !defined(CONFIG_AUDIO_EXCLUDE_VOLUME) && !defined(CONFIG_AUDIO_EXCLUDE_BALANCE)
  priv->balance    = b16HALF;            /* Center balance */
#endif

  /* Software reset.  This puts all WM8776 registers back in their
   * default state.
   */

  wm8776_writereg(priv, WM8776_SOFT_RESET, 0x00);

  /* Configure the WM8776 hardware as an audio input device */

  wm8776_audio_output(priv);

}

/****************************************************************************
 * Name: wm8776_workerthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 *
 ****************************************************************************/

static void *wm8776_workerthread(pthread_addr_t pvarg)
{
  FAR struct wm8776_dev_s *priv = (struct wm8776_dev_s *) pvarg;
  struct audio_msg_s msg;
  FAR struct ap_buffer_s *apb;
  int msglen;
  int prio;
  struct mq_attr attr;

  audinfo("Entry\n");

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  priv->terminating = false;
#endif

  priv->running = true;
  wm8776_setvolume(priv, priv->volume, false);

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
          /* Check if we can send more audio buffers to the WM8776 */

          wm8776_sendbuffer(priv);
        }

repeat:

      /* Wait for messages from our message queue */

      msglen = mq_receive(priv->mq, (FAR char *)&msg, sizeof(msg), &prio);

      /* Handle the case when we return with no message */

      if (msglen < sizeof(struct audio_msg_s))
        {
          auderr("ERROR: Message too small: %d\n", msglen);
          continue;
        }

      /* Process the message */

      switch (msg.msgId)
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
            wm8776_returnbuffers(priv);
            break;

          default:
            auderr("ERROR: Ignoring message ID %d\n", msg.msgId);
            break;
        }

      (void)mq_getattr(priv->mq, &attr);

      /* If there is a message in the queue, process it */

      if (0 < attr.mq_curmsgs)
        {
          goto repeat;
        }

    }

  /* Reset the WM8776 hardware */

  wm8776_hw_reset(priv);

  /* Return any pending buffers in our pending queue */

  wm8776_takesem(&priv->pendsem);
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

  wm8776_givesem(&priv->pendsem);

  /* Return any pending buffers in our done queue */

  wm8776_returnbuffers(priv);

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

  audinfo("Exit\n");
  return NULL;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wm8776_initialize
 *
 * Description:
 *   Initialize the WM8776 device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   i2s     - An I2S driver instance
 *   lower   - Persistent board configuration data
 *
 * Returned Value:
 *   A new lower half audio interface for the WM8776 device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *
  wm8776_initialize(FAR struct i2c_master_s *i2c,
                    FAR struct i2s_dev_s *i2s,
                    FAR const struct wm8776_lower_s *lower)
{
  FAR struct wm8776_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(i2c && i2s && lower);

  /* Allocate a WM8776 device structure */

  priv = (FAR struct wm8776_dev_s *)kmm_zalloc(sizeof(struct wm8776_dev_s));

  if (priv)
    {
      priv->dev.ops    = &g_audioops;
      priv->lower      = lower;
      priv->i2c        = i2c;
      priv->i2s        = i2s;

      nxsem_init(&priv->pendsem, 0, 1);
      dq_init(&priv->pendq);
      dq_init(&priv->doneq);

      /* Reset and reconfigure the WM8776 hardwaqre */

      wm8776_hw_reset(priv);
      return &priv->dev;
    }

  return NULL;
}
