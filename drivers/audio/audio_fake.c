/****************************************************************************
 * drivers/audio/audio_fake.c
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

#include <sys/ioctl.h>
#include <sys/types.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/audio_fake.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>
#include <nuttx/signal.h>
#include <sys/param.h>
#include <sys/time.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct audio_fake_params
{
  const char *dev_name;
  bool       playback;        /* True: playback, False: recording */
  uint32_t   samplerate[4];   /* Array of sample rate,eg. [44100, 48000, 32000, 22050] */
  uint8_t    channels[2];     /* Range of channels, [min_channel, max_channel] */
  uint8_t    format[4];       /* Array of format, eg. [8, 16, 32] */
  uint32_t   period_time;     /* Period time in milliseconds */
  uint32_t   periods;         /* Number of periods */
};

struct audio_fake_s
{
  struct audio_lowerhalf_s dev; /* Audio lower half (this device) */
  bool          playback;       /* True: playback, False: recording */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  volatile bool terminate;      /* True: request to terminate */
#endif
  uint8_t       format;         /* Request audio format */
  uint32_t      channels;       /* Request audio channels */
  uint32_t      sample_rate;    /* Request audio sample rate */
  uint32_t      scaler;         /* Data bytes to sec scaler (bytes per sec) */
  pthread_t     threadid;       /* ID of worker thread */
  char          mqname[16];     /* Our message queue name */
  struct file   mq;             /* Message queue for receiving messages */
  struct file   file;           /* Audio file for playback or capture */
  const struct audio_fake_params *dev_params;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int audio_fake_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                              FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_fake_configure(FAR struct audio_lowerhalf_s *dev,
                                FAR void *session,
                                FAR const struct audio_caps_s *caps);
#else
static int audio_fake_configure(FAR struct audio_lowerhalf_s *dev,
                                FAR const struct audio_caps_s *caps);
#endif
static int audio_fake_shutdown(FAR struct audio_lowerhalf_s *dev);
static void *audio_fake_workerthread(pthread_addr_t pvarg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_fake_start(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session);
#else
static int audio_fake_start(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_fake_stop(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session);
#else
static int audio_fake_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_fake_pause(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session);
static int audio_fake_resume(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session);
#else
static int audio_fake_pause(FAR struct audio_lowerhalf_s *dev);
static int audio_fake_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int audio_fake_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                    FAR struct ap_buffer_s *apb);
static int audio_fake_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                   FAR struct ap_buffer_s *apb);
static int audio_fake_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                            unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_fake_reserve(FAR struct audio_lowerhalf_s *dev,
                              FAR void **session);
#else
static int audio_fake_reserve(FAR struct audio_lowerhalf_s *dev);
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_fake_release(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session);
#else
static int audio_fake_release(FAR struct audio_lowerhalf_s *dev);
#endif
static int audio_fake_file_init(FAR struct audio_lowerhalf_s *dev);
static int audio_fake_file_deinit(FAR struct audio_lowerhalf_s *dev);
static int audio_fake_file_write(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb);
static int audio_fake_file_read(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb);
static int audio_fake_process_buffer(FAR struct audio_lowerhalf_s *dev,
                                     FAR struct ap_buffer_s *apb);
static uint8_t AUDIO_SUBFMT_CONVERT(uint8_t format);
static uint32_t AUDIO_SAMP_RATE_CONVERT(uint32_t samplerate);
static FAR struct audio_lowerhalf_s *
audio_fake_init_device(bool playback,
                       FAR const struct audio_fake_params *dev_params);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_fake_params g_dev_params[] =
{
#ifdef CONFIG_FAKE_AUDIO_DEVICE_PARAMS
    CONFIG_FAKE_AUDIO_DEVICE_PARAMS
#endif
};

static const struct audio_ops_s g_audioops =
{
    audio_fake_getcaps,       /* getcaps        */
    audio_fake_configure,     /* configure      */
    audio_fake_shutdown,      /* shutdown       */
    audio_fake_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
    audio_fake_stop,          /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
    audio_fake_pause,         /* pause          */
    audio_fake_resume,        /* resume         */
#endif
    NULL,                     /* allocbuffer    */
    NULL,                     /* freebuffer     */
    audio_fake_enqueuebuffer, /* enqueue_buffer */
    audio_fake_cancelbuffer,  /* cancel_buffer  */
    audio_fake_ioctl,         /* ioctl          */
    NULL,                     /* read           */
    NULL,                     /* write          */
    audio_fake_reserve,       /* reserve        */
    audio_fake_release        /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: AUDIO_SAMP_RATE_CONVERT
 *
 * Description: Convert the samplerate to Nuttx audio samplerate.
 *
 ****************************************************************************/

static uint32_t AUDIO_SAMP_RATE_CONVERT(uint32_t samplerate)
{
  if (samplerate == 8000)
    {
      return AUDIO_SAMP_RATE_8K;
    }
  else if (samplerate == 11025)
    {
      return AUDIO_SAMP_RATE_11K;
    }
  else if (samplerate == 16000)
    {
      return AUDIO_SAMP_RATE_16K;
    }
  else if (samplerate == 22050)
    {
      return AUDIO_SAMP_RATE_22K;
    }
  else if (samplerate == 32000)
    {
      return AUDIO_SAMP_RATE_32K;
    }
  else if (samplerate == 44100)
    {
      return AUDIO_SAMP_RATE_44K;
    }
  else if (samplerate == 48000)
    {
      return AUDIO_SAMP_RATE_48K;
    }
  else if (samplerate != 0)
    {
      auderr("ERROR: Unsupported sample rate %d\n", samplerate);
    }

  return 0;
}

/****************************************************************************
 * Name: AUDIO_SUBFMT_CONVERT
 *
 * Description: Convert the format to Nuttx audio format.
 *
 ****************************************************************************/

static uint8_t AUDIO_SUBFMT_CONVERT(uint8_t format)
{
  if (format == 8)
    {
      return AUDIO_SUBFMT_PCM_S8;
    }
  else if (format == 16)
    {
      return AUDIO_SUBFMT_PCM_S16_LE;
    }
  else if (format == 32)
    {
      return AUDIO_SUBFMT_PCM_S32_LE;
    }
  else if (format != AUDIO_SUBFMT_END)
    {
      auderr("ERROR: Unsupported format %d\n", format);
    }

  return 0;
}

/****************************************************************************
 * Name: audio_fake_file_init
 *
 * Description: Initialize the audio file for playback or capture virtual
 * audio driver.
 *
 ****************************************************************************/

static int audio_fake_file_init(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;
  char filename[64];
  int ret;

  snprintf(filename, sizeof(filename), "%s/%s_%d_%d_%d.pcm",
           CONFIG_FAKE_AUDIO_DATA_PATH, priv->dev_params->dev_name,
           priv->sample_rate, priv->channels, priv->format);

  if (priv->playback)
    {
      ret = file_open(&priv->file, filename, O_RDWR | O_CREAT | O_CLOEXEC,
                      0666);
    }
  else
    {
      ret = file_open(&priv->file, filename, O_RDONLY | O_CLOEXEC);
    }

  audwarn("open %s file %s\n", filename, (ret < 0) ? "fail" : "success");

  return ret;
}

/****************************************************************************
 * Name: audio_fake_file_deinit
 *
 * Description: Deinitialize the audio file.
 *
 ****************************************************************************/

static int audio_fake_file_deinit(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;

  audinfo("audio_fake_file_deinit, close file\n");
  file_close(&priv->file);

  return 0;
}

/****************************************************************************
 * Name: audio_fake_file_write
 *
 * Description: Write the audio data to file.
 *
 ****************************************************************************/

static int audio_fake_file_write(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb)
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;
  int ret;

  ret = file_write(&priv->file, apb->samp, apb->nbytes);
  if (ret < 0)
    {
      auderr("Error write data , ret %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: audio_fake_file_read
 *
 * Description: Read the audio data from file.
 *
 ****************************************************************************/

static int audio_fake_file_read(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;
  int ret;

  ret = file_read(&priv->file, apb->samp, apb->nmaxbytes);
  if (ret == 0)
    {
      audwarn("read file end\n");
      file_seek(&priv->file, 0, SEEK_SET);
      ret = file_read(&priv->file, apb->samp, apb->nmaxbytes);
    }

  if (ret < 0)
    {
      auderr("Error read data , ret %d\n", ret);
      return ret;
    }

  apb->nbytes  = ret;
  apb->curbyte = 0;
  apb->flags   = 0;

  return ret;
}

/****************************************************************************
 * Name: audio_fake_process_buffer
 *
 * Description: Process the audio buffer.
 *
 ****************************************************************************/

static int audio_fake_process_buffer(FAR struct audio_lowerhalf_s *dev,
                                     FAR struct ap_buffer_s *apb)
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;
  int64_t frame_time;
  int64_t diff_time;
  int64_t sleep_time;
  struct timeval tv1;
  struct timeval tv2;
  int ret;

  audinfo("process apb=%p, nbytes=%d\n", apb, apb->nbytes);

  /* Check if this was the last buffer in the stream */

  priv->terminate = ((apb->flags & AUDIO_APB_FINAL) != 0);

  gettimeofday(&tv1, NULL);

  if (priv->playback)
    {
      ret = audio_fake_file_write(dev, apb);
      if (ret < 0)
        {
          auderr("Error write data, ret %d\n", ret);
          goto out;
        }
    }
  else
    {
      ret = audio_fake_file_read(dev, apb);
      if (ret < 0)
        {
          auderr("Error read data , ret %d\n", ret);
          goto out;
        }
    }

  gettimeofday(&tv2, NULL);

  frame_time = ((int64_t)apb->nbytes * 1000 * 1000) / priv->scaler;

  diff_time = (int64_t)tv2.tv_sec * 1000000 + tv2.tv_usec -
              ((int64_t)tv1.tv_sec * 1000000 + tv1.tv_usec);

  if (diff_time >= frame_time)
    {
      audwarn("WARN: %s file time %" PRId64 " > frame time %" PRId64 ".\n",
              priv->playback ? "write" : "read",
              diff_time, frame_time);
      ret = OK;
      goto out;
    }

  sleep_time = frame_time - diff_time;

  nxsig_usleep(sleep_time);

  ret = OK;

out:
#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif

  return ret;
}

/****************************************************************************
 * Name: audio_fake_getcaps
 *
 * Description: Get the audio device capabilities
 *
 ****************************************************************************/

static int audio_fake_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                              FAR struct audio_caps_s *caps)
{
  FAR struct audio_fake_s *priv = (struct audio_fake_s *)dev;

  audinfo("type=%d\n", type);

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

        caps->ac_channels = (priv->dev_params->channels[0] << 4) |
                            (priv->dev_params->channels[1] & 0x0f);

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:
              /* We don't decode any formats!  Only something above us in
               * the audio stream can perform decoding on our behalf.
               */

              /* The types of audio units we implement */

              caps->ac_controls.b[0] =
                  priv->playback ? AUDIO_TYPE_OUTPUT : AUDIO_TYPE_INPUT;
              caps->ac_format.hw = 1 << (AUDIO_FMT_PCM - 1);

              break;

            case AUDIO_FMT_PCM:

              caps->ac_controls.b[0] =
                  AUDIO_SUBFMT_CONVERT(priv->dev_params->format[0]);
              caps->ac_controls.b[1] =
                  AUDIO_SUBFMT_CONVERT(priv->dev_params->format[1]);
              caps->ac_controls.b[2] =
                  AUDIO_SUBFMT_CONVERT(priv->dev_params->format[2]);
              caps->ac_controls.b[3] =
                  AUDIO_SUBFMT_CONVERT(priv->dev_params->format[3]);

              break;

            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;
          }

        break;

      /* Provide capabilities of our OUTPUT unit */

      case AUDIO_TYPE_OUTPUT:
      case AUDIO_TYPE_INPUT:

        caps->ac_channels = (priv->dev_params->channels[0] << 4) |
                            (priv->dev_params->channels[1] & 0x0f);

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              caps->ac_channels = (priv->dev_params->channels[0] << 4) |
                                  (priv->dev_params->channels[1] & 0x0f);

              /* Report the Sample rates we support */

              caps->ac_controls.hw[0] =
                  AUDIO_SAMP_RATE_CONVERT(priv->dev_params->samplerate[0]) |
                  AUDIO_SAMP_RATE_CONVERT(priv->dev_params->samplerate[1]) |
                  AUDIO_SAMP_RATE_CONVERT(priv->dev_params->samplerate[2]) |
                  AUDIO_SAMP_RATE_CONVERT(priv->dev_params->samplerate[3]);

              break;

            default:
              break;
          }

        break;

      /* All others we don't support */

      default:

        /* Zero out the fields to indicate no support */

        caps->ac_subtype  = 0;
        caps->ac_channels = 0;

        break;
    }

  /* Return the length of the audio_caps_s struct for validation of
   * proper Audio device type.
   */

  audinfo("Return %d\n", caps->ac_len);
  return caps->ac_len;
}

/****************************************************************************
 * Name: audio_fake_configure
 *
 * Description:
 *   Configure the audio device for the specified  mode of operation.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_fake_configure(FAR struct audio_lowerhalf_s *dev,
                                FAR void *session,
                                FAR const struct audio_caps_s *caps)
#else
static int audio_fake_configure(FAR struct audio_lowerhalf_s *dev,
                                FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;
  int ret;
  audinfo("ac_type: %d\n", caps->ac_type);

  if (priv->mqname[0] == '\0')
    {
      struct mq_attr attr;

      /* Create a message queue for the worker thread */

      snprintf(priv->mqname, sizeof(priv->mqname), "/tmp/%" PRIXPTR,
               (uintptr_t)priv);

      attr.mq_maxmsg  = 16;
      attr.mq_msgsize = sizeof(struct audio_msg_s);
      attr.mq_curmsgs = 0;
      attr.mq_flags   = 0;

      ret = file_mq_open(&priv->mq, priv->mqname, O_RDWR | O_CREAT, 0644,
                         &attr);
      if (ret < 0)
        {
          /* Error creating message queue! */

          auderr("ERROR: Couldn't allocate message queue\n");
          return ret;
        }
    }

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_OUTPUT:
      case AUDIO_TYPE_INPUT:
        priv->sample_rate =
            caps->ac_controls.hw[0] | (caps->ac_controls.b[3] << 16);
        priv->channels = caps->ac_channels;
        priv->format   = caps->ac_controls.b[2];

        priv->scaler = caps->ac_channels * caps->ac_controls.hw[0] *
                       caps->ac_controls.b[2] / 8;

        audinfo("Audio type: %s\n", (caps->ac_type == AUDIO_TYPE_OUTPUT)
                                        ? "AUDIO_TYPE_OUTPUT"
                                        : "AUDIO_TYPE_INPUT");
        audinfo("Number of channels: %u\n", caps->ac_channels);
        audinfo("Sample rate:        %u\n", caps->ac_controls.hw[0]);
        audinfo("Sample width:       %u\n", caps->ac_controls.b[2]);
        break;

      default:
        audinfo("default case: %d\n", caps->ac_type);
        break;
    }

  audinfo("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: audio_fake_shutdown
 *
 * Description:
 *   Shutdown the driver and put it in the lowest power state possible.
 *
 ****************************************************************************/

static int audio_fake_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  audinfo("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: audio_fake_workerthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 *
 ****************************************************************************/

static void *audio_fake_workerthread(pthread_addr_t pvarg)
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)pvarg;
  FAR struct ap_buffer_s *apb;
  struct audio_msg_s msg;
  struct mq_attr attr;
  unsigned int prio;
  int msglen;
  int ret;

  audinfo("Entry\n");

  /* Loop as long as we are supposed to be running */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  while (!priv->terminate)
#else
  for (; ; )
#endif
    {
      /* Wait for messages from our message queue */

      msglen =
          file_mq_receive(&priv->mq, (FAR char *)&msg, sizeof(msg), &prio);

      /* Handle the case when we return with no message */

      if (msglen < sizeof(struct audio_msg_s))
        {
          auderr("ERROR: Message too small: %d\n", msglen);
          continue;
        }

      /* Process the message */

      switch (msg.msg_id)
        {
          case AUDIO_MSG_DATA_REQUEST:
            break;

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
          case AUDIO_MSG_STOP:

            /* Consume all buffers on the bufferq after stop */

            for (; ; )
              {
                file_mq_getattr(&priv->mq, &attr);
                if (attr.mq_curmsgs > 0)
                  {
                    file_mq_receive(&priv->mq, (FAR char *)&msg, sizeof(msg),
                                    &prio);

                    /* direct dequeue buffer to application */

                    apb = (FAR struct ap_buffer_s *)msg.u.ptr;
#ifdef CONFIG_AUDIO_MULTI_SESSION
                    priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                                    apb, OK, NULL);
#else
                    priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                                    apb, OK);
#endif
                    continue;
                  }
                else
                  {
                    break;
                  }
              }

            priv->terminate = true;
            break;
#endif

          case AUDIO_MSG_ENQUEUE:
            apb = (FAR struct ap_buffer_s *)msg.u.ptr;
            ret = audio_fake_process_buffer(&priv->dev, apb);
            if (ret)
              {
                auderr("fake audio process error %d\n", ret);
                priv->terminate = true;
              }
            break;

          default:
            auderr("ERROR: Ignoring message ID %d\n", msg.msg_id);
            break;
        }
    }

  /* Close the message queue */

  file_mq_close(&priv->mq);
  file_mq_unlink(priv->mqname);
  priv->mqname[0] = '\0';
  priv->terminate = false;

  audio_fake_file_deinit(&priv->dev);

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif
  audinfo("Exit %s\n", priv->dev_params->dev_name);

  return NULL;
}

/****************************************************************************
 * Name: audio_fake_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_fake_start(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session)
#else
static int audio_fake_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;
  struct sched_param sparam;
  pthread_attr_t tattr;
  FAR void *value;
  int ret;

  priv->terminate = false;

  ret = audio_fake_file_init(dev);
  if (ret < 0)
    {
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
  pthread_attr_setstacksize(&tattr, CONFIG_AUDIO_FAKE_WORKER_STACKSIZE);

  audinfo("Starting worker thread\n");
  ret = pthread_create(&priv->threadid, &tattr, audio_fake_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("ERROR: pthread_create failed: %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "audio_fake");
      audinfo("Created worker thread\n");
    }

  audinfo("Return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: audio_fake_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_fake_stop(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session)
#else
static int audio_fake_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;
  struct audio_msg_s term_msg;
  FAR void *value;

  /* Send a message to stop all audio streaming */

  term_msg.msg_id = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  file_mq_send(&priv->mq, (FAR const char *)&term_msg, sizeof(term_msg),
               CONFIG_AUDIO_FAKE_MSG_PRIO);

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  dev->upper(dev->priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  dev->upper(dev->priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  audinfo("Return OK\n");
  return OK;
}
#endif

/****************************************************************************
 * Name: audio_fake_pause
 *
 * Description: Pauses the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_fake_pause(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session)
#else
static int audio_fake_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;
  audinfo("%s pause\n", priv->dev_params->dev_name);
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: audio_fake_resume
 *
 * Description: Resumes the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_fake_resume(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session)
#else
static int audio_fake_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;
  audinfo("%s resume\n", priv->dev_params->dev_name);
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: audio_fake_enqueuebuffer
 *
 * Description: Enqueue an Audio Pipeline Buffer for
 * playback/capture processing.
 *
 ****************************************************************************/

static int audio_fake_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                    FAR struct ap_buffer_s *apb)
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;
  struct audio_msg_s msg;
  int ret;

  DEBUGASSERT(priv && apb && priv->dev.upper);

  audinfo("apb=%p curbyte=%d nbytes=%d\n", apb, apb->curbyte, apb->nbytes);

  msg.msg_id = AUDIO_MSG_ENQUEUE;
  msg.u.ptr  = apb;

  ret = file_mq_send(&priv->mq, (FAR const char *)&msg, sizeof(msg),
                      CONFIG_AUDIO_FAKE_MSG_PRIO);
  if (ret < 0)
    {
      auderr("ERROR: file_mq_send failed: %d\n", ret);
    }

  audinfo("Return OK\n");
  return ret;
}

/****************************************************************************
 * Name: audio_fake_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/

static int audio_fake_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                   FAR struct ap_buffer_s *apb)
{
  audinfo("apb=%p curbyte=%d nbytes=%d, return OK\n", apb, apb->curbyte,
          apb->nbytes);

  return OK;
}

/****************************************************************************
 * Name: audio_fake_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/

static int audio_fake_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                            unsigned long arg)
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;
  int ret                       = OK;
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  FAR struct ap_buffer_info_s *bufinfo;
#endif

  audinfo("cmd=%d arg=%ld\n", cmd, arg);

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

      case AUDIOIOC_GETBUFFERINFO:
        {
          audinfo("AUDIOIOC_GETBUFFERINFO:\n");
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
          bufinfo = (FAR struct ap_buffer_info_s *)arg;
          bufinfo->buffer_size =
              (priv->scaler * priv->dev_params->period_time) / 1000;
          bufinfo->nbuffers = priv->dev_params->periods;
#else
          audwarn("AUDIOIOC_GETBUFFERINFO Return EPERM\n");
          return -EPERM;
#endif
        }
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  audinfo("Return OK\n");
  return ret;
}

/****************************************************************************
 * Name: audio_fake_reserve
 *
 * Description: Reserves a session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_fake_reserve(FAR struct audio_lowerhalf_s *dev,
                              FAR void **session)
#else
static int audio_fake_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;
  priv->terminate               = false;
  audinfo("%s reserve\n", priv->dev_params->dev_name);
  return OK;
}

/****************************************************************************
 * Name: audio_fake_release
 *
 * Description: Releases the session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_fake_release(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session)
#else
static int audio_fake_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_fake_s *priv = (FAR struct audio_fake_s *)dev;
  void *value;

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
      priv->threadid = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: audio_fake_init_device
 *
 * Description: Initialize the audio device.
 *
 ****************************************************************************/

static FAR struct audio_lowerhalf_s *
audio_fake_init_device(bool playback,
                       FAR const struct audio_fake_params *dev_params)
{
  FAR struct audio_fake_s *priv;

  /* Allocate the fake audio device structure */

  priv = (FAR struct audio_fake_s *)kmm_zalloc(sizeof(struct audio_fake_s));
  if (!priv)
    {
      auderr("ERROR: Failed to allocate fake audio device\n");
      return NULL;
    }

  priv->dev.ops    = &g_audioops;
  priv->dev_params = dev_params;
  priv->playback   = playback;
  priv->terminate  = false;

  return &priv->dev;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: audio_fake_initialize
 *
 * Description:
 *   Initialize and register fake audio device.
 *
 * Returned Value:
 *   0 is returned on success;
 *   others is returned on failure.
 *
 ****************************************************************************/

int audio_fake_initialize(void)
{
  int ret;
  int i;

  for (i = 0; i < nitems(g_dev_params); i++)
    {
      ret = audio_register(g_dev_params[i].dev_name,
                           audio_fake_init_device(g_dev_params[i].playback,
                                                  &g_dev_params[i]));
      if (ret < 0)
        {
          auderr("ERROR: Failed to register (%s) fake audio device.\n",
                 g_dev_params[i].dev_name);
          return ret;
        }
    }

  return 0;
}
