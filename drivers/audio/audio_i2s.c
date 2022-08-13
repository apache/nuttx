/****************************************************************************
 * drivers/audio/audio_i2s.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct audio_i2s_s
{
  struct audio_lowerhalf_s dev;
  struct i2s_dev_s *i2s;
  bool playback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int audio_i2s_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                             FAR struct audio_caps_s *caps);
static int audio_i2s_shutdown(FAR struct audio_lowerhalf_s *dev);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_i2s_configure(FAR struct audio_lowerhalf_s *dev,
                               FAR void *session,
                               FAR const struct audio_caps_s *caps);
static int audio_i2s_start(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int audio_i2s_stop(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int audio_i2s_pause(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session);
static int audio_i2s_resume(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session);
#endif
static int audio_i2s_reserve(FAR struct audio_lowerhalf_s *dev,
                             FAR void **session);
static int audio_i2s_release(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session);
#else
static int audio_i2s_configure(FAR struct audio_lowerhalf_s *dev,
                               FAR const struct audio_caps_s *caps);
static int audio_i2s_start(FAR struct audio_lowerhalf_s *dev);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int audio_i2s_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int audio_i2s_pause(FAR struct audio_lowerhalf_s *dev);
static int audio_i2s_resume(FAR struct audio_lowerhalf_s *dev);
#endif
static int audio_i2s_reserve(FAR struct audio_lowerhalf_s *dev);
static int audio_i2s_release(FAR struct audio_lowerhalf_s *dev);
#endif
static int audio_i2s_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct audio_buf_desc_s *bufdesc);
static int audio_i2s_freebuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct audio_buf_desc_s *bufdesc);
static int audio_i2s_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                   FAR struct ap_buffer_s *apb);
static int audio_i2s_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                           unsigned long arg);
static void audio_i2s_callback(struct i2s_dev_s *dev,
                               FAR struct ap_buffer_s *apb, FAR void *arg,
                               int result);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audio_i2s_ops =
{
  audio_i2s_getcaps,       /* getcaps        */
  audio_i2s_configure,     /* configure      */
  audio_i2s_shutdown,      /* shutdown       */
  audio_i2s_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  audio_i2s_stop,          /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  audio_i2s_pause,         /* pause          */
  audio_i2s_resume,        /* resume         */
#endif
  audio_i2s_allocbuffer,   /* allocbuffer    */
  audio_i2s_freebuffer,    /* freebuffer     */
  audio_i2s_enqueuebuffer, /* enqueue_buffer */
  NULL,                    /* cancel_buffer  */
  audio_i2s_ioctl,         /* ioctl          */
  NULL,                    /* read           */
  NULL,                    /* write          */
  audio_i2s_reserve,       /* reserve        */
  audio_i2s_release        /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int audio_i2s_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                             FAR struct audio_caps_s *caps)
{
  FAR struct audio_i2s_s *audio_i2s = (struct audio_i2s_s *)dev;
  FAR struct i2s_dev_s *i2s = audio_i2s->i2s;

  /* Validate the structure */

  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));
  audinfo("type=%d ac_type=%d\n", type, caps->ac_type);

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      /* Caller is querying for the types of units we support */

      case AUDIO_TYPE_QUERY:

        /* Provide our overall capabilities.  The interfacing software
         * must then call us back for specific info for each capability.
         */

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* We don't decode any formats!  Only something above us in
               * the audio stream can perform decoding on our behalf.
               */

              /* The types of audio units we implement */

              if (audio_i2s->playback)
                {
                  caps->ac_controls.b[0] = AUDIO_TYPE_OUTPUT;
                }
              else
                {
                  caps->ac_controls.b[0] = AUDIO_TYPE_INPUT;
                }

              caps->ac_format.hw = 1 << (AUDIO_FMT_PCM - 1);
              break;

            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;
          }
        break;

        /* Provide capabilities of our OUTPUT unit */

      case AUDIO_TYPE_OUTPUT:
      case AUDIO_TYPE_INPUT:

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

            /* Report the Sample rates we support */

              caps->ac_controls.hw[0] =
                AUDIO_SAMP_RATE_8K   | AUDIO_SAMP_RATE_11K  |
                AUDIO_SAMP_RATE_16K  | AUDIO_SAMP_RATE_22K  |
                AUDIO_SAMP_RATE_32K  | AUDIO_SAMP_RATE_44K  |
                AUDIO_SAMP_RATE_48K  | AUDIO_SAMP_RATE_96K  |
                AUDIO_SAMP_RATE_128K | AUDIO_SAMP_RATE_160K |
                AUDIO_SAMP_RATE_172K | AUDIO_SAMP_RATE_192K;
              break;

            default:
              I2S_IOCTL(i2s, AUDIOIOC_GETCAPS, (unsigned long)caps);
              break;
          }
        break;

      default:
        I2S_IOCTL(i2s, AUDIOIOC_GETCAPS, (unsigned long)caps);
        break;
   }

  return caps->ac_len;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_i2s_configure(FAR struct audio_lowerhalf_s *dev,
                               FAR void *session,
                               FAR const struct audio_caps_s *caps)
#else
static int audio_i2s_configure(FAR struct audio_lowerhalf_s *dev,
                               FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct audio_i2s_s *audio_i2s = (struct audio_i2s_s *)dev;
  FAR struct i2s_dev_s *i2s;
  int samprate;
  int nchannels;
  int bpsamp;
  int ret = OK;

  DEBUGASSERT(audio_i2s != NULL && caps != NULL);
  i2s = audio_i2s->i2s;
  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_OUTPUT:
      case AUDIO_TYPE_INPUT:

        /* Save the current stream configuration */

        samprate  = caps->ac_controls.hw[0] |
                    (caps->ac_controls.b[3] << 16);
        nchannels = caps->ac_channels;
        bpsamp    = caps->ac_controls.b[2];

        if (audio_i2s->playback)
          {
            I2S_TXCHANNELS(i2s, nchannels);
            I2S_TXDATAWIDTH(i2s, bpsamp);
            I2S_TXSAMPLERATE(i2s, samprate);
          }
        else
          {
            I2S_RXCHANNELS(i2s, nchannels);
            I2S_RXDATAWIDTH(i2s, bpsamp);
            I2S_RXSAMPLERATE(i2s, samprate);
          }
        break;

      default:
        ret = I2S_IOCTL(i2s, AUDIOIOC_CONFIGURE, (unsigned long)caps);
        break;
    }

  return ret;
}

static int audio_i2s_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct audio_i2s_s *audio_i2s = (struct audio_i2s_s *)dev;
  FAR struct i2s_dev_s *i2s = audio_i2s->i2s;

  return I2S_IOCTL(i2s, AUDIOIOC_SHUTDOWN, audio_i2s->playback);
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_i2s_start(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session)
#else
static int audio_i2s_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_i2s_s *audio_i2s = (struct audio_i2s_s *)dev;
  FAR struct i2s_dev_s *i2s = audio_i2s->i2s;

  return I2S_IOCTL(i2s, AUDIOIOC_START, audio_i2s->playback);
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_i2s_stop(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session)
#else
static int audio_i2s_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_i2s_s *audio_i2s = (struct audio_i2s_s *)dev;
  FAR struct i2s_dev_s *i2s = audio_i2s->i2s;

  return I2S_IOCTL(i2s, AUDIOIOC_STOP, audio_i2s->playback);
}
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_i2s_pause(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session)
#else
static int audio_i2s_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_i2s_s *audio_i2s = (struct audio_i2s_s *)dev;
  FAR struct i2s_dev_s *i2s = audio_i2s->i2s;

  return I2S_IOCTL(i2s, AUDIOIOC_PAUSE, audio_i2s->playback);
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_i2s_resume(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session)
#else
static int audio_i2s_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_i2s_s *audio_i2s = (struct audio_i2s_s *)dev;
  FAR struct i2s_dev_s *i2s = audio_i2s->i2s;

  return I2S_IOCTL(i2s, AUDIOIOC_RESUME, audio_i2s->playback);
}
#endif

static int audio_i2s_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct audio_buf_desc_s *bufdesc)
{
  FAR struct audio_i2s_s *audio_i2s = (struct audio_i2s_s *)dev;
  FAR struct i2s_dev_s *i2s = audio_i2s->i2s;

  return I2S_IOCTL(i2s, AUDIOIOC_ALLOCBUFFER, (unsigned long)bufdesc);
}

static int audio_i2s_freebuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct audio_buf_desc_s *bufdesc)
{
  FAR struct audio_i2s_s *audio_i2s = (struct audio_i2s_s *)dev;
  FAR struct i2s_dev_s *i2s = audio_i2s->i2s;

  return I2S_IOCTL(i2s, AUDIOIOC_FREEBUFFER, (unsigned long)bufdesc);
}

static int audio_i2s_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                   FAR struct ap_buffer_s *apb)
{
  FAR struct audio_i2s_s *audio_i2s = (struct audio_i2s_s *)dev;
  FAR struct i2s_dev_s *i2s = audio_i2s->i2s;

  if (audio_i2s->playback)
    {
      return I2S_SEND(i2s, apb, audio_i2s_callback, audio_i2s, 0);
    }
  else
    {
      return I2S_RECEIVE(i2s, apb, audio_i2s_callback, audio_i2s, 0);
    }
}

static int audio_i2s_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                           unsigned long arg)
{
  FAR struct audio_i2s_s *audio_i2s = (struct audio_i2s_s *)dev;
  FAR struct i2s_dev_s *i2s = audio_i2s->i2s;

  return I2S_IOCTL(i2s, cmd, arg);
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_i2s_reserve(FAR struct audio_lowerhalf_s *dev,
                             FAR void **session)
#else
static int audio_i2s_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
#ifdef CONFIG_AUDIO_MULTI_SESSION
  *session = (void *)audio_i2s->playback;
#endif
  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_i2s_release(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session)
#else
static int audio_i2s_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  return OK;
}

static void audio_i2s_callback(struct i2s_dev_s *dev,
                               FAR struct ap_buffer_s *apb,
                               FAR void *arg, int result)
{
  FAR struct audio_i2s_s *audio_i2s = arg;
  bool final = false;

  if ((apb->flags & AUDIO_APB_FINAL) != 0)
    {
      final = true;
    }

#ifdef CONFIG_AUDIO_MULTI_SESSION
  audio_i2s->dev.upper(audio_i2s->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb,
                       OK, NULL);
#else
  audio_i2s->dev.upper(audio_i2s->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb,
                       OK);
#endif
  if (final)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      audio_i2s->dev.upper(audio_i2s->dev.priv, AUDIO_CALLBACK_COMPLETE,
                           NULL, OK, NULL);
#else
      audio_i2s->dev.upper(audio_i2s->dev.priv, AUDIO_CALLBACK_COMPLETE,
                           NULL, OK);
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct audio_lowerhalf_s *audio_i2s_initialize(FAR struct i2s_dev_s *i2s,
                                                   bool playback)
{
  FAR struct audio_i2s_s *audio_i2s;

  if (i2s == NULL)
    {
      return NULL;
    }

  audio_i2s = kmm_zalloc(sizeof(struct audio_i2s_s));
  if (audio_i2s == NULL)
    {
      return NULL;
    }

  audio_i2s->playback = playback;
  audio_i2s->i2s = i2s;
  audio_i2s->dev.ops = &g_audio_i2s_ops;

  return &audio_i2s->dev;
}
