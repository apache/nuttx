/****************************************************************************
 * audio/audio_comp.c
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

#include <stdarg.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/audio_comp.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the internal state of the audio composite */

struct audio_comp_priv_s
{
  /* This is is our appearance to the outside world. This *MUST* be the
   * first element of the structure so that we can freely cast between
   * types struct audio_lowerhalf and struct audio_comp_dev_s.
   */

  struct audio_lowerhalf_s export;

  /* This is the contained, low-level audio device array and count. */

  FAR struct audio_lowerhalf_s **lower;
  int count;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int audio_comp_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                              FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_comp_configure(FAR struct audio_lowerhalf_s *dev,
                                FAR void *session,
                                FAR const struct audio_caps_s *caps);
#else
static int audio_comp_configure(FAR struct audio_lowerhalf_s *dev,
                                FAR const struct audio_caps_s *caps);
#endif
static int audio_comp_shutdown(FAR struct audio_lowerhalf_s *dev);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_comp_start(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session);
#else
static int audio_comp_start(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_comp_stop(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session);
#else
static int audio_comp_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_comp_pause(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session);
static int audio_comp_resume(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session);
#else
static int audio_comp_pause(FAR struct audio_lowerhalf_s *dev);
static int audio_comp_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int audio_comp_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                                  FAR struct audio_buf_desc_s *bufdesc);
static int audio_comp_freebuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct audio_buf_desc_s *bufdesc);
static int audio_comp_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                    FAR struct ap_buffer_s *apb);
static int audio_comp_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                   FAR struct ap_buffer_s *apb);
static int audio_comp_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                            unsigned long arg);
static int audio_comp_read(FAR struct audio_lowerhalf_s *dev,
                           FAR char *buffer, size_t buflen);
static int audio_comp_write(FAR struct audio_lowerhalf_s *dev,
                            FAR const char *buffer, size_t buflen);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_comp_reserve(FAR struct audio_lowerhalf_s *dev,
                              FAR void **session);
#else
static int audio_comp_reserve(FAR struct audio_lowerhalf_s *dev);
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_comp_release(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session);
#else
static int audio_comp_release(FAR struct audio_lowerhalf_s *dev);
#endif

#ifdef CONFIG_AUDIO_MULTI_SESSION
static void audio_comp_callback(FAR void *arg, uint16_t reason,
                                FAR struct ap_buffer_s *apb,
                                uint16_t status,
                                FAR void *session);
#else
static void audio_comp_callback(FAR void *arg, uint16_t reason,
                                FAR struct ap_buffer_s *apb,
                                uint16_t status);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audio_comp_ops =
{
  audio_comp_getcaps,       /* getcaps        */
  audio_comp_configure,     /* configure      */
  audio_comp_shutdown,      /* shutdown       */
  audio_comp_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  audio_comp_stop,          /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  audio_comp_pause,         /* pause          */
  audio_comp_resume,        /* resume         */
#endif
  audio_comp_allocbuffer,   /* allocbuffer    */
  audio_comp_freebuffer,    /* freebuffer     */
  audio_comp_enqueuebuffer, /* enqueue_buffer */
  audio_comp_cancelbuffer,  /* cancel_buffer  */
  audio_comp_ioctl,         /* ioctl          */
  audio_comp_read,          /* read           */
  audio_comp_write,         /* write          */
  audio_comp_reserve,       /* reserve        */
  audio_comp_release        /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: audio_comp_getcaps
 *
 * Description: Get the audio device capabilities
 *
 ****************************************************************************/

static int audio_comp_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                              FAR struct audio_caps_s *caps)
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
  int ret = -ENOTTY;
  int i;

  caps->ac_channels   = 0;
  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  for (i = 0; i < priv->count; i++)
    {
      if (lower[i]->ops->getcaps)
        {
          FAR struct audio_caps_s dup = *caps;

          int tmp = lower[i]->ops->getcaps(lower[i], type, &dup);
          if (tmp == -ENOTTY)
            {
              continue;
            }

          ret = tmp;
          if (ret < 0)
            {
              break;
            }

          if (caps->ac_channels < dup.ac_channels)
            {
              caps->ac_channels = dup.ac_channels;
            }

          caps->ac_format.hw   |= dup.ac_format.hw;
          caps->ac_controls.w  |= dup.ac_controls.w;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: audio_comp_configure
 *
 * Description:
 *   Configure the audio device for the specified  mode of operation.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_comp_configure(FAR struct audio_lowerhalf_s *dev,
                                FAR void *session,
                                FAR const struct audio_caps_s *caps)
#else
static int audio_comp_configure(FAR struct audio_lowerhalf_s *dev,
                                FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void **sess = session;
#endif
  int ret = -ENOTTY;
  int i;

  for (i = 0; i < priv->count; i++)
    {
      if (lower[i]->ops->configure)
        {
#ifdef CONFIG_AUDIO_MULTI_SESSION
          int tmp = lower[i]->ops->configure(lower[i], sess[i], caps);
#else
          int tmp = lower[i]->ops->configure(lower[i], caps);
#endif
          if (tmp == -ENOTTY)
            {
              continue;
            }

          ret = tmp;
          if (ret < 0)
            {
              break;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: audio_comp_shutdown
 *
 * Description:
 *   Shutdown the driver and put it in the lowest power state possible.
 *
 ****************************************************************************/

static int audio_comp_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
  int ret = -ENOTTY;
  int i;

  for (i = priv->count - 1; i >= 0; i--)
    {
      if (lower[i]->ops->shutdown)
        {
          int tmp = lower[i]->ops->shutdown(lower[i]);
          if (tmp == -ENOTTY)
            {
              continue;
            }

          if (tmp < 0 || ret == -ENOTTY || ret >= 0)
            {
              ret = tmp;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: audio_comp_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_comp_start(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session)
#else
static int audio_comp_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void **sess = session;
#endif
  int ret = -ENOTTY;
  int i;

  for (i = 0; i < priv->count; i++)
    {
      if (lower[i]->ops->start)
        {
#ifdef CONFIG_AUDIO_MULTI_SESSION
          int tmp = lower[i]->ops->start(lower[i], sess[i]);
#else
          int tmp = lower[i]->ops->start(lower[i]);
#endif
          if (tmp == -ENOTTY)
            {
              continue;
            }

          ret = tmp;
          if (ret >= 0)
            {
              continue;
            }

          while (--i >= 0)
            {
              if (lower[i]->ops->stop)
                {
#ifdef CONFIG_AUDIO_MULTI_SESSION
                  lower[i]->ops->stop(lower[i], sess[i]);
#else
                  lower[i]->ops->stop(lower[i]);
#endif
                }
            }
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: audio_comp_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_comp_stop(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session)
#else
static int audio_comp_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void **sess = session;
#endif
  int ret = -ENOTTY;
  int i;

  for (i = priv->count - 1; i >= 0; i--)
    {
      if (lower[i]->ops->stop)
        {
#ifdef CONFIG_AUDIO_MULTI_SESSION
          int tmp = lower[i]->ops->stop(lower[i], sess[i]);
#else
          int tmp = lower[i]->ops->stop(lower[i]);
#endif
          if (tmp == -ENOTTY)
            {
              continue;
            }

          if (tmp < 0 || ret == -ENOTTY || ret >= 0)
            {
              ret = tmp;
            }
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: audio_comp_pause
 *
 * Description: Pauses the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_comp_pause(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session)
#else
static int audio_comp_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void **sess = session;
#endif
  int ret = -ENOTTY;
  int i;

  for (i = priv->count - 1; i >= 0; i--)
    {
      if (lower[i]->ops->pause)
        {
#ifdef CONFIG_AUDIO_MULTI_SESSION
          int tmp = lower[i]->ops->pause(lower[i], sess[i]);
#else
          int tmp = lower[i]->ops->pause(lower[i]);
#endif
          if (tmp == -ENOTTY)
            {
              continue;
            }

          if (tmp < 0 || ret == -ENOTTY || ret >= 0)
            {
              ret = tmp;
            }
        }
    }

  return ret;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: audio_comp_resume
 *
 * Description: Resumes the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_comp_resume(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session)
#else
static int audio_comp_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void **sess = session;
#endif
  int ret = -ENOTTY;
  int i;

  for (i = 0; i < priv->count; i++)
    {
      if (lower[i]->ops->resume)
        {
#ifdef CONFIG_AUDIO_MULTI_SESSION
          int tmp = lower[i]->ops->resume(lower[i], sess[i]);
#else
          int tmp = lower[i]->ops->resume(lower[i]);
#endif
          if (tmp == -ENOTTY)
            {
              continue;
            }

          ret = tmp;
          if (ret >= 0)
            {
              continue;
            }

          while (--i >= 0)
            {
              if (lower[i]->ops->pause)
                {
#ifdef CONFIG_AUDIO_MULTI_SESSION
                  lower[i]->ops->pause(lower[i], sess[i]);
#else
                  lower[i]->ops->pause(lower[i]);
#endif
                }
            }
          break;
        }
    }

  return ret;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: audio_comp_allocbuffer
 *
 * Description: Allocate an audio pipeline buffer.
 *
 ****************************************************************************/

static int audio_comp_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                                  FAR struct audio_buf_desc_s *bufdesc)
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
  int ret = -ENOTTY;
  int i;

  for (i = 0; i < priv->count; i++)
    {
      if (lower[i]->ops->allocbuffer)
        {
          ret = lower[i]->ops->allocbuffer(lower[i], bufdesc);
          if (ret != -ENOTTY)
            {
              break;
            }
        }
    }

  if (ret == -ENOTTY)
    {
      ret = apb_alloc(bufdesc);
    }

  return ret;
}

/****************************************************************************
 * Name: audio_comp_freebuffer
 *
 * Description: Free an audio pipeline buffer.
 *
 ****************************************************************************/

static int audio_comp_freebuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct audio_buf_desc_s *bufdesc)
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
  int ret = -ENOTTY;
  int i;

  for (i = 0; i < priv->count; i++)
    {
      if (lower[i]->ops->freebuffer)
        {
          ret = lower[i]->ops->freebuffer(lower[i], bufdesc);
          if (ret != -ENOTTY)
            {
              break;
            }
        }
    }

  if (ret == -ENOTTY)
    {
      apb_free(bufdesc->u.buffer);
      ret = sizeof(*bufdesc);
    }

  return ret;
}

/****************************************************************************
 * Name: audio_comp_enqueuebuffer
 *
 * Description: Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 ****************************************************************************/

static int audio_comp_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                    FAR struct ap_buffer_s *apb)
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
  int ret = -ENOTTY;
  int i;

  for (i = 0; i < priv->count; i++)
    {
      if (lower[i]->ops->enqueuebuffer)
        {
          ret = lower[i]->ops->enqueuebuffer(lower[i], apb);
          if (ret != -ENOTTY)
            {
              break;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: audio_comp_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/

static int audio_comp_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                   FAR struct ap_buffer_s *apb)
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
  int ret = -ENOTTY;
  int i;

  for (i = 0; i < priv->count; i++)
    {
      if (lower[i]->ops->cancelbuffer)
        {
          ret = lower[i]->ops->cancelbuffer(lower[i], apb);
          if (ret != -ENOTTY)
            {
              break;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: audio_comp_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/

static int audio_comp_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                            unsigned long arg)
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
  int ret = -ENOTTY;
  int i;

  for (i = 0; i < priv->count; i++)
    {
      if (lower[i]->ops->ioctl)
        {
          int tmp = lower[i]->ops->ioctl(lower[i], cmd, arg);
          if (tmp == -ENOTTY)
            {
              continue;
            }

          ret = tmp;
          if (ret < 0)
            {
              break;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: audio_comp_read
 *
 * Description:  Lower-half logic for read commands.
 *
 ****************************************************************************/

static int audio_comp_read(FAR struct audio_lowerhalf_s *dev,
                           FAR char *buffer, size_t buflen)
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
  int ret = -ENOTTY;
  int i;

  for (i = 0; i < priv->count; i++)
    {
      if (lower[i]->ops->read)
        {
          ret = lower[i]->ops->read(lower[i], buffer, buflen);
          if (ret != -ENOTTY)
            {
              break;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: audio_comp_write
 *
 * Description:  Lower-half logic for write commands.
 *
 ****************************************************************************/

static int audio_comp_write(FAR struct audio_lowerhalf_s *dev,
                            FAR const char *buffer, size_t buflen)
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
  int ret = -ENOTTY;
  int i;

  for (i = 0; i < priv->count; i++)
    {
      if (lower[i]->ops->write)
        {
          ret = lower[i]->ops->write(lower[i], buffer, buflen);
          if (ret != -ENOTTY)
            {
              break;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: audio_comp_reserve
 *
 * Description: Reserves a session.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_comp_reserve(FAR struct audio_lowerhalf_s *dev,
                              FAR void **session)
#else
static int audio_comp_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void **sess;
#endif
  int ret = OK;
  int i;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  sess = kmm_calloc(priv->count, sizeof(*sess));
  if (sess == NULL)
    {
      return -ENOMEM;
    }
#endif

  for (i = 0; i < priv->count; i++)
    {
      if (lower[i]->ops->reserve)
        {
#ifdef CONFIG_AUDIO_MULTI_SESSION
          int tmp = lower[i]->ops->reserve(lower[i], &sess[i]);
#else
          int tmp = lower[i]->ops->reserve(lower[i]);
#endif
          if (tmp == -ENOTTY)
            {
              continue;
            }

          ret = tmp;
          if (ret >= 0)
            {
              continue;
            }

          while (--i >= 0)
            {
              if (lower[i]->ops->release)
                {
#ifdef CONFIG_AUDIO_MULTI_SESSION
                  lower[i]->ops->release(lower[i], sess[i]);
#else
                  lower[i]->ops->release(lower[i]);
#endif
                }
            }

#ifdef CONFIG_AUDIO_MULTI_SESSION
          kmm_free(sess);
          sess = NULL;
#endif
          break;
        }
    }

#ifdef CONFIG_AUDIO_MULTI_SESSION
  *session = sess;
#endif

  return ret;
}

/****************************************************************************
 * Name: audio_comp_release
 *
 * Description: Releases the session.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_comp_release(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session)
#else
static int audio_comp_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_comp_priv_s *priv = (FAR struct audio_comp_priv_s *)dev;
  FAR struct audio_lowerhalf_s **lower = priv->lower;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void **sess = session;
#endif
  int ret = OK;
  int i;

  for (i = priv->count - 1; i >= 0; i--)
    {
      if (lower[i]->ops->release)
        {
#ifdef CONFIG_AUDIO_MULTI_SESSION
          int tmp = lower[i]->ops->release(lower[i], sess[i]);
#else
          int tmp = lower[i]->ops->release(lower[i]);
#endif
          if (tmp == -ENOTTY)
            {
              continue;
            }

          if (tmp < 0 || ret >= 0)
            {
              ret = tmp;
            }
        }
    }

#ifdef CONFIG_AUDIO_MULTI_SESSION
  kmm_free(sess);
#endif

  return ret;
}

/****************************************************************************
 * Name: audio_comp_callback
 *
 * Description:
 *   Lower-to-upper level callback for buffer dequeueing.
 *
 * Input Parameters:
 *   arg - The value of the 'priv' field from audio_lowerhalf_s.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static void audio_comp_callback(FAR void *arg, uint16_t reason,
                                FAR struct ap_buffer_s *apb, uint16_t status,
                                FAR void *session)
#else
static void audio_comp_callback(FAR void *arg, uint16_t reason,
                                FAR struct ap_buffer_s *apb, uint16_t status)
#endif
{
  FAR struct audio_comp_priv_s *priv = arg;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->export.upper(priv->export.priv, reason, apb, status, session);
#else
  priv->export.upper(priv->export.priv, reason, apb, status);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: audio_comp_initialize
 *
 * Description:
 *   Initialize the composite audio device.
 *
 * Input Parameters:
 *   name - The name of the audio device.
 *   ...  - The list of the lower half audio driver.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Note
 *   The variable argument list must be NULL terminated.
 *
 ****************************************************************************/

int audio_comp_initialize(FAR const char *name, ...)
{
  FAR struct audio_comp_priv_s *priv;
  va_list ap;
  va_list cp;
  int ret = -ENOMEM;
  int i;

  va_start(ap, name);
  va_copy(cp, ap);

  priv = kmm_zalloc(sizeof(struct audio_comp_priv_s));
  if (priv == NULL)
    {
      goto end_va;
    }

  priv->export.ops = &g_audio_comp_ops;

  while (va_arg(ap, FAR struct audio_lowerhalf_s *))
    {
      priv->count++;
    }

  priv->lower = kmm_calloc(priv->count,
                           sizeof(FAR struct audio_lowerhalf_s *));
  if (priv->lower == NULL)
    {
      goto free_priv;
    }

  for (i = 0; i < priv->count; i++)
    {
      FAR struct audio_lowerhalf_s *tmp;

      tmp = va_arg(cp, FAR struct audio_lowerhalf_s *);
      tmp->upper = audio_comp_callback;
      tmp->priv = priv;

      priv->lower[i] = tmp;
    }

  ret = audio_register(name, &priv->export);
  if (ret < 0)
    {
      goto free_lower;
    }

  va_end(ap);
  return OK;

free_lower:
  kmm_free(priv->lower);
free_priv:
  kmm_free(priv);
end_va:
  va_end(ap);
  return ret;
}
