/****************************************************************************
 * drivers/audio/audio_dma.c
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
#include <nuttx/audio/audio_dma.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>

#include <debug.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct audio_dma_s
{
  struct audio_lowerhalf_s dev;
  struct dma_chan_s *chan;
  uintptr_t src_addr;
  uintptr_t dst_addr;
  uint8_t *alloc_addr;
  uint8_t alloc_index;
  uint8_t fifo_width;
  bool playback;
  bool xrun;
  struct dq_queue_s pendq;
  apb_samp_t buffer_size;
  apb_samp_t buffer_num;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int audio_dma_getcaps(struct audio_lowerhalf_s *dev, int type,
                             struct audio_caps_s *caps);
static int audio_dma_shutdown(struct audio_lowerhalf_s *dev);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_configure(struct audio_lowerhalf_s *dev,
                               void *session,
                               const struct audio_caps_s *caps);
static int audio_dma_start(struct audio_lowerhalf_s *dev,
                           void *session);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int audio_dma_stop(struct audio_lowerhalf_s *dev, void *session);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int audio_dma_pause(struct audio_lowerhalf_s *dev,
                           void *session);
static int audio_dma_resume(struct audio_lowerhalf_s *dev,
                            void *session);
#endif
static int audio_dma_reserve(struct audio_lowerhalf_s *dev,
                             void **session);
static int audio_dma_release(struct audio_lowerhalf_s *dev,
                             void *session);
#else
static int audio_dma_configure(struct audio_lowerhalf_s *dev,
                               const struct audio_caps_s *caps);
static int audio_dma_start(struct audio_lowerhalf_s *dev);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int audio_dma_stop(struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int audio_dma_pause(struct audio_lowerhalf_s *dev);
static int audio_dma_resume(struct audio_lowerhalf_s *dev);
#endif
static int audio_dma_reserve(struct audio_lowerhalf_s *dev);
static int audio_dma_release(struct audio_lowerhalf_s *dev);
#endif
static int audio_dma_allocbuffer(struct audio_lowerhalf_s *dev,
                                 struct audio_buf_desc_s *bufdesc);
static int audio_dma_freebuffer(struct audio_lowerhalf_s *dev,
                                struct audio_buf_desc_s *bufdesc);
static int audio_dma_enqueuebuffer(struct audio_lowerhalf_s *dev,
                                   struct ap_buffer_s *apb);
static int audio_dma_ioctl(struct audio_lowerhalf_s *dev, int cmd,
                           unsigned long arg);
static void audio_dma_callback(struct dma_chan_s *chan, void *arg,
                               ssize_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audio_dma_ops =
{
  .getcaps = audio_dma_getcaps,
  .configure = audio_dma_configure,
  .shutdown = audio_dma_shutdown,
  .start = audio_dma_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop = audio_dma_stop,
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  .pause = audio_dma_pause,
  .resume = audio_dma_resume,
#endif
  .allocbuffer = audio_dma_allocbuffer,
  .freebuffer = audio_dma_freebuffer,
  .enqueuebuffer = audio_dma_enqueuebuffer,
  .ioctl = audio_dma_ioctl,
  .reserve = audio_dma_reserve,
  .release = audio_dma_release,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int audio_dma_getcaps(struct audio_lowerhalf_s *dev, int type,
                             struct audio_caps_s *caps)
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;

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

              if (audio_dma->playback)
                caps->ac_controls.b[0] = AUDIO_TYPE_OUTPUT;
              else
                caps->ac_controls.b[0] = AUDIO_TYPE_INPUT;
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

        caps->ac_channels = 2;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

            /* Report the Sample rates we support */

              caps->ac_controls.hw[0] = AUDIO_SAMP_RATE_8K  |
                                        AUDIO_SAMP_RATE_11K |
                                        AUDIO_SAMP_RATE_16K |
                                        AUDIO_SAMP_RATE_22K |
                                        AUDIO_SAMP_RATE_32K |
                                        AUDIO_SAMP_RATE_44K |
                                        AUDIO_SAMP_RATE_48K |
                                        AUDIO_SAMP_RATE_96K |
                                        AUDIO_SAMP_RATE_128K |
                                        AUDIO_SAMP_RATE_160K |
                                        AUDIO_SAMP_RATE_172K |
                                        AUDIO_SAMP_RATE_192K;
              break;
          }

        break;
   }

  /* Return the length of the audio_caps_s struct for validation of
   * proper Audio device type.
   */

  return caps->ac_len;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_configure(struct audio_lowerhalf_s *dev,
                               void *session,
                               const struct audio_caps_s *caps)
#else
static int audio_dma_configure(struct audio_lowerhalf_s *dev,
                               const struct audio_caps_s *caps)
#endif
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;
  struct dma_config_s cfg;
  int ret = -EINVAL;

  DEBUGASSERT(audio_dma && caps);
  audinfo("ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_OUTPUT:
        if (audio_dma->playback)
          {
            memset(&cfg, 0, sizeof(struct dma_config_s));
            cfg.direction = DMA_MEM_TO_DEV;
            if (audio_dma->fifo_width)
              cfg.dst_width = audio_dma->fifo_width;
            else
              cfg.dst_width = caps->ac_controls.b[2] / 8;
            ret = DMA_CONFIG(audio_dma->chan, &cfg);
          }
        break;
      case AUDIO_TYPE_INPUT:
        if (!audio_dma->playback)
          {
            memset(&cfg, 0, sizeof(struct dma_config_s));
            cfg.direction = DMA_DEV_TO_MEM;
            if (audio_dma->fifo_width)
              cfg.src_width = audio_dma->fifo_width;
            else
              cfg.src_width = caps->ac_controls.b[2] / 8;
            ret = DMA_CONFIG(audio_dma->chan, &cfg);
          }
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static int audio_dma_shutdown(struct audio_lowerhalf_s *dev)
{
  /* apps enqueued buffers, but doesn't start. stop here to
   * clear audio_dma->pendq.
   */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
  audio_dma_stop(dev, NULL);
#else
  audio_dma_stop(dev);
#endif
#endif

  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_start(struct audio_lowerhalf_s *dev, void *session)
#else
static int audio_dma_start(struct audio_lowerhalf_s *dev)
#endif
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;

  return DMA_START_CYCLIC(audio_dma->chan, audio_dma_callback, audio_dma,
                          audio_dma->dst_addr, audio_dma->src_addr,
                          audio_dma->buffer_num * audio_dma->buffer_size,
                          audio_dma->buffer_size);
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_stop(struct audio_lowerhalf_s *dev, void *session)
#else
static int audio_dma_stop(struct audio_lowerhalf_s *dev)
#endif
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;
  struct ap_buffer_s *apb;

  DMA_STOP(audio_dma->chan);

  while (!dq_empty(&audio_dma->pendq))
    {
      apb = (struct ap_buffer_s *)dq_remfirst(&audio_dma->pendq);
#ifdef CONFIG_AUDIO_MULTI_SESSION
      audio_dma->dev.upper(audio_dma->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                           apb, OK, NULL);
#else
      audio_dma->dev.upper(audio_dma->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                           apb, OK);
#endif
    }

#ifdef CONFIG_AUDIO_MULTI_SESSION
  audio_dma->dev.upper(audio_dma->dev.priv, AUDIO_CALLBACK_COMPLETE,
                       NULL, OK, NULL);
#else
  audio_dma->dev.upper(audio_dma->dev.priv, AUDIO_CALLBACK_COMPLETE,
                       NULL, OK);
#endif
  audio_dma->xrun = false;
  return OK;
}
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_pause(struct audio_lowerhalf_s *dev, void *session)
#else
static int audio_dma_pause(struct audio_lowerhalf_s *dev)
#endif
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;

  return DMA_PAUSE(audio_dma->chan);
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_resume(struct audio_lowerhalf_s *dev, void *session)
#else
static int audio_dma_resume(struct audio_lowerhalf_s *dev)
#endif
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;

  if (dq_empty(&audio_dma->pendq))
    {
      return -EINVAL;
    }

  return DMA_RESUME(audio_dma->chan);
}
#endif

static int audio_dma_allocbuffer(struct audio_lowerhalf_s *dev,
                                 struct audio_buf_desc_s *bufdesc)
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;
  struct ap_buffer_s *apb;

  if (bufdesc->numbytes != audio_dma->buffer_size)
    {
      return -EINVAL;
    }

  if (audio_dma->alloc_index == audio_dma->buffer_num)
    {
      return -ENOMEM;
    }

  if (!audio_dma->alloc_addr)
    {
      audio_dma->alloc_addr = kumm_memalign(32,
                                            audio_dma->buffer_num *
                                            audio_dma->buffer_size);
      if (!audio_dma->alloc_addr)
        {
          return -ENOMEM;
        }

      if (audio_dma->playback)
        audio_dma->src_addr = up_addrenv_va_to_pa(audio_dma->alloc_addr);
      else
        audio_dma->dst_addr = up_addrenv_va_to_pa(audio_dma->alloc_addr);
    }

  apb = kumm_zalloc(sizeof(struct ap_buffer_s));
  *bufdesc->u.pbuffer = apb;

  /* Test if the allocation was successful or not */

  if (*bufdesc->u.pbuffer == NULL)
    {
      return -ENOMEM;
    }

  /* Populate the buffer contents */

  apb->i.channels = 2;
  apb->crefs      = 1;
  apb->nmaxbytes  = audio_dma->buffer_size;
  apb->samp = audio_dma->alloc_addr +
              audio_dma->alloc_index *
              audio_dma->buffer_size;
  audio_dma->alloc_index++;
  nxmutex_init(&apb->lock);

  return sizeof(struct audio_buf_desc_s);
}

static int audio_dma_freebuffer(struct audio_lowerhalf_s *dev,
                                struct audio_buf_desc_s *bufdesc)
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;
  struct ap_buffer_s *apb;

  apb = bufdesc->u.buffer;
  audio_dma->alloc_index--;
  nxmutex_destroy(&apb->lock);
  kumm_free(apb);

  if (audio_dma->alloc_index == 0)
    {
      kumm_free(audio_dma->alloc_addr);
      audio_dma->alloc_addr = NULL;
    }

  return sizeof(struct audio_buf_desc_s);
}

static int audio_dma_enqueuebuffer(struct audio_lowerhalf_s *dev,
                                   struct ap_buffer_s *apb)
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;
  irqstate_t flags;

  if (audio_dma->playback)
    up_clean_dcache((uintptr_t)apb->samp,
                    (uintptr_t)apb->samp + apb->nbytes);

  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;

  flags = enter_critical_section();
  dq_addlast(&apb->dq_entry, &audio_dma->pendq);
  leave_critical_section(flags);

  if (audio_dma->xrun)
    {
      audio_dma->xrun = false;
      return audio_dma_resume(dev);
    }

  return OK;
}

static int audio_dma_ioctl(struct audio_lowerhalf_s *dev, int cmd,
                           unsigned long arg)
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)dev;
  struct ap_buffer_info_s *bufinfo;

  switch (cmd)
    {
      /* Report our preferred buffer size and quantity */

      case AUDIOIOC_GETBUFFERINFO:
        audinfo("AUDIOIOC_GETBUFFERINFO:\n");
        bufinfo              = (struct ap_buffer_info_s *)arg;
        bufinfo->buffer_size = audio_dma->buffer_size;
        bufinfo->nbuffers    = audio_dma->buffer_num;

        return OK;

      case AUDIOIOC_SETBUFFERINFO:
        audinfo("AUDIOIOC_GETBUFFERINFO:\n");
        bufinfo                = (struct ap_buffer_info_s *)arg;
        audio_dma->buffer_size = bufinfo->buffer_size;
        audio_dma->buffer_num  = bufinfo->nbuffers;
        kumm_free(audio_dma->alloc_addr);
        audio_dma->alloc_addr = NULL;
        audio_dma->alloc_index = 0;

        return OK;
    }

  return -ENOTTY;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_reserve(struct audio_lowerhalf_s *dev, void **session)
#else
static int audio_dma_reserve(struct audio_lowerhalf_s *dev)
#endif
{
  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_dma_release(struct audio_lowerhalf_s *dev, void *session)
#else
static int audio_dma_release(struct audio_lowerhalf_s *dev)
#endif
{
  return OK;
}

static void audio_dma_callback(struct dma_chan_s *chan,
                               void *arg, ssize_t len)
{
  struct audio_dma_s *audio_dma = (struct audio_dma_s *)arg;
  struct ap_buffer_s *apb;
  bool final = false;

  apb = (struct ap_buffer_s *)dq_remfirst(&audio_dma->pendq);
  if (!apb)
    {
      /* xrun */

      DMA_PAUSE(audio_dma->chan);
      audio_dma->xrun = true;
      return;
    }

  if (!audio_dma->playback)
    up_invalidate_dcache((uintptr_t)apb->samp,
                         (uintptr_t)apb->samp + apb->nbytes);

  if ((apb->flags & AUDIO_APB_FINAL) != 0)
    final = true;

#ifdef CONFIG_AUDIO_MULTI_SESSION
    audio_dma->dev.upper(audio_dma->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                         apb, OK, NULL);
#else
    audio_dma->dev.upper(audio_dma->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                         apb, OK);
#endif
  if (final)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      audio_dma_stop(&audio_dma->dev, NULL);
#else
      audio_dma_stop(&audio_dma->dev);
#endif
    }
  else if (dq_empty(&audio_dma->pendq))
    {
      /* xrun */

      DMA_PAUSE(audio_dma->chan);
      audio_dma->xrun = true;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct audio_lowerhalf_s *audio_dma_initialize(struct dma_dev_s *dma_dev,
                                               uint8_t chan_num,
                                               bool playback,
                                               uint8_t fifo_width,
                                               uintptr_t fifo_addr)
{
  struct audio_dma_s *audio_dma;

  if (!dma_dev)
    {
      return NULL;
    }

  audio_dma = kmm_zalloc(sizeof(struct audio_dma_s));
  if (!audio_dma)
    {
      return NULL;
    }

  audio_dma->chan = DMA_GET_CHAN(dma_dev, chan_num);
  if (!audio_dma->chan)
    {
      kmm_free(audio_dma);
      return NULL;
    }

  audio_dma->playback = playback;
  audio_dma->fifo_width = fifo_width;

  if (audio_dma->playback)
    audio_dma->dst_addr = up_addrenv_va_to_pa((void *)fifo_addr);
  else
    audio_dma->src_addr = up_addrenv_va_to_pa((void *)fifo_addr);

  audio_dma->buffer_size = CONFIG_AUDIO_BUFFER_NUMBYTES;
  audio_dma->buffer_num  = CONFIG_AUDIO_NUM_BUFFERS;
  dq_init(&audio_dma->pendq);

  audio_dma->dev.ops = &g_audio_dma_ops;
  return &audio_dma->dev;
}
