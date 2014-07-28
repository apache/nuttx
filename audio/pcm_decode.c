/****************************************************************************
 * audio/pcm_decode.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Based on the original audio framework from:
 *
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/pcm.h>

#if defined(CONFIG_AUDIO) && defined(CONFIG_AUDIO_FORMAT_PCM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#define CONFIG_PCM_DEBUG 1 /* For now */

/* Often defined and re-defined macros */

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure describes the internal state of the PCM decoder */

struct pcm_decode_s
{
  /* This is is our our appearance to the outside world.  This *MUST* be the
   * first element of the structure so that we can freely cast between types
   * struct audio_lowerhalf and struct pcm_decode_s.
   */

  struct audio_lowerhalf_s export;

  /* These are our operations that intervene between the player application
   * and the lower level driver.  Unlike the ops in the struct
   * audio_lowerhalf_s, these are writeable because we need to customize a
   * few of the methods based upon what is supported by the the lower level
   * driver.
   */

  struct audio_ops_s ops;

  /* This is the contained, low-level DAC-type device and will receive the
   * decoded PCM audio data.
   */

  FAR struct audio_lowerhalf_s *lower;

  /* Session returned from the lower level driver */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  FAR void *session;
#endif

  /* These are values extracted from WAV file header */

  uint32_t samprate;               /* 8000, 44100, ... */
  uint32_t byterate;               /* samprate * nchannels * bpsamp / 8 */
  uint8_t  align;                  /* nchannels * bpsamp / 8 */
  uint8_t  bpsamp;                 /* Bits per sample: 8 bits = 8, 16 bits = 16 */
  uint8_t  nchannels;              /* Mono=1, Stereo=2 */
  bool     streaming;              /* Streaming PCM data chunk */

#ifndef CONFIG_AUDIO_EXCLUDE_FFORWARD
  /* Fast forward support */

  uint8_t  subsample;              /* Fast forward rate: See AUDIO_SUBSAMPLE_* defns */
  uint8_t  skip;                   /* Number of samples to be skipped */
  uint8_t  npartial;               /* Size of the partial sample */
  uint8_t  partial[4];             /* Holds the partial sample */

  /* REVISIT: I think we could get rid of all of this special buffer
   * processing.  It seems like we should be able to do the sub-sampling
   * in place without any additional buffers.
   */

  struct audio_buf_desc_s bufdesc; /* Allocated buffer description */
  FAR struct ap_buffer_s *ffwd;    /* Audio buffer being subsampled into */
  FAR struct ap_buffer_s *next;    /* Next available, empty audio buffer */
  FAR struct ap_buffer_s *apb;     /* Allocated audio buffer */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions *********************************************************/

#ifdef CONFIG_PCM_DEBUG
static void pcm_dump(FAR const struct wav_header_s *wav);
#else
#  define pcm_dump(w)
#endif

#ifdef CONFIG_ENDIAN_BIG
static uint16_t pcm_leuint16(uint16_t value);
static uint16_t pcm_leuint32(uint32_t value);
#else
#  define pcm_leuint16(v) (v)
#  define pcm_leuint32(v) (v)
#endif

static inline bool pcm_validwav(FAR const struct wav_header_s *wav);
static bool pcm_parsewav(FAR struct pcm_decode_s *priv, uint8_t *data);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static void pcm_upper_callback(FAR struct pcm_decode_s *priv,
              uint16_t reason, FAR struct ap_buffer_s *apb,
              uint16_t status, FAR void *session);
#else
static void pcm_upper_callback(FAR struct pcm_decode_s *priv,
              uint16_t reason, FAR struct ap_buffer_s *apb,
              uint16_t status);
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_FFORWARD
static int  pcm_subsample_configure(FAR struct pcm_decode_s *priv,
              uint8_t subsample);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static void pcm_subsample_callback(FAR struct pcm_decode_s *priv,
              uint16_t reason, FAR struct ap_buffer_s *apb,
              uint16_t status, FAR void *session);
#else
static void pcm_subsample_callback(FAR struct pcm_decode_s *priv,
              uint16_t reason, FAR struct ap_buffer_s *apb,
              uint16_t status);
#endif
static FAR struct ap_buffer_s *pcm_subsample(FAR struct pcm_decode_s *priv,
              FAR struct ap_buffer_s *apb);
#endif

/* struct audio_lowerhalf_s methods *****************************************/

static int  pcm_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
              FAR struct audio_caps_s *caps);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_configure(FAR struct audio_lowerhalf_s *dev,
              FAR void *session, FAR const struct audio_caps_s *caps);
#else
static int  pcm_configure(FAR struct audio_lowerhalf_s *dev,
              FAR const struct audio_caps_s *caps);
#endif

static int  pcm_shutdown(FAR struct audio_lowerhalf_s *dev);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_start(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int  pcm_start(FAR struct audio_lowerhalf_s *dev);
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int  pcm_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int  pcm_pause(FAR struct audio_lowerhalf_s *dev);
#endif

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int  pcm_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif

static int  pcm_allocbuffer(FAR struct audio_lowerhalf_s *dev,
              FAR struct audio_buf_desc_s *apb);
static int  pcm_freebuffer(FAR struct audio_lowerhalf_s *dev,
              FAR struct audio_buf_desc_s *apb);
static int  pcm_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
              FAR struct ap_buffer_s *apb);
static int  pcm_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
              FAR struct ap_buffer_s *apb);
static int  pcm_ioctl(FAR struct audio_lowerhalf_s *dev,
              int cmd, unsigned long arg);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_reserve(FAR struct audio_lowerhalf_s *dev, FAR void **session);
#else
static int  pcm_reserve(FAR struct audio_lowerhalf_s *dev);
#endif

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  pcm_release(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int  pcm_release(FAR struct audio_lowerhalf_s *dev);
#endif

/* Audio callback */

#ifdef CONFIG_AUDIO_MULTI_SESSION
static void pcm_callback(FAR void *arg, uint16_t reason,
              FAR struct ap_buffer_s *apb, uint16_t status,
              FAR void *session);
#else
static void pcm_callback(FAR void *arg, uint16_t reason,
              FAR struct ap_buffer_s *apb, uint16_t status);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcm_dump
 *
 * Description:
 *   Dump a WAV file header.
 *
 ****************************************************************************/

#ifdef CONFIG_PCM_DEBUG
static void pcm_dump(FAR const struct wav_header_s *wav)
{
  dbg( "Wave file header\n");
  dbg( "  Header Chunk:\n");
  dbg( "    Chunk ID:        0x%08x\n", wav->hdr.chunkid);
  dbg( "    Chunk Size:      %u\n",     wav->hdr.chunklen);
  dbg( "    Format:          0x%08x\n", wav->hdr.format);
  dbg( "  Format Chunk:\n");
  dbg( "    Chunk ID:        0x%08x\n", wav->fmt.chunkid);
  dbg( "    Chunk Size:      %u\n",     wav->fmt.chunklen);
  dbg( "    Audio Format:    0x%04x\n", wav->fmt.format);
  dbg( "    Num. Channels:   %d\n",     wav->fmt.nchannels);
  dbg( "    Sample Rate:     %u\n",     wav->fmt.samprate);
  dbg( "    Byte Rate:       %u\n",     wav->fmt.byterate);
  dbg( "    Block Align:     %d\n",     wav->fmt.align);
  dbg( "    Bits Per Sample: %d\n",     wav->fmt.bpsamp);
  dbg( "  Data Chunk:\n");
  dbg( "    Chunk ID:        0x%08x\n", wav->data.chunkid);
  dbg( "    Chunk Size:      %u\n",     wav->data.chunklen);
}
#endif

/****************************************************************************
 * Name: pcm_leuint16
 *
 * Description:
 *   Get a 16-bit value stored in little endian order for a big-endian
 *   machine.
 *
 ****************************************************************************/

#ifdef CONFIG_ENDIAN_BIG
static uint16_t pcm_leuint16(uint16_t value)
{
  return (((value & 0x00ff) << 8) |
          ((value >> 8) & 0x00ff));
}
#endif

/****************************************************************************
 * Name: pcm_leuint16
 *
 * Description:
 *   Get a 16-bit value stored in little endian order for a big-endian
 *   machine.
 *
 ****************************************************************************/

#ifdef CONFIG_ENDIAN_BIG
static uint16_t pcm_leuint32(uint32_t value)
{
  return (((value & 0x000000ff) << 24) |
          ((value & 0x0000ff00) <<  8) |
          ((value & 0x00ff0000) >>  8) |
          ((value & 0xff000000) >> 24));
}
#endif

/****************************************************************************
 * Name: pcm_validwav
 *
 * Description:
 *   Return true if this is a valid WAV file header
 *
 ****************************************************************************/

static inline bool pcm_validwav(FAR const struct wav_header_s *wav)
{
  return (wav->hdr.chunkid  == WAV_HDR_CHUNKID  &&
          wav->hdr.format   == WAV_HDR_FORMAT   &&
          wav->fmt.chunkid  == WAV_FMT_CHUNKID  &&
          wav->fmt.chunklen == WAV_FMT_CHUNKLEN &&
          wav->fmt.format   == WAV_FMT_FORMAT   &&
          wav->fmt.nchannels < 256              &&
          wav->fmt.align     < 256              &&
          wav->fmt.bpsamp    < 256              &&
          wav->data.chunkid == WAV_DATA_CHUNKID);
}

/****************************************************************************
 * Name: pcm_parsewav
 *
 * Description:
 *   Parse and verify the WAV file header.
 *
 ****************************************************************************/

static bool pcm_parsewav(FAR struct pcm_decode_s *priv, uint8_t *data)
{
  FAR const struct wav_header_s *wav = (FAR const struct wav_header_s *)data;
  struct wav_header_s localwav;
  bool ret;

  /* Transfer the purported WAV file header into our stack storage,
   * correcting for endian issues as needed.
   */

  localwav.hdr.chunkid   = pcm_leuint32(wav->hdr.chunkid);
  localwav.hdr.chunklen  = pcm_leuint32(wav->hdr.chunklen);
  localwav.hdr.format    = pcm_leuint32(wav->hdr.format);

  localwav.fmt.chunkid   = pcm_leuint32(wav->fmt.chunkid);
  localwav.fmt.chunklen  = pcm_leuint32(wav->fmt.chunklen);
  localwav.fmt.format    = pcm_leuint16(wav->fmt.format);
  localwav.fmt.nchannels = pcm_leuint16(wav->fmt.nchannels);
  localwav.fmt.samprate  = pcm_leuint32(wav->fmt.samprate);
  localwav.fmt.byterate  = pcm_leuint32(wav->fmt.byterate);
  localwav.fmt.align     = pcm_leuint16(wav->fmt.align);
  localwav.fmt.bpsamp    = pcm_leuint16(wav->fmt.bpsamp);

  localwav.data.chunkid  = pcm_leuint32(wav->data.chunkid);
  localwav.data.chunklen = pcm_leuint32(wav->data.chunklen);

  /* Dump the converted wave header information */

  pcm_dump(&localwav);

  /* Check if the file is a valid WAV header */

  ret = pcm_validwav(&localwav);
  if (ret)
    {
      /* Yes... pick off the relevant format values and save then in the
       * device structure.
       */

      priv->samprate    = localwav.fmt.samprate;  /* 8000, 44100, ... */
      priv->byterate    = localwav.fmt.byterate;  /* samprate * nchannels * bpsamp / 8 */
      priv->align       = localwav.fmt.align;     /* nchannels * bpsamp / 8 */
      priv->bpsamp      = localwav.fmt.bpsamp;    /* Bits per sample: 8 bits = 8, 16 bits = 16 */
      priv->nchannels   = localwav.fmt.nchannels; /* Mono=1, Stereo=2 */

#ifndef CONFIG_AUDIO_EXCLUDE_FFORWARD
      /* We are going to subsample, there then are some restrictions on the
       * number of channels and sample sizes that we can handle.
       */

      if (priv->bpsamp != 8 && priv->bpsamp != 16)
        {
          auddbg("ERROR: Cannot support bits per sample of %d in this mode\n",
                 priv->bpsamp);
          return -EINVAL;
        }

      if (priv->nchannels != 1 && priv->nchannels != 2)
        {
          auddbg("ERROR: Cannot support number of channles of %d in this mode\n",
                 priv->nchannels);
          return -EINVAL;
        }

      DEBUGASSERT(priv->align == priv->nchannels * priv->bpsamp / 8);
#endif
    }

  /* And return true if the the file is a valid WAV header file */

  return ret;
}

/****************************************************************************
 * Name: pcm_upper_callback
 *
 * Description:
 *   The lower level driver has just returned a buffer that belongs to the
 *   uppder level.  We need to forward the buffer to the upper level for
 *   disposition.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static void pcm_upper_callback(FAR struct pcm_decode_s *priv,
              uint16_t reason, FAR struct ap_buffer_s *apb,
              uint16_t status, FAR void *session)
#else
static void pcm_upper_callback(FAR struct pcm_decode_s *priv,
              uint16_t reason, FAR struct ap_buffer_s *apb,
              uint16_t status)
#endif
{
  DEBUGASSERT(priv && apb && priv->export.upper);

  /* The buffer belongs to to an upper level.  Just forward the event to
   * the next level up.
   */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->export.upper(priv->export.priv, reason, apb, status, session);
#else
  priv->export.upper(priv->export.priv, reason, apb, status);
#endif
}

/****************************************************************************
 * Name: pcm_subsample_configure
 *
 * Description:
 *   Configure to perform sub-sampling (or not) on the following audio
 *   buffers.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_FFORWARD
static int pcm_subsample_configure(FAR struct pcm_decode_s *priv,
                                   uint8_t subsample)
{
  FAR struct audio_lowerhalf_s *lower = priv->lower;
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  struct ap_buffer_info_s bufinfo;
#endif
  int ret;

  audvdbg("subsample: %d\n", subsample);

  /* Three possibilities:
   *
   * 1. We were playing normally and we have been requested to begin fast
   *    forwarding.
   */

  if (priv->subsample == AUDIO_SUBSAMPLE_NONE)
    {
      /* There should be no audio buffer allocated in this case */

      DEBUGASSERT(priv->ffwd == NULL);

      /* Ignore request to stop fast forwarding if we are already
       * fast forwarding.
       */

      if (subsample != AUDIO_SUBSAMPLE_NONE)
        {
          /* Check if we already have an audio buffer.  This might happen in
           * certain race conditions were we exit and return to fast forward
           * mode while the audio buffer is in possession of the lower level
           * driver.
           */

          audvdbg("Start subsampling: apb=%p\n", priv->apb);

          /* REVISIT: I think we could get rid of this special buffer.  It
           * seems like we should be able to do the sub-sampling in place
           * without any additional buffers.
           */

          if (priv->apb == NULL)
            {
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
              /* Get the lower level driver's preferred buffer properties */

              DEBUGASSERT(lower->ioctl);
              ret = lower->ioctl(lower, AUDIOIOC_GETBUFFERINFO,
                                 (unsigned long)&bufinfo))
              if (ret < 0)
                {
                  /* Driver doesn't report it's buffer size.  Use defaults. */

                  bufinfo.buffer_size = CONFIG_AUDIO_BUFFER_NUMBYTES;
                  bufinfo.nbuffers    = CONFIG_AUDIO_NUM_BUFFERS;
                }
#endif

              /* Configure the buffer descriptor */

#ifdef CONFIG_AUDIO_MULTI_SESSION
              priv->bufdesc.session    = priv->session;
#endif
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
              priv->bufdesc.numbytes   = bufinfo.buffer_size;
#else
              priv->bufdesc.numbytes   = CONFIG_AUDIO_BUFFER_NUMBYTES;
#endif
              priv->bufdesc.u.ppBuffer = &priv->apb;

              /* Perform the allocation */

              if (lower->ops->allocbuffer)
                {
                  ret = lower->ops->allocbuffer(lower, &priv->bufdesc);
                }
              else
                {
                  ret = apb_alloc(&priv->bufdesc);
                }

              /* Did we get an audio buffer? */

              if (ret < 0 || priv->apb == NULL)
                {
                  auddbg("ERROR: Failed to allocate an audio buffer\n");
                  return -ENOMEM;
                }
            }

          /* Save the current subsample setting. Subsampling will begin on
           * then next audio buffer that we receive.
           */

          priv->ffwd      = priv->apb;
          priv->next      = NULL;
          priv->npartial  = 0;
          priv->skip      = 0;
          priv->subsample = subsample;
        }
    }

  /* 2. Were already fast forwarding and we have been asked to change the
   *    subsampling rate.
   */

  else if (subsample != AUDIO_SUBSAMPLE_NONE)
    {
      /* We should already have an audio buffer allocated in this case */

      DEBUGASSERT(priv->apb != NULL && priv->ffwd != NULL);

      /* Just save the current subsample setting.  It will take effect
       * on the next audio buffer that we receive.
       */

       priv->subsample = subsample;
    }

  /* 3. We were already fast forwarding and we have been asked to return to
   *    normal play.
   */

  else if (subsample != AUDIO_SUBSAMPLE_NONE)
    {
      DEBUGASSERT(priv->apb != NULL && priv->ffwd != NULL);

      audvdbg("Stop subsampling\n");

      /* Are still holding on to buffers that belong the upper level? */

      if (priv->ffwd && priv->ffwd != priv->apb)
        {
#ifdef CONFIG_AUDIO_MULTI_SESSION
          pcm_upper_callback(priv, AUDIO_CALLBACK_DEQUEUE, priv->ffwd, OK, priv->session);
#else
          pcm_upper_callback(priv, AUDIO_CALLBACK_DEQUEUE, priv->ffwd, OK);
#endif
          priv->ffwd = NULL;
        }

      /* We need to free the our working audio buffer.  But we also need
       * to be careful:  The lower level audio driver may have it now.
       *
       * If the lower level driver has the audio buffer, then we will defer
       * freeing it until it is returned to us.
       */

      if (priv->ffwd)
        {
          /* We have it.  We can free the audio buffer.  NOTE:  This audio
           * buffer might not be empty, but we will discard it anyway.
           * Playing will begin at normal speed at the next, full audio
           * buffer.
           */

          if (lower->ops->freebuffer)
            {
              (void)lower->ops->freebuffer(lower, &priv->bufdesc);
            }
          else
            {
              apb_free(priv->apb);
            }

          priv->apb   = NULL;
          priv->ffwd  = NULL;
        }

      /* And indicate that we are in normal play mode.  This will take
       * effect when the next audio buffer is received.
       */

      priv->next      = NULL;
      priv->npartial  = 0;
      priv->skip      = 0;
      priv->subsample = AUDIO_SUBSAMPLE_NONE;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: pcm_subsample_callback
 *
 * Description:
 *   Our working audio buffer has been returned from the lower level driver.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_FFORWARD
#ifdef CONFIG_AUDIO_MULTI_SESSION
static void pcm_subsample_callback(FAR struct pcm_decode_s *priv,
                                   uint16_t reason,
                                   FAR struct ap_buffer_s *apb,
                                   uint16_t status, FAR void *session)
#else
static void pcm_subsample_callback(FAR struct pcm_decode_s *priv,
                                   uint16_t reason,
                                   FAR struct ap_buffer_s *apb,
                                   uint16_t status)
#endif
{
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv->apb);

  audvdbg("apb=%p subsample=%d\n", apb, priv->subsample);

  /* Mark the buffer as empty */

  apb->nbytes  = 0;
  apb->curbyte = 0;

  /* Possibilities:
   * 1. We are no longer streaming and we just need to free the buffer.
   *    This case happens if we exited fast forward mode while the lower-
   *    level driver had possession of or working audio buffer.
   */

  if (priv->subsample == AUDIO_SUBSAMPLE_NONE)
    {
      /* Free the working buffer */

      lower = priv->lower;
      if (lower->ops->freebuffer)
        {
          (void)lower->ops->freebuffer(lower, &priv->bufdesc);
        }
      else
        {
          apb_free(apb);
        }

      priv->apb = NULL;
    }

  /* We are still streaming.  At any given time, our working audio may be
   * (a) the current working buffer, (a) the next working buffer or it, or
   * (c) in possession of the lower level driver.  None of these are the
   * case now since we just got our working audio buffer back from the
   *
   * 2. We have no fast forward working buffer.  Use it buffer as our
   *    fast forward working buffer.  This should never happen.
   */

  else if (priv->ffwd == NULL)
    {
      /* Set the returned buffer as the current working buffer */

      priv->ffwd = apb;
    }

  /* 2. We already have a fast word working buffer, but no next working
   *    buffer.  Save out working audio buffer as the new next buffer.
   */

  else
    {
      /* Set the returned buffer as the next working buffer
       *
       * REVISIT:  This extra buffer is only used on the first buffer. There
       * is something wrong:
       *
       * 1. First buffer: Our private buffer is the working buffer.  The
       *    first audio buffer we receive is compressed into the working
       *    buffer and the working buffer goes to the lower level.  The
       *    first buffer becomes next working buffer.
       * 2. Next buffers: Are compressed into the working buffer, sent
       *    to the lower level, and become the next working buffer, etc.
       * 3. The private buffer eventually comes back here and is saved
       *    as the "next" buffer, but it is never used again.
       */

      DEBUGASSERT(priv->next == NULL);
      priv->next = apb;
    }
}
#endif

/****************************************************************************
 * Name: pcm_subsample
 *
 * Description:
 *   Given a newly received audio buffer, perform subsampling into our local
 *   working audio buffer.  Return the subsampled audio buffer to send to
 *   the lower level driver.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_FFORWARD
static FAR struct ap_buffer_s *pcm_subsample(FAR struct pcm_decode_s *priv,
                                             FAR struct ap_buffer_s *apb)
{
  FAR struct ap_buffer_s *work;
  FAR const uint8_t *src;
  FAR uint8_t *dest;
  unsigned int destsize;
  unsigned int srcsize;
  unsigned int skipsize;
  unsigned int i;

  /* Are we subsampling? */

  if (priv->subsample == AUDIO_SUBSAMPLE_NONE)
    {
      /* No.. do nothing to the buffer */

      return apb;
    }

  /* Yes.. then make sure that we have the required buffers in place */

  DEBUGASSERT(priv->ffwd && priv->apb);

  /* Yes.. we will need to subsample the newly received buffer. */

  work = priv->ffwd;
  dest = &work->samp[work->nbytes];

  DEBUGASSERT(work->nmaxbytes >= priv->align);

  /* This is the number of bytes that we need to skip between samples */

  skipsize = priv->align * (priv->subsample - 1);

  /* Let's deal with any partial sample data from the last buffer */

  if (priv->npartial > 0)
    {
      /* How much data is available in the new audio buffer */

      srcsize = apb->nbytes - apb->curbyte;

      /* Let's get an impossible corner case out of the way.  What if we
       * received a tiny audio buffer.  So small, that it (plus any previous
       * sample) is smaller than one sample.
       */

      if (priv->npartial + srcsize < priv->align)
        {
          dest = &priv->partial[priv->npartial];
          src  = &apb->samp[apb->curbyte];
          memcpy(dest, src, srcsize);

          priv->npartial += srcsize;
          apb->curbyte   += srcsize;
          goto exit_apb_empty;
        }

      /* No, we do at least have enough to complete the partial sample */
      /* Copy the first part of the partial sample that was saved from
       * the last buffer.
       */

      src = priv->partial;
      for (i = 0; i < priv->npartial; i++)
        {
          *dest++ = *src++;
        }

      priv->npartial = 0;

      /* Now copy data from new audio buffer to complete the sample */

      src  = &apb->samp[apb->curbyte];
      for (i = 0; i < priv->align; i++)
        {
          *dest++ = *src++;
          apb->curbyte++;
        }

      /* Update the number of bytes in the working buffer and reset the skip
       * value
       */

      work->nbytes += skipsize;
      priv->skip    = skipsize;
    }

  /* Now loop until either (1) the working audio buffer is full, or (2)
   * the newly received audio buffer is empty.
   */

  src      = &apb->samp[apb->curbyte];
  srcsize  = apb->nbytes - apb->curbyte;
  destsize = work->nmaxbytes - work->nbytes;

  while (srcsize > 0 && destsize > 0)
    {
      /* Do we need to skip ahead in the buffer? */

      if (priv->skip > 0)
        {
          /* How much can we skip in this buffer?  Depends on the smaller
           * of (1) the number of bytes that we need to skip and (2) the
           * number of bytes available in the newly received audio buffer.
           */

          unsigned int discard = MIN(priv->skip, srcsize);

          priv->skip   -= discard;
          apb->curbyte += discard;
          srcsize      -= discard;

          /* Did we skip to the end of the audio buffer? */

          if (srcsize <= 0)
            {
              goto exit_apb_empty;
            }
        }

      /* We have skipped over the required number of bytes and we are ready
       * to take the next sample.  Is there space for a whole sample in the
       * buffer?
       */

      else if (srcsize < priv->align)
        {
          /* No.. copy the partial sample to the partial sample buffer */

          dest = priv->partial;
          for (i = 0; i < srcsize; i++)
            {
              *dest++ = *src++;
            }

          priv->npartial = srcsize;
          apb->curbyte  += srcsize;

          /* We are finished with this audio buffer.  On the next buffer,
           * the fact that skip == 0 and that priv->npartial > 0 will kick
           * off the re-synchronization logic at the beginning of this
           * function.
           */

          goto exit_apb_empty;
        }

      /* Plenty of space!  Copy the sample from the audio buffer into the
       * working buffer.
       */

      else
        {
          /* Now copy the sample from new audio buffer to working buffer */

          for (i = 0; i < priv->align; i++)
            {
              *dest++ = *src++;
            }

          /* Update indices and sizes and reset the skip value */

          apb->curbyte  += priv->align;
          srcsize       -= priv->align;

          work->nbytes  += priv->align;
          destsize      -= priv->align;

          priv->skip     = skipsize;
        }
    }

  /* We get here in one of two cases.  Either (1) the source audio buffer
   * became empty, and/or (2) the working audio buffer became which.  Which
   * is the case?
   *
   * REVISIT:  Should there be a threshold here so that we do not send
   * really tiny buffers to the lower-level driver.
   */

  if (srcsize > 0)
    {
      /* How could the working buffer be empty in this case? */

      DEBUGASSERT(work->nbytes > 0);

      /* This seems overly complex.  We either need another buffer or else
       * we have to do this:  Sub-ample in place.
       */

      dest     = apb->samp;
      destsize = apb->nmaxbytes;

      while (srcsize > 0)
        {
          /* Do we need to skip ahead in the buffer? */

          if (priv->skip > 0)
            {
              /* How much can we skip in this buffer?  Depends on the
               * smaller of (1) the number of bytes that we need to skip
               * and (2) the number of bytes available in the audio buffer.
               */

              unsigned int discard = MIN(priv->skip, srcsize);

              priv->skip   -= discard;
              srcsize      -= discard;

              /* Did we skip to the end of the audio buffer? */

              if (srcsize <= 0)
                {
                  break;
                }
            }

          /* We have skipped over the required number of bytes and we are
           * ready to take the next sample.  Is there space for a whole
           * sample in the buffer?
           */

          else if (srcsize < priv->align)
            {
              /* No.. copy the partial sample to the partial sample buffer */

              dest = priv->partial;
              for (i = 0; i < srcsize; i++)
                {
                  *dest++ = *src++;
                }

              priv->npartial = srcsize;
              break;
            }

          /* Plenty of space!  Copy the sample from the end of the audio
           * buffer to the beginning.
           */

          else
            {
              /* Now copy the sample from new audio buffer to working buffer */

              for (i = 0; i < priv->align; i++)
                {
                  *dest++ = *src++;
                }

              /* Update indices and sizes and reset the skip value */

              srcsize       -= priv->align;
              destsize      -= priv->align;
              priv->skip     = skipsize;
            }
        }

      /* Set the buffer that we just emptied as the new working buffer */

      apb->curbyte = 0;
      apb->nbytes  = apb->nmaxbytes - destsize;
      priv->ffwd   = apb;

      /* Return the working buffer */

      return work;
    }

exit_apb_empty:
  /* We get here in the case that the source audio buffer is empty.  The
   * working buffer may be empty or non-empty.
   */

  /* Is there anything in the working buffer?
   *
   * REVISIT:  Should there be a threshold here so that we do not send
   * really tiny buffers to the lower-level driver.
   */

  if (work->nbytes > 0)
    {
      /* Set the buffer that we just emptied as the new working buffer */

      apb->curbyte = 0;
      apb->nbytes  = 0;
      priv->ffwd   = apb;

      /* Return the working buffer */

      return work;
    }

  /* We don't have any data to forward to the lower level driver */

  return NULL;
}
#endif

/****************************************************************************
 * Name: pcm_getcaps
 *
 * Description:
 *   This method is called to retrieve the lower-half device capabilities.
 *   It will be called with device type AUDIO_TYPE_QUERY to request the
 *   overall capabilities, such as to determine the types of devices supported
 *   audio formats supported, etc.  Then it may be called once or more with
 *   reported supported device types to determine the specific capabilities
 *   of that device type (such as MP3 encoder, WMA encoder, PCM output, etc.).
 *
 ****************************************************************************/

static int pcm_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                       FAR struct audio_caps_s *caps)
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;
  int ret;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->getcaps);

  /* Get the capabilities of the lower-level driver */

  ret = lower->ops->getcaps(lower, type, caps);
  if (ret < 0)
    {
      auddbg("Lower getcaps() failed: %d\n", ret);
      return ret;
    }

  /* Modify the capabilities reported by the lower driver:  PCM is the only
   * supported format that we will report, regardless of what the lower driver
   * reported.
   */

  if (caps->ac_subtype == AUDIO_TYPE_QUERY)
    {
      caps->ac_format.hw = (1 << (AUDIO_FMT_PCM - 1));
    }

  return caps->ac_len;
}

/****************************************************************************
 * Name: pcm_configure
 *
 * Description:
 *   This method is called to bind the lower-level driver to the upper-level
 *   driver and to configure the driver for a specific mode of
 *   operation defined by the parameters selected in supplied device caps
 *   structure.  The lower-level device should perform any initialization
 *   needed to prepare for operations in the specified mode.  It should not,
 *   however, process any audio data until the start method is called.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_configure(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session, FAR const struct audio_caps_s *caps)
#else
static int pcm_configure(FAR struct audio_lowerhalf_s *dev,
                         FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;
#ifndef CONFIG_AUDIO_EXCLUDE_FFORWARD
  int ret;
#endif

  DEBUGASSERT(priv);

#ifndef CONFIG_AUDIO_EXCLUDE_FFORWARD
  /* Pick off commands to perform sub-sampling.  Those are done by this
   * decoder.  All of configuration settings are handled by the lower level
   * audio driver.
   */

  if (caps->ac_type == AUDIO_TYPE_PROCESSING &&
      caps->ac_format.hw == AUDIO_PU_SUBSAMPLE_FORWARD)
    {
      /* Configure subsampling */

      ret = pcm_subsample_configure(priv, caps->ac_controls.b[0]);
      if (ret < 0)
        {
          auddbg("ERROR: pcm_subsample_configure failed: %d\n", ret);
        }

      /* Return to avoid forwarding the configuration to the lower level
       * driver.
       */

      return ret;
    }
#endif

  /* Defer all other operations to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->configure);

  audvdbg("Defer to lower configure\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  return lower->ops->configure(lower, session, caps);
#else
  return lower->ops->configure(lower, caps);
#endif
}

/****************************************************************************
 * Name: pcm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   should stop processing audio data, including terminating any active
 *   output generation.  It should also disable the audio hardware and put
 *   it into the lowest possible power usage state.
 *
 *   Any enqueued Audio Pipeline Buffers that have not been processed / dequeued
 *   should be dequeued by this function.
 *
 ****************************************************************************/

static int pcm_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->start);

  audvdbg("Defer to lower shutdown\n");
  return lower->ops->shutdown(lower);
}

/****************************************************************************
 * Name: pcm_start
 *
 * Description:
 *   Start audio streaming in the configured mode.  For input and synthesis
 *   devices, this means it should begin sending streaming audio data.  For output
 *   or processing type device, it means it should begin processing of any enqueued
 *   Audio Pipeline Buffers.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->start);

  audvdbg("Defer to lower start\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  return lower->ops->start(lower, session);
#else
  return lower->ops->start(lower);
#endif
}

/****************************************************************************
 * Name: pcm_stop
 *
 * Description:
 *   Stop audio streaming and/or processing of enqueued Audio Pipeline
 *   Buffers
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm_stop(FAR struct audio_lowerhalf_s *dev)
#endif
#endif
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->stop);

  audvdbg("Defer to lower stop\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  return lower->ops->stop(lower, session);
#else
  return lower->ops->stop(lower);
#endif
}

/****************************************************************************
 * Name: pcm_pause
 *
 * Description:
 *   Pause the audio stream.  Should keep current playback context active
 *   in case a resume is issued.  Could be called and then followed by a stop.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->pause);

  audvdbg("Defer to lower pause\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  return lower->ops->pause(lower, session);
#else
  return lower->ops->pause(lower);
#endif
}

/****************************************************************************
 * Name: pcm_resume
 *
 * Description:
 *   Resumes audio streaming after a pause.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm_resume(FAR struct audio_lowerhalf_s *dev)
#endif
#endif
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->resume);

  audvdbg("Defer to lower resume\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  return lower->ops->resume(lower, session);
#else
  return lower->ops->resume(lower);
#endif
}

/****************************************************************************
 * Name: pcm_allocbuffer
 *
 * Description:
 *   Allocate an audio pipeline buffer.  This routine provides the
 *   lower-half driver with the opportunity to perform special buffer
 *   allocation if needed, such as allocating from a specific memory
 *   region (DMA-able, etc.).  If not supplied, then the top-half
 *   driver will perform a standard kumalloc using normal user-space
 *   memory region.
 *
 ****************************************************************************/

static int pcm_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                           FAR struct audio_buf_desc_s *apb)
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->allocbuffer);

  audvdbg("Defer to lower allocbuffer\n");
  return lower->ops->allocbuffer(lower, apb);
}

/****************************************************************************
 * Name: pcm_freebuffer
 *
 * Description:
 *   Free an audio pipeline buffer.  If the lower-level driver provides an
 *   allocbuffer routine, it should also provide the freebuffer routine to
 *   perform the free operation.
 *
 ****************************************************************************/

static int pcm_freebuffer(FAR struct audio_lowerhalf_s *dev,
                          FAR struct audio_buf_desc_s *apb)
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->freebuffer);

  audvdbg("Defer to lower freebuffer, apb=%p\n", apb);
  return lower->ops->freebuffer(lower, apb);
}

/****************************************************************************
 * Name: pcm_enqueuebuffer
 *
 * Description:
 *   Enqueue a buffer for processing.  This is a non-blocking enqueue
 *   operation.  If the lower-half driver's buffer queue is full, then it
 *   should return an error code of -ENOMEM, and the upper-half driver can
 *   decide to either block the calling thread or deal with it in a non-
 *   blocking manner.
 *
 *   For each call to enqueuebuffer, the lower-half driver must call
 *   audio_dequeuebuffer when it is finished processing the bufferr, passing
 *   the previously enqueued apb and a dequeue status so that the upper-half
 *   driver can decide if a waiting thread needs to be release, if the
 *   dequeued buffer should be passed to the next block in the Audio
 *   Pipeline, etc.
 *
 ****************************************************************************/

static int pcm_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                             FAR struct ap_buffer_s *apb)
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;
  apb_samp_t bytesleft;
  int ret;

  DEBUGASSERT(priv);
  audvdbg("Received buffer %p, streaming=%d\n", apb, priv->streaming);

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->enqueuebuffer && lower->ops->configure);

  /* Are we streaming yet? */

  if (priv->streaming)
    {
      /* Yes, we are streaming */

#ifndef CONFIG_AUDIO_EXCLUDE_FFORWARD
      audvdbg("Received: apb=%p curbyte=%d nbytes=%d\n",
              apb, apb->curbyte, apb->nbytes);

      /* Perform any necessary subsampling operations */

      apb = pcm_subsample(priv, apb);
      if (apb)
#endif
        {
          /* Then give the audio buffer to the lower driver */

          audvdbg("Pass to lower enqueuebuffer: apb=%p curbyte=%d nbytes=%d\n",
                  apb, apb->curbyte, apb->nbytes);

          return lower->ops->enqueuebuffer(lower, apb);
        }

      return OK;
    }

  /* No.. then this must be the first buffer that we have seen (since we
   * will error out out if the first buffer is smaller than the WAV file
   * header.  There is no attempt to reconstruct the full header from
   * fragments in multiple, tiny audio buffers).
   */

  bytesleft = apb->nbytes - apb->curbyte;
  audvdbg("curbyte=%d nbytes=%d nmaxbytes=%d bytesleft=%d\n",
          apb->curbyte, apb->nbytes, apb->nmaxbytes, bytesleft);

  if (bytesleft >= sizeof(struct wav_header_s))
    {
      /* Parse and verify the candidate WAV file header */

      if (pcm_parsewav(priv, &apb->samp[apb->curbyte]))
        {
          struct audio_caps_s caps;

          /* Now we are streaming */

          priv->streaming = true;

          /* Configure the lower level for the number of channels, bitrate,
           * and sample bitwidth.
           */

          DEBUGASSERT(priv->samprate < 65535);

          caps.ac_len            = sizeof(struct audio_caps_s);
          caps.ac_type           = AUDIO_TYPE_OUTPUT;
          caps.ac_channels       = priv->nchannels;

          caps.ac_controls.hw[0] = (uint16_t)priv->samprate;
          caps.ac_controls.b[2]  = priv->bpsamp;

#ifdef CONFIG_AUDIO_MULTI_SESSION
          ret = lower->ops->configure(lower, priv->session, &caps);
#else
          ret = lower->ops->configure(lower, &caps);
#endif
          if (ret < 0)
            {
              auddbg("ERROR: Failed to set PCM configuration: %d\n", ret);
              return ret;
            }

          /* Bump up the data offset */

          apb->curbyte += sizeof(struct wav_header_s);

#ifndef CONFIG_AUDIO_EXCLUDE_FFORWARD
          audvdbg("Begin streaming: apb=%p curbyte=%d nbytes=%d\n",
                  apb, apb->curbyte, apb->nbytes);

          /* Perform any necessary subsampling operations */

          apb = pcm_subsample(priv, apb);
          if (apb)
#endif
            {
              /* Then give the audio buffer to the lower driver */

              audvdbg("Pass to lower enqueuebuffer: apb=%p curbyte=%d nbytes=%d\n",
                      apb, apb->curbyte, apb->nbytes);

              return lower->ops->enqueuebuffer(lower, apb);
            }

          return OK;
        }
    }

  /* This is not a WAV file! */

  auddbg("ERROR: Invalid WAV file\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: pcm_cancelbuffer
 *
 * Description:
 *   Cancel a previously enqueued buffer.
 *
 ****************************************************************************/

static int pcm_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                            FAR struct ap_buffer_s *apb)
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->cancelbuffer);

  audvdbg("Defer to lower cancelbuffer, apb=%p\n", apb);
  return lower->ops->cancelbuffer(lower, apb);
}

/****************************************************************************
 * Name: pcm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands.
 *
 ****************************************************************************/

static int pcm_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Defer the operation to the lower device driver */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->ioctl);

  audvdbg("Defer to lower ioctl, cmd=%d arg=%ld\n");
  return lower->ops->ioctl(lower, cmd, arg);
}

/****************************************************************************
 * Name: pcm_reserve
 *
 * Description:
 *   Reserve a session (may only be one per device or may be multiple) for
 *   use by a client.  Client software can open audio devices and issue
 *   AUDIOIOC_GETCAPS calls freely, but other operations require a
 *   reservation.  A session reservation will assign a context that must
 *   be passed with
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_reserve(FAR struct audio_lowerhalf_s *dev, FAR void **session)
#else
static int pcm_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;
  int ret;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  DEBUGASSERT(priv && session);
#else
  DEBUGASSERT(priv);
#endif

  /* It is not necessary to reserve the upper half.  What we really need to
   * do is to reserved the lower device driver for exclusive use by the PCM
   * decoder.  That effectively reserves the upper PCM decoder along with
   * the lower driver (which is then not available for use by other
   * decoders).
   *
   * We do, however, need to remember the session returned by the lower
   * level.
   */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->reserve);

  audvdbg("Defer to lower reserve\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  ret = lower->ops->reserve(lower, &priv->session);

  /* Return a copy of the session to the caller */

  *session = priv->session;

#else
  ret = lower->ops->reserve(lower);
#endif

  return ret;
}

/****************************************************************************
 * Name: pcm_release
 *
 * Description:
 *   Release a session.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int pcm_release(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int pcm_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)dev;
  FAR struct audio_lowerhalf_s *lower;

  DEBUGASSERT(priv);

  /* Release the lower driver.. it is then available for use by other
   * decoders (and we cannot use the lower driver wither unless we re-
   * reserve it).
   */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->release);

  audvdbg("Defer to lower release\n");
#ifdef CONFIG_AUDIO_MULTI_SESSION
  return lower->ops->release(lower, session);
#else
  return lower->ops->release(lower);
#endif
}

/****************************************************************************
 * Name: pcm_callback
 *
 * Description:
 *   Lower-to-upper level callback for buffer dequeueing.
 *
 * Input Parameters:
 *   priv - The value of the 'priv' field from out audio_lowerhalf_s.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

/* Audio callback */

#ifdef CONFIG_AUDIO_MULTI_SESSION

static void pcm_callback(FAR void *arg, uint16_t reason,
                         FAR struct ap_buffer_s *apb, uint16_t status,
                         FAR void *session)
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)arg;

#ifndef CONFIG_AUDIO_EXCLUDE_FFORWARD
  /* If this is our internal working audio buffer that we use for subsampling,
   * then we are the owner of the buffer and we don't want to return it to
   * the upper level.
   */

  if (priv->apb == apb)
    {
      pcm_subsample_callback(priv, reason, apb, status, session);
    }
  else
#endif
    {
      /* Return the buffer to the upper level */

      pcm_upper_callback(priv, reason, apb, status, session);
    }
}

#else /* CONFIG_AUDIO_MULTI_SESSION */

static void pcm_callback(FAR void *arg, uint16_t reason,
                         FAR struct ap_buffer_s *apb, uint16_t status)
{
  FAR struct pcm_decode_s *priv = (FAR struct pcm_decode_s *)arg;

#ifndef CONFIG_AUDIO_EXCLUDE_FFORWARD
  /* If this is our internal working audio buffer that we use for sub-sampling,
   * then we are the owner of the buffer and we don't want to return it to
   * the upper level.
   */

  if (priv->apb == apb)
    {
      pcm_subsample_callback(priv, reason, apb, status);
    }
  else
#endif
    {
      /* Return the buffer to the upper level */

      pcm_upper_callback(priv, reason, apb, status);
    }
}

#endif /* CONFIG_AUDIO_MULTI_SESSION */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcm_decode_initialize
 *
 * Description:
 *   Initialize the PCM device.  The PCM device accepts and contains a
 *   low-level audio DAC-type device.  It then returns a new audio lower
 *   half interface at adds a PCM decoding from end to the low-level
 *   audio device
 *
 * Input Parameters:
 *   dev - A reference to the low-level audio DAC-type device to contain.
 *
 * Returned Value:
 *   On success, a new audio device instance is returned that wraps the
 *   low-level device and provides a PCM decoding front end.  NULL is
 *   returned on failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *
  pcm_decode_initialize(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct pcm_decode_s *priv;
  FAR struct audio_ops_s *ops;

  /* Allocate an instance of our private data structure */

  priv = (FAR struct pcm_decode_s *)kzalloc(sizeof(struct pcm_decode_s));
  if (!priv)
    {
      auddbg("ERROR: Failed to allocate driver structure\n");
      return NULL;
    }

  /* Initialize our private data structure.  Since kzalloc() was used for
   * the allocation, we need to initialize only non-zero, non-NULL, non-
   * false fields.
   */

  /* Setup our operations */

  ops                  = &priv->ops;
  ops->getcaps         = pcm_getcaps;
  ops->configure       = pcm_configure;
  ops->shutdown        = pcm_shutdown;
  ops->start           = pcm_start;

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  ops->stop            = pcm_stop;
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  ops->pause           = pcm_pause;
  ops->resume          = pcm_resume;
#endif

  if (dev->ops->allocbuffer)
    {
      DEBUGASSERT(dev->ops->freebuffer);
      ops->allocbuffer = pcm_allocbuffer;
      ops->freebuffer  = pcm_freebuffer;
    }

  ops->enqueuebuffer   = pcm_enqueuebuffer;
  ops->cancelbuffer    = pcm_cancelbuffer;
  ops->ioctl           = pcm_ioctl;
  ops->reserve         = pcm_reserve;
  ops->release         = pcm_release;

  /* Set up our struct audio_lower_half that we will register with the
   * system.  The registration process will fill in the priv->export.upper
   * and priv fields with the correct callback information.
   */

  priv->export.ops     = &priv->ops;

  /* Save the struct audio_lower_half of the low-level audio device.  Set
   * out callback information for the lower-level audio device.  Our
   * callback will simply forward to the upper callback.
   */

  priv->lower          = dev;
  dev->upper           = pcm_callback;
  dev->priv            = priv;

  return &priv->export;
}

#endif /* CONFIG_AUDIO && CONFIG_AUDIO_FORMAT_PCM */

