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

  /* This is a copy of the WAV file header, in host endian order */

  struct wav_header_s wav;

  /* Set to true once we have parse a valid header and have begun stream
   * audio.
   */

  bool streaming;
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
  dbg( "  Chunk ID:        0x%08x\n", wav->hdr.chunkid);
  dbg( "  Chunk Size:      %u\n",     wav->hdr.chunklen);
  dbg( "  Format:          0x%08x\n", wav->hdr.format);
  dbg( "  SubChunk ID:     0x%08x\n", wav->fmt.chunkid);
  dbg( "  Subchunk1 Size:  %u\n",     wav->fmt.chunklen);
  dbg( "  Audio Format:    0x%04x\n", wav->fmt.format);
  dbg( "  Num. Channels:   %d\n",     wav->fmt.nchannels);
  dbg( "  Sample Rate:     %u\n",     wav->fmt.samprate);
  dbg( "  Byte Rate:       %u\n",     wav->fmt.byterate);
  dbg( "  Block Align:     %d\n",     wav->fmt.align);
  dbg( "  Bits Per Sample: %d\n",     wav->fmt.bpsamp);
  dbg( "  Subchunk2 ID:    0x%08x\n", wav->data.chunkid);
  dbg( "  Subchunk2 Size:  %u\n",     wav->data.chunklen);
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
  return (wav->hdr.chunkid  == WAV_HDR_CHUNKID &&
          wav->hdr.format   == WAV_HDR_FORMAT &&
          wav->fmt.chunkid  == WAV_FMT_CHUNKID &&
          wav->fmt.chunklen == WAV_FMT_CHUNKLEN &&
          wav->fmt.format   == WAV_FMT_FORMAT &&
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

  /* Transfer the purported WAV file header into our private storage,
   * correcting for endian issues as needed.
   */

  priv->wav.hdr.chunkid   = pcm_leuint32(wav->hdr.chunkid);
  priv->wav.hdr.chunklen  = pcm_leuint32(wav->hdr.chunklen);
  priv->wav.hdr.format    = pcm_leuint32(wav->hdr.format);

  priv->wav.fmt.chunkid   = pcm_leuint32(wav->fmt.chunkid);
  priv->wav.fmt.chunklen  = pcm_leuint32(wav->fmt.chunklen);
  priv->wav.fmt.format    = pcm_leuint16(wav->fmt.format);
  priv->wav.fmt.nchannels = pcm_leuint16(wav->fmt.nchannels);
  priv->wav.fmt.samprate  = pcm_leuint32(wav->fmt.samprate);
  priv->wav.fmt.byterate  = pcm_leuint32(wav->fmt.byterate);
  priv->wav.fmt.align     = pcm_leuint16(wav->fmt.align);
  priv->wav.fmt.bpsamp    = pcm_leuint16(wav->fmt.bpsamp);

  priv->wav.data.chunkid  = pcm_leuint32(wav->data.chunkid);
  priv->wav.data.chunklen = pcm_leuint32(wav->data.chunklen);

  /* Dump the converted wave header information */

  pcm_dump(&priv->wav);

  /* And return true if the the file is a valid WAV header file */

  return pcm_validwav(&priv->wav);
}

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
      *((uint16_t *)&caps->ac_format[0]) = (1 << (AUDIO_FMT_PCM - 1));
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
#warning Missing logic
  return -ENOSYS;
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
#warning Missing logic
  return -ENOSYS;
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
#warning Missing logic
  return -ENOSYS;
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
#warning Missing logic
  return -ENOSYS;
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
#warning Missing logic
  return -ENOSYS;
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
#warning Missing logic
  return -ENOSYS;
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

  DEBUGASSERT(priv);
  audvdbg("Received buffer %p, streaming=%d\n", apb, priv->streaming);

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->enqueuebuffer);

  /* Are we streaming yet? */

  if (priv->streaming)
    {
      /* Yes, just give the buffer to the lower driver */

      return lower->ops->enqueuebuffer(lower, apb);
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
          /* Now we are streaming */

          priv->streaming = true;

          /* Bump up the data offset and pass the buffer to the lower level */

          apb->curbyte += sizeof(struct wav_header_s);
          return lower->ops->enqueuebuffer(lower, apb);
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
#warning Missing logic
  return -ENOSYS;
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

  DEBUGASSERT(priv);

  /* It is not necessary to reserve the upper half.  What we really need to
   * do is to reserved the lower device driver for exclusive use by the PCM
   * decoder.  That effectively reserves the upper PCM decoder along with
   * the lower driver (which is then not available for use by other
   * decoders).
   */

  lower = priv->lower;
  DEBUGASSERT(lower && lower->ops->reserve);

#ifdef CONFIG_AUDIO_MULTI_SESSION
  return lower->ops->reserve(lower, session);
#else
  return lower->ops->reserve(lower);
#endif
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

#ifdef CONFIG_AUDIO_MULTI_SESSION
  return lower->ops->release(lower, session);
#else
  return lower->ops->release(lower);
#endif
}

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

  priv->export.ops     = &priv->ops;
  priv->export.priv    = priv;
  priv->lower          = dev;

  return &priv->export;
}

#endif /* CONFIG_AUDIO && CONFIG_AUDIO_FORMAT_PCM */

