/****************************************************************************
 * include/nuttx/audio/pcm.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_PCM_H
#define __INCLUDE_NUTTX_AUDIO_PCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

#ifdef CONFIG_AUDIO_FORMAT_PCM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************
 *
 * CONFIG_AUDIO_FORMAT_PCM - Enabled PCM support
 */

/* Pre-requisites */

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO is required for PCM support
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error CONFIG_SCHED_WORKQUEUE is required by the PCM decoder
#endif

/* Default configuration values */

/* WAVE Header Definitions **************************************************/
/* All values are little 32-bit or 16-bit endian */

#define WAV_HDR_CHUNKID   0x46464952  /* "RIFF" */
#define WAV_HDR_FORMAT    0x45564157  /* "WAVE" */
#define WAV_FMT_CHUNKID   0x20746d66  /* "fmt " */
#define WAV_FMT_CHUNKLEN  16          /* Size of a PCM subchunk */
#define WAV_FMT_FORMAT    1           /* Linear quantization */
#define WAV_FMT_MONO      1           /* nchannels=1 */
#define WAV_FMT_STEREO    2           /* nchannels=2 */
#define WAV_DATA_CHUNKID  0x61746164  /* "data" */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* The standard WAV header consist of three chunks
 *
 * 1. A WAV header chunk,
 * 2. A format chunk, and
 * 3. A data chunk.
 */

struct wav_hdrchunk_s
{
  uint32_t chunkid;       /* Contains the letters "RIFF" in ASCII form. */
  uint32_t chunklen;      /* Size of the rest of the following chunk */
  uint32_t format;        /* Contains the letters "WAVE" */
};

struct wav_formatchunk_s
{
  uint32_t chunkid;       /* Contains the letters "fmt " */
  uint32_t chunklen;      /* Size of the following chunk (16 for PCM) */
  uint16_t format;        /* PCM=1 (i.e. Linear quantization) */
  uint16_t nchannels;     /* Mono=1, Stereo=2 */
  uint32_t samprate;      /* 8000, 44100, ... */
  uint32_t byterate;      /* samprate * nchannels * bpsamp / 8 */
  uint16_t align;         /* nchannels * bpsamp / 8 */
  uint16_t bpsamp;        /* Bits per sample: 8 bits = 8, 16 bits = 16 */
};

struct wav_datachunk_s
{
  uint32_t chunkid;       /* Contains the letters "data" */
  uint32_t chunklen;      /* Number of bytes in the data */
};

/* The standard WAV file header format is then these three chunks */

struct wav_header_s
{
  struct wav_hdrchunk_s    hdr;
  struct wav_formatchunk_s fmt;
  struct wav_datachunk_s   data;
};

 /****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
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
  pcm_decode_initialize(FAR struct audio_lowerhalf_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AUDIO_FORMAT_PCM */
#endif /* __INCLUDE_NUTTX_AUDIO_PCM_H */
