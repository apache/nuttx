/****************************************************************************
 * arch/sim/src/sim/sim_openh264dec.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>

#include <wels/codec_api.h>
#include <wels/codec_app_def.h>

#include "sim_openh264dec.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct openh264_decoder_s
{
  ISVCDecoder *dec;
  SBufferInfo bufinfo;
  uint8_t *pdst[4];
  int remaining_frames;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct openh264_decoder_s *openh264_decoder_open(void)
{
  struct openh264_decoder_s *decoder;

  decoder = calloc(1, sizeof(struct openh264_decoder_s));
  if (decoder == NULL)
    {
      syslog(LOG_ERR, "Init host decoder failed\n");
    }

  return decoder;
}

int openh264_decoder_close(struct openh264_decoder_s *decoder)
{
  free(decoder);
  return 0;
}

int openh264_decoder_streamon(struct openh264_decoder_s *decoder)
{
  SDecodingParam param;
  int level = WELS_LOG_RESV;
  int ret;

  memset(&param, 0, sizeof(SDecodingParam));
  param.eEcActiveIdc = ERROR_CON_DISABLE;
  param.sVideoProperty.eVideoBsType = VIDEO_BITSTREAM_DEFAULT;

  ret = WelsCreateDecoder(&decoder->dec);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to create decoder\n");
      return ret;
    }

  (*decoder->dec)->SetOption(decoder->dec,
                             DECODER_OPTION_TRACE_LEVEL,
                             &level);

  ret = (*decoder->dec)->Initialize(decoder->dec, &param);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize decoder\n");
      WelsDestroyDecoder(decoder->dec);
    }

  return ret;
}

int openh264_decoder_streamoff(struct openh264_decoder_s *decoder)
{
  (*decoder->dec)->Uninitialize(decoder->dec);
  WelsDestroyDecoder(decoder->dec);

  decoder->remaining_frames = 0;
  return 0;
}

int openh264_decoder_enqueue(struct openh264_decoder_s *decoder,
                             void *data, int64_t pts, int size)
{
  DECODING_STATE state;

  memset(&decoder->bufinfo, 0, sizeof(SBufferInfo));
  memset(decoder->pdst, 0, sizeof(decoder->pdst));

  if (decoder->remaining_frames == 0)
    {
      /* When data is NULL and remaining_frames has
       * not been assigned, call DecodeFrame2 to obtain
       * the pending buffer.
       */

      decoder->bufinfo.uiInBsTimeStamp = pts;
      state = (*decoder->dec)->DecodeFrame2(decoder->dec,
                                            data,
                                            size,
                                            decoder->pdst,
                                            &decoder->bufinfo);
      if (state != dsErrorFree)
        {
          syslog(LOG_ERR, "hostdec - decode failed 0x%04x\n", state);
          return -EINVAL;
        }

      if (data == NULL)
        {
          (*decoder->dec)->GetOption(decoder->dec,
                            DECODER_OPTION_NUM_OF_FRAMES_REMAINING_IN_BUFFER,
                            &decoder->remaining_frames);
        }
    }
  else
    {
      /* When remaining_frames is assigned and greater than
       * zero, call FlushFrame to get the last buffers.
       */

      state = (*decoder->dec)->FlushFrame(decoder->dec,
                                          decoder->pdst,
                                          &decoder->bufinfo);
      if (state != dsErrorFree)
        {
          syslog(LOG_ERR, "hostdec - flush failed 0x%04x\n", state);
          return -EINVAL;
        }

      decoder->remaining_frames--;
    }

  return decoder->bufinfo.iBufferStatus;
}

int openh264_decoder_dequeue(struct openh264_decoder_s *decoder,
                             void *data, int64_t *pts, uint32_t *size)
{
  uint8_t *dst_addr = data;
  int plane;

  for (plane = 0; plane < 3; plane++)
    {
      uint8_t *src_addr = decoder->pdst[plane];
      int width  = decoder->bufinfo.UsrData.sSystemBuffer.iWidth;
      int height = decoder->bufinfo.UsrData.sSystemBuffer.iHeight;
      int stride = decoder->bufinfo.UsrData.sSystemBuffer.iStride[0];
      int row;

      if (plane > 0)
        {
          /* UV plane stride is iStride[1] */

          width  = width  / 2;
          height = height / 2;
          stride = decoder->bufinfo.UsrData.sSystemBuffer.iStride[1];
        }

      for (row = 0; row < height; row++)
        {
          memcpy(dst_addr, src_addr, width);
          src_addr += stride;
          dst_addr += width;
        }
    }

  *pts  = decoder->bufinfo.uiOutYuvTimeStamp;
  *size = decoder->bufinfo.UsrData.sSystemBuffer.iWidth  *
          decoder->bufinfo.UsrData.sSystemBuffer.iHeight * 3 / 2;

  return decoder->remaining_frames;
}

