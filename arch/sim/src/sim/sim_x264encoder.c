/****************************************************************************
 * arch/sim/src/sim/sim_x264encoder.c
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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <linux/videodev2.h>
#include <x264.h>

#include "sim_x264encoder.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct x264_wrapper_s
{
  x264_t *enc_ctx;
  x264_picture_t pic_in;
  x264_picture_t pic_out;
  x264_nal_t *nal;
  x264_param_t param;
  int i_nal;
  int remaining_frames;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct x264_wrapper_s *x264_wrapper_open(void)
{
  return calloc(1, sizeof(struct x264_wrapper_s));
}

int x264_wrapper_close(struct x264_wrapper_s *encoder)
{
  free(encoder);
  return 0;
}

int x264_wrapper_streamon(struct x264_wrapper_s *encoder,
                          int width, int height, int fps, int bframe)
{
  int ret;

  memset(&encoder->param, 0, sizeof(x264_param_t));

  ret = x264_param_default_preset(&encoder->param,
                                  "fast",
                                  "zerolatency");
  if (ret < 0)
    {
      return ret;
    }

  encoder->param.i_width      = width;
  encoder->param.i_height     = height;
  encoder->param.i_fps_num    = fps;
  encoder->param.i_fps_den    = 1;
  encoder->param.b_annexb     = 1;
  encoder->param.i_csp        = X264_CSP_I420;
  encoder->param.i_keyint_max = 50;
  encoder->param.i_keyint_min = 25;
  encoder->param.i_bframe     = bframe;

  ret = x264_picture_alloc(&encoder->pic_in,
                           X264_CSP_I420,
                           width,
                           height);
  if (ret < 0)
    {
      return ret;
    }

  encoder->enc_ctx = x264_encoder_open(&encoder->param);
  if (!encoder->enc_ctx)
    {
      x264_picture_clean(&encoder->pic_in);
      return -EINVAL;
    }

  return 0;
}

int x264_wrapper_streamoff(struct x264_wrapper_s *encoder)
{
  x264_encoder_close(encoder->enc_ctx);
  x264_picture_clean(&encoder->pic_in);
  encoder->remaining_frames = 0;

  return 0;
}

int x264_wrapper_enqueue(struct x264_wrapper_s *encoder,
                         uint8_t *data, uint32_t size, int64_t pts)
{
  int ret;

  if (data != NULL)
    {
      int width  = encoder->param.i_width;
      int height = encoder->param.i_height;

      encoder->pic_in.i_pts = pts;
      memcpy(encoder->pic_in.img.plane[0], data, width * height);
      memcpy(encoder->pic_in.img.plane[1], data + width * height,
                                           width * height / 4);
      memcpy(encoder->pic_in.img.plane[2], data + width * height * 5 / 4,
                                           width * height / 4);
    }

  ret = x264_encoder_encode(encoder->enc_ctx,
                            &encoder->nal,
                            &encoder->i_nal,
                            (data != NULL ? &encoder->pic_in : NULL),
                            &encoder->pic_out);

  if (data == NULL)
    {
      encoder->remaining_frames =
        x264_encoder_delayed_frames(encoder->enc_ctx);
    }

  if (ret >= 0)
    {
      return 1;
    }

  return ret;
}

int x264_wrapper_dequeue(struct x264_wrapper_s *encoder,
                         uint8_t *data, uint32_t *size,
                         int64_t *pts, uint32_t *flags)
{
  int total_size = 0;
  int i;

  for (i = 0; i < encoder->i_nal; i++)
    {
      memcpy(data + total_size,
             encoder->nal[i].p_payload, encoder->nal[i].i_payload);
      total_size += encoder->nal[i].i_payload;
    }

  *size = total_size;
  *pts  = encoder->pic_out.i_pts;

  switch (encoder->pic_out.i_type)
    {
      case X264_TYPE_IDR:
      case X264_TYPE_I:
        *flags = V4L2_BUF_FLAG_KEYFRAME;
        break;

      case X264_TYPE_P:
        *flags = V4L2_BUF_FLAG_PFRAME;
        break;

      case X264_TYPE_B:
      case X264_TYPE_BREF:
        *flags = V4L2_BUF_FLAG_BFRAME;
        break;
    }

  return encoder->remaining_frames;
}
