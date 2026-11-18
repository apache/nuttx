/****************************************************************************
 * arch/sim/src/sim/sim_jpegdec.c
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

#include <errno.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <jpeglib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/video/video.h>

#include "sim_jpegdec.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Alias hides the mixed-case libjpeg enum from nxstyle. */

#define SIM_JPEG_JCS_YCBCR JCS_YCbCr

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_jpegdec_s
{
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr         jerr;
  jmp_buf                       env;
  void                         *scratch;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sim_jpegdec_error_exit(j_common_ptr cinfo)
{
  struct sim_jpegdec_s *priv = (struct sim_jpegdec_s *)cinfo->client_data;

  longjmp(priv->env, 1);
}

static void sim_jpegdec_output_message(j_common_ptr cinfo)
{
  /* Warnings are not fatal; keep the console quiet. */

  (void)cinfo;
}

/* Store one RGB triplet in the requested capture format.  The padding
 * byte of the 32 bit formats is 0xff, matching libjpeg-turbo.
 */

static void sim_jpegdec_store_rgb(uint8_t *dst, uint32_t out_fmt,
                                  uint8_t r, uint8_t g, uint8_t b)
{
  switch (out_fmt)
    {
      case V4L2_PIX_FMT_RGB24:
        dst[0] = r;
        dst[1] = g;
        dst[2] = b;
        break;

      case V4L2_PIX_FMT_BGR24:
        dst[0] = b;
        dst[1] = g;
        dst[2] = r;
        break;

      case V4L2_PIX_FMT_XRGB32:
        dst[0] = 0xff;
        dst[1] = r;
        dst[2] = g;
        dst[3] = b;
        break;

      case V4L2_PIX_FMT_XBGR32:
        dst[0] = 0xff;
        dst[1] = b;
        dst[2] = g;
        dst[3] = r;
        break;

      default:
        break;
    }
}

/* Decode to packed YUV420P or NV12 with the raw data interface.  Only
 * 4:2:0 and grayscale input map onto those output layouts; anything
 * else is rejected instead of overrunning the caller buffer.
 */

static int sim_jpegdec_decode_yuv(struct sim_jpegdec_s *priv,
                                  void *dst, uint32_t *dst_size,
                                  uint32_t out_fmt)
{
  struct jpeg_decompress_struct *cinfo = &priv->cinfo;
  uint32_t w;
  uint32_t h;
  uint32_t cw;
  uint32_t ch;
  uint32_t ystride;
  uint32_t cstride = 0;
  uint32_t ymcu;
  uint32_t cmcu = 0;
  uint32_t y;
  uint32_t i;
  uint8_t *ydst = dst;
  uint8_t *udst;
  uint8_t *vdst;
  bool gray = false;
  JSAMPROW yrows[2 * DCTSIZE];
  JSAMPROW urows[DCTSIZE];
  JSAMPROW vrows[DCTSIZE];
  JSAMPARRAY planes[3];

  if (cinfo->num_components == 3 &&
      cinfo->jpeg_color_space == SIM_JPEG_JCS_YCBCR)
    {
      if (cinfo->comp_info[0].h_samp_factor != 2 ||
          cinfo->comp_info[0].v_samp_factor != 2 ||
          cinfo->comp_info[1].h_samp_factor != 1 ||
          cinfo->comp_info[1].v_samp_factor != 1 ||
          cinfo->comp_info[2].h_samp_factor != 1 ||
          cinfo->comp_info[2].v_samp_factor != 1)
        {
          return -EINVAL;
        }
    }
  else if (cinfo->num_components == 1 &&
           cinfo->jpeg_color_space == JCS_GRAYSCALE)
    {
      gray = true;
    }
  else
    {
      return -EINVAL;
    }

  cinfo->raw_data_out = TRUE;
  jpeg_start_decompress(cinfo);

  w = cinfo->output_width;
  h = cinfo->output_height;
  cw = (w + 1) / 2;
  ch = (h + 1) / 2;

  ystride = cinfo->comp_info[0].width_in_blocks * DCTSIZE;
  ymcu = cinfo->comp_info[0].v_samp_factor * DCTSIZE;
  if (!gray)
    {
      cstride = cinfo->comp_info[1].width_in_blocks * DCTSIZE;
      cmcu = cinfo->comp_info[1].v_samp_factor * DCTSIZE;
    }

  priv->scratch = kmm_malloc(ymcu * ystride + 2 * cmcu * cstride);
  if (priv->scratch == NULL)
    {
      return -ENOMEM;
    }

  for (i = 0; i < ymcu; i++)
    {
      yrows[i] = (JSAMPROW)((uint8_t *)priv->scratch + i * ystride);
    }

  udst = (uint8_t *)priv->scratch + ymcu * ystride;
  vdst = udst + cmcu * cstride;
  for (i = 0; i < cmcu; i++)
    {
      urows[i] = (JSAMPROW)(udst + i * cstride);
      vrows[i] = (JSAMPROW)(vdst + i * cstride);
    }

  planes[0] = yrows;
  planes[1] = urows;
  planes[2] = vrows;

  /* A grayscale image has no chroma; 0x80 is neutral. */

  if (gray)
    {
      memset((uint8_t *)dst + w * h, 0x80, 2 * cw * ch);
    }

  for (y = 0; y < h; y += ymcu)
    {
      jpeg_read_raw_data(cinfo, planes, ymcu);

      for (i = 0; i < ymcu && y + i < h; i++)
        {
          memcpy(ydst + (y + i) * w, yrows[i], w);
        }

      if (gray)
        {
          continue;
        }

      for (i = 0; i < cmcu && y / 2 + i < ch; i++)
        {
          uint32_t crow = y / 2 + i;

          if (out_fmt == V4L2_PIX_FMT_NV12)
            {
              uint8_t *uv = ydst + w * h + crow * cw * 2;
              uint32_t x;

              for (x = 0; x < cw; x++)
                {
                  uv[2 * x] = urows[i][x];
                  uv[2 * x + 1] = vrows[i][x];
                }
            }
          else
            {
              memcpy(ydst + w * h + crow * cw, urows[i], cw);
              memcpy(ydst + w * h + ch * cw + crow * cw, vrows[i], cw);
            }
        }
    }

  *dst_size = w * h + 2 * cw * ch;
  return 0;
}

/* Decode to an RGB format through the scanline interface.  Any input
 * sampling works here because libjpeg upsamples to full resolution.
 */

static int sim_jpegdec_decode_rgb(struct sim_jpegdec_s *priv,
                                  void *dst, uint32_t *dst_size,
                                  uint32_t out_fmt)
{
  struct jpeg_decompress_struct *cinfo = &priv->cinfo;
  uint32_t w;
  uint32_t h;
  uint32_t bpp;
  uint32_t row;
  uint32_t x;
  uint8_t *row_rgb;
  uint8_t *row_src;
  bool gray;
  JSAMPROW rowptr;

  gray = cinfo->num_components == 1 &&
         cinfo->jpeg_color_space == JCS_GRAYSCALE;
  if (!gray && cinfo->num_components != 3)
    {
      return -EINVAL;
    }

  switch (out_fmt)
    {
      case V4L2_PIX_FMT_RGB24:
      case V4L2_PIX_FMT_BGR24:
        bpp = 3;
        break;

      case V4L2_PIX_FMT_XRGB32:
      case V4L2_PIX_FMT_XBGR32:
        bpp = 4;
        break;

      default:
        return -EINVAL;
    }

  cinfo->out_color_space = gray ? JCS_GRAYSCALE : JCS_RGB;
  jpeg_start_decompress(cinfo);

  w = cinfo->output_width;
  h = cinfo->output_height;

  priv->scratch = kmm_malloc(w * (gray ? 4 : 3));
  if (priv->scratch == NULL)
    {
      return -ENOMEM;
    }

  row_src = priv->scratch;
  row_rgb = gray ? row_src + w : row_src;

  for (row = 0; row < h; row++)
    {
      rowptr = (JSAMPROW)row_src;
      jpeg_read_scanlines(cinfo, &rowptr, 1);

      if (gray)
        {
          for (x = 0; x < w; x++)
            {
              row_rgb[3 * x] = row_src[x];
              row_rgb[3 * x + 1] = row_src[x];
              row_rgb[3 * x + 2] = row_src[x];
            }
        }

      for (x = 0; x < w; x++)
        {
          sim_jpegdec_store_rgb((uint8_t *)dst + (row * w + x) * bpp,
                                out_fmt, row_rgb[3 * x],
                                row_rgb[3 * x + 1], row_rgb[3 * x + 2]);
        }
    }

  *dst_size = w * h * bpp;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void *sim_jpegdec_open(void)
{
  struct sim_jpegdec_s *priv;

  priv = kmm_zalloc(sizeof(struct sim_jpegdec_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->cinfo.err = jpeg_std_error(&priv->jerr);
  priv->jerr.error_exit = sim_jpegdec_error_exit;
  priv->jerr.output_message = sim_jpegdec_output_message;
  priv->cinfo.client_data = priv;

  jpeg_create_decompress(&priv->cinfo);
  return priv;
}

void sim_jpegdec_close(void *dec)
{
  struct sim_jpegdec_s *priv = dec;

  if (priv != NULL)
    {
      jpeg_destroy_decompress(&priv->cinfo);
      kmm_free(priv);
    }
}

int sim_jpegdec_decode(void *dec,
                       const void *src, uint32_t src_size,
                       void *dst, uint32_t *dst_size,
                       uint32_t *width, uint32_t *height,
                       uint32_t out_fmt)
{
  struct sim_jpegdec_s *priv = dec;
  int ret;

  if (priv == NULL || src == NULL || dst == NULL)
    {
      return -EINVAL;
    }

  if (setjmp(priv->env))
    {
      kmm_free(priv->scratch);
      priv->scratch = NULL;
      jpeg_abort_decompress(&priv->cinfo);
      return -EIO;
    }

  jpeg_mem_src(&priv->cinfo, (const unsigned char *)src, src_size);
  if (jpeg_read_header(&priv->cinfo, TRUE) != JPEG_HEADER_OK)
    {
      jpeg_abort_decompress(&priv->cinfo);
      return -EIO;
    }

  switch (out_fmt)
    {
      case V4L2_PIX_FMT_YUV420:
      case V4L2_PIX_FMT_NV12:
        ret = sim_jpegdec_decode_yuv(priv, dst, dst_size, out_fmt);
        break;

      case V4L2_PIX_FMT_RGB24:
      case V4L2_PIX_FMT_BGR24:
      case V4L2_PIX_FMT_XRGB32:
      case V4L2_PIX_FMT_XBGR32:
        ret = sim_jpegdec_decode_rgb(priv, dst, dst_size, out_fmt);
        break;

      default:
        ret = -EINVAL;
        break;
    }

  if (ret != 0)
    {
      kmm_free(priv->scratch);
      priv->scratch = NULL;
      jpeg_abort_decompress(&priv->cinfo);
      return ret;
    }

  kmm_free(priv->scratch);
  priv->scratch = NULL;
  jpeg_finish_decompress(&priv->cinfo);
  *width = priv->cinfo.output_width;
  *height = priv->cinfo.output_height;
  return 0;
}
