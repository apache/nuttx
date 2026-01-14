/****************************************************************************
 * include/nuttx/video/v4l2_m2m.h
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_VIDEO_V4L2_M2M_H
#define __INCLUDE_NUTTX_VIDEO_V4L2_M2M_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/videoio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Method access helper macros */

#define CODEC_OPEN(codec, cookie, priv) \
  ((codec)->ops->open ? \
   (codec)->ops->open(cookie, priv) : -ENOTTY)

#define CODEC_CLOSE(codec, priv) \
  ((codec)->ops->close ? \
   (codec)->ops->close(priv) : -ENOTTY)

#define CODEC_CAPTURE_STREAMON(codec, priv) \
  ((codec)->ops->capture_streamon ? \
   (codec)->ops->capture_streamon(priv) : -ENOTTY)

#define CODEC_OUTPUT_STREAMON(codec, priv) \
  ((codec)->ops->output_streamon ? \
   (codec)->ops->output_streamon(priv) : -ENOTTY)

#define CODEC_CAPTURE_STREAMOFF(codec, priv) \
  ((codec)->ops->capture_streamoff ? \
   (codec)->ops->capture_streamoff(priv) : -ENOTTY)

#define CODEC_OUTPUT_STREAMOFF(codec, priv) \
  ((codec)->ops->output_streamoff ? \
   (codec)->ops->output_streamoff(priv) : -ENOTTY)

#define CODEC_CAPTURE_AVAILABLE(codec, priv) \
  ((codec)->ops->capture_available ? \
   (codec)->ops->capture_available(priv) : 0)

#define CODEC_OUTPUT_AVAILABLE(codec, priv) \
  ((codec)->ops->output_available ? \
  (codec)->ops->output_available(priv) : -ENOTTY)

#define CODEC_QUERYCAP(codec, priv, cap) \
  ((codec)->ops->querycap ? \
   (codec)->ops->querycap(priv, cap) : -ENOTTY)

#define CODEC_CAPTURE_ENUM_FMT(codec, priv, fmt) \
  ((codec)->ops->capture_enum_fmt ? \
   (codec)->ops->capture_enum_fmt(priv, fmt) : -ENOTTY)

#define CODEC_OUTPUT_ENUM_FMT(codec, priv, fmt) \
  ((codec)->ops->output_enum_fmt ? \
   (codec)->ops->output_enum_fmt(priv, fmt) : -ENOTTY)

#define CODEC_CAPTURE_G_FMT(codec, priv, fmt) \
  ((codec)->ops->capture_g_fmt ? \
   (codec)->ops->capture_g_fmt(priv, fmt) : -ENOTTY)

#define CODEC_OUTPUT_G_FMT(codec, priv, fmt) \
  ((codec)->ops->output_g_fmt ? \
   (codec)->ops->output_g_fmt(priv, fmt) : -ENOTTY)

#define CODEC_CAPTURE_S_FMT(codec, priv, fmt) \
  ((codec)->ops->capture_s_fmt ? \
   (codec)->ops->capture_s_fmt(priv, fmt) : -ENOTTY)

#define CODEC_OUTPUT_S_FMT(codec, priv, fmt) \
  ((codec)->ops->output_s_fmt ? \
   (codec)->ops->output_s_fmt(priv, fmt) : -ENOTTY)

#define CODEC_CAPTURE_TRY_FMT(codec, priv, fmt) \
  ((codec)->ops->capture_try_fmt ? \
   (codec)->ops->capture_try_fmt(priv, fmt) : -ENOTTY)

#define CODEC_OUTPUT_TRY_FMT(codec, priv, fmt) \
  ((codec)->ops->output_try_fmt ? \
   (codec)->ops->output_try_fmt(priv, fmt) : -ENOTTY)

#define CODEC_CAPTURE_G_PARM(codec, priv, parm) \
  ((codec)->ops->capture_g_parm ? \
   (codec)->ops->capture_g_parm(priv, parm) : -ENOTTY)

#define CODEC_OUTPUT_G_PARM(codec, priv, parm) \
  ((codec)->ops->output_g_parm ? \
   (codec)->ops->output_g_parm(priv, parm) : -ENOTTY)

#define CODEC_CAPTURE_S_PARM(codec, priv, parm) \
  ((codec)->ops->capture_s_parm ? \
   (codec)->ops->capture_s_parm(priv, parm) : -ENOTTY)

#define CODEC_OUTPUT_S_PARM(codec, priv, parm) \
  ((codec)->ops->output_s_parm ? \
   (codec)->ops->output_s_parm(priv, parm) : -ENOTTY)

#define CODEC_G_EXT_CTRLS(codec, priv, ctrls) \
  ((codec)->ops->g_ext_ctrls ? \
   (codec)->ops->g_ext_ctrls(priv, ctrls) : -ENOTTY)

#define CODEC_S_EXT_CTRLS(codec, priv, ctrls) \
  ((codec)->ops->s_ext_ctrls ? \
   (codec)->ops->s_ext_ctrls(priv, ctrls) : -ENOTTY)

#define CODEC_CAPTURE_G_SELECTION(codec, priv, clip) \
  ((codec)->ops->capture_g_selection ? \
   (codec)->ops->capture_g_selection(priv, clip) : -ENOTTY)

#define CODEC_OUTPUT_G_SELECTION(codec, priv, clip) \
  ((codec)->ops->output_g_selection ? \
   (codec)->ops->output_g_selection(priv, clip) : -ENOTTY)

#define CODEC_CAPTURE_S_SELECTION(codec, priv, clip) \
  ((codec)->ops->capture_s_selection ? \
   (codec)->ops->capture_s_selection(priv, clip) : -ENOTTY)

#define CODEC_OUTPUT_S_SELECTION(codec, priv, clip) \
  ((codec)->ops->output_s_selection ? \
   (codec)->ops->output_s_selection(priv, clip) : -ENOTTY)

#define CODEC_CAPTURE_CROPCAP(codec, priv, cropcap) \
  ((codec)->ops->capture_cropcap ? \
   (codec)->ops->capture_cropcap(priv, cropcap) : -ENOTTY)

#define CODEC_OUTPUT_CROPCAP(codec, priv, cropcap) \
  ((codec)->ops->output_cropcap ? \
   (codec)->ops->output_cropcap(priv, cropcap) : -ENOTTY)

#define CODEC_DQEVENT(codec, priv, event) \
  ((codec)->ops->dqevent ? \
   (codec)->ops->dqevent(priv, event) : -ENOTTY)

#define CODEC_SUBSCRIBE_EVENT(codec, priv, sub) \
  ((codec)->ops->subscribe_event ? \
   (codec)->ops->subscribe_event(priv, sub) : -ENOTTY)

#define CODEC_DECODER_CMD(codec, priv, cmd) \
  ((codec)->ops->decoder_cmd ? \
   (codec)->ops->decoder_cmd(priv, cmd) : -ENOTTY)

#define CODEC_ENCODER_CMD(codec, priv, cmd) \
  ((codec)->ops->encoder_cmd ? \
   (codec)->ops->encoder_cmd(priv, cmd) : -ENOTTY)

#define CODEC_CAPTURE_G_BUFSIZE(codec, priv) \
  ((codec)->ops->capture_g_bufsize ? \
   (codec)->ops->capture_g_bufsize(priv) : -ENOTTY)

#define CODEC_OUTPUT_G_BUFSIZE(codec, priv) \
  ((codec)->ops->output_g_bufsize ? \
   (codec)->ops->output_g_bufsize(priv) : -ENOTTY)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct codec_s;
struct codec_ops_s
{
  CODE int (*open)(FAR void *cookie, FAR void **priv);
  CODE int (*close)(FAR void *priv);

  CODE int (*capture_streamon)(FAR void *priv);
  CODE int (*output_streamon)(FAR void *priv);
  CODE int (*capture_streamoff)(FAR void *priv);
  CODE int (*output_streamoff)(FAR void *priv);

  CODE int (*capture_available)(FAR void *priv);
  CODE int (*output_available)(FAR void *priv);

  /* VIDIOC_QUERYCAP handler */

  CODE int (*querycap)(FAR void *priv,
                       FAR struct v4l2_capability *cap);

  /* VIDIOC_ENUM_FMT handlers */

  CODE int (*capture_enum_fmt)(FAR void *priv,
                               FAR struct v4l2_fmtdesc *fmt);

  CODE int (*output_enum_fmt)(FAR void *priv,
                              FAR struct v4l2_fmtdesc *fmt);

  /* VIDIOC_G_FMT handlers */

  CODE int (*capture_g_fmt)(FAR void *priv,
                            FAR struct v4l2_format *fmt);
  CODE int (*output_g_fmt)(FAR void *priv,
                           FAR struct v4l2_format *fmt);

  /* VIDIOC_S_FMT handlers */

  CODE int (*capture_s_fmt)(FAR void *priv,
                            FAR struct v4l2_format *fmt);
  CODE int (*output_s_fmt)(FAR void *priv,
                           FAR struct v4l2_format *fmt);

  /* VIDIOC_TRY_FMT handlers */

  CODE int (*capture_try_fmt)(FAR void *priv,
                              FAR struct v4l2_format *fmt);
  CODE int (*output_try_fmt)(FAR void *priv,
                             FAR struct v4l2_format *fmt);

  /* Buffer handlers  */

  CODE size_t (*capture_g_bufsize)(FAR void *priv);
  CODE size_t (*output_g_bufsize)(FAR void *priv);

  /* Stream type-dependent parameter ioctls */

  CODE int (*capture_g_parm)(FAR void *priv,
                             FAR struct v4l2_streamparm *parm);
  CODE int (*output_g_parm)(FAR void *priv,
                            FAR struct v4l2_streamparm *parm);
  CODE int (*capture_s_parm)(FAR void *priv,
                             FAR struct v4l2_streamparm *parm);
  CODE int (*output_s_parm)(FAR void *priv,
                            FAR struct v4l2_streamparm *parm);

  /* Control handlers */

  CODE int (*g_ext_ctrls)(FAR void *priv,
                          FAR struct v4l2_ext_controls *ctrls);
  CODE int (*s_ext_ctrls)(FAR void *priv,
                          FAR struct v4l2_ext_controls *ctrls);

  /* Crop ioctls */

  CODE int (*capture_g_selection)(FAR void *priv,
                                  FAR struct v4l2_selection *clip);
  CODE int (*output_g_selection)(FAR void *priv,
                                 FAR struct v4l2_selection *clip);
  CODE int (*capture_s_selection)(FAR void *priv,
                                  FAR struct v4l2_selection *clip);
  CODE int (*output_s_selection)(FAR void *priv,
                                 FAR struct v4l2_selection *clip);
  CODE int (*capture_cropcap)(FAR void *priv,
                              FAR struct v4l2_cropcap *cropcap);
  CODE int (*output_cropcap)(FAR void *priv,
                             FAR struct v4l2_cropcap *cropcap);

  /* Event handlers */

  CODE int (*subscribe_event)(FAR void *priv,
                              FAR struct v4l2_event_subscription *sub);

  /* Command handlers */

  CODE int (*decoder_cmd)(FAR void *priv,
                          FAR struct v4l2_decoder_cmd *cmd);
  CODE int (*encoder_cmd)(FAR void *priv,
                          FAR struct v4l2_encoder_cmd *cmd);
};

struct codec_s
{
  FAR const struct codec_ops_s *ops;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int codec_register(FAR const char *devpath, FAR struct codec_s *codec);
int codec_unregister(FAR const char *devpath);
FAR struct v4l2_buffer *codec_output_get_buf(FAR void *cookie);
FAR struct v4l2_buffer *codec_capture_get_buf(FAR void *cookie);
int codec_output_put_buf(FAR void *cookie, FAR struct v4l2_buffer *buf);
int codec_capture_put_buf(FAR void *cookie, FAR struct v4l2_buffer *buf);
int codec_queue_event(FAR void *cookie, FAR struct v4l2_event *evt);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_V4L2_M2M_H */
