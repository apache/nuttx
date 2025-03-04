/****************************************************************************
 * include/nuttx/video/video.h
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
 * The definitions and prototypes in this file are created with reference to
 * the FreeBSD V4L2 driver header at: https://github.com/freebsd/freebsd-src
 * /sys/contrib/v4l/videodev2.h
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_VIDEO_VIDEO_H
#define __INCLUDE_NUTTX_VIDEO_VIDEO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/videoio.h>
#include <nuttx/fs/fs.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIDEO_HSIZE_QVGA        (320)   /* QVGA    horizontal size */
#define VIDEO_VSIZE_QVGA        (240)   /* QVGA    vertical   size */
#define VIDEO_HSIZE_VGA         (640)   /* VGA     horizontal size */
#define VIDEO_VSIZE_VGA         (480)   /* VGA     vertical   size */
#define VIDEO_HSIZE_QUADVGA     (1280)  /* QUADVGA horizontal size */
#define VIDEO_VSIZE_QUADVGA     (960)   /* QUADVGA vertical   size */
#define VIDEO_HSIZE_HD          (1280)  /* HD      horizontal size */
#define VIDEO_VSIZE_HD          (720)   /* HD      vertical   size */
#define VIDEO_HSIZE_FULLHD      (1920)  /* FULLHD  horizontal size */
#define VIDEO_VSIZE_FULLHD      (1080)  /* FULLHD  vertical   size */
#define VIDEO_HSIZE_5M          (2560)  /* 5M      horizontal size */
#define VIDEO_VSIZE_5M          (1920)  /* 5M      vertical   size */
#define VIDEO_HSIZE_3M          (2048)  /* 3M      horizontal size */
#define VIDEO_VSIZE_3M          (1536)  /* 3M      vertical   size */

/****************************************************************************
* Public Types
*****************************************************************************/

struct v4l2_s
{
  FAR const struct v4l2_ops_s      *vops;
  FAR const struct file_operations *fops;
};

struct v4l2_ops_s
{
  CODE int (*querycap)(FAR struct file *filep,
                       FAR struct v4l2_capability *cap);
  CODE int (*g_input)(FAR int *num);
  CODE int (*enum_input)(FAR struct file *filep,
                         FAR struct v4l2_input *input);
  CODE int (*reqbufs)(FAR struct file *filep,
                      FAR struct v4l2_requestbuffers *reqbufs);
  CODE int (*querybuf)(FAR struct file *filep,
                       FAR struct v4l2_buffer *buf);
  CODE int (*qbuf)(FAR struct file *filep,
                   FAR struct v4l2_buffer *buf);
  CODE int (*dqbuf)(FAR struct file *filep,
                    FAR struct v4l2_buffer *buf);
  CODE int (*cancel_dqbuf)(FAR struct file *filep,
                           enum v4l2_buf_type type);
  CODE int (*g_fmt)(FAR struct file *filep,
                    FAR struct v4l2_format *fmt);
  CODE int (*s_fmt)(FAR struct file *filep,
                    FAR struct v4l2_format *fmt);
  CODE int (*try_fmt)(FAR struct file *filep,
                      FAR struct v4l2_format *v4l2);
  CODE int (*g_parm)(FAR struct file *filep,
                     FAR struct v4l2_streamparm *parm);
  CODE int (*s_parm)(FAR struct file *filep,
                     FAR struct v4l2_streamparm *parm);
  CODE int (*streamon)(FAR struct file *filep,
                       FAR enum v4l2_buf_type *type);
  CODE int (*streamoff)(FAR struct file *filep,
                        FAR enum v4l2_buf_type *type);
  CODE int (*do_halfpush)(FAR struct file *filep,
                          bool enable);
  CODE int (*takepict_start)(FAR struct file *filep,
                             int32_t capture_num);
  CODE int (*takepict_stop)(FAR struct file *filep,
                            bool halfpush);
  CODE int (*s_selection)(FAR struct file *filep,
                          FAR struct v4l2_selection *clip);
  CODE int (*g_selection)(FAR struct file *filep,
                          FAR struct v4l2_selection *clip);
  CODE int (*queryctrl)(FAR struct file *filep,
                        FAR struct v4l2_queryctrl *ctrl);
  CODE int (*query_ext_ctrl)(FAR struct file *filep,
                             FAR struct v4l2_query_ext_ctrl *ctrl);
  CODE int (*querymenu)(FAR struct file *filep,
                        FAR struct v4l2_querymenu *menu);
  CODE int (*g_ctrl)(FAR struct file *filep,
                     FAR struct v4l2_control *ctrl);
  CODE int (*s_ctrl)(FAR struct file *filep,
                     FAR struct v4l2_control *ctrl);
  CODE int (*g_ext_ctrls)(FAR struct file *filep,
                          FAR struct v4l2_ext_controls *ctrls);
  CODE int (*s_ext_ctrls)(FAR struct file *filep,
                          FAR struct v4l2_ext_controls *ctrls);
  CODE int (*query_ext_ctrl_scene)(FAR struct file *filep,
              FAR struct v4s_query_ext_ctrl_scene *ctrl);
  CODE int (*querymenu_scene)(FAR struct file *filep,
              FAR struct v4s_querymenu_scene *menu);
  CODE int (*g_ext_ctrls_scene)(FAR struct file *filep,
              FAR struct v4s_ext_controls_scene *ctrls);
  CODE int (*s_ext_ctrls_scene)(FAR struct file *filep,
              FAR struct v4s_ext_controls_scene *ctrls);
  CODE int (*enum_fmt)(FAR struct file *filep,
                       FAR struct v4l2_fmtdesc *f);
  CODE int (*enum_frminterval)(FAR struct file *filep,
                               FAR struct v4l2_frmivalenum *f);
  CODE int (*enum_frmsize)(FAR struct file *filep,
                           FAR struct v4l2_frmsizeenum *f);
  CODE int (*cropcap)(FAR struct file *filep,
                      FAR struct v4l2_cropcap *cropcap);
  CODE int (*dqevent)(FAR struct file *filep,
                      FAR struct v4l2_event *event);
  CODE int (*subscribe_event)(FAR struct file *filep,
                              FAR struct v4l2_event_subscription *sub);
  CODE int (*decoder_cmd)(FAR struct file *filep,
                          FAR struct v4l2_decoder_cmd *cmd);
  CODE int (*encoder_cmd)(FAR struct file *filep,
                          FAR struct v4l2_encoder_cmd *cmd);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* New API to register video driver.
 *
 *  param [in] devpath: path to video device
 *  param [in] data: provide imgdata ops
 *  param [in] sensor: provide imgsensor ops array
 *  param [in] sensor_num: the number of imgsensor ops array
 *
 *  Return on success, 0 is returned. On failure,
 *  negative value is returned.
 */

int video_register(FAR const char *devpath,
                   FAR struct v4l2_s *ctx);

/* New API to Unregister video driver.
 *
 *  param [in] devpath: path to video device
 *
 *  Return on success, 0 is returned. On failure,
 *  negative value is returned.
 */

int video_unregister(FAR const char *devpath);

#ifdef __cplusplus
}
#endif
#endif /* __INCLUDE_NUTTX_VIDEO_VIDEO_H */
