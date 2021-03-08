/****************************************************************************
 * include/nuttx/video/video_halif.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_HALIF_H
#define __INCLUDE_NUTTX_VIDEO_HALIF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/video/video.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct video_devops_s
{
  CODE int (*open)(FAR void *video_priv);
  CODE int (*close)(void);

  CODE int (*do_halfpush)(bool enable);
  CODE int (*set_buftype)(enum v4l2_buf_type type);
  CODE int (*set_buf)(uint32_t bufaddr, uint32_t bufsize);
  CODE int (*cancel_dma)(void);
  CODE int (*get_range_of_fmt)(FAR struct v4l2_fmtdesc *format);
  CODE int (*get_range_of_framesize)(FAR struct v4l2_frmsizeenum *frmsize);
  CODE int (*try_format)(FAR struct v4l2_format *format);
  CODE int (*set_format)(FAR struct v4l2_format *format);
  CODE int (*get_range_of_frameinterval)
           (FAR struct v4l2_frmivalenum *frmival);
  CODE int (*set_frameinterval)(FAR struct v4l2_streamparm *parm);
  CODE int (*get_range_of_ctrlvalue)(FAR struct v4l2_query_ext_ctrl *range);
  CODE int (*get_menu_of_ctrlvalue)(FAR struct v4l2_querymenu *menu);
  CODE int (*get_ctrlvalue)(uint16_t ctrl_class,
                            FAR struct v4l2_ext_control *control);
  CODE int (*set_ctrlvalue)(uint16_t ctrl_class,
                            FAR struct v4l2_ext_control *control);
  CODE int (*refresh)(void);
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
 * Public Functions Definitions
 ****************************************************************************/

int video_common_notify_dma_done(uint8_t  err_code,
                                 uint32_t buf_type,
                                 uint32_t datasize,
                                 FAR void *priv);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_HALIF_H */
