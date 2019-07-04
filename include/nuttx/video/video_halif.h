/****************************************************************************
 * modules/include/video/video_halif.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
 * Public Functions
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
