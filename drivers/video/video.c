/****************************************************************************
 * drivers/video/video.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <sys/ioctl.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include <nuttx/video/video_halif.h>

#include "video_framebuff.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define video_printf(format, ...)   _info(format, ##__VA_ARGS__)

#define MAX_VIDEO_FILE_PATH     (32)

#define VIDEO_TRUE              (1)
#define VIDEO_FALSE             (0)

#define VIDEO_REMAINING_CAPNUM_INFINITY (-1)

/* Debug option */

#ifdef CONFIG_DEBUG_VIDEO_ERROR
#define videoerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#define videoerr(x...)
#endif

#ifdef CONFIG_DEBUG_VIDEO_WARN
#define videowarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#define videowarn(x...)
#endif

#ifdef CONFIG_DEBUG_VIDEO_INFO
#define videoinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#define videoinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum video_state_e
{
  VIDEO_STATE_STREAMOFF = 0, /* DMA trigger event is not received */
  VIDEO_STATE_STREAMON  = 1, /* DMA trigger event is received,
                              * but DMA is not operated.
                              */
  VIDEO_STATE_DMA       = 2, /* On DMA */
};

enum video_state_transition_cause
{
  CAUSE_VIDEO_STOP  = 0,     /* Stop  DMA event for video stream */
  CAUSE_VIDEO_START = 1,     /* Start DMA event for video stream */
  CAUSE_VIDEO_DQBUF = 2,     /* DQBUF timing    for video stream */
  CAUSE_STILL_STOP  = 3,     /* Stop  DMA event for still stream */
  CAUSE_STILL_START = 4,     /* Start DMA event for still stream */
};

enum video_waitend_cause_e
{
  VIDEO_WAITEND_CAUSE_DMADONE   = 0,
  VIDEO_WAITEND_CAUSE_DQCANCEL  = 1,
  VIDEO_WAITEND_CAUSE_STILLSTOP = 2,
};

struct video_wait_dma_s
{
  FAR sem_t            dqbuf_wait_flg;
  FAR vbuf_container_t *done_container; /* Save container which dma done */
  enum video_waitend_cause_e waitend_cause;
};

typedef struct video_wait_dma_s video_wait_dma_t;

struct video_type_inf_s
{
  sem_t                lock_state;
  enum video_state_e   state;
  int32_t              remaining_capnum;
  video_wait_dma_t     wait_dma;
  video_framebuff_t    bufinf;
};

typedef struct video_type_inf_s video_type_inf_t;

struct video_mng_s
{
  FAR char           *devpath;     /* parameter of video_initialize() */
  sem_t              lock_open_num;
  uint8_t            open_num;
  FAR struct pollfd  *poll_wait;   /* poll(setup) information */
  video_type_inf_t   video_inf;
  video_type_inf_t   still_inf;
};

typedef struct video_mng_s video_mng_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods. */

static int video_open(FAR struct file *filep);
static int video_close(FAR struct file *filep);
static int video_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int video_poll(FAR struct file   *filep,
                      FAR struct pollfd *fds,
                      bool              setup);

/* Common function */

static int video_lock(FAR sem_t *sem);
static int video_unlock(FAR sem_t *sem);
static FAR video_type_inf_t *get_video_type_inf
           (FAR video_mng_t *vmng, uint8_t type);
static enum video_state_e estimate_next_video_state
            (FAR video_mng_t                   *vmng,
             enum video_state_transition_cause cause);
static void change_video_state(FAR video_mng_t    *vmng,
                               enum video_state_e next_state);
static bool is_taking_still_picture(FAR video_mng_t *vmng);
static bool is_bufsize_sufficient(FAR video_mng_t *vmng, uint32_t bufsize);
static void cleanup_resources(FAR video_mng_t *vmng);
static bool is_sem_waited(FAR sem_t *sem);

/* internal function for each cmds of ioctl */

static int video_reqbufs(FAR struct video_mng_s *vmng,
                         FAR struct v4l2_requestbuffers *reqbufs);
static int video_qbuf(FAR struct video_mng_s *vmng,
                      FAR struct v4l2_buffer *buf);
static int video_dqbuf(FAR struct video_mng_s *vmng,
                       FAR struct v4l2_buffer *buf);
static int video_cancel_dqbuf(FAR struct video_mng_s *vmng,
                              enum v4l2_buf_type type);
static int video_enum_fmt(FAR struct v4l2_fmtdesc *fmt);
static int video_enum_framesizes(FAR struct v4l2_frmsizeenum *frmsize);
static int video_s_fmt(FAR struct video_mng_s *priv,
                       FAR struct v4l2_format *fmt);
static int video_enum_frameintervals(FAR struct v4l2_frmivalenum *frmival);
static int video_s_parm(FAR struct video_mng_s *priv,
                        FAR struct v4l2_streamparm *parm);
static int video_streamon(FAR struct video_mng_s *vmng,
                          FAR enum v4l2_buf_type *type);
static int video_streamoff(FAR struct video_mng_s *vmng,
                           FAR enum v4l2_buf_type *type);
static int video_do_halfpush(bool enable);
static int video_takepict_start(FAR struct video_mng_s *vmng,
                                int32_t                capture_num);
static int video_takepict_stop(FAR struct video_mng_s *vmng,
                               bool halfpush);
static int video_queryctrl(FAR struct v4l2_queryctrl *ctrl);
static int video_query_ext_ctrl(FAR struct v4l2_query_ext_ctrl *ctrl);
static int video_querymenu(FAR struct v4l2_querymenu *menu);
static int video_g_ctrl(FAR struct video_mng_s *priv,
                        FAR struct v4l2_control *ctrl);
static int video_s_ctrl(FAR struct video_mng_s *priv,
                        FAR struct v4l2_control *ctrl);
static int video_g_ext_ctrls(FAR struct video_mng_s *priv,
                             FAR struct v4l2_ext_controls *ctrls);
static int video_s_ext_ctrls(FAR struct video_mng_s *priv,
                             FAR struct v4l2_ext_controls *ctrls);

/****************************************************************************
 * Public Data
 ****************************************************************************/

static const struct video_devops_s *g_video_devops;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_video_fops =
{
  video_open,               /* open */
  video_close,              /* close */
  0,                        /* read */
  0,                        /* write */
  0,                        /* seek */
  video_ioctl,              /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  video_poll,               /* poll */
#endif
  0                         /* unlink */
};

static bool is_initialized = false;
static FAR void *video_handler;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int video_lock(FAR sem_t *sem)
{
  if (sem == NULL)
    {
      return -EINVAL;
    }

  return nxsem_wait_uninterruptible(sem);
}

static int video_unlock(FAR sem_t *sem)
{
  if (sem == NULL)
    {
      return -EINVAL;
    }

  return nxsem_post(sem);
}

static FAR video_type_inf_t *get_video_type_inf
(FAR video_mng_t *vmng, uint8_t type)
{
  FAR video_type_inf_t *type_inf;

  switch (type)
    {
      case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        type_inf = &vmng->video_inf;
        break;

      case V4L2_BUF_TYPE_STILL_CAPTURE:
        type_inf = &vmng->still_inf;
        break;

      default:  /* Error case */
        type_inf = NULL;
        break;
    }

  return type_inf;
}

static enum video_state_e estimate_next_video_state
            (FAR video_mng_t                   *vmng,
             enum video_state_transition_cause cause)
{
  enum video_state_e current_state = vmng->video_inf.state;

  switch (cause)
    {
      case CAUSE_VIDEO_STOP:
        return VIDEO_STATE_STREAMOFF;

      case CAUSE_VIDEO_START:
        if (is_taking_still_picture(vmng))
          {
            return VIDEO_STATE_STREAMON;
          }
        else
          {
            return VIDEO_STATE_DMA;
          }

      case CAUSE_STILL_STOP:
        if (current_state == VIDEO_STATE_STREAMON)
          {
            return VIDEO_STATE_DMA;
          }
        else
          {
            return current_state;
          }

      case CAUSE_STILL_START:
        if (current_state == VIDEO_STATE_DMA)
          {
            return VIDEO_STATE_STREAMON;
          }
        else
          {
            return current_state;
          }

      case CAUSE_VIDEO_DQBUF:
        if ((current_state == VIDEO_STATE_STREAMON) &&
             !is_taking_still_picture(vmng))
          {
            return VIDEO_STATE_DMA;
          }
        else
          {
            return current_state;
          }

      default:
        return current_state;
    }
}

static void change_video_state(FAR video_mng_t    *vmng,
                               enum video_state_e next_state)
{
  enum video_state_e current_state = vmng->video_inf.state;
  enum video_state_e updated_next_state = next_state;
  FAR vbuf_container_t *dma_container;

  if ((current_state != VIDEO_STATE_DMA) &&
      (next_state    == VIDEO_STATE_DMA))
    {
      dma_container =
              video_framebuff_get_dma_container(&vmng->video_inf.bufinf);
      if (dma_container)
        {
          g_video_devops->set_buftype(V4L2_BUF_TYPE_VIDEO_CAPTURE);
          g_video_devops->set_buf(dma_container->buf.m.userptr,
                                  dma_container->buf.length);
        }
      else
        {
          updated_next_state = VIDEO_STATE_STREAMON;
        }
    }
  else
    {
      if ((current_state == VIDEO_STATE_DMA) &&
          (next_state    != VIDEO_STATE_DMA))
        {
          g_video_devops->cancel_dma();
        }
    }

  vmng->video_inf.state = updated_next_state;

  return;
}

static bool is_taking_still_picture(FAR video_mng_t *vmng)
{
  return ((vmng->still_inf.state == VIDEO_STATE_STREAMON) ||
          (vmng->still_inf.state == VIDEO_STATE_DMA));
}

static bool is_bufsize_sufficient(FAR video_mng_t *vmng, uint32_t bufsize)
{
  /* Depend on format, frame size, and JPEG compression quality */

  return true;
}

static void initialize_streamresources(FAR video_type_inf_t *type_inf)
{
  memset(type_inf, 0, sizeof(video_type_inf_t));
  type_inf->remaining_capnum = VIDEO_REMAINING_CAPNUM_INFINITY;
  nxsem_init(&type_inf->lock_state, 0, 1);
  nxsem_init(&type_inf->wait_dma.dqbuf_wait_flg, 0, 0);
  video_framebuff_init(&type_inf->bufinf);

  return;
}

static void initialize_resources(FAR video_mng_t *vmng)
{
  initialize_streamresources(&vmng->video_inf);
  initialize_streamresources(&vmng->still_inf);

  return;
}

static void cleanup_streamresources(FAR video_type_inf_t *type_inf)
{
  video_framebuff_uninit(&type_inf->bufinf);
  nxsem_destroy(&type_inf->wait_dma.dqbuf_wait_flg);
  nxsem_destroy(&type_inf->lock_state);
  memset(type_inf, 0, sizeof(video_type_inf_t));
  type_inf->remaining_capnum = VIDEO_REMAINING_CAPNUM_INFINITY;

  return;
}

static void cleanup_resources(FAR video_mng_t *vmng)
{
  /* clean up resource */

  if ((vmng->video_inf.state == VIDEO_STATE_DMA) ||
      (vmng->still_inf.state == VIDEO_STATE_DMA))
    {
      /* If in DMA, stop */

      g_video_devops->cancel_dma();
    }

  cleanup_streamresources(&vmng->video_inf);
  cleanup_streamresources(&vmng->still_inf);

  return;
}

static bool is_sem_waited(FAR sem_t *sem)
{
  int ret;
  int semcount;

  ret = nxsem_get_value(sem, &semcount);
  if ((ret == OK) && (semcount < 0))
    {
      return true;
    }
  else
    {
      return false;
    }
}

static int video_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR video_mng_t  *priv  = (FAR video_mng_t *)inode->i_private;
  int ret = OK;

  video_lock(&priv->lock_open_num);
  if (priv->open_num == 0)
    {
      /* Only in first execution, open device */

      ret = g_video_devops->open(priv);
      if (ret == OK)
        {
          initialize_resources(priv);
        }
    }

  /* In second or later execution, ret is initial value(=OK) */

  if (ret == OK)
    {
      priv->open_num++;
    }

  video_unlock(&priv->lock_open_num);

  return ret;
}

static int video_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR video_mng_t  *priv  = (FAR video_mng_t *)inode->i_private;
  int ret = ERROR;

  video_lock(&priv->lock_open_num);
  if (priv->open_num == 0)
    {
      return OK;
    }

  priv->open_num--;

  if (priv->open_num == 0)
    {
      cleanup_resources(priv);
      g_video_devops->close();
    }

  video_unlock(&priv->lock_open_num);

  return ret;
}

static int video_reqbufs(FAR struct video_mng_s         *vmng,
                         FAR struct v4l2_requestbuffers *reqbufs)
{
  int ret = OK;
  FAR video_type_inf_t *type_inf;
  irqstate_t           flags;

  if ((vmng == NULL) || (reqbufs == NULL))
    {
      return -EINVAL;
    }

  type_inf = get_video_type_inf(vmng, reqbufs->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  if (type_inf->state == VIDEO_STATE_DMA)
    {
      /* In DMA, REQBUFS is not permitted */

      ret = -EPERM;
    }
  else
    {
      video_framebuff_change_mode(&type_inf->bufinf, reqbufs->mode);

      ret = video_framebuff_realloc_container(&type_inf->bufinf,
                                              reqbufs->count);
    }

  leave_critical_section(flags);

  return ret;
}

static int video_qbuf(FAR struct video_mng_s *vmng,
                      FAR struct v4l2_buffer *buf)
{
  FAR video_type_inf_t *type_inf;
  FAR vbuf_container_t *container;
  enum video_state_e   next_video_state;
  irqstate_t           flags;

  if ((vmng == NULL) || (buf == NULL))
    {
      return -EINVAL;
    }

  type_inf = get_video_type_inf(vmng, buf->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (!is_bufsize_sufficient(vmng, buf->length))
    {
      return -EINVAL;
    }

  container = video_framebuff_get_container(&type_inf->bufinf);
  if (container == NULL)
    {
      return -ENOMEM;
    }

  memcpy(&container->buf, buf, sizeof(struct v4l2_buffer));
  video_framebuff_queue_container(&type_inf->bufinf, container);

  video_lock(&type_inf->lock_state);
  flags = enter_critical_section();
  if (type_inf->state == VIDEO_STATE_STREAMON)
    {
      leave_critical_section(flags);

      if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
        {
          video_lock(&vmng->still_inf.lock_state);
          next_video_state = estimate_next_video_state
                             (vmng, CAUSE_VIDEO_START);
          change_video_state(vmng, next_video_state);
          video_unlock(&vmng->still_inf.lock_state);
        }
      else
        {
          container = video_framebuff_get_dma_container(&type_inf->bufinf);
          if (container)
            {
              g_video_devops->set_buftype(buf->type);
              g_video_devops->set_buf(container->buf.m.userptr,
                                      container->buf.length);
              type_inf->state = VIDEO_STATE_DMA;
            }
        }
    }
  else
    {
      leave_critical_section(flags);
    }

  video_unlock(&type_inf->lock_state);

  return OK;
}

static int video_dqbuf(FAR struct video_mng_s *vmng,
                       FAR struct v4l2_buffer *buf)
{
  irqstate_t           flags;
  FAR video_type_inf_t *type_inf;
  FAR vbuf_container_t *container;
  sem_t                *dqbuf_wait_flg;
  enum video_state_e   next_video_state;

  if ((vmng == NULL) || (buf == NULL))
    {
      return -EINVAL;
    }

  type_inf = get_video_type_inf(vmng, buf->type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  container = video_framebuff_dq_valid_container(&type_inf->bufinf);
  if (container == NULL)
    {
      /* Not yet done DMA. Wait done */

      dqbuf_wait_flg = &type_inf->wait_dma.dqbuf_wait_flg;

      /* Loop until semaphore is unlocked by DMA done or DQCANCEL */

      do
        {
          if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
            {
              /* If start DMA condition is satisfied, start DMA */

              flags = enter_critical_section();
              next_video_state = estimate_next_video_state
                                  (vmng, CAUSE_VIDEO_DQBUF);
              change_video_state(vmng, next_video_state);
              leave_critical_section(flags);
            }

          nxsem_wait(dqbuf_wait_flg);
        }
      while (type_inf->wait_dma.waitend_cause ==
                   VIDEO_WAITEND_CAUSE_STILLSTOP);

      container = type_inf->wait_dma.done_container;

      if (!container)
        {
          /* Waking up without DMA data means abort.
           * Therefore, Check cause.
           */

          if (type_inf->wait_dma.waitend_cause
               == VIDEO_WAITEND_CAUSE_DQCANCEL)
            {
              return -ECANCELED;
            }
        }

      type_inf->wait_dma.done_container = NULL;
    }

  memcpy(buf, &container->buf, sizeof(struct v4l2_buffer));

  video_framebuff_free_container(&type_inf->bufinf, container);

  return OK;
}

static int video_cancel_dqbuf(FAR struct video_mng_s *vmng,
                              enum v4l2_buf_type type)
{
  FAR video_type_inf_t *type_inf;

  type_inf = get_video_type_inf(vmng, type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (!is_sem_waited(&type_inf->wait_dma.dqbuf_wait_flg))
    {
      /* In not waiting DQBUF case, return OK */

      return OK;
    }

  type_inf->wait_dma.waitend_cause = VIDEO_WAITEND_CAUSE_DQCANCEL;

  /* If DMA is done before nxsem_post, cause is overwritten */

  nxsem_post(&type_inf->wait_dma.dqbuf_wait_flg);

  return OK;
}

static int video_enum_fmt(FAR struct v4l2_fmtdesc *fmt)
{
  int ret;

  if ((g_video_devops == NULL) ||
      (g_video_devops->get_range_of_fmt == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_devops->get_range_of_fmt(fmt);

  return ret;
}

static int video_enum_framesizes(FAR struct v4l2_frmsizeenum *frmsize)
{
  int ret;

  if ((g_video_devops == NULL) ||
      (g_video_devops->get_range_of_framesize == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_devops->get_range_of_framesize(frmsize);

  return ret;
}

static int video_try_fmt(FAR struct v4l2_format *fmt)
{
  int ret;

  if ((g_video_devops == NULL) || (g_video_devops->try_format == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_devops->try_format(fmt);

  return ret;
}

static int video_s_fmt(FAR struct video_mng_s *priv,
                       FAR struct v4l2_format *fmt)
{
  int ret;

  if ((g_video_devops == NULL) || (g_video_devops->set_format == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_devops->set_format(fmt);

  return ret;
}

static int video_enum_frameintervals(FAR struct v4l2_frmivalenum *frmival)
{
  int ret;

  if ((g_video_devops == NULL) ||
      (g_video_devops->get_range_of_frameinterval == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_devops->get_range_of_frameinterval(frmival);

  return ret;
}

static int video_s_parm(FAR struct video_mng_s *priv,
                        FAR struct v4l2_streamparm *parm)
{
  int ret;

  if ((g_video_devops == NULL) ||
      (g_video_devops->set_frameinterval == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_devops->set_frameinterval(parm);

  return ret;
}

static int video_streamon(FAR struct video_mng_s *vmng,
                          FAR enum v4l2_buf_type *type)
{
  FAR video_type_inf_t *type_inf;
  enum video_state_e   next_video_state;
  int                  ret = OK;

  if ((vmng == NULL) || (type == NULL))
    {
      return -EINVAL;
    }

  type_inf = get_video_type_inf(vmng, *type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
      /* No procedure for VIDIOC_STREAMON(STILL_CAPTURE) */

      return OK;
    }

  video_lock(&type_inf->lock_state);

  if (type_inf->state != VIDEO_STATE_STREAMOFF)
    {
      ret = -EPERM;
    }
  else
    {
      next_video_state = estimate_next_video_state
                          (vmng, CAUSE_VIDEO_START);
      change_video_state(vmng, next_video_state);
    }

  video_unlock(&type_inf->lock_state);

  return ret;
}

static int video_streamoff(FAR struct video_mng_s *vmng,
                           FAR enum v4l2_buf_type *type)
{
  FAR video_type_inf_t *type_inf;
  enum video_state_e   next_video_state;
  irqstate_t           flags;
  int                  ret = OK;

  if ((vmng == NULL) || (type == NULL))
    {
      return -EINVAL;
    }

  type_inf = get_video_type_inf(vmng, *type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
      /* No procedure for VIDIOC_STREAMOFF(STILL_CAPTURE) */

      return OK;
    }

  flags = enter_critical_section();

  if (type_inf->state == VIDEO_STATE_STREAMOFF)
    {
      ret = -EPERM;
    }
  else
    {
      next_video_state = estimate_next_video_state
                          (vmng, CAUSE_VIDEO_STOP);
      change_video_state(vmng, next_video_state);
    }

  leave_critical_section(flags);

  return ret;
}

static int video_do_halfpush(bool enable)
{
  if ((g_video_devops == NULL) || (g_video_devops->do_halfpush == NULL))
    {
      return -EINVAL;
    }

  return g_video_devops->do_halfpush(enable);
}

static int video_takepict_start(FAR struct video_mng_s *vmng,
                                int32_t capture_num)
{
  irqstate_t           flags;
  enum video_state_e   next_video_state;
  FAR vbuf_container_t *dma_container;
  int                  ret = OK;

  if (vmng == NULL)
    {
      return -EINVAL;
    }

  video_lock(&vmng->still_inf.lock_state);

  if (vmng->still_inf.state != VIDEO_STATE_STREAMOFF)
    {
    ret = -EPERM;
    }
  else
    {
      if (capture_num > 0)
        {
         vmng->still_inf.remaining_capnum = capture_num;
        }
      else
        {
         vmng->still_inf.remaining_capnum = VIDEO_REMAINING_CAPNUM_INFINITY;
        }

      /* Control video stream prior to still stream */

      flags = enter_critical_section();

      next_video_state = estimate_next_video_state(vmng,
                                                   CAUSE_STILL_START);
      change_video_state(vmng, next_video_state);

      leave_critical_section(flags);

      dma_container = video_framebuff_get_dma_container
                       (&vmng->still_inf.bufinf);
      if (dma_container)
        {
          /* Start video stream DMA */

          g_video_devops->set_buftype(V4L2_BUF_TYPE_STILL_CAPTURE);
          g_video_devops->set_buf(dma_container->buf.m.userptr,
                                  dma_container->buf.length);
          vmng->still_inf.state = VIDEO_STATE_DMA;
        }
      else
        {
          vmng->still_inf.state = VIDEO_STATE_STREAMON;
        }
    }

  video_unlock(&vmng->still_inf.lock_state);

  return ret;
}

static int video_takepict_stop(FAR struct video_mng_s *vmng, bool halfpush)
{
  int        ret = OK;
  irqstate_t flags;
  enum video_state_e next_video_state;

  if (vmng == NULL)
    {
      return -EINVAL;
    }

  video_lock(&vmng->still_inf.lock_state);

  if ((vmng->still_inf.state == VIDEO_STATE_STREAMOFF) &&
      (vmng->still_inf.remaining_capnum == VIDEO_REMAINING_CAPNUM_INFINITY))
    {
      ret = -EPERM;
    }
  else
    {
      flags = enter_critical_section();
      if (vmng->still_inf.state == VIDEO_STATE_DMA)
        {
          g_video_devops->cancel_dma();
        }

      leave_critical_section(flags);

      vmng->still_inf.state = VIDEO_STATE_STREAMOFF;
      vmng->still_inf.remaining_capnum = VIDEO_REMAINING_CAPNUM_INFINITY;

      /* Control video stream */

      video_lock(&vmng->video_inf.lock_state);
      next_video_state = estimate_next_video_state(vmng,
                                                   CAUSE_STILL_STOP);
      change_video_state(vmng, next_video_state);
      video_unlock(&vmng->video_inf.lock_state);
    }

  video_unlock(&vmng->still_inf.lock_state);

  return ret;
}

static int video_queryctrl(FAR struct v4l2_queryctrl *ctrl)
{
  int                        ret;
  struct v4l2_query_ext_ctrl ext_ctrl;

  if (ctrl == NULL)
    {
      return -EINVAL;
    }

  /* Replace to VIDIOC_QUERY_EXT_CTRL format */

  ext_ctrl.ctrl_class     = ctrl->ctrl_class;
  ext_ctrl.id             = ctrl->id;
  ext_ctrl.type           = ctrl->type;
  ext_ctrl.minimum        = ctrl->minimum;
  ext_ctrl.maximum        = ctrl->maximum;
  ext_ctrl.step           = ctrl->step;
  ext_ctrl.default_value  = ctrl->default_value;
  ext_ctrl.flags          = ctrl->flags;

  ret = video_query_ext_ctrl(&ext_ctrl);

  if (ret != OK)
    {
      return ret;
    }

  if ((ext_ctrl.type == V4L2_CTRL_TYPE_INTEGER64) ||
      (ext_ctrl.type == V4L2_CTRL_TYPE_U8) ||
      (ext_ctrl.type == V4L2_CTRL_TYPE_U16) ||
      (ext_ctrl.type == V4L2_CTRL_TYPE_U32))
    {
      /* Unsupported type in VIDIOC_QUERYCTRL */

      return -EINVAL;
    }

  /* Replace gotten value to VIDIOC_QUERYCTRL */

  ctrl->type          = ext_ctrl.type;
  ctrl->minimum       = ext_ctrl.minimum;
  ctrl->maximum       = ext_ctrl.maximum;
  ctrl->step          = ext_ctrl.step;
  ctrl->default_value = ext_ctrl.default_value;
  ctrl->flags         = ext_ctrl.flags;
  strncpy(ctrl->name, ext_ctrl.name, sizeof(ctrl->name));

  return OK;
}

static int video_query_ext_ctrl(FAR struct v4l2_query_ext_ctrl *ctrl)
{
  int ret;

  if ((g_video_devops == NULL) ||
      (g_video_devops->get_range_of_ctrlvalue == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_devops->get_range_of_ctrlvalue(ctrl);

  return ret;
}

static int video_querymenu(FAR struct v4l2_querymenu *menu)
{
  int ret;

  if ((g_video_devops == NULL) ||
      (g_video_devops->get_menu_of_ctrlvalue == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_devops->get_menu_of_ctrlvalue(menu);

  return ret;
}

static int video_g_ctrl(FAR struct video_mng_s *priv,
                        FAR struct v4l2_control *ctrl)
{
  int                      ret;
  struct v4l2_ext_controls ext_controls;
  struct v4l2_ext_control  control;

  if (ctrl == NULL)
    {
      return -EINVAL;
    }

  /* Replace to VIDIOC_G_EXT_CTRLS format */

  control.id    = ctrl->id;

  ext_controls.ctrl_class = V4L2_CTRL_CLASS_USER;
  ext_controls.count      = 1;
  ext_controls.controls   = &control;

  /* Execute VIDIOC_G_EXT_CTRLS */

  ret = video_g_ext_ctrls(priv, &ext_controls);

  if (ret == OK)
    {
      /* Replace gotten value to VIDIOC_G_CTRL parameter */

      ctrl->value = control.value;
    }

  return ret;
}

static int video_s_ctrl(FAR struct video_mng_s *priv,
                        FAR struct v4l2_control *ctrl)
{
  int ret;
  struct v4l2_ext_controls ext_controls;
  struct v4l2_ext_control  control;

  if (ctrl == NULL)
    {
      return -EINVAL;
    }

  /* Replace to VIDIOC_S_EXT_CTRLS format */

  control.id    = ctrl->id;
  control.value = ctrl->value;

  ext_controls.ctrl_class = V4L2_CTRL_CLASS_USER;
  ext_controls.count      = 1;
  ext_controls.controls   = &control;

  /* Execute VIDIOC_S_EXT_CTRLS */

  ret = video_s_ext_ctrls(priv, &ext_controls);

  return ret;
}

static int video_g_ext_ctrls(FAR struct video_mng_s *priv,
                             FAR struct v4l2_ext_controls *ctrls)
{
  int ret = OK;
  int cnt;
  FAR struct v4l2_ext_control *control;

  if ((priv == NULL) || (ctrls == NULL))
    {
      return -EINVAL;
    }

  for (cnt = 0, control = ctrls->controls;
       cnt < ctrls->count;
       cnt++, control++)
    {
      ret = g_video_devops->get_ctrlvalue(ctrls->ctrl_class, control);

      if (ret < 0)
        {
          /* Set cnt in that error occurred */

          ctrls->error_idx = cnt;
          return ret;
        }
    }

  return ret;
}

static int video_s_ext_ctrls(FAR struct video_mng_s *priv,
                             FAR struct v4l2_ext_controls *ctrls)
{
  int ret = OK;
  int cnt;
  FAR struct v4l2_ext_control *control;

  if ((priv == NULL) || (ctrls == NULL))
    {
      return -EINVAL;
    }

  for (cnt = 0, control = ctrls->controls;
       cnt < ctrls->count;
       cnt++, control++)
    {
      ret = g_video_devops->set_ctrlvalue(ctrls->ctrl_class, control);

      if (ret < 0)
        {
          /* Set cnt in that error occurred */

          ctrls->error_idx = cnt;
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: video_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int video_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR video_mng_t  *priv  = (FAR video_mng_t *)inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      case VIDIOC_REQBUFS:
        ret = video_reqbufs(priv, (FAR struct v4l2_requestbuffers *)arg);

        break;

      case VIDIOC_QBUF:
        ret = video_qbuf(priv, (FAR struct v4l2_buffer *)arg);

        break;

      case VIDIOC_DQBUF:
        ret = video_dqbuf(priv, (FAR struct v4l2_buffer *)arg);

        break;

      case VIDIOC_CANCEL_DQBUF:
        ret = video_cancel_dqbuf(priv, (FAR enum v4l2_buf_type)arg);

        break;

      case VIDIOC_STREAMON:
        ret = video_streamon(priv, (FAR enum v4l2_buf_type *)arg);

        break;

      case VIDIOC_STREAMOFF:
        ret = video_streamoff(priv, (FAR enum v4l2_buf_type *)arg);

        break;

      case VIDIOC_DO_HALFPUSH:
        ret = video_do_halfpush(arg);

        break;

      case VIDIOC_TAKEPICT_START:
        ret = video_takepict_start(priv, (int32_t)arg);

        break;

      case VIDIOC_TAKEPICT_STOP:
        ret = video_takepict_stop(priv, arg);

        break;

      case VIDIOC_ENUM_FMT:
        ret = video_enum_fmt((FAR struct v4l2_fmtdesc *)arg);

        break;

      case VIDIOC_ENUM_FRAMESIZES:
        ret = video_enum_framesizes((FAR struct v4l2_frmsizeenum *)arg);

        break;

      case VIDIOC_TRY_FMT:
        ret = video_try_fmt((FAR struct v4l2_format *)arg);

        break;

      case VIDIOC_S_FMT:
        ret = video_s_fmt(priv, (FAR struct v4l2_format *)arg);

        break;

      case VIDIOC_ENUM_FRAMEINTERVALS:
        ret = video_enum_frameintervals((FAR struct v4l2_frmivalenum *)arg);

        break;

      case VIDIOC_S_PARM:
        ret = video_s_parm(priv, (FAR struct v4l2_streamparm *)arg);

        break;

      case VIDIOC_QUERYCTRL:
        ret = video_queryctrl((FAR struct v4l2_queryctrl *)arg);

        break;

      case VIDIOC_QUERY_EXT_CTRL:
        ret = video_query_ext_ctrl((FAR struct v4l2_query_ext_ctrl *)arg);

        break;

      case VIDIOC_QUERYMENU:
        ret = video_querymenu((FAR struct v4l2_querymenu *)arg);

        break;

      case VIDIOC_G_CTRL:
        ret = video_g_ctrl(priv, (FAR struct v4l2_control *)arg);

        break;

      case VIDIOC_S_CTRL:
        ret = video_s_ctrl(priv, (FAR struct v4l2_control *)arg);

        break;

      case VIDIOC_G_EXT_CTRLS:
        ret = video_g_ext_ctrls(priv, (FAR struct v4l2_ext_controls *)arg);

        break;

      case VIDIOC_S_EXT_CTRLS:
        ret = video_s_ext_ctrls(priv, (FAR struct v4l2_ext_controls *)arg);

        break;

      default:
        videoerr("Unrecognized cmd: %d\n", cmd);
        ret = - ENOTTY;
        break;
    }

  return ret;
}

static int video_poll_setup(FAR struct video_mng_s *priv,
                            FAR struct pollfd      *fds)
{
  if ((fds->events & POLLIN) == 0)
    {
      return -EDEADLK;
    }

  /* TODO: If data exists, get and nxsem_post If no data, wait dma */

  return OK;
}

static int video_poll_teardown(FAR struct video_mng_s *priv,
                               FAR struct pollfd      *fds)
{
  /* TODO: Delete poll wait information */

  return OK;
}

static int video_poll(FAR struct file   *filep,
                      FAR struct pollfd *fds,
                      bool              setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR video_mng_t  *priv  = inode->i_private;

  if (setup)
    {
      return video_poll_setup(priv, fds);
    }
  else
    {
      return video_poll_teardown(priv, fds);
    }

  return OK;
}

static FAR void *video_register(FAR const char *devpath)
{
  FAR    video_mng_t *priv;
  int    ret;
  size_t allocsize;

  /* Input devpath Error Check */

  if (!devpath)
    {
      return NULL;
    }

  allocsize = strnlen(devpath, MAX_VIDEO_FILE_PATH - 1/* Space for '\0' */);
  if ((allocsize < 2)     ||
      (devpath[0] != '/') ||
      ((allocsize == (MAX_VIDEO_FILE_PATH - 1)) &&
       (devpath[MAX_VIDEO_FILE_PATH] != '\0')))
    {
      return NULL;
    }

  /* Initialize video device structure */

  priv = (FAR video_mng_t *)kmm_malloc(sizeof(video_mng_t));
  if (!priv)
    {
      videoerr("Failed to allocate instance\n");
      return NULL;
    }

  memset(priv, 0, sizeof(video_mng_t));

  /* Save device path */

  priv->devpath = (FAR char *)kmm_malloc(allocsize + 1);
  if (!priv->devpath)
    {
      kmm_free(priv);
      return NULL;
    }

  memcpy(priv->devpath, devpath, allocsize);
  priv->devpath[allocsize] = '\0';

  /* Initialize semaphore */

  nxsem_init(&priv->lock_open_num, 0, 1);

  /* Register the character driver */

  ret = register_driver(priv->devpath, &g_video_fops, 0666, priv);
  if (ret < 0)
    {
      videoerr("Failed to register driver: %d\n", ret);
      kmm_free(priv->devpath);
      kmm_free(priv);
      return NULL;
    }

  return (FAR void *)priv;
}

static int video_unregister(FAR video_mng_t *v_mgr)
{
  int ret = OK;

  if (!v_mgr)
    {
      ret = -ENODEV;
    }
  else
    {
      nxsem_destroy(&v_mgr->lock_open_num);

      unregister_driver((const char *)v_mgr->devpath);

      kmm_free(v_mgr->devpath);
      kmm_free(v_mgr);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int video_initialize(FAR const char *devpath,
                     FAR const struct video_devops_s *devops)
{
  if (is_initialized)
    {
      return OK;
    }

  video_handler = video_register(devpath);

  g_video_devops = devops;

  is_initialized = true;

  return OK;
}

int video_uninitialize(void)
{
  if (is_initialized)
    {
      return OK;
    }

  video_unregister(video_handler);

  g_video_devops = NULL;

  is_initialized = false;

  return OK;
}

int video_common_notify_dma_done(uint8_t  err_code,
                                 uint32_t buf_type,
                                 uint32_t datasize,
                                 FAR void *priv)
{
  FAR video_mng_t      *vmng = (FAR video_mng_t *)priv;
  FAR video_type_inf_t *type_inf;
  FAR vbuf_container_t *container = NULL;

  type_inf = get_video_type_inf(vmng, buf_type);
  if (type_inf == NULL)
    {
      return -EINVAL;
    }

  if (err_code == 0)
    {
      type_inf->bufinf.vbuf_dma->buf.flags = 0;
      if (type_inf->remaining_capnum > 0)
        {
          type_inf->remaining_capnum--;
        }
    }
  else
    {
      type_inf->bufinf.vbuf_dma->buf.flags = V4L2_BUF_FLAG_ERROR;
    }

  type_inf->bufinf.vbuf_dma->buf.bytesused = datasize;
  video_framebuff_dma_done(&type_inf->bufinf);

  if (is_sem_waited(&type_inf->wait_dma.dqbuf_wait_flg))
    {
      /* If waiting DMA done in DQBUF,
       * get/save container and unlock wait
       */

      type_inf->wait_dma.done_container
        = video_framebuff_pop_curr_container(&type_inf->bufinf);
      type_inf->wait_dma.waitend_cause
        = VIDEO_WAITEND_CAUSE_DMADONE;
      nxsem_post(&type_inf->wait_dma.dqbuf_wait_flg);

      /* TODO:  in poll wait, unlock wait */
    }

  if (type_inf->remaining_capnum == 0)
    {
      g_video_devops->cancel_dma();
      type_inf->state = VIDEO_STATE_STREAMOFF;

      /* If stop still stream, notify it to video stream */

      if ((buf_type == V4L2_BUF_TYPE_STILL_CAPTURE) &&
           is_sem_waited(&vmng->video_inf.wait_dma.dqbuf_wait_flg))
        {
          vmng->video_inf.wait_dma.waitend_cause
            = VIDEO_WAITEND_CAUSE_STILLSTOP;
          nxsem_post(&vmng->video_inf.wait_dma.dqbuf_wait_flg);
        }
    }
  else
    {
      container = video_framebuff_get_dma_container(&type_inf->bufinf);
      if (!container)
        {
          g_video_devops->cancel_dma();
          type_inf->state = VIDEO_STATE_STREAMON;
        }
      else
        {
          g_video_devops->set_buf(container->buf.m.userptr,
                                  container->buf.length);
        }
    }

  return OK;
}
