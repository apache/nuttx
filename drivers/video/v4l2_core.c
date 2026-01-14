/****************************************************************************
 * drivers/video/v4l2_core.c
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

#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/video/video.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods. */

static int v4l2_open(FAR struct file *filep);
static int v4l2_close(FAR struct file *filep);
static ssize_t v4l2_read(FAR struct file *filep,
                         FAR char *buffer, size_t buflen);
static ssize_t v4l2_write(FAR struct file *filep,
                          FAR const char *buffer, size_t buflen);
static int v4l2_ioctl(FAR struct file *filep,
                      int cmd, unsigned long arg);
static int v4l2_mmap(FAR struct file *filep,
                     FAR struct mm_map_entry_s *map);
static int v4l2_poll(FAR struct file *filep,
                     FAR struct pollfd *fds, bool setup);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int v4l2_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

static const struct file_operations g_v4l2_fops =
{
  v4l2_open,               /* open */
  v4l2_close,              /* close */
  v4l2_read,               /* read */
  v4l2_write,              /* write */
  NULL,                    /* seek */
  v4l2_ioctl,              /* ioctl */
  v4l2_mmap,               /* mmap */
  NULL,                    /* truncate */
  v4l2_poll,               /* poll */
  NULL,                    /* readv */
  NULL,                    /* writev */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  v4l2_unlink,             /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int v4l2_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct v4l2_s *v4l2 = inode->i_private;

  if (v4l2->fops->open != NULL)
    {
      return v4l2->fops->open(filep);
    }

  return -ENOTSUP;
}

static int v4l2_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct v4l2_s *v4l2 = inode->i_private;

  if (v4l2->fops->close != NULL)
    {
      return v4l2->fops->close(filep);
    }

  return -ENOTSUP;
}

static ssize_t v4l2_read(FAR struct file *filep,
                         FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct v4l2_s *v4l2 = inode->i_private;

  if (v4l2->fops->read != NULL)
    {
      return v4l2->fops->read(filep, buffer, buflen);
    }

  return -ENOTSUP;
}

static ssize_t v4l2_write(FAR struct file *filep,
                          FAR const char  *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct v4l2_s *v4l2 = inode->i_private;

  if (v4l2->fops->write != NULL)
    {
      return v4l2->fops->write(filep, buffer, buflen);
    }

  return -ENOTSUP;
}

static int v4l2_ioctl(FAR struct file *filep,
                      int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct v4l2_s *v4l2 = inode->i_private;

  switch (cmd)
    {
      case VIDIOC_QUERYCAP:
        if (v4l2->vops->querycap == NULL)
          {
            break;
          }

        return v4l2->vops->querycap(filep,
                             (FAR struct v4l2_capability *)arg);

      case VIDIOC_G_INPUT:
        if (v4l2->vops->g_input == NULL)
          {
            break;
          }

        return v4l2->vops->g_input((FAR int *)arg);

      case VIDIOC_ENUMINPUT:
        if (v4l2->vops->enum_input == NULL)
          {
            break;
          }

        return v4l2->vops->enum_input(filep,
                             (FAR struct v4l2_input *)arg);

      case VIDIOC_REQBUFS:
        if (v4l2->vops->reqbufs == NULL)
          {
            break;
          }

        return v4l2->vops->reqbufs(filep,
                             (FAR struct v4l2_requestbuffers *)arg);

      case VIDIOC_QUERYBUF:
        if (v4l2->vops->querybuf == NULL)
          {
            break;
          }

        return v4l2->vops->querybuf(filep,
                             (FAR struct v4l2_buffer *)arg);

      case VIDIOC_QBUF:
        if (v4l2->vops->qbuf == NULL)
          {
            break;
          }

        return v4l2->vops->qbuf(filep,
                             (FAR struct v4l2_buffer *)arg);

      case VIDIOC_DQBUF:
        if (v4l2->vops->dqbuf == NULL)
          {
            break;
          }

        return v4l2->vops->dqbuf(filep,
                             (FAR struct v4l2_buffer *)arg);

      case VIDIOC_CANCEL_DQBUF:
        if (v4l2->vops->cancel_dqbuf == NULL)
          {
            break;
          }

        return v4l2->vops->cancel_dqbuf(filep,
                             (FAR enum v4l2_buf_type)arg);

      case VIDIOC_STREAMON:
        if (v4l2->vops->streamon == NULL)
          {
            break;
          }

        return v4l2->vops->streamon(filep,
                             (FAR enum v4l2_buf_type *)arg);

      case VIDIOC_STREAMOFF:
        if (v4l2->vops->streamoff == NULL)
          {
            break;
          }

        return v4l2->vops->streamoff(filep,
                             (FAR enum v4l2_buf_type *)arg);

      case VIDIOC_DO_HALFPUSH:
        if (v4l2->vops->do_halfpush == NULL)
          {
            break;
          }

        return v4l2->vops->do_halfpush(filep, arg);

      case VIDIOC_TAKEPICT_START:
        if (v4l2->vops->takepict_start == NULL)
          {
            break;
          }

        return v4l2->vops->takepict_start(filep, (int32_t)arg);

      case VIDIOC_TAKEPICT_STOP:
        if (v4l2->vops->takepict_stop == NULL)
          {
            break;
          }

        return v4l2->vops->takepict_stop(filep, arg);

      case VIDIOC_S_SELECTION:
        if (v4l2->vops->s_selection == NULL)
          {
            break;
          }

        return v4l2->vops->s_selection(filep,
                             (FAR struct v4l2_selection *)arg);

      case VIDIOC_G_SELECTION:
        if (v4l2->vops->g_selection == NULL)
          {
            break;
          }

        return v4l2->vops->g_selection(filep,
                             (FAR struct v4l2_selection *)arg);

      case VIDIOC_TRY_FMT:
        if (v4l2->vops->try_fmt == NULL)
          {
            break;
          }

        return v4l2->vops->try_fmt(filep,
                             (FAR struct v4l2_format *)arg);

      case VIDIOC_G_FMT:
        if (v4l2->vops->g_fmt == NULL)
          {
            break;
          }

        return v4l2->vops->g_fmt(filep,
                             (FAR struct v4l2_format *)arg);

      case VIDIOC_S_FMT:
        if (v4l2->vops->s_fmt == NULL)
          {
            break;
          }

        return v4l2->vops->s_fmt(filep,
                             (FAR struct v4l2_format *)arg);

      case VIDIOC_S_PARM:
        if (v4l2->vops->s_parm == NULL)
          {
            break;
          }

        return v4l2->vops->s_parm(filep,
                             (FAR struct v4l2_streamparm *)arg);

      case VIDIOC_G_PARM:
        if (v4l2->vops->g_parm == NULL)
          {
            break;
          }

        return v4l2->vops->g_parm(filep,
                             (FAR struct v4l2_streamparm *)arg);

      case VIDIOC_QUERYCTRL:
        if (v4l2->vops->queryctrl == NULL)
          {
            break;
          }

        return v4l2->vops->queryctrl(filep,
                             (FAR struct v4l2_queryctrl *)arg);

      case VIDIOC_QUERY_EXT_CTRL:
        if (v4l2->vops->query_ext_ctrl == NULL)
          {
            break;
          }

        return v4l2->vops->query_ext_ctrl(filep,
                             (FAR struct v4l2_query_ext_ctrl *)arg);

      case VIDIOC_QUERYMENU:
        if (v4l2->vops->querymenu == NULL)
          {
            break;
          }

        return v4l2->vops->querymenu(filep,
                             (FAR struct v4l2_querymenu *)arg);

      case VIDIOC_G_CTRL:
        if (v4l2->vops->g_ctrl == NULL)
          {
            break;
          }

        return v4l2->vops->g_ctrl(filep,
                             (FAR struct v4l2_control *)arg);

      case VIDIOC_S_CTRL:
        if (v4l2->vops->s_ctrl == NULL)
          {
            break;
          }

        return v4l2->vops->s_ctrl(filep,
                             (FAR struct v4l2_control *)arg);

      case VIDIOC_G_EXT_CTRLS:
        if (v4l2->vops->g_ext_ctrls == NULL)
          {
            break;
          }

        return v4l2->vops->g_ext_ctrls(filep,
                             (FAR struct v4l2_ext_controls *)arg);

      case VIDIOC_S_EXT_CTRLS:
        if (v4l2->vops->s_ext_ctrls == NULL)
          {
            break;
          }

        return v4l2->vops->s_ext_ctrls(filep,
                             (FAR struct v4l2_ext_controls *)arg);

      case VIDIOC_G_STD:
        break;

      case VIDIOC_S_STD:
        break;

      case V4SIOC_QUERY_EXT_CTRL_SCENE:
        if (v4l2->vops->query_ext_ctrl_scene == NULL)
          {
            break;
          }

        return v4l2->vops->query_ext_ctrl_scene(filep,
                             (FAR struct v4s_query_ext_ctrl_scene *)arg);

      case V4SIOC_QUERYMENU_SCENE:
        if (v4l2->vops->querymenu_scene == NULL)
          {
            break;
          }

        return v4l2->vops->querymenu_scene(filep,
                             (FAR struct v4s_querymenu_scene *)arg);

      case V4SIOC_G_EXT_CTRLS_SCENE:
        if (v4l2->vops->g_ext_ctrls_scene == NULL)
          {
            break;
          }

        return v4l2->vops->g_ext_ctrls_scene(filep,
                             (FAR struct v4s_ext_controls_scene *)arg);

      case V4SIOC_S_EXT_CTRLS_SCENE:
        if (v4l2->vops->s_ext_ctrls_scene == NULL)
          {
            break;
          }

        return v4l2->vops->s_ext_ctrls_scene(filep,
                             (FAR struct v4s_ext_controls_scene *)arg);

      case VIDIOC_ENUM_FMT:
        if (v4l2->vops->enum_fmt == NULL)
          {
            break;
          }

        return v4l2->vops->enum_fmt(filep,
                             (FAR struct v4l2_fmtdesc *)arg);

      case VIDIOC_ENUM_FRAMEINTERVALS:
        if (v4l2->vops->enum_frminterval == NULL)
          {
            break;
          }

        return v4l2->vops->enum_frminterval(filep,
                             (FAR struct v4l2_frmivalenum *)arg);

      case VIDIOC_ENUM_FRAMESIZES:
        if (v4l2->vops->enum_frmsize == NULL)
          {
            break;
          }

        return v4l2->vops->enum_frmsize(filep,
                             (FAR struct v4l2_frmsizeenum *)arg);

      case VIDIOC_CROPCAP:
        if (v4l2->vops->cropcap == NULL)
          {
            break;
          }

        return v4l2->vops->cropcap(filep,
                             (FAR struct v4l2_cropcap *)arg);

      case VIDIOC_DQEVENT:
        if (v4l2->vops->dqevent == NULL)
          {
            break;
          }

        return v4l2->vops->dqevent(filep,
                             (FAR struct v4l2_event *)arg);

      case VIDIOC_SUBSCRIBE_EVENT:
        if (v4l2->vops->subscribe_event == NULL)
          {
            break;
          }

        return v4l2->vops->subscribe_event(filep,
                             (FAR struct v4l2_event_subscription *)arg);

      case VIDIOC_DECODER_CMD:
        if (v4l2->vops->decoder_cmd == NULL)
          {
            break;
          }

        return v4l2->vops->decoder_cmd(filep,
                             (FAR struct v4l2_decoder_cmd *)arg);

      case VIDIOC_ENCODER_CMD:
        if (v4l2->vops->encoder_cmd == NULL)
          {
            break;
          }

        return v4l2->vops->encoder_cmd(filep,
                             (FAR struct v4l2_encoder_cmd *)arg);

      default:
        verr("Unrecognized cmd: %d\n", cmd);
        break;
    }

  return -ENOTTY;
}

static int v4l2_mmap(FAR struct file *filep,
                     FAR struct mm_map_entry_s *map)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct v4l2_s *v4l2 = inode->i_private;

  if (v4l2->fops->mmap != NULL)
    {
      return v4l2->fops->mmap(filep, map);
    }

  return -ENOTSUP;
}

static int v4l2_poll(FAR struct file *filep,
                     FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct v4l2_s *v4l2 = inode->i_private;

  if (v4l2->fops->poll != NULL)
    {
      return v4l2->fops->poll(filep, fds, setup);
    }

  return -ENOTSUP;
}

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int v4l2_unlink(FAR struct inode *inode)
{
  FAR struct v4l2_s *v4l2 = inode->i_private;

  if (v4l2->fops->unlink != NULL)
    {
      return v4l2->fops->unlink(inode);
    }

  return -ENOTSUP;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int video_register(FAR const char *devpath, FAR struct v4l2_s *v4l2)
{
  int ret;

  /* Input devpath Error Check */

  if (devpath[0] != '/')
    {
      return -EINVAL;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_v4l2_fops, 0666, v4l2);
  if (ret < 0)
    {
      verr("Failed to register driver: %d\n", ret);
      return ret;
    }

  return OK;
}

int video_unregister(FAR const char *devpath)
{
  return unregister_driver(devpath);
}
