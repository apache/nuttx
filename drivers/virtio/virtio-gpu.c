/****************************************************************************
 * drivers/virtio/virtio-gpu.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/video/fb.h>
#include <nuttx/virtio/virtio.h>

#include "virtio-gpu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_GPU_BPP          32
#define VIRTIO_GPU_FB_FMT       FB_FMT_RGB32
#define VIRTIO_GPU_FMT          VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM

#define VIRTIO_GPU_CTL          0
#define VIRTIO_GPU_NUM          1

#define VIRTIO_GPU_MAX_DISP     4
#define VIRTIO_GPU_MAX_PLANE    1
#define VIRTIO_GPU_MAX_NENTS    4

#define VIRTIO_GPU_FB_NUMS      2
#define VIRTIO_GPU_VSYNC_PERIOD 16

#define VIRTIO_GPU_MAP_ERR(e)   ((e) == VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY ? \
                                -ENOMEM : -EINVAL)

/* We use the last bit of address to indicate whether a cmd is blocking */

#define VIRTIO_GPU_CMD_BLOCKING ((uintptr_t)0x01)
#define VIRTIO_GPU_CMD_SET_BLOCKING(p) \
  ((FAR void *)((uintptr_t)(p) | VIRTIO_GPU_CMD_BLOCKING))
#define VIRTIO_GPU_CMD_IS_BLOCKING(p) \
  ((uintptr_t)(p) & VIRTIO_GPU_CMD_BLOCKING)
#define VIRTIO_GPU_CMD_GET_POINTER(p) \
  ((FAR void *)((uintptr_t)(p) & ~VIRTIO_GPU_CMD_BLOCKING))

/****************************************************************************
 * Private Types
 ****************************************************************************/

union virtio_gpu_cmd_u
{
  FAR union virtio_gpu_cmd_u *next;
  struct virtio_gpu_resource_flush flush;
};

struct virtio_gpu_priv_s
{
  struct fb_vtable_s vtable;           /* Must be cast compatible with virtio_gpu_priv_s */
  FAR struct virtio_device *vdev;      /* Contained virtio device */
  FAR union virtio_gpu_cmd_u *freecmd; /* Command data free list */
  FAR uint8_t *fbmem;                  /* Allocated framebuffer */
  size_t fblen;                        /* Size of the framebuffer in bytes */
  fb_coord_t xres;                     /* Horizontal resolution in pixel columns */
  fb_coord_t yres;                     /* Vertical resolution in pixel rows */
  fb_coord_t stride;                   /* Width of a row in bytes */
  uint8_t display;                     /* Display number */
  spinlock_t lock;                     /* Lock */
  struct work_s work;                  /* Work structure for vsync loop */
};

struct virtio_gpu_backing_s
{
  struct virtio_gpu_resource_attach_backing cmd;
  struct virtio_gpu_mem_entry ents[VIRTIO_GPU_MAX_NENTS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int virtio_gpu_send_cmd(FAR struct virtqueue *vq,
                               FAR struct virtqueue_buf *buf_list,
                               int readable, int writable, FAR void *cookie);
static void virtio_gpu_done(FAR struct virtqueue *vq);
static int virtio_gpu_init(FAR struct virtio_gpu_priv_s *priv,
                           FAR struct virtio_device *vdev);
static int virtio_gpu_get_display_info(FAR struct virtio_gpu_priv_s *priv);
static int virtio_gpu_create_2d(FAR struct virtio_gpu_priv_s *priv,
                                int resource_id, int width, int height);
static int virtio_gpu_attach_backing(FAR struct virtio_gpu_priv_s *priv,
                                     int resource_id,
                                     FAR struct virtio_gpu_mem_entry *ents,
                                     uint32_t nents);
static int virtio_gpu_set_scanout(FAR struct virtio_gpu_priv_s *priv,
                                  int scanout_id, int resource_id,
                                  int width, int height);
static int virtio_gpu_transfer_to_host_2d(FAR struct virtio_gpu_priv_s *priv,
                                          int resource_id, int x, int y,
                                          int width, int height);
static int virtio_gpu_flush_resource(FAR struct virtio_gpu_priv_s *priv,
                                     int resource_id, int x, int y,
                                     int width, int height);
static int virtio_gpu_probe(FAR struct virtio_device *vdev);
static void virtio_gpu_remove(FAR struct virtio_device *vdev);
static int virtio_gpu_getvideoinfo(FAR struct fb_vtable_s *vtable,
                                   FAR struct fb_videoinfo_s *vinfo);
static int virtio_gpu_getplaneinfo(FAR struct fb_vtable_s *vtable,
                                   int planeno,
                                   FAR struct fb_planeinfo_s *pinfo);
#ifdef CONFIG_FB_UPDATE
static int virtio_gpu_updatearea(FAR struct fb_vtable_s *vtable,
                                 FAR const struct fb_area_s *area);
#endif
static int virtio_gpu_init_fbmem(FAR struct virtio_device *vdev,
                                 FAR struct virtio_gpu_priv_s *priv,
                                 int fb_count);
static int virtio_gpu_flush_output(FAR struct fb_vtable_s *vtable,
                                   int resource_id);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_driver g_virtio_gpu_driver =
{
  .node   = LIST_INITIAL_VALUE(g_virtio_gpu_driver.node), /* node */
  .device = VIRTIO_ID_GPU,                                /* device id */
  .probe  = virtio_gpu_probe,                             /* probe */
  .remove = virtio_gpu_remove,                            /* remove */
};

static FAR struct virtio_gpu_priv_s *g_virtio_gpu[VIRTIO_GPU_MAX_DISP];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_gpu_zalloc_cmd
 ****************************************************************************/

static FAR void *virtio_gpu_zalloc_cmd(FAR struct virtio_gpu_priv_s *priv)
{
  irqstate_t flags = spin_lock_irqsave(&priv->lock);
  FAR union virtio_gpu_cmd_u *cmd = priv->freecmd;
  if (cmd != NULL)
    {
      priv->freecmd = cmd->next;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  if (cmd != NULL)
    {
      memset(cmd, 0, sizeof(*cmd));
    }
  else
    {
      cmd = virtio_zalloc_buf(priv->vdev, sizeof(*cmd), 16);
    }

  return cmd;
}

/****************************************************************************
 * Name: virtio_gpu_free_cmd
 ****************************************************************************/

static void virtio_gpu_free_cmd(FAR struct virtio_gpu_priv_s *priv,
                                FAR void *cmd)
{
  FAR union virtio_gpu_cmd_u *node = cmd;
  irqstate_t flags = spin_lock_irqsave(&priv->lock);
  node->next = priv->freecmd;
  priv->freecmd = node;
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: virtio_gpu_send_cmd
 * Note: the caller should not touch `buf` after calling this, as it will be
 *       freed either here or in virtio_gpu_done().
 ****************************************************************************/

static int virtio_gpu_send_cmd(FAR struct virtqueue *vq,
                               FAR struct virtqueue_buf *buf_list,
                               int readable, int writable, FAR void *buf)
{
  FAR struct virtio_gpu_priv_s *priv = vq->vq_dev->priv;
  irqstate_t flags;
  int ret;

  if (writable > 0)
    {
      sem_t sem;

      if (buf)
        {
          virtio_gpu_free_cmd(priv, buf);
        }

      nxsem_init(&sem, 0, 0);
      flags = spin_lock_irqsave(&priv->lock);
      ret = virtqueue_add_buffer(vq, buf_list, readable, writable,
                                 VIRTIO_GPU_CMD_SET_BLOCKING(&sem));
      if (ret >= 0)
        {
          virtqueue_kick(vq);
          spin_unlock_irqrestore(&priv->lock, flags);
          nxsem_wait(&sem);
        }
      else
        {
          spin_unlock_irqrestore(&priv->lock, flags);
        }

      nxsem_destroy(&sem);
    }
  else
    {
      flags = spin_lock_irqsave(&priv->lock);
      ret = virtqueue_add_buffer(vq, buf_list, readable, writable, buf);
      if (ret >= 0)
        {
          virtqueue_kick(vq);
        }

      spin_unlock_irqrestore(&priv->lock, flags);

      if (buf && ret < 0)
        {
          virtio_gpu_free_cmd(priv, buf);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_gpu_done
 ****************************************************************************/

static void virtio_gpu_done(FAR struct virtqueue *vq)
{
  FAR struct virtio_gpu_priv_s *priv = vq->vq_dev->priv;
  FAR void *cookie;

  while ((cookie =
          virtqueue_get_buffer_lock(vq, NULL, NULL, &priv->lock)) != NULL)
    {
      if (VIRTIO_GPU_CMD_IS_BLOCKING(cookie))
        {
          nxsem_post(VIRTIO_GPU_CMD_GET_POINTER(cookie));
        }
      else
        {
          virtio_gpu_free_cmd(priv, cookie);
        }
    }
}

/****************************************************************************
 * Name: virtio_gpu_init
 ****************************************************************************/

static int virtio_gpu_init(FAR struct virtio_gpu_priv_s *priv,
                           FAR struct virtio_device *vdev)
{
  FAR const char *vqnames[VIRTIO_GPU_NUM];
  vq_callback callbacks[VIRTIO_GPU_NUM];
  int ret;

  spin_lock_init(&priv->lock);
  priv->vdev = vdev;
  vdev->priv = priv;

  /* Initialize the virtio device */

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
  virtio_set_features(vdev, 0);
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  vqnames[VIRTIO_GPU_CTL]   = "virtio_gpu_ctl";
  callbacks[VIRTIO_GPU_CTL] = virtio_gpu_done;
  ret = virtio_create_virtqueues(vdev, 0, VIRTIO_GPU_NUM, vqnames,
                                 callbacks, NULL);
  if (ret < 0)
    {
      vrterr("virtio_device_create_virtqueue failed, ret=%d", ret);
      return ret;
    }

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);
  return OK;
}

/****************************************************************************
 * Name: virtio_gpu_get_display_info
 ****************************************************************************/

static int virtio_gpu_get_display_info(FAR struct virtio_gpu_priv_s *priv)
{
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_GPU_CTL].vq;
  struct virtio_gpu_ctrl_hdr cmd;
  struct virtio_gpu_resp_display_info info;
  struct virtqueue_buf vb[2];
  int ret;

  memset(&cmd, 0, sizeof(cmd));
  cmd.type = VIRTIO_GPU_CMD_GET_DISPLAY_INFO;

  vb[0].buf = &cmd;
  vb[0].len = sizeof(cmd);
  vb[1].buf = &info;
  vb[1].len = sizeof(info);

  ret = virtio_gpu_send_cmd(vq, vb, 1, 1, NULL);
  if (ret < 0)
    {
      return ret;
    }

  if (info.hdr.type != VIRTIO_GPU_RESP_OK_DISPLAY_INFO)
    {
      return VIRTIO_GPU_MAP_ERR(info.hdr.type);
    }

  priv->xres = info.pmodes[0].r.width;
  priv->yres = info.pmodes[0].r.height;
  vrtinfo("Setting resolution: (%d,%d)", priv->xres, priv->yres);
  return OK;
}

/****************************************************************************
 * Name: virtio_gpu_create_2d
 ****************************************************************************/

static int virtio_gpu_create_2d(FAR struct virtio_gpu_priv_s *priv,
                                int resource_id, int width, int height)
{
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_GPU_CTL].vq;
  struct virtio_gpu_resource_create_2d cmd;
  struct virtio_gpu_ctrl_hdr resp;
  struct virtqueue_buf vb[2];
  int ret;

  memset(&cmd, 0, sizeof(cmd));
  cmd.hdr.type = VIRTIO_GPU_CMD_RESOURCE_CREATE_2D;
  cmd.resource_id = resource_id;
  cmd.format = VIRTIO_GPU_FMT;
  cmd.width = width;
  cmd.height = height;

  vb[0].buf = &cmd;
  vb[0].len = sizeof(cmd);
  vb[1].buf = &resp;
  vb[1].len = sizeof(resp);

  ret = virtio_gpu_send_cmd(vq, vb, 1, 1, NULL);
  if (ret >= 0 && resp.type != VIRTIO_GPU_RESP_OK_NODATA)
    {
      ret = VIRTIO_GPU_MAP_ERR(resp.type);
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_gpu_attach_backing
 ****************************************************************************/

static int virtio_gpu_attach_backing(FAR struct virtio_gpu_priv_s *priv,
                                     int resource_id,
                                     FAR struct virtio_gpu_mem_entry *ents,
                                     uint32_t nents)
{
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_GPU_CTL].vq;
  struct virtio_gpu_backing_s backing;
  struct virtio_gpu_ctrl_hdr resp;
  struct virtqueue_buf vb[2];
  size_t i;
  int ret;

  if (nents > VIRTIO_GPU_MAX_NENTS)
    {
      vrterr("ERROR: Backing memory entries count %" PRId32 "exceeds %d",
             nents, VIRTIO_GPU_MAX_NENTS);
      return -E2BIG;
    }

  memset(&backing.cmd, 0, sizeof(backing.cmd));
  backing.cmd.hdr.type = VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING;
  backing.cmd.resource_id = resource_id;
  backing.cmd.nr_entries = nents;

  for (i = 0; i < nents; i++)
    {
      backing.ents[i] = ents[i];
    }

  vb[0].buf = &backing;
  vb[0].len = sizeof(backing.cmd) + nents * sizeof(backing.ents[0]);
  vb[1].buf = &resp;
  vb[1].len = sizeof(resp);

  ret = virtio_gpu_send_cmd(vq, vb, 1, 1, NULL);
  if (ret >= 0 && resp.type != VIRTIO_GPU_RESP_OK_NODATA)
    {
      ret = VIRTIO_GPU_MAP_ERR(resp.type);
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_gpu_set_scanout
 ****************************************************************************/

static int virtio_gpu_set_scanout(FAR struct virtio_gpu_priv_s *priv,
                                  int scanout_id, int resource_id,
                                  int width, int height)
{
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_GPU_CTL].vq;
  struct virtio_gpu_set_scanout cmd;
  struct virtio_gpu_ctrl_hdr resp;
  struct virtqueue_buf vb[2];
  int ret;

  memset(&cmd, 0, sizeof(cmd));
  cmd.hdr.type = VIRTIO_GPU_CMD_SET_SCANOUT;
  cmd.scanout_id = scanout_id;
  cmd.resource_id = resource_id;
  cmd.r.width = width;
  cmd.r.height = height;

  vb[0].buf = &cmd;
  vb[0].len = sizeof(cmd);
  vb[1].buf = &resp;
  vb[1].len = sizeof(resp);

  ret = virtio_gpu_send_cmd(vq, vb, 1, 1, NULL);
  if (ret >= 0 && resp.type != VIRTIO_GPU_RESP_OK_NODATA)
    {
      ret = VIRTIO_GPU_MAP_ERR(resp.type);
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_gpu_transfer_to_host_2d
 ****************************************************************************/

static int virtio_gpu_transfer_to_host_2d(FAR struct virtio_gpu_priv_s *priv,
                                          int resource_id, int x, int y,
                                          int width, int height)
{
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_GPU_CTL].vq;
  struct virtio_gpu_transfer_to_host_2d cmd;
  struct virtio_gpu_ctrl_hdr resp;
  struct virtqueue_buf vb[2];
  int ret;

  memset(&cmd, 0, sizeof(cmd));

  cmd.hdr.type = VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D;
  cmd.resource_id = resource_id;
  cmd.offset = y * priv->stride + x * (VIRTIO_GPU_BPP >> 3);
  cmd.r.x = x;
  cmd.r.y = y;
  cmd.r.width = width;
  cmd.r.height = height;

  vb[0].buf = &cmd;
  vb[0].len = sizeof(cmd);
  vb[1].buf = &resp;
  vb[1].len = sizeof(resp);

  ret = virtio_gpu_send_cmd(vq, vb, 1, 1, NULL);
  if (ret >= 0 && resp.type != VIRTIO_GPU_RESP_OK_NODATA)
    {
      ret = VIRTIO_GPU_MAP_ERR(resp.type);
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_gpu_flush_resource
 ****************************************************************************/

static int virtio_gpu_flush_resource(FAR struct virtio_gpu_priv_s *priv,
                                     int resource_id, int x, int y,
                                     int width, int height)
{
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_GPU_CTL].vq;
  FAR struct virtio_gpu_resource_flush *cmd;
  struct virtqueue_buf vb;

  cmd = virtio_gpu_zalloc_cmd(priv);
  if (cmd == NULL)
    {
      vrterr("ERROR: Failed to allocate cmd buffer");
      return -ENOMEM;
    }

  cmd->hdr.type = VIRTIO_GPU_CMD_RESOURCE_FLUSH;
  cmd->resource_id = resource_id;
  cmd->r.x = x;
  cmd->r.y = y;
  cmd->r.width = width;
  cmd->r.height = height;

  vb.buf = cmd;
  vb.len = sizeof(*cmd);

  return virtio_gpu_send_cmd(vq, &vb, 1, 0, cmd);
}

/****************************************************************************
 * Name: virtio_gpu_flush_output
 ****************************************************************************/

static int virtio_gpu_flush_output(FAR struct fb_vtable_s *vtable,
                                   int resource_id)
{
  FAR struct virtio_gpu_priv_s *priv =
    (FAR struct virtio_gpu_priv_s *)vtable;
  int ret;

  ret = virtio_gpu_set_scanout(priv, 0, resource_id, priv->xres, priv->yres);
  if (ret < 0)
    {
      vrterr("virtio_gpu_set_scanout failed: %d", ret);
      return ret;
    }

  return virtio_gpu_flush_resource(priv, resource_id, 0, 0,
                                   priv->xres, priv->yres);
}

/****************************************************************************
 * Name: virtio_gpu_vsync_loop
 ****************************************************************************/

static void virtio_gpu_vsync_loop(FAR void *arg)
{
  FAR struct virtio_gpu_priv_s *priv = (FAR struct virtio_gpu_priv_s *)arg;
  union fb_paninfo_u info;

  fb_notify_vsync(&priv->vtable);

  if (fb_peek_paninfo(&priv->vtable, &info, FB_NO_OVERLAY) == OK)
    {
      virtio_gpu_flush_output(&priv->vtable,
                              info.planeinfo.yoffset == 0 ? 1 : 2);
      fb_remove_paninfo(&priv->vtable, FB_NO_OVERLAY);
    }
}

/****************************************************************************
 * Name: virtio_gpu_vsync_interrupt
 ****************************************************************************/

static void virtio_gpu_vsync_interrupt(FAR void *arg)
{
  FAR struct virtio_gpu_priv_s *priv = (FAR struct virtio_gpu_priv_s *)arg;
  virtio_gpu_vsync_loop(arg);
  work_queue(HPWORK, &priv->work, virtio_gpu_vsync_interrupt,
             arg, VIRTIO_GPU_VSYNC_PERIOD);
}

/****************************************************************************
 * Name: virtio_gpu_init_fbmem
 ****************************************************************************/

static int virtio_gpu_init_fbmem(FAR struct virtio_device *vdev,
                                 FAR struct virtio_gpu_priv_s *priv,
                                 int fb_count)
{
  struct virtio_gpu_mem_entry ent;
  size_t fblen;
  int ret;
  int i;

  priv->stride = priv->xres * VIRTIO_GPU_BPP >> 3;
  fblen = priv->stride * priv->yres;

  priv->fbmem = (FAR uint8_t *)virtio_zalloc_buf(vdev, fblen * fb_count, 16);
  if (priv->fbmem == NULL)
    {
      vrterr("ERROR: Failed to allocate frame buffer memory");
      return -ENOMEM;
    }

  for (i = 1; i <= fb_count; i++)
    {
      ret = virtio_gpu_create_2d(priv, i, priv->xres, priv->yres);
      if (ret < 0)
        {
          vrterr("virtio_gpu_create_2d error");
          goto error;
        }

      ent.addr = up_addrenv_va_to_pa(priv->fbmem + fblen * (i - 1));
      ent.length = fblen;

      ret = virtio_gpu_attach_backing(priv, i, &ent, 1);
      if (ret < 0)
        {
          vrterr("virtio_gpu_attach_backing error");
          goto error;
        }

      ret = virtio_gpu_transfer_to_host_2d(priv, i, 0, 0,
                                           priv->xres, priv->yres);
      if (ret < 0)
        {
          vrterr("virtio_gpu_transfer_to_host_2d failed: %d", ret);
          goto error;
        }
    }

  priv->fblen = fblen * fb_count;
  return ret;

error:
  virtio_free_buf(vdev, priv->fbmem);
  return ret;
}

/****************************************************************************
 * Name: virtio_gpu_probe
 ****************************************************************************/

static int virtio_gpu_probe(FAR struct virtio_device *vdev)
{
  FAR struct virtio_gpu_priv_s *priv;
  int disp;
  int ret;

  for (disp = 0; disp < VIRTIO_GPU_MAX_DISP; disp++)
    {
      if (g_virtio_gpu[disp] == NULL)
        {
          break;
        }
    }

  if (disp == VIRTIO_GPU_MAX_DISP)
    {
      return -EMFILE;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  ret = virtio_gpu_init(priv, vdev);
  if (ret < 0)
    {
      goto err_out;
    }

  ret = virtio_gpu_get_display_info(priv);
  if (ret < 0)
    {
      goto err_init;
    }

  /* Initialize the LCD-independent fields of the state structure */

  priv->vtable.getvideoinfo = virtio_gpu_getvideoinfo,
  priv->vtable.getplaneinfo = virtio_gpu_getplaneinfo,
#ifdef CONFIG_FB_UPDATE
  priv->vtable.updatearea   = virtio_gpu_updatearea,
#endif

  /* Allocate (and clear) the framebuffer */

  ret = virtio_gpu_init_fbmem(vdev, priv, VIRTIO_GPU_FB_NUMS);
  if (ret < 0)
    {
      vrterr("virtio_gpu_init_fbmem error");
      goto err_init;
    }

  ret = virtio_gpu_flush_output(&priv->vtable, 1);

  if (ret < 0)
    {
      vrterr("virtio_gpu_flush_output error");
      goto err_init_fb;
    }

  g_virtio_gpu[disp] = priv;
  priv->display = disp;

  ret = virtio_gpu_fb_register(disp);
  if (ret < 0)
    {
      vrterr("ERROR: Failed to initialize framebuffer driver, ret=%d",
             ret);
      g_virtio_gpu[disp] = NULL;
      goto err_init_fb;
    }

  work_queue(HPWORK, &priv->work, virtio_gpu_vsync_interrupt,
             priv, VIRTIO_GPU_VSYNC_PERIOD);

  return ret;

err_init_fb:
  virtio_free_buf(vdev, priv->fbmem);
err_init:
  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
err_out:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: virtio_gpu_remove
 ****************************************************************************/

static void virtio_gpu_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_gpu_priv_s *priv = vdev->priv;

  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
  g_virtio_gpu[priv->display] = NULL;
  virtio_free_buf(vdev, priv->fbmem);
  kmm_free(priv);
}

/****************************************************************************
 * Name: virtio_gpu_getvideoinfo
 ****************************************************************************/

static int virtio_gpu_getvideoinfo(FAR struct fb_vtable_s *vtable,
                                   FAR struct fb_videoinfo_s *vinfo)
{
  FAR struct virtio_gpu_priv_s *priv =
    (FAR struct virtio_gpu_priv_s *)vtable;

  vinfo->fmt = VIRTIO_GPU_FB_FMT;
  vinfo->nplanes = VIRTIO_GPU_MAX_PLANE;
  vinfo->xres = priv->xres;
  vinfo->yres = priv->yres;
  return OK;
}

/****************************************************************************
 * Name: virtio_gpu_getplaneinfo
 ****************************************************************************/

static int virtio_gpu_getplaneinfo(FAR struct fb_vtable_s *vtable,
                                   int planeno,
                                   FAR struct fb_planeinfo_s *pinfo)
{
  FAR struct virtio_gpu_priv_s *priv =
    (FAR struct virtio_gpu_priv_s *)vtable;

  if (planeno >= VIRTIO_GPU_MAX_PLANE)
    {
      vrterr("ERROR: plane number %d exceeds %d",
             planeno, VIRTIO_GPU_MAX_PLANE - 1);
      return -EINVAL;
    }

  memset(pinfo, 0, sizeof(*pinfo));
  pinfo->bpp = VIRTIO_GPU_BPP;
  pinfo->display = priv->display;
  pinfo->fblen = priv->fblen;
  pinfo->fbmem = priv->fbmem;
  pinfo->stride = priv->stride;
  pinfo->xres_virtual = priv->xres;
  pinfo->yres_virtual = priv->yres * VIRTIO_GPU_FB_NUMS;
  return OK;
}

#ifdef CONFIG_FB_UPDATE
/****************************************************************************
 * Name: virtio_gpu_updatearea
 ****************************************************************************/

static int virtio_gpu_updatearea(FAR struct fb_vtable_s *vtable,
                                 FAR const struct fb_area_s *area)
{
  FAR struct virtio_gpu_priv_s *priv =
    (FAR struct virtio_gpu_priv_s *)vtable;
  int resource_id = 1;
  int ret = OK;

  vrtinfo("update disp %d:(%d %d)[%d %d]", priv->display,
          area->x, area->y, area->w, area->h);

  if (area->y >= priv->yres)
    {
      resource_id = 2;
    }

  ret = virtio_gpu_transfer_to_host_2d(priv, resource_id, 0, 0,
                                       priv->xres, priv->yres);
  if (ret < 0)
    {
      vrterr("virtio_gpu_transfer_to_host_2d failed: %d", ret);
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_register_gpu_driver
 ****************************************************************************/

int virtio_register_gpu_driver(void)
{
  return virtio_register_driver(&g_virtio_gpu_driver);
}

/****************************************************************************
 * Name: virtio_gpu_fb_register
 *
 * Description:
 *   Initialize the framebuffer video hardware associated with the display.
 *
 * Input Parameters:
 *   display - The display number for the case of boards supporting multiple
 *             displays or for hardware that supports multiple
 *             layers (each layer is consider a display).  Typically zero.
 *
 * Returned Value:
 *   Zero (OK) is returned success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int virtio_gpu_fb_register(int display)
{
  FAR struct fb_vtable_s *vtable;

  if (display < 0 || display >= VIRTIO_GPU_MAX_DISP ||
     !g_virtio_gpu[display])
    {
      vrterr("ERROR: display number %d is out of range [%d, %d]",
             display, 0, VIRTIO_GPU_MAX_DISP - 1);
      return -EINVAL;
    }

  vtable =  &g_virtio_gpu[display]->vtable;

  if (vtable == NULL)
    {
      vrterr("ERROR: get vtable failed\n");
      return -EINVAL;
    }

  return fb_register_device(display, 0, vtable);
}
