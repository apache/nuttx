/****************************************************************************
 * drivers/video/goldfish_gpu_fb.c
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
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <nuttx/kthread.h>
#include <nuttx/video/fb.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define EGL_RGB                  0x1907
#define EGL_RGBA                 0x1908
#define EGL_BGRA                 0x80e1
#define EGL_RGB565               0x8d62

#define EGL_UNSIGNED_BYTE        0x1401
#define EGL_UNSIGNED_SHORT_5_6_5 0x8363

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum
{
  EGL_FB_WIDTH  = 1,
  EGL_FB_HEIGHT = 2,
};

enum
{
  OP_GET_FB_PARAM        = 10007,
  OP_FB_POST             = 10018,
  OP_CREATE_COLOR_BUFFER = 10012,
  OP_UPDATE_COLOR_BUFFER = 10024,
};

struct goldfish_gpu_fb_s
{
  struct fb_vtable_s vtable;
  struct fb_planeinfo_s planeinfo;
  struct fb_videoinfo_s videoinfo;
  struct file pipe;
  int colorbuffer;
  int colorformat;
  int colortype;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int goldfish_gpu_fb_getvideoinfo(FAR struct fb_vtable_s *vtable,
                                        FAR struct fb_videoinfo_s *vinfo);
static int goldfish_gpu_fb_getplaneinfo(FAR struct fb_vtable_s *vtable,
                                        int planeno,
                                        FAR struct fb_planeinfo_s *pinfo);
static int goldfish_gpu_fb_vsync_thread(int argc, FAR char** argv);
static int goldfish_gpu_fb_init_pipe(FAR struct file *filep,
                                     FAR const char *ns,
                                     FAR const char *pipe_name,
                                     int flags);
static int goldfish_gpu_fb_read_pipe(FAR struct file *pipe,
                                     FAR void *buffer,
                                     size_t size);
static int goldfish_gpu_fb_write_pipe(FAR struct file *pipe,
                                      FAR const void *buffer,
                                      size_t size);
static int goldfish_gpu_fb_get_param(FAR struct file *pipe, int type);
static int goldfish_gpu_fb_create_colorbuffer(FAR struct file *pipe,
                                              int width,
                                              int height,
                                              int format);
static int goldfish_gpu_fb_update_colorbuffer(FAR struct file *pipe,
                                              int colorbuffer,
                                              int x,
                                              int y,
                                              int width,
                                              int height,
                                              int format,
                                              int type,
                                              FAR void *pixel,
                                              size_t size);
static int goldfish_gpu_fb_post(FAR struct file *pipe, int colorbuffer);
static int goldfish_gpu_fb_commit(FAR struct goldfish_gpu_fb_s *fb,
                                  FAR void * buf);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldfish_gpu_fb_read_pipe
 ****************************************************************************/

static int goldfish_gpu_fb_read_pipe(FAR struct file *pipe,
                                     FAR void *buffer,
                                     size_t size)
{
  FAR char *p = (FAR char *)buffer;

  while (size > 0)
    {
      ssize_t n = file_read(pipe, p, size);
      if (n < 0)
        {
          return n;
        }

      p += n;
      size -= n;
    }

  return 0;
}

/****************************************************************************
 * Name: goldfish_gpu_fb_write_pipe
 ****************************************************************************/

static int goldfish_gpu_fb_write_pipe(FAR struct file *pipe,
                                      FAR const void *buffer,
                                      size_t size)
{
  FAR const char *p = (FAR const char *)buffer;

  while (size > 0)
    {
      ssize_t n = file_write(pipe, p, size);
      if (n < 0)
        {
          return n;
        }

      p += n;
      size -= n;
    }

  return 0;
}

/****************************************************************************
 * Name: goldfish_gpu_fb_init_pipe
 ****************************************************************************/

static int goldfish_gpu_fb_init_pipe(FAR struct file *filep,
                                     FAR const char *ns,
                                     FAR const char *pipe_name,
                                     int flags)
{
  int zero_flag = 0;
  char buf[256];
  int buf_len;
  int ret;

  ret = file_open(filep, "/dev/goldfish_pipe", flags);
  if (ret < 0)
    {
      gerr("Could not open /dev/goldfish_pipe: %s", strerror(-ret));
      return ret;
    }

  if (ns)
    {
      buf_len = snprintf(buf, sizeof(buf), "pipe:%s:%s", ns, pipe_name);
    }
  else
    {
      buf_len = snprintf(buf, sizeof(buf), "pipe:%s", pipe_name);
    }

  ret = goldfish_gpu_fb_write_pipe(filep, buf, buf_len + 1);
  if (ret < 0)
    {
      gerr("Could not connect to the '%s' error: %s",
            buf, strerror(-ret));
      file_close(filep);
      return ret;
    }

  ret = goldfish_gpu_fb_write_pipe(filep, &zero_flag, sizeof(zero_flag));
  if (ret < 0)
    {
      gerr("Could not write zero flag to the '%s' error: %s",
            buf, strerror(-ret));
      file_close(filep);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: goldfish_gpu_fb_get_param
 ****************************************************************************/

static int goldfish_gpu_fb_get_param(FAR struct file *pipe, int type)
{
  int cmdbuf[3];
  int ret;
  int res;

  cmdbuf[0] = OP_GET_FB_PARAM;
  cmdbuf[1] = sizeof(cmdbuf);
  cmdbuf[2] = type;

  ret = goldfish_gpu_fb_write_pipe(pipe, cmdbuf, sizeof(cmdbuf));
  if (ret < 0)
    {
      gerr("Write fb param cmd error: %s", strerror(-ret));
      return ret;
    }

  ret = goldfish_gpu_fb_read_pipe(pipe, &res, sizeof(res));
  if (ret < 0)
    {
      gerr("Read fb param result error: %s", strerror(-ret));
      return ret;
    }

  return res;
}

/****************************************************************************
 * Name: goldfish_gpu_fb_post
 ****************************************************************************/

static int goldfish_gpu_fb_post(FAR struct file *pipe, int colorbuffer)
{
  int cmdbuf[3];
  int ret;

  cmdbuf[0] = OP_FB_POST;
  cmdbuf[1] = sizeof(cmdbuf);
  cmdbuf[2] = colorbuffer;

  ret = goldfish_gpu_fb_write_pipe(pipe, cmdbuf, sizeof(cmdbuf));
  if (ret < 0)
    {
      gerr("Write fb post cmd error: %s", strerror(-ret));
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: goldfish_gpu_fb_create_colorbuffer
 ****************************************************************************/

static int goldfish_gpu_fb_create_colorbuffer(FAR struct file *pipe,
                                              int width,
                                              int height,
                                              int format)
{
  int cmdbuf[5];
  int ret;
  int res;

  cmdbuf[0] = OP_CREATE_COLOR_BUFFER;
  cmdbuf[1] = sizeof(cmdbuf);
  cmdbuf[2] = width;
  cmdbuf[3] = height;
  cmdbuf[4] = format;

  ret = goldfish_gpu_fb_write_pipe(pipe, cmdbuf, sizeof(cmdbuf));
  if (ret < 0)
    {
      gerr("Write create colorbuffer cmd error: %s", strerror(-ret));
      return ret;
    }

  ret = goldfish_gpu_fb_read_pipe(pipe, &res, sizeof(res));
  if (ret < 0)
    {
      gerr("Read create colorbuffer result error: %s", strerror(-ret));
      return ret;
    }

  return res;
}

/****************************************************************************
 * Name: goldfish_gpu_fb_update_colorbuffer
 ****************************************************************************/

static int goldfish_gpu_fb_update_colorbuffer(FAR struct file *pipe,
                                              int colorbuffer,
                                              int x,
                                              int y,
                                              int width,
                                              int height,
                                              int format,
                                              int type,
                                              FAR void *pixel,
                                              size_t size)
{
  int cmdbuf[10];
  int ret;
  int res;

  cmdbuf[0] = OP_UPDATE_COLOR_BUFFER;
  cmdbuf[1] = sizeof(cmdbuf) + size;
  cmdbuf[2] = colorbuffer;
  cmdbuf[3] = x;
  cmdbuf[4] = y;
  cmdbuf[5] = width;
  cmdbuf[6] = height;
  cmdbuf[7] = format;
  cmdbuf[8] = type;
  cmdbuf[9] = size;

  ret = goldfish_gpu_fb_write_pipe(pipe, cmdbuf, sizeof(cmdbuf));
  if (ret < 0)
    {
      gerr("Write update colorbuffer cmd error: %s", strerror(-ret));
      return ret;
    }

  ret = goldfish_gpu_fb_write_pipe(pipe, pixel, size);
  if (ret < 0)
    {
      gerr("Write update colorbuffer data error: %s", strerror(-ret));
      return ret;
    }

  ret = goldfish_gpu_fb_read_pipe(pipe, &res, sizeof(res));
  if (ret < 0)
    {
      gerr("Read update colorbuffer result error: %s", strerror(-ret));
      return ret;
    }

  return res;
}

/****************************************************************************
 * Name: goldfish_gpu_fb_commit
 ****************************************************************************/

static int goldfish_gpu_fb_commit(FAR struct goldfish_gpu_fb_s *fb,
                                  FAR void * buf)
{
  int ret;

  ret = goldfish_gpu_fb_update_colorbuffer(&fb->pipe, fb->colorbuffer, 0, 0,
                                           fb->videoinfo.xres,
                                           fb->videoinfo.yres,
                                           fb->colorformat,
                                           fb->colortype,
                                           buf,
                                           fb->planeinfo.stride
                                           * fb->videoinfo.yres);
  if (ret < 0)
    {
      gerr("Failed to update colorbuffer: %d\n", ret);
      return ret;
    }

  ret = goldfish_gpu_fb_post(&fb->pipe, fb->colorbuffer);
  if (ret < 0)
    {
      gerr("Failed to post colorbuffer: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: goldfish_gpu_fb_getvideoinfo
 ****************************************************************************/

static int goldfish_gpu_fb_getvideoinfo(FAR struct fb_vtable_s *vtable,
                                        FAR struct fb_videoinfo_s *vinfo)
{
  FAR struct goldfish_gpu_fb_s *fb = (FAR struct goldfish_gpu_fb_s *)vtable;

  ginfo("vtable=%p vinfo=%p\n", vtable, vinfo);
  if (fb && vinfo)
    {
      memcpy(vinfo, &fb->videoinfo, sizeof(struct fb_videoinfo_s));
      return OK;
    }

  gerr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: goldfish_gpu_fb_getplaneinfo
 ****************************************************************************/

static int goldfish_gpu_fb_getplaneinfo(FAR struct fb_vtable_s *vtable,
                                        int planeno,
                                        FAR struct fb_planeinfo_s *pinfo)
{
  FAR struct goldfish_gpu_fb_s *fb = (FAR struct goldfish_gpu_fb_s *)vtable;

  ginfo("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);
  if (fb && planeno == 0 && pinfo)
    {
      memcpy(pinfo, &fb->planeinfo, sizeof(struct fb_planeinfo_s));
      return OK;
    }

  gerr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: goldfish_gpu_fb_vsync_thread
 ****************************************************************************/

static int goldfish_gpu_fb_vsync_thread(int argc, FAR char** argv)
{
  FAR struct goldfish_gpu_fb_s *fb = (FAR struct goldfish_gpu_fb_s *)
                                ((uintptr_t)strtoul(argv[1], NULL, 0));
  union fb_paninfo_u info;
  clock_t last = 0;

  while (1)
    {
      clock_t now = clock_systime_ticks();

      if (now - last >= MSEC2TICK(16))
        {
          last = now;
          fb_notify_vsync(&fb->vtable);

          if (fb_peek_paninfo(&fb->vtable, &info, FB_NO_OVERLAY) == OK)
            {
              FAR void *buf = fb->planeinfo.fbmem +
                              fb->planeinfo.stride *
                              info.planeinfo.yoffset;

              goldfish_gpu_fb_commit(fb, buf);
              fb_remove_paninfo(&fb->vtable, FB_NO_OVERLAY);
            }
        }

      /* Sleep 8ms, let the idle run */

      usleep(8000);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldfish_gpu_fb_register
 ****************************************************************************/

int goldfish_gpu_fb_register(int display)
{
  FAR struct goldfish_gpu_fb_s *fb;
  FAR char *argv[2];
  char arg1[32];
  int ret = OK;
  int pid;

  fb = kmm_zalloc(sizeof(*fb));
  if (fb == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize the pipe */

  ret = goldfish_gpu_fb_init_pipe(&fb->pipe, NULL,
                                  "opengles", O_RDWR | O_CLOEXEC);
  if (ret < 0)
    {
      gerr("Failed to initialize pipe: %d\n", ret);
      goto err_fb_init_failed;
    }

  /* Get the framebuffer parameters */

  ret = goldfish_gpu_fb_get_param(&fb->pipe, EGL_FB_WIDTH);
  if (ret < 0)
    {
      gerr("Failed to get fb width: %d\n", ret);
      goto err_fb_get_param_failed;
    }

  fb->videoinfo.xres = ret;

  ret = goldfish_gpu_fb_get_param(&fb->pipe, EGL_FB_HEIGHT);
  if (ret < 0)
    {
      gerr("Failed to get fb height: %d\n", ret);
      goto err_fb_get_param_failed;
    }

  fb->videoinfo.yres = ret;

#ifdef CONFIG_GOLDFISH_GPU_FB_BGRA8888
  fb->colorformat   = EGL_BGRA;
  fb->colortype     = EGL_UNSIGNED_BYTE;
  fb->videoinfo.fmt = FB_FMT_RGB32;
  fb->planeinfo.bpp = 32;
#else
  fb->colorformat   = EGL_RGB565;
  fb->colortype     = EGL_UNSIGNED_SHORT_5_6_5;
  fb->videoinfo.fmt = FB_FMT_RGB16_565;
  fb->planeinfo.bpp = 16;
#endif
  fb->videoinfo.nplanes = 1;
  fb->planeinfo.stride = fb->videoinfo.xres * (fb->planeinfo.bpp >> 3);
  fb->planeinfo.yres_virtual = fb->videoinfo.yres * 2;
  fb->planeinfo.xres_virtual = fb->videoinfo.xres;

  /* Create the colorbuffer */

  ret = goldfish_gpu_fb_create_colorbuffer(&fb->pipe,
                                           fb->videoinfo.xres,
                                           fb->videoinfo.yres,
                                           fb->colorformat);
  if (ret < 0)
    {
      gerr("Failed to create colorbuffer: %d\n", ret);
      goto err_fb_get_param_failed;
    }

  fb->colorbuffer = ret;

  fb->planeinfo.fblen = fb->planeinfo.stride * fb->planeinfo.yres_virtual;
  fb->planeinfo.fbmem = kmm_zalloc(fb->planeinfo.fblen);

  if (fb->planeinfo.fbmem == NULL)
    {
      gerr("ERROR: Failed to allocate framebuffer memory: %zu KB\n",
           fb->planeinfo.fblen / 1024);
      ret = -ENOMEM;
      goto err_fb_get_param_failed;
    }

  fb->vtable.getplaneinfo = goldfish_gpu_fb_getplaneinfo;
  fb->vtable.getvideoinfo = goldfish_gpu_fb_getvideoinfo;

  /* Create the vsync thread */

  snprintf(arg1, 32, "%p", fb);
  argv[0] = arg1;
  argv[1] = NULL;
  pid = kthread_create("goldfish_gpu_fb_thread",
                       CONFIG_GOLDFISH_GPU_FB_PRIORITY,
                       CONFIG_DEFAULT_TASK_STACKSIZE,
                       goldfish_gpu_fb_vsync_thread, argv);
  if (pid < 0)
    {
      gerr("Failed to create vsync thread: %d\n", pid);
      goto err_fb_thrad_create_failed;
    }

  /* Register the framebuffer */

  ret = fb_register_device(display, 0, (FAR struct fb_vtable_s *)fb);
  if (ret < 0)
    {
      goto err_fb_register_failed;
    }

  return OK;

err_fb_register_failed:
  kthread_delete(pid);
err_fb_thrad_create_failed:
  kmm_free(fb->planeinfo.fbmem);
err_fb_get_param_failed:
  file_close(&fb->pipe);
err_fb_init_failed:
  kmm_free(fb);
  return ret;
}
