/****************************************************************************
 * drivers/video/goldfish_fb.c
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
#include <nuttx/config.h>
#include <nuttx/video/fb.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/circbuf.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#ifndef putreg32
#define putreg32(v, x) (*(volatile uint32_t *)(x) = (v))
#endif

#ifndef getreg32
#define getreg32(x) (*(uint32_t *)(x))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum
{
  GOLDFISH_FB_GET_WIDTH = 0x00,
  GOLDFISH_FB_GET_HEIGHT = 0x04,
  GOLDFISH_FB_INT_STATUS = 0x08,
  GOLDFISH_FB_INT_ENABLE = 0x0c,
  GOLDFISH_FB_SET_BASE = 0x10,
  GOLDFISH_FB_SET_ROTATION = 0x14,
  GOLDFISH_FB_SET_BLANK = 0x18,
  GOLDFISH_FB_GET_PHYS_WIDTH = 0x1c,
  GOLDFISH_FB_GET_PHYS_HEIGHT = 0x20,
  GOLDFISH_FB_GET_FORMAT = 0x24,
  GOLDFISH_FB_INT_VSYNC = 1U << 0,
  GOLDFISH_FB_INT_UPDATE_DONE = 1U << 1,
  GOLDFISH_FB_FORMAT_BRGA_8888 = 1,
  GOLDFISH_FB_FORMAT_RGBX_8888 = 2,
  GOLDFISH_FB_FORMAT_RGB_888 = 3,
  GOLDFISH_FB_FORMAT_RGB_565 = 4,
  GOLDFISH_FB_FORMAT_BGRA_8888 = 5,
  GOLDFISH_FB_FORMAT_RGBA_5551 = 6,
  GOLDFISH_FB_FORMAT_RGBA_4444 = 8
};

struct goldfish_fb_format_s
{
  uint8_t fmt;
  uint8_t bpp;
};

struct goldfish_fb_s
{
  struct fb_vtable_s vtable;
  struct fb_planeinfo_s planeinfo;
  struct fb_videoinfo_s videoinfo;
  FAR void *base;
  int irq;
  struct circbuf_s vsync;   /* Vsync event queued */
#ifdef CONFIG_GOLDFISH_FB_VIDEO_MODE
  bool busy; /* Only used in the video mode */
  uintptr_t cur_buf;
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct goldfish_fb_s *g_goldfish_fb;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int goldfish_fb_pan_display(FAR struct fb_vtable_s *vtable,
                                   FAR struct fb_planeinfo_s *pinfo);
static int goldfish_getvideoinfo(FAR struct fb_vtable_s *vtable,
                                 FAR struct fb_videoinfo_s *vinfo);
static int goldfish_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                                 FAR struct fb_planeinfo_s *pinfo);
static int goldfish_fb_interrupt(int irq, FAR void *dev_id, FAR void *arg);
static void goldfish_fb_vsync_irq(FAR struct goldfish_fb_s *fb);
static void goldfish_fb_framedone_irq(FAR struct goldfish_fb_s *fb);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldfish_fb_vsync_irq
 ****************************************************************************/

#ifdef CONFIG_GOLDFISH_FB_VIDEO_MODE
static void goldfish_fb_vsync_irq(FAR struct goldfish_fb_s *fb)
{
  struct fb_planeinfo_s pinfo;

  /* Attempte to retrieve a frame from the vsync queue */

  ssize_t ret = circbuf_read(&fb->vsync, &pinfo,
                             sizeof(struct fb_planeinfo_s));
  DEBUGASSERT(ret <= 0 || ret == sizeof(struct fb_planeinfo_s));

  fb->busy = true;
  if (ret > 0)
    {
      fb->cur_buf = (uintptr_t)((uint8_t *)fb->planeinfo.fbmem +
                                           fb->planeinfo.stride *
                                           pinfo.yoffset);
    }

  if (fb->cur_buf)
    {
      /* Send buffer addr to GOLDFISH */

      putreg32(fb->cur_buf, fb->base + GOLDFISH_FB_SET_BASE);
    }
}
#else
static void goldfish_fb_vsync_irq(FAR struct goldfish_fb_s *fb)
{
  struct fb_planeinfo_s pinfo;

  /* Attempte to retrieve a frame from the vsync queue */

  ssize_t ret = circbuf_read(&fb->vsync, &pinfo,
                             sizeof(struct fb_planeinfo_s));
  DEBUGASSERT(ret <= 0 || ret == sizeof(struct fb_planeinfo_s));

  if (ret > 0)
    {
      uintptr_t buf = (uintptr_t)((uint8_t *)fb->planeinfo.fbmem +
                                             fb->planeinfo.stride *
                                             pinfo.yoffset);

      /* Send buffer addr to GOLDFISH */

      putreg32(buf, fb->base + GOLDFISH_FB_SET_BASE);
    }
}
#endif

/****************************************************************************
 * Name: goldfish_fb_framedone_irq
 ****************************************************************************/

#ifdef CONFIG_GOLDFISH_FB_VIDEO_MODE
static void goldfish_fb_framedone_irq(FAR struct goldfish_fb_s *fb)
{
  fb->busy = false;

  if (fb->cur_buf && !circbuf_is_empty(&fb->vsync))
    {
      /* Clear the current frame buffer */

      fb->cur_buf = 0;

      /* After the sending is completed, notify the upper
       * layer that the framebuffer can be written.
       */

      fb_pollnotify(&fb->vtable);
    }
}
#else
static void goldfish_fb_framedone_irq(FAR struct goldfish_fb_s *fb)
{
  /* After the sending is completed, notify the upper
   * layer that the framebuffer can be written.
   */

  fb_pollnotify(&fb->vtable);
}
#endif

/****************************************************************************
 * Name: goldfish_fb_interrupt
 ****************************************************************************/

static int goldfish_fb_interrupt(int irq, FAR void *dev_id, FAR void *arg)
{
  FAR struct goldfish_fb_s *fb = arg;
  irqstate_t flags;
  uint32_t status;

  flags = enter_critical_section();
  status = getreg32(fb->base + GOLDFISH_FB_INT_STATUS);
  if (status & GOLDFISH_FB_INT_VSYNC)
    {
      goldfish_fb_vsync_irq(fb);
    }

  else if (status & GOLDFISH_FB_INT_UPDATE_DONE)
    {
      goldfish_fb_framedone_irq(fb);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: goldfish_fb_pan_display
 ****************************************************************************/

static int goldfish_fb_pan_display(FAR struct fb_vtable_s *vtable,
                                   FAR struct fb_planeinfo_s *pinfo)
{
  struct goldfish_fb_s *fb = (FAR struct goldfish_fb_s *)vtable;
  irqstate_t flags;
  ssize_t ret;

  /** Disable the interrupt when writing to the queue to
   *  prevent it from being modified by the interrupted
   * thread during the writing process.
   */

  flags = enter_critical_section();

  /* Write the planeinfo information submitted
   * by the renderer to the queue
   */

  ret = circbuf_write(&fb->vsync, pinfo,
                      sizeof(struct fb_planeinfo_s));
  DEBUGASSERT(ret == sizeof(struct fb_planeinfo_s));

#ifdef CONFIG_GOLDFISH_FB_VIDEO_MODE
  if (fb->cur_buf && !fb->busy)
    {
      /* Clear the current frame buffer if not busy in transfering */

      fb->cur_buf = 0;

      /* Notify the upper layer that the framebuffer can be written */

      fb_pollnotify(&fb->vtable);
    }
#endif

  /* Re-enable interrupts */

  leave_critical_section(flags);
  return ret < 0 ? ret : 0;
}

/****************************************************************************
 * Name: goldfish_getvideoinfo
 ****************************************************************************/

static int goldfish_getvideoinfo(FAR struct fb_vtable_s *vtable,
                                 FAR struct fb_videoinfo_s *vinfo)
{
  FAR struct goldfish_fb_s *fb = (FAR struct goldfish_fb_s *)vtable;

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
 * Name: goldfish_getplaneinfo
 ****************************************************************************/

static int goldfish_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                                 FAR struct fb_planeinfo_s *pinfo)
{
  FAR struct goldfish_fb_s *fb = (FAR struct goldfish_fb_s *)vtable;

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 ****************************************************************************/

int up_fbinitialize(int display)
{
  FAR struct goldfish_fb_s *fb;
  uint32_t fmt;
  int ret = OK;

  const struct goldfish_fb_format_s format_map[] =
  {
    [GOLDFISH_FB_FORMAT_BRGA_8888] =
    {FB_FMT_RGBA32, 32},
    [GOLDFISH_FB_FORMAT_RGBX_8888] =
    {FB_FMT_RGB32, 32},
    [GOLDFISH_FB_FORMAT_RGB_888] =
    {FB_FMT_RGB24, 24},
    [GOLDFISH_FB_FORMAT_RGB_565] =
    {FB_FMT_RGB16_565, 16},
    [GOLDFISH_FB_FORMAT_BGRA_8888] =
    {FB_FMT_RGBA32, 32},
    [GOLDFISH_FB_FORMAT_RGBA_5551] =
    {FB_FMT_RGB16_555, 16},
    [GOLDFISH_FB_FORMAT_RGBA_4444] =
    {FB_FMT_RGBA16, 16},
  };

  fb = kmm_zalloc(sizeof(*fb));
  if (fb == NULL)
    {
      return -ENOMEM;
    }

  fb->base = (FAR void *)CONFIG_GOLDFISH_FB_BASE;
  fb->irq = CONFIG_GOLDFISH_FB_IRQ;

  /* Initialize vsync queue */

  ret = circbuf_init(&fb->vsync, NULL,
                     CONFIG_GOLDFISH_FB_FRAME_NBUFFER *
                     sizeof(struct fb_planeinfo_s));
  if (ret < 0)
    {
      goto err_circbuf_alloc_failed;
    }

  fmt = getreg32(fb->base + GOLDFISH_FB_GET_FORMAT);

  fb->videoinfo.xres = getreg32(fb->base + GOLDFISH_FB_GET_WIDTH);
  fb->videoinfo.yres = getreg32(fb->base + GOLDFISH_FB_GET_HEIGHT);
  fb->videoinfo.nplanes = 1;
  fb->videoinfo.fmt = format_map[fmt].fmt;

  fb->planeinfo.bpp = format_map[fmt].bpp;
  fb->planeinfo.stride = fb->videoinfo.xres * (fb->planeinfo.bpp >> 3);
  fb->planeinfo.yres_virtual = fb->videoinfo.yres *
                               CONFIG_GOLDFISH_FB_FRAME_NBUFFER;
  fb->planeinfo.xres_virtual = fb->videoinfo.xres;

  fb->planeinfo.fblen = fb->planeinfo.stride * fb->planeinfo.yres_virtual;
  fb->planeinfo.fbmem = kmm_zalloc(fb->planeinfo.fblen);
  if (fb->planeinfo.fbmem == NULL)
    {
      gerr("ERROR: Failed to allocate framebuffer memory: %zu KB\n",
           fb->planeinfo.fblen / 1024);
      ret = -ENOMEM;
      goto err_fbmem_alloc_failed;
    }

  fb->vtable.pandisplay = goldfish_fb_pan_display;
  fb->vtable.getplaneinfo = goldfish_getplaneinfo;
  fb->vtable.getvideoinfo = goldfish_getvideoinfo;

  ret = irq_attach(fb->irq, goldfish_fb_interrupt, fb);
  if (ret < 0)
    {
      goto err_irq_attach_failed;
    }

  up_enable_irq(fb->irq);
  putreg32(GOLDFISH_FB_INT_VSYNC | GOLDFISH_FB_INT_UPDATE_DONE,
           fb->base + GOLDFISH_FB_INT_ENABLE);

  /* Updates base */

  putreg32((uintptr_t)fb->planeinfo.fbmem,
           fb->base + GOLDFISH_FB_SET_BASE);

  g_goldfish_fb = fb;
  return OK;

err_irq_attach_failed:
  kmm_free(fb->planeinfo.fbmem);
err_fbmem_alloc_failed:
  circbuf_uninit(&fb->vsync);
err_circbuf_alloc_failed:
  kmm_free(fb);
  return ret;
}

/****************************************************************************
 * Name: up_fbgetvplane
 ****************************************************************************/

FAR struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  return vplane || display ? NULL : &(g_goldfish_fb->vtable);
}

/****************************************************************************
 * Name: up_fbuninitialize
 ****************************************************************************/

void up_fbuninitialize(int display)
{
  if (display == 0)
    {
      FAR struct goldfish_fb_s *fb = g_goldfish_fb;

      irq_detach(fb->irq);
      circbuf_uninit(&fb->vsync);
      kmm_free(fb->planeinfo.fbmem);
      kmm_free(fb);
      g_goldfish_fb = NULL;
    }
}
