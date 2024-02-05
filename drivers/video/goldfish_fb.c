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
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct goldfish_fb_s *g_goldfish_fb;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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

static void goldfish_fb_vsync_irq(FAR struct goldfish_fb_s *fb)
{
  union fb_paninfo_u info;
#ifdef CONFIG_GOLDFISH_FB_VIDEO_MODE
  int count;

  count = fb_paninfo_count(&fb->vtable, FB_NO_OVERLAY);
  if (count <= 0)
    {
      return;
    }

  if (count > 1)
    {
      fb_remove_paninfo(&fb->vtable, FB_NO_OVERLAY);
    }
#endif

  if (fb_peek_paninfo(&fb->vtable, &info, FB_NO_OVERLAY) == OK)
    {
      uintptr_t buf = (uintptr_t)(fb->planeinfo.fbmem +
                                  fb->planeinfo.stride *
                                  info.planeinfo.yoffset);

      /* Send buffer addr to GOLDFISH */

      putreg32(buf, fb->base + GOLDFISH_FB_SET_BASE);
    }
}

/****************************************************************************
 * Name: goldfish_fb_framedone_irq
 ****************************************************************************/

static void goldfish_fb_framedone_irq(FAR struct goldfish_fb_s *fb)
{
#ifndef CONFIG_GOLDFISH_FB_VIDEO_MODE
  /* After the sending is completed, remove it from the panbuf queue.
   */

  fb_remove_paninfo(&fb->vtable, FB_NO_OVERLAY);
#endif
}

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
 * Name: goldfish_fb_register
 ****************************************************************************/

int goldfish_fb_register(int display, FAR void *regs, int irq)
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

  fb->base = regs;
  fb->irq = irq;

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

  ret = fb_register_device(display, 0, (FAR struct fb_vtable_s *)fb);
  if (ret < 0)
    {
      goto err_fb_register_failed;
    }

  g_goldfish_fb = fb;
  return OK;

err_fb_register_failed:
  irq_detach(fb->irq);
err_irq_attach_failed:
  kmm_free(fb->planeinfo.fbmem);
err_fbmem_alloc_failed:
  kmm_free(fb);
  return ret;
}
