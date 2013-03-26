/****************************************************************************
 * arch/arm/src//lpc17xx/lpc17_lcd.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fb.h>
#include "lpc17_internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Framebuffer characteristics in bytes */

#define FB_WIDTH ((CONFIG_LPC17_LCD_HWIDTH * CONFIG_LPC17_LCD_BPP + 7) / 8)
#define FB_SIZE  (FB_WIDTH * CONFIG_LPC17_LCD_VHEIGHT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Get information about the video controller configuration and the
 * configuration of each color plane.
 */

static int lpc17_getvideoinfo(FAR struct fb_vtable_s *vtable,
             FAR struct fb_videoinfo_s *vinfo);
static int lpc17_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
             FAR struct fb_planeinfo_s *pinfo);

/* The following is provided only if the video hardware supports RGB color
 * mapping
 */

#ifdef CONFIG_FB_CMAP
static int lpc17_getcmap(FAR struct fb_vtable_s *vtable,
             FAR struct fb_cmap_s *cmap);
static int lpc17_putcmap(FAR struct fb_vtable_s *vtable,
             FAR const struct fb_cmap_s *cmap);
#endif

/* The following is provided only if the video hardware supports a hardware
 * cursor
 */

#ifdef CONFIG_FB_HWCURSOR
static int lpc17_getcursor(FAR struct fb_vtable_s *vtable,
             FAR struct fb_cursorattrib_s *attrib);
static int lpc17_setcursor(FAR struct fb_vtable_s *vtable,
             FAR struct fb_setcursor_s *setttings);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure describes the simulated video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt      = FB_FMT,
  .xres     = CONFIG_LPC17_LCD_HWIDTH,
  .yres     = CONFIG_LPC17_LCD_VHEIGHT,
  .nplanes  = 1,
};

/* This structure describes the single, simulated color plane */

static const struct fb_planeinfo_s g_planeinfo =
{
  .fbmem    = (FAR void *)LCD_VRAM_BASE,
  .fblen    = FB_SIZE,
  .stride   = FB_WIDTH,
  .bpp      = CONFIG_LPC17_LCD_BPP,
};

/* Current cursor position */

#ifdef CONFIG_FB_HWCURSOR
static struct fb_cursorpos_s g_cpos;

/* Current cursor size */

#ifdef CONFIG_FB_HWCURSORSIZE
static struct fb_cursorsize_s g_csize;
#endif
#endif

/* The framebuffer object -- There is no private state information in this simple
 * framebuffer simulation.
 */

struct fb_vtable_s g_fbobject =
{
  .getvideoinfo  = lpc17_getvideoinfo,
  .getplaneinfo  = lpc17_getplaneinfo,
#ifdef CONFIG_FB_CMAP
  .getcmap       = lpc17_getcmap,
  .putcmap       = lpc17_putcmap,
#endif
#ifdef CONFIG_FB_HWCURSOR
  .getcursor     = lpc17_getcursor,
  .setcursor     = lpc17_setcursor,
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_getvideoinfo
 ****************************************************************************/

static int lpc17_getvideoinfo(FAR struct fb_vtable_s *vtable,
                              FAR struct fb_videoinfo_s *vinfo)
{
  dbg("vtable=%p vinfo=%p\n", vtable, vinfo);
  if (vtable && vinfo)
    {
      memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
      return OK;
    }

  dbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: lpc17_getplaneinfo
 ****************************************************************************/

static int lpc17_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                              FAR struct fb_planeinfo_s *pinfo)
{
  dbg("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);
  if (vtable && planeno == 0 && pinfo)
    {
      memcpy(pinfo, &g_planeinfo, sizeof(struct fb_planeinfo_s));
      return OK;
    }

  dbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: lpc17_getcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int lpc17_getcmap(FAR struct fb_vtable_s *vtable, FAR struct fb_cmap_s *cmap)
{
  int len;
  int i;

  dbg("vtable=%p cmap=%p len=%d\n", vtable, cmap, cmap->len);
  if (vtable && cmap)
    {
      for (i = cmap->first, len = 0; i < 256 && len < cmap->len; i++, len++)
        {
          cmap->red[i]    = i;
          cmap->green[i]  = i;
          cmap->blue[i]   = i;
#ifdef CONFIG_FB_TRANSPARENCY
          cmap->transp[i] = i;
#endif
        }

      cmap->len = len;
      return OK;
    }

  dbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: lpc17_putcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int lpc17_putcmap(FAR struct fb_vtable_s *vtable, FAR const struct fb_cmap_s *cmap)
{
  dbg("vtable=%p cmap=%p len=%d\n", vtable, cmap, cmap->len);
  if (vtable && cmap)
    {
      return OK;
    }

  dbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: lpc17_getcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int lpc17_getcursor(FAR struct fb_vtable_s *vtable,
                        FAR struct fb_cursorattrib_s *attrib)
{
  dbg("vtable=%p attrib=%p\n", vtable, attrib);
  if (vtable && attrib)
    {
#ifdef CONFIG_FB_HWCURSORIMAGE
      attrib->fmt      = FB_FMT;
#endif
      dbg("pos:      (x=%d, y=%d)\n", g_cpos.x, g_cpos.y);
      attrib->pos      = g_cpos;
#ifdef CONFIG_FB_HWCURSORSIZE
      attrib->mxsize.h = CONFIG_LPC17_LCD_VHEIGHT;
      attrib->mxsize.w = CONFIG_LPC17_LCD_HWIDTH;
      dbg("size:     (h=%d, w=%d)\n", g_csize.h, g_csize.w);
      attrib->size     = g_csize;
#endif
      return OK;
    }

  dbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: lpc17_setcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int lpc17_setcursor(FAR struct fb_vtable_s *vtable,
                       FAR struct fb_setcursor_s *setttings)
{
  dbg("vtable=%p setttings=%p\n", vtable, setttings);
  if (vtable && setttings)
    {
      dbg("flags:   %02x\n", settings->flags);
      if ((flags & FB_CUR_SETPOSITION) != 0)
        {
          g_cpos = settings->pos;
          dbg("pos:     (h:%d, w:%d)\n", g_cpos.x, g_cpos.y);
        }
#ifdef CONFIG_FB_HWCURSORSIZE
      if ((flags & FB_CUR_SETSIZE) != 0)
        {
          g_csize = settings->size;
          dbg("size:    (h:%d, w:%d)\n", g_csize.h, g_csize.w);
        }
#endif
#ifdef CONFIG_FB_HWCURSORIMAGE
      if ((flags & FB_CUR_SETIMAGE) != 0)
        {
          dbg("image:   (h:%d, w:%d) @ %p\n",
              settings->img.height, settings->img.width, settings->img.image);
        }
#endif
      return OK;
    }

  dbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer video hardware
 *
 ****************************************************************************/

int up_fbinitialize(void)
{
  /* Disable LCD controller */
#warning "Missing logioc"

  /* Configure pins */
  /* R */

  lpc17_configgpio(GPIO_LCD_VD0);
  lpc17_configgpio(GPIO_LCD_VD1);
  lpc17_configgpio(GPIO_LCD_VD2);
  lpc17_configgpio(GPIO_LCD_VD3);
  lpc17_configgpio(GPIO_LCD_VD4);
  lpc17_configgpio(GPIO_LCD_VD5);
  lpc17_configgpio(GPIO_LCD_VD6);
  lpc17_configgpio(GPIO_LCD_VD7);

  /* G */     

  lpc17_configgpio(GPIO_LCD_VD8);
  lpc17_configgpio(GPIO_LCD_VD9);
  lpc17_configgpio(GPIO_LCD_VD10);
  lpc17_configgpio(GPIO_LCD_VD11);
  lpc17_configgpio(GPIO_LCD_VD12);
  lpc17_configgpio(GPIO_LCD_VD13);
  lpc17_configgpio(GPIO_LCD_VD14);
  lpc17_configgpio(GPIO_LCD_VD15);

  /* B */

  lpc17_configgpio(GPIO_LCD_VD16);
  lpc17_configgpio(GPIO_LCD_VD17);
  lpc17_configgpio(GPIO_LCD_VD18);
  lpc17_configgpio(GPIO_LCD_VD19);
  lpc17_configgpio(GPIO_LCD_VD20);
  lpc17_configgpio(GPIO_LCD_VD21);
  lpc17_configgpio(GPIO_LCD_VD22);
  lpc17_configgpio(GPIO_LCD_VD23);

  /* Other pins */

  lpc17_configgpio(GPIO_LCD_DCLK);
  lpc17_configgpio(GPIO_LCD_LP);
  lpc17_configgpio(GPIO_LCD_FP);
  lpc17_configgpio(GPIO_LCD_ENABM);
  lpc17_configgpio(GPIO_LCD_PWR);

  return OK;
}

/****************************************************************************
 * Name: lpc17_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane.
 *
 * Input parameters:
 *   None
 *
 * Returned value:
 *   Reference to the framebuffer object (NULL on failure)
 *
 ***************************************************************************/

FAR struct fb_vtable_s *up_fbgetvplane(int vplane)
{
  if (vplane == 0)
    {
      return &g_fbobject;
    }
  else
    {
      return NULL;
    }
}

/****************************************************************************
 * Name: lpc17_fbinitialize
 *
 * Description:
 *   Unitialize the framebuffer support
 *
 ****************************************************************************/

void fb_uninitialize(void)
{
  return OK;
}

