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

#include <nuttx/video/fb.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip/lpc17_syscon.h"
#include "lpc17_gpio.h"
#include "lpc17_lcd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LPC17_LCD_CLK_PER_LINE \
  (CONFIG_LPC17_LCD_HWIDTH + CONFIG_LPC17_LCD_HPULSE + \
   CONFIG_LPC17_LCD_HFRONTPORCH + CONFIG_LPC17_LCD_HBACKPORCH)
#define LPC17_LCD_LINES_PER_FRAME \
  (CONFIG_LPC17_LCD_VHEIGHT + CONFIG_LPC17_LCD_VPULSE + \
   CONFIG_LPC17_LCD_VFRONTPORCH + CONFIG_LPC17_LCD_VBACKPORCH)
#define LPC17_LCD_PIXEL_CLOCK \
  (LPC17_LCD_CLK_PER_LINE * LPC17_LCD_LINES_PER_FRAME * \
   CONFIG_LPC17_LCD_REFRESH_FREQ)

/* Framebuffer characteristics in bytes */

#if defined(CONFIG_LPC17_LCD_BPP1)
#  define LPC17_STRIDE ((CONFIG_LPC17_LCD_HWIDTH * 1 + 7) / 8)
#elif defined(CONFIG_LPC17_LCD_BPP2)
#  define LPC17_STRIDE ((CONFIG_LPC17_LCD_HWIDTH * 2 + 7) / 8)
#elif defined(CONFIG_LPC17_LCD_BPP4)
#  define LPC17_STRIDE ((CONFIG_LPC17_LCD_HWIDTH * 4 + 7) / 8)
#elif defined(CONFIG_LPC17_LCD_BPP8)
#  define LPC17_STRIDE ((CONFIG_LPC17_LCD_HWIDTH * 8 + 7) / 8)
#elif defined(CONFIG_LPC17_LCD_BPP16)
#  define LPC17_STRIDE ((CONFIG_LPC17_LCD_HWIDTH * 16 + 7) / 8)
#elif defined(CONFIG_LPC17_LCD_BPP24)
#  define LPC17_STRIDE ((CONFIG_LPC17_LCD_HWIDTH * 32 + 7) / 8)
#elif defined(CONFIG_LPC17_LCD_BPP16_565)
#  define LPC17_STRIDE ((CONFIG_LPC17_LCD_HWIDTH * 16 + 7) / 8)
#else /* defined(CONFIG_LPC17_LCD_BPP12_444) */
#  define LPC17_STRIDE ((CONFIG_LPC17_LCD_HWIDTH * 16 + 7) / 8)
#endif

#define LPC17_FBSIZE (LPC17_STRIDE * CONFIG_LPC17_LCD_VHEIGHT)

/* Delays */

#define LPC17_LCD_PWRDIS_DELAY 10000
#define LPC17_LCD_PWREN_DELAY  10000

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

/* This structure describes the video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt      = LPC17_COLOR_FMT,
  .xres     = CONFIG_LPC17_LCD_HWIDTH,
  .yres     = CONFIG_LPC17_LCD_VHEIGHT,
  .nplanes  = 1,
};

/* This structure describes the single color plane */

static const struct fb_planeinfo_s g_planeinfo =
{
  .fbmem    = (FAR void *)CONFIG_LPC17_LCD_VRAMBASE,
  .fblen    = LPC17_FBSIZE,
  .stride   = LPC17_STRIDE,
  .bpp      = LPC17_BPP,
};

/* Current cursor position */

#ifdef CONFIG_FB_HWCURSOR
static struct fb_cursorpos_s g_cpos;

/* Current cursor size */

#ifdef CONFIG_FB_HWCURSORSIZE
static struct fb_cursorsize_s g_csize;
#endif
#endif

/* The framebuffer object -- There is no private state information in this
 * framebuffer driver.
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
  gvdbg("vtable=%p vinfo=%p\n", vtable, vinfo);
  if (vtable && vinfo)
    {
      memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: lpc17_getplaneinfo
 ****************************************************************************/

static int lpc17_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                              FAR struct fb_planeinfo_s *pinfo)
{
  gvdbg("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);
  if (vtable && planeno == 0 && pinfo)
    {
      memcpy(pinfo, &g_planeinfo, sizeof(struct fb_planeinfo_s));
      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: lpc17_getcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int lpc17_getcmap(FAR struct fb_vtable_s *vtable,
                         FAR struct fb_cmap_s *cmap)
{
  uint32_t *pal;
  uint32_t rgb;
  int last;
  int i;

  gvdbg("vtable=%p cmap=%p first=%d len=%d\n",
        vtable, cmap, cmap->first, cmap->len);

  DEBUGASSERT(vtable && cmap &&
              cmap->first < 256 && (cmap->first + cmap->len) < 256);

  pal  = (uint32_t *)LPC17_LCD_PAL(cmap->first >> 1);
  last = cmap->first + cmap->len;

  /* Handle the case where the first color starts on an odd boundary */

  i = cmap->first;
  if ((i & 1) != 0)
    {
      rgb  = *pal++;
      i++;

      /* Save the odd palette value */

      cmap->red[i]    = (rgb & LCD_PAL_R1_MASK) >> LCD_PAL_R1_SHIFT;
      cmap->green[i]  = (rgb & LCD_PAL_G1_MASK) >> LCD_PAL_G1_SHIFT;
      cmap->blue[i]   = (rgb & LCD_PAL_B1_MASK) >> LCD_PAL_B1_SHIFT;
#ifdef CONFIG_FB_TRANSPARENCY
      cmap->transp[i] = 0;
#endif
    }

  /* Handle even colors */

  for (; i < last; i += 2)
    {
      rgb  = *pal++;

      /* Save the even palette value */

      cmap->red[i]    = (rgb & LCD_PAL_R0_MASK) >> LCD_PAL_R0_SHIFT;
      cmap->green[i]  = (rgb & LCD_PAL_G0_MASK) >> LCD_PAL_G0_SHIFT;
      cmap->blue[i]   = (rgb & LCD_PAL_B0_MASK) >> LCD_PAL_B0_SHIFT;
#ifdef CONFIG_FB_TRANSPARENCY
      cmap->transp[i] = 0;
#endif

      /* Handle the case where the len ends on an odd boudary */

      if ((i + 1) < last)
        {
          /* Save the even palette value */

          cmap->red[i+1]    = (rgb & LCD_PAL_R1_MASK) >> LCD_PAL_R1_SHIFT;
          cmap->green[i+1]  = (rgb & LCD_PAL_G1_MASK) >> LCD_PAL_G1_SHIFT;
          cmap->blue[i+1]   = (rgb & LCD_PAL_B1_MASK) >> LCD_PAL_B1_SHIFT;
#ifdef CONFIG_FB_TRANSPARENCY
          cmap->transp[i+1] = 0;
#endif
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: lpc17_putcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int lpc17_putcmap(FAR struct fb_vtable_s *vtable,
                         FAR const struct fb_cmap_s *cmap)
{
  uint32_t *pal;
  uint32_t rgb0;
  uint32_t rgb1;
  int last;
  int i;

  gvdbg("vtable=%p cmap=%p first=%d len=%d\n",
        vtable, cmap, cmap->first, cmap->len);

  DEBUGASSERT(vtable && cmap);

  pal  = (uint32_t *)LPC17_LCD_PAL(cmap->first >> 1);
  last = cmap->first + cmap->len;

  /* Handle the case where the first color starts on an odd boundary */

  i = cmap->first;
  if ((i & 1) != 0)
    {
      rgb0  = *pal;
      rgb0 &= (LCD_PAL_R0_MASK | LCD_PAL_G0_MASK | LCD_PAL_B0_MASK | LCD_PAL_I0);
      rgb1 |= ((uint32_t)cmap->red[i]   << LCD_PAL_R0_SHIFT |
               (uint32_t)cmap->green[i] << LCD_PAL_G0_SHIFT |
               (uint32_t)cmap->blue[i]  << LCD_PAL_B0_SHIFT);

      /* Save the new palette value */

      *pal++ = (rgb0 | rgb1);
      i++;
    }

  /* Handle even colors */

  for (; i < last; i += 2)
    {
      uint32_t rgb0 = ((uint32_t)cmap->red[i]   << LCD_PAL_R0_SHIFT |
                       (uint32_t)cmap->green[i] << LCD_PAL_G0_SHIFT |
                       (uint32_t)cmap->blue[i]  << LCD_PAL_B0_SHIFT);

      /* Handle the case where the len ends on an odd boudary */

      if ((i + 1) >= last)
        {
          rgb1  = *pal;
          rgb1 &= (LCD_PAL_R1_MASK | LCD_PAL_G1_MASK | LCD_PAL_B1_MASK | LCD_PAL_I1);
        }
      else
        {
          rgb1  = ((uint32_t)cmap->red[i+1]   << LCD_PAL_R1_SHIFT |
                   (uint32_t)cmap->green[i+1] << LCD_PAL_G1_SHIFT |
                   (uint32_t)cmap->blue[i+1]  << LCD_PAL_B1_SHIFT);
        }

      /* Save the new pallete value */

      *pal++ = (rgb0 | rgb1);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: lpc17_getcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int lpc17_getcursor(FAR struct fb_vtable_s *vtable,
                        FAR struct fb_cursorattrib_s *attrib)
{
  gvdbg("vtable=%p attrib=%p\n", vtable, attrib);
  if (vtable && attrib)
    {
#ifdef CONFIG_FB_HWCURSORIMAGE
      attrib->fmt = LPC17_COLOR_FMT;
#endif

      gvdbg("pos: (x=%d, y=%d)\n", g_cpos.x, g_cpos.y);
      attrib->pos = g_cpos;

#ifdef CONFIG_FB_HWCURSORSIZE
      attrib->mxsize.h = CONFIG_LPC17_LCD_VHEIGHT;
      attrib->mxsize.w = CONFIG_LPC17_LCD_HWIDTH;

      gvdbg("size: (h=%d, w=%d)\n", g_csize.h, g_csize.w);
      attrib->size = g_csize;
#endif
      return OK;
    }

  gdbg("Returning EINVAL\n");
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
  gvdbg("vtable=%p setttings=%p\n", vtable, setttings);
  if (vtable && setttings)
    {
      gvdbg("flags: %02x\n", settings->flags);
      if ((flags & FB_CUR_SETPOSITION) != 0)
        {
          g_cpos = settings->pos;
          gvdbg("pos: (h:%d, w:%d)\n", g_cpos.x, g_cpos.y);
        }
#ifdef CONFIG_FB_HWCURSORSIZE
      if ((flags & FB_CUR_SETSIZE) != 0)
        {
          g_csize = settings->size;
          gvdbg("size: (h:%d, w:%d)\n", g_csize.h, g_csize.w);
        }
#endif
#ifdef CONFIG_FB_HWCURSORIMAGE
      if ((flags & FB_CUR_SETIMAGE) != 0)
        {
          gvdbg("image: (h:%d, w:%d) @ %p\n",
                settings->img.height, settings->img.width,
                settings->img.image);
        }
#endif
      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer video hardware
 *
 ****************************************************************************/

int up_fbinitialize(void)
{
  uint32_t regval;
  int i;

  gvdbg("Entry\n");

  /* Give LCD bus priority */

  regval = ((SYSCON_MATRIXARB_PRI_ICODE(SYSCON_MATRIXARB_PRI_LOW)) |
            (SYSCON_MATRIXARB_PRI_DCODE(SYSCON_MATRIXARB_PRI_HIGHEST)) |
            (SYSCON_MATRIXARB_PRI_LCD(SYSCON_MATRIXARB_PRI_HIGHEST)));
  putreg32(regval, LPC17_SYSCON_MATRIXARB);

  /* Configure pins */
  /* Video data */

  gvdbg("Configuring pins\n");

  lpc17_configgpio(GPIO_LCD_VD0);
  lpc17_configgpio(GPIO_LCD_VD1);
  lpc17_configgpio(GPIO_LCD_VD2);
  lpc17_configgpio(GPIO_LCD_VD3);
  lpc17_configgpio(GPIO_LCD_VD4);
  lpc17_configgpio(GPIO_LCD_VD5);
  lpc17_configgpio(GPIO_LCD_VD6);
  lpc17_configgpio(GPIO_LCD_VD7);

  lpc17_configgpio(GPIO_LCD_VD8);
  lpc17_configgpio(GPIO_LCD_VD9);
  lpc17_configgpio(GPIO_LCD_VD10);
  lpc17_configgpio(GPIO_LCD_VD11);
  lpc17_configgpio(GPIO_LCD_VD12);
  lpc17_configgpio(GPIO_LCD_VD13);
  lpc17_configgpio(GPIO_LCD_VD14);
  lpc17_configgpio(GPIO_LCD_VD15);

#if LPC17_BPP > 16
  lpc17_configgpio(GPIO_LCD_VD16);
  lpc17_configgpio(GPIO_LCD_VD17);
  lpc17_configgpio(GPIO_LCD_VD18);
  lpc17_configgpio(GPIO_LCD_VD19);
  lpc17_configgpio(GPIO_LCD_VD20);
  lpc17_configgpio(GPIO_LCD_VD21);
  lpc17_configgpio(GPIO_LCD_VD22);
  lpc17_configgpio(GPIO_LCD_VD23);
#endif

  /* Other pins */

  lpc17_configgpio(GPIO_LCD_DCLK);
  lpc17_configgpio(GPIO_LCD_LP);
  lpc17_configgpio(GPIO_LCD_FP);
  lpc17_configgpio(GPIO_LCD_ENABM);
  lpc17_configgpio(GPIO_LCD_PWR);

  /* Turn on LCD clock */

  modifyreg32(LPC17_SYSCON_PCONP, 0, SYSCON_PCONP_PCLCD);

  gvdbg("Configuring the LCD controller\n");

  /* Disable the cursor */

  regval  = getreg32(LPC17_LCD_CRSR_CRTL);
  regval &= ~LCD_CRSR_CTRL_CRSON;
  putreg32(regval, LPC17_LCD_CRSR_CRTL);

  /* Clear any pending interrupts */

  putreg32(LCD_INTCLR_ALL, LPC17_LCD_INTCLR);

  /* Disable GLCD controller */

  putreg32(0, LPC17_LCD_CTRL);

  /* Initialize pixel clock (assuming clock source is the peripheral clock) */

  putreg32(((uint32_t)BOARD_PCLK_FREQUENCY / (uint32_t)LPC17_LCD_PIXEL_CLOCK)+1,
           LPC17_SYSCON_LCDCFG);

  /* Set the bits per pixel */

  regval  = getreg32(LPC17_LCD_CTRL);
  regval &= ~LCD_CTRL_LCDBPP_MASK;

#if defined(CONFIG_LPC17_LCD_BPP1)
  regval |= LCD_CTRL_LCDBPP_1;      /* 1 bpp */
#elif defined(CONFIG_LPC17_LCD_BPP2)
  regval |= LCD_CTRL_LCDBPP_2;      /* 2 bpp */
#elif defined(CONFIG_LPC17_LCD_BPP4)
  regval |= LCD_CTRL_LCDBPP_4;      /* 4 bpp */
#elif defined(CONFIG_LPC17_LCD_BPP8)
  regval |= LCD_CTRL_LCDBPP_8;      /* 8 bpp */
#elif defined(CONFIG_LPC17_LCD_BPP16)
  regval |= LCD_CTRL_LCDBPP_16;     /* 16 bpp */
#elif defined(CONFIG_LPC17_LCD_BPP24)
  regval |= LCD_CTRL_LCDBPP_24;     /* 24-bit TFT panel only */
#elif defined(CONFIG_LPC17_LCD_BPP16_565)
  regval |= LCD_CTRL_LCDBPP_565;    /* 16 bpp, 5:6:5 mode */
#else /* defined(CONFIG_LPC17_LCD_BPP12_444) */
  regval |= LCD_CTRL_LCDBPP_444;    /* 12 bpp, 4:4:4 mode */
#endif

  /* TFT panel */

#if CONFIG_LPC17_LCD_TFTPANEL
  regval |= LCD_CTRL_LCDTFT;
#endif

  /* Swap red and blue.  The colors will be 0x00RRGGBB, not 0x00BBGGRR. */

  regval |= LCD_CTRL_BGR;

  /* Single panel */

  regval &= ~LCD_CTRL_LCDDUAL;

  /* Select monochrome or color LCD */

#ifdef CONFIG_LPC17_LCD_MONOCHROME
  /* Select monochrome LCD */

  regval &= ~LCD_CTRL_BGR;

  /* Select 4- or 8-bit monochrome interface */

#  if LPC17_BPP > 4
  regval |= LCD_CTRL_LCDMONO8;
#  else
  regval &= ~LCD_CTRL_LCDMONO8;
#  endif

#else
  /* Select color LCD */

  regval &= ~(LCD_CTRL_LCDBW | LCD_CTRL_LCDMONO8);

#endif /* CONFIG_LPC17_LCD_MONOCHROME */

  /* Little endian byte order */

  regval &= ~LCD_CTRL_BEBO;

  /* Little endian pixel order */

  regval &= ~LCD_CTRL_BEPO;
  putreg32(regval, LPC17_LCD_CTRL);

  /* Initialize horizontal timing */

  putreg32(0, LPC17_LCD_TIMH);

  regval = (((CONFIG_LPC17_LCD_HWIDTH/16) - 1) << LCD_TIMH_PPL_SHIFT |
            (CONFIG_LPC17_LCD_HPULSE - 1)      << LCD_TIMH_HSW_SHIFT |
            (CONFIG_LPC17_LCD_HFRONTPORCH - 1) << LCD_TIMH_HFP_SHIFT |
            (CONFIG_LPC17_LCD_HBACKPORCH - 1)  << LCD_TIMH_HBP_SHIFT);
  putreg32(regval, LPC17_LCD_TIMH);

  /* Initialize vertical timing */

  putreg32(0, LPC17_LCD_TIMV);

  regval = ((CONFIG_LPC17_LCD_VHEIGHT - 1) << LCD_TIMV_LPP_SHIFT |
            (CONFIG_LPC17_LCD_VPULSE - 1)  << LCD_TIMV_VSW_SHIFT |
            (CONFIG_LPC17_LCD_VFRONTPORCH) << LCD_TIMV_VFP_SHIFT |
            (CONFIG_LPC17_LCD_VBACKPORCH)  << LCD_TIMV_VBP_SHIFT);
  putreg32(regval, LPC17_LCD_TIMV);

  /* Initialize clock and signal polarity */

  regval = getreg32(LPC17_LCD_POL);

  /* LCDFP pin is active LOW and inactive HIGH */

  regval |= LCD_POL_IVS;

  /* LCDLP pin is active LOW and inactive HIGH */

  regval |= LCD_POL_IHS;

  /* Data is driven out into the LCD on the falling edge */

  regval &= ~LCD_POL_IPC;

  /* Set number of clocks per line */

  regval |= ((CONFIG_LPC17_LCD_HWIDTH-1) << LCD_POL_CPL_SHIFT);

  /* Bypass internal pixel clock divider */

  regval |= LCD_POL_BCD;

  /* LCD_ENAB_M is active high */

  regval &= ~LCD_POL_IOE;

  /* Select CCLK for the LCD block clock source */

  regval &= ~LCD_POL_CLKSEL;
  putreg32(regval, LPC17_LCD_POL);

  /* Frame base address doubleword aligned */

  putreg32(CONFIG_LPC17_LCD_VRAMBASE & ~7, LPC17_LCD_UPBASE);
  putreg32(CONFIG_LPC17_LCD_VRAMBASE & ~7, LPC17_LCD_LPBASE);

  /* Clear the display */

  lpc17_lcdclear(CONFIG_LPC17_LCD_BACKCOLOR);

#ifdef CONFIG_LPC17_LCD_BACKLIGHT
  /* Turn on the back light */

  lpc17_backlight(true);
#endif

  putreg32(0, LPC17_LCD_INTMSK);
  gvdbg("Enabling the display\n");

  for (i = LPC17_LCD_PWREN_DELAY; i; i--);

  /* Enable LCD */

  regval  = getreg32(LPC17_LCD_CTRL);
  regval |= LCD_CTRL_LCDEN;
  putreg32(regval, LPC17_LCD_CTRL);

  /* Enable LCD power */

  for (i = LPC17_LCD_PWREN_DELAY; i; i--);

  regval  = getreg32(LPC17_LCD_CTRL);
  regval |= LCD_CTRL_LCDPWR;
  putreg32(regval, LPC17_LCD_CTRL);

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
  gvdbg("vplane: %d\n", vplane);
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
 * Name: fb_uninitialize
 *
 * Description:
 *   Unitialize the framebuffer support
 *
 ****************************************************************************/

void fb_uninitialize(void)
{
  uint32_t regval;
  int i;

  /* We assume there is only one use of the LCD and so we do not need to
   * worry about mutually exclusive access to the LCD hardware.
   */

#ifdef CONFIG_LPC17_LCD_BACKLIGHT
  /* Turn off the back light */

  lpc17_backlight(false);
#endif

  /* Disable the LCD controller */

  regval = getreg32(LPC17_LCD_CTRL);
  regval &= ~LCD_CTRL_LCDPWR;
  putreg32(regval, LPC17_LCD_CTRL);

  for (i = LPC17_LCD_PWRDIS_DELAY; i; i--);

  regval &= ~LCD_CTRL_LCDEN;
  putreg32(regval, LPC17_LCD_CTRL);

  /* Turn off clocking to the LCD. modifyreg32() can do this atomically. */

  modifyreg32(LPC17_SYSCON_PCONP, SYSCON_PCONP_PCLCD, 0);
  return OK;
}

/************************************************************************************
 * Name:  lpc17_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the LPC17xx.  Clearing the display
 *   in the normal way by writing a sequences of runs that covers the entire display
 *   can be slow.  Here the display is cleared by simply setting all VRAM memory to
 *   the specified color.
 *
 ************************************************************************************/

void lpc17_lcdclear(nxgl_mxpixel_t color)
{
  int i;
#if LPC17_BPP > 16
  uint32_t *dest = (uint32_t*)CONFIG_LPC17_LCD_VRAMBASE;

  gvdbg("Clearing display: color=%08x VRAM=%08x size=%d\n",
        color, CONFIG_LPC17_LCD_VRAMBASE,
        CONFIG_LPC17_LCD_HWIDTH * CONFIG_LPC17_LCD_VHEIGHT * sizeof(uint32_t));

#else
  uint16_t *dest = (uint16_t*)CONFIG_LPC17_LCD_VRAMBASE;

  gvdbg("Clearing display: color=%08x VRAM=%08x size=%d\n",
        color, CONFIG_LPC17_LCD_VRAMBASE,
        CONFIG_LPC17_LCD_HWIDTH * CONFIG_LPC17_LCD_VHEIGHT * sizeof(uint16_t));
#endif

  for (i = 0; i < (CONFIG_LPC17_LCD_HWIDTH * CONFIG_LPC17_LCD_VHEIGHT); i++)
    {
      *dest++ = color;
    }
}
