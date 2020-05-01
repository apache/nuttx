/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_lcd.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This driver derives from the LPC1788 LCD driver.  The LPC1788 LCD is
 * identical tot he LPC54628 LCD other than some minor clocking differences.
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

#include "arm_arch.h"
#include "hardware/lpc54_syscon.h"
#include "hardware/lpc54_pinmux.h"
#include "lpc54_config.h"
#include "lpc54_enableclk.h"
#include "lpc54_gpio.h"
#include "lpc54_reset.h"
#include "lpc54_lcd.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LPC54_LCD_CLK_PER_LINE \
  (CONFIG_LPC54_LCD_HWIDTH + CONFIG_LPC54_LCD_HPULSE + \
   CONFIG_LPC54_LCD_HFRONTPORCH + CONFIG_LPC54_LCD_HBACKPORCH)
#define LPC54_LCD_LINES_PER_FRAME \
  (CONFIG_LPC54_LCD_VHEIGHT + CONFIG_LPC54_LCD_VPULSE + \
   CONFIG_LPC54_LCD_VFRONTPORCH + CONFIG_LPC54_LCD_VBACKPORCH)
#define LPC54_LCD_PIXEL_CLOCK \
  (LPC54_LCD_CLK_PER_LINE * LPC54_LCD_LINES_PER_FRAME * \
   CONFIG_LPC54_LCD_REFRESH_FREQ)

/* Framebuffer characteristics in bytes */

#define LPC54_STRIDE ((CONFIG_LPC54_LCD_HWIDTH * LPC54_BPP + 7) / 8)
#define LPC54_FBSIZE (LPC54_STRIDE * CONFIG_LPC54_LCD_VHEIGHT)

/* Delays */

#define LPC54_LCD_PWRDIS_DELAY 10000
#define LPC54_LCD_PWREN_DELAY  10000

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Get information about the video controller configuration and the
 * configuration of each color plane.
 */

static int lpc54_getvideoinfo(FAR struct fb_vtable_s *vtable,
             FAR struct fb_videoinfo_s *vinfo);
static int lpc54_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
             FAR struct fb_planeinfo_s *pinfo);

/* The following is provided only if the video hardware supports RGB color
 * mapping
 */

#ifdef CONFIG_FB_CMAP
static int lpc54_getcmap(FAR struct fb_vtable_s *vtable,
             FAR struct fb_cmap_s *cmap);
static int lpc54_putcmap(FAR struct fb_vtable_s *vtable,
             FAR const struct fb_cmap_s *cmap);
#endif

/* The following is provided only if the video hardware supports a hardware
 * cursor
 */

#ifdef CONFIG_FB_HWCURSOR
static int lpc54_getcursor(FAR struct fb_vtable_s *vtable,
             FAR struct fb_cursorattrib_s *attrib);
static int lpc54_setcursor(FAR struct fb_vtable_s *vtable,
             FAR struct fb_setcursor_s *settings);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure describes the video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt      = LPC54_COLOR_FMT,
  .xres     = CONFIG_LPC54_LCD_HWIDTH,
  .yres     = CONFIG_LPC54_LCD_VHEIGHT,
  .nplanes  = 1,
};

/* This structure describes the single color plane */

static const struct fb_planeinfo_s g_planeinfo =
{
  .fbmem    = (FAR void *)CONFIG_LPC54_LCD_VRAMBASE,
  .fblen    = LPC54_FBSIZE,
  .stride   = LPC54_STRIDE,
  .display  = 0,
  .bpp      = LPC54_BPP,
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
  .getvideoinfo  = lpc54_getvideoinfo,
  .getplaneinfo  = lpc54_getplaneinfo,
#ifdef CONFIG_FB_CMAP
  .getcmap       = lpc54_getcmap,
  .putcmap       = lpc54_putcmap,
#endif
#ifdef CONFIG_FB_HWCURSOR
  .getcursor     = lpc54_getcursor,
  .setcursor     = lpc54_setcursor,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_getvideoinfo
 ****************************************************************************/

static int lpc54_getvideoinfo(FAR struct fb_vtable_s *vtable,
                              FAR struct fb_videoinfo_s *vinfo)
{
  lcdinfo("vtable=%p vinfo=%p\n", vtable, vinfo);
  if (vtable && vinfo)
    {
      memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: lpc54_getplaneinfo
 ****************************************************************************/

static int lpc54_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                              FAR struct fb_planeinfo_s *pinfo)
{
  lcdinfo("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);
  if (vtable && planeno == 0 && pinfo)
    {
      memcpy(pinfo, &g_planeinfo, sizeof(struct fb_planeinfo_s));
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: lpc54_getcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int lpc54_getcmap(FAR struct fb_vtable_s *vtable,
                         FAR struct fb_cmap_s *cmap)
{
  uint32_t *pal;
  uint32_t rgb;
  int last;
  int i;

  lcdinfo("vtable=%p cmap=%p first=%d len=%d\n",
          vtable, cmap, cmap->first, cmap->len);

  DEBUGASSERT(vtable && cmap &&
              cmap->first < 256 && (cmap->first + cmap->len) < 256);

  pal  = (uint32_t *)LPC54_LCD_PAL(cmap->first >> 1);
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

      /* Handle the case where the len ends on an odd boundary */

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
 * Name: lpc54_putcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int lpc54_putcmap(FAR struct fb_vtable_s *vtable,
                         FAR const struct fb_cmap_s *cmap)
{
  uint32_t *pal;
  uint32_t rgb0;
  uint32_t rgb1;
  int last;
  int i;

  lcdinfo("vtable=%p cmap=%p first=%d len=%d\n",
          vtable, cmap, cmap->first, cmap->len);

  DEBUGASSERT(vtable && cmap);

  pal  = (uint32_t *)LPC54_LCD_PAL(cmap->first >> 1);
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

      /* Handle the case where the len ends on an odd boundary */

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

      /* Save the new palette value */

      *pal++ = (rgb0 | rgb1);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: lpc54_getcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int lpc54_getcursor(FAR struct fb_vtable_s *vtable,
                        FAR struct fb_cursorattrib_s *attrib)
{
  lcdinfo("vtable=%p attrib=%p\n", vtable, attrib);
  if (vtable && attrib)
    {
#ifdef CONFIG_FB_HWCURSORIMAGE
      attrib->fmt = LPC54_COLOR_FMT;
#endif

      lcdinfo("pos: (x=%d, y=%d)\n", g_cpos.x, g_cpos.y);
      attrib->pos = g_cpos;

#ifdef CONFIG_FB_HWCURSORSIZE
      attrib->mxsize.h = CONFIG_LPC54_LCD_VHEIGHT;
      attrib->mxsize.w = CONFIG_LPC54_LCD_HWIDTH;

      lcdinfo("size: (h=%d, w=%d)\n", g_csize.h, g_csize.w);
      attrib->size = g_csize;
#endif
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: lpc54_setcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int lpc54_setcursor(FAR struct fb_vtable_s *vtable,
                       FAR struct fb_setcursor_s *settings)
{
  lcdinfo("vtable=%p settings=%p\n", vtable, settings);
  if (vtable && settings)
    {
      lcdinfo("flags: %02x\n", settings->flags);
      if ((flags & FB_CUR_SETPOSITION) != 0)
        {
          g_cpos = settings->pos;
          lcdinfo("pos: (h:%d, w:%d)\n", g_cpos.x, g_cpos.y);
        }
#ifdef CONFIG_FB_HWCURSORSIZE
      if ((flags & FB_CUR_SETSIZE) != 0)
        {
          g_csize = settings->size;
          lcdinfo("size: (h:%d, w:%d)\n", g_csize.h, g_csize.w);
        }
#endif
#ifdef CONFIG_FB_HWCURSORIMAGE
      if ((flags & FB_CUR_SETIMAGE) != 0)
        {
          lcdinfo("image: (h:%d, w:%d) @ %p\n",
                  settings->img.height, settings->img.width,
                  settings->img.image);
        }
#endif
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
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
 *   Initialize the framebuffer video hardware associated with the display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  uint32_t regval;
  uint32_t bcd;
  uint32_t pcd;
  int i;

  /* Configure pins */
  /* LCD panel data. Pins used depend on the panel configuration:
   *
   * STN  4BPP: VD0-VD3           (single panel)
   *            VD0-VD3, VD8-VD11 (dual panel)
   * STN  8BPP: VD0-VD7           (single panel)
   *            VD0-VD15          (dual panel)
   * TFT 12BPP: VD4-VD7, VD12-VD15, VD20-VD23
   * TFT 16BPP: VD3-VD7, VD10-VD15, VD19-VD23
   * TFT 24BPP: VD0-VD23
  */

  lcdinfo("Configuring pins\n");

#if !defined(CONFIG_LPC54_LCD_BPP16_565) && !defined(CONFIG_LPC54_LCD_BPP12_444)
  lpc54_gpio_config(GPIO_LCD_VD0);
  lpc54_gpio_config(GPIO_LCD_VD1);
  lpc54_gpio_config(GPIO_LCD_VD2);
#endif
#ifndef CONFIG_LPC54_LCD_BPP12_444
  lpc54_gpio_config(GPIO_LCD_VD3);
#endif
  lpc54_gpio_config(GPIO_LCD_VD4);
  lpc54_gpio_config(GPIO_LCD_VD5);
  lpc54_gpio_config(GPIO_LCD_VD6);
  lpc54_gpio_config(GPIO_LCD_VD7);

#if LPC54_BPP > 8 /* Or STN 8-BPP Dual panel */
#if !defined(CONFIG_LPC54_LCD_BPP16_565) && !defined(CONFIG_LPC54_LCD_BPP12_444)
  lpc54_gpio_config(GPIO_LCD_VD8);
  lpc54_gpio_config(GPIO_LCD_VD9);
#endif
#ifndef CONFIG_LPC54_LCD_BPP12_444
  lpc54_gpio_config(GPIO_LCD_VD10);
  lpc54_gpio_config(GPIO_LCD_VD11);
#endif
  lpc54_gpio_config(GPIO_LCD_VD12);
  lpc54_gpio_config(GPIO_LCD_VD13);
  lpc54_gpio_config(GPIO_LCD_VD14);
  lpc54_gpio_config(GPIO_LCD_VD15);
#endif

#if LPC54_BPP > 16 || defined(CONFIG_LPC54_LCD_TFTPANEL)
#if !defined(CONFIG_LPC54_LCD_BPP16_565) && !defined(CONFIG_LPC54_LCD_BPP12_444)
  lpc54_gpio_config(GPIO_LCD_VD16);
  lpc54_gpio_config(GPIO_LCD_VD17);
  lpc54_gpio_config(GPIO_LCD_VD18);
#endif
#ifndef CONFIG_LPC54_LCD_BPP12_444
  lpc54_gpio_config(GPIO_LCD_VD19);
#endif
  lpc54_gpio_config(GPIO_LCD_VD20);
  lpc54_gpio_config(GPIO_LCD_VD21);
  lpc54_gpio_config(GPIO_LCD_VD22);
  lpc54_gpio_config(GPIO_LCD_VD23);
#endif

  /* Other pins */

  lpc54_gpio_config(GPIO_LCD_AC);    /* STN AC bias drive or TFT data enable output */
  lpc54_gpio_config(GPIO_LCD_DCLK);  /* LCD panel clock */
  lpc54_gpio_config(GPIO_LCD_FP);    /* Frame pulse (STN).
                                      * Vertical synchronization pulse (TFT) */
  lpc54_gpio_config(GPIO_LCD_LE);    /* Line end signal */
  lpc54_gpio_config(GPIO_LCD_LP);    /* Line synchronization pulse (STN).
                                      * Horizontal synchronization pulse (TFT) */
  lpc54_gpio_config(GPIO_LCD_PWR);   /* LCD panel power enable */

#ifdef CONFIG_LPC54_LCD_USE_CLKIN
  lpc54_gpio_config(GPIO_LCD_CLKIN); /* Optional clock input */
#endif

  lcdinfo("Enable clocking to the LCD controller\n");

  /* Enable clocking to the LCD peripheral */

  lpc54_lcd_enableclk();

  /* Route Main clock (or LCK CLKIN) to the LCD. */

#ifdef CONFIG_LPC54_LCD_USE_CLKIN
  putreg32(SYSCON_LCDCLKSEL_LCDCLKIN, LPC54_SYSCON_LCDCLKSEL);
#else
  putreg32(SYSCON_LCDCLKSEL_MAINCLK, LPC54_SYSCON_LCDCLKSEL);
#endif

  /* Set the LCD clock divider to one. */

  putreg32(SYSCON_LCDCLKDIV_DIV(1), LPC54_SYSCON_LCDCLKDIV);
  putreg32(SYSCON_LCDCLKDIV_DIV(1) | SYSCON_LCDCLKDIV_REQFLAG,
           LPC54_SYSCON_LCDCLKDIV);

  /* Reset the LCD */

  lpc54_reset_lcd();

  lcdinfo("Configuring the LCD controller\n");

  /* Disable the cursor */

  regval  = getreg32(LPC54_LCD_CRSR_CRTL);
  regval &= ~LCD_CRSR_CTRL_CRSON;
  putreg32(regval, LPC54_LCD_CRSR_CRTL);

  /* Clear any pending interrupts */

  putreg32(LCD_INTCLR_ALL, LPC54_LCD_INTCLR);

  /* Disable the LCD controller */

  putreg32(0, LPC54_LCD_CTRL);

  /* Set the bits per pixel */

  regval  = getreg32(LPC54_LCD_CTRL);
  regval &= ~LCD_CTRL_LCDBPP_MASK;

#if defined(CONFIG_LPC54_LCD_BPP1)
  regval |= LCD_CTRL_LCDBPP_1;      /* 1 bpp */
#elif defined(CONFIG_LPC54_LCD_BPP2)
  regval |= LCD_CTRL_LCDBPP_2;      /* 2 bpp */
#elif defined(CONFIG_LPC54_LCD_BPP4)
  regval |= LCD_CTRL_LCDBPP_4;      /* 4 bpp */
#elif defined(CONFIG_LPC54_LCD_BPP8)
  regval |= LCD_CTRL_LCDBPP_8;      /* 8 bpp */
#elif defined(CONFIG_LPC54_LCD_BPP16)
  regval |= LCD_CTRL_LCDBPP_16;     /* 16 bpp */
#elif defined(CONFIG_LPC54_LCD_BPP24)
  regval |= LCD_CTRL_LCDBPP_24;     /* 24-bit TFT panel only */
#elif defined(CONFIG_LPC54_LCD_BPP16_565)
  regval |= LCD_CTRL_LCDBPP_565;    /* 16 bpp, 5:6:5 mode */
#else /* defined(CONFIG_LPC54_LCD_BPP12_444) */
  regval |= LCD_CTRL_LCDBPP_444;    /* 12 bpp, 4:4:4 mode */
#endif

#ifdef CONFIG_LPC54_LCD_TFTPANEL
  /* TFT panel */

  regval |= LCD_CTRL_LCDTFT;
#else
  /* STN panel */

  regval &= ~LCD_CTRL_LCDTFT;
#endif

#ifdef CONFIG_LPC54_LCD_BGR
  /* Swap red and blue.  The colors will be 0x00RRGGBB, not 0x00BBGGRR. */

  regval |= LCD_CTRL_BGR;
#else
  regval &= ~LCD_CTRL_BGR;
#endif

  /* Single panel */

  regval &= ~LCD_CTRL_LCDDUAL;

  /* Select monochrome or color LCD */

#ifdef CONFIG_LPC54_LCD_MONOCHROME
  /* Select monochrome LCD */

  regval &= ~LCD_CTRL_BGR;

  /* Select 4- or 8-bit monochrome interface */

#if LPC54_BPP > 4
  regval |= LCD_CTRL_LCDMONO8;
#else
  regval &= ~LCD_CTRL_LCDMONO8;
#endif

#else
  /* Select color LCD */

  regval &= ~(LCD_CTRL_LCDBW | LCD_CTRL_LCDMONO8);

#endif /* CONFIG_LPC54_LCD_MONOCHROME */

  /* Little endian byte order */

  regval &= ~LCD_CTRL_BEBO;

  /* Little endian pixel order */

  regval &= ~LCD_CTRL_BEPO;
  putreg32(regval, LPC54_LCD_CTRL);

  /* Initialize horizontal timing */

  putreg32(0, LPC54_LCD_TIMH);

  regval = (((CONFIG_LPC54_LCD_HWIDTH/16) - 1) << LCD_TIMH_PPL_SHIFT |
            (CONFIG_LPC54_LCD_HPULSE - 1)      << LCD_TIMH_HSW_SHIFT |
            (CONFIG_LPC54_LCD_HFRONTPORCH - 1) << LCD_TIMH_HFP_SHIFT |
            (CONFIG_LPC54_LCD_HBACKPORCH - 1)  << LCD_TIMH_HBP_SHIFT);
  putreg32(regval, LPC54_LCD_TIMH);

  /* Initialize vertical timing */

  putreg32(0, LPC54_LCD_TIMV);

  regval = ((CONFIG_LPC54_LCD_VHEIGHT - 1) << LCD_TIMV_LPP_SHIFT |
            (CONFIG_LPC54_LCD_VPULSE - 1)  << LCD_TIMV_VSW_SHIFT |
            (CONFIG_LPC54_LCD_VFRONTPORCH) << LCD_TIMV_VFP_SHIFT |
            (CONFIG_LPC54_LCD_VBACKPORCH)  << LCD_TIMV_VBP_SHIFT);
  putreg32(regval, LPC54_LCD_TIMV);

  /* Get the pixel clock divider */

#ifdef CONFIG_LPC54_LCD_USE_CLKIN
  pcd = ((uint32_t)CONFIG_LPC54_LCD_CLKIN_FREQUENCY /
         (uint32_t)LPC54_LCD_PIXEL_CLOCK) + 1;
#else
  pcd = ((uint32_t)BOARD_MAIN_CLK / (uint32_t)LPC54_LCD_PIXEL_CLOCK);
#endif

  /* Check the range of pcd */

  bcd = 0;

#ifndef CONFIG_LPC54_LCD_TFTPANEL
  DEBUGASSERT(pcd >= 2);
#else
  if (pcd <= 1)
    {
      /* Just bypass the LCD divider */

      pcd = 0;
      bcd = LCD_POL_BCD;
    }
  else
#endif
    {
      /* The register value is PCD - 2 */

      pcd -= 2;
    }

  /* Initialize clock and signal polarity.
   *
   * REVISIT:  These need to be configurable.
   */

  regval = getreg32(LPC54_LCD_POL);

  /* LCDFP pin is active LOW and inactive HIGH */

  regval |= LCD_POL_IVS;

  /* LCDLP pin is active LOW and inactive HIGH */

  regval |= LCD_POL_IHS;

  /* Data is driven out into the LCD on the falling edge */

  regval &= ~LCD_POL_IPC;

  /* Set number of clocks per line */

  regval &= ~LCD_POL_CPL_MASK;

#if defined(CONFIG_LPC54_LCD_TFTPANEL)
  /* TFT panel */

  regval |= LCD_POL_CPL(CONFIG_LPC54_LCD_HWIDTH - 1);
#else
  /* STN panel */

#if defined(CONFIG_LPC54_LCD_BPP8)
  /* 8-bit monochrome STN panel */

  regval |= LCD_POL_CPL((CONFIG_LPC54_LCD_HWIDTH / 8) - 1);
#elif defined(CONFIG_LPC54_LCD_BPP4)
  /* 4-bit monochrome STN panel */

  regval |= LCD_POL_CPL((CONFIG_LPC54_LCD_HWIDTH / 4) - 1);
#else
  /* Color STN panel. */

  regval |= LCD_POL_CPL(((CONFIG_LPC54_LCD_HWIDTH * 3) / 8) - 1);
#endif
#endif /* CONFIG_LPC54_LCD_TFTPANEL */

  /* Set pixel clock divisor (or bypass) */

  regval &= ~(LCD_POL_PCDLO_MASK | LCD_POL_PCDHI_MASK | LCD_POL_BCD);
  regval |= LCD_POL_PCDLO(pcd) & LCD_POL_PCDLO_MASK;
  regval |= LCD_POL_PCDHI(pcd >> 5) & LCD_POL_PCDHI_MASK;
  regval |= bcd;

  /* LCD_ENAB_M is active high */

  regval &= ~LCD_POL_IOE;
  putreg32(regval, LPC54_LCD_POL);

  /* Frame base address doubleword aligned */

  putreg32(CONFIG_LPC54_LCD_VRAMBASE & ~7, LPC54_LCD_UPBASE);
  putreg32(CONFIG_LPC54_LCD_VRAMBASE & ~7, LPC54_LCD_LPBASE);

  /* Clear the display */

  lpc54_lcdclear(CONFIG_LPC54_LCD_BACKCOLOR);

#ifdef CONFIG_LPC54_LCD_BACKLIGHT
  /* Turn on the back light */

  lpc54_backlight(true);
#endif

  putreg32(0, LPC54_LCD_INTMSK);
  lcdinfo("Enabling the display\n");

  for (i = LPC54_LCD_PWREN_DELAY; i; i--)
    {
    }

  /* Enable LCD */

  regval  = getreg32(LPC54_LCD_CTRL);
  regval |= LCD_CTRL_LCDEN;
  putreg32(regval, LPC54_LCD_CTRL);

  /* Enable LCD power */

  for (i = LPC54_LCD_PWREN_DELAY; i; i--)
    {
    }

  regval  = getreg32(LPC54_LCD_CTRL);
  regval |= LCD_CTRL_LCDPWR;
  putreg32(regval, LPC54_LCD_CTRL);

  return OK;
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane of the specified plane.  Many OSDs support multiple planes of video.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *   vplane - Identifies the plane being queried.
 *
 * Returned Value:
 *   A non-NULL pointer to the frame buffer access structure is returned on
 *   success; NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  lcdinfo("vplane: %d\n", vplane);
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
 * Name: up_fbuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer support for the specified display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_fbuninitialize(int display)
{
  uint32_t regval;
  int i;

  /* We assume there is only one use of the LCD and so we do not need to
   * worry about mutually exclusive access to the LCD hardware.
   */

#ifdef CONFIG_LPC54_LCD_BACKLIGHT
  /* Turn off the back light */

  lpc54_backlight(false);
#endif

  /* Disable the LCD controller */

  regval = getreg32(LPC54_LCD_CTRL);
  regval &= ~LCD_CTRL_LCDPWR;
  putreg32(regval, LPC54_LCD_CTRL);

  for (i = LPC54_LCD_PWRDIS_DELAY; i; i--)
    {
    }

  regval &= ~LCD_CTRL_LCDEN;
  putreg32(regval, LPC54_LCD_CTRL);

  /* Turn off clocking to the LCD. modifyreg32() can do this atomically. */

  putreg32(SYSCON_LCDCLKSEL_NONE, LPC54_SYSCON_LCDCLKSEL);

  /* Disable clocking to the LCD peripheral */

  lpc54_lcd_disableclk();
}

/****************************************************************************
 * Name:  lpc54_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the LPC54xx.  Clearing
 *   the display in the normal way by writing a sequences of runs that
 *   covers the entire display can be slow.  Here the display is cleared by
 *   simply setting all VRAM memory to the specified color.
 *
 ****************************************************************************/

void lpc54_lcdclear(nxgl_mxpixel_t color)
{
  int i;
#if LPC54_BPP > 16
  uint32_t *dest = (uint32_t *)CONFIG_LPC54_LCD_VRAMBASE;

  lcdinfo("Clearing display: color=%08x VRAM=%08x size=%d\n",
          color, CONFIG_LPC54_LCD_VRAMBASE,
          CONFIG_LPC54_LCD_HWIDTH * CONFIG_LPC54_LCD_VHEIGHT * sizeof(uint32_t));

#else
  uint16_t *dest = (uint16_t *)CONFIG_LPC54_LCD_VRAMBASE;

  lcdinfo("Clearing display: color=%08x VRAM=%08x size=%d\n",
          color, CONFIG_LPC54_LCD_VRAMBASE,
          CONFIG_LPC54_LCD_HWIDTH * CONFIG_LPC54_LCD_VHEIGHT * sizeof(uint16_t));
#endif

  for (i = 0; i < (CONFIG_LPC54_LCD_HWIDTH * CONFIG_LPC54_LCD_VHEIGHT); i++)
    {
      *dest++ = color;
    }
}
