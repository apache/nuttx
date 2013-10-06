/****************************************************************************
 * arch/arm/src/sama5/sam_lcd.c
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
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip/sam_syscon.h"
#include "sam_gpio.h"
#include "sam_lcd.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define SAM_LCD_CLK_PER_LINE \
  (CONFIG_SAM_LCD_HWIDTH + CONFIG_SAM_LCD_HPULSE + \
   CONFIG_SAM_LCD_HFRONTPORCH + CONFIG_SAM_LCD_HBACKPORCH)
#define SAM_LCD_LINES_PER_FRAME \
  (CONFIG_SAM_LCD_VHEIGHT + CONFIG_SAM_LCD_VPULSE + \
   CONFIG_SAM_LCD_VFRONTPORCH + CONFIG_SAM_LCD_VBACKPORCH)
#define SAM_LCD_PIXEL_CLOCK \
  (SAM_LCD_CLK_PER_LINE * SAM_LCD_LINES_PER_FRAME * \
   CONFIG_SAM_LCD_REFRESH_FREQ)

/* Framebuffer characteristics in bytes */

#if defined(CONFIG_SAM_LCD_BPP1)
#  define SAM_STRIDE ((CONFIG_SAM_LCD_HWIDTH * 1 + 7) / 8)
#elif defined(CONFIG_SAM_LCD_BPP2)
#  define SAM_STRIDE ((CONFIG_SAM_LCD_HWIDTH * 2 + 7) / 8)
#elif defined(CONFIG_SAM_LCD_BPP4)
#  define SAM_STRIDE ((CONFIG_SAM_LCD_HWIDTH * 4 + 7) / 8)
#elif defined(CONFIG_SAM_LCD_BPP8)
#  define SAM_STRIDE ((CONFIG_SAM_LCD_HWIDTH * 8 + 7) / 8)
#elif defined(CONFIG_SAM_LCD_BPP16)
#  define SAM_STRIDE ((CONFIG_SAM_LCD_HWIDTH * 16 + 7) / 8)
#elif defined(CONFIG_SAM_LCD_BPP24)
#  define SAM_STRIDE ((CONFIG_SAM_LCD_HWIDTH * 32 + 7) / 8)
#elif defined(CONFIG_SAM_LCD_BPP16_565)
#  define SAM_STRIDE ((CONFIG_SAM_LCD_HWIDTH * 16 + 7) / 8)
#else /* defined(CONFIG_SAM_LCD_BPP12_444) */
#  define SAM_STRIDE ((CONFIG_SAM_LCD_HWIDTH * 16 + 7) / 8)
#endif

#define SAM_FBSIZE (SAM_STRIDE * CONFIG_SAM_LCD_VHEIGHT)

/* Delays */

#define SAM_LCD_PWRDIS_DELAY 10000
#define SAM_LCD_PWREN_DELAY  10000

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure provides the overall state of the LCDC */

struct sam_lcdc_s
{
  struct fb_vtable_s fboject;     /* The framebuffer object */

#ifdef CONFIG_FB_HWCURSOR
  struct fb_cursorpos_s cpos;     /* Current cursor position */
#ifdef CONFIG_FB_HWCURSORSIZE
  struct fb_cursorsize_s csize;   /* Current cursor size */
#endif
#endif

  /* Debug stuff */

#ifdef CONFIG_SAMA5_GMAC_REGDEBUG
   bool wrlast;                   /* True: Last access was a write */
   uintptr_t addrlast;            /* Last address accessed */
   uint32_t vallast;              /* Last value read or written */
   int ntimes;                    /* Number of consecutive accesses */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#if defined(CONFIG_SAMA5_LCDC_REGDEBUG) && defined(CONFIG_DEBUG)
static bool sam_checkreg(bool wr, uint32_t regval, uintptr_t address);
static uint32_t sam_getreg(uintptr_t addr);
static void sam_putreg(uintptr_t addr, uint32_t val);
#else
# define sam_getreg(addr)      getreg32(addr)
# define sam_putreg(addr,val)  putreg32(val,addr)
#endif

/* Get information about the video controller configuration and the
 * configuration of each color plane.
 */

static int sam_getvideoinfo(FAR struct fb_vtable_s *vtable,
             FAR struct fb_videoinfo_s *vinfo);
static int sam_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
             FAR struct fb_planeinfo_s *pinfo);

/* The following is provided only if the video hardware supports RGB color
 * mapping
 */

#ifdef CONFIG_FB_CMAP
static int sam_getcmap(FAR struct fb_vtable_s *vtable,
             FAR struct fb_cmap_s *cmap);
static int sam_putcmap(FAR struct fb_vtable_s *vtable,
             FAR const struct fb_cmap_s *cmap);
#endif

/* The following is provided only if the video hardware supports a hardware
 * cursor
 */

#ifdef CONFIG_FB_HWCURSOR
static int sam_getcursor(FAR struct fb_vtable_s *vtable,
             FAR struct fb_cursorattrib_s *attrib);
static int sam_setcursor(FAR struct fb_vtable_s *vtable,
             FAR struct fb_setcursor_s *setttings);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure describes the simulated video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt      = SAM_COLOR_FMT,
  .xres     = CONFIG_SAM_LCD_HWIDTH,
  .yres     = CONFIG_SAM_LCD_VHEIGHT,
  .nplanes  = 1,
};

/* This structure describes the single, simulated color plane */

static const struct fb_planeinfo_s g_planeinfo =
{
  .fbmem    = (FAR void *)CONFIG_SAM_LCD_VRAMBASE,
  .fblen    = SAM_FBSIZE,
  .stride   = SAM_STRIDE,
  .bpp      = SAM_BPP,
};

/* This structure provides the overall state of the LCDC */

struct sam_lcdc_s g_lcdc;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval  - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_LCDC_REGDEBUG
static bool sam_checkreg(bool wr, uint32_t regval, uintptr_t address)
{
  if (wr      == g_lcdc.wrlast &&   /* Same kind of access? */
      regval  == g_lcdc.vallast &&  /* Same value? */
      address == g_lcdc.addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      g_lcdc.ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (g_lcdc.ntimes > 0)
        {
          /* Yes... show how many times we did it */

          lldbg("...[Repeats %d times]...\n", g_lcdc.ntimes);
        }

      /* Save information about the new access */

      g_lcdc.wrlast   = wr;
      g_lcdc.vallast  = regval;
      g_lcdc.addrlast = address;
      g_lcdc.ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: sam_getreg
 *
 * Description:
 *  Read any 32-bit register using an absolute
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_LCDC_REGDEBUG
static uint32_t sam_getreg(uintptr_t address)
{
  uint32_t regval = getreg32(address);

  if (sam_checkreg(false, regval, address))
    {
      lldbg("%08x->%08x\n", address, regval);
    }

  return regval;
}
#endif

/****************************************************************************
 * Name: sam_putreg
 *
 * Description:
 *  Write to any 32-bit register using an absolute address
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_LCDC_REGDEBUG
static void sam_putreg(uintptr_t address, uint32_t regval)
{
  if (sam_checkreg(true, regval, address))
    {
      lldbg("%08x<-%08x\n", address, regval);
    }

  putreg32(regval, address);
}
#endif

/****************************************************************************
 * Name: sam_getvideoinfo
 ****************************************************************************/

static int sam_getvideoinfo(FAR struct fb_vtable_s *vtable,
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
 * Name: sam_getplaneinfo
 ****************************************************************************/

static int sam_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
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
 * Name: sam_getcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int sam_getcmap(FAR struct fb_vtable_s *vtable,
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

  pal  = (uint32_t *)SAM_LCD_PAL(cmap->first >> 1);
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
 * Name: sam_putcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int sam_putcmap(FAR struct fb_vtable_s *vtable,
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

  pal  = (uint32_t *)SAM_LCD_PAL(cmap->first >> 1);
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
 * Name: sam_getcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int sam_getcursor(FAR struct fb_vtable_s *vtable,
                        FAR struct fb_cursorattrib_s *attrib)
{
  gvdbg("vtable=%p attrib=%p\n", vtable, attrib);
  if (vtable && attrib)
    {
#ifdef CONFIG_FB_HWCURSORIMAGE
      attrib->fmt = SAM_COLOR_FMT;
#endif

      gvdbg("pos: (x=%d, y=%d)\n", g_lcdc.cpos.x, g_lcdc.cpos.y);
      attrib->pos = g_lcdc.cpos;

#ifdef CONFIG_FB_HWCURSORSIZE
      attrib->mxsize.h = CONFIG_SAM_LCD_VHEIGHT;
      attrib->mxsize.w = CONFIG_SAM_LCD_HWIDTH;

      gvdbg("size: (h=%d, w=%d)\n", g_lcdc.csize.h, g_lcdc.csize.w);
      attrib->size = g_lcdc.csize;
#endif
      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: sam_setcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int sam_setcursor(FAR struct fb_vtable_s *vtable,
                       FAR struct fb_setcursor_s *setttings)
{
  gvdbg("vtable=%p setttings=%p\n", vtable, setttings);
  if (vtable && setttings)
    {
      gvdbg("flags: %02x\n", settings->flags);
      if ((flags & FB_CUR_SETPOSITION) != 0)
        {
          g_lcdc.cpos = settings->pos;
          gvdbg("pos: (h:%d, w:%d)\n", g_lcdc.cpos.x, g_lcdc.cpos.y);
        }
#ifdef CONFIG_FB_HWCURSORSIZE
      if ((flags & FB_CUR_SETSIZE) != 0)
        {
          g_lcdc.csize = settings->size;
          gvdbg("size: (h:%d, w:%d)\n", g_lcdc.csize.h, g_lcdc.csize.w);
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
 * Name: sam_fbinitialize
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

  /* Configure PIO pins */
  /* Video data */

  gvdbg("Configuring pins\n");

  sam_configgpio(GPIO_LCD_VD0);
  sam_configgpio(GPIO_LCD_VD1);
  sam_configgpio(GPIO_LCD_VD2);
  sam_configgpio(GPIO_LCD_VD3);
  sam_configgpio(GPIO_LCD_VD4);
  sam_configgpio(GPIO_LCD_VD5);
  sam_configgpio(GPIO_LCD_VD6);
  sam_configgpio(GPIO_LCD_VD7);

  sam_configgpio(GPIO_LCD_VD8);
  sam_configgpio(GPIO_LCD_VD9);
  sam_configgpio(GPIO_LCD_VD10);
  sam_configgpio(GPIO_LCD_VD11);
  sam_configgpio(GPIO_LCD_VD12);
  sam_configgpio(GPIO_LCD_VD13);
  sam_configgpio(GPIO_LCD_VD14);
  sam_configgpio(GPIO_LCD_VD15);

#if SAM_BPP > 16
  sam_configgpio(GPIO_LCD_VD16);
  sam_configgpio(GPIO_LCD_VD17);
  sam_configgpio(GPIO_LCD_VD18);
  sam_configgpio(GPIO_LCD_VD19);
  sam_configgpio(GPIO_LCD_VD20);
  sam_configgpio(GPIO_LCD_VD21);
  sam_configgpio(GPIO_LCD_VD22);
  sam_configgpio(GPIO_LCD_VD23);
#endif

  /* Other pins */

  sam_configgpio(GPIO_LCD_DCLK);
  sam_configgpio(GPIO_LCD_LP);
  sam_configgpio(GPIO_LCD_FP);
  sam_configgpio(GPIO_LCD_ENABM);
  sam_configgpio(GPIO_LCD_PWR);

  /* Turn on LCD clock */
#warning Missing lobi

  gvdbg("Configuring the LCD controller\n");

  /* Disable the cursor */
#warning Missing logic

  /* Clear any pending interrupts */
#warning Missing logic

  /* Disable LCDC controller */
#warning Missing logic

  /* Initialize pixel clock */
#warning Missing logic

  /* Set the bits per pixel */
#warning Missing logic

#if defined(CONFIG_SAM_LCD_BPP1)
#  warning Missing logic      /* 1 bpp */
#elif defined(CONFIG_SAM_LCD_BPP2)
#  warning Missing logic      /* 2 bpp */
#elif defined(CONFIG_SAM_LCD_BPP4)
#  warning Missing logic      /* 4 bpp */
#elif defined(CONFIG_SAM_LCD_BPP8)
#  warning Missing logic     /* 8 bpp */
#elif defined(CONFIG_SAM_LCD_BPP16)
#  warning Missing logic     /* 16 bpp */
#elif defined(CONFIG_SAM_LCD_BPP24)
#  warning Missing logic     /* 24-bit TFT panel only */
#elif defined(CONFIG_SAM_LCD_BPP16_565)
#  warning Missing logic    /* 16 bpp, 5:6:5 mode */
#else /* defined(CONFIG_SAM_LCD_BPP12_444) */
#  warning Missing logic    /* 12 bpp, 4:4:4 mode */
#endif

  /* TFT panel */
#warning Missing logic

  /* Select monochrome or color LCD */

#ifdef CONFIG_SAM_LCD_MONOCHROME
#warning Missing logic

#else
  /* Select color LCD */
#warning Missing logic

#endif /* CONFIG_SAM_LCD_MONOCHROME */

  /* Initialize horizontal timing */
#warning Missing logic

  /* Initialize vertical timing */
#warning Missing logic

  /* Initialize clock and signal polarity */
#warning Missing logic

  /* Clear the display */
#warning Missing logic

  sam_lcdclear(CONFIG_SAM_LCD_BACKCOLOR);

#ifdef CONFIG_SAM_LCD_BACKLIGHT
  /* Turn on the back light */

  sam_backlight(true);
#endif

  /* Enable LCD */

  gvdbg("Enabling the display\n");
#warning Missing logic

  /* Enable LCD power */
#warning Missing logic

  return OK;
}

/****************************************************************************
 * Name: sam_fbgetvplane
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
      return &g_lcdc.fboject;
    }
  else
    {
      return NULL;
    }
}

/****************************************************************************
 * Name: sam_fbinitialize
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

#ifdef CONFIG_SAM_LCD_BACKLIGHT
  /* Turn off the back light */

  sam_backlight(false);
#endif

  /* Disable the LCD controller */
#warning Missing logic

  /* Initialize the g_lcdc data structure */

  g_lcdc.fbobject.getvideoinfo  = sam_getvideoinfo;
  g_lcdc.fbobject.getplaneinfo  = sam_getplaneinfo;
#ifdef CONFIG_FB_CMAP
  g_lcdc.fbobject.getcmap       = sam_getcmap;
  g_lcdc.fbobject.putcmap       = sam_putcmap;
#endif
#ifdef CONFIG_FB_HWCURSOR
  g_lcdc.fbobject.getcursor     = sam_getcursor;
  g_lcdc.fbobject.setcursor     = sam_setcursor;
#endif

  /* Turn off clocking to the LCD. modifyreg32() can do this atomically. */

  modifyreg32(SAM_SYSCON_PCONP, SYSCON_PCONP_PCLCD, 0);
  return OK;
}

/************************************************************************************
 * Name:  sam_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the SAMA5.  Clearing the display
 *   in the normal way by writing a sequences of runs that covers the entire display
 *   can be slow.  Here the display is cleared by simply setting all VRAM memory to
 *   the specified color.
 *
 ************************************************************************************/

void sam_lcdclear(nxgl_mxpixel_t color)
{
  int i;
#if SAM_BPP > 16
  uint32_t *dest = (uint32_t*)CONFIG_SAM_LCD_VRAMBASE;

  gvdbg("Clearing display: color=%08x VRAM=%08x size=%d\n",
        color, CONFIG_SAM_LCD_VRAMBASE,
        CONFIG_SAM_LCD_HWIDTH * CONFIG_SAM_LCD_VHEIGHT * sizeof(uint32_t));

#else
  uint16_t *dest = (uint16_t*)CONFIG_SAM_LCD_VRAMBASE;

  gvdbg("Clearing display: color=%08x VRAM=%08x size=%d\n",
        color, CONFIG_SAM_LCD_VRAMBASE,
        CONFIG_SAM_LCD_HWIDTH * CONFIG_SAM_LCD_VHEIGHT * sizeof(uint16_t));
#endif

  for (i = 0; i < (CONFIG_SAM_LCD_HWIDTH * CONFIG_SAM_LCD_VHEIGHT); i++)
    {
      *dest++ = color;
    }
}
