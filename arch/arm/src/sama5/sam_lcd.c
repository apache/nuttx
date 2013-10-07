/****************************************************************************
 * arch/arm/src/sama5/sam_lcd.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 *
 * The Atmel sample code has a BSD compatibile license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2012, Atmel Corporation
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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
/* Configuration ************************************************************/

#ifndef CONFIG_SAMA5_LCDC_DEFBACKLIGHT
#  define CONFIG_SAMA5_LCDC_DEFBACKLIGHT 0xf0
#endif
#define SAMA5_LCDC_BACKLIGHT_OFF 0x00

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
/* This enumeration names each layer supported by the hardware */

enum sam_layer_e
{
  LCDC_BASE = 0,    /* LCD base layer, display fixed size image */
  LCDC_OVR1,        /* LCD Overlay 1 */
  LCDC_OVR2,        /* LCD Overlay 2 */
  LCDC_HEO,         /* LCD HighEndOverlay, support resize */
  LCDC_CUR          /* LCD Cursor, max size 128x128 */
};

/* CLUT information */

struct sam_clutinfo_s
{
  uint8_t bpp;
  uint8_t ncolors;
};

/* LCDC General Layer information */

struct sam_layer_s
{
  /* These is the DMA descriptors as seen by the hardware.  This descriptor
   * must be the first element of the structure and each structure instance
   * must conform to the alignment requirements of the DMA descriptor.
   */

  struct sam_dscr_s dscr;

  /* Driver data that accompanies the descriptor may follow */

  void *buffer;
  struct sam_clutinfo_s clut;
  uint16_t pad;
};

/* LCDC HEO Layer information */

struct sam_heolayer_s
{
  /* These are the HEO DMA descriptors as seen by the hardware.  This array
   * must be the first element of the structure and each structure instance
   * must conform to the alignment requirements of the DMA descriptor.
   */

  struct sam_dscr_s dscr[3];

  /* Driver data that accompanies the descriptor may follow */

  void *buffer;
  struct sam_clutinfo_s clut;
  uint16_t pad;
};

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

/* Frame buffer interface ***************************************************/
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

/* Initialization ***********************************************************/

static void sam_pio_config(void);
static void sam_backlight(uint32_t level);
static void sam_base_disable(void);
static void sam_ovr1_disable(void);
static void sam_ovr2_disable(void);
static void sam_heo_disable(void);
static void sam_hcr_disable(void);
static void sam_lcd_disable(void);
static void sam_layer_config(void);
static void sam_lcd_enable(void);

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

/* Preallocated LCDC layer structures */

/* Base layer */

static struct sam_layer_s g_baselayer __attribute__((aligned(64)))

/* Overlay 1/2 Layers */

static struct sam_layer_s g_ovr1layer __attribute__((aligned(64)));
static struct sam_layer_s g_ovr2layer __attribute__((aligned(64)));

/* High End Overlay (HEO) Layer */

static struct sam_heolayer_s g_heolayer __attribute__((aligned(64)));

/* Hardware cursor (HRC) Layer */

static struct sam_layer_s g_hcrlayer __attribute__((aligned(64)));

/* PIO pin configurations */

static pio_pinset_t g_lcdcpins[] =
{
  PIO_LCD_DAT0,  PIO_LCD_DAT2,  PIO_LCD_DAT1,  PIO_LCD_DAT3,
  PIO_LCD_DAT4,  PIO_LCD_DAT5,  PIO_LCD_DAT6,  PIO_LCD_DAT7,

  PIO_LCD_DAT8,  PIO_LCD_DAT9,  PIO_LCD_DAT10, PIO_LCD_DAT11,
  PIO_LCD_DAT12, PIO_LCD_DAT13, PIO_LCD_DAT14, PIO_LCD_DAT15,

#if SAM_BPP > 16
  PIO_LCD_DAT16, PIO_LCD_DAT17, PIO_LCD_DAT18, PIO_LCD_DAT19,
  PIO_LCD_DAT20, PIO_LCD_DAT21, PIO_LCD_DAT22, PIO_LCD_DAT23,
#endif

  PIO_LCD_PWM,   PIO_LCD_DISP,  PIO_LCD_VSYNC, PIO_LCD_HSYNC,
  PIO_LCD_PCK,   PIO_LCD_DEN
}
#define SAMA5_LCDC_NPINCONFIGS (sizeof(g_lcdcpins) / sizeof(pio_pinset_t))

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
 * Name: sam_pio_config
 *
 * Description:
 *   Configure PIO pins for use with the LCDC
 *
 ****************************************************************************/

static void sam_pio_config(void)
{
  int i;

  gvdbg("Configuring pins\n");

  /* Configure each pin */

  for (i = 0; i < SAMA5_LCDC_NPINCONFIGS; i++)
    {
      sam_configpio(g_lcdcpins[i]);
    }
}

/****************************************************************************
 * Name: sam_backlight
 *
 * Description:
 *   Set the backlight level
 *
 ****************************************************************************/

static void sam_backlight(uint32_t level)
{
  uint32_t regval;

  /* Are we turning the backlight off? */

  if (level == SAMA5_LCDC_BACKLIGHT_OFF)
    {
      /* Disable the backlight */

      sam_putreg(SAM_LCDC_LCDDIS, LCDC_LCDDIS_PWM);

      /* And wait for the PWM to be disabled */

      while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_PWM) != 0);
    }
#ifdef CONFIG_SAM_LCD_BACKLIGHT
  else
    {
      /* Set the backight level */

      regval = sam_getreg(SAM_LCDC_LCDCFG6);
      regval &= ~LCDC_LCDCFG6_PWMCVAL_MASK;
      regval |=  LCDC_LCDCFG6_PWMCVAL(level);
      sam_putreg(SAM_LCDC_LCDCFG6, regval);

      /* Enable the backlight */

      sam_putreg(SAM_LCDC_LCDEN, LCDC_LCDEN_PWM);
    }
#endif
}

/****************************************************************************
 * Name: sam_base_disable
 *
 * Description:
 *   Disable the base layer
 *
 ****************************************************************************/

static void sam_base_disable(void)
{
  uintptr_t physaddr;
  uintptr_t dscr;
  uint32_t regval;

  /* 1. Clear the DFETCH bit in the DSCR.CHXCTRL field of the DSCR structure
   *    will disable the channel at the end of the frame.
   */

  g_baselayer.dscr.ctrl &= ~LCDC_BASECTRL_DFETCH;

  regval = sam_getreg(SAM_LCDC_BASECTRL);
  reval &= ~LCDC_BASECTRL_DFETCH;
  putreg(SAM_LCDC_BASECTRL, regval);

  /* 2. Set the DSCR.CHXNEXT field of the DSCR structure will disable the
   *    channel at the end of the frame.
   */

  dscr                  = (uintptr_t)&g_baselayer.dscr
  physaddr              = sam_physramaddr(dscr);
  g_baselayer.dscr.next = physaddr;

  putreg(SAM_LCDC_BASENEXT, physaddr);

  /* Flush the modified DMA descriptor to RAM */

  cp15_clean_dcache(dscr, dscr + sizeof(struct sam_dscr_s));

  /* 3. Writing one to the CHDIS field of the CHXCHDR register will disable
   *    the channel at the end of the frame.
   */

  sam_putreg(SAM_LCDC_BASECHDR, LCDC_BASECHDR_CH);

  /* 4. Writing one to the CHRST field of the CHXCHDR register will disable
   *    the channel immediately. This may occur in the middle of the image.
   */

  /* 5. Poll CHSR field in the CHXCHSR register until the channel is
   *    successfully disabled.
   */

  while ((sam_getreg(SAM_LCDC_BASECHSR & LCDC_BASECHSR_CH) != 0);
}

/****************************************************************************
 * Name: sam_ovr1_disable
 *
 * Description:
 *   Disable the overlay 1 layer
 *
 ****************************************************************************/

static void sam_ovr1_disable(void)
{
  uintptr_t physaddr;
  uintptr_t dscr;
  uint32_t regval;

  /* 1. Clear the DFETCH bit in the DSCR.CHXCTRL field of the DSCR structure
   *    will disable the channel at the end of the frame.
   */

  g_ovr1layer.dscr.ctrl &= ~LCDC_OVR1CTRL_DFETCH;

  regval = sam_getreg(SAM_LCDC_OVR1CTRL);
  reval &= ~LCDC_OVR1CTRL_DFETCH;
  putreg(SAM_LCDC_OVR1CTRL, regval);

  /* 2. Set the DSCR.CHXNEXT field of the DSCR structure will disable the
   *    channel at the end of the frame.
   */

  dscr                  = (uintptr_t)&g_ovr1layer.dscr
  physaddr              = sam_physramaddr(dscr);
  g_ovr1layer.dscr.next = physaddr;

  putreg(SAM_LCDC_OVR1NEXT, physaddr);

  /* Flush the modified DMA descriptor to RAM */

  cp15_clean_dcache(dscr, dscr + sizeof(struct sam_dscr_s));

  /* 3. Writing one to the CHDIS field of the CHXCHDR register will disable
   *    the channel at the end of the frame.
   */

  sam_putreg(SAM_LCDC_OVR1CHDR, LCDC_OVR1CHDR_CH);

  /* 4. Writing one to the CHRST field of the CHXCHDR register will disable
   *    the channel immediately. This may occur in the middle of the image.
   */

  /* 5. Poll CHSR field in the CHXCHSR register until the channel is
   *    successfully disabled.
   */

  while ((sam_getreg(SAM_LCDC_OVR1CHSR & LCDC_OVR1CHSR_CH) != 0);
}

/****************************************************************************
 * Name: sam_ovr2_disable
 *
 * Description:
 *   Disable the overlay 2 layer
 *
 ****************************************************************************/

static void sam_ovr2_disable(void)
{
  uintptr_t physaddr;
  uintptr_t dscr;
  uint32_t regval;

  /* 1. Clear the DFETCH bit in the DSCR.CHXCTRL field of the DSCR structure
   *    will disable the channel at the end of the frame.
   */

  g_ovr2layer.dscr.ctrl &= ~LCDC_OVR2CTRL_DFETCH;

  regval = sam_getreg(SAM_LCDC_OVR2CTRL);
  reval &= ~LCDC_OVR2CTRL_DFETCH;
  putreg(SAM_LCDC_OVR2CTRL, regval);

  /* 2. Set the DSCR.CHXNEXT field of the DSCR structure will disable the
   *    channel at the end of the frame.
   */

  dscr                  = (uintptr_t)&g_ovr1layer.dscr
  physaddr              = sam_physramaddr(dscr);
  g_ovr2layer.dscr.next = physaddr;

  putreg(SAM_LCDC_OVR2NEXT, physaddr);

  /* Flush the modified DMA descriptor to RAM */

  cp15_clean_dcache(dscr, dscr + sizeof(struct sam_dscr_s));

  /* 3. Writing one to the CHDIS field of the CHXCHDR register will disable
   *    the channel at the end of the frame.
   */

  sam_putreg(SAM_LCDC_OVR2CHDR, LCDC_OVR2CHDR_CH);

  /* 4. Writing one to the CHRST field of the CHXCHDR register will disable
   *    the channel immediately. This may occur in the middle of the image.
   */

  /* 5. Poll CHSR field in the CHXCHSR register until the channel is
   *    successfully disabled.
   */

  while ((sam_getreg(SAM_LCDC_OVR2CHSR & LCDC_OVR2CHSR_CH) != 0);
}

/****************************************************************************
 * Name: sam_heo_disable
 *
 * Description:
 *   Disable the High End Overlay (HEO) layer
 *
 ****************************************************************************/

static void sam_heo_disable(void)
{
  uintptr_t physaddr;
  uintptr_t dscr;
  uint32_t regval;

  /* 1. Clear the DFETCH bit in the DSCR.CHXCTRL field of the DSCR structure
   *    will disable the channel at the end of the frame.
   */

  g_heolayer.dscr[0].ctrl &= ~LCDC_HEOCTRL_DFETCH;
  g_heolayer.dscr[1].ctrl &= ~LCDC_HEOUCTRL_DFETCH;
  g_heolayer.dscr[2].ctrl &= ~LCDC_HEOVCTRL_DFETCH;

  regval = sam_getreg(SAM_LCDC_HEOCTRL);
  reval &= ~LCDC_HEOCTRL_DFETCH;
  putreg(SAM_LCDC_HEOCTRL, regval);

  regval = sam_getreg(SAM_LCDC_HEOUCTRL);
  reval &= ~LCDC_HEOUCTRL_DFETCH;
  putreg(SAM_LCDC_HEOUCTRL, regval);

  regval = sam_getreg(SAM_LCDC_HEOVCTRL);
  reval &= ~LCDC_HEOVCTRL_DFETCH;
  putreg(SAM_LCDC_HEOVCTRL, regval);

  /* 2. Set the DSCR.CHXNEXT field of the DSCR structure will disable the
   *    channel at the end of the frame.
   */

  physaddr = sam_physramaddr((uintptr_t)&g_heolayer.dscr[0]);
  g_heolayer.dscr[0].next = physaddr;
  putreg(SAM_LCDC_HEONEXT, physaddr);

  physaddr = sam_physramaddr((uintptr_t)&g_heolayer.dscr[1]);
  g_heolayer.dscr[1].next = physaddr;
  putreg(SAM_LCDC_HEOUNEXT, physaddr);

  physaddr = sam_physramaddr((uintptr_t)&g_heolayer.dscr[2]);
  g_heolayer.dscr[2].next = physaddr;
  putreg(SAM_LCDC_HEOVNEXT, physaddr);

  /* Flush the modified DMA descriptors to RAM */

  dscr = sam_physramaddr((uintptr_t)g_heolayer.dscr);
  cp15_clean_dcache(dscr, dscr + sizeof(struct sam_dscr_s));

  /* 3. Writing one to the CHDIS field of the CHXCHDR register will disable
   *    the channel at the end of the frame.
   */

  sam_putreg(SAM_LCDC_HEOCHDR, LCDC_HEOCHDR_CH);

  /* 4. Writing one to the CHRST field of the CHXCHDR register will disable
   *    the channel immediately. This may occur in the middle of the image.
   */

  /* 5. Poll CHSR field in the CHXCHSR register until the channel is
   *    successfully disabled.
   */

  while ((sam_getreg(SAM_LCDC_HEOCHSR & LCDC_HEOCHSR_CH) != 0);
}

/****************************************************************************
 * Name: sam_hcr_disable
 *
 * Description:
 *   Disable the Hardware Cursor Channel (HCR) layer
 *
 ****************************************************************************/

static void sam_hcr_disable(void)
{
  uintptr_t physaddr;
  uintptr_t dscr;
  uint32_t regval;

  /* 1. Clear the DFETCH bit in the DSCR.CHXCTRL field of the DSCR structure
   *    will disable the channel at the end of the frame.
   */

  g_hcrlayer.dscr.ctrl &= ~LCDC_HCRCTRL_DFETCH;

  regval = sam_getreg(SAM_LCDC_HCRCTRL);
  reval &= ~LCDC_HCRCTRL_DFETCH;
  putreg(SAM_LCDC_HCRCTRL, regval);

  /* 2. Set the DSCR.CHXNEXT field of the DSCR structure will disable the
   *    channel at the end of the frame.
   */

  dscr                 = (uintptr_t)&g_hcrlayer.dscr
  physaddr             = sam_physramaddr(dscr);
  g_hcrlayer.dscr.next = physaddr;

  putreg(SAM_LCDC_HCRNEXT, physaddr);

  /* Flush the modified DMA descriptor to RAM */

  cp15_clean_dcache(dscr, dscr + sizeof(struct sam_dscr_s));

  /* 3. Writing one to the CHDIS field of the CHXCHDR register will disable
   *    the channel at the end of the frame.
   */

  sam_putreg(SAM_LCDC_HCRCHDR, LCDC_HCRCHDR_CH);

  /* 4. Writing one to the CHRST field of the CHXCHDR register will disable
   *    the channel immediately. This may occur in the middle of the image.
   */

  /* 5. Poll CHSR field in the CHXCHSR register until the channel is
   *    successfully disabled.
   */

  while ((sam_getreg(SAM_LCDC_HCRCHSR & LCDC_HCRCHSR_CH) != 0);
}

/****************************************************************************
 * Name: sam_lcd_disable
 *
 * Description:
 *   Disable the LCD peripheral
 *
 ****************************************************************************/

static void sam_lcd_disable(void)
{
  Lcdc *pHw  = LCDC;
  Pmc  *pPmc = PMC;

  /* Disable layers */

  sam_base_disable();
  sam_ovr1_disable();
  sam_ovr2_disable();
  sam_heo_disable();
  sam_hcr_disable();

  /* Disable DMA path */

  sam_putreg(SAM_LCDC_BASECFG4, 0);

  /* Turn off the back light */

  sam_backlight(SAMA5_LCDC_BACKLIGHT_OFF);

  /* Timing Engine Power Down Software Operation */

  /* 1. Disable the DISP signal writing DISPDIS field of the LCDC_LCDDIS
   *    register.
   */

  sam_putreg(SAM_LCDC_LCDDIS, LCDC_LCDDIS_DISP);

  /* 2. Poll DISPSTS field of the LCDC_LCDSR register to verify that the DISP
   *    is no longer activated.
   */

  while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_DISP) != 0);

  /* 3. Disable the hsync and vsync signals by writing one to SYNCDIS field of
   *    the LCDC_LCDDIS register.
   */

  sam_putreg(SAM_LCDC_LCDDIS, LCDC_LCDDIS_SYNC);

  /* 4. Poll LCDSTS field of the LCDC_LCDSR register to check that the
   *    synchronization is off.
   */

  while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_LCD) != 0);

  /* 5. Disable the Pixel clock by writing one in the CLKDIS field of the
   *    LCDC_LCDDIS register.
   */

  sam_putreg(SAM_LCDC_LCDDIS, LCDC_LCDDIS_CLK);

  /* 6. Poll CLKSTS field of the LCDC_CLKSR register to check that Pixel Clock
   *    is disabled.
   */

  while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_CLK) != 0);

  /* Disable peripheral clock */

  sam_lcdc_disableclk();

  /* Disable the LCD clock */

  sam_putreg(SAM_PMC_SCDR, PMC_LCDCK);
}

/****************************************************************************
 * Name: sam_layer_config
 *
 * Description:
 *   Configure LCDC layers
 *
 ****************************************************************************/

static void sam_layer_config(void)
{
  /* Base channel */

#ifdef CONFIG_SAMA5_LCDC_BASE_RGB888P
  sam_putreg(SAM_LCDC_BASECFG0,
             LCDC_BASECFG0_DLBO | LCDC_BASECFG0_BLEN_INCR16);
  sam_putreg(SAM_LCDC_BASECFG1,
             LCDC_BASECFG1_24BPP_RGB888P);
#else
# error Support for this resolution is not yet supported
#endif

#ifdef CONFIG_SAMA5_LCDC_OVR1
#  ifdef CONFIG_SAMA5_LCDC_OVR1_RGB888P
  /* Overlay 1, GA 0xff */

  sam_putreg(SAM_LCDC_OVR1CFG0,
             LCDC_OVR1CFG0_DLBO | LCDC_OVR1CFG0_BLEN_INCR16 |
             LCDC_OVR1CFG0_ROTDIS);
  sam_putreg(SAM_LCDC_OVR1CFG1,
             LCDC_OVR1CFG1_24BPP_RGB888P);
  sam_putreg(SAM_LCDC_OVR1CFG9,
             LCDC_OVR1CFG9_GA(0xff) | LCDC_OVR1CFG9_GAEN);
#  else
#   error Support for this resolution is not yet supported
#  endif
#endif

#ifdef CONFIG_SAMA5_LCDC_OVR2
#  ifdef CONFIG_SAMA5_LCDC_OVR2_RGB888P
  /* Overlay 2, GA 0xff */

  sam_putreg(SAM_LCDC_OVR2CFG0,
             LCDC_OVR2CFG0_DLBO | LCDC_OVR2CFG0_BLEN_INCR16 |
             LCDC_OVR2CFG0_ROTDIS;
  sam_putreg(SAM_LCDC_OVR2CFG1,
             LCDC_OVR2CFG1_24BPP_RGB888P);
  sam_putreg(SAM_LCDC_OVR2CFG9,
             LCDC_OVR2CFG9_GA(0xff) | LCDC_OVR2CFG9_GAEN;
#  else
#   error Support for this resolution is not yet supported
#  endif
#endif

#ifdef CONFIG_SAMA5_LCDC_OVR2
#  ifdef CONFIG_SAMA5_LCDC_HEO_RGB888P
  /* High End Overlay, GA 0xff */

  sam_putreg(SAM_LCDC_HEOCFG0,
             LCDC_HEOCFG0_DLBO | LCDC_HEOCFG0_BLEN_INCR16 |
             LCDC_HEOCFG0_ROTDIS);
  sam_putreg(SAM_LCDC_HEOCFG1,
             LCDC_HEOCFG1_24BPP_RGB888P);
  sam_putreg(SAM_LCDC_HEOCFG12,
             LCDC_HEOCFG12_GA(0xff) | LCDC_HEOCFG12_GAEN);
#  else
#   error Support for this resolution is not yet supported
#  endif
#endif

#ifdef CONFIG_FB_HWCURSOR
#  ifdef CONFIG_SAMA5_LCDC_HEO_RGB888P
  /* Hardware Cursor, GA 0xff, Key #000000 */

  sam_putreg(SAM_LCDC_HCRCFG0,
             LCDC_HCRCFG0_DLBO | LCDC_HCRCFG0_BLEN_INCR16);
  sam_putreg(SAM_LCDC_HCRCFG1,
             LCDC_HCRCFG1_24BPP_RGB888P;
  sam_putreg(SAM_LCDC_HCRCFG7,
             0x000000);
  sam_putreg(SAM_LCDC_HCRCFG8,
             0xffffff);
  sam_putreg(SAM_LCDC_HCRCFG9,
             LCDC_HCRCFG9_GAEN(0xff) | LCDC_HCRCFG9_GAEN);
#  else
#   error Support for this resolution is not yet supported
#  endif
#endif
}

/****************************************************************************
 * Name: sam_lcd_enable
 *
 * Description:
 *   Enable the LCD for normal use
 *
 ****************************************************************************/

static void sam_lcd_enable(void)
{
  uint32_t regval;
  uint32_t div;

  /* Enable the LCD peripheral clock */

  sam_lcdc_enableclk();

  /* Enable the LCD clock */

  sam_putreg(SAM_PMC_SCER, PMC_LCDCK);

  /* 1. Configure LCD timing parameters, signal polarity and clock period. */

  div = ((2 * BOARD_MCK_FREQUENCY) / BOARD_LCD_PIXELCLOCK) - 2;
  regval = LCDC_LCDCFG0_CLKDIV(div) | LCDC_LCDCFG0_CGDISHCR |
           LCDC_LCDCFG0_CGDISHEO | LCDC_LCDCFG0_CGDISOVR1 |
           LCDC_LCDCFG0_CGDISOVR2 | LCDC_LCDCFG0_CGDISBASE |
           LCDC_LCDCFG0_CLKPWMSEL | LCDC_LCDCFG0_CLKSEL |
           LCDC_LCDCFG0_CLKPOL;
  sam_putreg(SAM_LCDC_LCDCFG0, regval);

  regval = LCDC_LCDCFG1_VSPW(BOARD_LCD_TIMING_VPW - 1) |
           LCDC_LCDCFG1_HSPW(BOARD_LCD_TIMING_HPW - 1);
  sam_putreg(SAM_LCDC_LCDCFG1, regval);

  regval = LCDC_LCDCFG2_VBPW(BOARD_LCD_TIMING_VBP) |
           LCDC_LCDCFG2_VFPW(BOARD_LCD_TIMING_VFP - 1);
  sam_putreg(SAM_LCDC_LCDCFG2, regval);

  regval = LCDC_LCDCFG3_HBPW(BOARD_LCD_TIMING_HBP - 1) |
           LCDC_LCDCFG3_HFPW(BOARD_LCD_TIMING_HFP - 1);
  sam_putreg(SAM_LCDC_LCDCFG3, regval);

  regval = LCDC_LCDCFG4_RPF(BOARD_LCD_HEIGHT - 1) |
           LCDC_LCDCFG4_PPL(BOARD_LCD_WIDTH - 1);
  sam_putreg(SAM_LCDC_LCDCFG4, regval);

  regval = LCDC_LCDCFG5_GUARDTIME(30) | LCDC_LCDCFG5_MODE_OUTPUT_24BPP |
           LCDC_LCDCFG5_DISPDLY | LCDC_LCDCFG5_VSPDLYS | LCDC_LCDCFG5_VSPOL |
           LCDC_LCDCFG5_HSPOL;
  sam_putreg(SAM_LCDC_LCDCFG5, regval);

  regval = LCDC_LCDCFG6_PWMCVAL(0xf0) | LCDC_LCDCFG6_PWMPOL |
           LCDC_LCDCFG6_PWMPS(6);
  sam_putreg(SAM_LCDC_LCDCFG6, regval);

  /* 2. Enable the Pixel Clock by writing one to the CLKEN field of the
        LCDC_LCDEN register. */

  sam_putreg(SAM_LCDC_LCDEN, LCDC_LCDEN_CLK);

  /* 3. Poll CLKSTS field of the LCDC_LCDSR register to check that the clock
   * is running.
   */

  while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_CLK) == 0);

  /* 4. Enable Horizontal and Vertical Synchronization by writing one to the
   *    SYNCEN field of the LCDC_LCDEN register.
   */

  sam_putreg(SAM_LCDC_LCDEN, LCDC_LCDEN_SYNC);

  /* 5. Poll LCDSTS field of the LCDC_LCDSR register to check that the
   *    synchronization is up.
   */

  while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_LCD) == 0);

  /* 6. Enable the display power signal writing one to the DISPEN field of the
   *    LCDC_LCDEN register.
   */

  sam_putreg(LCDC_LCDEN, LCDC_LCDEN_DISP);

  /* 7. Poll DISPSTS field of the LCDC_LCDSR register to check that the power
   *    signal is activated.
   */

  while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_DISP) == 0);
}

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

  sam_pio_config();

  /* Reset layer information */

  memset(&g_baselayer, 0, sizeof(struct sam_layer_s));
  memset(&g_ovr1layer, 0, sizeof(struct sam_layer_s));
  memset(&g_ovr2layer, 0, sizeof(struct sam_layer_s));
  memset(&g_heolayer,  0, sizeof(struct sam_heolayer_s));
  memset(&g_hcrlayer,  0, sizeof(struct sam_layer_s));

  /* Disable the LCD */

  sam_lcd_disable();

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

  gvdbg("Configuring the LCD controller\n");

  /* Enable the LCD peripheral clock */

  sam_lcdc_enableclk();

  /* Enable the LCD clock */

  sam_putreg(SAM_PMC_SCER, PMC_LCDCK);

  /* Disable LCD interrupts */

  sam_putreg(SAM_LCDC_LCDIDR, LCDC_LCDINT_ALL);

  /* Configure layers */

  sam_layer_config();

  /* Clear the display memory */

  sam_lcdclear(CONFIG_SAM_LCD_BACKCOLOR);

  /* And turn the LCD on */

  gvdbg("Enabling the display\n");
  sam_lcd_enable();

  /* Show the base layer */
#warning Missing logic

  /* Enable the backlight.
   *
   * REVISIT:  Backlight level could be dynamically adjustable
   */

  sam_backlight(CONFIG_SAMA5_LCDC_DEFBACKLIGHT);

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

  /* We assume there is only one user of the LCD and so we do not need to
   * worry about mutually exclusive access to the LCD hardware.
   */

  /* Turn off the back light */

  sam_backlight(SAMA5_LCDC_BACKLIGHT_OFF);

  /* Disable the LCD controller */
#warning Missing logic

  /* Turn off clocking to the LCD. */

  sam_lcdc_disableclks();
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
