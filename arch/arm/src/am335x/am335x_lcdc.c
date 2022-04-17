/****************************************************************************
 * arch/arm/src/am335x/am335x_lcdc.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This driver derives from the LPC54xx LCD driver but also includes
 * information from the FreeBSD AM335x LCD driver which was released under
 * a two-clause BSD license:
 *
 *   Copyright 2013 Oleksandr Tymoshenko <gonzo@freebsd.org>
 *   All rights reserved.
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/video/fb.h>

#include "arm_internal.h"
#include "hardware/am335x_prcm.h"
#include "am335x_pinmux.h"
#include "am335x_config.h"
#include "am335x_gpio.h"
#include "am335x_sysclk.h"
#include "am335x_lcdc.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DPLL_MAX_MUL            0x800
#define DPLL_MAX_DIV            0x80

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Get information about the video controller configuration and the
 * configuration of each color plane.
 */

static int am335x_getvideoinfo(struct fb_vtable_s *vtable,
                               struct fb_videoinfo_s *vinfo);
static int am335x_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                               struct fb_planeinfo_s *pinfo);

/* The following is provided only if the video hardware supports RGB color
 * mapping
 */

#ifdef CONFIG_FB_CMAP
static int am335x_getcmap(struct fb_vtable_s *vtable,
                          struct fb_cmap_s *cmap);
static int am335x_putcmap(struct fb_vtable_s *vtable,
                          const struct fb_cmap_s *cmap);
#endif

/* The following is provided only if the video hardware supports a hardware
 * cursor
 */

#ifdef CONFIG_FB_HWCURSOR
static int am335x_getcursor(struct fb_vtable_s *vtable,
                            struct fb_cursorattrib_s *attrib);
static int am335x_setcursor(struct fb_vtable_s *vtable,
                            struct fb_setcursor_s *settings);
#endif

/* Miscellaneous internal functions */

static int am335x_lcd_interrupt(int irq, void *context, void *arg);
static uint32_t am335x_lcd_divisor(uint32_t reference, uint32_t frequency);
static int am335x_set_refclk(uint32_t frequency);
static int am335x_get_refclk(uint32_t *frequency);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The framebuffer object -- There is no private state information in this
 * framebuffer driver.
 */

struct fb_vtable_s g_fbinterface =
{
  .getvideoinfo  = am335x_getvideoinfo,
  .getplaneinfo  = am335x_getplaneinfo,
#ifdef CONFIG_FB_CMAP
  .getcmap       = am335x_getcmap,
  .putcmap       = am335x_putcmap,
#endif
#ifdef CONFIG_FB_HWCURSOR
  .getcursor     = am335x_getcursor,
  .setcursor     = am335x_setcursor,
#endif
};

struct am335x_lcd_dev_s
{
  /* Saved panel configuration */

  struct am335x_panel_info_s panel;

  sem_t exclsem;        /* Assure mutually exclusive access */
  nxgl_coord_t stride;  /* Width of framebuffer in bytes */
  size_t fbsize;        /* Size of the framebuffer allocation */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Since there is only a single LCD controller, its state structure can be
 * allocated in .bss.
 */

static struct am335x_lcd_dev_s g_lcddev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_getvideoinfo
 ****************************************************************************/

static int am335x_getvideoinfo(struct fb_vtable_s *vtable,
                               struct fb_videoinfo_s *vinfo)
{
  struct am335x_lcd_dev_s *priv = &g_lcddev;

  lcdinfo("vtable=%p vinfo=%p\n", vtable, vinfo);
  if (vtable && vinfo)
    {
      vinfo->fmt       = AM335X_COLOR_FMT;    /* REVISIT */
      vinfo->xres      = priv->panel.width;
      vinfo->yres      = priv->panel.height;
      vinfo->nplanes   = 1;
#ifdef CONFIG_FB_OVERLAY
      vinfo->noverlays = 0;
#endif
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: am335x_getplaneinfo
 ****************************************************************************/

static int am335x_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                               struct fb_planeinfo_s *pinfo)
{
  struct am335x_lcd_dev_s *priv = &g_lcddev;

  lcdinfo("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);

  if (vtable != NULL && planeno == 0 && pinfo != NULL)
    {
#ifdef CONFIG_BUILD_KERNEL
      pinfo->fbmem   = (void *)CONFIG_AM335X_LCDC_FB_PBASE;
#else
      pinfo->fbmem   = (void *)CONFIG_AM335X_LCDC_FB_VBASE;
#endif
      pinfo->fblen   = priv->fbsize;
      pinfo->stride  = priv->stride;
      pinfo->display = 0;
      pinfo->bpp     = priv->panel.bpp;
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: am335x_getcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int am335x_getcmap(struct fb_vtable_s *vtable,
                          struct fb_cmap_s *cmap)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: am335x_putcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int am335x_putcmap(struct fb_vtable_s *vtable,
                          const struct fb_cmap_s *cmap)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: am335x_getcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int am335x_getcursor(struct fb_vtable_s *vtable,
                            struct fb_cursorattrib_s *attrib)
{
  lcdinfo("vtable=%p attrib=%p\n", vtable, attrib);
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: am335x_setcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int am335x_setcursor(struct fb_vtable_s *vtable,
                            struct fb_setcursor_s *settings)
{
  lcdinfo("vtable=%p settings=%p\n", vtable, settings);
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: am335x_lcd_interrupt
 ****************************************************************************/

static int am335x_lcd_interrupt(int irq, void *context, void *arg)
{
  struct am335x_lcd_dev_s *priv = (struct am335x_lcd_dev_s *)arg;
  uint32_t regval;

  regval = getreg32(AM335X_LCD_IRQ_STAT);
  putreg32(AM335X_LCD_IRQ_STAT, regval);

  /* Read value back to make sure it reached the hardware */

  regval = getreg32(AM335X_LCD_IRQ_STAT);
  if ((regval & LCD_IRQ_SYNC) != 0 || (regval & LCD_IRQ_PL) != 0)
    {
      regval = getreg32(AM335X_LCD_RASTER_CTRL);
      regval &= ~LCD_RASTER_CTRL_LCD_EN;
      putreg32(AM335X_LCD_RASTER_CTRL, regval);

      regval = getreg32(AM335X_LCD_RASTER_CTRL);
      regval |= LCD_RASTER_CTRL_LCD_EN;
      putreg32(AM335X_LCD_RASTER_CTRL, regval);
      goto done;
    }

  if ((regval & LCD_IRQ_EOF0) != 0)
    {
      putreg32(AM335X_LCD_DMA_FB0_BASE, CONFIG_AM335X_LCDC_FB_PBASE);
      putreg32(AM335X_LCD_DMA_FB0_CEIL,
               CONFIG_AM335X_LCDC_FB_PBASE + priv->fbsize - 1);
      regval &= ~LCD_IRQ_EOF0;
    }

  if ((regval & LCD_IRQ_EOF1) != 0)
    {
      putreg32(AM335X_LCD_DMA_FB1_BASE, CONFIG_AM335X_LCDC_FB_PBASE);
      putreg32(AM335X_LCD_DMA_FB1_CEIL,
               CONFIG_AM335X_LCDC_FB_PBASE + priv->fbsize - 1);
      regval &= ~LCD_IRQ_EOF1;
    }

  if ((regval & LCD_IRQ_FUF) != 0)
    {
      /* TODO: Handle FUF */

      regval &= ~LCD_IRQ_FUF;
    }

  if ((regval & LCD_IRQ_ACB) != 0)
    {
      /* TODO: Handle ACB */

      regval &= ~LCD_IRQ_ACB;
    }

done:
  putreg32(AM335X_LCD_END_INT, 0);

  /* Read value back to make sure it reached the hardware */

  regval = getreg32(AM335X_LCD_END_INT);
  return OK;
}

/****************************************************************************
 * Name: am335x_lcd_divisor
 ****************************************************************************/

static uint32_t am335x_lcd_divisor(uint32_t reference, uint32_t frequency)
{
  uint32_t div;
  uint32_t delta;
  uint32_t mindelta;
  int i;

  mindelta = frequency;
  div      = 255;

  /* Raster mode case: divisors are in range from 2 to 255 */

  for (i = 2; i < 255; i++)
    {
      delta = reference / i - frequency;
      if (delta < 0)
        {
          delta    = -delta;
        }

      if (delta < mindelta)
        {
          div      = i;
          mindelta = delta;
        }
    }

  return div;
}

/****************************************************************************
 * Name: am335x_set_refclk
 ****************************************************************************/

static int am335x_set_refclk(uint32_t frequency)
{
  uint32_t sysclk;
  uint32_t mul;
  uint32_t div;
  uint32_t delta;
  uint32_t mindelta;
  int timeout;
  int i;
  int j;

  sysclk = am335x_get_sysclk();

  /* Bypass mode */

  putreg32(AM335X_CM_WKUP_CLKMODE_DPLL_DISP, 0x4);

  /* Make sure it's in bypass mode */

  while ((getreg32(AM335X_CM_WKUP_IDLEST_DPLL_DISP) & (1 << 8)) == 0)
    {
      up_udelay(10);
    }

  /* Dumb and non-optimal implementation */

  mul      = 0;
  div      = 0;
  mindelta = frequency;

  for (i = 1; i < DPLL_MAX_MUL; i++)
    {
      for (j = 1; j < DPLL_MAX_DIV; j++)
        {
          delta = frequency - i * (sysclk / j);
          if (delta < 0)
            {
              delta    = -delta;
            }

          if (delta < mindelta)
            {
              mul      = i;
              div      = j;
              mindelta = delta;
            }

          if (mindelta == 0)
            {
              break;
            }
        }
    }

  putreg32(AM335X_CM_WKUP_CLKSEL_DPLL_DISP, (mul << 8) | (div - 1));

  /* Locked mode */

  putreg32(AM335X_CM_WKUP_CLKMODE_DPLL_DISP, 0x7);

  timeout = 10000;
  while ((getreg32(AM335X_CM_WKUP_IDLEST_DPLL_DISP) & (1 << 0)) == 0 &&
         timeout-- > 0)
    {
      up_udelay(10);
    }

  return timeout > 0 ? OK : -ETIMEDOUT;
}

/****************************************************************************
 * Name: am335x_get_refclk
 ****************************************************************************/

static int am335x_get_refclk(uint32_t *frequency)
{
  uint32_t regval;
  uint32_t sysclk;

  regval = getreg32(AM335X_CM_WKUP_CLKMODE_DPLL_DISP);

  /* Check if we are running in bypass */

  if ((regval & (1 << 23)) != 0)
    {
      return -ENXIO;
    }

  sysclk     = am335x_get_sysclk();
  *frequency = ((regval >> 8) & 0x7ff) * (sysclk / ((regval & 0x7f) + 1));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_lcd_initialize
 *
 * Description:
 *   Initialize the AM335x for use the display described by the provided
 *   instance of struct am335x_panel_info_s.
 *
 *   This function must be called by board specific logic to initialize the
 *   LCD.  Normally the calling sequence is as follows:
 *
 *   1a. Graphics application starts and initializes the NX server via
 *       boardctl(BOARDIOC_NX_START).  This calls the graphics
 *       initialization function nxmu_start() which, in turn, will call
 *       up_fbinitialize().  Or,
 *   1b. The framebuffer character driver is initialized and calls
 *       up_fbinitialize().
 *   2.  The function up_fbinitialize() must reside in board specific logic
 *       under boards/.  It must create the instance of struct
 *       am335x_panel_info_s and call this function with that instance.
 *
 *   For a directly connected LCD, either (1) the struct am335x_panel_info_s
 *   may be initialized with constant data or (2) the desired video mode can
 *   obtained via lookup from edid_mode_lookup() and the struct
 *   am335x_panel_info_s can be created with am335x_lcd_videomode().
 *
 *   If there is access to Extended Display Identification Data (EDID), then
 *   the board-specific logic may read the EDID data and use
 *   am335x_lcd_edid() to use the EDID data to initialize the struct
 *   am335x_panel_info_s instance.
 *
 * Input Parameters:
 *   panel  - Provides information about the connect LCD panel.
 *
 * Returned value:
 *   Zero (OK) is returned on success; a negated errno value is returned in
 *   the the case of a failure.
 *
 ****************************************************************************/

int am335x_lcd_initialize(const struct am335x_panel_info_s *panel)
{
  struct am335x_lcd_dev_s *priv = &g_lcddev;
  uint32_t regval;
  uint32_t reffreq;
  uint32_t timing0;
  uint32_t timing1;
  uint32_t timing2;
  uint32_t logburst;
  uint32_t hbp;
  uint32_t hfp;
  uint32_t hsw;
  uint32_t vbp;
  uint32_t vfp;
  uint32_t vsw;
  uint32_t width;
  uint32_t height;
  int div;
  int ret;

  DEBUGASSERT(panel != NULL);

  /* Configure LCD pins */

  lcdinfo("Configuring pins\n");

#if !defined(CONFIG_AM335X_LCDC_BPP16_565) && !defined(CONFIG_AM335X_LCDC_BPP12_444)
  am335x_gpio_config(GPIO_LCD_DATA0);
  am335x_gpio_config(GPIO_LCD_DATA1);
  am335x_gpio_config(GPIO_LCD_DATA2);
#endif
#ifndef CONFIG_AM335X_LCDC_BPP12_444
  am335x_gpio_config(GPIO_LCD_DATA3);
#endif
  am335x_gpio_config(GPIO_LCD_DATA4);
  am335x_gpio_config(GPIO_LCD_DATA5);
  am335x_gpio_config(GPIO_LCD_DATA6);
  am335x_gpio_config(GPIO_LCD_DATA7);

#if AM335X_BPP > 8 /* Or STN 8-BPP Dual panel */
#if !defined(CONFIG_AM335X_LCDC_BPP16_565) && !defined(CONFIG_AM335X_LCDC_BPP12_444)
  am335x_gpio_config(GPIO_LCD_DATA8);
  am335x_gpio_config(GPIO_LCD_DATA9);
#endif
#ifndef CONFIG_AM335X_LCDC_BPP12_444
  am335x_gpio_config(GPIO_LCD_DATA10);
  am335x_gpio_config(GPIO_LCD_DATA11);
#endif
  am335x_gpio_config(GPIO_LCD_DATA12);
  am335x_gpio_config(GPIO_LCD_DATA13);
  am335x_gpio_config(GPIO_LCD_DATA14);
  am335x_gpio_config(GPIO_LCD_DATA15);
#endif

#if AM335X_BPP > 16 || defined(CONFIG_AM335X_LCDC_TFTPANEL)
#if !defined(CONFIG_AM335X_LCDC_BPP16_565) && !defined(CONFIG_AM335X_LCDC_BPP12_444)
  am335x_gpio_config(GPIO_LCD_DATA16);
  am335x_gpio_config(GPIO_LCD_DATA17);
  am335x_gpio_config(GPIO_LCD_DATA18);
#endif
#ifndef CONFIG_AM335X_LCDC_BPP12_444
  am335x_gpio_config(GPIO_LCD_DATA19);
#endif
  am335x_gpio_config(GPIO_LCD_DATA20);
  am335x_gpio_config(GPIO_LCD_DATA21);
  am335x_gpio_config(GPIO_LCD_DATA22);
  am335x_gpio_config(GPIO_LCD_DATA23);
#endif

  /* Other pins */

  am335x_gpio_config(GPIO_LCD_HSYNC);
  am335x_gpio_config(GPIO_LCD_VSYNC);
  am335x_gpio_config(GPIO_LCD_AC_BIAS_EN);
  am335x_gpio_config(GPIO_LCD_MEMORY_CLK_1);
  am335x_gpio_config(GPIO_LCD_MEMORY_CLK_2);

  /* Initialize the device state singleton */

  nxsem_init(&priv->exclsem, 0, 1);
  memcpy(&priv->panel, panel, sizeof(struct am335x_panel_info_s));

  /* Save framebuffer information */

  priv->stride = (priv->panel.width * priv->panel.bpp + 7) >> 3;
  priv->fbsize = priv->stride * priv->panel.height;
  DEBUGASSERT(priv->fbsize <= AM335X_LCDC_FB_SIZE);

  /* Attach the LCD interrupt */

  ret = irq_attach(AM335X_IRQ_LCDC, am335x_lcd_interrupt, priv);
  if (ret < 0)
    {
      lcderr("ERROR:  Failed to attach LCDC interrupt.");
    }

  /* Adjust reference clock to get double of requested pixel clock frequency
   * HDMI/DVI displays are very sensitive to error in frequency value.
   */

  ret = am335x_set_refclk(2 * priv->panel.pixclk);
  if (ret < 0)
    {
      lcderr("ERROR:  Failed to set source frequency\n");
      return ret;
    }

  /* Read back the actual reference clock frequency */

  ret = am335x_get_refclk(&reffreq);
  if (ret < 0)
    {
      lcderr("ERROR:  Failed to get reference frequency\n");
      return ret;
    }

  /* Panel initialization */

  /* Put the LCD framebuffer memory in a known state */

  am335x_lcdclear(CONFIG_AM335X_LCDC_BACKCOLOR);

  /* Only raster mode is supported */

  regval   = LCD_CTRL_MODE_RASTER;
  div      = am335x_lcd_divisor(reffreq, priv->panel.pixclk);
  regval  |= (div << LCD_CTRL_CLKDIV_SHIFT);
  putreg32(AM335X_LCD_CTRL, regval);

  /* Set timing */

  timing0  = timing1 = timing2 = 0;

  hbp      = priv->panel.hbp - 1;
  hfp      = priv->panel.hfp - 1;
  hsw      = priv->panel.hsw - 1;

  vbp      = priv->panel.vbp;
  vfp      = priv->panel.vfp;
  vsw      = priv->panel.vsw - 1;

  height   = priv->panel.height - 1;
  width    = priv->panel.width - 1;

  /* Horizontal back porch */

  timing0 |= (hbp & 0xff) << LCD_RASTER_TIMING_0_HBP_SHIFT;
  timing2 |= ((hbp >> 8) & 3) << LCD_RASTER_TIMING_2_HBP_HBITS_SHIFT;

  /* Horizontal front porch */

  timing0 |= (hfp & 0xff) << LCD_RASTER_TIMING_0_HFP_SHIFT;
  timing2 |= ((hfp >> 8) & 3) << LCD_RASTER_TIMING_2_HFP_HBITS_SHIFT;

  /* Horizontal sync width */

  timing0 |= (hsw & 0x3f) << LCD_RASTER_TIMING_0_HSW_SHIFT;
  timing2 |= ((hsw >> 6) & 0xf) << LCD_RASTER_TIMING_2_HSW_HBITS_SHIFT;

  /* Vertical back porch, front porch, sync width */

  timing1 |= (vbp & 0xff) << LCD_RASTER_TIMING_1_VBP_SHIFT;
  timing1 |= (vfp & 0xff) << LCD_RASTER_TIMING_1_VFP_SHIFT;
  timing1 |= (vsw & 0x3f) << LCD_RASTER_TIMING_1_VSW_SHIFT;

  /* Pixels per line */

  timing0 |= ((width >> 10) & 1) << LCD_RASTER_TIMING_0_PPLLSB_SHIFT;
  timing0 |= ((width >> 4) & 0x3f) << LCD_RASTER_TIMING_0_PPLLSB_SHIFT;

  /* Lines per panel */

  timing1 |= (height & 0x3ff) << LCD_RASTER_TIMING_1_LPP_SHIFT;
  timing2 |= ((height >> 10) & 1) << LCD_RASTER_TIMING_2_LPP_B10_SHIFT;

  /* Clock signal settings */

  if (priv->panel.sync_ctrl)
    {
      timing2 |= LCD_RASTER_TIMING_2_PHSVS_ON;
    }

  if (priv->panel.sync_edge)
    {
      timing2 |= LCD_RASTER_TIMING_2_PHSVS_RF;
    }

  if (!priv->panel.hsync_active)
    {
      timing2 |= LCD_RASTER_TIMING_2_IHS;
    }

  if (!priv->panel.vsync_active)
    {
      timing2 |= LCD_RASTER_TIMING_2_IVS;
    }

  if (!priv->panel.pixelclk_active)
    {
      timing2 |= LCD_RASTER_TIMING_2_IPC;
    }

  /* AC bias */

  timing2 |= (priv->panel.acbias << LCD_RASTER_TIMING_2_ACB_SHIFT);
  timing2 |= (priv->panel.acbias_pint << LCD_RASTER_TIMING_2_ACBI_SHIFT);

  putreg32(AM335X_LCD_RASTER_TIMING_0, timing0);
  putreg32(AM335X_LCD_RASTER_TIMING_1, timing1);
  putreg32(AM335X_LCD_RASTER_TIMING_2, timing2);

  /* DMA settings */

  regval = LCD_DMA_CTRL_FRAME_MODE;

  /* Find power of 2 for current burst size */

  switch (priv->panel.dma_burstsz)
    {
    case 1:
      logburst = 0;
      break;

    case 2:
      logburst = 1;
      break;

    case 4:
      logburst = 2;
      break;

    case 8:
      logburst = 3;
      break;

    case 16:
    default:
      logburst = 4;
      break;
    }

  regval |= (logburst << LCD_DMA_CTRL_BURST_SIZE_SHIFT);

  /* XXX: FIFO TH */

  regval |= (0 << LCD_DMA_CTRL_TH_FIFO_RDY_SHIFT);
  putreg32(AM335X_LCD_DMA_CTRL, regval);

  putreg32(AM335X_LCD_DMA_FB0_BASE, CONFIG_AM335X_LCDC_FB_PBASE);
  putreg32(AM335X_LCD_DMA_FB0_BASE,
           CONFIG_AM335X_LCDC_FB_PBASE + priv->fbsize - 1);
  putreg32(AM335X_LCD_DMA_FB1_BASE, CONFIG_AM335X_LCDC_FB_PBASE);
  putreg32(AM335X_LCD_DMA_FB1_CEIL,
           CONFIG_AM335X_LCDC_FB_PBASE + priv->fbsize - 1);

  /* Enable LCD */

  regval  = LCD_RASTER_CTRL_LCD_TFT;
  regval |= (priv->panel.fdd << LCD_RASTER_CTRL_REQDLY_SHIFT);
  regval |= LCD_RASTER_CTRL_DATA;

  if (priv->panel.bpp >= 24)
    {
      regval |= LCD_RASTER_CTRL_TFT24;
    }

  if (priv->panel.bpp == 32)
    {
      regval |= LCD_RASTER_CTRL_TFT24_UNPACKED;
    }

  putreg32(AM335X_LCD_RASTER_CTRL, regval);

  putreg32(AM335X_LCD_CLKC_ENABLE,
           LCD_CLKC_ENABLE_CORE | LCD_CLKC_ENABLE_LIDD |
           LCD_CLKC_ENABLE_DMA);

  putreg32(AM335X_LCD_CLKC_RESET, LCD_CLKC_RESET_MAIN);
  up_udelay(100);
  putreg32(AM335X_LCD_CLKC_RESET, 0);

  regval  = LCD_IRQ_DONE | LCD_IRQ_RR_DONE | LCD_IRQ_SYNC | LCD_IRQ_ACB |
           LCD_IRQ_PL | LCD_IRQ_FUF | LCD_IRQ_EOF0 | LCD_IRQ_EOF1;
  putreg32(AM335X_LCD_IRQ_EN_SET, regval);

  regval  = getreg32(AM335X_LCD_RASTER_CTRL);
  regval |= LCD_RASTER_CTRL_LCD_EN;
  putreg32(AM335X_LCD_RASTER_CTRL, regval);

  putreg32(AM335X_LCD_SYSC, LCD_SYSC_IDLE_SMART | LCD_SYSC_STANDBY_SMART);

#ifdef CONFIG_AM335X_LCDC_BACKLIGHT
  /* Turn on the back light
   * REVISIT:  Current assumes a discrete ON/OFF control.  Needs additional
   * support for backlight level control.
   */

  am335x_backlight(true);
#endif

  /* Enable interrupts at the interrupt controller */

  up_enable_irq(AM335X_IRQ_LCDC);
  return OK;
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane of the specified plane.  Many OSDs support multiple planes of
 *   video.
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

struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  lcdinfo("vplane: %d\n", vplane);
  if (vplane == 0)
    {
      return &g_fbinterface;
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
#ifdef CONFIG_AM335X_LCDC_BACKLIGHT
  /* Turn off the back light */

  am335x_backlight(false);
#endif

  /* Reset/Disable the LCD controller */

  /* Disable clocking to the LCD peripheral */

  /* Detach and disable the LCDC interrupt */
}

/****************************************************************************
 * Name:  am335x_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the AM335x.  Clearing
 *   the display in the normal way by writing a sequences of runs that
 *   covers the entire display can be slow.  Here the display is cleared by
 *   simply setting all VRAM memory to the specified color.
 *
 ****************************************************************************/

void am335x_lcdclear(nxgl_mxpixel_t color)
{
  struct am335x_lcd_dev_s *priv = &g_lcddev;
#if AM335X_BPP > 16
  uint32_t *dest = (uint32_t *)CONFIG_AM335X_LCDC_FB_VBASE;
  int incr = sizeof(uint32_t);
#else
  uint16_t *dest = (uint16_t *)CONFIG_AM335X_LCDC_FB_VBASE;
  int incr = sizeof(uint16_t);
#endif
  int i;

  lcdinfo("Clearing display: color=%04jx VRAM=%08lx size=%lu\n",
          (uintmax_t)color, (unsigned long)CONFIG_AM335X_LCDC_FB_VBASE,
          (unsigned long)priv->fbsize);

  for (i = 0; i < priv->fbsize; i += incr)
    {
      *dest++ = color;
    }
}
