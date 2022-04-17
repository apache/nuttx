/****************************************************************************
 * arch/arm/src/imxrt/imxrt_lcd.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017, NXP Semiconductors, Inc.
 *   Author: Johannes Schock (Port)
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

#include "imxrt_lcd.h"
#include "imxrt_periphclks.h"

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/video/fb.h>

#include "arm_internal.h"

#include <nuttx/board.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"

#include "hardware/imxrt_pinmux.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_LCD_CLK_PER_LINE \
  (CONFIG_IMXRT_LCD_HWIDTH + CONFIG_IMXRT_LCD_HPULSE + \
   CONFIG_IMXRT_LCD_HFRONTPORCH + CONFIG_IMXRT_LCD_HBACKPORCH)
#define IMXRT_LCD_LINES_PER_FRAME \
  (CONFIG_IMXRT_LCD_VHEIGHT + CONFIG_IMXRT_LCD_VPULSE + \
   CONFIG_IMXRT_LCD_VFRONTPORCH + CONFIG_IMXRT_LCD_VBACKPORCH)
#define IMXRT_LCD_PIXEL_CLOCK \
  (IMXRT_LCD_CLK_PER_LINE * IMXRT_LCD_LINES_PER_FRAME * \
   CONFIG_IMXRT_LCD_REFRESH_FREQ)

/* Framebuffer characteristics in bytes */

#define IMXRT_STRIDE ((CONFIG_IMXRT_LCD_HWIDTH * IMXRT_BPP + 7) / 8)
#define IMXRT_FBSIZE (IMXRT_STRIDE * CONFIG_IMXRT_LCD_VHEIGHT)

/* Delays */

#define IMXRT_LCD_RESET_DELAY (0x100)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Get information about the video controller configuration and the
 * configuration of each color plane.
 */

static int imxrt_getvideoinfo(struct fb_vtable_s *vtable,
             struct fb_videoinfo_s *vinfo);
static int imxrt_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
             struct fb_planeinfo_s *pinfo);

/* The following is provided only if the video hardware supports RGB color
 * mapping
 */

#ifdef CONFIG_FB_CMAP
static int imxrt_getcmap(struct fb_vtable_s *vtable,
                         struct fb_cmap_s *cmap);
static int imxrt_putcmap(struct fb_vtable_s *vtable,
                         const struct fb_cmap_s *cmap);
#endif

#ifdef CONFIG_FB_HWCURSOR
#  error "Cursor control not supported by this driver"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure describes the video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt      = IMXRT_COLOR_FMT,
  .xres     = CONFIG_IMXRT_LCD_HWIDTH,
  .yres     = CONFIG_IMXRT_LCD_VHEIGHT,
  .nplanes  = 1,
};

/* This structure describes the single color plane */

static const struct fb_planeinfo_s g_planeinfo =
{
  .fbmem    = (void *)CONFIG_IMXRT_LCD_VRAMBASE,
  .fblen    = IMXRT_FBSIZE,
  .stride   = IMXRT_STRIDE,
  .display  = 0,
  .bpp      = IMXRT_BPP,
};

struct pixel_format_reg
{
    uint32_t reg_ctrl;  /* Value of register CTRL. */
    uint32_t reg_ctrl1; /* Value of register CTRL1. */
};

#if defined (CONFIG_IMXRT_LCD_INPUT_BPP8) || defined (CONFIG_IMXRT_LCD_INPUT_BPP8_LUT)
  static const struct pixel_format_reg pixel_format =
  {
    /* Register CTRL. */

    LCDIF_CTRL_WORD_LENGTH(1U),

    /* Register CTRL1. */

    LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x0fu)
  };
  const uint32_t data_bus = LCDIF_CTRL_LCD_DATABUS_WIDTH(1);

#else

#  if defined (CONFIG_IMXRT_LCD_INPUT_BPP15)
  static const struct pixel_format_reg pixel_format =
  {
    /* Register CTRL. */

    LCDIF_CTRL_WORD_LENGTH(0U) | LCDIF_CTRL_SET_DATA_FORMAT_16_BIT_MASK,

    /* Register CTRL1. */

    LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x0fu)
  };
#  elif defined (CONFIG_IMXRT_LCD_INPUT_BPP16)
  static const struct pixel_format_reg pixel_format =
  {
    /* Register CTRL. */

    LCDIF_CTRL_WORD_LENGTH(0U),

    /* Register CTRL1. */

    LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x0fu)
  };
#  elif defined (CONFIG_IMXRT_LCD_INPUT_BPP24)
  static const struct pixel_format_reg pixel_format =
  {
    /* Register CTRL. 24-bit. */

    LCDIF_CTRL_WORD_LENGTH(3U),

    /* Register CTRL1. */

    LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x0fu)
  };
#  elif defined (CONFIG_IMXRT_LCD_INPUT_BPP32)
  static const struct pixel_format_reg pixel_format =
  {
    /* Register CTRL. 24-bit. */

    LCDIF_CTRL_WORD_LENGTH(3U),

    /* Register CTRL1. */

    LCDIF_CTRL1_BYTE_PACKING_FORMAT(0x07u)
  };
#  endif

#  if defined (CONFIG_IMXRT_LCD_OUTPUT_8)
  const uint32_t data_bus = LCDIF_CTRL_LCD_DATABUS_WIDTH(1);
#  elif defined (CONFIG_IMXRT_LCD_OUTPUT_16)
  const uint32_t data_bus = LCDIF_CTRL_LCD_DATABUS_WIDTH(0);
#  elif defined (CONFIG_IMXRT_LCD_OUTPUT_18)
  const uint32_t data_bus = LCDIF_CTRL_LCD_DATABUS_WIDTH(2);
#  elif defined (CONFIG_IMXRT_LCD_OUTPUT_24)
  const uint32_t data_bus = LCDIF_CTRL_LCD_DATABUS_WIDTH(3);
#  endif

#endif

/* The framebuffer object -- There is no private state information in this
 * framebuffer driver.
 */

struct fb_vtable_s g_fbobject =
{
  .getvideoinfo  = imxrt_getvideoinfo,
  .getplaneinfo  = imxrt_getplaneinfo,
#ifdef CONFIG_FB_CMAP
  .getcmap       = imxrt_getcmap,
  .putcmap       = imxrt_putcmap,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_getvideoinfo
 ****************************************************************************/

static int imxrt_getvideoinfo(struct fb_vtable_s *vtable,
                              struct fb_videoinfo_s *vinfo)
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
 * Name: imxrt_getplaneinfo
 ****************************************************************************/

static int imxrt_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                              struct fb_planeinfo_s *pinfo)
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
 * Name: imxrt_getcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int imxrt_getcmap(struct fb_vtable_s *vtable,
                         struct fb_cmap_s *cmap)
{
  uint32_t n;
  uint32_t reg;

  lcdinfo("vtable=%p cmap=%p first=%d len=%d\n",
      vtable, cmap, cmap->first, cmap->len);

  DEBUGASSERT(vtable && cmap);

  /* Only has 256 entries. */

  if (cmap->first + cmap->len > IMXRT_LCDIF_LUT_ENTRY_NUM)
    {
      return ERROR;
    }

  putreg32(cmap->first, IMXRT_LCDIF_LUT0_ADDR);

  for (n = 0; n < cmap->len; n++)
    {
      reg = getreg32(IMXRT_LCDIF_LUT0_DATA);

#if defined (CONFIG_IMXRT_LCD_OUTPUT_24)
      cmap->red[n] =   (reg >> 16) & 0xff;
      cmap->green[n] = (reg >>  8) & 0xff;
      cmap->blue[n] =  (reg >>  0) & 0xff;
#elif defined (CONFIG_IMXRT_LCD_OUTPUT_18)
      cmap->red[n] =   (reg >> 10) & 0xfc;
      cmap->green[n] = (reg >>  4) & 0xfc;
      cmap->blue[n] =  (reg <<  2) & 0xfc;
#elif defined (CONFIG_IMXRT_LCD_OUTPUT_16)
      cmap->red[n] =   (reg >>  8) & 0xf8;
      cmap->green[n] = (reg >>  3) & 0xfc;
      cmap->blue[n] =  (reg <<  3) & 0xf8;
#endif
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_putcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int imxrt_putcmap(struct fb_vtable_s *vtable,
                         const struct fb_cmap_s *cmap)
{
  uint32_t n;

  lcdinfo("vtable=%p cmap=%p first=%d len=%d\n",
          vtable, cmap, cmap->first, cmap->len);

  DEBUGASSERT(vtable && cmap);

  /* Only has 256 entries. */

  if (cmap->first + cmap->len > IMXRT_LCDIF_LUT_ENTRY_NUM)
    {
      return ERROR;
    }

  putreg32(cmap->first, IMXRT_LCDIF_LUT0_ADDR);

  for (n = 0; n < cmap->len; n++)
    {
#if defined (CONFIG_IMXRT_LCD_OUTPUT_24)
      putreg32((uint32_t)0xff000000             |
               ((uint32_t)cmap->red[n]   << 16) |
               ((uint32_t)cmap->green[n] <<  8) |
               ((uint32_t)cmap->blue[n]  <<  0),
               IMXRT_LCDIF_LUT0_DATA);
#elif defined (CONFIG_IMXRT_LCD_OUTPUT_18)
      putreg32((uint32_t)0xfffc0000                      |
               (((uint32_t)cmap->red[n] & 0xfc)   << 10) |
               (((uint32_t)cmap->green[n] & 0xfc) <<  4) |
               (((uint32_t)cmap->blue[n] & 0xfc)  >>  2),
               IMXRT_LCDIF_LUT0_DATA);
#elif defined (CONFIG_IMXRT_LCD_OUTPUT_16)
      putreg32((uint32_t)0xffff0000                      |
               (((uint32_t)cmap->red[n] & 0xf8)   <<  8) |
               (((uint32_t)cmap->green[n] & 0xfc) <<  3) |
               (((uint32_t)cmap->blue[n] & 0xf8)  >>  3),
               IMXRT_LCDIF_LUT0_DATA);
#endif
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_lcd_reset
 ****************************************************************************/

static void imxrt_lcdreset(void)
{
  volatile uint32_t i = IMXRT_LCD_RESET_DELAY;

  /* Disable the clock gate. */

  putreg32(LCDIF_CTRL_CLKGATE_MASK, IMXRT_LCDIF_CTRL_CLR);

  /* Confirm the clock gate is disabled. */

  while ((getreg32(IMXRT_LCDIF_CTRL) & LCDIF_CTRL_CLKGATE_MASK) != 0)
    {
    }

  /* Reset the block. */

  putreg32(LCDIF_CTRL_SFTRST_MASK, IMXRT_LCDIF_CTRL_SET);

  /* Confirm the reset bit is set. */

  while ((getreg32(IMXRT_LCDIF_CTRL) & LCDIF_CTRL_SFTRST_MASK) == 0)
    {
    }

  /* Delay for the reset. */

  while (i--)
    {
    }

  /* Bring the module out of reset. */

  putreg32(LCDIF_CTRL_SFTRST_MASK, IMXRT_LCDIF_CTRL_CLR);

  /* Disable the clock gate. */

  putreg32(LCDIF_CTRL_CLKGATE_MASK, IMXRT_LCDIF_CTRL_CLR);
}

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
#if defined (CONFIG_IMXRT_LCD_INPUT_BPP8_LUT) || defined (CONFIG_IMXRT_LCD_INPUT_BPP8)
  uint32_t n;
#endif

  /* Configure pins
   *
   * LCD panel data. Pins used depend on the panel configuration:
   */

  lcdinfo("Configuring pins\n");

#if defined(CONFIG_IMXRT_LCD_OUTPUT_24)

  imxrt_config_gpio(GPIO_LCD_DATA23);
  imxrt_config_gpio(GPIO_LCD_DATA22);
  imxrt_config_gpio(GPIO_LCD_DATA21);
  imxrt_config_gpio(GPIO_LCD_DATA20);
  imxrt_config_gpio(GPIO_LCD_DATA19);
  imxrt_config_gpio(GPIO_LCD_DATA18);

#endif
#if defined(CONFIG_IMXRT_LCD_OUTPUT_24) || \
    defined(CONFIG_IMXRT_LCD_OUTPUT_18)

  imxrt_config_gpio(GPIO_LCD_DATA17);
  imxrt_config_gpio(GPIO_LCD_DATA16);

#endif
#if defined(CONFIG_IMXRT_LCD_OUTPUT_24) || \
    defined(CONFIG_IMXRT_LCD_OUTPUT_18) || \
    defined(CONFIG_IMXRT_LCD_OUTPUT_16)

  imxrt_config_gpio(GPIO_LCD_DATA15);
  imxrt_config_gpio(GPIO_LCD_DATA14);
  imxrt_config_gpio(GPIO_LCD_DATA13);
  imxrt_config_gpio(GPIO_LCD_DATA12);
  imxrt_config_gpio(GPIO_LCD_DATA11);
  imxrt_config_gpio(GPIO_LCD_DATA10);
  imxrt_config_gpio(GPIO_LCD_DATA09);
  imxrt_config_gpio(GPIO_LCD_DATA08);

#endif

  imxrt_config_gpio(GPIO_LCD_DATA07);
  imxrt_config_gpio(GPIO_LCD_DATA06);
  imxrt_config_gpio(GPIO_LCD_DATA05);
  imxrt_config_gpio(GPIO_LCD_DATA04);
  imxrt_config_gpio(GPIO_LCD_DATA03);
  imxrt_config_gpio(GPIO_LCD_DATA02);
  imxrt_config_gpio(GPIO_LCD_DATA01);
  imxrt_config_gpio(GPIO_LCD_DATA00);

  /* Other pins */

  imxrt_config_gpio(GPIO_LCD_ENABLE);
  imxrt_config_gpio(GPIO_LCD_HSYNC);
  imxrt_config_gpio(GPIO_LCD_VSYNC);
  imxrt_config_gpio(GPIO_LCD_CLK);

  lcdinfo("Enable clocking to the LCD controller\n");

  /* Enable clocking to the LCD peripheral */

  imxrt_clockall_pxp();

  imxrt_clockall_lcd();

  imxrt_clockall_lcdif_pix();

  /* Reset the LCD */

  imxrt_lcd_initialize();

  imxrt_lcdreset();

  lcdinfo("Configuring the LCD controller\n");

  putreg32(pixel_format.reg_ctrl | data_bus |
      LCDIF_CTRL_DOTCLK_MODE_MASK |
      LCDIF_CTRL_BYPASS_COUNT_MASK |
      LCDIF_CTRL_MASTER_MASK, IMXRT_LCDIF_CTRL);

  putreg32(pixel_format.reg_ctrl1, IMXRT_LCDIF_CTRL1);

  putreg32((CONFIG_IMXRT_LCD_VHEIGHT << LCDIF_TRANSFER_COUNT_V_COUNT_SHIFT) |
      (CONFIG_IMXRT_LCD_HWIDTH << LCDIF_TRANSFER_COUNT_H_COUNT_SHIFT),
      IMXRT_LCDIF_TRANSFER_COUNT);

  putreg32(LCDIF_VDCTRL0_ENABLE_PRESENT_MASK |
      LCDIF_VDCTRL0_VSYNC_PERIOD_UNIT_MASK |
      LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_UNIT_MASK |
      VSYNC_POL | HSYNC_POL | DATAEN_POL | DATA_EDGE |
      CONFIG_IMXRT_LCD_VPULSE,
      IMXRT_LCDIF_VDCTRL0);

  putreg32(CONFIG_IMXRT_LCD_VPULSE + CONFIG_IMXRT_LCD_VHEIGHT +
      CONFIG_IMXRT_LCD_VFRONTPORCH + CONFIG_IMXRT_LCD_VBACKPORCH,
      IMXRT_LCDIF_VDCTRL1);

  putreg32((CONFIG_IMXRT_LCD_HPULSE <<
            LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH_SHIFT) |
      ((CONFIG_IMXRT_LCD_HFRONTPORCH + CONFIG_IMXRT_LCD_HBACKPORCH +
        CONFIG_IMXRT_LCD_HWIDTH + CONFIG_IMXRT_LCD_HPULSE)
          << LCDIF_VDCTRL2_HSYNC_PERIOD_SHIFT),
      IMXRT_LCDIF_VDCTRL2);

  putreg32(((CONFIG_IMXRT_LCD_HBACKPORCH + CONFIG_IMXRT_LCD_HPULSE)
        << LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT_SHIFT) |
      ((CONFIG_IMXRT_LCD_VBACKPORCH + CONFIG_IMXRT_LCD_VPULSE)
        << LCDIF_VDCTRL3_VERTICAL_WAIT_CNT_SHIFT),
      IMXRT_LCDIF_VDCTRL3);

  putreg32(LCDIF_VDCTRL4_SYNC_SIGNALS_ON_MASK |
      (CONFIG_IMXRT_LCD_HWIDTH <<
       LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT_SHIFT),
      IMXRT_LCDIF_VDCTRL4);

#ifdef CONFIG_IMXRT_LCD_BGR
  /* Swap red and blue.  The colors will be 0x00RRGGBB, not 0x00bbGGRR. */

  /* TODO */
#endif

#if defined (CONFIG_IMXRT_LCD_INPUT_BPP8_LUT) || defined (CONFIG_IMXRT_LCD_INPUT_BPP8)

  putreg32(0, IMXRT_LCDIF_LUT0_ADDR);

  for (n = 0; n < IMXRT_LCDIF_LUT_ENTRY_NUM; n++)
    {
      uint8_t  red;
      uint8_t  green;
      uint8_t  blue;

      red = (n << 0) & 0xe0;
      green = (n << 3) & 0xe0;
      blue = (n << 6) & 0xc0;

#if defined (CONFIG_IMXRT_LCD_OUTPUT_24)
      putreg32((uint32_t)0xff000000    |
               ((uint32_t)red   << 16) |
               ((uint32_t)green <<  8) |
               ((uint32_t)blue  <<  0),
               IMXRT_LCDIF_LUT0_DATA);
#elif defined (CONFIG_IMXRT_LCD_OUTPUT_18)
      putreg32((uint32_t)0xfffc0000             |
               (((uint32_t)red & 0xfc)   << 10) |
               (((uint32_t)green & 0xfc) <<  4) |
               (((uint32_t)blue & 0xfc)  >>  2),
               IMXRT_LCDIF_LUT0_DATA);
#elif defined (CONFIG_IMXRT_LCD_OUTPUT_16)
      putreg32((uint32_t)0xffff0000             |
               (((uint32_t)red & 0xf8)   <<  8) |
               (((uint32_t)green & 0xfc) <<  3) |
               (((uint32_t)blue & 0xf8)  >>  3),
               IMXRT_LCDIF_LUT0_DATA);
#endif
    }

  putreg32(0, IMXRT_LCDIF_LUT_CTRL);

#endif

  putreg32(CONFIG_IMXRT_LCD_VRAMBASE, IMXRT_LCDIF_CUR_BUF);
  putreg32(CONFIG_IMXRT_LCD_VRAMBASE, IMXRT_LCDIF_NEXT_BUF);

  /* Clear the display */

  imxrt_lcdclear(CONFIG_IMXRT_LCD_BACKCOLOR);

#ifdef CONFIG_IMXRT_LCD_BACKLIGHT

  /* Turn on the back light */

  imxrt_backlight(true);

#endif

  putreg32(LCDIF_CTRL_RUN_MASK | LCDIF_CTRL_DOTCLK_MODE_MASK,
      IMXRT_LCDIF_CTRL_SET);

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
  /* We assume there is only one use of the LCD and so we do not need to
   * worry about mutually exclusive access to the LCD hardware.
   */

#ifdef CONFIG_IMXRT_LCD_BACKLIGHT
  /* Turn off the back light */

  imxrt_backlight(false);
#endif

  /* Disable the clock gate. */

  putreg32(LCDIF_CTRL_CLKGATE_MASK, IMXRT_LCDIF_CTRL_CLR);

  /* Disable clocking to the LCD peripheral */

  imxrt_clockoff_lcdif_pix();

  imxrt_clockoff_lcd();
}

/****************************************************************************
 * Name:  imxrt_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the IMXRT.  Clearing
 *   the display in the normal way by writing a sequences of runs that
 *   covers the entire display can be slow.  Here the display is cleared by
 *   simply setting all VRAM memory to the specified color.
 *
 ****************************************************************************/

void imxrt_lcdclear(nxgl_mxpixel_t color)
{
  int i;
  int size;

  size = CONFIG_IMXRT_LCD_HWIDTH * CONFIG_IMXRT_LCD_VHEIGHT;

  #if IMXRT_BPP > 24
  uint32_t *dest = (uint32_t *)CONFIG_IMXRT_LCD_VRAMBASE;

  lcdinfo("Clearing display: color=%08x VRAM=%08x size=%d\n",
          color, CONFIG_IMXRT_LCD_VRAMBASE,
          size * sizeof(uint32_t));

#elif IMXRT_BPP > 16
  uint32_t *dest = (uint32_t *)CONFIG_IMXRT_LCD_VRAMBASE;

  size = (size * 3) >> 2;

  lcdinfo("Clearing display: color=%06x VRAM=%08x size=%d\n",
          color, CONFIG_IMXRT_LCD_VRAMBASE,
          size * sizeof(uint32_t));

#elif IMXRT_BPP > 8
  uint16_t *dest = (uint16_t *)CONFIG_IMXRT_LCD_VRAMBASE;

  lcdinfo("Clearing display: color=%04jx VRAM=%08x size=%d\n",
          (uintmax_t)color, CONFIG_IMXRT_LCD_VRAMBASE,
          size * sizeof(uint16_t));
#else
  uint8_t *dest = (uint8_t *)CONFIG_IMXRT_LCD_VRAMBASE;

  lcdinfo("Clearing display: color=%02x VRAM=%08x size=%d\n",
          color, CONFIG_IMXRT_LCD_VRAMBASE,
          size * sizeof(uint8_t));
#endif

  for (i = 0; i < size; i++)
    {
      *dest++ = color;
    }
}
