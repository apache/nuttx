/****************************************************************************
 *  arch/x86_64/src/intel64/intel64_lowsetup.c
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

#include <nuttx/config.h>

#include <string.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/multiboot2.h>

#ifdef CONFIG_MULTBOOT2_FB_TERM
#include <nuttx/nx/nxfonts.h>
#endif

#include "x86_64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

struct multiboot_fb_s
{
  void *baseaddr;
  uint32_t height;
  uint32_t width;
  uint32_t pitch;
  uint8_t bpp;
  uint8_t type;
};

#ifdef CONFIG_MULTBOOT2_FB_TERM
struct fb_term_s
{
  const struct nx_fontpackage_s *font;
  uint32_t cursor_x;
  uint32_t cursor_y;
};

void fb_term_initialize(void);
#endif  /* CONFIG_MULTBOOT2_FB_TERM */

void fb_clear(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct multiboot_fb_s fb =
{
  .baseaddr = NULL
};

#ifdef CONFIG_MULTBOOT2_FB_TERM
struct fb_term_s fb_term;
#endif  /* CONFIG_MULTBOOT2_FB_TERM */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  fb_draw_pixel
 *
 * Description:
 *   Draw a pixel on the framebuffer.  Note that the color paramter must
 *   be in the format specified by the bpp of the framebuffer.
 *
 ****************************************************************************/

static void fb_draw_pixel(uint32_t color, uint32_t x, uint32_t y)
{
  /* Check if we support this type of framebuffer */

  if (fb.type != MULTIBOOT_FRAMEBUFFER_TYPE_RGB)
    return;

  /* Make sure we are within the bounds */

  if (x >= fb.width || y >= fb.height)
    return;

  switch (fb.bpp)
    {
      case 8:
        {
          uint8_t *pixel = (uint8_t *)(
              (uintptr_t)fb.baseaddr + (fb.pitch * y) + x);
          *pixel = (uint8_t)color;
          break;
        }

      case 15:
      case 16:
        {
          uint16_t *pixel = (uint16_t *)(
            (uintptr_t)fb.baseaddr + (fb.pitch * y) + x * 2);
          *pixel = (uint16_t)color;
          break;
        }

      case 24:
        {
          /* We have to be careful here to not overwrite the lower 8bits
            * of the next pixel in the buffer.
            */

          uint32_t *pixel = (uint32_t *)(
            (uintptr_t)fb.baseaddr + (fb.pitch * y) + x * 3);
          *pixel = (color & 0xffffff) | (*pixel & 0xff000000);
          break;
        }

      case 32:
        {
          uint32_t *pixel = (uint32_t *)(
            (uintptr_t)fb.baseaddr + (fb.pitch * y) + x * 4);
          *pixel = color;
          break;
        }
    }
}

#if 0
/****************************************************************************
 * Function:  fb_test_line
 *
 * Description:
 *   This is a simple test function that can be used to draw a 45deg
 *   line across the screen.
 *
 ****************************************************************************/

static void fb_test_line(void)
{
  size_t idx;
  uint32_t color;

  switch (fb.bpp)
    {
      case 8:
        color = 0xff;
        break;
      case 15:
      case 16:
        color = 0x7fff;
        break;
      case 24:
        color = 0xffffff;
        break;
      case 32:
        color = 0xffffffff;
        break;
      default:
        return;
    }

  for (idx = 0; (idx < fb.height) && (idx < fb.width); idx++)
    {
      fb_draw_pixel(color, idx, idx);
    }
}
#endif

#ifdef CONFIG_MULTBOOT2_FB_TERM
static void fb_scroll(void)
{
  void *destp = fb.baseaddr;
  uint32_t save_rows = ((fb.height / fb_term.font->metrics.mxheight) - 1);
  size_t row_size = fb.pitch * fb_term.font->metrics.mxheight;
  uint32_t pxl_row = 0;

  for (; pxl_row < save_rows * fb_term.font->metrics.mxheight; pxl_row++)
    {
      memcpy(destp, destp + row_size, fb.pitch);
      destp += fb.pitch;
    }

  memset(destp, 0, fb.pitch * (fb.height - pxl_row));

  fb_term.cursor_y -= fb_term.font->metrics.mxheight;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void x86_64_mb2_fbinitialize(struct multiboot_tag_framebuffer *fbt)
{
  fb.baseaddr = (void *)(uintptr_t)fbt->common.framebuffer_addr;
  fb.width = fbt->common.framebuffer_width;
  fb.height = fbt->common.framebuffer_height;
  fb.pitch = fbt->common.framebuffer_pitch;
  fb.bpp = fbt->common.framebuffer_bpp;
  fb.type = fbt->common.framebuffer_type;

  up_map_region(fb.baseaddr, fb.pitch * fb.height,
    X86_PAGE_WR | X86_PAGE_PRESENT |
    X86_PAGE_NOCACHE | X86_PAGE_GLOBAL);

  fb_clear();
 
#ifdef CONFIG_MULTBOOT2_FB_TERM
  fb_term_initialize();
#endif

}

void fb_clear(void)
{
  if (fb.baseaddr == NULL)
    return;

  memset(fb.baseaddr, 0, fb.pitch * fb.height);
}

#ifdef CONFIG_MULTBOOT2_FB_TERM
void fb_term_initialize(void)
{
  fb_term.font = nxf_getfonthandle(FONTID_DEFAULT);
  fb_term.cursor_x = 0;
  fb_term.cursor_y = 0;
}

void fb_putc(char ch)
{
  uint8_t gly_x;
  uint8_t gly_y;
  const struct nx_fontbitmap_s *fbm;

  if (fb.baseaddr == NULL)
    return;

  if (ch == '\n')
    {
      fb_term.cursor_y += fb_term.font->metrics.mxheight;
      return;
    }

  if (ch == '\r')
    {
      fb_term.cursor_x = 0;
      return;
    }

  fbm = nxf_getbitmap((NXHANDLE)fb_term.font, ch);
  if (fbm == NULL)
    {
      fb_putc('.');
      return;
    }

  for (gly_y = 0; gly_y < fbm->metric.height; gly_y++)
    {
      if (fb_term.cursor_y + gly_y >= fb.height)
        {
          fb_scroll();
          fb_putc(ch);
          return;
        }

      for (gly_x = 0; gly_x < fbm->metric.width; gly_x++)
        {
          if (fb_term.cursor_x + gly_x >= fb.width)
            {
              break;
            }

          uint8_t stride = (fbm->metric.width + 7) >> 3;
          uint8_t gly_byte = stride * gly_y + (gly_x >> 3);
          uint8_t gly_bit = gly_x & 0x7;
          uint32_t color = 0;  /* Black no matter the color depth */
          if ((fbm->bitmap[gly_byte] >> (7 - gly_bit)) & 0x01)
            color = 0xffffffff;  /* Black no matter the color depth */

          fb_draw_pixel(
            color, fb_term.cursor_x + gly_x, fb_term.cursor_y + gly_y);
        }
    }

  fb_term.cursor_x += fbm->metric.width;
}
#endif  /* CONFIG_MULTBOOT2_FB_TERM */
