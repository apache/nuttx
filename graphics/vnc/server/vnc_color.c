/****************************************************************************
 * graphics/vnc/vnc_color.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include "vnc_server.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_convert_rgbNN
 *
 * Description:
 *  Convert the native framebuffer color format (either RGB16 5:6:5 or RGB32
 *  8:8:8) to the remote framebuffer color format (either RGB16 5:6:5,
 *  RGB16 5:5:5, or RGB32 8:8:)
 *
 * Input Parameters:
 *   pixel - The src color in local framebuffer format.
 *
 * Returned Value:
 *   The pixel in the remote framebuffer color format.
 *
 ****************************************************************************/

#if defined(CONFIG_VNCSERVER_COLORFMT_RGB16)

uint8_t vnc_convert_rgb8_222(lfb_color_t rgb)
{
  /* 111111
   * 54321098 76543210
   * -----------------
   * RRRRRGGG GGGBBBBB
   *          ..RRGGBB
   */

  return (uint8_t)(((rgb >> 10) & 0x0030)  |
                   ((rgb >> 7)  & 0x000c)  |
                   ((rgb >> 3)  & 0x0003));
}

uint8_t vnc_convert_rgb8_332(lfb_color_t rgb)
{
  /* 111111
   * 54321098 76543210
   * -----------------
   * RRRRRGGG GGGBBBBB
   *          RRRGGGBB
   */

  return (uint8_t)(((rgb >> 8) & 0x0070)  |
                   ((rgb >> 6) & 0x001c)  |
                   ((rgb >> 3) & 0x0003));
}

uint16_t vnc_convert_rgb16_555(lfb_color_t rgb)
{
  /* 111111
   * 54321098 76543210
   * -----------------
   * RRRRRGGG GGGBBBBB
   * .RRRRRGG GGGBBBBB
   */

  return (((rgb >> 1) & ~0x1f) | (rgb & 0x1f));
}

uint16_t vnc_convert_rgb16_565(lfb_color_t rgb)
{
  /* Identity mapping */

  return rgb;
}

uint32_t vnc_convert_rgb32_888(lfb_color_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * ----------------------------------
   *                   RRRRRGGG GGGBBBBB
   *          RRRRR... GGGGGG.. BBBBB...
   */

  return (((uint32_t)rgb << 8) & 0x00f80000) |
         (((uint32_t)rgb << 6) & 0x0000fc00) |
         (((uint32_t)rgb << 3) & 0x000000f8);
}

#elif defined(CONFIG_VNCSERVER_COLORFMT_RGB32)

uint8_t vnc_convert_rgb8_222(lfb_color_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * -----------------------------------
   *          RRRRRRRR GGGGGGGG BBBBBBBB
   *                            ..RRGGBB
   */

  return (uint8_t)(((rgb >> 18) & 0x0030)  |
                   ((rgb >> 12) & 0x000c)  |
                    (rgb >> 6)  & 0x0003));
}

uint8_t vnc_convert_rgb8_332(lfb_color_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * -----------------------------------
   *          RRRRRRRR GGGGGGGG BBBBBBBB
   *                            RRRGGGBB
   */

  return (uint8_t)(((rgb >> 16) & 0x0070)  |
                   ((rgb >> 11) & 0x001c)  |
                    (rgb >> 6)  & 0x0003));
}

uint16_t vnc_convert_rgb16_555(lfb_color_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * -----------------------------------
   *          RRRRRRRR GGGGGGGG BBBBBBBB
   *                   .RRRRRGG GGGBBBBB
   */

  return (uint16_t)
    (((rgb >> 9) & 0x00007c00) |
     ((rgb >> 6) & 0x000003e0) |
     ((rgb >> 3) & 0x0000001f));
}

uint16_t vnc_convert_rgb16_565(lfb_color_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * -----------------------------------
   *          RRRRRRRR GGGGGGGG BBBBBBBB
   *                   RRRRRGGG GGGBBBBB
   */

  return (uint16_t)
    (((rgb >> 8) & 0x0000f800) |
     ((rgb >> 5) & 0x000007e0) |
     ((rgb >> 3) & 0x0000001f));
}

uint32_t vnc_convert_rgb32_888(lfb_color_t rgb)
{
  /* Identity mapping */

  return rgb;
}
#else
#  error Unspecified/unsupported color format
#endif
