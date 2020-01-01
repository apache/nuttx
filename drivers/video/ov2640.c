/****************************************************************************
 * drivers/video/ov2640.c
 * OV2640 Color CMOS UXGA (2.0 MegaPixel) CameraChip
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * WARNING:  Some of the information in the data tables below came from other
 * projects with conflicting licenses:  Linux and ArduCAM.  Those both have
 * GPL licenses.  I am not sure if it is proper or not to lift the content
 * of those tables and still retain this BSD license.  I am guessing so, but
 * I am not copyright attorney so you should use this driver in products at
 * your own risk.
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

#include <sys/types.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/ov2640.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 7-bit I2C address.  Default: 0x21 */

#ifndef CONFIG_OV2640_I2CADDR
#  define CONFIG_OV2640_I2CADDR 0x21
#endif

#ifdef CONFIG_OV2640_JPEG
#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION

#else
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION
#endif

#if defined(CONFIG_OV2640_QCIF_RESOLUTION) || \
    defined(CONFIG_OV2640_JPEG_QCIF_RESOLUTION)

#  define OV2460_IMAGE_WIDTH  176
#  define OV2460_IMAGE_HEIGHT 144

#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_QVGA_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_QVGA_RESOLUTION)

#  define OV2460_IMAGE_WIDTH  320
#  define OV2460_IMAGE_HEIGHT 240

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_CIF_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_CIF_RESOLUTION)

#  define OV2460_IMAGE_WIDTH  352
#  define OV2460_IMAGE_HEIGHT  288

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_VGA_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_VGA_RESOLUTION)

#  define OV2460_IMAGE_WIDTH  640
#  define OV2460_IMAGE_HEIGHT 480

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_SVGA_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_SVGA_RESOLUTION)

#  define OV2460_IMAGE_WIDTH  800
#  define OV2460_IMAGE_HEIGHT 600

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_XVGA_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_XVGA_RESOLUTION)

#  define OV2460_IMAGE_WIDTH  1024
#  define OV2460_IMAGE_HEIGHT 768

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_SXGA_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_SXVGA_RESOLUTION)

#  define OV2460_IMAGE_WIDTH  1280
#  define OV2460_IMAGE_HEIGHT 1024

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_UXGA_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_UXGA_RESOLUTION)

#  define OV2460_IMAGE_WIDTH  1600
#  define OV2460_IMAGE_HEIGHT 1200

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION

#else
#  error Unknown Resolution
#endif

/* Chip identification */

#define OVR2640_MANUFACTURER_IDL 0xa2
#define OVR2640_MANUFACTURER_IDH 0x7f
#define OVR2640_PRODUCT_IDL      0x42
#define OVR2640_PRODUCT_IDH      0x26

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ovr2640_reg_s
{
  uint8_t regaddr;
  uint8_t regval;
};
#define ARRAY_NENTRIES(a) (sizeof(a)/sizeof(struct ovr2640_reg_s))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* OV2640 register operations */

static int     ov2640_putreg(FAR struct i2c_master_s *i2c, uint8_t regaddr,
                 uint8_t regval);
static uint8_t ov2640_getreg(FAR struct i2c_master_s *i2c, uint8_t regaddr);
static int     ov2640_putreglist(FAR struct i2c_master_s *i2c,
                 FAR const struct ovr2640_reg_s *reglist, size_t nentries);

/* Initialization */

static int     ovr2640_chipid(FAR struct i2c_master_s *i2c);
static int     ov2640_reset(FAR struct i2c_master_s *i2c);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* OV2640 reset */

static const struct ovr2640_reg_s g_ov2640_reset[] =
{
  {0xff, 0x01}, {0x12, 0x80}
};
#define OV2640_RESET_NENTRIES ARRAY_NENTRIES(g_ov2640_reset)

#ifndef CONFIG_OV2640_JPEG
/* Initial register settings */

static const struct ovr2640_reg_s g_ov2640_initialregs[] =
{
  {0xff, 0x00},  {0x2c, 0xff},  {0x2e, 0xdf},  {0xff, 0x01},  {0x3c, 0x32},
  {0x11, 0x00},  {0x09, 0x02},  {0x04, 0x28},  {0x13, 0xe5},  {0x14, 0x48},
  {0x2c, 0x0c},  {0x33, 0x78},  {0x3a, 0x33},  {0x3b, 0xfb},  {0x3e, 0x00},
  {0x43, 0x11},  {0x16, 0x10},  {0x39, 0x02},  {0x35, 0x88},  {0x22, 0x0a},
  {0x37, 0x40},  {0x23, 0x00},  {0x34, 0xa0},  {0x06, 0x02},  {0x06, 0x88},
  {0x07, 0xc0},  {0x0d, 0xb7},  {0x0e, 0x01},  {0x4c, 0x00},  {0x4a, 0x81},
  {0x21, 0x99},  {0x24, 0x40},  {0x25, 0x38},  {0x26, 0x82},  {0x5c, 0x00},
  {0x63, 0x00},  {0x46, 0x22},  {0x0c, 0x3a},  {0x5d, 0x55},  {0x5e, 0x7d},
  {0x5f, 0x7d},  {0x60, 0x55},  {0x61, 0x70},  {0x62, 0x80},  {0x7c, 0x05},
  {0x20, 0x80},  {0x28, 0x30},  {0x6c, 0x00},  {0x6d, 0x80},  {0x6e, 0x00},
  {0x70, 0x02},  {0x71, 0x94},  {0x73, 0xc1},  {0x3d, 0x34},  {0x12, 0x04},
  {0x5a, 0x57},  {0x4f, 0xbb},  {0x50, 0x9c},  {0xff, 0x00},  {0xe5, 0x7f},
  {0xf9, 0xc0},  {0x41, 0x24},  {0xe0, 0x14},  {0x76, 0xff},  {0x33, 0xa0},
  {0x42, 0x20},  {0x43, 0x18},  {0x4c, 0x00},  {0x87, 0xd0},  {0x88, 0x3f},
  {0xd7, 0x03},  {0xd9, 0x10},  {0xd3, 0x82},  {0xc8, 0x08},  {0xc9, 0x80},
  {0x7c, 0x00},  {0x7d, 0x00},  {0x7c, 0x03},  {0x7d, 0x48},  {0x7d, 0x48},
  {0x7c, 0x08},  {0x7d, 0x20},  {0x7d, 0x10},  {0x7d, 0x0e},  {0x90, 0x00},
  {0x91, 0x0e},  {0x91, 0x1a},  {0x91, 0x31},  {0x91, 0x5a},  {0x91, 0x69},
  {0x91, 0x75},  {0x91, 0x7e},  {0x91, 0x88},  {0x91, 0x8f},  {0x91, 0x96},
  {0x91, 0xa3},  {0x91, 0xaf},  {0x91, 0xc4},  {0x91, 0xd7},  {0x91, 0xe8},
  {0x91, 0x20},  {0x92, 0x00},  {0x93, 0x06},  {0x93, 0xe3},  {0x93, 0x03},
  {0x93, 0x03},  {0x93, 0x00},  {0x93, 0x02},  {0x93, 0x00},  {0x93, 0x00},
  {0x93, 0x00},  {0x93, 0x00},  {0x93, 0x00},  {0x93, 0x00},  {0x93, 0x00},
  {0x96, 0x00},  {0x97, 0x08},  {0x97, 0x19},  {0x97, 0x02},  {0x97, 0x0c},
  {0x97, 0x24},  {0x97, 0x30},  {0x97, 0x28},  {0x97, 0x26},  {0x97, 0x02},
  {0x97, 0x98},  {0x97, 0x80},  {0x97, 0x00},  {0x97, 0x00},  {0xa4, 0x00},
  {0xa8, 0x00},  {0xc5, 0x11},  {0xc6, 0x51},  {0xbf, 0x80},  {0xc7, 0x10},
  {0xb6, 0x66},  {0xb8, 0xa5},  {0xb7, 0x64},  {0xb9, 0x7c},  {0xb3, 0xaf},
  {0xb4, 0x97},  {0xb5, 0xff},  {0xb0, 0xc5},  {0xb1, 0x94},  {0xb2, 0x0f},
  {0xc4, 0x5c},  {0xa6, 0x00},  {0xa7, 0x20},  {0xa7, 0xd8},  {0xa7, 0x1b},
  {0xa7, 0x31},  {0xa7, 0x00},  {0xa7, 0x18},  {0xa7, 0x20},  {0xa7, 0xd8},
  {0xa7, 0x19},  {0xa7, 0x31},  {0xa7, 0x00},  {0xa7, 0x18},  {0xa7, 0x20},
  {0xa7, 0xd8},  {0xa7, 0x19},  {0xa7, 0x31},  {0xa7, 0x00},  {0xa7, 0x18},
  {0x7f, 0x00},  {0xe5, 0x1f},  {0xe1, 0x77},  {0xdd, 0x7f},  {0xc2, 0x0e}
};
#define OV2640_INITIALREGS_NENTRIES ARRAY_NENTRIES(g_ov2640_initialregs)

/* Resolution register settings */

static const struct ovr2640_reg_s g_ov2640_resolution_common[] =
{
  {0xff, 0x00},  {0xe0, 0x04},  {0xc0, 0xc8},  {0xc1, 0x96},  {0x86, 0x3d},
  {0x51, 0x90},  {0x52, 0x2c},  {0x53, 0x00},  {0x54, 0x00},  {0x55, 0x88},
  {0x57, 0x00}
};
#define OV2640_RESOLUTION_COMMON_NENTRIES ARRAY_NENTRIES(g_ov2640_resolution_common)

#if defined(CONFIG_OV2640_QCIF_RESOLUTION)
static const struct ovr2640_reg_s g_ov2640_qcif_resolution[] =
{
  {0x50, 0x9b},  {0x5a, 0x2c},  {0x5b, 0x24},  {0x5c, 0x00},  {0xd3, 0x04},
  {0xe0, 0x00}
};
#define OV2640_QCIF_RESOLUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_qcif_resolution)

#elif defined(CONFIG_OV2640_QVGA_RESOLUTION)
static const struct ovr2640_reg_s g_ov2640_qvga_resolution[] =
{
  {0x50, 0x92},  {0x5a, 0x50},  {0x5b, 0x3c},  {0x5c, 0x00},  {0xd3, 0x04},
  {0xe0, 0x00},
};
#define OV2640_QVGA_RESOLUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_qvga_resolution)

#elif defined(CONFIG_OV2640_CIF_RESOLUTION)
static const struct ovr2640_reg_s g_ov2640_cif_resolution[] =
{
  {0x50, 0x92},  {0x5a, 0x58},  {0x5b, 0x48},  {0x5c, 0x00},  {0xd3, 0x08},
  {0xe0, 0x00}
};
#define OV2640_CIF_RESOLUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_cif_resolution)

#elif defined(CONFIG_OV2640_VGA_RESOLUTION)
static const struct ovr2640_reg_s g_ov2640_vga_resolution[] =
{
  {0x50, 0x80},  {0x5a, 0xa0},  {0x5b, 0x78},  {0x5c, 0x00},  {0xd3, 0x02},
  {0xe0, 0x00}
};
#define OV2640_VGA_RESOLUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_vga_resolution)

#elif defined(CONFIG_OV2640_SVGA_RESOLUTION)
static const struct ovr2640_reg_s g_ov2640_svga_resolution[] =
{
  {0x50, 0x89},  {0x5a, 0xc8},  {0x5b, 0x96},  {0x5c, 0x00},  {0xd3, 0x02},
  {0xe0, 0x00}
};
#define OV2640_SVGA_RESOLUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_svga_resolution)

#elif defined(CONFIG_OV2640_XVGA_RESOLUTION)
static const struct ovr2640_reg_s g_ov2640_xga_resolution[] =
{
  {0x50, 0x80},  {0x5a, 0x00},  {0x5b, 0xc0},  {0x5c, 0x01},  {0xd3, 0x02},
  {0xe0, 0x00},  {0x50, 0x00}
};
#define OV2640_XGA_RESOLUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_xga_resolution)

#elif defined(CONFIG_OV2640_SXGA_RESOLUTION)
static const struct ovr2640_reg_s g_ov2640_sxga_resolution[] =
{
  {0x50, 0x80},  {0x5a, 0x40},  {0x5b, 0x00},  {0x5c, 0x05},  {0xd3, 0x02},
  {0xe0, 0x00},  {0x50, 0x00},  {0xd3, 0x82}
};
#define OV2640_SXGA_RESOLUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_sxga_resolution)

#elif defined(CONFIG_OV2640_UXGA_RESOLUTION)
static const struct ovr2640_reg_s g_ov2640_uxga_resolution[] =
{
  {0x50, 0x80},  {0x5a, 0x90},  {0x5b, 0x2c},  {0x5c, 0x05},  {0xd3, 0x00},
  {0xe0, 0x00},  {0x50, 0x00},  {0xd3, 0x80}
};
#define OV2640_UXGA_RESOLUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_uxga_resolution)

#else
#  error Unknown image resolution
#endif

/* Color format register settings */

static const struct ovr2640_reg_s g_ov2640_colorfmt_common[] =
{
  {0xff, 0x00},  {0x05, 0x00}
};
#define OV2640_COLORFMT_COMMON_NENTRIES ARRAY_NENTRIES(g_ov2640_colorfmt_common)

#if defined(CONFIG_OV2640_YUV422_COLORFMT)
static const struct ovr2640_reg_s g_ov2640_yuv422_colorfmt[] =
{
  {0xda, 0x01},  {0xd7, 0x01},  {0x33, 0xa0},  {0xe1, 0x67},  {0xe0, 0x00},
  {0x05, 0x00}
};
#define OV2640_YUV422_COLORFMT_NENTRIES ARRAY_NENTRIES(g_ov2640_yuv422_colorfmt)

#elif defined(CONFIG_OV2640_RGB565_COLORFMT)
static const struct ovr2640_reg_s g_ov2640_rgb565_colorfmt[] =
{
  {0xda, 0x09},  {0xd7, 0x03},  {0xe0, 0x00},  {0x05, 0x00}
};
#define OV2640_RGB565_COLORFMT_NENTRIES ARRAY_NENTRIES(g_ov2640_rgb565_colorfmt)

#else
#  error Unknown color format
#endif
#endif /* !CONFIG_OV2640_JPEG */

#ifdef CONFIG_OV2640_JPEG
static const struct ovr2640_reg_s g_ov2640_jpeg_init[] =
{
  {0xff, 0x00},  {0x2c, 0xff},  {0x2e, 0xdf},  {0xff, 0x01},  {0x3c, 0x32},
  {0x11, 0x04},  {0x09, 0x02},  {0x04, 0x28},  {0x13, 0xe5},  {0x14, 0x48},
  {0x2c, 0x0c},  {0x33, 0x78},  {0x3a, 0x33},  {0x3b, 0xfb},  {0x3e, 0x00},
  {0x43, 0x11},  {0x16, 0x10},  {0x39, 0x92},  {0x35, 0xda},  {0x22, 0x1a},
  {0x37, 0xc3},  {0x23, 0x00},  {0x34, 0xc0},  {0x36, 0x1a},  {0x06, 0x88},
  {0x07, 0xc0},  {0x0d, 0x87},  {0x0e, 0x41},  {0x4c, 0x00},  {0x48, 0x00},
  {0x5b, 0x00},  {0x42, 0x03},  {0x4a, 0x81},  {0x21, 0x99},  {0x24, 0x40},
  {0x25, 0x38},  {0x26, 0x82},  {0x5c, 0x00},  {0x63, 0x00},  {0x61, 0x70},
  {0x62, 0x80},  {0x7c, 0x05},  {0x20, 0x80},  {0x28, 0x30},  {0x6c, 0x00},
  {0x6d, 0x80},  {0x6e, 0x00},  {0x70, 0x02},  {0x71, 0x94},  {0x73, 0xc1},
  {0x12, 0x40},  {0x17, 0x11},  {0x18, 0x43},  {0x19, 0x00},  {0x1a, 0x4b},
  {0x32, 0x09},  {0x37, 0xc0},  {0x4f, 0x60},  {0x50, 0xa8},  {0x6d, 0x00},
  {0x3d, 0x38},  {0x46, 0x3f},  {0x4f, 0x60},  {0x0c, 0x3c},  {0xff, 0x00},
  {0xe5, 0x7f},  {0xf9, 0xc0},  {0x41, 0x24},  {0xe0, 0x14},  {0x76, 0xff},
  {0x33, 0xa0},  {0x42, 0x20},  {0x43, 0x18},  {0x4c, 0x00},  {0x87, 0xd5},
  {0x88, 0x3f},  {0xd7, 0x03},  {0xd9, 0x10},  {0xd3, 0x82},  {0xc8, 0x08},
  {0xc9, 0x80},  {0x7c, 0x00},  {0x7d, 0x00},  {0x7c, 0x03},  {0x7d, 0x48},
  {0x7d, 0x48},  {0x7c, 0x08},  {0x7d, 0x20},  {0x7d, 0x10},  {0x7d, 0x0e},
  {0x90, 0x00},  {0x91, 0x0e},  {0x91, 0x1a},  {0x91, 0x31},  {0x91, 0x5a},
  {0x91, 0x69},  {0x91, 0x75},  {0x91, 0x7e},  {0x91, 0x88},  {0x91, 0x8f},
  {0x91, 0x96},  {0x91, 0xa3},  {0x91, 0xaf},  {0x91, 0xc4},  {0x91, 0xd7},
  {0x91, 0xe8},  {0x91, 0x20},  {0x92, 0x00},  {0x93, 0x06},  {0x93, 0xe3},
  {0x93, 0x05},  {0x93, 0x05},  {0x93, 0x00},  {0x93, 0x04},  {0x93, 0x00},
  {0x93, 0x00},  {0x93, 0x00},  {0x93, 0x00},  {0x93, 0x00},  {0x93, 0x00},
  {0x93, 0x00},  {0x96, 0x00},  {0x97, 0x08},  {0x97, 0x19},  {0x97, 0x02},
  {0x97, 0x0c},  {0x97, 0x24},  {0x97, 0x30},  {0x97, 0x28},  {0x97, 0x26},
  {0x97, 0x02},  {0x97, 0x98},  {0x97, 0x80},  {0x97, 0x00},  {0x97, 0x00},
  {0xc3, 0xed},  {0xa4, 0x00},  {0xa8, 0x00},  {0xc5, 0x11},  {0xc6, 0x51},
  {0xbf, 0x80},  {0xc7, 0x10},  {0xb6, 0x66},  {0xb8, 0xa5},  {0xb7, 0x64},
  {0xb9, 0x7c},  {0xb3, 0xaf},  {0xb4, 0x97},  {0xb5, 0xff},  {0xb0, 0xc5},
  {0xb1, 0x94},  {0xb2, 0x0f},  {0xc4, 0x5c},  {0xc0, 0x64},  {0xc1, 0x4b},
  {0x8c, 0x00},  {0x86, 0x3d},  {0x50, 0x00},  {0x51, 0xc8},  {0x52, 0x96},
  {0x53, 0x00},  {0x54, 0x00},  {0x55, 0x00},  {0x5a, 0xc8},  {0x5b, 0x96},
  {0x5c, 0x00},  {0xd3, 0x00},  {0xc3, 0xed},  {0x7f, 0x00},  {0xda, 0x00},
  {0xe5, 0x1f},  {0xe1, 0x67},  {0xe0, 0x00},  {0xdd, 0x7f},  {0x05, 0x00},
  {0x12, 0x40},  {0xd3, 0x04},  {0xc0, 0x16},  {0xc1, 0x12},  {0x8c, 0x00},
  {0x86, 0x3d},  {0x50, 0x00},  {0x51, 0x2c},  {0x52, 0x24},  {0x53, 0x00},
  {0x54, 0x00},  {0x55, 0x00},  {0x5a, 0x2c},  {0x5b, 0x24},  {0x5c, 0x00},
};

#define OV2640_JPEG_INIT_NENTRIES ARRAY_NENTRIES(g_ov2640_jpeg_init)
#endif

#ifdef CONFIG_OV2640_JPEG
static const struct ovr2640_reg_s g_ov2640_yuv422[] =
{
  {0xff, 0x00},  {0x05, 0x00},  {0xda, 0x10},  {0xd7, 0x03},  {0xdf, 0x00},
  {0x33, 0x80},  {0x3c, 0x40},  {0xe1, 0x77},  {0x00, 0x00}
};

#define OV2640_YUV422_NENTRIES ARRAY_NENTRIES(g_ov2640_yuv422)
#endif

#ifdef CONFIG_OV2640_JPEG
static const struct ovr2640_reg_s g_ov2640_jpeg[] =
{
  {0xe0, 0x14},  {0xe1, 0x77},  {0xe5, 0x1f},  {0xd7, 0x03},  {0xda, 0x10},
  {0xe0, 0x00},  {0xff, 0x01},  {0x04, 0x08}
};

#define OV2640_JPEG_NENTRIES ARRAY_NENTRIES(g_ov2640_jpeg)
#endif

/* JPEG QCIF 176x144 */

#ifdef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
static const struct ovr2640_reg_s g_ov2640_jpeg_qcif_resolution[] =
{
  {0xff, 0x01},  {0x12, 0x40},  {0x17, 0x11},  {0x18, 0x43},  {0x19, 0x00},
  {0x1a, 0x4b},  {0x32, 0x09},  {0x4f, 0xca},  {0x50, 0xa8},  {0x5a, 0x23},
  {0x6d, 0x00},  {0x39, 0x12},  {0x35, 0xda},  {0x22, 0x1a},  {0x37, 0xc3},
  {0x23, 0x00},  {0x34, 0xc0},  {0x36, 0x1a},  {0x06, 0x88},  {0x07, 0xc0},
  {0x0d, 0x87},  {0x0e, 0x41},  {0x4c, 0x00},  {0xff, 0x00},  {0xe0, 0x04},
  {0xc0, 0x64},  {0xc1, 0x4b},  {0x86, 0x35},  {0x50, 0x92},  {0x51, 0xc8},
  {0x52, 0x96},  {0x53, 0x00},  {0x54, 0x00},  {0x55, 0x00},  {0x57, 0x00},
  {0x5a, 0x2c},  {0x5b, 0x24},  {0x5c, 0x00},  {0xe0, 0x00}
};

#define OV2640_JPEG_QCIF_RESOUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_jpeg_qcif_resolution)
#endif

/* JPEG QVGA 320x240 */

#ifdef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
static const struct ovr2640_reg_s g_ov2640_jpeg_qvga_resolution[] =
{
  {0xff, 0x01},  {0x12, 0x40},  {0x17, 0x11},  {0x18, 0x43},  {0x19, 0x00},
  {0x1a, 0x4b},  {0x32, 0x09},  {0x4f, 0xca},  {0x50, 0xa8},  {0x5a, 0x23},
  {0x6d, 0x00},  {0x39, 0x12},  {0x35, 0xda},  {0x22, 0x1a},  {0x37, 0xc3},
  {0x23, 0x00},  {0x34, 0xc0},  {0x36, 0x1a},  {0x06, 0x88},  {0x07, 0xc0},
  {0x0d, 0x87},  {0x0e, 0x41},  {0x4c, 0x00},  {0xff, 0x00},  {0xe0, 0x04},
  {0xc0, 0x64},  {0xc1, 0x4b},  {0x86, 0x35},  {0x50, 0x89},  {0x51, 0xc8},
  {0x52, 0x96},  {0x53, 0x00},  {0x54, 0x00},  {0x55, 0x00},  {0x57, 0x00},
  {0x5a, 0x50},  {0x5b, 0x3c},  {0x5c, 0x00},  {0xe0, 0x00}
};

#define OV2640_JPEG_QVGA_RESOUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_jpeg_qvga_resolution)
#endif

/* JPEG CIF 352x288 */

#ifdef CONFIG_OV2640_JPEG_CIF_RESOLUTION
static const struct ovr2640_reg_s g_ov2640_jpeg_cif_resolution[] =
{
  {0xff, 0x01},  {0x12, 0x40},  {0x17, 0x11},  {0x18, 0x43},  {0x19, 0x00},
  {0x1a, 0x4b},  {0x32, 0x09},  {0x4f, 0xca},  {0x50, 0xa8},  {0x5a, 0x23},
  {0x6d, 0x00},  {0x39, 0x12},  {0x35, 0xda},  {0x22, 0x1a},  {0x37, 0xc3},
  {0x23, 0x00},  {0x34, 0xc0},  {0x36, 0x1a},  {0x06, 0x88},  {0x07, 0xc0},
  {0x0d, 0x87},  {0x0e, 0x41},  {0x4c, 0x00},  {0xff, 0x00},  {0xe0, 0x04},
  {0xc0, 0x64},  {0xc1, 0x4b},  {0x86, 0x35},  {0x50, 0x89},  {0x51, 0xc8},
  {0x52, 0x96},  {0x53, 0x00},  {0x54, 0x00},  {0x55, 0x00},  {0x57, 0x00},
  {0x5a, 0x58},  {0x5b, 0x48},  {0x5c, 0x00},  {0xe0, 0x00}
};

#define OV2640_JPEG_CIF_RESOUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_jpeg_cif_resolution)
#endif

/* JPEG VGA 640x480 */

#ifdef CONFIG_OV2640_JPEG_VGA_RESOLUTION
static const struct ovr2640_reg_s g_ov2640_jpeg_vga_resolution[] =
{
  {0xff, 0x01},  {0x11, 0x01},  {0x12, 0x00},  {0x17, 0x11},  {0x18, 0x75},
  {0x32, 0x36},  {0x19, 0x01},  {0x1a, 0x97},  {0x03, 0x0f},  {0x37, 0x40},
  {0x4f, 0xbb},  {0x50, 0x9c},  {0x5a, 0x57},  {0x6d, 0x80},  {0x3d, 0x34},
  {0x39, 0x02},  {0x35, 0x88},  {0x22, 0x0a},  {0x37, 0x40},  {0x34, 0xa0},
  {0x06, 0x02},  {0x0d, 0xb7},  {0x0e, 0x01},  {0xff, 0x00},  {0xe0, 0x04},
  {0xc0, 0xc8},  {0xc1, 0x96},  {0x86, 0x3d},  {0x50, 0x89},  {0x51, 0x90},
  {0x52, 0x2c},  {0x53, 0x00},  {0x54, 0x00},  {0x55, 0x88},  {0x57, 0x00},
  {0x5a, 0xa0},  {0x5b, 0x78},  {0x5c, 0x00},  {0xd3, 0x04},  {0xe0, 0x00}
};

#define OV2640_JPEG_VGA_RESOUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_jpeg_vga_resolution)
#endif

/* JPEG SVGA 800x600 */

#ifdef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
static const struct ovr2640_reg_s g_ov2640_jpeg_svga_resolution[] =
{
  {0xff, 0x01},  {0x11, 0x01},  {0x12, 0x00},  {0x17, 0x11},  {0x18, 0x75},
  {0x32, 0x36},  {0x19, 0x01},  {0x1a, 0x97},  {0x03, 0x0f},  {0x37, 0x40},
  {0x4f, 0xbb},  {0x50, 0x9c},  {0x5a, 0x57},  {0x6d, 0x80},  {0x3d, 0x34},
  {0x39, 0x02},  {0x35, 0x88},  {0x22, 0x0a},  {0x37, 0x40},  {0x34, 0xa0},
  {0x06, 0x02},  {0x0d, 0xb7},  {0x0e, 0x01},  {0xff, 0x00},  {0xe0, 0x04},
  {0xc0, 0xc8},  {0xc1, 0x96},  {0x86, 0x35},  {0x50, 0x89},  {0x51, 0x90},
  {0x52, 0x2c},  {0x53, 0x00},  {0x54, 0x00},  {0x55, 0x88},  {0x57, 0x00},
  {0x5a, 0xc8},  {0x5b, 0x96},  {0x5c, 0x00},  {0xd3, 0x02},  {0xe0, 0x00}
};

#define OV2640_JPEG_SVGA_RESOUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_jpeg_svga_resolution)
#endif

/* JPEG 1024x768 */

#ifdef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
static const struct ovr2640_reg_s g_ov2640_jpeg_xvga_resolution[] =
{
  {0xff, 0x01},  {0x11, 0x01},  {0x12, 0x00},  {0x17, 0x11},  {0x18, 0x75},
  {0x32, 0x36},  {0x19, 0x01},  {0x1a, 0x97},  {0x03, 0x0f},  {0x37, 0x40},
  {0x4f, 0xbb},  {0x50, 0x9c},  {0x5a, 0x57},  {0x6d, 0x80},  {0x3d, 0x34},
  {0x39, 0x02},  {0x35, 0x88},  {0x22, 0x0a},  {0x37, 0x40},  {0x34, 0xa0},
  {0x06, 0x02},  {0x0d, 0xb7},  {0x0e, 0x01},  {0xff, 0x00},  {0xc0, 0xc8},
  {0xc1, 0x96},  {0x8c, 0x00},  {0x86, 0x3d},  {0x50, 0x00},  {0x51, 0x90},
  {0x52, 0x2c},  {0x53, 0x00},  {0x54, 0x00},  {0x55, 0x88},  {0x5a, 0x00},
  {0x5b, 0xc0},  {0x5c, 0x01},  {0xd3, 0x02}
};

#define OV2640_JPEG_XVGA_RESOUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_jpeg_xvga_resolution)
#endif

/* JPEG 1280x1024 */

#ifdef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
static const struct ovr2640_reg_s g_ov2640_jpeg_sxvga_resolution[] =
{
  {0xff, 0x01},  {0x11, 0x01},  {0x12, 0x00},  {0x17, 0x11},  {0x18, 0x75},
  {0x32, 0x36},  {0x19, 0x01},  {0x1a, 0x97},  {0x03, 0x0f},  {0x37, 0x40},
  {0x4f, 0xbb},  {0x50, 0x9c},  {0x5a, 0x57},  {0x6d, 0x80},  {0x3d, 0x34},
  {0x39, 0x02},  {0x35, 0x88},  {0x22, 0x0a},  {0x37, 0x40},  {0x34, 0xa0},
  {0x06, 0x02},  {0x0d, 0xb7},  {0x0e, 0x01},  {0xff, 0x00},  {0xe0, 0x04},
  {0xc0, 0xc8},  {0xc1, 0x96},  {0x86, 0x3d},  {0x50, 0x00},  {0x51, 0x90},
  {0x52, 0x2c},  {0x53, 0x00},  {0x54, 0x00},  {0x55, 0x88},  {0x57, 0x00},
  {0x5a, 0x40},  {0x5b, 0xf0},  {0x5c, 0x01},  {0xd3, 0x02},  {0xe0, 0x00}
};

#define OV2640_JPEG_SXVGA_RESOUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_jpeg_sxvga_resolution)
#endif

/* JPEG 1600x1200 */

#ifdef CONFIG_OV2640_JPEG_UXGA_RESOLUTION
static const struct ovr2640_reg_s g_ov2640_jpeg_uxga_resolution[] =
{
  {0xff, 0x01},  {0x11, 0x01},  {0x12, 0x00},  {0x17, 0x11},  {0x18, 0x75},
  {0x32, 0x36},  {0x19, 0x01},  {0x1a, 0x97},  {0x03, 0x0f},  {0x37, 0x40},
  {0x4f, 0xbb},  {0x50, 0x9c},  {0x5a, 0x57},  {0x6d, 0x80},  {0x3d, 0x34},
  {0x39, 0x02},  {0x35, 0x88},  {0x22, 0x0a},  {0x37, 0x40},  {0x34, 0xa0},
  {0x06, 0x02},  {0x0d, 0xb7},  {0x0e, 0x01},  {0xff, 0x00},  {0xe0, 0x04},
  {0xc0, 0xc8},  {0xc1, 0x96},  {0x86, 0x3d},  {0x50, 0x00},  {0x51, 0x90},
  {0x52, 0x2c},  {0x53, 0x00},  {0x54, 0x00},  {0x55, 0x88},  {0x57, 0x00},
  {0x5a, 0x90},  {0x5b, 0x2c},  {0x5c, 0x05},  {0xd3, 0x02},  {0xe0, 0x00}
};

#define OV2640_JPEG_UXGA_RESOUTION_NENTRIES ARRAY_NENTRIES(g_ov2640_jpeg_uxga_resolution)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov2640_putreg
 *
 * Description:
 *   Set one OV2640 register
 *
 * Input Parameters:
 *   i2c - Reference to the I2C driver structure
 *   regaddr - The address of the OV2640 register to set
 *   regval - The new value of the register
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int ov2640_putreg(FAR struct i2c_master_s *i2c, uint8_t regaddr,
                         uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

#ifdef CONFIG_OV2640_REGDEBUG
   _err("%02x <- %02x\n", regaddr, regval);
#endif

  /* Set up for the transfer */

  buffer[0] = regaddr; /* Register address */
  buffer[1] = regval;  /* New register value */

  /* Set up the I2C configuration */

  config.frequency = CONFIG_OV2640_FREQUENCY;
  config.address   = CONFIG_OV2640_I2CADDR;
  config.addrlen   = 7;

  /* And do it */

  ret = i2c_write(i2c, &config, buffer, 2);
  if (ret < 0)
    {
      gerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ov2640_getreg
 *
 * Description:
 *   Set one OV2640 register
 *
 * Input Parameters:
 *   i2c - Reference to the I2C driver structure
 *   regaddr - The address of the OV2640 register to set
 *   regval - The new value of the register
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static uint8_t ov2640_getreg(FAR struct i2c_master_s *i2c, uint8_t regaddr)
{
  struct i2c_config_s config;
  uint8_t regval;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_OV2640_FREQUENCY;
  config.address   = CONFIG_OV2640_I2CADDR;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      gerr("ERROR: i2c_write failed: %d\n", ret);
      return 0;
    }

  /* Restart and read 8-bits from the register */

  ret = i2c_read(i2c, &config, &regval, 1);
  if (ret < 0)
    {
      gerr("ERROR: i2c_read failed: %d\n", ret);
      return 0;
    }
#ifdef CONFIG_OV2640_REGDEBUG
  else
    {
      _err("%02x -> %02x\n", regaddr, regval);
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: ov2640_putreglist
 *
 * Description:
 *   Set a list of OV2640 register values.
 *
 * Input Parameters:
 *   i2c - Reference to the I2C driver structure
 *   reglist - The address of list of OV2640 register settings
 *   nentries - The number of entries in the list
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int ov2640_putreglist(FAR struct i2c_master_s *i2c,
                             FAR const struct ovr2640_reg_s *reglist,
                             size_t nentries)
{
  FAR const struct ovr2640_reg_s *entry;
  int ret;

  for (entry = reglist; nentries > 0; nentries--, entry++)
    {
      ret = ov2640_putreg(i2c, entry->regaddr, entry->regval);
      if (ret < 0)
        {
          gerr("ERROR: ov2640_putreg failed: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ovr2640_chipid
 *
 * Description:
 *   Read and verify the CHIP ID
 *
 * Input Parameters:
 *   i2c - Reference to the I2C driver structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int ovr2640_chipid(FAR struct i2c_master_s *i2c)
{
  uint8_t pidl;
  uint8_t pidh;
#ifdef CONFIG_DEBUG_GRAPHICS
  uint8_t midh;
  uint8_t midl;
#endif
  int ret;

  /* Check and show product ID and manufacturer ID */

  ret = ov2640_putreg(i2c, 0xff, 0x01); /* Select the sensor address bank */
  if (ret < 0)
    {
      gerr("ERROR: ov2640_putreg failed: %d\n", ret);
      return ret;
    }

  pidl = ov2640_getreg(i2c, 0x0a);      /* Product ID (MS) */
  pidh = ov2640_getreg(i2c, 0x0b);      /* Product ID (LS) */

#ifdef CONFIG_DEBUG_GRAPHICS
  midh = ov2640_getreg(i2c, 0x1c); /* Manufacturer ID (high) = 0x7f */
  midl = ov2640_getreg(i2c, 0x1d); /* Manufacturer ID (low) = 0xa2 */
#endif

  if (pidl != OVR2640_PRODUCT_IDL || pidh != OVR2640_PRODUCT_IDH)
    {
      gerr("ERROR: Unsupported PID=%02x$02x MID=%02x%02x\n",
            pidh, pidl, midh, midl);
      return -ENOSYS;
    }

  ginfo("PID=%02x$02x MID=%02x%02x\n", pidh, pidl, midh, midl);
  return OK;
}

/****************************************************************************
 * Name: ov2640_reset
 *
 * Description:
 *   Reset the OV2640.
 *
 * Input Parameters:
 *   i2c - Reference to the I2C driver structure
 *   reglist - The address of list of OV2640 register settings
 *   nentries - The number of entries in the list
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int ov2640_reset(FAR struct i2c_master_s *i2c)
{
  int ret;

  ret = ov2640_putreglist(i2c, g_ov2640_reset, OV2640_RESET_NENTRIES);
  if (ret < 0)
    {
      gerr("ERROR: ov2640_putreglist failed: %d\n", ret);
      return ret;
    }

  up_mdelay(5);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: ov2640_initialize
 *
 * Description:
 *   Initialize the OV2640 camera.
 *
 * Input Parameters:
 *   i2c - Reference to the I2C driver structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int ov2640_initialize(FAR struct i2c_master_s *i2c)
{
  int ret;

  /* Reset the OVR2640 */

  ret = ov2640_reset(i2c);
  if (ret < 0)
    {
      gerr("ERROR: ov2640_reset failed: %d\n", ret);
      goto errout;
    }

  /* Check the CHIP ID */

  ret = ovr2640_chipid(i2c);
  if (ret < 0)
    {
      gerr("ERROR: ovr2640_chipid failed: %d\n", ret);
      goto errout;
    }

  /* Initialize the OV2640 hardware */

#ifdef CONFIG_OV2640_JPEG
  /* Initialize for JPEG output */

  ret = ov2640_putreglist(i2c, g_ov2640_jpeg_init, OV2640_JPEG_INIT_NENTRIES);
  if (ret < 0)
    {
      gerr("ERROR: ov2640_putreglist failed: %d\n", ret);
      goto errout;
    }

  ret = ov2640_putreglist(i2c, g_ov2640_yuv422, OV2640_YUV422_NENTRIES);
  if (ret < 0)
    {
      gerr("ERROR: ov2640_putreglist failed: %d\n", ret);
      goto errout;
    }

  ret = ov2640_putreglist(i2c, g_ov2640_jpeg, OV2640_JPEG_NENTRIES);
  if (ret < 0)
    {
      gerr("ERROR: ov2640_putreglist failed: %d\n", ret);
      goto errout;
    }

  ret = ov2640_putreg(i2c, 0xff, 0x01);
  if (ret < 0)
    {
      gerr("ERROR: ov2640_putreg failed: %d\n", ret);
      goto errout;
    }

  ret = ov2640_putreg(i2c, 0x15, 0x00);
  if (ret < 0)
    {
      gerr("ERROR: ov2640_putreg failed: %d\n", ret);
      goto errout;
    }

#if defined(CONFIG_OV2640_JPEG_QCIF_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_jpeg_qcif_resolution,
                          OV2640_JPEG_QCIF_RESOUTION_NENTRIES);

#elif defined(CONFIG_OV2640_JPEG_QVGA_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_jpeg_qvga_resolution,
                          OV2640_JPEG_QVGA_RESOUTION_NENTRIES);

#elif defined(CONFIG_OV2640_JPEG_CIF_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_jpeg_cif_resolution,
                          OV2640_JPEG_CIF_RESOUTION_NENTRIES);

#elif defined(CONFIG_OV2640_JPEG_VGA_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_jpeg_vga_resolution,
                          OV2640_JPEG_VGA_RESOUTION_NENTRIES);

#elif defined(CONFIG_OV2640_JPEG_SVGA_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_jpeg_svga_resolution,
                          OV2640_JPEG_SVGA_RESOUTION_NENTRIES);

#elif defined(CONFIG_OV2640_JPEG_XVGA_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_jpeg_xvga_resolution,
                          OV2640_JPEG_XVGA_RESOUTION_NENTRIES);

#elif defined(CONFIG_OV2640_JPEG_SXVGA_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_jpeg_sxvga_resolution,
                          OV2640_JPEG_SXVGA_RESOUTION_NENTRIES);

#elif defined(CONFIG_OV2640_JPEG_UXGA_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_jpeg_uxga_resolution,
                          OV2640_JPEG_UXGA_RESOUTION_NENTRIES);

#else
#  error Unspecified JPEG resolution
#endif

  if (ret < 0)
    {
      gerr("ERROR: ov2640_putreglist failed: %d\n", ret);
      goto errout;
    }

#else /* CONFIG_OV2640_JPEG */

  /* Setup initial register values */

  ret = ov2640_putreglist(i2c, g_ov2640_initialregs,
                          OV2640_INITIALREGS_NENTRIES);
  if (ret < 0)
    {
      gerr("ERROR: ov2640_putreglist failed: %d\n", ret);
      goto errout;
    }

  /* Setup image resolution */

  ret = ov2640_putreglist(i2c, g_ov2640_resolution_common,
                          OV2640_RESOLUTION_COMMON_NENTRIES);
  if (ret < 0)
    {
      gerr("ERROR: ov2640_putreglist failed: %d\n", ret);
      goto errout;
    }

#if defined(CONFIG_OV2640_QCIF_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_qcif_resolution,
                          OV2640_QCIF_RESOLUTION_NENTRIES);

#elif defined(CONFIG_OV2640_QVGA_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_qvga_resolution,
                          OV2640_QVGA_RESOLUTION_NENTRIES);

#elif defined(CONFIG_OV2640_CIF_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_cif_resolution,
                          OV2640_CIF_RESOLUTION_NENTRIES);

#elif defined(CONFIG_OV2640_VGA_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_vga_resolution,
                          OV2640_VGA_RESOLUTION_NENTRIES);

#elif defined(CONFIG_OV2640_SVGA_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_svga_resolution,
                          OV2640_SVGA_RESOLUTION_NENTRIES);

#elif defined(CONFIG_OV2640_XGA_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_xga_resolution,
                          OV2640_XGA_RESOLUTION_NENTRIES);

#elif defined(CONFIG_OV2640_SXGA_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_sxga_resolution,
                          OV2640_SXGA_RESOLUTION_NENTRIES);

#elif defined(CONFIG_OV2640_UXGA_RESOLUTION)
  ret = ov2640_putreglist(i2c, g_ov2640_uxga_resolution,
                          OV2640_UXGA_RESOLUTION_NENTRIES);

#else
#  error Unknown image resolution
#endif

  if (ret < 0)
    {
      gerr("ERROR: ov2640_putreglist failed: %d\n", ret);
      goto errout;
    }

/* Color format register settings */

  ret = ov2640_putreglist(i2c, g_ov2640_colorfmt_common,
                    OV2640_COLORFMT_COMMON_NENTRIES);
  if (ret < 0)
    {
      gerr("ERROR: ov2640_putreglist failed: %d\n", ret);
      goto errout;
    }

#if defined(CONFIG_OV2640_YUV422_COLORFMT)
  ret = ov2640_putreglist(i2c, g_ov2640_yuv422_colorfmt,
                    OV2640_YUV422_COLORFMT_NENTRIES);

#elif defined(CONFIG_OV2640_RGB565_COLORFMT)
  ret = ov2640_putreglist(i2c, g_ov2640_rgb565_colorfmt,
                    OV2640_RGB565_COLORFMT_NENTRIES);

#else
#  error Unknown color format
#endif

  if (ret < 0)
    {
      gerr("ERROR: ov2640_putreglist failed: %d\n", ret);
      goto errout;
    }

#endif /* CONFIG_OV2640_JPEG */

  return OK;

errout:
  gerr("ERROR: Failed to initialize the OV2640: %d\n", ret);
  ov2640_reset(i2c);
  return ret;
}
