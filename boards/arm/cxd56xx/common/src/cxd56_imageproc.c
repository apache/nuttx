/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_imageproc.c
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

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <debug.h>

#include <arch/chip/ge2d.h>
#include <arch/chip/chip.h>
#include <arch/board/cxd56_imageproc.h>

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_IMAGEPROC_GEDEVNAME
#define GEDEVNAME CONFIG_IMAGEPROC_GEDEVNAME
#else
#define GEDEVNAME "/dev/ge"
#endif

#define CXD56_ROT_BASE      (CXD56_ADSP_BASE + 0x02101400)
#define ROT_INTR_STATUS     (CXD56_ROT_BASE  + 0x0000)
#define ROT_INTR_ENABLE     (CXD56_ROT_BASE  + 0x0004)
#define ROT_INTR_DISABLE    (CXD56_ROT_BASE  + 0x0008)
#define ROT_INTR_CLEAR      (CXD56_ROT_BASE  + 0x000C)
#define ROT_SET_DIRECTION   (CXD56_ROT_BASE  + 0x0014)
#define ROT_SET_SRC_HSIZE   (CXD56_ROT_BASE  + 0x0018)
#define ROT_SET_SRC_VSIZE   (CXD56_ROT_BASE  + 0x001C)
#define ROT_SET_SRC_ADDRESS (CXD56_ROT_BASE  + 0x0020)
#define ROT_SET_SRC_PITCH   (CXD56_ROT_BASE  + 0x0024)
#define ROT_SET_DST_ADDRESS (CXD56_ROT_BASE  + 0x0028)
#define ROT_SET_DST_PITCH   (CXD56_ROT_BASE  + 0x002C)
#define ROT_CONV_CTRL       (CXD56_ROT_BASE  + 0x0034)
#define ROT_RGB_ALIGNMENT   (CXD56_ROT_BASE  + 0x0038)
#define ROT_COMMAND         (CXD56_ROT_BASE  + 0x0010)

#define MSEL     1

/* limit size */

#define HSIZE_MIN           (12)
#define VSIZE_MIN           (12)
#define HSIZE_MAX           (4096)      /* Not used scaler(ISE) */
#define VSIZE_MAX           (4096)      /* Not used scaler(ISE) */
#define ISE_SRC_HSIZE_MAX   (2880)
#define ISE_SRC_VSIZE_MAX   (2160)
#define ISE_DST_HSIZE_MAX   (768)
#define ISE_DST_VSIZE_MAX   (1024)
#define MAX_RATIO           (64)

/* Command code */

#define COPYCMD  0x4
#define ROPCMD   0x8
#define ABCMD    0xa

/* Command options */

#define SRC16BPP (1 << 10)
#define SCALING  (1 << 12)
#define PATMONO  (1 << 15)

/* Raster operation code */

#define SRCCOPY     0xcc
#define SRCPAINT    0xee
#define SRCAND      0x88
#define SRCINVERT   0x66
#define SRCERASE    0x44
#define NOTSRCCOPY  0x33
#define NOTSRCERASE 0x11
#define MARGECOPY   0xc0
#define MERGEPAINT  0xbb
#define PATCOPY     0xf0
#define PATPAINT    0xfb
#define PATINVERT   0x5a
#define DSTINVERT   0x55

/* Raster operation options */

#define CONV8BPP    (1 << 7)
#define FIXEDCOLOR  (1 << 3)
#define CMPDST      (3 << 1)
#define CMPSRC      (2 << 1)
#define CMPPAT      (1 << 1)
#define WBUNMATCHED (1 << 0)

/* Alpha blending options */

#define ALPHA1BPP   (1 << 15)
#define FIXEDSRC    (1 << 14)
#define MSBFIRST    (1 << 13)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Copy command (32 bytes) */

struct aligned_data(16) ge2d_copycmd_s
{
  uint32_t cmd;               /* 0x00 */
  uint16_t srch;              /* 0x04 */
  uint16_t srcv;              /* 0x06 */
  uint32_t saddr;             /* 0x08 */
  uint32_t daddr;             /* 0x0c */
  uint16_t spitch;            /* 0x10 */
  uint16_t dpitch;            /* 0x12 */
  uint32_t reserved[3];
};

/* Raster operation (ROP) command (32 bytes + scaling OP 16 bytes) */

struct aligned_data(16) ge2d_ropcmd_s
{
  uint16_t cmd;               /* 0x00 */
  uint8_t rop;                /* 0x02 */
  uint8_t options;            /* 0x03 */
  uint16_t srch;              /* 0x04 */
  uint16_t srcv;              /* 0x06 */
  uint32_t saddr;             /* 0x08 */
  uint32_t daddr;             /* 0x0c */
  uint16_t spitch;            /* 0x10 */
  uint16_t dpitch;            /* 0x12 */

  uint32_t fixedcolor;        /* 0x14 */
  uint32_t pataddr;           /* 0x18 */
  uint16_t patpitch;          /* 0x1c */
  uint8_t pathoffset;         /* 0x1e */
  uint8_t patvoffset;         /* 0x1f */
};

struct aligned_data(16) ge2d_ropcmd_scaling_s
{
  uint16_t desth;             /* 0x20 */
  uint16_t destv;             /* 0x22 */
  uint16_t ratioh;            /* 0x24 */
  uint16_t ratiov;            /* 0x26 */

  uint8_t hphaseinit;         /* 0x28 */
  uint8_t hphaseoffset;       /* 0x29: must be 0 */
  uint8_t vphaseinit;         /* 0x2a */
  uint8_t vphaseoffset;       /* 0x2b: must be 0 */

  uint32_t intpmode;          /* 0x2c: interpolation mode */
};

/* Alpha blending (AB) command (32 bytes) */

struct aligned_data(16) ge2d_abcmd_s
{
  uint16_t cmd;               /* 0x00 */
  uint16_t mode;              /* 0x02 */
  uint16_t srch;              /* 0x04 */
  uint16_t srcv;              /* 0x06 */
  uint32_t saddr;             /* 0x08 */
  uint32_t daddr;             /* 0x0c */
  uint16_t spitch;            /* 0x10 */
  uint16_t dpitch;            /* 0x12 */

  uint32_t fixedsrc;          /* 0x14 */
  uint32_t aaddr;             /* 0x18 */
  uint16_t apitch;            /* 0x1c */
  uint16_t reserved;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_imageprocinitialized = false;
static sem_t g_rotwait = SEM_INITIALIZER(0);
static mutex_t g_rotlock = NXMUTEX_INITIALIZER;
static mutex_t g_gelock = NXMUTEX_INITIALIZER;
static mutex_t g_ablock = NXMUTEX_INITIALIZER;

static struct file g_gfile;
static char g_gcmdbuf[256] aligned_data(16);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int intr_handler_rot(int irq, void *context, void *arg)
{
  putreg32(1, ROT_INTR_CLEAR);
  putreg32(0, ROT_INTR_ENABLE);
  putreg32(1, ROT_INTR_DISABLE);

  nxsem_post(&g_rotwait);
  return 0;
}

static int ratio_check(uint16_t src, uint16_t dest)
{
  uint16_t ratio = 1;

  if (src > dest)
    {
      ratio = src / dest;
    }
  else if (src < dest)
    {
      ratio = dest / src;
    }

  if (ratio > MAX_RATIO)
    {
      return -1;
    }

  return 0;
}

static uint16_t calc_ratio(uint16_t src, uint16_t dest)
{
  uint16_t r;

  if (src > dest)
    {
      r = src / dest;
      if (r == 2 || r == 4 || r == 8 || r == 16 || r == 32 || r == 64)
        {
          return 256 * r;
        }
    }
  else if (src < dest)
    {
      r = dest / src;
      if (r == 2 || r == 4 || r == 8 || r == 16 || r == 32 || r == 64)
        {
          return 256 / r;
        }
    }
  else
    {
      return 256;
    }

  return 0;
}

static void *set_rop_cmd(void *cmdbuf,
                         void *srcaddr,
                         void *destaddr,
                         uint16_t srcwidth,
                         uint16_t srcheight,
                         uint16_t srcpitch,
                         uint16_t destwidth,
                         uint16_t destheight,
                         uint16_t destpitch,
                         uint8_t bpp,
                         uint8_t rop,
                         uint8_t options,
                         uint16_t patcolor)
{
  struct ge2d_ropcmd_s *rc = (struct ge2d_ropcmd_s *)cmdbuf;
  struct ge2d_ropcmd_scaling_s *sc;
  uint16_t rv;
  uint16_t rh;
  uint16_t cmd = ROPCMD;

  if (((uintptr_t) srcaddr & 1) || ((uintptr_t) destaddr & 1))
    {
      return NULL;
    }

  if (srcwidth & 1 || destwidth & 1)
    {
      return NULL;
    }

  rv = calc_ratio(srcheight, destheight);
  if (rv == 0)
    {
      return NULL;
    }

  rh = calc_ratio(srcwidth, destwidth);
  if (rh == 0)
    {
      return NULL;
    }

  /* If ratio is not 256 (x1), then set scaling bit. */

  if (rv != 256 || rh != 256 || options & CONV8BPP)
    {
      cmd |= SCALING;
    }

  memset(rc, 0, sizeof(struct ge2d_ropcmd_s));

  if (bpp == 16)
    {
      cmd |= SRC16BPP;
    }

  rc->cmd = cmd;
  rc->rop = rop;
  rc->options = options;
  rc->fixedcolor = patcolor;
  rc->srch = srcwidth - 1;
  rc->srcv = srcheight - 1;
  rc->saddr = CXD56_PHYSADDR(srcaddr) | MSEL;
  rc->daddr = CXD56_PHYSADDR(destaddr) | MSEL;
  rc->spitch = srcpitch - 1;
  rc->dpitch = destpitch - 1;

  /* Shift to next command area */

  cmdbuf = (void *)((uintptr_t)cmdbuf + sizeof(struct ge2d_ropcmd_s));

  /* Set scaling information */

  if (cmd & SCALING)
    {
      sc = (struct ge2d_ropcmd_scaling_s *)cmdbuf;

      sc->desth = destwidth - 1;
      sc->destv = destheight - 1;
      sc->ratiov = rv - 1;
      sc->ratioh = rh - 1;
      sc->hphaseinit = 1;
      sc->vphaseinit = 1;
      sc->intpmode = 0;         /* XXX: HV Linear interpolation */

      /* Shift to next command area */

      cmdbuf = (void *)((uintptr_t)cmdbuf
                        + sizeof(struct ge2d_ropcmd_scaling_s));
    }

  /* return next command area */

  return cmdbuf;
}

static void *set_ab_cmd(void *cmdbuf, void *srcaddr, void *destaddr,
                        uint16_t srcwidth, uint16_t srcheight,
                        uint16_t srcpitch, uint16_t destpitch,
                        void *aaddr, uint16_t apitch,
                        int options, uint16_t fixedsrc, uint16_t fixedalpha)
{
  struct ge2d_abcmd_s *ac = (struct ge2d_abcmd_s *)cmdbuf;

  memset(ac, 0, sizeof(struct ge2d_abcmd_s));

  ac->cmd = ABCMD | options;
  ac->mode = fixedalpha;
  ac->srch = srcwidth - 1;
  ac->srcv = srcheight - 1;
  ac->saddr = CXD56_PHYSADDR(srcaddr) | MSEL;
  ac->daddr = CXD56_PHYSADDR(destaddr) | MSEL;
  ac->spitch = srcpitch - 1;
  ac->dpitch = destpitch - 1;
  ac->fixedsrc = (uint32_t)fixedsrc;
  if (aaddr)
    {
      ac->aaddr = CXD56_PHYSADDR(aaddr) | MSEL;
      ac->apitch = apitch - 1;
    }
  else
    {
      ac->aaddr = CXD56_PHYSADDR(destaddr) | MSEL;
      ac->apitch = destpitch - 1;
    }

  return (void *)((uintptr_t)cmdbuf + sizeof(struct ge2d_abcmd_s));
}

static void *set_halt_cmd(void *cmdbuf)
{
  memset(cmdbuf, 0, 16);
  return (void *)((uintptr_t) cmdbuf + 16);
}

static int imageproc_convert_(int is_yuv2rgb,
                              uint8_t *ibuf,
                              uint32_t hsize,
                              uint32_t vsize)
{
  int ret;

  if (!g_imageprocinitialized)
    {
      return -EPERM;
    }

  if ((hsize & 1) || (vsize & 1))
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_rotlock);
  if (ret)
    {
      return ret;
    }

  /* Image processing hardware want to be set horizontal/vertical size
   * to actual size - 1.
   */

  --hsize;
  --vsize;

  putreg32(1, ROT_INTR_ENABLE);
  putreg32(0, ROT_INTR_DISABLE);
  putreg32(0, ROT_SET_DIRECTION);

  putreg32(hsize, ROT_SET_SRC_HSIZE);
  putreg32(vsize, ROT_SET_SRC_VSIZE);
  putreg32(CXD56_PHYSADDR(ibuf), ROT_SET_SRC_ADDRESS);

  putreg32(hsize, ROT_SET_SRC_PITCH);
  putreg32(CXD56_PHYSADDR(ibuf), ROT_SET_DST_ADDRESS);

  putreg32(hsize, ROT_SET_DST_PITCH);

  putreg32(is_yuv2rgb ? 1 : 2, ROT_CONV_CTRL);
  putreg32(0, ROT_RGB_ALIGNMENT);
  putreg32(1, ROT_COMMAND);

  nxsem_wait_uninterruptible(&g_rotwait);
  nxmutex_unlock(&g_rotlock);

  return 0;
}

static void get_rect_info(imageproc_imginfo_t *imginfo,
                          int *offset, int *w, int *h)
{
  if (imginfo->rect)
    {
      *offset = (imginfo->rect->y1 * imginfo->w)
              + imginfo->rect->x1;
      *w      = imginfo->rect->x2 - imginfo->rect->x1 + 1;
      *h      = imginfo->rect->y2 - imginfo->rect->y1 + 1;
    }
  else
    {
      *offset = 0;
      *w      = imginfo->w;
      *h      = imginfo->h;
    }
}

static int  chk_imgsize(imageproc_imginfo_t *imginfo)
{
  if (!imginfo)
    {
      return -EINVAL;
    }

  if ((imginfo->w > HSIZE_MAX) || (imginfo->w < HSIZE_MIN) ||
      (imginfo->h > VSIZE_MAX) || (imginfo->h < VSIZE_MIN))
    {
      return -EINVAL;
    }

  if (imginfo->rect)
    {
      if ((imginfo->rect->x2 <= imginfo->rect->x1) ||
          (imginfo->rect->y2 <= imginfo->rect->y1))
        {
          return -EINVAL;
        }

      if ((imginfo->rect->x2 >= imginfo->w) ||
          (imginfo->rect->y2 >= imginfo->h))
        {
          return -EINVAL;
        }
    }

  return 0;
}

static void *get_blendarea(imageproc_imginfo_t *imginfo, int offset)
{
  switch (imginfo->type)
    {
      case IMAGEPROC_IMGTYPE_8BPP:
        return imginfo->img.p_u8 + offset;

      case IMAGEPROC_IMGTYPE_16BPP:
        return imginfo->img.p_u16 + offset;

      case IMAGEPROC_IMGTYPE_BINARY:
        return imginfo->img.binary.p_u8 + offset / 8;

      default:
        return NULL;
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imageproc_initialize(void)
{
  if (g_imageprocinitialized)
    {
      return;
    }

  g_imageprocinitialized = true;
  cxd56_ge2dinitialize(GEDEVNAME);
  file_open(&g_gfile, GEDEVNAME, O_RDWR);

  putreg32(1, ROT_INTR_CLEAR);
  putreg32(0, ROT_INTR_ENABLE);
  putreg32(1, ROT_INTR_DISABLE);

  irq_attach(CXD56_IRQ_ROT, intr_handler_rot, NULL);
  up_enable_irq(CXD56_IRQ_ROT);
}

void imageproc_finalize(void)
{
  if (!g_imageprocinitialized)
    {
      return;
    }

  up_disable_irq(CXD56_IRQ_ROT);
  irq_detach(CXD56_IRQ_ROT);

  if (g_gfile.f_inode)
    {
      file_close(&g_gfile);
    }

  cxd56_ge2duninitialize(GEDEVNAME);
  g_imageprocinitialized = false;
}

int imageproc_convert_yuv2rgb(uint8_t *ibuf,
                              uint32_t hsize,
                              uint32_t vsize)
{
  return imageproc_convert_(1, ibuf, hsize, vsize);
}

int imageproc_convert_rgb2yuv(uint8_t *ibuf,
                              uint32_t hsize,
                              uint32_t vsize)
{
  return imageproc_convert_(0, ibuf, hsize, vsize);
}

void imageproc_convert_yuv2gray(uint8_t * ibuf, uint8_t * obuf, size_t hsize,
                                size_t vsize)
{
  uint16_t *p_src = (uint16_t *)ibuf;
  size_t ix;
  size_t iy;

  for (iy = 0; iy < vsize; iy++)
    {
      for (ix = 0; ix < hsize; ix++)
        {
          *obuf++ = (uint8_t) ((*p_src++ & 0xff00) >> 8);
        }
    }
}

int imageproc_resize(uint8_t * ibuf,
                     uint16_t ihsize,
                     uint16_t ivsize,
                     uint8_t * obuf,
                     uint16_t ohsize,
                     uint16_t ovsize,
                     int bpp)
{
  void *cmd = g_gcmdbuf;
  size_t len;
  int ret;

  if (g_gfile.f_inode == NULL)
    {
      return -ENODEV;
    }

  if (bpp != 8 && bpp != 16)
    {
      return -EINVAL;
    }

  if ((ihsize > ISE_SRC_HSIZE_MAX || ihsize < HSIZE_MIN) ||
      (ivsize > ISE_SRC_VSIZE_MAX || ihsize < VSIZE_MIN) ||
      (ohsize > ISE_DST_HSIZE_MAX || ohsize < HSIZE_MIN) ||
      (ovsize > ISE_DST_VSIZE_MAX || ovsize < VSIZE_MIN))
    {
      return -EINVAL;
    }

  if ((ratio_check(ihsize, ohsize) != 0) ||
      (ratio_check(ivsize, ovsize) != 0))
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_gelock);
  if (ret)
    {
      return ret;
    }

  /* Create descriptor to graphics engine */

  cmd = set_rop_cmd(cmd,
                    ibuf,
                    obuf,
                    ihsize,
                    ivsize,
                    ihsize,
                    ohsize,
                    ovsize,
                    ohsize,
                    bpp,
                    SRCCOPY,
                    FIXEDCOLOR,
                    0x0080);
  if (cmd == NULL)
    {
      nxmutex_unlock(&g_gelock);
      return -EINVAL;
    }

  /* Terminate command */

  cmd = set_halt_cmd(cmd);

  /* Process resize */

  len = (uintptr_t) cmd - (uintptr_t) g_gcmdbuf;
  ret = file_write(&g_gfile, g_gcmdbuf, len);
  if (ret < 0)
    {
      nxmutex_unlock(&g_gelock);
      return -EFAULT;
    }

  nxmutex_unlock(&g_gelock);
  return 0;
}

int imageproc_clip_and_resize(uint8_t * ibuf,
                              uint16_t ihsize,
                              uint16_t ivsize,
                              uint8_t * obuf,
                              uint16_t ohsize,
                              uint16_t ovsize,
                              int bpp,
                              imageproc_rect_t * clip_rect)
{
  void *cmd = g_gcmdbuf;
  size_t len;
  int ret;
  uint8_t pix_bytes;
  uint16_t clip_width = 0;
  uint16_t clip_height = 0;

  if (g_gfile.f_inode == NULL)
    {
      return -ENODEV;
    }

  if (bpp != 8 && bpp != 16)
    {
      return -EINVAL;
    }

  if ((ihsize > ISE_SRC_HSIZE_MAX || ihsize < HSIZE_MIN) ||
      (ivsize > ISE_SRC_VSIZE_MAX || ihsize < VSIZE_MIN) ||
      (ohsize > ISE_DST_HSIZE_MAX || ohsize < HSIZE_MIN) ||
      (ovsize > ISE_DST_VSIZE_MAX || ovsize < VSIZE_MIN))
    {
      return -EINVAL;
    }

  if (clip_rect != NULL)
    {
      if ((clip_rect->x2 < clip_rect->x1) ||
          (clip_rect->y2 < clip_rect->y1))
        {
          return -EINVAL;
        }

      if ((clip_rect->x2 > ihsize) || (clip_rect->y2 > ivsize))
        {
          return -EINVAL;
        }

      clip_width = clip_rect->x2 - clip_rect->x1 + 1;
      clip_height = clip_rect->y2 - clip_rect->y1 + 1;

      if ((ratio_check(clip_width, ohsize) != 0) ||
          (ratio_check(clip_height, ovsize) != 0))
        {
          return -EINVAL;
        }

      pix_bytes = bpp >> 3;
      ibuf = ibuf + (clip_rect->x1 * pix_bytes +
                     clip_rect->y1 * ihsize * pix_bytes);
    }
  else
    {
      if ((ratio_check(ihsize, ohsize) != 0) ||
          (ratio_check(ivsize, ovsize) != 0))
        {
          return -EINVAL;
        }

      clip_width = ihsize;
      clip_height = ivsize;
    }

  ret = nxmutex_lock(&g_gelock);
  if (ret)
    {
      return ret;
    }

  /* Create descriptor to graphics engine */

  cmd = set_rop_cmd(cmd,
                    ibuf,
                    obuf,
                    clip_width,
                    clip_height,
                    ihsize,
                    ohsize,
                    ovsize,
                    ohsize,
                    bpp,
                    SRCCOPY,
                    FIXEDCOLOR,
                    0x0080);

  if (cmd == NULL)
    {
      nxmutex_unlock(&g_gelock);
      return -EINVAL;
    }

  /* Terminate command */

  cmd = set_halt_cmd(cmd);

  /* Process resize */

  len = (uintptr_t) cmd - (uintptr_t) g_gcmdbuf;
  ret = file_write(&g_gfile, g_gcmdbuf, len);
  if (ret < 0)
    {
      nxmutex_unlock(&g_gelock);
      return -EFAULT;
    }

  nxmutex_unlock(&g_gelock);
  return 0;
}

int imageproc_alpha_blend(imageproc_imginfo_t *dst,
                          int pos_x,
                          int pos_y,
                          imageproc_imginfo_t *src,
                          imageproc_imginfo_t *alpha)
{
  int ret;

  /* Graphic engine control */

  void *cmd = g_gcmdbuf;
  size_t len;

  /* alpha blend options */

  uint16_t fixed_alpha;
  uint16_t fixed_src;
  int      options;

  /* blended rectangles information */

  void *dst_addr;
  void *src_addr;
  void *a_addr;

  int  dst_offset;
  int  dst_w;
  int  dst_h;
  int  src_offset;
  int  src_w;
  int  src_h;
  int  a_offset;
  int  a_w;
  int  a_h;

  int blendarea_left;
  int blendarea_right;
  int blendarea_top;
  int blendarea_bottom;

  /* Parameter range check */

  ret = chk_imgsize(dst);
  if (ret < 0)
    {
      return ret;
    }

  ret = chk_imgsize(src);
  if (ret < 0)
    {
      return ret;
    }

  ret = chk_imgsize(alpha);
  if (ret < 0)
    {
      return ret;
    }

  /* Determine alpha blend options */

  fixed_src   = 0;
  fixed_alpha = 0;
  options     = 0;

  switch (alpha->type)
    {
      case IMAGEPROC_IMGTYPE_SINGLE:
        fixed_alpha = 0x0800 | (uint8_t)alpha->img.single;
        break;

      case IMAGEPROC_IMGTYPE_BINARY:
        fixed_alpha = (uint8_t)alpha->img.binary.multiplier;
        options |= ALPHA1BPP;

        break;

      case IMAGEPROC_IMGTYPE_8BPP:

        /* In this case, no option */

        break;

      default:
        return -EINVAL;
    }

  switch (src->type)
    {
      case IMAGEPROC_IMGTYPE_SINGLE:
        options   |= FIXEDSRC;
        fixed_src =  src->img.single;
        break;

      case IMAGEPROC_IMGTYPE_16BPP:

        /* In this case, no option */

        break;

      default:
        return -EINVAL;
    }

  switch (dst->type)
    {
      case IMAGEPROC_IMGTYPE_16BPP:

        /* In this case, no option */

        break;

      default:
        return -EINVAL;
    }

  /* Determine offset, width, height of rectangles from IN parameter */

  get_rect_info(dst,   &dst_offset, &dst_w, &dst_h);
  get_rect_info(src,   &src_offset, &src_w, &src_h);
  get_rect_info(alpha, &a_offset,   &a_w,   &a_h);

  /* Recalculate offset by calculating overlapped area. */

  blendarea_left   = - MIN(0, pos_x);
  blendarea_right  = MIN(MIN(a_w, src_w), dst_w - pos_x);
  blendarea_top    = - MIN(0, pos_y);
  blendarea_bottom = MIN(MIN(a_h, src_h), dst_h - pos_y);

  if ((blendarea_right  <= blendarea_left) ||
      (blendarea_bottom <= blendarea_top))
    {
      return 0;  /* Not blend due to no overlapped area */
    }

  dst_offset += ((blendarea_top + pos_y) * dst->w)
             +  (blendarea_left + pos_x);
  src_offset += (blendarea_top * src->w) + blendarea_left;
  a_offset   += (blendarea_top * alpha->w) + blendarea_left;

  dst_addr = get_blendarea(dst,   dst_offset);
  src_addr = get_blendarea(src,   src_offset);
  a_addr   = get_blendarea(alpha, a_offset);

  ret = nxmutex_lock(&g_ablock);
  if (ret)
    {
      return ret; /* -EINTR */
    }

  /* Create descriptor to graphics engine */

  cmd = set_ab_cmd(cmd,
                   src_addr,
                   dst_addr,
                   blendarea_right  - blendarea_left, /* width   of blended area  */
                   blendarea_bottom - blendarea_top,  /* height  of blended area  */
                   src->w,                            /* pitch of src image */
                   dst->w,                            /* pitch of dst image */
                   a_addr,
                   alpha->w,                          /* pitch of alpha plane */
                   options,
                   fixed_src,
                   fixed_alpha);

  if (cmd == NULL)
    {
      nxmutex_unlock(&g_ablock);
      return -EINVAL;
    }

  /* Terminate command */

  cmd = set_halt_cmd(cmd);

  /* Process alpha blending */

  len = (uintptr_t)cmd - (uintptr_t)g_gcmdbuf;
  ret = file_write(&g_gfile, g_gcmdbuf, len);
  if (ret < 0)
    {
      nxmutex_unlock(&g_ablock);
      return -EFAULT;
    }

  nxmutex_unlock(&g_ablock);
  return 0;
}

