/****************************************************************************
 * arch/x86/src/qemu/qemu_vga.c
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *   History: 0.1 2012-11-30 initial version
 *
 * Derived from drivers/lcd/skeleton.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/fs/fs.h>

#include <arch/io.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define R_COM  0x63 /* "common" bits */

#define R_W256 0x00
#define R_W320 0x00
#define R_W360 0x04
#define R_W376 0x04
#define R_W400 0x04

#define R_H200 0x00
#define R_H224 0x80
#define R_H240 0x80
#define R_H256 0x80
#define R_H270 0x80
#define R_H300 0x80
#define R_H360 0x00
#define R_H400 0x00
#define R_H480 0x80
#define R_H564 0x80
#define R_H600 0x80

#define SZ(x) (sizeof(x)/sizeof(x[0]))

#define VGA_XRES         320
#define VGA_YRES         240
#define VGA_BPP          8
#define VGA_FBSIZE       (VGA_XRES * VGA_YRES * VGA_BPP / 8)
#define VGA_COLORFMT     FB_FMT_RGB8

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int init_graph_vga(int width, int height, int chain4);
static int vga_putrun(fb_coord_t row,
                      fb_coord_t col, const uint8_t *buffer,
                      size_t npixels);
static int vga_getrun(fb_coord_t row, fb_coord_t col, uint8_t *buffer,
                      size_t npixels);
static int vga_getvideoinfo(struct lcd_dev_s *dev,
                            struct fb_videoinfo_s *vinfo);
static int vga_getplaneinfo(struct lcd_dev_s *dev, unsigned int planeno,
                            struct lcd_planeinfo_s *pinfo);
static int vga_getpower(struct lcd_dev_s *dev);
static int vga_setpower(struct lcd_dev_s *dev, int power);
static int vga_getcontrast(struct lcd_dev_s *dev);
static int vga_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);
static ssize_t vga_read(struct file *filep, char *buf, size_t buflen);
static ssize_t vga_write(struct file *filep, const char *buf,
                         size_t buflen);
static off_t vga_seek(struct file *filp, off_t offset, int whence);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_hor_regs[]  =
{
  0x0,  0x1,  0x2,  0x3,  0x4, 0x5,  0x13
};

static const uint8_t g_width_256[] =
{
  0x5f, 0x3f, 0x40, 0x82, 0x4a, 0x9a, 0x20
};

static const uint8_t g_width_320[] =
{
  0x5f, 0x4f, 0x50, 0x82, 0x54, 0x80, 0x28
};

static const uint8_t g_width_360[] =
{
  0x6b, 0x59, 0x5a, 0x8e, 0x5e, 0x8a, 0x2d
};

static const uint8_t g_width_376[] =
{
  0x6e, 0x5d, 0x5e, 0x91, 0x62, 0x8f, 0x2f
};

static const uint8_t g_width_400[] =
{
  0x70, 0x63, 0x64, 0x92, 0x65, 0x82, 0x32
};

static const uint8_t g_ver_regs[]  =
{
  0x6,  0x7,  0x9,  0x10, 0x11, 0x12, 0x15, 0x16
};

static const uint8_t height_200[]  =
{
  0xbf, 0x1f, 0x41, 0x9c, 0x8e, 0x8f, 0x96, 0xb9
};

static const uint8_t height_224[]  =
{
  0x0b, 0x3e, 0x41, 0xda, 0x9c, 0xbf, 0xc7, 0x04
};

static const uint8_t height_240[]  =
{
  0x0d, 0x3e, 0x41, 0xea, 0xac, 0xdf, 0xe7, 0x06
};

static const uint8_t height_256[]  =
{
  0x23, 0xb2, 0x61, 0x0a, 0xac, 0xff, 0x07, 0x1a
};

static const uint8_t height_270[]  =
{
  0x30, 0xf0, 0x61, 0x20, 0xa9, 0x1b, 0x1f, 0x2f
};

static const uint8_t height_300[]  =
{
  0x70, 0xf0, 0x61, 0x5b, 0x8c, 0x57, 0x58, 0x70
};

static const uint8_t height_360[]  =
{
  0xbf, 0x1f, 0x40, 0x88, 0x85, 0x67, 0x6d, 0xba
};

static const uint8_t height_400[]  =
{
  0xbf, 0x1f, 0x40, 0x9c, 0x8e, 0x8f, 0x96, 0xb9
};

static const uint8_t height_480[]  =
{
  0x0d, 0x3e, 0x40, 0xea, 0xac, 0xdf, 0xe7, 0x06
};

static const uint8_t height_564[]  =
{
  0x62, 0xf0, 0x60, 0x37, 0x89, 0x33, 0x3c, 0x5c
};

static const uint8_t height_600[]  =
{
  0x70, 0xf0, 0x60, 0x5b, 0x8c, 0x57, 0x58, 0x70
};

static const uint8_t g_bg_color    = 0x0f;
static const uint8_t g_fg_color    = 0x01;

static uint8_t g_runbuffer[VGA_XRES];
static uint8_t *g_pscreen = (uint8_t *)(0xa0000);

static off_t g_curpos;

/* This is the standard, NuttX LCD driver object */

static struct lcd_dev_s g_lcddev =
{
  .getvideoinfo = vga_getvideoinfo,
  .getplaneinfo = vga_getplaneinfo,

  .getpower     = vga_getpower,
  .setpower     = vga_setpower,
  .getcontrast  = vga_getcontrast,
  .setcontrast  = vga_setcontrast,
};

static const struct file_operations g_vgaops =
{
  NULL,         /* open */
  NULL,         /* close */
  vga_read,     /* read */
  vga_write,    /* write */
  vga_seek,     /* seek */
  NULL,         /* ioctl */
  NULL          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* The chain4 parameter should be 1 for normal 13h-type mode, but
 * only allows 320x200 256x200, 256x240 and 256x256 because you
 * can only access the first 64kb
 *
 * If chain4 is 0, then plane mode is used (tweaked modes), and
 * you'll need to switch planes to access the whole screen but
 * that allows you using any resolution, up to 400x600
 *
 * Returned Value:
 *   0=ok, -n=fail
 */

static int init_graph_vga(int width, int height, int chain4)
{
  const uint8_t *w;
  const uint8_t *h;
  uint8_t val;
  int a;

  switch (width)
    {
      case 256:
        w = g_width_256;
        val = R_COM + R_W256;
        break;

      case 320:
        w = g_width_320;
        val = R_COM + R_W320;
        break;

      case 360:
        w = g_width_360;
        val = R_COM + R_W360;
        break;

      case 376:
        w = g_width_376;
        val = R_COM + R_W376;
        break;

      case 400:
        w = g_width_400;
        val = R_COM + R_W400;
        break;

      default:
        return -1; /* fail */
    }

  switch (height)
    {
      case 200:
        h = height_200;
        val |= R_H200;
        break;

      case 224:
        h = height_224;
        val |= R_H224;
        break;

      case 240:
        h = height_240;
        val |= R_H240;
        break;

      case 256:
        h = height_256;
        val |= R_H256;
        break;

      case 270:
        h = height_270;
        val |= R_H270;
        break;

      case 300:
        h = height_300;
        val |= R_H300;
        break;

      case 360:
        h = height_360;
        val |= R_H360;
        break;

      case 400:
        h = height_400;
        val |= R_H400;
        break;

      case 480:
        h = height_480;
        val |= R_H480;
        break;

      case 564:
        h = height_564;
        val |= R_H564;
        break;

      case 600:
        h = height_600;
        val |= R_H600;
        break;

      default:
        return -2; /* fail */
    }

  /* chain4 not available if mode takes over 64k */

  /* if (chain4 && (long)width*(long)height>65536L) return -3; */

  /* here goes the actual modeswitch */

  outb(val, 0x3c2);
  outw(0x0e11, 0x3d4); /* enable regs 0-7 */

  for (a = 0; a < SZ(g_hor_regs); ++a)
    {
      outw((uint16_t)((w[a] << 8) + g_hor_regs[a]), 0x3d4);
    }

  for (a = 0; a < SZ(g_ver_regs); ++a)
    {
      outw((uint16_t)((h[a] << 8) + g_ver_regs[a]), 0x3d4);
    }

  outw(0x0008, 0x3d4); /* vert.panning = 0 */

  if (chain4)
    {
      outw(0x4014, 0x3d4);
      outw(0xa317, 0x3d4);
      outw(0x0e04, 0x3c4);
    }
  else
    {
      outw(0x0014, 0x3d4);
      outw(0xe317, 0x3d4);
      outw(0x0604, 0x3c4);
    }

  outw(0x0101, 0x3c4);
  outw(0x0f02, 0x3c4); /* Enable writing to all planes */
  outw(0x4005, 0x3ce); /* 256 color mode */
  outw(0x0106, 0x3ce); /* Extend graph mode & a000-bfff */

  inb(0x3da);
  outb(0x30, 0x3c0);
  outb(0x41, 0x3c0);
  outb(0x33, 0x3c0);
  outb(0x00, 0x3c0);

  for (a = 0; a < 16; a++)    /* ega pal */
    {
      outb((uint8_t)a, 0x3c0);
      outb((uint8_t)a, 0x3c0);
    }

  outb(0x20, 0x3c0); /* enable video */

  return 0;
}

static int vga_putrun(fb_coord_t row,
                      fb_coord_t col, const uint8_t *buffer,
                      size_t npixels)
{
  memcpy(&g_pscreen[row*VGA_XRES + col], buffer, npixels);
  return OK;
}

static int vga_getrun(fb_coord_t row, fb_coord_t col, uint8_t *buffer,
                      size_t npixels)
{
  memcpy(buffer, &g_pscreen[row*VGA_XRES + col], npixels);
  return OK;
}

static int vga_getvideoinfo(struct lcd_dev_s *dev,
                            struct fb_videoinfo_s *vinfo)
{
  vinfo->fmt     = VGA_COLORFMT;
  vinfo->xres    = VGA_XRES;        /* Horizontal resolution in pixel columns */
  vinfo->yres    = VGA_YRES;        /* Vertical resolution in pixel rows */
  vinfo->nplanes = 1;               /* Number of color planes supported */
  return OK;
}

static int vga_getplaneinfo(struct lcd_dev_s *dev, unsigned int planeno,
                            struct lcd_planeinfo_s *pinfo)
{
  pinfo->putrun  = vga_putrun;      /* Put a run into LCD memory */
  pinfo->getrun  = vga_getrun;      /* Get a run from LCD memory */
  pinfo->buffer  = g_runbuffer;     /* Run scratch buffer */
  pinfo->display = 0;
  pinfo->bpp     = VGA_BPP;         /* Bits-per-pixel */
  return OK;
}

static int vga_getpower(struct lcd_dev_s *dev)
{
  return 0;
}

static int vga_setpower(struct lcd_dev_s *dev, int power)
{
  return OK;
}

static int vga_getcontrast(struct lcd_dev_s *dev)
{
  return -ENOSYS;
}

static int vga_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  return -ENOSYS;
}

static ssize_t vga_read(struct file *filep, char *buf, size_t buflen)
{
  if (buf == NULL || buflen < 1)
    {
      return -EINVAL;
    }

  /* memcpy(&buf,&g_pscreen[y*VGA_XRES + x],buflen); */

  return buflen;
}

static ssize_t vga_write(struct file *filep, const char *buf,
                         size_t buflen)
{
  int i;
  int j;

  if (buf == NULL || buflen < 1)
    {
      return -EINVAL;
    }

  for (j = 0; j < buflen && g_curpos < VGA_FBSIZE; j++)
    {
      uint8_t dots = buf[j];
      for (i = 0; i < 8 && g_curpos < VGA_FBSIZE; i++)
        {
          g_pscreen[g_curpos++] = dots & 0x80 ? g_fg_color : g_bg_color;
          dots <<= 1;
        }
    }

  return buflen;
}

static off_t vga_seek(struct file *filp, off_t offset, int whence)
{
  ssize_t newpos;

  /* Adjust the cursor position as requested */

  switch (whence)
    {
      case SEEK_SET:  /* From the start of the file */
        newpos = (ssize_t)offset;
        break;

      case SEEK_CUR:  /* From the current file offset */
        newpos = g_curpos + offset;
        break;

      case SEEK_END:  /* From the end of the file */
        newpos = (VGA_FBSIZE - 1) - offset;
        break;

      default:
        return g_curpos;
    }

  /* Make sure that the new cursor position lies within the frame buffer */

  if (newpos < 0)
    {
      newpos = 0;
    }
  else if (newpos > VGA_FBSIZE)
    {
      newpos = VGA_FBSIZE;
    }

  /* Set the new cursor position */

  g_curpos = (off_t)newpos;
  return g_curpos;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  qemu_vga_initialize
 *
 * Description:
 *   Initialize the QEMU VGA video hardware.
 *
 ****************************************************************************/

struct lcd_dev_s *qemu_vga_initialize(void)
{
  int ret = init_graph_vga(VGA_XRES, VGA_YRES, 1);
  if (ret < 0)
    {
      gerr("ERROR: init_graph_vga returned %d\n", ret);
    }

  memset(g_pscreen, 0, VGA_XRES * VGA_YRES);
  return &g_lcddev;
}

void qemu_vga(void)
{
  int ret = init_graph_vga(VGA_XRES, VGA_YRES, 1);
  if (ret < 0)
    {
      gerr("ERROR: init_graph_vga returned %d\n", ret);
    }

  memset(g_pscreen, g_bg_color, VGA_XRES * VGA_YRES);
  register_driver("/dev/lcd", &g_vgaops, 0666, NULL);
}
