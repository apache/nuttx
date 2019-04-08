/****************************************************************************
 * include/nuttx/video/fb.h
 *
 *   Copyright (C) 2008-2011, 2013, 2016-2018 Gregory Nutt. All rights
 *     reserved.
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

#ifndef __INCLUDE_NUTTX_VIDEO_FB_H
#define __INCLUDE_NUTTX_VIDEO_FB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Color format definitions.  The pretty much define the color pixel processing
 * organization of the video controller.
 */

/* Monochrome Formats *******************************************************/

#define FB_FMT_Y1             0         /* BPP=1, monochrome */
#define FB_FMT_Y2             1         /* BPP=2, 2-bit uncompressed greyscale */
#define FB_FMT_Y4             2         /* BPP=4, 4-bit uncompressed greyscale */
#define FB_FMT_Y8             3         /* BPP=8, 8-bit uncompressed greyscale */
#define FB_FMT_Y16            4         /* BPP=16, 16-bit uncompressed greyscale */
#define FB_FMT_GREY           FB_FMT_Y8 /* BPP=8 */
#define FB_FMT_Y800           FB_FMT_Y8 /* BPP=8 */

#define FB_ISMONO(f)          ((f) >= FB_FMT_Y4) && (f) <= FB_FMT_Y16)

/* RGB video formats ********************************************************/

/* Standard RGB */

#define FB_FMT_RGB1           FB_FMT_Y1   /* BPP=1 */
#define FB_FMT_RGB4           5           /* BPP=4 */
#define FB_FMT_RGB8           6           /* BPP=8 RGB palette index */
#define FB_FMT_RGB8_222       7           /* BPP=8  R=2, G=2, B=2 */
#define FB_FMT_RGB8_332       8           /* BPP=8  R=3, G=3, B=2 */
#define FB_FMT_RGB12_444      9           /* BPP=12 R=4, G=4, B=4 */
#define FB_FMT_RGB16_555      10          /* BPP=16 R=5, G=5, B=5 (1 unused bit) */
#define FB_FMT_RGB16_565      11          /* BPP=16 R=6, G=6, B=5 */
#define FB_FMT_RGB24          12          /* BPP=24 */
#define FB_FMT_RGB32          13          /* BPP=32 */

/* Run length encoded RGB */

#define FB_FMT_RGBRLE4        14          /* BPP=4 */
#define FB_FMT_RGBRLE8        15          /* BPP=8 */

/* Raw RGB */

#define FB_FMT_RGBRAW         16          /* BPP=? */

/* Raw RGB with arbitrary sample packing within a pixel. Packing and precision
 * of R, G and B components is determined by bit masks for each.
 */

#define FB_FMT_RGBBTFLD16     17          /* BPP=16 */
#define FB_FMT_RGBBTFLD24     18          /* BPP=24 */
#define FB_FMT_RGBBTFLD32     19          /* BPP=32 */
#define FB_FMT_RGBA16         20          /* BPP=16 Raw RGB with alpha */
#define FB_FMT_RGBA32         21          /* BPP=32 Raw RGB with alpha */

/* Raw RGB with a transparency field. Layout is as for standard RGB at 16 and
 * 32 bits per pixel but the msb in each pixel indicates whether the pixel is
 * transparent or not.
 */

#define FB_FMT_RGBT16         22          /* BPP=16 */
#define FB_FMT_RGBT32         23          /* BPP=32 */

#define FB_ISRGB(f)           ((f) >= FB_FMT_RGB1) && (f) <= FB_FMT_RGBT32)

/* Packed YUV Formats *******************************************************/

#define FB_FMT_AYUV           24          /* BPP=32  Combined YUV and alpha */
#define FB_FMT_CLJR           25          /* BPP=8   4 pixels packed into a uint32_t.
                                           *         YUV 4:1:1 with l< 8 bits
                                           *         per YUV sample */
#define FB_FMT_CYUV           26          /* BPP=16  UYVY except that height is
                                           *         reversed */
#define FB_FMT_IRAW           27          /* BPP=?   Intel uncompressed YUV */
#define FB_FMT_IUYV           28          /* BPP=16  Interlaced UYVY (line order
                                           *         0,2,4,.., 1,3,5...) */
#define FB_FMT_IY41           29          /* BPP=12  Interlaced Y41P (line order
                                           *         0,2,4,.., 1,3,5...) */
#define FB_FMT_IYU2           30          /* BPP=24 */
#define FB_FMT_HDYC           31          /* BPP=16  UYVY except uses the BT709
                                           *         color space  */
#define FB_FMT_UYVP           32          /* BPP=24? YCbCr 4:2:2, 10-bits per
                                           *         component in U0Y0V0Y1 order */
#define FB_FMT_UYVY           33          /* BPP=16  YUV 4:2:2 */
#define FB_FMT_UYNV           FB_FMT_UYVY /* BPP=16  */
#define FB_FMT_Y422           FB_FMT_UYVY /* BPP=16  */
#define FB_FMT_V210           34          /* BPP=32  10-bit 4:2:2 YCrCb */
#define FB_FMT_V422           35          /* BPP=16  Upside down version of UYVY */
#define FB_FMT_V655           36          /* BPP=16? 16-bit YUV 4:2:2 */
#define FB_FMT_VYUY           37          /* BPP=?   ATI Packed YUV Data */
#define FB_FMT_YUYV           38          /* BPP=16  YUV 4:2:2 */
#define FB_FMT_YUY2           FB_FMT_YUYV /* BPP=16  YUV 4:2:2 */
#define FB_FMT_YUNV           FB_FMT_YUYV /* BPP=16  YUV 4:2:2 */
#define FB_FMT_YVYU           39          /* BPP=16  YUV 4:2:2 */
#define FB_FMT_Y41P           40          /* BPP=12  YUV 4:1:1 */
#define FB_FMT_Y411           41          /* BPP=12  YUV 4:1:1 */
#define FB_FMT_Y211           42          /* BPP=8  */
#define FB_FMT_Y41T           43          /* BPP=12  Y41P LSB for transparency */
#define FB_FMT_Y42T           44          /* BPP=16  UYVY LSB for transparency */
#define FB_FMT_YUVP           45          /* BPP=24? YCbCr 4:2:2 Y0U0Y1V0 order */

#define FB_ISYUVPACKED(f)     ((f) >= FB_FMT_AYUV) && (f) <= FB_FMT_YUVP)

/* Packed Planar YUV Formats ************************************************/

#define FB_FMT_YVU9           46          /* BPP=9   8-bit Y followed by 8-bit
                                           *         4x4 VU */
#define FB_FMT_YUV9           47          /* BPP=9? */
#define FB_FMT_IF09           48          /* BPP=9.5 YVU9 + 4x4 plane of delta
                                           *         relative to tframe. */
#define FB_FMT_YV16           49          /* BPP=16  8-bit Y followed by 8-bit
                                           *         2x1 VU */
#define FB_FMT_YV12           50          /* BPP=12  8-bit Y followed by 8-bit
                                           *         2x2 VU */
#define FB_FMT_I420           51          /* BPP=12  8-bit Y followed by 8-bit
                                           *         2x2 UV */
#define FB_FMT_IYUV           FB_FMT_I420 /* BPP=12 */
#define FB_FMT_NV12           52          /* BPP=12  8-bit Y followed by an
                                           *         interleaved 2x2 UV */
#define FB_FMT_NV21           53          /* BPP=12  NV12 with UV reversed */
#define FB_FMT_IMC1           54          /* BPP=12  YV12 except UV planes same
                                           *         stride as Y */
#define FB_FMT_IMC2           55          /* BPP=12  IMC1 except UV lines
                                           *         interleaved at half stride
                                           *         boundaries */
#define FB_FMT_IMC3           56          /* BPP=12  As IMC1 except that UV
                                           *         swapped */
#define FB_FMT_IMC4           57          /* BPP=12  As IMC2  except that UV
                                           *         swapped */
#define FB_FMT_CLPL           58          /* BPP=12  YV12 but including a level
                                           *         of indirection. */
#define FB_FMT_Y41B           59          /* BPP=12?  4:1:1 planar. */
#define FB_FMT_Y42B           60          /* BPP=16?  YUV 4:2:2 planar. */
#define FB_FMT_CXY1           61          /* BPP=12 */
#define FB_FMT_CXY2           62          /* BPP=16 */

#define FB_ISYUVPLANAR(f)     (((f) >= FB_FMT_AYUV) && (f) <= FB_FMT_YUVP)
#define FB_ISYUV(f)           (FB_ISYUVPACKED(f) || FB_ISYUVPLANAR(f))

/* Hardware cursor control **************************************************/

#ifdef CONFIG_FB_HWCURSOR
#  define FB_CUR_ENABLE       0x01        /* Enable the cursor */
#  define FB_CUR_SETIMAGE     0x02        /* Set the cursor image */
#  define FB_CUR_SETPOSITION  0x04        /* Set the position of the cursor */
#  define FB_CUR_SETSIZE      0x08        /* Set the size of the cursor */
#  define FB_CUR_XOR          0x10        /* Use XOR vs COPY ROP on image */
#endif

/* Hardware overlay acceleration *******************************************/

#ifdef CONFIG_FB_OVERLAY
#  define FB_ACCL_TRANSP      0x01        /* Hardware tranparency support */
#  define FB_ACCL_CHROMA      0x02        /* Hardware chromakey support */
#  define FB_ACCL_COLOR       0x04        /* Hardware color support */
#  define FB_ACCL_AREA        0x08        /* Hardware support area selection */

#ifdef CONFIG_FB_OVERLAY_BLIT
#  define FB_ACCL_BLIT        0x10        /* Hardware blit support */
#  define FB_ACCL_BLEND       0x20        /* Hardware blend support */
#endif

/* Overlay transparency mode ************************************************/

#  define FB_CONST_ALPHA      0x00         /* Transparency by alpha value */
#  define FB_PIXEL_ALPHA      0x01         /* Transparency by pixel alpha value */

#endif /* CONFIG_FB_OVERLAY */

/* FB character driver IOCTL commands ***************************************/

/* ioctls */

#define FBIOGET_VIDEOINFO     _FBIOC(0x0001)  /* Get color plane info */
                                              /* Argument: writable struct
                                               *           fb_videoinfo_s */
#define FBIOGET_PLANEINFO     _FBIOC(0x0002)  /* Get video plane info */
                                              /* Argument: writable struct
                                               *           fb_planeinfo_s */

#ifdef CONFIG_FB_CMAP
#  define FBIOGET_CMAP        _FBIOC(0x0003)  /* Get RGB color mapping */
                                              /* Argument: writable struct
                                               *           fb_cmap_s */
#  define FBIOPUT_CMAP        _FBIOC(0x0004)  /* Put RGB color mapping */
                                              /* Argument: read-only struct
                                               *           fb_cmap_s */
#endif

#ifdef CONFIG_FB_HWCURSOR
#  define FBIOGET_CURSOR      _FBIOC(0x0005)  /* Get cursor attributes */
                                              /* Argument: writable struct
                                               *           fb_cursorattrib_s */
#  define FBIOPUT_CURSOR      _FBIOC(0x0006)  /* Set cursor attributes */
                                              /* Argument: read-only struct
                                               *           fb_setcursor_s */
#endif

#ifdef CONFIG_LCD_UPDATE
#  define FBIO_UPDATE         _FBIOC(0x0007)  /* Update a rectangular region in
                                               * the framebuffer
                                               * Argument: read-only struct
                                               *           nxgl_rect_s */
#endif

#ifdef CONFIG_FB_SYNC
#  define FBIO_WAITFORVSYNC   _FBIOC(0x0008)  /* Wait for vertical sync */
#endif

#ifdef CONFIG_FB_OVERLAY
#  define FBIOGET_OVERLAYINFO _FBIOC(0x0009)  /* Get video overlay info */
                                              /* Argument: writable struct
                                               *           fb_overlayinfo_s */
#  define FBIO_SELECT_OVERLAY _FBIOC(0x000a)  /* Select overlay */
                                              /* Argument: read-only
                                               *           unsigned long */
#  define FBIOSET_TRANSP      _FBIOC(0x000b)  /* Set opacity or transparency
                                               * Argument: read-only struct
                                               *           fb_overlayinfo_s */
#  define FBIOSET_CHROMAKEY   _FBIOC(0x000c)  /* Set chroma key
                                               * Argument: read-only struct
                                               *           fb_overlayinfo_s */
#  define FBIOSET_COLOR       _FBIOC(0x000d)  /* Set color
                                               * Aŕgument: read-only struct
                                               *           fb_overlayinfo_s */
#  define FBIOSET_BLANK       _FBIOC(0x000e)  /* Blank or unblank
                                               * Argument: read-only struct
                                               *           fb_overlayinfo_s */
#  define FBIOSET_AREA        _FBIOC(0x000f)  /* Set active overlay area
                                               * Argument: read-only struct
                                               *           fb_overlayinfo_s */
#ifdef CONFIG_FB_OVERLAY_BLIT
#  define FBIOSET_BLIT        _FBIOC(0x0010)  /* Blit area between overlays
                                               * Argument: read-only struct
                                               *           fb_overlayblit_s */
#  define FBIOSET_BLEND       _FBIOC(0x0011)  /* Blend area between overlays
                                               * Argument: read-only struct
                                               *           fb_overlayblend_s */
#endif
#endif /* CONFIG_FB_OVERLAY */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* If any dimension of the display exceeds 65,536 pixels, then the following
 * type will need to change:
 */

typedef uint16_t fb_coord_t;

/* This structure describes the overall video controller */

struct fb_videoinfo_s
{
  uint8_t    fmt;         /* see FB_FMT_*  */
  fb_coord_t xres;        /* Horizontal resolution in pixel columns */
  fb_coord_t yres;        /* Vertical resolution in pixel rows */
  uint8_t    nplanes;     /* Number of color planes supported */
#ifdef CONFIG_FB_OVERLAY
  uint8_t    noverlays;   /* Number of overlays supported */
#endif
};

/* This structure describes one color plane.  Some YUV formats may support
 * up to 4 planes
 */

struct fb_planeinfo_s
{
  FAR void  *fbmem;       /* Start of frame buffer memory */
  size_t     fblen;       /* Length of frame buffer memory in bytes */
  fb_coord_t stride;      /* Length of a line in bytes */
  uint8_t    display;     /* Display number */
  uint8_t    bpp;         /* Bits per pixel */
};

#ifdef CONFIG_FB_OVERLAY
/* This structure describes the transparency. */

struct fb_transp_s
{
  uint8_t    transp;      /* Transparency */
  uint8_t    transp_mode; /* Transparency mode */
};

/* This structure describes an area. */

struct fb_area_s
{
  fb_coord_t x;           /* x-offset of the area */
  fb_coord_t y;           /* y-offset of the area */
  fb_coord_t w;           /* Width of the area */
  fb_coord_t h;           /* Height of the area */
};

/* This structure describes one overlay. */

struct fb_overlayinfo_s
{
  FAR void   *fbmem;          /* Start of frame buffer memory */
  size_t     fblen;           /* Length of frame buffer memory in bytes */
  fb_coord_t stride;          /* Length of a line in bytes */
  uint8_t    overlay;         /* Overlay number */
  uint8_t    bpp;             /* Bits per pixel */
  uint8_t    blank;           /* Blank or unblank */
  uint32_t   chromakey;       /* Chroma key argb8888 formatted */
  uint32_t   color;           /* Color argb8888 formatted */
  struct fb_transp_s transp;  /* Transparency */
  struct fb_area_s sarea;     /* Selected area within the overlay */
  uint32_t   accl;            /* Supported hardware acceleration */
};

#ifdef CONFIG_FB_OVERLAY_BLIT
/* This structure describes an overlay area within a whole overlay */

struct fb_overlayarea_s
{
  uint8_t  overlay;      /* Number overlay */
  struct fb_area_s area; /* Overlay area */
};

/* This structure describes blit operation */

struct fb_overlayblit_s
{
  struct fb_overlayarea_s dest;       /* The destination overlay area */
  struct fb_overlayarea_s src;        /* The source overlay area */
};

/* This structure describes blend operation */

struct fb_overlayblend_s
{
  struct fb_overlayarea_s dest;       /* The destination overlay area */
  struct fb_overlayarea_s foreground; /* The foreground overlay area */
  struct fb_overlayarea_s background; /* The background overlay area */
};
#endif
#endif /* CONFIG_FB_OVERLAY */

/* On video controllers that support mapping of a pixel palette value
 * to an RGB encoding, the following structure may be used to define
 * that mapping.
 */

#ifdef CONFIG_FB_CMAP
struct fb_cmap_s
{
  uint16_t  first;        /* Offset offset first color entry in tables */
  uint16_t  len;          /* Number of color entries  in tables */

  /* Tables of  color component.  Any may be NULL if not used */

  uint8_t *red;           /* Table of 8-bit red values */
  uint8_t *green;         /* Table of 8-bit green values */
  uint8_t *blue;          /* Table of 8-bit blue values */
#ifdef CONFIG_FB_TRANSPARENCY
  uint8_t *transp;        /* Table of 8-bit transparency */
#endif
};
#endif

#ifdef CONFIG_FB_HWCURSOR
#ifdef CONFIG_FB_HWCURSORIMAGE
/* If the video controller hardware supports a hardware cursor and
 * that hardware cursor supports user-provided images, then the
 * following structure may be used to provide the cursor image
 */

struct fb_cursorimage_s
{
  fb_coord_t     width;    /* Width of the cursor image in pixels */
  fb_coord_t     height    /* Height of the cursor image in pixels */
  const uint8_t *image;    /* Pointer to image data */
};
#endif

/* The following structure defines the cursor position/size */

struct fb_cursorpos_s
{
  fb_coord_t x;            /* X position in pixels */
  fb_coord_t y;            /* Y position in rows */
};

/* If the hardware supports setting the cursor size, then this structure
 * is used to provide the size.
 */

#ifdef CONFIG_FB_HWCURSORSIZE
struct fb_cursorsize_s
{
  fb_coord_t h;            /* Height in rows */
  fb_coord_t w;            /* Width in pixels */
};
#endif

/* The following are used to get/get the cursor attributes via IOCTL command. */

struct fb_cursorattrib_s
{
#ifdef CONFIG_FB_HWCURSORIMAGE
  uint8_t fmt;                   /* Video format of cursor */
#endif
  struct fb_cursorpos_s  pos;    /* Current cursor position */
#ifdef CONFIG_FB_HWCURSORSIZE
  struct fb_cursorsize_s mxsize; /* Maximum cursor size */
  struct fb_cursorsize_s size;   /* Current size */
#endif
};

struct fb_setcursor_s
{
  uint8_t flags;                 /* See FB_CUR_* definitions */
  struct fb_cursorpos_s pos;     /* Cursor position */
#ifdef CONFIG_FB_HWCURSORSIZE
  struct fb_cursorsize_s  size;  /* Cursor size */
#endif
#ifdef CONFIG_FB_HWCURSORIMAGE
  struct fb_cursorimage_s img;   /* Cursor image */
#endif
};
#endif

/* The framebuffer "object" is accessed through within the OS via
 * the following vtable:
 */

struct fb_vtable_s
{
  /* Get information about the video controller configuration and the
   * configuration of each color plane.
   */

  int (*getvideoinfo)(FAR struct fb_vtable_s *vtable,
                      FAR struct fb_videoinfo_s *vinfo);
  int (*getplaneinfo)(FAR struct fb_vtable_s *vtable, int planeno,
                      FAR struct fb_planeinfo_s *pinfo);

#ifdef CONFIG_FB_CMAP
  /* The following are provided only if the video hardware supports RGB
   * color mapping
   */

  int (*getcmap)(FAR struct fb_vtable_s *vtable,
                 FAR struct fb_cmap_s *cmap);
  int (*putcmap)(FAR struct fb_vtable_s *vtable,
                 FAR const struct fb_cmap_s *cmap);
#endif

#ifdef CONFIG_FB_HWCURSOR
  /* The following are provided only if the video hardware supports a
   * hardware cursor.
   */

  int (*getcursor)(FAR struct fb_vtable_s *vtable,
                   FAR struct fb_cursorattrib_s *attrib);
  int (*setcursor)(FAR struct fb_vtable_s *vtable,
                   FAR struct fb_setcursor_s *settings);
#endif

#ifdef CONFIG_FB_SYNC
  /* The following are provided only if the video hardware signals
   * vertical snyc.
   */

  int (*waitforvsync)(FAR struct fb_vtable_s *vtable);
#endif

#ifdef CONFIG_FB_OVERLAY
  /* Get information about the video controller configuration and the
   * configuration of each overlay.
   */

  int (*getoverlayinfo)(FAR struct fb_vtable_s *vtable, int overlayno,
                        FAR struct fb_overlayinfo_s *oinfo);

  /* The following are provided only if the video hardware supports
   * transparency
   */

  int (*settransp)(FAR struct fb_vtable_s *vtable,
                   FAR const struct fb_overlayinfo_s *oinfo);

  /* The following are provided only if the video hardware supports
   * chromakey
   */

  int (*setchromakey)(FAR struct fb_vtable_s *vtable,
                      FAR const struct fb_overlayinfo_s *oinfo);

  /* The following are provided only if the video hardware supports
   * filling the overlay with a color.
   */

  int (*setcolor)(FAR struct fb_vtable_s *vtable,
                  FAR const struct fb_overlayinfo_s *oinfo);

  /* The following allows to switch the overlay on or off */

  int (*setblank)(FAR struct fb_vtable_s *vtable,
                  FAR const struct fb_overlayinfo_s *oinfo);

  /* The following allows to set the active area for subsequently overlay
   * operations.
   */

  int (*setarea)(FAR struct fb_vtable_s *vtable,
                 FAR const struct fb_overlayinfo_s *oinfo);

# ifdef CONFIG_FB_OVERLAY_BLIT
  /* The following are provided only if the video hardware supports
   * blit operation between overlays.
   */

  int (*blit)(FAR struct fb_vtable_s *vtable,
              FAR const struct fb_overlayblit_s *blit);

  /* The following are provided only if the video hardware supports
   * blend operation between overlays.
   */

  int (*blend)(FAR struct fb_vtable_s *vtable,
               FAR const struct fb_overlayblend_s *blend);
# endif
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * If an architecture supports a framebuffer, then it must provide the
 * following APIs to access the framebuffer.
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer video hardware associated with the display.
 *
 *   There are multiple logic paths that may call up_fbinitialize() so any
 *   implementation of up_fbinitialize() should be tolerant of being called
 *   multiple times.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *             specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int up_fbinitialize(int display);

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane of the specified plane.  Many OSDs support multiple planes of video.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *             specifies the display.  Normally this is zero.
 *   vplane  - Identifies the plane being queried.
 *
 * Returned Value:
 *   A non-NULL pointer to the frame buffer access structure is returned on
 *   success; NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct fb_vtable_s *up_fbgetvplane(int display, int vplane);

/****************************************************************************
 * Name: up_fbuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer support for the specified display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *             specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_fbuninitialize(int display);

/****************************************************************************
 * Name: fb_register
 *
 * Description:
 *   Register the framebuffer character device at /dev/fbN where N is the
 *   display number if the devices supports only a single plane.  If the
 *   hardware supports multiple color planes, then the device will be
 *   registered at /dev/fbN.M where N is the again display number but M
 *   is the display plane.
 *
 * Input Parameters:
 *   display - The display number for the case of boards supporting multiple
 *             displays or for hardware that supports multiple
 *             layers (each layer is consider a display).  Typically zero.
 *   plane   - Identifies the color plane on hardware that supports separate
 *             framebuffer "planes" for each color component.
 *
 * Returned Value:
 *   Zero (OK) is returned success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int fb_register(int display, int plane);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_FB_H */
