/****************************************************************************
 * include/nuttx/video/fb.h
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

/* Color format definitions.  This pretty much defines the color pixel
 * processing organization of the video controller.
 */

/* Monochrome Formats *******************************************************/

#define FB_FMT_Y1             0         /* BPP=1, monochrome */
#define FB_FMT_Y2             1         /* BPP=2, 2-bit uncompressed greyscale */
#define FB_FMT_Y4             2         /* BPP=4, 4-bit uncompressed greyscale */
#define FB_FMT_Y8             3         /* BPP=8, 8-bit uncompressed greyscale */
#define FB_FMT_Y16            4         /* BPP=16, 16-bit uncompressed greyscale */
#define FB_FMT_GREY           FB_FMT_Y8 /* BPP=8 */
#define FB_FMT_Y800           FB_FMT_Y8 /* BPP=8 */

#define FB_ISMONO(f)          (((f) >= FB_FMT_Y1) && (f) <= FB_FMT_Y16)

/* RGB video formats ********************************************************/

/* Standard RGB */

#define FB_FMT_RGB4           5           /* BPP=4 */
#define FB_FMT_RGB8           6           /* BPP=8 RGB palette index */
#define FB_FMT_RGB8_222       7           /* BPP=8  R=2, G=2, B=2 */
#define FB_FMT_RGB8_332       8           /* BPP=8  R=3, G=3, B=2 */
#define FB_FMT_RGB12_444      9           /* BPP=12 R=4, G=4, B=4 */
#define FB_FMT_RGB16_555      10          /* BPP=16 R=5, G=5, B=5 (1 unused bit) */
#define FB_FMT_RGB16_565      11          /* BPP=16 R=5, G=6, B=5 */
#define FB_FMT_RGB24          12          /* BPP=24 */
#define FB_FMT_RGB32          13          /* BPP=32 */

/* Run length encoded RGB */

#define FB_FMT_RGBRLE4        14          /* BPP=4 */
#define FB_FMT_RGBRLE8        15          /* BPP=8 */

/* Raw RGB */

#define FB_FMT_RGBRAW         16          /* BPP=? */

/* Raw RGB with arbitrary sample packing within a pixel. Packing and
 * precision of R, G and B components is determined by bit masks for each.
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

#define FB_ISRGB(f)           (((f) >= FB_FMT_RGB4) && (f) <= FB_FMT_RGBT32)

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

#define FB_ISYUVPACKED(f)     (((f) >= FB_FMT_AYUV) && (f) <= FB_FMT_YUVP)

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

#define FB_ISYUVPLANAR(f)     (((f) >= FB_FMT_YVU9) && (f) <= FB_FMT_CXY2)
#define FB_ISYUV(f)           (FB_ISYUVPACKED(f) || FB_ISYUVPLANAR(f))

/* Hardware cursor control **************************************************/

#ifdef CONFIG_FB_HWCURSOR
#  define FB_CUR_ENABLE       0x01        /* Enable the cursor */
#  define FB_CUR_SETIMAGE     0x02        /* Set the cursor image */
#  define FB_CUR_SETPOSITION  0x04        /* Set the position of the cursor */
#  define FB_CUR_SETSIZE      0x08        /* Set the size of the cursor */
#  define FB_CUR_XOR          0x10        /* Use XOR vs COPY ROP on image */
#endif

/* Hardware overlay acceleration ********************************************/

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

#ifdef CONFIG_FB_UPDATE
#  define FBIO_UPDATE         _FBIOC(0x0007)  /* Update a rectangular region in
                                               * the framebuffer
                                               * Argument: read-only struct
                                               *           fb_area_s */
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

/* Specific Controls ********************************************************/

#define FBIOSET_POWER         _FBIOC(0x0012)  /* Set panel power
                                               * Argument:             int */
#define FBIOGET_POWER         _FBIOC(0x0013)  /* Get panel current power
                                               * Argument:            int* */
#define FBIOSET_FRAMERATE     _FBIOC(0x0014)  /* Set frame rate
                                               * Argument:             int */
#define FBIOGET_FRAMERATE     _FBIOC(0x0015)  /* Get frame rate
                                               * Argument:            int* */

#define FBIOPAN_DISPLAY       _FBIOC(0x0016)  /* Pan display
                                               * Argument: read-only struct
                                               *           fb_planeinfo_s* */

/* Linux Support ************************************************************/

#define FBIOGET_VSCREENINFO   _FBIOC(0x0017)  /* Get video variable info */
                                              /* Argument: writable struct
                                               *           fb_var_screeninfo */
#define FBIOGET_FSCREENINFO   _FBIOC(0x0018)  /* Get video fix info */
                                              /* Argument: writable struct
                                               *           fb_fix_screeninfo */

#define FB_TYPE_PACKED_PIXELS        0      /* Packed Pixels */
#define FB_TYPE_PLANES               1      /* Non interleaved planes */
#define FB_TYPE_INTERLEAVED_PLANES   2      /* Interleaved planes */
#define FB_TYPE_TEXT                 3      /* Text/attributes */
#define FB_TYPE_VGA_PLANES           4      /* EGA/VGA planes  */
#define FB_TYPE_FOURCC               5      /* Type identified by a V4L2 FOURCC */

#define FB_AUX_TEXT_MDA              0      /* Monochrome text */
#define FB_AUX_TEXT_CGA              1      /* CGA/EGA/VGA Color text */
#define FB_AUX_TEXT_S3_MMIO          2      /* S3 MMIO fasttext */
#define FB_AUX_TEXT_MGA_STEP16       3      /* MGA Millenium I: text, attr, */
                                            /* 14 reserved bytes */
#define FB_AUX_TEXT_MGA_STEP8        4      /* other MGAs: text, attr, */
                                            /* 6 reserved bytes */
#define FB_AUX_TEXT_SVGA_GROUP       8      /* 8-15: SVGA tileblit */
                                            /* compatible modes */
#define FB_AUX_TEXT_SVGA_MASK        7      /* lower three bits says step */
#define FB_AUX_TEXT_SVGA_STEP2       8      /* SVGA text mode: text, attr */
#define FB_AUX_TEXT_SVGA_STEP4       9      /* SVGA text mode: text, attr, */
                                            /* 2 reserved bytes */
#define FB_AUX_TEXT_SVGA_STEP8       10     /* SVGA text mode: text, attr, */
                                            /* 6 reserved bytes */
#define FB_AUX_TEXT_SVGA_STEP16      11     /* SVGA text mode: text, attr, */
                                            /* 14 reserved bytes */
#define FB_AUX_TEXT_SVGA_LAST        15     /* reserved up to 15 */

#define FB_AUX_VGA_PLANES_VGA4       0      /* 16 color planes (EGA/VGA) */
#define FB_AUX_VGA_PLANES_CFB4       1      /* CFB4 in planes (VGA) */
#define FB_AUX_VGA_PLANES_CFB8       2      /* CFB8 in planes (VGA) */

#define FB_VISUAL_MONO01             0      /* Monochr. 1=Black 0=White */
#define FB_VISUAL_MONO10             1      /* Monochr. 1=White 0=Black */
#define FB_VISUAL_TRUECOLOR          2      /* True color */
#define FB_VISUAL_PSEUDOCOLOR        3      /* Pseudo color (like atari) */
#define FB_VISUAL_DIRECTCOLOR        4      /* Direct color */
#define FB_VISUAL_STATIC_PSEUDOCOLOR 5      /* Pseudo color readonly */
#define FB_VISUAL_FOURCC             6      /* Visual identified by */
                                            /* a V4L2 FOURCC */

#define FB_ACCEL_NONE                0      /* no hardware accelerator */
#define FB_ACCEL_ATARIBLITT          1      /* Atari Blitter */
#define FB_ACCEL_AMIGABLITT          2      /* Amiga Blitter */
#define FB_ACCEL_S3_TRIO64           3      /* Cybervision64 (S3 Trio64) */
#define FB_ACCEL_NCR_77C32BLT        4      /* RetinaZ3 (NCR 77C32BLT) */
#define FB_ACCEL_S3_VIRGE            5      /* Cybervision64/3D (S3 ViRGE) */
#define FB_ACCEL_ATI_MACH64GX        6      /* ATI Mach 64GX family */
#define FB_ACCEL_DEC_TGA             7      /* DEC 21030 TGA */
#define FB_ACCEL_ATI_MACH64CT        8      /* ATI Mach 64CT family */
#define FB_ACCEL_ATI_MACH64VT        9      /* ATI Mach 64CT family VT class */
#define FB_ACCEL_ATI_MACH64GT        10     /* ATI Mach 64CT family GT class */
#define FB_ACCEL_SUN_CREATOR         11     /* Sun Creator/Creator3D */
#define FB_ACCEL_SUN_CGSIX           12     /* Sun cg6 */
#define FB_ACCEL_SUN_LEO             13     /* Sun leo/zx */
#define FB_ACCEL_IMS_TWINTURBO       14     /* IMS Twin Turbo */
#define FB_ACCEL_3DLABS_PERMEDIA2    15     /* 3Dlabs Permedia 2 */
#define FB_ACCEL_MATROX_MGA2064W     16     /* Matrox MGA2064W (Millenium) */
#define FB_ACCEL_MATROX_MGA1064SG    17     /* Matrox MGA1064SG (Mystique) */
#define FB_ACCEL_MATROX_MGA2164W     18     /* Matrox MGA2164W (Millenium II) */
#define FB_ACCEL_MATROX_MGA2164W_AGP 19     /* Matrox MGA2164W (Millenium II) */
#define FB_ACCEL_MATROX_MGAG100      20     /* Matrox G100 (Productiva G100) */
#define FB_ACCEL_MATROX_MGAG200      21     /* Matrox G200 (Myst, Mill, ...) */
#define FB_ACCEL_SUN_CG14            22     /* Sun cgfourteen */
#define FB_ACCEL_SUN_BWTWO           23     /* Sun bwtwo */
#define FB_ACCEL_SUN_CGTHREE         24     /* Sun cgthree */
#define FB_ACCEL_SUN_TCX             25     /* Sun tcx */
#define FB_ACCEL_MATROX_MGAG400      26     /* Matrox G400 */
#define FB_ACCEL_NV3                 27     /* nVidia RIVA 128 */
#define FB_ACCEL_NV4                 28     /* nVidia RIVA TNT */
#define FB_ACCEL_NV5                 29     /* nVidia RIVA TNT2 */
#define FB_ACCEL_CT_6555x            30     /* C&T 6555x */
#define FB_ACCEL_3DFX_BANSHEE        31     /* 3Dfx Banshee */
#define FB_ACCEL_ATI_RAGE128         32     /* ATI Rage128 family */
#define FB_ACCEL_IGS_CYBER2000       33     /* CyberPro 2000 */
#define FB_ACCEL_IGS_CYBER2010       34     /* CyberPro 2010 */
#define FB_ACCEL_IGS_CYBER5000       35     /* CyberPro 5000 */
#define FB_ACCEL_SIS_GLAMOUR         36     /* SiS 300/630/540 */
#define FB_ACCEL_3DLABS_PERMEDIA3    37     /* 3Dlabs Permedia 3 */
#define FB_ACCEL_ATI_RADEON          38     /* ATI Radeon family */
#define FB_ACCEL_I810                39     /* Intel 810/815 */
#define FB_ACCEL_SIS_GLAMOUR_2       40     /* SiS 315, 650, 740 */
#define FB_ACCEL_SIS_XABRE           41     /* SiS 330 ("Xabre") */
#define FB_ACCEL_I830                42     /* Intel 830M/845G/85x/865G */
#define FB_ACCEL_NV_10               43     /* nVidia Arch 10 */
#define FB_ACCEL_NV_20               44     /* nVidia Arch 20 */
#define FB_ACCEL_NV_30               45     /* nVidia Arch 30 */
#define FB_ACCEL_NV_40               46     /* nVidia Arch 40 */
#define FB_ACCEL_XGI_VOLARI_V        47     /* XGI Volari V3XT, V5, V8 */
#define FB_ACCEL_XGI_VOLARI_Z        48     /* XGI Volari Z7 */
#define FB_ACCEL_OMAP1610            49     /* TI OMAP16xx */
#define FB_ACCEL_TRIDENT_TGUI        50     /* Trident TGUI */
#define FB_ACCEL_TRIDENT_3DIMAGE     51     /* Trident 3DImage */
#define FB_ACCEL_TRIDENT_BLADE3D     52     /* Trident Blade3D */
#define FB_ACCEL_TRIDENT_BLADEXP     53     /* Trident BladeXP */
#define FB_ACCEL_CIRRUS_ALPINE       53     /* Cirrus Logic 543x/544x/5480 */
#define FB_ACCEL_NEOMAGIC_NM2070     90     /* NeoMagic NM2070 */
#define FB_ACCEL_NEOMAGIC_NM2090     91     /* NeoMagic NM2090 */
#define FB_ACCEL_NEOMAGIC_NM2093     92     /* NeoMagic NM2093 */
#define FB_ACCEL_NEOMAGIC_NM2097     93     /* NeoMagic NM2097 */
#define FB_ACCEL_NEOMAGIC_NM2160     94     /* NeoMagic NM2160 */
#define FB_ACCEL_NEOMAGIC_NM2200     95     /* NeoMagic NM2200 */
#define FB_ACCEL_NEOMAGIC_NM2230     96     /* NeoMagic NM2230 */
#define FB_ACCEL_NEOMAGIC_NM2360     97     /* NeoMagic NM2360 */
#define FB_ACCEL_NEOMAGIC_NM2380     98     /* NeoMagic NM2380 */
#define FB_ACCEL_PXA3XX              99     /* PXA3xx */

#define FB_ACCEL_SAVAGE4             0x80   /* S3 Savage4 */
#define FB_ACCEL_SAVAGE3D            0x81   /* S3 Savage3D */
#define FB_ACCEL_SAVAGE3D_MV         0x82   /* S3 Savage3D-MV */
#define FB_ACCEL_SAVAGE2000          0x83   /* S3 Savage2000 */
#define FB_ACCEL_SAVAGE_MX_MV        0x84   /* S3 Savage/MX-MV */
#define FB_ACCEL_SAVAGE_MX           0x85   /* S3 Savage/MX */
#define FB_ACCEL_SAVAGE_IX_MV        0x86   /* S3 Savage/IX-MV */
#define FB_ACCEL_SAVAGE_IX           0x87   /* S3 Savage/IX */
#define FB_ACCEL_PROSAVAGE_PM        0x88   /* S3 ProSavage PM133 */
#define FB_ACCEL_PROSAVAGE_KM        0x89   /* S3 ProSavage KM133 */
#define FB_ACCEL_S3TWISTER_P         0x8a   /* S3 Twister */
#define FB_ACCEL_S3TWISTER_K         0x8b   /* S3 TwisterK */
#define FB_ACCEL_SUPERSAVAGE         0x8c   /* S3 Supersavage */
#define FB_ACCEL_PROSAVAGE_DDR       0x8d   /* S3 ProSavage DDR */
#define FB_ACCEL_PROSAVAGE_DDRK      0x8e   /* S3 ProSavage DDR-K */

#define FB_ACCEL_PUV3_UNIGFX         0xa0   /* PKUnity-v3 Unigfx */

#define FB_NONSTD_HAM                1      /* Hold-And-Modify (HAM) */
#define FB_NONSTD_REV_PIX_IN_B       2      /* Order of pixels in each byte */
                                            /* is reversed */

#define FB_ACTIVATE_NOW              0      /* Set values immediately (or vbl) */
#define FB_ACTIVATE_NXTOPEN          1      /* Activate on next open */
#define FB_ACTIVATE_TEST             2      /* Don't set, round up impossible */
#define FB_ACTIVATE_MASK             15     /* values */
#define FB_ACTIVATE_VBL              16     /* Activate values on next vbl  */
#define FB_CHANGE_CMAP_VBL           32     /* Change colormap on vbl */
#define FB_ACTIVATE_ALL              64     /* Change all VCs on this fb */
#define FB_ACTIVATE_FORCE            128    /* Force apply even when no change */
#define FB_ACTIVATE_INV_MODE         256    /* Invalidate videomode */

#define FB_ACCELF_TEXT               1      /* (OBSOLETE) See fb_info.flags */
                                            /* and vc_mode */

#define FB_SYNC_HOR_HIGH_ACT         1      /* Horizontal sync high active */
#define FB_SYNC_VERT_HIGH_ACT        2      /* Vertical sync high active */
#define FB_SYNC_EXT                  4      /* External sync */
#define FB_SYNC_COMP_HIGH_ACT        8      /* Composite sync high active */
#define FB_SYNC_BROADCAST            16     /* Broadcast video timings */
                                            /* vtotal = 144d/288n/576i => PAL */
                                            /* vtotal = 121d/242n/484i => NTSC */
#define FB_SYNC_ON_GREEN             32     /* Sync on green */

#define FB_VMODE_NONINTERLACED       0      /* Non interlaced */
#define FB_VMODE_INTERLACED          1      /* Interlaced */
#define FB_VMODE_DOUBLE              2      /* Double scan */
#define FB_VMODE_ODD_FLD_FIRST       4      /* Interlaced: top line first */
#define FB_VMODE_MASK                255

#define FB_VMODE_YWRAP               256    /* Ywrap instead of panning */
#define FB_VMODE_SMOOTH_XPAN         512    /* Smooth xpan possible */
                                            /* (internally used) */
#define FB_VMODE_CONUPDATE           512    /* Don't update x/yoffset */

#define FB_ROTATE_UR                 0
#define FB_ROTATE_CW                 1
#define FB_ROTATE_UD                 2
#define FB_ROTATE_CCW                3

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
  uint8_t    fmt;               /* see FB_FMT_*  */
  fb_coord_t xres;              /* Horizontal resolution in pixel columns */
  fb_coord_t yres;              /* Vertical resolution in pixel rows */
  uint8_t    nplanes;           /* Number of color planes supported */
#ifdef CONFIG_FB_OVERLAY
  uint8_t    noverlays;         /* Number of overlays supported */
#endif
#ifdef CONFIG_FB_MODULEINFO
  uint8_t    moduleinfo[128];   /* Module information filled by vendor */
#endif
};

/* This structure describes one color plane.  Some YUV formats may support
 * up to 4 planes
 */

struct fb_planeinfo_s
{
  FAR void  *fbmem;        /* Start of frame buffer memory */
  size_t     fblen;        /* Length of frame buffer memory in bytes */
  fb_coord_t stride;       /* Length of a line in bytes */
  uint8_t    display;      /* Display number */
  uint8_t    bpp;          /* Bits per pixel */
  uint32_t   xres_virtual; /* Virtual Horizontal resolution in pixel columns */
  uint32_t   yres_virtual; /* Virtual Vertical resolution in pixel rows */
  uint32_t   xoffset;      /* Offset from virtual to visible resolution */
  uint32_t   yoffset;      /* Offset from virtual to visible resolution */
};

/* This structure describes an area. */

struct fb_area_s
{
  fb_coord_t x;           /* x-offset of the area */
  fb_coord_t y;           /* y-offset of the area */
  fb_coord_t w;           /* Width of the area */
  fb_coord_t h;           /* Height of the area */
};

#ifdef CONFIG_FB_OVERLAY
/* This structure describes the transparency. */

struct fb_transp_s
{
  uint8_t    transp;      /* Transparency */
  uint8_t    transp_mode; /* Transparency mode */
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

/* The following are used to get/set the cursor attributes via IOCTL
 * command.
 */

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

#ifdef CONFIG_FB_UPDATE
  /* The following are provided only if the video hardware need extera
   * notification to update display content.
   */

  int (*updatearea)(FAR struct fb_vtable_s *vtable,
                    FAR const struct fb_area_s *area);
#endif

#ifdef CONFIG_FB_SYNC
  /* The following are provided only if the video hardware signals
   * vertical sync.
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

  /* Pan display for multiple buffers. */

  int (*pandisplay)(FAR struct fb_vtable_s *vtable,
                    FAR struct fb_planeinfo_s *pinfo);

  /* Specific Controls ******************************************************/

  /* Set the frequency of the framebuffer update panel (0: disable refresh) */

  int (*setframerate)(FAR struct fb_vtable_s *vtable, int rate);

  /* Get the frequency of the framebuffer update panel (0: disable refresh) */

  int (*getframerate)(FAR struct fb_vtable_s *vtable);

  /* Get the panel power status (0: full off). */

  int (*getpower)(FAR struct fb_vtable_s *vtable);

  /* Enable/disable panel power (0: full off). */

  int (*setpower)(FAR struct fb_vtable_s *vtable, int power);

  /* Pointer to framebuffer device private data. */

  FAR void *priv;
};

/* Linux Support ************************************************************/

struct fb_fix_screeninfo
{
  char          id[16];       /* Identification string eg "TT Builtin" */
  unsigned long smem_start;   /* Start of frame buffer mem */
                              /* (physical address) */
  uint32_t      smem_len;     /* Length of frame buffer mem */
  uint32_t      type;         /* See FB_TYPE_* */
  uint32_t      type_aux;     /* Interleave for interleaved Planes */
  uint32_t      visual;       /* See FB_VISUAL_* */
  uint16_t      xpanstep;     /* Zero if no hardware panning  */
  uint16_t      ypanstep;     /* Zero if no hardware panning  */
  uint16_t      ywrapstep;    /* Zero if no hardware ywrap    */
  uint32_t      line_length;  /* Length of a line in bytes    */
  unsigned long mmio_start;   /* Start of Memory Mapped I/O */
                              /* (physical address) */
  uint32_t      mmio_len;     /* Length of Memory Mapped I/O  */
  uint32_t      accel;        /* Indicate to driver which */
                              /* specific chip/card we have */
  uint16_t      capabilities; /* See FB_CAP_* */
  uint16_t      reserved[2];  /* Reserved for future compatibility */
};

/* Interpretation of offset for color fields: All offsets are from the right,
 * inside a "pixel" value, which is exactly 'bits_per_pixel' wide (means: you
 * can use the offset as right argument to <<). A pixel afterwards is a bit
 * stream and is written to video memory as that unmodified.
 *
 * For pseudocolor: offset and length should be the same for all color
 * components. Offset specifies the position of the least significant bit
 * of the pallette index in a pixel value. Length indicates the number
 * of available palette entries (i.e. # of entries = 1 << length).
 */

struct fb_bitfield
{
  uint32_t offset;    /* Beginning of bitfield */
  uint32_t length;    /* Length of bitfield */
  uint32_t msb_right; /* != 0 : Most significant bit is  right */
};

struct fb_var_screeninfo
{
  uint32_t xres;             /* Visible resolution */
  uint32_t yres;
  uint32_t xres_virtual;     /* Virtual resolution */
  uint32_t yres_virtual;
  uint32_t xoffset;          /* Offset from virtual to visible */
  uint32_t yoffset;          /* Resolution */
  uint32_t bits_per_pixel;   /* Guess what */
  uint32_t grayscale;        /* 0 = color, 1 = grayscale, >1 = FOURCC */
  struct fb_bitfield red;    /* Bitfield in fb mem if true color, */
  struct fb_bitfield green;  /* else only length is significant */
  struct fb_bitfield blue;
  struct fb_bitfield transp; /* Transparency */
  uint32_t nonstd;           /* != 0 Non standard pixel format */
  uint32_t activate;         /* See FB_ACTIVATE_* */
  uint32_t height;           /* Height of picture in mm */
  uint32_t width;            /* Width of picture in mm */
  uint32_t accel_flags;      /* (OBSOLETE) See fb_info.flags */

  /* Timing: All values in pixclocks, except pixclock (of course) */

  uint32_t pixclock;         /* Pixel clock in ps (pico seconds) */
  uint32_t left_margin;      /* Time from sync to picture */
  uint32_t right_margin;     /* Time from picture to sync */
  uint32_t upper_margin;     /* Time from sync to picture */
  uint32_t lower_margin;
  uint32_t hsync_len;        /* Length of horizontal sync */
  uint32_t vsync_len;        /* Length of vertical sync */
  uint32_t sync;             /* See FB_SYNC_* */
  uint32_t vmode;            /* See FB_VMODE_* */
  uint32_t rotate;           /* Angle we rotate counter clockwise */
  uint32_t colorspace;       /* Colorspace for FOURCC-based modes */
  uint32_t reserved[4];      /* Reserved for future compatibility */
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
 *   plane of the specified plane.  Many OSDs support multiple planes of
 *   video.
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
 * Name: fb_pollnotify
 *
 * Description:
 *   Notify the waiting thread that the framebuffer can be written.
 *
 * Input Parameters:
 *   vtable - Pointer to framebuffer's virtual table.
 *
 ****************************************************************************/

void fb_pollnotify(FAR struct fb_vtable_s *vtable);

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
