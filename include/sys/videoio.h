/****************************************************************************
 * include/sys/videoio.h
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

#ifndef __INCLUDE_SYS_VIDEOIO_H
#define __INCLUDE_SYS_VIDEOIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/video_controls.h>

#include <nuttx/fs/ioctl.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIDEO_MAX_FRAME               32
#define VIDEO_MAX_PLANES               8

/*  Four-character-code (FOURCC) */

#define v4l2_fourcc(a, b, c, d)\
  ((uint32_t)(a)        | ((uint32_t)(b) << 8) | \
  ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))
#define v4l2_fourcc_be(a, b, c, d)    (v4l2_fourcc(a, b, c, d) | (1 << 31))

/* RGB formats */

#define V4L2_PIX_FMT_RGB332   v4l2_fourcc('R', 'G', 'B', '1')
#define V4L2_PIX_FMT_RGB444   v4l2_fourcc('R', '4', '4', '4')
#define V4L2_PIX_FMT_ARGB444  v4l2_fourcc('A', 'R', '1', '2')
#define V4L2_PIX_FMT_XRGB444  v4l2_fourcc('X', 'R', '1', '2')
#define V4L2_PIX_FMT_RGB555   v4l2_fourcc('R', 'G', 'B', 'O')
#define V4L2_PIX_FMT_ARGB555  v4l2_fourcc('A', 'R', '1', '5')
#define V4L2_PIX_FMT_XRGB555  v4l2_fourcc('X', 'R', '1', '5')
#define V4L2_PIX_FMT_RGB565   v4l2_fourcc('R', 'G', 'B', 'P')
#define V4L2_PIX_FMT_RGB555X  v4l2_fourcc('R', 'G', 'B', 'Q')
#define V4L2_PIX_FMT_ARGB555X v4l2_fourcc_be('A', 'R', '1', '5')
#define V4L2_PIX_FMT_XRGB555X v4l2_fourcc_be('X', 'R', '1', '5')
#define V4L2_PIX_FMT_RGB565X  v4l2_fourcc('R', 'G', 'B', 'R')
#define V4L2_PIX_FMT_BGR666   v4l2_fourcc('B', 'G', 'R', 'H')
#define V4L2_PIX_FMT_BGR24    v4l2_fourcc('B', 'G', 'R', '3')
#define V4L2_PIX_FMT_RGB24    v4l2_fourcc('R', 'G', 'B', '3')
#define V4L2_PIX_FMT_BGR32    v4l2_fourcc('B', 'G', 'R', '4')
#define V4L2_PIX_FMT_ABGR32   v4l2_fourcc('A', 'R', '2', '4')
#define V4L2_PIX_FMT_XBGR32   v4l2_fourcc('X', 'R', '2', '4')
#define V4L2_PIX_FMT_RGB32    v4l2_fourcc('R', 'G', 'B', '4')
#define V4L2_PIX_FMT_ARGB32   v4l2_fourcc('B', 'A', '2', '4')
#define V4L2_PIX_FMT_XRGB32   v4l2_fourcc('B', 'X', '2', '4')

/* Grey formats */

#define V4L2_PIX_FMT_GREY     v4l2_fourcc('G', 'R', 'E', 'Y')
#define V4L2_PIX_FMT_Y4       v4l2_fourcc('Y', '0', '4', ' ')
#define V4L2_PIX_FMT_Y6       v4l2_fourcc('Y', '0', '6', ' ')
#define V4L2_PIX_FMT_Y10      v4l2_fourcc('Y', '1', '0', ' ')
#define V4L2_PIX_FMT_Y12      v4l2_fourcc('Y', '1', '2', ' ')
#define V4L2_PIX_FMT_Y16      v4l2_fourcc('Y', '1', '6', ' ')
#define V4L2_PIX_FMT_Y16_BE   v4l2_fourcc_be('Y', '1', '6', ' ')

/* Grey bit-packed formats */

#define V4L2_PIX_FMT_Y10BPACK v4l2_fourcc('Y', '1', '0', 'B')

/* Palette formats */

#define V4L2_PIX_FMT_PAL8     v4l2_fourcc('P', 'A', 'L', '8')

/* Chrominance formats */

#define V4L2_PIX_FMT_UV8      v4l2_fourcc('U', 'V', '8', ' ')

/* Luminance+Chrominance formats */

#define V4L2_PIX_FMT_YUYV     v4l2_fourcc('Y', 'U', 'Y', 'V')
#define V4L2_PIX_FMT_YYUV     v4l2_fourcc('Y', 'Y', 'U', 'V')
#define V4L2_PIX_FMT_YVYU     v4l2_fourcc('Y', 'V', 'Y', 'U')
#define V4L2_PIX_FMT_UYVY     v4l2_fourcc('U', 'Y', 'V', 'Y')
#define V4L2_PIX_FMT_VYUY     v4l2_fourcc('V', 'Y', 'U', 'Y')
#define V4L2_PIX_FMT_Y41P     v4l2_fourcc('Y', '4', '1', 'P')
#define V4L2_PIX_FMT_YUV444   v4l2_fourcc('Y', '4', '4', '4')
#define V4L2_PIX_FMT_YUV555   v4l2_fourcc('Y', 'U', 'V', 'O')
#define V4L2_PIX_FMT_YUV565   v4l2_fourcc('Y', 'U', 'V', 'P')
#define V4L2_PIX_FMT_YUV32    v4l2_fourcc('Y', 'U', 'V', '4')
#define V4L2_PIX_FMT_HI240    v4l2_fourcc('H', 'I', '2', '4')
#define V4L2_PIX_FMT_HM12     v4l2_fourcc('H', 'M', '1', '2')
#define V4L2_PIX_FMT_M420     v4l2_fourcc('M', '4', '2', '0')

/* Two planes -- one Y, one Cr + Cb interleaved  */

#define V4L2_PIX_FMT_NV12     v4l2_fourcc('N', 'V', '1', '2')
#define V4L2_PIX_FMT_NV21     v4l2_fourcc('N', 'V', '2', '1')
#define V4L2_PIX_FMT_NV16     v4l2_fourcc('N', 'V', '1', '6')
#define V4L2_PIX_FMT_NV61     v4l2_fourcc('N', 'V', '6', '1')
#define V4L2_PIX_FMT_NV24     v4l2_fourcc('N', 'V', '2', '4')
#define V4L2_PIX_FMT_NV42     v4l2_fourcc('N', 'V', '4', '2')

/* Two non contiguous planes - one Y, one Cr + Cb interleaved  */

#define V4L2_PIX_FMT_NV12M    v4l2_fourcc('N', 'M', '1', '2')
#define V4L2_PIX_FMT_NV21M    v4l2_fourcc('N', 'M', '2', '1')
#define V4L2_PIX_FMT_NV16M    v4l2_fourcc('N', 'M', '1', '6')
#define V4L2_PIX_FMT_NV61M    v4l2_fourcc('N', 'M', '6', '1')
#define V4L2_PIX_FMT_NV12MT   v4l2_fourcc('T', 'M', '1', '2')
#define V4L2_PIX_FMT_NV12MT_16X16 v4l2_fourcc('V', 'M', '1', '2')

/* Three planes - Y Cb, Cr */

#define V4L2_PIX_FMT_YUV410   v4l2_fourcc('Y', 'U', 'V', '9')
#define V4L2_PIX_FMT_YVU410   v4l2_fourcc('Y', 'V', 'U', '9')
#define V4L2_PIX_FMT_YUV411P  v4l2_fourcc('4', '1', '1', 'P')
#define V4L2_PIX_FMT_YUV420   v4l2_fourcc('Y', 'U', '1', '2')
#define V4L2_PIX_FMT_YVU420   v4l2_fourcc('Y', 'V', '1', '2')
#define V4L2_PIX_FMT_YUV422P  v4l2_fourcc('4', '2', '2', 'P')

/* Three non contiguous planes - Y, Cb, Cr */

#define V4L2_PIX_FMT_YUV420M  v4l2_fourcc('Y', 'M', '1', '2')
#define V4L2_PIX_FMT_YVU420M  v4l2_fourcc('Y', 'M', '2', '1')
#define V4L2_PIX_FMT_YUV422M  v4l2_fourcc('Y', 'M', '1', '6')
#define V4L2_PIX_FMT_YVU422M  v4l2_fourcc('Y', 'M', '6', '1')
#define V4L2_PIX_FMT_YUV444M  v4l2_fourcc('Y', 'M', '2', '4')
#define V4L2_PIX_FMT_YVU444M  v4l2_fourcc('Y', 'M', '4', '2')

/* Bayer formats - see http://www.siliconimaging.com/RGB%20Bayer.htm */

#define V4L2_PIX_FMT_SBGGR8   v4l2_fourcc('B', 'A', '8', '1')
#define V4L2_PIX_FMT_SGBRG8   v4l2_fourcc('G', 'B', 'R', 'G')
#define V4L2_PIX_FMT_SGRBG8   v4l2_fourcc('G', 'R', 'B', 'G')
#define V4L2_PIX_FMT_SRGGB8   v4l2_fourcc('R', 'G', 'G', 'B')
#define V4L2_PIX_FMT_SBGGR10  v4l2_fourcc('B', 'G', '1', '0')
#define V4L2_PIX_FMT_SGBRG10  v4l2_fourcc('G', 'B', '1', '0')
#define V4L2_PIX_FMT_SGRBG10  v4l2_fourcc('B', 'A', '1', '0')
#define V4L2_PIX_FMT_SRGGB10  v4l2_fourcc('R', 'G', '1', '0')

/* 10bit raw bayer packed, 5 bytes for every 4 pixels */

#define V4L2_PIX_FMT_SBGGR10P v4l2_fourcc('p', 'B', 'A', 'A')
#define V4L2_PIX_FMT_SGBRG10P v4l2_fourcc('p', 'G', 'A', 'A')
#define V4L2_PIX_FMT_SGRBG10P v4l2_fourcc('p', 'g', 'A', 'A')
#define V4L2_PIX_FMT_SRGGB10P v4l2_fourcc('p', 'R', 'A', 'A')

/* 10bit raw bayer a-law compressed to 8 bits */

#define V4L2_PIX_FMT_SBGGR10ALAW8 v4l2_fourcc('a', 'B', 'A', '8')
#define V4L2_PIX_FMT_SGBRG10ALAW8 v4l2_fourcc('a', 'G', 'A', '8')
#define V4L2_PIX_FMT_SGRBG10ALAW8 v4l2_fourcc('a', 'g', 'A', '8')
#define V4L2_PIX_FMT_SRGGB10ALAW8 v4l2_fourcc('a', 'R', 'A', '8')

/* 10bit raw bayer DPCM compressed to 8 bits */

#define V4L2_PIX_FMT_SBGGR10DPCM8 v4l2_fourcc('b', 'B', 'A', '8')
#define V4L2_PIX_FMT_SGBRG10DPCM8 v4l2_fourcc('b', 'G', 'A', '8')
#define V4L2_PIX_FMT_SGRBG10DPCM8 v4l2_fourcc('B', 'D', '1', '0')
#define V4L2_PIX_FMT_SRGGB10DPCM8 v4l2_fourcc('b', 'R', 'A', '8')
#define V4L2_PIX_FMT_SBGGR12      v4l2_fourcc('B', 'G', '1', '2')
#define V4L2_PIX_FMT_SGBRG12      v4l2_fourcc('G', 'B', '1', '2')
#define V4L2_PIX_FMT_SGRBG12      v4l2_fourcc('B', 'A', '1', '2')
#define V4L2_PIX_FMT_SRGGB12      v4l2_fourcc('R', 'G', '1', '2')
#define V4L2_PIX_FMT_SBGGR16      v4l2_fourcc('B', 'Y', 'R', '2')
#define V4L2_PIX_FMT_SGBRG16      v4l2_fourcc('G', 'B', '1', '6')
#define V4L2_PIX_FMT_SGRBG16      v4l2_fourcc('G', 'R', '1', '6')
#define V4L2_PIX_FMT_SRGGB16      v4l2_fourcc('R', 'G', '1', '6')

/* HSV formats */

#define V4L2_PIX_FMT_HSV24    v4l2_fourcc('H', 'S', 'V', '3')
#define V4L2_PIX_FMT_HSV32    v4l2_fourcc('H', 'S', 'V', '4')

/* Compressed formats */

#define V4L2_PIX_FMT_MJPEG    v4l2_fourcc('M', 'J', 'P', 'G')
#define V4L2_PIX_FMT_JPEG     v4l2_fourcc('J', 'P', 'E', 'G')
#define V4L2_PIX_FMT_DV       v4l2_fourcc('d', 'v', 's', 'd')
#define V4L2_PIX_FMT_MPEG     v4l2_fourcc('M', 'P', 'E', 'G')
#define V4L2_PIX_FMT_H264     v4l2_fourcc('H', '2', '6', '4')
#define V4L2_PIX_FMT_H264_NO_SC v4l2_fourcc('A', 'V', 'C', '1')
#define V4L2_PIX_FMT_H264_MVC v4l2_fourcc('M', '2', '6', '4')
#define V4L2_PIX_FMT_H263     v4l2_fourcc('H', '2', '6', '3')
#define V4L2_PIX_FMT_MPEG1    v4l2_fourcc('M', 'P', 'G', '1')
#define V4L2_PIX_FMT_MPEG2    v4l2_fourcc('M', 'P', 'G', '2')
#define V4L2_PIX_FMT_MPEG4    v4l2_fourcc('M', 'P', 'G', '4')
#define V4L2_PIX_FMT_XVID     v4l2_fourcc('X', 'V', 'I', 'D')
#define V4L2_PIX_FMT_VC1_ANNEX_G v4l2_fourcc('V', 'C', '1', 'G')
#define V4L2_PIX_FMT_VC1_ANNEX_L v4l2_fourcc('V', 'C', '1', 'L')
#define V4L2_PIX_FMT_VP8      v4l2_fourcc('V', 'P', '8', '0')
#define V4L2_PIX_FMT_VP9      v4l2_fourcc('V', 'P', '9', '0')
#define V4L2_PIX_FMT_HEVC     v4l2_fourcc('H', 'E', 'V', 'C')

/* JPEG + sub image */

#define V4L2_PIX_FMT_JPEG_WITH_SUBIMG v4l2_fourcc('J', 'S', 'U', 'B')

/* YUV 4:2:2 for sub image */

#define V4L2_PIX_FMT_SUBIMG_UYVY v4l2_fourcc('S', 'Y', 'U', 'V')

/* RGB565 for sub image */

#define V4L2_PIX_FMT_SUBIMG_RGB565 v4l2_fourcc('S', 'R', 'G', 'B')

/* MAX length of v4l2_fmtdesc description string */

#define V4L2_FMT_DSC_MAX       (32)

/* MAX length of v4l2_query_ext_ctrl dims array */

#define V4L2_CTRL_MAX_DIMS     (4)

/* MAX value of VIDIOC_REQBUFS count parameter */

#define V4L2_REQBUFS_COUNT_MAX CONFIG_VIDEO_REQBUFS_COUNT_MAX

/* Buffer error flag */

#define V4L2_BUF_FLAG_ERROR    (0x0001)

/* Values for v4l2_std_id */

/* One bit for each */

#define V4L2_STD_PAL_B          ((v4l2_std_id)0x00000001)
#define V4L2_STD_PAL_B1         ((v4l2_std_id)0x00000002)
#define V4L2_STD_PAL_G          ((v4l2_std_id)0x00000004)
#define V4L2_STD_PAL_H          ((v4l2_std_id)0x00000008)
#define V4L2_STD_PAL_I          ((v4l2_std_id)0x00000010)
#define V4L2_STD_PAL_D          ((v4l2_std_id)0x00000020)
#define V4L2_STD_PAL_D1         ((v4l2_std_id)0x00000040)
#define V4L2_STD_PAL_K          ((v4l2_std_id)0x00000080)

#define V4L2_STD_PAL_M          ((v4l2_std_id)0x00000100)
#define V4L2_STD_PAL_N          ((v4l2_std_id)0x00000200)
#define V4L2_STD_PAL_Nc         ((v4l2_std_id)0x00000400)
#define V4L2_STD_PAL_60         ((v4l2_std_id)0x00000800)

#define V4L2_STD_NTSC_M         ((v4l2_std_id)0x00001000)    /* BTSC */
#define V4L2_STD_NTSC_M_JP      ((v4l2_std_id)0x00002000)    /* EIA-J */
#define V4L2_STD_NTSC_443       ((v4l2_std_id)0x00004000)
#define V4L2_STD_NTSC_M_KR      ((v4l2_std_id)0x00008000)    /* FM A2 */

#define V4L2_STD_SECAM_B        ((v4l2_std_id)0x00010000)
#define V4L2_STD_SECAM_D        ((v4l2_std_id)0x00020000)
#define V4L2_STD_SECAM_G        ((v4l2_std_id)0x00040000)
#define V4L2_STD_SECAM_H        ((v4l2_std_id)0x00080000)
#define V4L2_STD_SECAM_K        ((v4l2_std_id)0x00100000)
#define V4L2_STD_SECAM_K1       ((v4l2_std_id)0x00200000)
#define V4L2_STD_SECAM_L        ((v4l2_std_id)0x00400000)
#define V4L2_STD_SECAM_LC       ((v4l2_std_id)0x00800000)

/* ATSC/HDTV */

#define V4L2_STD_ATSC_8_VSB     ((v4l2_std_id)0x01000000)
#define V4L2_STD_ATSC_16_VSB    ((v4l2_std_id)0x02000000)

/* Some macros to merge video standards in order to make live easier for the
 * drivers and V4L2 applications
 */

/* "Common" NTSC/M - It should be noticed that V4L2_STD_NTSC_443 is
 * Missing here.
 */

#define V4L2_STD_NTSC           (V4L2_STD_NTSC_M     | \
                                 V4L2_STD_NTSC_M_JP  | \
                                 V4L2_STD_NTSC_M_KR)

/* Secam macros */

#define V4L2_STD_SECAM_DK       (V4L2_STD_SECAM_D    | \
                                 V4L2_STD_SECAM_K    | \
                                 V4L2_STD_SECAM_K1)

/* All Secam Standards */

#define V4L2_STD_SECAM          (V4L2_STD_SECAM_B    | \
                                 V4L2_STD_SECAM_G    | \
                                 V4L2_STD_SECAM_H    | \
                                 V4L2_STD_SECAM_DK   | \
                                 V4L2_STD_SECAM_L    | \
                                 V4L2_STD_SECAM_LC)

/* PAL macros */
#define V4L2_STD_PAL_BG         (V4L2_STD_PAL_B      | \
                                 V4L2_STD_PAL_B1     | \
                                 V4L2_STD_PAL_G)
#define V4L2_STD_PAL_DK         (V4L2_STD_PAL_D      | \
                                 V4L2_STD_PAL_D1     | \
                                 V4L2_STD_PAL_K)

/* "Common" PAL - This macro is there to be compatible with the old
 * V4L1 concept of "PAL": /BGDKHI.
 * Several PAL standards are missing here: /M, /N and /Nc
 */

#define V4L2_STD_PAL            (V4L2_STD_PAL_BG     | \
                                 V4L2_STD_PAL_DK     | \
                                 V4L2_STD_PAL_H      | \
                                 V4L2_STD_PAL_I)

/* Chroma "agnostic" standards */

#define V4L2_STD_B              (V4L2_STD_PAL_B      | \
                                 V4L2_STD_PAL_B1     | \
                                 V4L2_STD_SECAM_B)
#define V4L2_STD_G              (V4L2_STD_PAL_G      | \
                                 V4L2_STD_SECAM_G)
#define V4L2_STD_H              (V4L2_STD_PAL_H      | \
                                 V4L2_STD_SECAM_H)
#define V4L2_STD_L              (V4L2_STD_SECAM_L    | \
                                 V4L2_STD_SECAM_LC)
#define V4L2_STD_GH             (V4L2_STD_G          | \
                                 V4L2_STD_H)
#define V4L2_STD_DK             (V4L2_STD_PAL_DK     | \
                                 V4L2_STD_SECAM_DK)
#define V4L2_STD_BG             (V4L2_STD_B          | \
                                 V4L2_STD_G)
#define V4L2_STD_MN             (V4L2_STD_PAL_M      | \
                                 V4L2_STD_PAL_N      | \
                                 V4L2_STD_PAL_Nc     | \
                                 V4L2_STD_NTSC)

/* Standards where MTS/BTSC stereo could be found */

#define V4L2_STD_MTS            (V4L2_STD_NTSC_M     | \
                                 V4L2_STD_PAL_M      | \
                                 V4L2_STD_PAL_N      | \
                                 V4L2_STD_PAL_Nc)

/* Standards for Countries with 60Hz Line frequency */

#define V4L2_STD_525_60         (V4L2_STD_PAL_M      | \
                                 V4L2_STD_PAL_60     | \
                                 V4L2_STD_NTSC       | \
                                 V4L2_STD_NTSC_443)

/* Standards for Countries with 50Hz Line frequency */

#define V4L2_STD_625_50         (V4L2_STD_PAL        | \
                                 V4L2_STD_PAL_N      | \
                                 V4L2_STD_PAL_Nc     | \
                                 V4L2_STD_SECAM)
#define V4L2_STD_ATSC           (V4L2_STD_ATSC_8_VSB | \
                                 V4L2_STD_ATSC_16_VSB)

/* Macros with none and all analog standards */

#define V4L2_STD_UNKNOWN        0
#define V4L2_STD_ALL            (V4L2_STD_525_60     | \
                                 V4L2_STD_625_50)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* V4L2 device capabilities for VIDIOC_QUERYCAP.
 * Currently, only member "driver" is supported.
 */

struct v4l2_capability
{
  uint8_t  driver[16];   /* Name of driver module(e.g. "bttv" */
  uint8_t  card[32];     /* Name of the card(e.g. "Yoyodyne TV/FM" */
  uint8_t  bus_info[32]; /* Name of the bus(e.g. "PCI:0000:05:06.0" */
  uint32_t version;      /* Version number of the driver */
  uint32_t capabilities; /* Available capabilities of the physical device */
  uint32_t device_caps;  /* Device capabilities of the opened device */
};

/* Values for 'capabilities' field */

enum v4l2_capabilities
{
  V4L2_CAP_VIDEO_CAPTURE          = 0x00000001,  /* Is a video capture device */
  V4L2_CAP_VIDEO_OUTPUT           = 0x00000002,  /* Is a video output device */
  V4L2_CAP_VIDEO_OVERLAY          = 0x00000004,  /* Can do video overlay */
  V4L2_CAP_VBI_CAPTURE            = 0x00000010,  /* Is a raw VBI capture device */
  V4L2_CAP_VBI_OUTPUT             = 0x00000020,  /* Is a raw VBI output device */
  V4L2_CAP_SLICED_VBI_CAPTURE     = 0x00000040,  /* Is a sliced VBI capture device */
  V4L2_CAP_SLICED_VBI_OUTPUT      = 0x00000080,  /* Is a sliced VBI output device */
  V4L2_CAP_RDS_CAPTURE            = 0x00000100,  /* RDS data capture */
  V4L2_CAP_VIDEO_OUTPUT_OVERLAY   = 0x00000200,  /* Can do video output overlay */
  V4L2_CAP_HW_FREQ_SEEK           = 0x00000400,  /* Can do hardware frequency seek */
  V4L2_CAP_RDS_OUTPUT             = 0x00000800,  /* Is an RDS encoder */
  V4L2_CAP_VIDEO_CAPTURE_MPLANE   = 0x00001000,  /* Is a video capture device that supports multiplanar formats */
  V4L2_CAP_VIDEO_OUTPUT_MPLANE    = 0x00002000,  /* Is a video output device that supports multiplanar formats */
  V4L2_CAP_VIDEO_M2M_MPLANE       = 0x00004000,  /* Is a video mem-to-mem device that supports multiplanar formats */
  V4L2_CAP_VIDEO_M2M              = 0x00008000,  /* Is a video mem-to-mem device */
  V4L2_CAP_TUNER                  = 0x00010000,  /* Has a tuner */
  V4L2_CAP_AUDIO                  = 0x00020000,  /* Has audio support */
  V4L2_CAP_RADIO                  = 0x00040000,  /* Is a radio device */
  V4L2_CAP_MODULATOR              = 0x00080000,  /* Has a modulator */
  V4L2_CAP_SDR_CAPTURE            = 0x00100000,  /* Is a SDR capture device */
  V4L2_CAP_EXT_PIX_FORMAT         = 0x00200000,  /* Supports the extended pixel format */
  V4L2_CAP_SDR_OUTPUT             = 0x00400000,  /* Is a SDR output device */
  V4L2_CAP_READWRITE              = 0x01000000,  /* Read/write systemcalls */
  V4L2_CAP_ASYNCIO                = 0x02000000,  /* Async I/O */
  V4L2_CAP_STREAMING              = 0x04000000,  /* Streaming I/O ioctls */
  V4L2_CAP_TOUCH                  = 0x10000000,  /* Is a touch device */
  V4L2_CAP_DEVICE_CAPS            = 0x80000000,  /* Sets device capabilities field */
};

/* Rectangle information */

struct v4l2_rect
{
  /* Horizontal offset of the top, left corner of the rectangle, in pixels. */

  int32_t left;

  /* Vertical offset of the top, left corner of the rectangle, in pixels. */

  int32_t top;

  /* Width of the rectangle, in pixels. */

  uint32_t width;

  /* Height of the rectangle, in pixels. */

  uint32_t height;
};

/* Fraction */

struct v4l2_fract
{
  uint32_t numerator;                     /* Numerator */
  uint32_t denominator;                   /* Denominator */
};

/* V4L2 selection info for VIDIOC_S_SELECTION and VIDIOC_G_SELECTION.
 * Currently, only member type and r are supported.
 */

struct v4l2_selection
{
  uint32_t type;       /* Buffer type */
  uint32_t target;
  uint32_t flags;
  struct v4l2_rect r;  /* The selection rectangle. */
};

typedef uint64_t v4l2_std_id;

struct v4l2_standard
{
  uint32_t            index;
  v4l2_std_id         id;
  uint8_t             name[24];
  struct v4l2_fract   frameperiod; /* Frames, not fields */
  uint32_t            framelines;
  uint32_t            reserved[4];
};

/* Buffer type.
 *  Currently, support only V4L2_BUF_TYPE_VIDEO_CAPTURE and
 *  V4L2_BUF_TYPE_STILL_CAPTURE.
 */

enum v4l2_buf_type
{
  V4L2_BUF_TYPE_VIDEO_CAPTURE        = 1,    /* Single-planar video capture stream */
  V4L2_BUF_TYPE_VIDEO_OUTPUT         = 2,    /* Single-planar video output stream */
  V4L2_BUF_TYPE_VIDEO_OVERLAY        = 3,    /* Video overlay */
  V4L2_BUF_TYPE_VBI_CAPTURE          = 4,    /* Raw VBI capture stream */
  V4L2_BUF_TYPE_VBI_OUTPUT           = 5,    /* Raw VBI output stream */
  V4L2_BUF_TYPE_SLICED_VBI_CAPTURE   = 6,    /* Sliced VBI capture stream */
  V4L2_BUF_TYPE_SLICED_VBI_OUTPUT    = 7,    /* Sliced VBI output stream */
  V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY = 8,    /* Video output overlay  */
  V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE = 9,    /* Multi-planar video capture stream */
  V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE  = 10,   /* Multi-planar video output stream */
  V4L2_BUF_TYPE_SDR_CAPTURE          = 11,   /* Software Defined Radio capture stream */
  V4L2_BUF_TYPE_SDR_OUTPUT           = 12,   /* Software Defined Radio output stream */
  V4L2_BUF_TYPE_META_CAPTURE         = 13,   /* Metadata capture */
  V4L2_BUF_TYPE_META_OUTPUT          = 14,   /* Metadata output */
  V4L2_BUF_TYPE_PRIVATE              = 0x80, /* Deprecated, do not use */
  V4L2_BUF_TYPE_STILL_CAPTURE        = 0x81  /* Single-planar still capture stream */
};

#define V4L2_TYPE_IS_MULTIPLANAR(type)             \
  ((type) == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE    \
   || (type) == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)

#define V4L2_TYPE_IS_OUTPUT(type)                  \
  ((type) == V4L2_BUF_TYPE_VIDEO_OUTPUT            \
   || (type) == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE  \
   || (type) == V4L2_BUF_TYPE_VIDEO_OVERLAY        \
   || (type) == V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY \
   || (type) == V4L2_BUF_TYPE_VBI_OUTPUT           \
   || (type) == V4L2_BUF_TYPE_SLICED_VBI_OUTPUT    \
   || (type) == V4L2_BUF_TYPE_SDR_OUTPUT           \
   || (type) == V4L2_BUF_TYPE_META_OUTPUT)

#define V4L2_TYPE_IS_CAPTURE(type) (!V4L2_TYPE_IS_OUTPUT(type))

/* Memory I/O method. Currently, support only V4L2_MEMORY_USERPTR. */

enum v4l2_memory
{
  V4L2_MEMORY_MMAP         = 1,  /* Memory mapping I/O */
  V4L2_MEMORY_USERPTR      = 2,  /* User pointer I/O  */
  V4L2_MEMORY_OVERLAY      = 3,  /* Overlay I/O */
  V4L2_MEMORY_DMABUF       = 4,  /* DMA shared buffer I/O */
};

/* See also http://vektor.theorem.ca/graphics/ycbcr/ */

enum v4l2_colorspace
{
  /* Default colorspace, i.e. let the driver figure it out.
   *  Can only be used with video capture.
   */

  V4L2_COLORSPACE_DEFAULT       = 0,

  /* SMPTE 170M: used for broadcast NTSC/PAL SDTV */

  V4L2_COLORSPACE_SMPTE170M     = 1,

  /* Obsolete pre-1998 SMPTE 240M HDTV standard, superseded by Rec 709 */

  V4L2_COLORSPACE_SMPTE240M     = 2,

  /* Rec.709: used for HDTV */

  V4L2_COLORSPACE_REC709        = 3,

  /* Deprecated, do not use. No driver will ever return this. This was
   *  based on a misunderstanding of the bt878 datasheet.
   */

  V4L2_COLORSPACE_BT878         = 4,

  /* NTSC 1953 colorspace. This only makes sense when dealing with
   *  really, really old NTSC recordings. Superseded by SMPTE 170M.
   */

  V4L2_COLORSPACE_470_SYSTEM_M  = 5,

  /* EBU Tech 3213 PAL/SECAM colorspace. This only makes sense when
   * dealing with really old PAL/SECAM recordings. Superseded by
   * SMPTE 170M.
   */

  V4L2_COLORSPACE_470_SYSTEM_BG = 6,

  /* Effectively shorthand for V4L2_COLORSPACE_SRGB, V4L2_YCBCR_ENC_601
   *  and V4L2_QUANTIZATION_FULL_RANGE. To be used for (Motion-)JPEG.
   */

  V4L2_COLORSPACE_JPEG          = 7,

  /* For RGB colorspaces such as produces by most webcams. */

  V4L2_COLORSPACE_SRGB          = 8,

  /* opRGB colorspace */

  V4L2_COLORSPACE_OPRGB         = 9,

  /* BT.2020 colorspace, used for UHDTV. */

  V4L2_COLORSPACE_BT2020        = 10,

  /* Raw colorspace: for RAW unprocessed images */

  V4L2_COLORSPACE_RAW           = 11,

  /* DCI-P3 colorspace, used by cinema projectors */

  V4L2_COLORSPACE_DCI_P3        = 12,
};

/* Determine how COLORSPACE_DEFAULT should map to a proper colorspace.
 * This depends on whether this is a SDTV image (use SMPTE 170M), an
 * HDTV image (use Rec. 709), or something else (use sRGB).
 */

#define V4L2_MAP_COLORSPACE_DEFAULT(is_sdtv, is_hdtv) \
        ((is_sdtv) ? V4L2_COLORSPACE_SMPTE170M : \
        ((is_hdtv) ? V4L2_COLORSPACE_REC709 : V4L2_COLORSPACE_SRGB))

enum v4l2_xfer_func
{
/* Mapping of V4L2_XFER_FUNC_DEFAULT to actual transfer functions
 * for the various colorspaces:
 *
 * V4L2_COLORSPACE_SMPTE170M, V4L2_COLORSPACE_470_SYSTEM_M,
 * V4L2_COLORSPACE_470_SYSTEM_BG, V4L2_COLORSPACE_REC709 and
 * V4L2_COLORSPACE_BT2020: V4L2_XFER_FUNC_709
 *
 * V4L2_COLORSPACE_SRGB, V4L2_COLORSPACE_JPEG: V4L2_XFER_FUNC_SRGB
 *
 * V4L2_COLORSPACE_OPRGB: V4L2_XFER_FUNC_OPRGB
 *
 * V4L2_COLORSPACE_SMPTE240M: V4L2_XFER_FUNC_SMPTE240M
 *
 * V4L2_COLORSPACE_RAW: V4L2_XFER_FUNC_NONE
 *
 * V4L2_COLORSPACE_DCI_P3: V4L2_XFER_FUNC_DCI_P3
 */

  V4L2_XFER_FUNC_DEFAULT     = 0,
  V4L2_XFER_FUNC_709         = 1,
  V4L2_XFER_FUNC_SRGB        = 2,
  V4L2_XFER_FUNC_OPRGB       = 3,
  V4L2_XFER_FUNC_SMPTE240M   = 4,
  V4L2_XFER_FUNC_NONE        = 5,
  V4L2_XFER_FUNC_DCI_P3      = 6,
  V4L2_XFER_FUNC_SMPTE2084   = 7,
};

enum v4l2_ycbcr_encoding
{
/* Mapping of V4L2_YCBCR_ENC_DEFAULT to actual encodings for the
 * various colorspaces:
 *
 * V4L2_COLORSPACE_SMPTE170M, V4L2_COLORSPACE_470_SYSTEM_M,
 * V4L2_COLORSPACE_470_SYSTEM_BG, V4L2_COLORSPACE_SRGB,
 * V4L2_COLORSPACE_OPRGB and V4L2_COLORSPACE_JPEG: V4L2_YCBCR_ENC_601
 *
 * V4L2_COLORSPACE_REC709 and V4L2_COLORSPACE_DCI_P3: V4L2_YCBCR_ENC_709
 *
 * V4L2_COLORSPACE_BT2020: V4L2_YCBCR_ENC_BT2020
 *
 * V4L2_COLORSPACE_SMPTE240M: V4L2_YCBCR_ENC_SMPTE240M
 */

  V4L2_YCBCR_ENC_DEFAULT        = 0,

  /* ITU-R 601 -- SDTV */

  V4L2_YCBCR_ENC_601            = 1,

  /* Rec. 709 -- HDTV */

  V4L2_YCBCR_ENC_709            = 2,

  /* ITU-R 601/EN 61966-2-4 Extended Gamut -- SDTV */

  V4L2_YCBCR_ENC_XV601          = 3,

  /* Rec. 709/EN 61966-2-4 Extended Gamut -- HDTV */

  V4L2_YCBCR_ENC_XV709          = 4,

/* sYCC (Y'CbCr encoding of sRGB), identical to ENC_601. It was added
 * originally due to a misunderstanding of the sYCC standard. It should
 * not be used, instead use V4L2_YCBCR_ENC_601.
 */

  V4L2_YCBCR_ENC_SYCC           = 5,

  /* BT.2020 Non-constant Luminance Y'CbCr */

  V4L2_YCBCR_ENC_BT2020         = 6,

  /* BT.2020 Constant Luminance Y'CbcCrc */

  V4L2_YCBCR_ENC_BT2020_CONST_LUM = 7,

  /* SMPTE 240M -- Obsolete HDTV */

  V4L2_YCBCR_ENC_SMPTE240M      = 8,
};

enum v4l2_quantization
{
/* The default for R'G'B' quantization is always full range.
 * For Y'CbCr the quantization is always limited range, except
 * for COLORSPACE_JPEG: this is full range.
 */

  V4L2_QUANTIZATION_DEFAULT     = 0,
  V4L2_QUANTIZATION_FULL_RANGE  = 1,
  V4L2_QUANTIZATION_LIM_RANGE   = 2,
};

/* Field order. Currently, support only V4L2_FIELD_ANY */

enum v4l2_field
{
  V4L2_FIELD_ANY           = 0, /* Driver can choose from none, */
  V4L2_FIELD_NONE          = 1, /* This device has no fields ... */
  V4L2_FIELD_TOP           = 2, /* Top field only */
  V4L2_FIELD_BOTTOM        = 3, /* Bottom field only */
  V4L2_FIELD_INTERLACED    = 4, /* Both fields interlaced */
  V4L2_FIELD_SEQ_TB        = 5, /* Both fields sequential into one */
  V4L2_FIELD_SEQ_BT        = 6, /* Same as above + bottom-top order */
  V4L2_FIELD_ALTERNATE     = 7, /* Both fields alternating into */
  V4L2_FIELD_INTERLACED_TB = 8, /* Both fields interlaced, top field */
  V4L2_FIELD_INTERLACED_BT = 9, /* Both fields interlaced, top field */
};

/* Buffer mode */

enum v4l2_buf_mode
{
  V4L2_BUF_MODE_RING = 0,  /* Ring structure */
  V4L2_BUF_MODE_FIFO = 1,  /* FIFO structure */
};

struct v4l2_requestbuffers
{
  uint32_t count;    /* The number of buffers requested.
                      * Supported maximum is
                      * is V4L2_REQBUFS_COUNT_MAX
                      */
  uint32_t type;     /* enum #v4l2_buf_type */
  uint32_t memory;   /* enum #v4l2_memory */
  uint32_t mode;     /* enum #v4l2_buf_mode */
};

typedef struct v4l2_requestbuffers v4l2_requestbuffers_t;

struct v4l2_timecode
{
  uint32_t type;
  uint32_t flags;
  uint8_t  frames;
  uint8_t  seconds;
  uint8_t  minutes;
  uint8_t  hours;
  uint8_t  userbits[4];
};

typedef struct v4l2_timecode v4l2_timecode_t;

struct v4l2_plane
{
  uint32_t        bytesused;
  uint32_t        length;
  union
  {
    uint32_t      mem_offset;
    unsigned long userptr;
    int           fd;
  } m;
  uint32_t        data_offset;
  uint32_t        reserved[11];
};

typedef struct v4l2_plane v4l2_plane_t;

/* struct v4l2_buffer
 * Parameter of ioctl(VIDIOC_QBUF) and ioctl(VIDIOC_DQBUF).
 * Currently, support only index, type, bytesused, memory,
 * m.userptr, and length.
 */

struct v4l2_buffer
{
  uint16_t             index;     /* Buffer id */
  uint16_t             type;      /* enum #v4l2_buf_type */
  uint32_t             bytesused; /* Driver sets the image size */
  uint16_t             flags;     /* Buffer flags. */
  uint16_t             field;     /* The field order of the image */
  struct timeval       timestamp; /* Frame timestamp */
  struct v4l2_timecode timecode;  /* Frame timecode */
  uint16_t             sequence;  /* Frame sequence number */
  uint16_t             memory;    /* enum #v4l2_memory */
  union
  {
    uint32_t           offset;
    unsigned long      userptr;   /* Address of buffer */
    struct v4l2_plane  *planes;
    int                fd;
  } m;
  uint32_t             length;    /* User set the buffer size */
};

typedef struct v4l2_buffer v4l2_buffer_t;

/* Image is a keyframe (I-frame) */

#define V4L2_BUF_FLAG_KEYFRAME                  0x00000008

/* mem2mem encoder/decoder */

#define V4L2_BUF_FLAG_LAST                      0x00100000

struct v4l2_fmtdesc
{
  uint16_t index;                           /* Format number      */
  uint16_t type;                            /* enum v4l2_buf_type */
  uint32_t flags;
  char     description[V4L2_FMT_DSC_MAX];   /* Description string */
  uint32_t pixelformat;                     /* Format fourcc      */
};

enum v4l2_fmt_flag
{
  V4L2_FMT_FLAG_COMPRESSED = 0x0001, /* This is a compressed format */
  V4L2_FMT_FLAG_EMULATED   = 0x0002, /* This format is not native */
};

enum v4l2_frmsizetypes
{
  V4L2_FRMSIZE_TYPE_DISCRETE      = 1,   /* Discrete value   */
  V4L2_FRMSIZE_TYPE_CONTINUOUS    = 2,   /* Continuous value */
  V4L2_FRMSIZE_TYPE_STEPWISE      = 3,   /* Step value       */
};

struct v4l2_frmsize_discrete
{
  uint16_t width;          /* Frame width [pixel] */
  uint16_t height;         /* Frame height [pixel] */
};

struct v4l2_frmsize_stepwise
{
  uint16_t min_width;      /* Minimum frame width [pixel] */
  uint16_t max_width;      /* Maximum frame width [pixel] */
  uint16_t step_width;     /* Frame width step size [pixel] */
  uint16_t min_height;     /* Minimum frame height [pixel] */
  uint16_t max_height;     /* Maximum frame height [pixel] */
  uint16_t step_height;    /* Frame height step size [pixel] */
};

struct v4l2_frmsizeenum
{
  uint32_t  index;              /* Frame size number */
  uint32_t  buf_type;           /* enum #v4l2_buf_type */
  uint32_t  pixel_format;       /* Pixel format */
  uint32_t  type;               /* Frame size type the device supports. */

  /* In type == V4L2_FRMSIZE_TYPE_DISCRETE case, use discrete.
   * Otherwise, use stepwise.
   */

  union
  {
    struct v4l2_frmsize_discrete discrete;
    struct v4l2_frmsize_stepwise stepwise;
  };
};

/* Type of frame interval enumeration */

enum v4l2_frmivaltypes
{
  V4L2_FRMIVAL_TYPE_DISCRETE      = 1,   /* Discrete value */
  V4L2_FRMIVAL_TYPE_CONTINUOUS    = 2,   /* Continuous value */
  V4L2_FRMIVAL_TYPE_STEPWISE      = 3,   /* Step value */
};

/* Frame interval enumeration with stepwise format */

struct v4l2_frmival_stepwise
{
  struct v4l2_fract       min;            /* Minimum frame interval [s] */
  struct v4l2_fract       max;            /* Maximum frame interval [s] */
  struct v4l2_fract       step;           /* Frame interval step size [s] */
};

struct v4l2_frmivalenum
{
  uint32_t index;               /* Frame format index */
  uint32_t buf_type;            /* enum #v4l2_buf_type */
  uint32_t pixel_format;        /* Pixel format */
  uint16_t width;               /* Frame width */
  uint16_t height;              /* Frame height */
  uint32_t type;                /* Frame interval type */
  union
  {                             /* Frame interval */
    struct v4l2_fract               discrete;
    struct v4l2_frmival_stepwise    stepwise;
  };
};

/* Single-planar format structure. */

struct v4l2_pix_format
{
  uint16_t  width;              /* Image width in pixels */
  uint16_t  height;             /* Image height in pixels */
  uint32_t  pixelformat;        /* The pixel format or type of compression. */
  uint32_t  field;              /* enum #v4l2_field */
  uint32_t  bytesperline;       /* For padding, zero if unused */
  uint32_t  sizeimage;          /* Size in bytes of the buffer to hold a complete image */
  uint32_t  colorspace;         /* Image colorspace */
  uint32_t  priv;               /* Private data, depends on pixelformat */
  uint32_t  flags;              /* Format flags (V4L2_PIX_FMT_FLAG_*) */
  union
  {
    uint32_t ycbcr_enc;         /* enum v4l2_ycbcr_encoding */
    uint32_t hsv_enc;           /* enum v4l2_hsv_encoding */
  };
  uint32_t  quantization;       /* enum v4l2_quantization */
  uint32_t  xfer_func;          /* enum v4l2_xfer_func */
};

typedef struct v4l2_pix_format v4l2_pix_format_t;

/* struct v4l2_plane_pix_format - additional, per-plane format definition
 * sizeimage: maximum size in bytes required for data, for which
 * this plane will be used
 * bytesperline: distance in bytes between the leftmost pixels in two
 * adjacent lines
 */

struct v4l2_plane_pix_format
{
  uint32_t  sizeimage;
  uint32_t  bytesperline;
  uint16_t  reserved[6];
};

/* struct v4l2_pix_format_mplane - multiplanar format definition
 * width: image width in pixels
 * height: image height in pixels
 * pixelformat: little endian four character code (fourcc)
 * field: enum v4l2_field; field order (for interlaced video)
 * colorspace: enum v4l2_colorspace; supplemental to pixelformat
 * plane_fmt: per-plane information
 * num_planes: number of planes for this format
 * flags: format flags (V4L2_PIX_FMT_FLAG_*)
 * ycbcr_enc: enum v4l2_ycbcr_encoding, Y'CbCr encoding
 * quantization: enum v4l2_quantization, colorspace quantization
 * xfer_func: enum v4l2_xfer_func, colorspace transfer function
 */

struct v4l2_pix_format_mplane
{
  uint32_t      width;
  uint32_t      height;
  uint32_t      pixelformat;
  uint32_t      field;
  uint32_t      colorspace;

  struct v4l2_plane_pix_format plane_fmt[VIDEO_MAX_PLANES];
  uint8_t                      num_planes;
  uint8_t                      flags;
  union
  {
    uint8_t                    ycbcr_enc;
    uint8_t                    hsv_enc;
  };
  uint8_t                      quantization;
  uint8_t                      xfer_func;
  uint8_t                      reserved[7];
};

struct v4l2_format
{
  uint32_t  type;                         /* enum #v4l2_buf_type. */
  union
  {
    struct v4l2_pix_format        pix;    /* Image format */
    struct v4l2_pix_format_mplane pix_mp; /* V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE */
  } fmt;
};

typedef struct v4l2_format v4l2_format_t;

struct v4l2_captureparm
{
  uint32_t           capability;    /*  Supported modes */
  uint32_t           capturemode;   /*  Current mode */
  struct v4l2_fract  timeperframe;  /*  Time per frame in seconds */
  uint32_t           extendedmode;  /*  Driver-specific extensions */
  uint32_t           readbuffers;   /*  # of buffers for read */
};

struct v4l2_outputparm
{
  uint32_t          capability;      /*  Supported modes */
  uint32_t          outputmode;      /*  Current mode */
  struct v4l2_fract timeperframe;    /*  Time per frame in seconds */
  uint32_t          extendedmode;    /*  Driver-specific extensions */
  uint32_t          writebuffers;    /*  # of buffers for write */
  uint32_t          reserved[4];
};

struct v4l2_cropcap
{
  uint32_t                type; /* enum v4l2_buf_type */
  struct v4l2_rect        bounds;
  struct v4l2_rect        defrect;
  struct v4l2_fract       pixelaspect;
};

/*  Flags for 'capability' and 'capturemode' fields */

enum v4l2_capture_mode
{
  V4L2_MODE_HIGHQUALITY = 0x0001,    /*  High quality imaging mode */
};

enum v4l2_capture_capability
{
  V4L2_CAP_TIMEPERFRAME = 0x1000,    /*  timeperframe field is supported */
};

struct v4l2_streamparm
{
  uint32_t    type;                  /* enum v4l2_buf_type */
  union
  {
    struct v4l2_captureparm capture;
    struct v4l2_outputparm  output;
  } parm;
  uint8_t  raw_data[200];            /* user-defined */
};

/* E V E N T S */

#define V4L2_EVENT_ALL                          0
#define V4L2_EVENT_VSYNC                        1
#define V4L2_EVENT_EOS                          2
#define V4L2_EVENT_CTRL                         3
#define V4L2_EVENT_FRAME_SYNC                   4
#define V4L2_EVENT_SOURCE_CHANGE                5
#define V4L2_EVENT_MOTION_DET                   6
#define V4L2_EVENT_PRIVATE_START                0x08000000

/* Payload for V4L2_EVENT_VSYNC */

struct v4l2_event_vsync
{
  /* Can be V4L2_FIELD_ANY, _NONE, _TOP or _BOTTOM */

  uint8_t field;
};

/* Payload for V4L2_EVENT_CTRL */

#define V4L2_EVENT_CTRL_CH_VALUE                (1 << 0)
#define V4L2_EVENT_CTRL_CH_FLAGS                (1 << 1)
#define V4L2_EVENT_CTRL_CH_RANGE                (1 << 2)

struct v4l2_event_ctrl
{
  uint32_t changes;
  uint32_t type;
  union
  {
    int32_t value;
    int64_t value64;
  };
  uint32_t flags;
  int32_t minimum;
  int32_t maximum;
  int32_t step;
  int32_t default_value;
};

struct v4l2_event_frame_sync
{
  uint32_t frame_sequence;
};

#define V4L2_EVENT_SRC_CH_RESOLUTION        (1 << 0)

struct v4l2_event_src_change
{
  uint32_t changes;
};

#define V4L2_EVENT_MD_FL_HAVE_FRAME_SEQ     (1 << 0)

/* struct v4l2_event_motion_det - motion detection event
 * flags:             if V4L2_EVENT_MD_FL_HAVE_FRAME_SEQ is set, then the
 *                    frame_sequence field is valid.
 * frame_sequence:    the frame sequence number associated with this event.
 * region_mask:       which regions detected motion.
 */

struct v4l2_event_motion_det
{
  uint32_t flags;
  uint32_t frame_sequence;
  uint32_t region_mask;
};

struct v4l2_event
{
  uint32_t        type;
  union
  {
    struct v4l2_event_vsync       vsync;
    struct v4l2_event_ctrl        ctrl;
    struct v4l2_event_frame_sync  frame_sync;
    struct v4l2_event_src_change  src_change;
    struct v4l2_event_motion_det  motion_det;
    uint8_t                       data[64];
  } u;
  uint32_t        pending;
  uint32_t        sequence;
  struct timespec timestamp;
  uint32_t        id;
  uint32_t        reserved[8];
};

#define V4L2_EVENT_SUB_FL_SEND_INITIAL   (1 << 0)
#define V4L2_EVENT_SUB_FL_ALLOW_FEEDBACK (1 << 1)

struct v4l2_event_subscription
{
  uint32_t       type;
  uint32_t       id;
  uint32_t       flags;
  uint32_t       reserved[5];
};

enum v4l2_ctrl_type
{
  V4L2_CTRL_TYPE_INTEGER          = 1,
  V4L2_CTRL_TYPE_BOOLEAN          = 2,
  V4L2_CTRL_TYPE_MENU             = 3,
  V4L2_CTRL_TYPE_BUTTON           = 4,
  V4L2_CTRL_TYPE_INTEGER64        = 5,
  V4L2_CTRL_TYPE_CTRL_CLASS       = 6,
  V4L2_CTRL_TYPE_STRING           = 7,
  V4L2_CTRL_TYPE_BITMASK          = 8,
  V4L2_CTRL_TYPE_INTEGER_MENU     = 9,
  V4L2_CTRL_TYPE_U8FIXEDPOINT_Q7  = 10,
  V4L2_CTRL_TYPE_U16FIXEDPOINT_Q8 = 11,
  V4L2_CTRL_TYPE_INTEGER_TIMES_3  = 12,

  /* Compound types are >= 0x0100 */

  V4L2_CTRL_COMPOUND_TYPES       = 0x0100,
  V4L2_CTRL_TYPE_U8              = 0x0100,
  V4L2_CTRL_TYPE_U16             = 0x0101,
  V4L2_CTRL_TYPE_U32             = 0x0102,
};

struct v4l2_queryctrl
{
  uint16_t   ctrl_class;               /* Control class(not used) */
  uint16_t   id;                       /* Control id */
  uint16_t   type;                     /* enum #v4l2_ctrl_type */
  char       name[32];                 /* Name of control */
  int32_t    minimum;                  /* Minimum value */
  int32_t    maximum;                  /* Maximum value */
  uint32_t   step;                     /* Step */
  int32_t    default_value;            /* Default value */
  uint32_t   flags;                    /* Flag */};

struct v4l2_query_ext_ctrl
{
  uint16_t   ctrl_class;               /* Control class(not used) */
  uint16_t   id;                       /* Control id */
  uint16_t   type;                     /* enum #v4l2_ctrl_type */
  char       name[32];                 /* Name of control */
  int64_t    minimum;                  /* Minimum value */
  int64_t    maximum;                  /* Maximum value */
  uint64_t   step;                     /* Step */
  int64_t    default_value;            /* Default value */
  uint32_t   flags;                    /* Flag */
  uint32_t   elem_size;                /* Size of each element */
  uint32_t   elems;                    /* Number of elements */
  uint32_t   nr_of_dims;               /* Number of dimensions */
  uint32_t   dims[V4L2_CTRL_MAX_DIMS]; /* Dimensions */
};

struct v4l2_querymenu
{
  uint16_t   ctrl_class;    /* Camera control class(not used) */
  uint16_t   id;            /* Camera control id    */
  uint32_t   index;         /* Index of menu.       */
  union
  {
    char    name[32];       /* Name of menu  */
    int64_t value;          /* Value of menu */
  };
};

struct v4l2_input
{
  uint32_t      index;       /*  Which input */
  uint8_t       name[32];    /*  Label */
  uint32_t      type;        /*  Type of input */
  uint32_t      audioset;    /*  Associated audios (bitfield) */
  uint32_t      tuner;       /*  enum v4l2_tuner_type */
  v4l2_std_id   std;
  uint32_t      status;
  uint32_t      capabilities;
  uint32_t      reserved[3];
};

/*  Values for the 'type' field */

enum v4l2_input_type
{
  V4L2_INPUT_TYPE_TUNER  = 1,
  V4L2_INPUT_TYPE_CAMERA = 2,
  V4L2_INPUT_TYPE_TOUCH  = 3,
};

enum v4l2_input_status
{
  /* Field 'status' - general */

  V4L2_IN_ST_NO_POWER    = 0x00000001,  /* Attached device is off */
  V4L2_IN_ST_NO_SIGNAL   = 0x00000002,
  V4L2_IN_ST_NO_COLOR    = 0x00000004,

  /* Field 'status' - sensor orientation */

  /* If sensor is mounted upside down set both bits */

  V4L2_IN_ST_HFLIP       = 0x00000010,  /* Frames are flipped horizontally */
  V4L2_IN_ST_VFLIP       = 0x00000020,  /* Frames are flipped vertically */

  /* Field 'status' - analog */

  V4L2_IN_ST_NO_H_LOCK   = 0x00000100,  /* No horizontal sync lock */
  V4L2_IN_ST_COLOR_KILL  = 0x00000200,  /* Color killer is active */
  V4L2_IN_ST_NO_V_LOCK   = 0x00000400,  /* No vertical sync lock */
  V4L2_IN_ST_NO_STD_LOCK = 0x00000800,  /* No standard format lock */

  /* Field 'status' - digital */

  V4L2_IN_ST_NO_SYNC     = 0x00010000,  /* No synchronization lock */
  V4L2_IN_ST_NO_EQU      = 0x00020000,  /* No equalizer lock */
  V4L2_IN_ST_NO_CARRIER  = 0x00040000,  /* Carrier recovery failed */

  /* Field 'status' - VCR and set-top box */

  V4L2_IN_ST_MACROVISION = 0x01000000,  /* Macrovision detected */
  V4L2_IN_ST_NO_ACCESS   = 0x02000000,  /* Conditional access denied */
  V4L2_IN_ST_VTR         = 0x04000000,  /* VTR time constant */
};

/* Capabilities flags */

enum v4l2_input_capabilities
{
  V4L2_IN_CAP_DV_TIMINGS  = 0x00000002, /* Supports S_DV_TIMINGS */
  V4L2_IN_CAP_STD         = 0x00000004, /* Supports S_STD */
  V4L2_IN_CAP_NATIVE_SIZE = 0x00000008, /* Supports setting native size */
};

enum v4l2_input_capabilites_compat
{
  V4L2_IN_CAP_CUSTOM_TIMINGS = V4L2_IN_CAP_DV_TIMINGS,  /* For compatibility */
};

struct v4l2_output
{
  uint32_t      index;           /*  Which output */
  uint8_t       name[32];        /*  Label */
  uint32_t      type;            /*  Type of output */
  uint32_t      audioset;        /*  Associated audios (bitfield) */
  uint32_t      modulator;       /*  Associated modulator */
  v4l2_std_id   std;
  uint32_t      capabilities;
  uint32_t      reserved[3];
};

/*  Values for the 'type' field */

enum v4l2_output_type
{
  V4L2_OUTPUT_TYPE_MODULATOR        = 1,
  V4L2_OUTPUT_TYPE_ANALOG           = 2,
  V4L2_OUTPUT_TYPE_ANALOGVGAOVERLAY = 3,
};

/* Capabilities flags */

enum v4l2_output_capabilities
{
  V4L2_OUT_CAP_DV_TIMINGS  = 0x00000002, /* Supports S_DV_TIMINGS */
  V4L2_OUT_CAP_STD         = 0x00000004, /* Supports S_STD */
  V4L2_OUT_CAP_NATIVE_SIZE = 0x00000008, /* Supports setting native size */
};

enum v4l2_output_capabilites_compat
{
  V4L2_OUT_CAP_CUSTOM_TIMINGS = V4L2_OUT_CAP_DV_TIMINGS, /* For compatibility */
};

struct v4l2_control
{
  uint16_t id;
  int32_t  value;
};

/* Structure for each control of
 *  ioctl(VIDIOC_G_EXT_CTRLS / VIDIOC_S_EXT_CTRLS)
 */

struct v4l2_ext_control
{
  uint16_t   id;       /* Camera control id */
  uint16_t   size;     /* Size of value(not use) */
  union
  {
    int32_t  value;    /* QUERY_EXT_CTRL type = INTEGER, xxx */
    int64_t  value64;  /* QUERY_EXT_CTRL type = INTEGER64, MENU */
    char     *string;
    uint8_t  *p_u8;    /* QUERY_EXT_CTRL type = U8  */
    uint16_t *p_u16;   /* QUERY_EXT_CTRL type = U16 */
    uint32_t *p_u32;   /* QUERY_EXT_CTRL type = U32 */
    void     *ptr;
  };
};

struct v4l2_ext_controls
{
  union
  {
    uint16_t              ctrl_class;  /* Camera control class         */
    uint16_t              which;
  };
  uint16_t                count;       /* Number of requests           */
  uint16_t                error_idx;   /* Index in that error occurred */
  struct v4l2_ext_control *controls;   /* Each control information     */
};

/* Structure for V4SIOC_S_EXT_CTRLS and V4SIOC_G_EXT_CTRLS */

struct v4s_ext_controls_scene
{
  enum v4l2_scene_mode     mode;       /* Scene mode to be controled */
  struct v4l2_ext_controls control;    /* Same as VIDIOC_S_EXT_CTRLS */
};

/* Structure for V4SIOC_QUERY_EXT_CTRL */

struct v4s_query_ext_ctrl_scene
{
  enum v4l2_scene_mode       mode;     /* Scene mode to be queried */
  struct v4l2_query_ext_ctrl control;  /* Same as VIDIOC_QUERY_EXT_CTRL */
};

/* Structure for V4SIOC_QUERYMENU */

struct v4s_querymenu_scene
{
  enum v4l2_scene_mode       mode;     /* Scene mode to be queried */
  struct v4l2_querymenu      menu;     /* Same as VIDIOC_QUERYMENU */
};

#define V4L2_ENC_CMD_START      (0)
#define V4L2_ENC_CMD_STOP       (1)
#define V4L2_ENC_CMD_PAUSE      (2)
#define V4L2_ENC_CMD_RESUME     (3)

/* Flags for V4L2_ENC_CMD_STOP */

#define V4L2_ENC_CMD_STOP_AT_GOP_END (1 << 0)

struct v4l2_encoder_cmd
{
  uint32_t cmd;
  uint32_t flags;
  union
  {
    struct
    {
      uint32_t data[8];
    } raw;
  };
};

/* Decoder commands */

#define V4L2_DEC_CMD_START       (0)
#define V4L2_DEC_CMD_STOP        (1)
#define V4L2_DEC_CMD_PAUSE       (2)
#define V4L2_DEC_CMD_RESUME      (3)

/* The structure must be zeroed before use by the application
 *  This ensures it can be extended safely in the future.
 */

struct v4l2_decoder_cmd
{
  uint32_t cmd;
  uint32_t flags;
  union
  {
    struct
    {
      uint64_t pts;
    } stop;

    struct
    {
/* 0 or 1000 specifies normal speed,
 * 1 specifies forward single stepping,
 * -1 specifies backward single stepping,
 * >1: playback at speed/1000 of the normal speed,
 * <-1: reverse playback at (-speed/1000) of the normal speed.
 */

      int32_t speed;
      uint32_t format;
    } start;

    struct
    {
      uint32_t data[16];
    } raw;
  };
};

/* Query device capability
 * Address pointing to struct v4l2_capability
 */

#define VIDIOC_QUERYCAP               _VIDIOC(0x0000)

/* Enumerate the formats supported by device */

#define VIDIOC_ENUM_FMT               _VIDIOC(0x0002)

/* Get the data format */

#define VIDIOC_G_FMT                  _VIDIOC(0x0004)

/* Set the data format */

#define VIDIOC_S_FMT                  _VIDIOC(0x0005)

/* Initiate user pointer I/O */

#define VIDIOC_REQBUFS                _VIDIOC(0x0008)

/* Query the status of a buffer */

#define VIDIOC_QUERYBUF               _VIDIOC(0x0009)

/* Get frame buffer overlay parameters */

#define VIDIOC_G_FBUF                 _VIDIOC(0x000a)

/* Set frame buffer overlay parameters */

#define VIDIOC_S_FBUF                 _VIDIOC(0x000b)

/* Start or stop video overlay */

#define VIDIOC_OVERLAY                _VIDIOC(0x000e)

/* Enqueue an empty buffer */

#define VIDIOC_QBUF                   _VIDIOC(0x000f)

/* Export a buffer as a DMABUF file descriptor */

#define VIDIOC_EXPBUF                 _VIDIOC(0x0010)

/* Dequeue a filled buffer */

#define VIDIOC_DQBUF                  _VIDIOC(0x0011)

/* Start streaming */

#define VIDIOC_STREAMON               _VIDIOC(0x0012)

/* Stop streaming */

#define VIDIOC_STREAMOFF              _VIDIOC(0x0013)

/* Get streaming parameters */

#define VIDIOC_G_PARM                 _VIDIOC(0x0015)

/* Set streaming parameters */

#define VIDIOC_S_PARM                 _VIDIOC(0x0016)

/* Query the video standard of the current input */

#define VIDIOC_G_STD                  _VIDIOC(0x0017)

/* Select the video standard of the current input */

#define VIDIOC_S_STD                  _VIDIOC(0x0018)

/* Enumerate supported video standards */

#define VIDIOC_ENUMSTD                _VIDIOC(0x0019)

/* Enumerate video inputs */

#define VIDIOC_ENUMINPUT              _VIDIOC(0x001a)

/* Get current control value.
 *  This request is a special case of VIDIOC_G_EXT_CTRLS.
 *  Address pointing to struct #v4l2_control
 */

#define VIDIOC_G_CTRL                 _VIDIOC(0x001b)

/* Set control value.
 *  This request is a special case of VIDIOC_S_EXT_CTRLS.
 *  Address pointing to struct #v4l2_control
 */

#define VIDIOC_S_CTRL                 _VIDIOC(0x001c)

/* Query control */

#define VIDIOC_QUERYCTRL              _VIDIOC(0x0024)

/* Query menu */

#define VIDIOC_QUERYMENU              _VIDIOC(0x0025)

/* Get video input */

#define VIDIOC_G_INPUT                _VIDIOC(0x0026)

/* Set video input */

#define VIDIOC_S_INPUT                _VIDIOC(0x0027)

/* Set video crop and scale */

#define VIDIOC_CROPCAP                _VIDIOC(0x003a)

/* Query video standard */

#define VIDIOC_QUERYSTD               _VIDIOC(0x003f)

/* Try format */

#define VIDIOC_TRY_FMT                _VIDIOC(0x0040)

/* Get current control value
 *  Address pointing to struct #v4l2_ext_controls
 */

#define VIDIOC_G_EXT_CTRLS            _VIDIOC(0x0047)

/* Set control value
 *  Address pointing to struct #v4l2_ext_controls
 */

#define VIDIOC_S_EXT_CTRLS            _VIDIOC(0x0048)

/* Try control value
 *  Address pointing to struct #v4l2_ext_controls
 */

#define VIDIOC_TRY_EXT_CTRLS          _VIDIOC(0x0049)

/* Enumerate the framesizes supported by device */

#define VIDIOC_ENUM_FRAMESIZES        _VIDIOC(0x004a)

/* Enumerate the frameintervals supported by device */

#define VIDIOC_ENUM_FRAMEINTERVALS    _VIDIOC(0x004b)

/* Execute an encoder command */

#define VIDIOC_ENCODER_CMD            _VIDIOC(0x004d)

/* Dequeue event */

#define VIDIOC_DQEVENT                _VIDIOC(0x0059)

/* Subscribe or unsubscribe event */

#define VIDIOC_SUBSCRIBE_EVENT        _VIDIOC(0x005a)

/* Get clip
 * Address pointing to struct v4l2_selection
 */

#define VIDIOC_G_SELECTION            _VIDIOC(0x005e)

/* Set clip
 * Address pointing to struct v4l2_selection
 */

#define VIDIOC_S_SELECTION            _VIDIOC(0x005f)

/* Execute an decoder command */

#define VIDIOC_DECODER_CMD            _VIDIOC(0x0060)

/* Query control */

#define VIDIOC_QUERY_EXT_CTRL         _VIDIOC(0x00c0)

/* Cancel DQBUF
 *  enum #v4l2_buf_type
 */

#define VIDIOC_CANCEL_DQBUF           _VIDIOC(0x00c1)

/* Do halfpush */

#define VIDIOC_DO_HALFPUSH            _VIDIOC(0x00c2)

/* Start taking picture
 *
 * Type is int32_t, not address pointer.\n
 * 0 or negative value means continuing until VIDIOC_TAKEPICT_STOP. \n
 * Positive value(to be supported) means continuing
 * up to a specified number of times  or until VIDIOC_TAKEPICT_STOP.
 */

#define VIDIOC_TAKEPICT_START         _VIDIOC(0x00c3)

/* Stop taking picture */

#define VIDIOC_TAKEPICT_STOP          _VIDIOC(0x00c4)

/* Query control for scene parameter
 *  Address pointing to struct v4s_query_ext_ctrl_scene
 */

#define V4SIOC_QUERY_EXT_CTRL_SCENE   _VIDIOC(0x00c5)

/* Query menu for scene parameter
 *  Address pointing to struct v4s_querymenu_scene
 */

#define V4SIOC_QUERYMENU_SCENE        _VIDIOC(0x00c6)

/* Get current control value
 *  Address pointing to struct v4s_ext_controls_scene
 */

#define V4SIOC_G_EXT_CTRLS_SCENE      _VIDIOC(0x00c7)

/* Set control value
 *  Address pointing to struct v4s_ext_controls_scene
 */

#define V4SIOC_S_EXT_CTRLS_SCENE      _VIDIOC(0x00c8)

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SYS_VIDEOIO_H */
