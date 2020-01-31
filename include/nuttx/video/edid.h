/********************************************************************************************
 * include/nuttx/video/edid.h
 * EDID (Extended Display Identification Data) Format
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:  Wikipedia (initial version)
 *
 * Updated and extended with definitions from FreeBSD which has a compatible 2-clause BSD
 * license:
 *
 *  Copyright (c) 2006 Itronix Inc. All rights reserved.
 *  Written by Garrett D'Amore for Itronix Inc.
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_VIDEO_EDID_H
#define __INCLUDE_NUTTX_VIDEO_EDID_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <stdint.h>
#include <nuttx/video/videomode.h>

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define EDID_LENGTH                   128

/* EDID data offsets ************************************************************************/
/* Bytes 0-7:  Header Information */

#define EDID_HEADER_MAGIC_OFFSET        0     /* Fixed header pattern: 00 FF FF FF FF FF FF 00 */
#define EDID_HEADER_MAGIC_SIZE          8

/* Bytes 8-17:  Vendor Information */

#define EDID_VENDOR_MANUFACTURER_OFFSET 8     /* Encoded 3-character manufacture ID */
#define EDID_VENDOR_MANUFACTURER_SIZE   2     /* 16-bit, big endian value */

#define EDID_VENDOR_PRODUCTCODE_OFFSET  10    /* Product code.  16-bit, little endian value */
#define EDID_VENDOR_PRODUCTCODE_SIZE    2

#define EDID_VENDOR_SERIALNO_OFFSET     12    /* Serial number:  32-bit, little endian value */
#define EDID_VENDOR_SERIALNO_SIZE       4

#define EDID_VENDOR_WEEK_OFFSET         16    /* Week of manufacture or model year flag */
#define EDID_VENDOR_YEAR_OFFSET         17    /* Year of manufacture (minus 1990) */

/* Bytes 18-19:  EDID Version */

#define EDID_VERSION_MAJOR_OFFSET       18    /* EDID version, usually 1 (for 1.3)  */
#define EDID_VERSION_MINOR_OFFSET       19    /* EDID revision, usually 3 (for 1.3)  */

/* Bytes 20-44:  Display Information */

#define EDID_DISPLAY_INPUT_OFFSET       20   /* Video input parameters bitmap */
#define EDID_DISPLAY_HSIZE_OFFSET       21   /* Horizontal screen size, in centimetres */
#define EDID_DISPLAY_VSIZE_OFFSET       22   /* Vertical screen size, in centimetres */
#define EDID_DISPLAY_GAMMA_OFFSET       23   /* Display gamma, factory default */
#define EDID_DISPLAY_FEATURES_OFFSET    24   /* Support features bitmap */

/* Bytes 25-34:  Chromaticity */

#define EDID_CHROMA_RG_LOW_OFFSET       25   /* Red and green least-significant bits (2^9, 2^10) */
#define EDID_CHROMA_BW_LOW_OFFSET       26   /* Blue and white least-significant 2 bits */
#define EDID_CHROMA_REDX_OFFSET         27   /* Red x value most significant 8 bits (2^1,...,2^8) */
#define EDID_CHROMA_REDY_OFFSET         28   /* Red y value most significant 8 bits  */
#define EDID_CHROMA_GREENX_OFFSET       29   /* Green x value most significant 8 bits */
#define EDID_CHROMA_GREENY_OFFSET       30   /* Green y value most significant 8 bits */
#define EDID_CHROMA_BLUEX_OFFSET        31   /* Blue x value most significant 8 bits */
#define EDID_CHROMA_BLUEY_OFFSET        32   /* Blue y value most significant 8 bits */
#define EDID_CHROMA_WHITEX_OFFSET       33   /* Default white point x value most significant 8 bits */
#define EDID_CHROMA_WHITEY_OFFSET       34   /* Default white point y value most significant 8 bits */

/* Bytes 35-37:  Established timing bitmap */

#define EDID_TIMING_OFFSET_1            35
#define EDID_TIMING_OFFSET_2            36
#define EDID_TIMING_OFFSET_3            37

/* Bytes 38-53:  Standard Timing Information */

#define EDID_STDTIMING_OFFSET           38   /* Each is size two bytes. */
#define EDID_STDTIMING_OFFSET_1         38
#define EDID_STDTIMING_OFFSET_2         40
#define EDID_STDTIMING_OFFSET_3         42
#define EDID_STDTIMING_OFFSET_4         44
#define EDID_STDTIMING_OFFSET_5         45
#define EDID_STDTIMING_OFFSET_6         48
#define EDID_STDTIMING_OFFSET_7         50
#define EDID_STDTIMING_OFFSET_8         52

#define EDID_STDTIMING_NUMBER           8
#define EDID_STDTIMING_SIZE             2

/* Bytes 54-125:  Descriptor Blocks */

#define EDID_DESCRIPTOR_OFFSET          54   /* Each is size 18 bytes */
#define EDID_DESCRIPTOR_OFFSET_1        54
#define EDID_DESCRIPTOR_OFFSET_2        72
#define EDID_DESCRIPTOR_OFFSET_3        90
#define EDID_DESCRIPTOR_OFFSET_4        108

#define EDID_DESCRIPTOR_NUMBER          4
#define EDID_DESCRIPTOR_SIZE            18

/* Bits 126-127:  Trailer */

#define EDID_TRAILER_NEXTENSIONS_OFFSET 126  /* Number of extensions to follow */
#define EDID_TRAILER_CHECKSUM_OFFSET    127  /* Checksum. Sum of all 128 bytes should equal 0 */

/* EDID Bitfield Definitions ****************************************************************/

#define EDID_MAGIC                        {0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0}

/* Vendor Section: Manufacturer ID */

#define EDID_VENDOR_MANUFACTURER_1(n)     ((((n) >> 10) & 0x1f) + '@')
#define EDID_VENDOR_MANUFACTURER_2(n)     ((((n) >>  5) & 0x1f) + '@')
#define EDID_VENDOR_MANUFACTURER_3(n)     ((((n) >>  0) & 0x1f) + '@')

/* Display Section: Display Input */

#define BIT_DEPTH_6                       1     /* Values for bit depth */
#define BIT_DEPTH_8                       2
#define BIT_DEPTH_10                      3
#define BIT_DEPTH_12                      4
#define BIT_DEPTH_14                      5
#define BIT_DEPTH_16                      6

#define VIDEO_INTF_HDMIA                  1    /* Values for video interface */
#define VIDEO_INTF_HDMIB                  2
#define VIDEO_INTF_MDDI                   4
#define VIDEO_INTF_DISPLAYPORT            5

#define EDID_DISPLAY_INPUT_DIGITAL        (1 << 7)  /* Bit 7: Digital input */

                                                    /* For digital input: */
#define EDID_DISPLAY_INPUT_VIDIF_SHIFT    (0)       /* Bits 0-3: Video interface */
#define EDID_DISPLAY_INPUT_VIDIF_MASK     (15 << EDID_DISPLAY_INPUT_VIDIF_SHIFT)
#  define EDID_DISPLAY_INPUT_DFP1_COMPAT  (1 << EDID_DISPLAY_INPUT_VIDIF_SHIFT)
#define EDID_DISPLAY_INPUT_BITDEPTH_SHIFT (4)       /* Bits 4-6: Bit depth */
#define EDID_DISPLAY_INPUT_BITDEPTH_MASK  (7 << EDID_DISPLAY_INPUT_BITDEPTH_SHIFT)

                                                    /* For analog input: */
#define EDID_DISPLAY_INPUT_VSERRATED      (1 << 0)  /* Bit 0:  VSync pulse must be serrated
                                                     *         when composite or sync-on-green
                                                     *         is used */
#define EDID_DISPLAY_INPUT_GREEN          (1 << 1)  /* Bit 1:  Sync on green supported */
#define EDID_DISPLAY_INPUT_COMPOSITE      (1 << 2)  /* Bit 2:  Composite sync (on HSync)
                                                     *         supported */
#define EDID_DISPLAY_INPUT_SYNC           (1 << 3)  /* Bit 3:  Separate sync supported */
#define EDID_DISPLAY_INPUT_BLANK2BLACK    (1 << 4)  /* Bit 4:  Blank to black setup */
#define EDID_DISPLAY_INPUT_LEVELS_SHIFT   (5)       /* Bits 5-6: Video white and sync levels,
                                                     *           relative to blank */
#define EDID_DISPLAY_INPUT_LEVELS_MASK    (3 << EDID_DISPLAY_INPUT_LEVELS_SHIFT)
#  define EDID_DISPLAY_INPUT_LEVEL_1      (0 << EDID_DISPLAY_INPUT_LEVELS_SHIFT) /* -0.7, 0.3V */
#  define EDID_DISPLAY_INPUT_LEVEL_2      (1 << EDID_DISPLAY_INPUT_LEVELS_SHIFT) /* -0.714, 0.286V */
#  define EDID_DISPLAY_INPUT_LEVEL_3      (2 << EDID_DISPLAY_INPUT_LEVELS_SHIFT) /* -1.0, 0.4V */
#  define EDID_DISPLAY_INPUT_LEVEL_4      (3 << EDID_DISPLAY_INPUT_LEVELS_SHIFT) /* -0.7, 0.0V */

/* Display Section: Supported Features */

#define ANALOG_DISPLAY_TYPE_MONOCHOME     0         /* Monochrome or Grayscale */
#define ANALOG_DISPLAY_TYPE_RGB           1         /* RGB color */
#define ANALOG_DISPLAY_TYPE_NONRGB        2         /* Non-RGB color */

#define DIGITAL_DISPLAY_TYPE_RGB444       0         /* RGB 4:4:4 */
#define DIGITAL_DISPLAY_TYPE_RGBYCRCB_1   1         /* RGB 4:4:4 + YCrCb 4:4:4 */
#define DIGITAL_DISPLAY_TYPE_RGBYCRCB_2   2         /* RGB 4:4:4 + YCrCb 4:2:2 */
#define DIGITAL_DISPLAY_TYPE_RGBYCRCB_3   3         /* RGB 4:4:4 + YCrCb 4:4:4 + YCrCb 4:2:2 */

#define EDID_DISPLAY_FEATURE_CONTINUOUS   (1 << 0)  /* Bit 0:  Continuous timings with GTF
                                                     *         or CVT */
#define EDID_DISPLAY_FEATURE_MODE         (1 << 1)  /* Bit 1:  Preferred timing mode specified
                                                     *         in descriptor block 1 */
#define EDID_DISPLAY_FEATURE_STDRGB       (1 << 2)  /* Bit 2: Standard sRGB colour space */
#define EDID_DISPLAY_FEATURE_ATYPE_SHIFT  (3)       /* Bits 3-4: Display type (analog) */
#define EDID_DISPLAY_FEATURE_ATYPE_MASK   (3 << EDID_DISPLAY_FEATURE_ATYPE_SHIFT)
#define EDID_DISPLAY_FEATURE_DTYPE_SHIFT  (3)       /* Bits 3-4: Display type (digital) */
#define EDID_DISPLAY_FEATURE_DTYPE_MASK   (3 << EDID_DISPLAY_FEATURE_DTYPE_SHIFT)
#  define EDID_ISPLAY_FEATURES_DTYPE_MONO      (0 << EDID_DISPLAY_FEATURE_DTYPE_SHIFT)
#  define EDID_ISPLAY_FEATURES_DTYPE_RGB       (1 << EDID_DISPLAY_FEATURE_DTYPE_SHIFT)
#  define EDID_ISPLAY_FEATURES_DTYPE_NON_RGB   (2 << EDID_DISPLAY_FEATURE_DTYPE_SHIFT)
#  define EDID_ISPLAY_FEATURES_DTYPE_UNDEFINED (3 << EDID_DISPLAY_FEATURE_DTYPE_SHIFT)
#define EDID_DISPLAY_FEATURE_DPMSOFF      (1 << 5)  /* Bit 5: DPMS active-off supported */
#define EDID_DISPLAY_FEATURE_DPMSSUSP     (1 << 6)  /* Bit 6: DPMS suspend supported */
#define EDID_DISPLAY_FEATURE_DPMSSTDBY    (1 << 7)  /* Bit 7: DPMS standby supported  */

/* Chromaticity Section: Red and green least-significant bits */

#define EDID_CHROMA_RG_LOW_GREEN_Y_SHIFT  (0)       /* Bits 0-1:  Green y value least-significant 2 bits */
#define EDID_CHROMA_RG_LOW_GREEN_Y_MASK   (3 << EDID_CHROMA_RG_LOW_GREEN_Y_SHIFT)
#define EDID_CHROMA_RG_LOW_GREEN_X_SHIFT  (2)       /* Bits 2-3:  Green x value least-significant 2 bits */
#define EDID_CHROMA_RG_LOW_GREEN_X_MASK   (3 << EDID_CHROMA_RG_LOW_GREEN_X_SHIFT)
#define EDID_CHROMA_RG_LOW_RED_Y_SHIFT    (4)       /* Bits 4-5:  Red y value least-significant 2 bits */
#define EDID_CHROMA_RG_LOW_RED_Y_MASK     (3 << EDID_CHROMA_RG_LOW_RED_Y_SHIFT)
#define EDID_CHROMA_RG_LOW_RED_X_SHIFT    (6)       /* Bits 6-7:  Red x value least-significant 2 bits */
#define EDID_CHROMA_RG_LOW_RED_X_MASK     (3 << EDID_CHROMA_RG_LOW_RED_X_SHIFT)

#define EDID_CHROMA_BW_LOW_WHITE_Y_SHIFT  (0)       /* Bits 0-1:  White y value least-significant 2 bits */
#define EDID_CHROMA_BW_LOW_WHITE_Y_MASK   (3 << EDID_CHROMA_BW_LOW_WHITE_Y_SHIFT)
#define EDID_CHROMA_BW_LOW_WHITE_X_SHIFT  (2)       /* Bits 2-3:  White x value least-significant 2 bits */
#define EDID_CHROMA_BW_LOW_WHITE_X_MASK   (3 << EDID_CHROMA_BW_LOW_WHITE_X_SHIFT)
#define EDID_CHROMA_BW_LOW_BLUE_Y_SHIFT   (4)       /* Bits 4-5:  Blue y value least-significant 2 bits */
#define EDID_CHROMA_BW_LOW_BLUE_Y_MASK    (3 << EDID_CHROMA_BW_LOW_BLUE_Y_SHIFT)
#define EDID_CHROMA_BW_LOW_BLUE_X_SHIFT   (6)       /* Bits 6-7:  Blue x value least-significant 2 bits */
#define EDID_CHROMA_BW_LOW_BLUE_X_MASK    (3 << EDID_CHROMA_RG_LOW_RED_X_SHIFT)

#define _CHLO(b,s)                        (((b) >> (s)) & 0x3)
#define _CHHI(b)                          ((b) << 2)
#define _CHHILO(p,l,s,h)                  (_CHLO((p)[l], s) | _CHHI((p)[h]))
#define _CHROMA(p,l,s,h)                  ((_CHHILO(p,l,s,h) * 1000) / 1024)

#define EDID_CHROMA_RED_X(p) \
  (_CHROMA(p, EDID_CHROMA_RG_LOW_OFFSET, EDID_CHROMA_RG_LOW_RED_X_SHIFT, \
           EDID_CHROMA_REDX_OFFSET))
#define EDID_CHROMA_RED_Y(p) \
  (_CHROMA(p, EDID_CHROMA_RG_LOW_OFFSET, EDID_CHROMA_RG_LOW_RED_Y_SHIFT, \
           EDID_CHROMA_REDY_OFFSET))
#define EDID_CHROMA_GREEN_X(p) \
  (_CHROMA(p, EDID_CHROMA_RG_LOW_OFFSET, EDID_CHROMA_RG_LOW_GREEN_X_SHIFT, \
           EDID_CHROMA_GREENX_OFFSET))
#define EDID_CHROMA_GREEN_Y(p) \
  (_CHROMA(p, EDID_CHROMA_RG_LOW_OFFSET, EDID_CHROMA_RG_LOW_GREEN_Y_SHIFT, \
           EDID_CHROMA_GREENY_OFFSET))
#define EDID_CHROMA_BLUE_X(p) \
  (_CHROMA(p, EDID_CHROMA_BW_LOW_OFFSET, EDID_CHROMA_BW_LOW_BLUE_X_SHIFT, \
           EDID_CHROMA_BLUEX_OFFSET))
#define EDID_CHROMA_BLUE_Y(p) \
  (_CHROMA(p, EDID_CHROMA_BW_LOW_OFFSET, EDID_CHROMA_BW_LOW_BLUE_Y_SHIFT, \
           EDID_CHROMA_BLUEY_OFFSET))
#define EDID_CHROMA_WHITE_X(p) \
  (_CHROMA(p, EDID_CHROMA_BW_LOW_OFFSET, EDID_CHROMA_BW_LOW_WHITE_X_SHIFT, \
           EDID_CHROMA_WHITEX_OFFSET))
#define EDID_CHROMA_WHITE_Y(p) \
  (_CHROMA(p, EDID_CHROMA_BW_LOW_OFFSET, EDID_CHROMA_BW_LOW_WHITE_Y_SHIFT, \
           EDID_CHROMA_WHITEY_OFFSET))

/* Bytes 35-37:  Established timing bitmap */

#define EDID_TIMING_1_800x600_60Hz        (1 << 0)
#define EDID_TIMING_1_800x600_56Hz        (1 << 1)
#define EDID_TIMING_1_640x600_75Hz        (1 << 2)
#define EDID_TIMING_1_640x480_72Hz        (1 << 3)
#define EDID_TIMING_1_640x480_67Hz        (1 << 4)
#define EDID_TIMING_1_640x480_60Hz        (1 << 5)
#define EDID_TIMING_1_720x400_88Hz        (1 << 6)
#define EDID_TIMING_1_720x400_70Hz        (1 << 7)

#define EDID_TIMING_2_1280x1024_75Hz      (1 << 0)
#define EDID_TIMING_2_1024x768_75Hz       (1 << 1)
#define EDID_TIMING_2_1024x768_70Hz       (1 << 2)
#define EDID_TIMING_2_1024x768_60Hz       (1 << 3)
#define EDID_TIMING_2_1024x768_87Hz       (1 << 4)
#define EDID_TIMING_2_832x624_75Hz        (1 << 5)
#define EDID_TIMING_2_800x600_75Hz        (1 << 6)
#define EDID_TIMING_2_800x600_72Hz        (1 << 7)

#define EDID_TIMING_3_VENDOR_SHIFT        (0)       /* Bits 0-6: Manufacturer-specific display modes */
#define EDID_TIMING_3_VENDOR_MASK         (0x7f << EDID_TIMING_3_VENDOR_SHIFT)
#define EDID_TIMING_3_1152x870_75Hz       (1 << 7)

/* Standard Timing Information */

#define ASPECT_RATIO_16_10                0         /* Aspect ratio: 16:10 */
#define ASPECT_RATIO_4_3                  1         /* Aspect ratio: 4:3 */
#define ASPECT_RATIO_5_4                  2         /* Aspect ratio: 5:4 */
#define ASPECT_RATIO_16_9                 3         /* Aspect ratio: 16:9 */

#define EDID_STDTIMING_XRES_OFFSET        (0)       /* Byte 0: X resolution, divided by 8, less 31 */
#  define EDID_STDTIMING_ASPECT_SHIFT     (6)       /* Bits 6-7: Image aspect ratio */
#  define EDID_STDTIMING_ASPECT_MASK      (3 << EDID_STDTIMING_ASPECT_SHIFT)
#    define EDID_STDTIMING_ASPECT_16_10   (ASPECT_RATIO_16_10 << EDID_STDTIMING_ASPECT_SHIFT)
#    define EDID_STDTIMING_ASPECT_4_3     (ASPECT_RATIO_4_3 << EDID_STDTIMING_ASPECT_SHIFT)
#    define EDID_STDTIMING_ASPECT_5_4     (ASPECT_RATIO_5_4 << EDID_STDTIMING_ASPECT_SHIFT)
#    define EDID_STDTIMING_ASPECT_16_9    (ASPECT_RATIO_16_9 << EDID_STDTIMING_ASPECT_SHIFT)
#  define EDID_STDTIMING_VFREQ_SHIFT      (0)       /* Bits 0-5: Vertical frequency, less 60 */
#  define EDID_STDTIMING_VFREQ_MASK       (0x3f << EDID_STDTIMING_VFREQ_SHIFT)

#define EDID_STDTIMING_INFO_OFFSET        (1)       /* Byte 1: Image Aspect Ratio / Vertical Frequency */

/* Display Descriptor: EDID Detailed Timing Descriptor */

#define EDID_STEROMODE_FIELDSEQ_RIGHT     1         /* Field sequential, sync=1 during right (bit0=0) */
#define EDID_STEROMODE_FIELDSEQ_LEFT      2         /* Field sequential, sync=1 during left (bit0=0) */
#define EDID_STEROMODE_4WAY_INTERLEAVED   3         /* 4-way interleaved stereo (bit0=0) */

#define EDID_STEROMODE_RIGHT              1         /* Right image on even lines (bit0=1) */
#define EDID_STEROMODE_LEFT               2         /* Left image on even lines (bit0=1) */
#define EDID_STEROMODE_SIDEBYSIDE         3         /* Side-by-side (bit0=1) */

#define EDID_DESC_PIXCLOCK_OFFSET         0         /* Bytes 0-1: Pixel clock in 10 kHz units */
#define EDID_DESC_HPIXELS_LSBITS_OFFSET   2         /* Byte 2: Horizontal active pixels 8 LS bits */
#define EDID_DESC_HBLANK_LSBITS_OFFSET    3         /* Byte 3: Horizontal blanking pixels 8 LS bits */
#define EDID_DESC_HMSBITS_OFFSET          4         /* Byte 4: Horizontal MS bits */
#  define EDID_DESC_HBLANK_MSBITS_SHIFT   (0)       /* Bits 0-3: Horizontal blanking pixels 4 MS bits */
#  define EDID_DESC_HBLANK_MSBITS_MASK    (15 << EDID_DESC_HBLANK_MSBITS_SHIFT)
#  define EDID_DESC_HPIXELS_MSBITS_SHIFT  (4)       /* Bits 4-7: Horizontal active pixels 4 MS bits */
#  define EDID_DESC_HPIXELS_MSBITS_MASK   (15 << EDID_DESC_HPIXELS_MSBITS_SHIFT)
#define EDID_DESC_VLINES_LSBITS_OFFSET    5         /* Byte 5: Vertical active lines 8 LS bits */
#define EDID_DESC_VBLANK_LSBITS_OFFSET    6         /* Byte 6: Vertical blanking lines 8 LS bits */
#define EDID_DESC_VMSBITS_OFFSET          7         /* Byte 7: Vertical MS bits */
#  define EDID_DESC_VBLANK_MSBITS_SHIFT   (0)       /* Bits 0-3: Vertical blanking lines 4 MS bits */
#  define EDID_DESC_VBLANK_MSBITS_MASK    (15 << EDID_DESC_VBLANK_MSBITS_SHIFT)
#  define EDID_DESC_VLINES_MSBITS_SHIFT   (4)       /* Bits 4-7: Vertical active lines 4 MS bits */
#  define EDID_DESC_VLINES_MSBITS_MASK    (15 << EDID_DESC_VLINES_MSBITS_SHIFT)
#define EDID_DESC_HPORCH_LSBITS_OFFSET    8         /* Byte 8: Horizontal front porch pixels 8 LS bits */
#define EDID_DESC_HPW_LSBITS_OFFSET       9         /* Byte 9: Horizontal sync pulse width pixels 8 LS bits */
#define EDID_DESC_VPORCH_LSBITS_OFFSET    10        /* Byte 10: Vertical front porch and pulsewidth LS bits */
#  define EDID_DESC_VPW_LSBITS_SHIFT      (0)       /* Bits 0-3: Vertical sync pulsewidth 4 LS bits */
#  define EDID_DESC_VPW_LSBITS_MASK       (15 << EDID_DESC_VPW_LSBITS_SHIFT)
#  define EDID_DESC_VPORCH_LSBITS_SHIFT   (4)       /* Bits 4-7: Vertical front portch 4 LS bits */
#  define EDID_DESC_VPORCH_LSBITS_MASK    (15 << EDID_DESC_VPORCH_LSBITS_SHIFT)
#define EDID_DESC_PORCH_MSBITS_OFFSET     11        /* Byte 11: Vertical MS bits */
#  define EDID_DESC_VPW_MSBITS_SHIFT      (0)       /* Bits 0-1: Vertical sync pulsewidth lines 2 MS bits */
#  define EDID_DESC_VPW_MSBITS_MASK       (3 << EDID_DESC_VPW_MSBITS_SHIFT)
#  define EDID_DESC_VPORCH_MSBITS_SHIFT   (2)       /* Bits 2-3: Vertical front porch lines 2 MS bits */
#  define EDID_DESC_VPORCH_MSBITS_MASK    (3 << EDID_DESC_VPORCH_MSBITS_SHIFT)
#  define EDID_DESC_HPW_MSBITS_SHIFT      (4)       /* Bits 4-5: Horizontal sync pulsewidth pixels 2 MS bits */
#  define EDID_DESC_HPW_MSBITS_MASK       (3 << EDID_DESC_HPW_MSBITS_SHIFT)
#  define EDID_DESC_HPORCH_MSBITS_SHIFT   (6)       /* Bits 6-7: Horizontal front porch pixels 2 MS bits */
#  define EDID_DESC_HPORCH_MSBITS_MASK    (3 << EDID_DESC_HPORCH_MSBITS_SHIFT)
#define EDID_DESC_HSIZE_LSBITS_OFFSET     12        /* Byte 12: Horizontal image size, mm, 8 LS bits */
#define EDID_DESC_VSIZE_LSBITS_OFFSET     13        /* Byte 13: Vertical image size, mm, 8 LS bits */
#define EDID_DESC_SIZE_MSBITS_OFFSET      14        /* Byte 14: Image size MS bits */
#  define EDID_DESC_VSIZE_MSBITS_SHIFT    (0)       /* Bits 0-3: Vertical image size, mm, 4 MS bits */
#  define EDID_DESC_VSIZE_MSBITS_MASK     (15 << EDID_DESC_VSIZE_MSBITS_SHIFT)
#  define EDID_DESC_HSIZE_MSBITS_SHIFT    (4)       /* Bits 4-7: Horizontal image size, mm, 4 MS bits  */
#  define EDID_DESC_HSIZE_MSBITS_MASK     (15 << EDID_DESC_HSIZE_MSBITS_SHIFT)
#define EDID_DESC_HBORDER_OFFSET          15        /* Byte 15: Horizontal border pixels (one side) */
#define EDID_DESC_VBORDER_OFFSET          16        /* Byte 16: Vertical border lines (one side) */
#define EDID_DESC_FEATURES_OFFSET         17        /* Byte 17: Features bitmap */
                                                    /* If bits 5-6=00: */
#  define EDID_DESC_STEREO_INTERLEAVE     (1 << 0)  /* Bit 0: 2-way line-interleaved or side-by-side
                                                     *        interleaved stereo */
                                                    /* If bits 3-4=0x: */
#  define EDID_DESC_ANALOG_SYNCALL        (1 << 1)  /* Bit 1: Sync on all 3 RGB lines (else green only) */
#  define EDID_DESC_ANALOG_VSERRATION     (1 << 2)  /* Bit 2: VSync serration */
#  define EDID_DESC_ANALOG_SYNCTYPE       (1 << 3)  /* Bit 3: 0=Analog composite; 1=Bipolar analog composite */
                                                    /* If bits 3-4=10: */
#  define EDID DESC_DIGITAL_VPOLARITY     (1 << 2)  /* Bit 2: Vertical sync polarity (0=negative, 1=positive) */
                                                    /* If bits 3-4=11: */
#  define EDID_DESC_DIGITAL_HPOLARITY     (1 << 1)  /* Bit 1: Horizontal Sync polarity (0=negative, 1=positive)  */
#  define EDID_DESC_DIGITAL_VSERRATION    (1 << 2)  /* Bit 2: VSync serration */
#  define EDID_DESC_DIGITAL_SYNCTYPE      (1 << 3)  /* Bit 3: 0=Digital composite; 1=Digital separate sync */
#  define EDID_DESC_DIGITAL_SYNC          (1 << 4)  /* Bit 4: Digital sync */
#  define EDID_DESC_STEREO_SHIFT          (5)       /* Bits 5-6:  Stero mode */
#  define EDID_DESC_STEREO_MASK           (3 << EDID_DESC_STEREO_SHIFT)
#  define EDID_DESC_INTERLACED            (1 << 7)  /* Bit 7: Interlaced */

/* Descriptor helpers */

#define _VACT_LO(p)                       ((p)[EDID_DESC_VLINES_LSBITS_OFFSET])
#define _VBLK_LO(p)                       ((p)[EDID_DESC_VBLANK_LSBITS_OFFSET])
#define _VACT_HI(p)                       (((p)[EDID_DESC_VMSBITS_OFFSET] & EDID_DESC_VLINES_MSBITS_MASK) << 4)
#define _VBLK_HI(p)                       (((p)[EDID_DESC_VMSBITS_OFFSET] & EDID_DESC_VBLANK_MSBITS_MASK) << 8)
#define EDID_DESC_VACTIVE(p)              (_VACT_LO(p) | _VACT_HI(p))
#define EDID_DESC_VBLANK(p)               (_VBLK_LO(p) | _VBLK_HI(p))

#define _HACT_LO(p)                       ((p)[EDID_DESC_HPIXELS_LSBITS_OFFSET])
#define _HBLK_LO(p)                       ((p)[EDID_DESC_HBLANK_LSBITS_OFFSET])
#define _HACT_HI(p)                       (((p)[EDID_DESC_HMSBITS_OFFSET] & EDID_DESC_HPIXELS_MSBITS_SHIFT) << 4)
#define _HBLK_HI(p)                       (((p)[EDID_DESC_HMSBITS_OFFSET] & EDID_DESC_HBLANK_MSBITS_SHIFT) << 8)
#define EDID_DESC_HACTIVE(p)              (_HACT_LO(p) | _HACT_HI(p))
#define EDID_DESC_HBLANK(p)               (_HBLK_LO(p) | _HBLK_HI(p))

#define _HOFF_LO(p)                       ((p)[EDID_DESC_HPORCH_LSBITS_OFFSET])
#define _HWID_LO(p)                       ((p)[EDID_DESC_HPW_LSBITS_OFFSET])
#define _VOFF_LO(p)                       ((p)[EDID_DESC_VPORCH_LSBITS_OFFSET] >> EDID_DESC_VPORCH_LSBITS_SHIFT)
#define _VWID_LO(p)                       ((p)[EDID_DESC_VPORCH_LSBITS_OFFSET] & EDID_DESC_VPW_LSBITS_MASK)
#define _HOFF_HI(p)                       (((p)[EDID_DESC_PORCH_MSBITS_OFFSET] & EDID_DESC_HPORCH_MSBITS_MASK) << 2)
#define _HWID_HI(p)                       (((p)[EDID_DESC_PORCH_MSBITS_OFFSET] & EDID_DESC_HPW_MSBITS_MASK) << 4)
#define _VOFF_HI(p)                       (((p)[EDID_DESC_PORCH_MSBITS_OFFSET] & EDID_DESC_VPORCH_MSBITS_MASK) << 2)
#define _VWID_HI(p)                       (((p)[EDID_DESC_PORCH_MSBITS_OFFSET] & EDID_DESC_VPW_MSBITS_MASK) << 4)
#define EDID_DESC_HSYNC_OFFSET(p)         (_HOFF_LO(p) | _HOFF_HI(p))
#define EDID_DESC_HSYNC_WIDTH(p)          (_HWID_LO(p) | _HWID_HI(p))
#define EDID_DESC_VSYNC_OFFSET(p)         (_VOFF_LO(p) | _VOFF_HI(p))
#define EDID_DESC_VSYNC_WIDTH(p)          (_VWID_LO(p) | _VWID_HI(p))

#define _HSZ_LO(p)                        ((p)[EDID_DESC_HSIZE_LSBITS_OFFSET])
#define _VSZ_LO(p)                        ((p)[EDID_DESC_VSIZE_LSBITS_OFFSET])
#define _HSZ_HI(p)                        (((p)[EDID_DESC_SIZE_MSBITS_OFFSET] & EDID_DESC_HSIZE_MSBITS_MASK) << 4)
#define _VSZ_HI(p)                        (((p)[EDID_DESC_SIZE_MSBITS_OFFSET] & EDID_DESC_VSIZE_MSBITS_MASK) << 8)
#define EDID_DESC_HSIZE(p)                (_HSZ_LO(p) | _HSZ_HI(p))
#define EDID_DESC_VSIZE(p)                (_VSZ_LO(p) | _VSZ_HI(p))

/* Display Descriptor: EDID Other Monitor Descriptors */

#define EDID_DESC_ZERO_1                  0         /* Bytes 0-1: Zero=not a detailed timing descriptor */
#define EDID_DESC_ZERO_2                  2         /* Byte 2: Zero */
#define EDID_DESC_DESCTYPE                3         /* Byte 3: Descriptor type */
#define EDID_DESC_ZERO_3                  4         /* Byte 4: Zero */
#define EDID_DESC_INFO                    5         /* Bytes 5-17: Determined by descriptor type */

                                                    /* 0x00-0x0f:  Manufacturer reserved descriptors */
#define EDID_DESCTYPE_DUMMY               0x10      /* Dummy identifier */
#define EDID_DESCTYPE_STDTIMING           0xf7      /* Additional standard timing 3 */
#define EDID_DESCTYPE_CVT                 0xf8      /* CVT 3-Byte Timing Codes */
#define EDID_DESCTYPE_DCM                 0xf9      /* Display Color Management (DCM) */
#define EDID_DESCTYPE_STDTIMING_ID        0xfa      /* Additional standard timing identifiers */
#define EDID_DESCTYPE_WHITEPOINT          0xfb      /* Additional white point data */
#define EDID_DESCTYPE_NAME                0xfc      /* Display name (ASCII text) */
#define EDID_DESCTYPE_LIMITS              0xfd      /* Display range limits */
#define EDID_DESCTYPE_TEXT                0xfe      /* Unspecified text (ASCII text) */
#define EDID_DESCTYPE_SERIALNO            0xff      /* Display serial number (ASCII text) */

/* Used for descriptors 0xff, 0xfe, and 0xfc */

#define EDID_DESC_ASCII_DATA_OFFSET       5
#define EDID_DESC_ASCII_DATA_LEN          13

#define EDID_DESC_RANGE_MIN_VFREQ(p)      ((p)[5])        /* Hz */
#define EDID_DESC_RANGE_MAX_VFREQ(p)      ((p)[6])        /* Hz */
#define EDID_DESC_RANGE_MIN_HFREQ(p)      ((p)[7])        /* kHz */
#define EDID_DESC_RANGE_MAX_HFREQ(p)      ((p)[8])        /* kHz */
#define EDID_DESC_RANGE_MAX_CLOCK(p)      (((p)[9]) * 10) /* MHz */
#define EDID_DESC_RANGE_HAVE_GTF2(p)      (((p)[10]) == 0x02)
#define EDID_DESC_RANGE_GTF2_HFREQ(p)     (((p)[12]) * 2)
#define EDID_DESC_RANGE_GTF2_C(p)         (((p)[13]) / 2)
#define EDID_DESC_RANGE_GTF2_M(p)         ((p)[14] + ((p)[15] << 8))
#define EDID_DESC_RANGE_GTF2_K(p)         ((p)[16])
#define EDID_DESC_RANGE_GTF2_J(p)         ((p)[17] / 2)

#define EDID_DESC_COLOR_WHITEX(p)
#define EDID_DESC_COLOR_WHITE_INDEX_1(p)  ((p)[5])
#define EDID_DESC_COLOR_WHITEX_1(p)       _CHROMA(p, 6, 2, 7)
#define EDID_DESC_COLOR_WHITEY_1(p)       _CHROMA(p, 6, 0, 8)
#define EDID_DESC_COLOR_GAMMA_1(p)        _GAMMA(p[9])
#define EDID_DESC_COLOR_WHITE_INDEX_2(p)  ((p)[10])
#define EDID_DESC_COLOR_WHITEX_2(p)       _CHROMA(p, 11, 2, 12)
#define EDID_DESC_COLOR_WHITEY_2(p)       _CHROMA(p, 11, 0, 13)
#define EDID_DESC_COLOR_GAMMA_2(p)        _GAMMA(p[14])

#define EDID_DESC_STD_TIMING_START_OFFSET 5
#define EDID_DESC_STD_TIMING_COUNT_OFFSET 6

/* Extended EDID data offsets ****************************************************************/
/* To be provided */

/* EDID Extensions assigned by VESA (First byte of the Extended EDID block) */

#define EDID_EXT_TIMING                   0x00      /* Timing Extension */
#define EDID_EXT_CEA                      0x02      /* Additional Timing Data Block (CEA EDID Timing Extension) */
#define EDID_EXT_VTBEXT                   0x10      /* Video Timing Block Extension (VTB-EXT) */
#define EDID_EXT_V2p0                     0x20      /* EDID 2.0 Extension */
#define EDID_EXT_DIEXT                    0x40      /* Display Information Extension (DI-EXT) */
#define EDID_EXT_LSEXT                    0x50      /* Localized String Extension (LS-EXT) */
#define EDID_EXT_MIEXT                    0x60      /* Microdisplay Interface Extension (MI-EXT) */
#define EDID_EXT_DIDEXIT                  0x70      /* Display ID Extension */
#define EDID_EXT_DTCDB_1                  0xa7      /* Display Transfer Characteristics Data Block (DTCDB) */
#define EDID_EXT_DTCDB_2                  0xaf      /* Display Transfer Characteristics Data Block (DTCDB) */
#define EDID_EXT_DTCDB_3                  0xbf      /* Display Transfer Characteristics Data Block (DTCDB) */
#define EDID_EXT_BLOCKMAP                 0xf0      /* Block Map */
#define EDID_EXT_DDDB                     0xff      /* Display Device Data Block (DDDB) */
#define EDID_EXT_VENDOR                   0xff      /* Extension defined by monitor manufacturer.
                                                     * According to LS-EXT, actual contents varies
                                                     * from manufacturer.  However, the value is
                                                     * later used by DDDB. */

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* These structures is a user-friendly digest of the EDID data. */

struct edid_chroma_s
{
  uint16_t ec_redx;
  uint16_t ec_redy;
  uint16_t ec_greenx;
  uint16_t ec_greeny;
  uint16_t ec_bluex;
  uint16_t ec_bluey;
  uint16_t ec_whitex;
  uint16_t ec_whitey;
};

struct edid_range_s
{
  uint16_t er_min_vfreq;  /* Hz */
  uint16_t er_max_vfreq;  /* Hz */
  uint16_t er_min_hfreq;  /* kHz */
  uint16_t er_max_hfreq;  /* kHz */
  uint16_t er_max_clock;  /* MHz */
  uint16_t er_gtf2_hfreq;

  bool     er_have_gtf2;
  uint16_t er_gtf2_c;
  uint16_t er_gtf2_m;
  uint16_t er_gtf2_k;
  uint16_t er_gtf2_j;
};

struct edid_info_s
{
  uint8_t  edid_manufacturer[4];
  uint8_t  edid_version;
  uint8_t  edid_revision;
  uint8_t  edid_video_input;
  uint8_t  edid_max_hsize;    /* in cm */
  uint8_t  edid_max_vsize;    /* in cm */
  uint8_t  edid_gamma;
  uint8_t  edid_features;
  uint8_t  edid_ext_block_count;
  uint16_t edid_product;
  uint32_t edid_serial;
  uint16_t edid_year;
  uint8_t  edid_week;
  bool     edid_have_range;

  struct edid_range_s  edid_range;
  struct edid_chroma_s edid_chroma;

  /* Parsed modes */

  FAR struct videomode_s *edid_preferred_mode;
  int      edid_nmodes;
  struct videomode_s edid_modes[64];
};

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

/********************************************************************************************
 * Name:  edid_parse
 *
 * Description:
 *   Given a block of raw EDID data, parse the data and convert it to the 'digested' form
 *   of struct edid_info_s.
 *
 * Input Parameters:
 *   data - A reference to the raw EDID data
 *   edid - The location to return the digested EDID data.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ********************************************************************************************/

int edid_parse(FAR const uint8_t *data, FAR struct edid_info_s *edid);

/********************************************************************************************
 * Name:  edid_dump
 *
 * Description:
 *   Dump the full content of the EDID
 *
 * Input Parameters:
 *   edid - The edid to be dumped
 *
 * Returned Value:
 *   None
 *
 ********************************************************************************************/

void edid_dump(FAR const struct edid_info_s *edid);

#endif /* __INCLUDE_NUTTX_VIDEO_EDID_H */
