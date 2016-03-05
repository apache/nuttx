/****************************************************************************
 * arch/arm/include/imx6/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_IMX6_CHIP_H
#define __ARCH_ARM_INCLUDE_IMX6_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The i.MX6 6Quad and 6Dual/DualLite are the only support i.MX6 family
 * members.  Individual differences between members of the families are not
 * accounted for.
 */

#if defined(CONFIG_ARCH_CHIP_IMX6_6QUAD)
#  define IMX_OCRAM_SIZE   (256*1024)   /* Size of the On-Chip RAM (OCRAM) */
#  define IMX_L2CACHE_SIZE (1024*1024)  /* 1MB L2 Cache */
#  define IMX_NXCPUS       4            /* Four CPUs */
#  define IMX_NGPU3D       1            /* One 3D graphics engine */
#  define IMX_N32SHADERS   4            /* Four 3D shaders */
#  define IMX_NGPU2D       2            /* Two 2D graphics engines */
#  define IMX_HAVE_DDR64   1            /* 64-bit DDR3 */
#  undef  IMX_HAVE_DDR32                /* 32-bit DDR3 */
#  define IMX_HAVE_DDR32x2 1            /* Two channel 32-bit DDR3 */
#  define IMX_HAVE_SATAII  1            /* Integrated SATA-II */
#  undef  IMX_HAVE_EPD                  /* No interated EPD controller */
#elif defined(CONFIG_ARCH_CHIP_IMX6_6DUAL)
#  define IMX_OCRAM_SIZE   (256*1024)   /* Size of the On-Chip RAM (OCRAM) */
#  define IMX_L2CACHE_SIZE (1024*1024)  /* 1MB L2 Cache */
#  define IMX_NXCPUS       2            /* Two CPUs */
#  define IMX_NGPU3D       1            /* One 3D graphics engine */
#  define IMX_N32SHADERS   4            /* Four 3D shaders */
#  define IMX_NGPU2D       2            /* Two 2D graphics engines */
#  define IMX_HAVE_DDR64   1            /* 64-bit DDR3 */
#  undef  IMX_HAVE_DDR32                /* 32-bit DDR3 */
#  define IMX_HAVE_DDR32x2 1            /* Two channel 32-bit DDR3 */
#  define IMX_HAVE_SATAII  1            /* Integrated SATA-II */
#  undef  IMX_HAVE_EPD                  /* No interated EPD controller */
#elif defined(CONFIG_ARCH_CHIP_IMX6_6DUALLITE)
#  define IMX_OCRAM_SIZE   (256*1024)   /* Size of the On-Chip RAM (OCRAM) */
#  define IMX_L2CACHE_SIZE (512*1024)   /* 512KB L2 Cache */
#  define IMX_NXCPUS       2            /* Two CPUs */
#  define IMX_NGPU3D       1            /* One 3D graphics engine */
#  define IMX_N32SHADERS   1            /* One 3D shaders */
#  define IMX_NGPU2D       1            /* One 2D graphics engine */
#  define IMX_HAVE_DDR64   1            /* 64-bit DDR3 */
#  undef  IMX_HAVE_DDR32                /* No 32-bit DDR3 */
#  define IMX_HAVE_DDR32x2 1            /* Two channel 32-bit DDR3 */
#  undef  IMX_HAVE_SATAII               /* No integrated SATA-II */
#  define IMX_HAVE_EPD     1            /* Interated EPD controller */
#elif defined(CONFIG_ARCH_CHIP_IMX6_6SOLO)
#  define IMX_OCRAM_SIZE   (256*1024)   /* Size of the On-Chip RAM (OCRAM) */
#  define IMX_L2CACHE_SIZE (512*1024)   /* 512KB L2 Cache */
#  define IMX_NXCPUS       1            /* One CPU */
#  define IMX_NGPU3D       1            /* One 3D graphics engine */
#  define IMX_N32SHADERS   1            /* One 3D shaders */
#  define IMX_NGPU2D       1            /* One 2D graphics engine */
#  undef  IMX_HAVE_DDR64                /* No 64-bit DDR3 */
#  define IMX_HAVE_DDR32   1            /* 32-bit DDR3 */
#  undef  IMX_HAVE_DDR32x2              /* No two channel 32-bit DDR3 */
#  undef  IMX_HAVE_SATAII               /* No integrated SATA-II */
#  define IMX_HAVE_EPD     1            /* Interated EPD controller */
#elif defined(CONFIG_ARCH_CHIP_IMX6_6SOLOLITE)
#  define IMX_OCRAM_SIZE   (256*1024)   /* Size of the On-Chip RAM (OCRAM) */
#  define IMX_L2CACHE_SIZE (256*1024)   /* 256KB L2 Cache */
#  define IMX_NXCPUS       1            /* One CPU */
#  undef  IMX_NGPU3D                    /* No 3D graphics engine */
#  define IMX_N32SHADERS   0            /* No 3D shaders */
#  define IMX_NGPU2D       1            /* One 2D graphics engine */
#  undef  IMX_HAVE_DDR64                /* No 64-bit DDR3 */
#  define IMX_HAVE_DDR32   1            /* 32-bit DDR3 */
#  undef  IMX_HAVE_DDR32x2              /* No two channel 32-bit DDR3 */
#  undef  IMX_HAVE_SATAII               /* No integrated SATA-II */
#  define IMX_HAVE_EPD     1            /* Interated EPD controller */
#else
#  error Unspecified i.MX6 chip
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_IMX6_CHIP_H */
