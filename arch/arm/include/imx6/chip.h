/****************************************************************************
 * arch/arm/include/imx6/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_IMX6_CHIP_H
#define __ARCH_ARM_INCLUDE_IMX6_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
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
#  undef  IMX_HAVE_EPD                  /* No integrated EPD controller */
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
#  undef  IMX_HAVE_EPD                  /* No integrated EPD controller */
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
#  define IMX_HAVE_EPD     1            /* Integrated EPD controller */
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
#  define IMX_HAVE_EPD     1            /* Integrated EPD controller */
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
#  define IMX_HAVE_EPD     1            /* Integrated EPD controller */
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
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_IMX6_CHIP_H */
