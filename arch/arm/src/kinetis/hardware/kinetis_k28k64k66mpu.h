/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_k28k64k66mpu.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_K28K64K66MPU_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_K28K64K66MPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_MPU_CESR_OFFSET        0x0000 /* Control/Error Status Register */

#define KINETIS_MPU_EAR_OFFSET(n)      (0x0010+((n)<<3)) /* Error Address Register, Slave Port n */
#define KINETIS_MPU_EDR_OFFSET(n)      (0x0014+((n)<<3)) /* Error Detail Register, Slave Port n */

#define KINETIS_MPU_EAR0_OFFSET        0x0010 /* Error Address Register, Slave Port 0 */
#define KINETIS_MPU_EDR0_OFFSET        0x0014 /* Error Detail Register, Slave Port 0 */
#define KINETIS_MPU_EAR1_OFFSET        0x0018 /* Error Address Register, Slave Port 1 */
#define KINETIS_MPU_EDR1_OFFSET        0x001c /* Error Detail Register, Slave Port 1 */
#define KINETIS_MPU_EAR2_OFFSET        0x0020 /* Error Address Register, Slave Port 2 */
#define KINETIS_MPU_EDR2_OFFSET        0x0024 /* Error Detail Register, Slave Port 2 */
#define KINETIS_MPU_EAR3_OFFSET        0x0028 /* Error Address Register, Slave Port 3 */
#define KINETIS_MPU_EDR3_OFFSET        0x002c /* Error Detail Register, Slave Port 3 */
#define KINETIS_MPU_EAR4_OFFSET        0x0030 /* Error Address Register, Slave Port 4 */
#define KINETIS_MPU_EDR4_OFFSET        0x0034 /* Error Detail Register, Slave Port 4 */

#if defined(KINETIS_K28)
#  define KINETIS_MPU_EAR5_OFFSET      0x0038 /* Error Address Register, Slave Port 5 */
#  define KINETIS_MPU_EDR5_OFFSET      0x003c /* Error Detail Register, Slave Port 5 */
#  define KINETIS_MPU_EAR6_OFFSET      0x0040 /* Error Address Register, Slave Port 6 */
#  define KINETIS_MPU_EDR6_OFFSET      0x0044 /* Error Detail Register, Slave Port 6 */
#endif

#define KINETIS_MPU_RGD_WORD_OFFSET(n,m) (x0400+((n)<<4)+((m)<< 2) /* Region Descriptor n, Word m */

#define KINETIS_MPU_RGD0_WORD0_OFFSET  0x0400 /* Region Descriptor 0, Word 0 */
#define KINETIS_MPU_RGD0_WORD1_OFFSET  0x0404 /* Region Descriptor 0, Word 1 */
#define KINETIS_MPU_RGD0_WORD2_OFFSET  0x0408 /* Region Descriptor 0, Word 2 */
#define KINETIS_MPU_RGD0_WORD3_OFFSET  0x040c /* Region Descriptor 0, Word 3 */
#define KINETIS_MPU_RGD1_WORD0_OFFSET  0x0410 /* Region Descriptor 1, Word 0 */
#define KINETIS_MPU_RGD1_WORD1_OFFSET  0x0414 /* Region Descriptor 1, Word 1 */
#define KINETIS_MPU_RGD1_WORD2_OFFSET  0x0418 /* Region Descriptor 1, Word 2 */
#define KINETIS_MPU_RGD1_WORD3_OFFSET  0x041c /* Region Descriptor 1, Word 3 */
#define KINETIS_MPU_RGD2_WORD0_OFFSET  0x0420 /* Region Descriptor 2, Word 0 */
#define KINETIS_MPU_RGD2_WORD1_OFFSET  0x0424 /* Region Descriptor 2, Word 1 */
#define KINETIS_MPU_RGD2_WORD2_OFFSET  0x0428 /* Region Descriptor 2, Word 2 */
#define KINETIS_MPU_RGD2_WORD3_OFFSET  0x042c /* Region Descriptor 2, Word 3 */
#define KINETIS_MPU_RGD3_WORD0_OFFSET  0x0430 /* Region Descriptor 3, Word 0 */
#define KINETIS_MPU_RGD3_WORD1_OFFSET  0x0434 /* Region Descriptor 3, Word 1 */
#define KINETIS_MPU_RGD3_WORD2_OFFSET  0x0438 /* Region Descriptor 3, Word 2 */
#define KINETIS_MPU_RGD3_WORD3_OFFSET  0x043c /* Region Descriptor 3, Word 3 */
#define KINETIS_MPU_RGD4_WORD0_OFFSET  0x0440 /* Region Descriptor 4, Word 0 */
#define KINETIS_MPU_RGD4_WORD1_OFFSET  0x0444 /* Region Descriptor 4, Word 1 */
#define KINETIS_MPU_RGD4_WORD2_OFFSET  0x0448 /* Region Descriptor 4, Word 2 */
#define KINETIS_MPU_RGD4_WORD3_OFFSET  0x044c /* Region Descriptor 4, Word 3 */
#define KINETIS_MPU_RGD5_WORD0_OFFSET  0x0450 /* Region Descriptor 5, Word 0 */
#define KINETIS_MPU_RGD5_WORD1_OFFSET  0x0454 /* Region Descriptor 5, Word 1 */
#define KINETIS_MPU_RGD5_WORD2_OFFSET  0x0458 /* Region Descriptor 5, Word 2 */
#define KINETIS_MPU_RGD5_WORD3_OFFSET  0x045c /* Region Descriptor 5, Word 3 */
#define KINETIS_MPU_RGD6_WORD0_OFFSET  0x0460 /* Region Descriptor 6, Word 0 */
#define KINETIS_MPU_RGD6_WORD1_OFFSET  0x0464 /* Region Descriptor 6, Word 1 */
#define KINETIS_MPU_RGD6_WORD2_OFFSET  0x0468 /* Region Descriptor 6, Word 2 */
#define KINETIS_MPU_RGD6_WORD3_OFFSET  0x046c /* Region Descriptor 6, Word 3 */
#define KINETIS_MPU_RGD7_WORD0_OFFSET  0x0470 /* Region Descriptor 7, Word 0 */
#define KINETIS_MPU_RGD7_WORD1_OFFSET  0x0474 /* Region Descriptor 7, Word 1 */
#define KINETIS_MPU_RGD7_WORD2_OFFSET  0x0478 /* Region Descriptor 7, Word 2 */
#define KINETIS_MPU_RGD7_WORD3_OFFSET  0x047c /* Region Descriptor 7, Word 3 */
#define KINETIS_MPU_RGD8_WORD0_OFFSET  0x0480 /* Region Descriptor 8, Word 0 */
#define KINETIS_MPU_RGD8_WORD1_OFFSET  0x0484 /* Region Descriptor 8, Word 1 */
#define KINETIS_MPU_RGD8_WORD2_OFFSET  0x0488 /* Region Descriptor 8, Word 2 */
#define KINETIS_MPU_RGD8_WORD3_OFFSET  0x048c /* Region Descriptor 8, Word 3 */
#define KINETIS_MPU_RGD9_WORD0_OFFSET  0x0490 /* Region Descriptor 9, Word 0 */
#define KINETIS_MPU_RGD9_WORD1_OFFSET  0x0494 /* Region Descriptor 9, Word 1 */
#define KINETIS_MPU_RGD9_WORD2_OFFSET  0x0498 /* Region Descriptor 9, Word 2 */
#define KINETIS_MPU_RGD9_WORD3_OFFSET  0x049c /* Region Descriptor 9, Word 3 */
#define KINETIS_MPU_RGD10_WORD0_OFFSET 0x04a0 /* Region Descriptor 10, Word 0 */
#define KINETIS_MPU_RGD10_WORD1_OFFSET 0x04a4 /* Region Descriptor 10, Word 1 */
#define KINETIS_MPU_RGD10_WORD2_OFFSET 0x04a8 /* Region Descriptor 10, Word 2 */
#define KINETIS_MPU_RGD10_WORD3_OFFSET 0x04ac /* Region Descriptor 10, Word 3 */
#define KINETIS_MPU_RGD11_WORD0_OFFSET 0x04b0 /* Region Descriptor 11, Word 0 */
#define KINETIS_MPU_RGD11_WORD1_OFFSET 0x04b4 /* Region Descriptor 11, Word 1 */
#define KINETIS_MPU_RGD11_WORD2_OFFSET 0x04b8 /* Region Descriptor 11, Word 2 */
#define KINETIS_MPU_RGD11_WORD3_OFFSET 0x04bc /* Region Descriptor 11, Word 3 */

#define KINETIS_MPU_RGDAAC_OFFSET(n)   (0x0800+((n)<<2)) /* Region Descriptor Alternate Access Control n */

#define KINETIS_MPU_RGDAAC0_OFFSET     0x0800 /* Region Descriptor Alternate Access Control 0 */
#define KINETIS_MPU_RGDAAC1_OFFSET     0x0804 /* Region Descriptor Alternate Access Control 1 */
#define KINETIS_MPU_RGDAAC2_OFFSET     0x0808 /* Region Descriptor Alternate Access Control 2 */
#define KINETIS_MPU_RGDAAC3_OFFSET     0x080c /* Region Descriptor Alternate Access Control 3 */
#define KINETIS_MPU_RGDAAC4_OFFSET     0x0810 /* Region Descriptor Alternate Access Control 4 */
#define KINETIS_MPU_RGDAAC5_OFFSET     0x0814 /* Region Descriptor Alternate Access Control 5 */
#define KINETIS_MPU_RGDAAC6_OFFSET     0x0818 /* Region Descriptor Alternate Access Control 6 */
#define KINETIS_MPU_RGDAAC7_OFFSET     0x081c /* Region Descriptor Alternate Access Control 7 */
#define KINETIS_MPU_RGDAAC8_OFFSET     0x0820 /* Region Descriptor Alternate Access Control 8 */
#define KINETIS_MPU_RGDAAC9_OFFSET     0x0824 /* Region Descriptor Alternate Access Control 9 */
#define KINETIS_MPU_RGDAAC10_OFFSET    0x0828 /* Region Descriptor Alternate Access Control 10 */
#define KINETIS_MPU_RGDAAC11_OFFSET    0x082c /* Region Descriptor Alternate Access Control 11 */

/* Register Addresses *******************************************************/

#define KINETIS_MPU_CESR               (KINETIS_MPU_BASE+KINETIS_MPU_CESR_OFFSET)

#define KINETIS_MPU_EAR(n)             (KINETIS_MPU_BASE+KINETIS_MPU_EAR_OFFSET(n))
#define KINETIS_MPU_EDR(n)             (KINETIS_MPU_BASE+KINETIS_MPU_EDR_OFFSET(n))

#define KINETIS_MPU_EAR0               (KINETIS_MPU_BASE+KINETIS_MPU_EAR0_OFFSET)
#define KINETIS_MPU_EDR0               (KINETIS_MPU_BASE+KINETIS_MPU_EDR0_OFFSET)
#define KINETIS_MPU_EAR1               (KINETIS_MPU_BASE+KINETIS_MPU_EAR1_OFFSET)
#define KINETIS_MPU_EDR1               (KINETIS_MPU_BASE+KINETIS_MPU_EDR1_OFFSET)
#define KINETIS_MPU_EAR2               (KINETIS_MPU_BASE+KINETIS_MPU_EAR2_OFFSET)
#define KINETIS_MPU_EDR2               (KINETIS_MPU_BASE+KINETIS_MPU_EDR2_OFFSET)
#define KINETIS_MPU_EAR3               (KINETIS_MPU_BASE+KINETIS_MPU_EAR3_OFFSET)
#define KINETIS_MPU_EDR3               (KINETIS_MPU_BASE+KINETIS_MPU_EDR3_OFFSET)
#define KINETIS_MPU_EAR4               (KINETIS_MPU_BASE+KINETIS_MPU_EAR4_OFFSET)
#define KINETIS_MPU_EDR4               (KINETIS_MPU_BASE+KINETIS_MPU_EDR4_OFFSET)

#if defined(KINETIS_K28)
#  define KINETIS_MPU_EAR5             (KINETIS_MPU_BASE+KINETIS_MPU_EAR5_OFFSET)
#  define KINETIS_MPU_EDR5             (KINETIS_MPU_BASE+KINETIS_MPU_EDR5_OFFSET)
#  define KINETIS_MPU_EAR6             (KINETIS_MPU_BASE+KINETIS_MPU_EAR6_OFFSET)
#  define KINETIS_MPU_EDR7             (KINETIS_MPU_BASE+KINETIS_MPU_EDR6_OFFSET)
#endif

#define KINETIS_MPU_RGD_WORD(n,m)      (KINETIS_MPU_BASE+KINETIS_MPU_RGD_WORD_OFFSET(n,m))

#define KINETIS_MPU_RGD0_WORD0         (KINETIS_MPU_BASE+KINETIS_MPU_RGD0_WORD0_OFFSET)
#define KINETIS_MPU_RGD0_WORD1         (KINETIS_MPU_BASE+KINETIS_MPU_RGD0_WORD1_OFFSET)
#define KINETIS_MPU_RGD0_WORD2         (KINETIS_MPU_BASE+KINETIS_MPU_RGD0_WORD2_OFFSET)
#define KINETIS_MPU_RGD0_WORD3         (KINETIS_MPU_BASE+KINETIS_MPU_RGD0_WORD3_OFFSET)
#define KINETIS_MPU_RGD1_WORD0         (KINETIS_MPU_BASE+KINETIS_MPU_RGD1_WORD0_OFFSET)
#define KINETIS_MPU_RGD1_WORD1         (KINETIS_MPU_BASE+KINETIS_MPU_RGD1_WORD1_OFFSET)
#define KINETIS_MPU_RGD1_WORD2         (KINETIS_MPU_BASE+KINETIS_MPU_RGD1_WORD2_OFFSET)
#define KINETIS_MPU_RGD1_WORD3         (KINETIS_MPU_BASE+KINETIS_MPU_RGD1_WORD3_OFFSET)
#define KINETIS_MPU_RGD2_WORD0         (KINETIS_MPU_BASE+KINETIS_MPU_RGD2_WORD0_OFFSET)
#define KINETIS_MPU_RGD2_WORD1         (KINETIS_MPU_BASE+KINETIS_MPU_RGD2_WORD1_OFFSET)
#define KINETIS_MPU_RGD2_WORD2         (KINETIS_MPU_BASE+KINETIS_MPU_RGD2_WORD2_OFFSET)
#define KINETIS_MPU_RGD2_WORD3         (KINETIS_MPU_BASE+KINETIS_MPU_RGD2_WORD3_OFFSET)
#define KINETIS_MPU_RGD3_WORD0         (KINETIS_MPU_BASE+KINETIS_MPU_RGD3_WORD0_OFFSET)
#define KINETIS_MPU_RGD3_WORD1         (KINETIS_MPU_BASE+KINETIS_MPU_RGD3_WORD1_OFFSET)
#define KINETIS_MPU_RGD3_WORD2         (KINETIS_MPU_BASE+KINETIS_MPU_RGD3_WORD2_OFFSET)
#define KINETIS_MPU_RGD3_WORD3         (KINETIS_MPU_BASE+KINETIS_MPU_RGD3_WORD3_OFFSET)
#define KINETIS_MPU_RGD4_WORD0         (KINETIS_MPU_BASE+KINETIS_MPU_RGD4_WORD0_OFFSET)
#define KINETIS_MPU_RGD4_WORD1         (KINETIS_MPU_BASE+KINETIS_MPU_RGD4_WORD1_OFFSET)
#define KINETIS_MPU_RGD4_WORD2         (KINETIS_MPU_BASE+KINETIS_MPU_RGD4_WORD2_OFFSET)
#define KINETIS_MPU_RGD4_WORD3         (KINETIS_MPU_BASE+KINETIS_MPU_RGD4_WORD3_OFFSET)
#define KINETIS_MPU_RGD5_WORD0         (KINETIS_MPU_BASE+KINETIS_MPU_RGD5_WORD0_OFFSET)
#define KINETIS_MPU_RGD5_WORD1         (KINETIS_MPU_BASE+KINETIS_MPU_RGD5_WORD1_OFFSET)
#define KINETIS_MPU_RGD5_WORD2         (KINETIS_MPU_BASE+KINETIS_MPU_RGD5_WORD2_OFFSET)
#define KINETIS_MPU_RGD5_WORD3         (KINETIS_MPU_BASE+KINETIS_MPU_RGD5_WORD3_OFFSET)
#define KINETIS_MPU_RGD6_WORD0         (KINETIS_MPU_BASE+KINETIS_MPU_RGD6_WORD0_OFFSET)
#define KINETIS_MPU_RGD6_WORD1         (KINETIS_MPU_BASE+KINETIS_MPU_RGD6_WORD1_OFFSET)
#define KINETIS_MPU_RGD6_WORD2         (KINETIS_MPU_BASE+KINETIS_MPU_RGD6_WORD2_OFFSET)
#define KINETIS_MPU_RGD6_WORD3         (KINETIS_MPU_BASE+KINETIS_MPU_RGD6_WORD3_OFFSET)
#define KINETIS_MPU_RGD7_WORD0         (KINETIS_MPU_BASE+KINETIS_MPU_RGD7_WORD0_OFFSET)
#define KINETIS_MPU_RGD7_WORD1         (KINETIS_MPU_BASE+KINETIS_MPU_RGD7_WORD1_OFFSET)
#define KINETIS_MPU_RGD7_WORD2         (KINETIS_MPU_BASE+KINETIS_MPU_RGD7_WORD2_OFFSET)
#define KINETIS_MPU_RGD7_WORD3         (KINETIS_MPU_BASE+KINETIS_MPU_RGD7_WORD3_OFFSET)
#define KINETIS_MPU_RGD8_WORD0         (KINETIS_MPU_BASE+KINETIS_MPU_RGD8_WORD0_OFFSET)
#define KINETIS_MPU_RGD8_WORD1         (KINETIS_MPU_BASE+KINETIS_MPU_RGD8_WORD1_OFFSET)
#define KINETIS_MPU_RGD8_WORD2         (KINETIS_MPU_BASE+KINETIS_MPU_RGD8_WORD2_OFFSET)
#define KINETIS_MPU_RGD8_WORD3         (KINETIS_MPU_BASE+KINETIS_MPU_RGD8_WORD3_OFFSET)
#define KINETIS_MPU_RGD9_WORD0         (KINETIS_MPU_BASE+KINETIS_MPU_RGD9_WORD0_OFFSET)
#define KINETIS_MPU_RGD9_WORD1         (KINETIS_MPU_BASE+KINETIS_MPU_RGD9_WORD1_OFFSET)
#define KINETIS_MPU_RGD9_WORD2         (KINETIS_MPU_BASE+KINETIS_MPU_RGD9_WORD2_OFFSET)
#define KINETIS_MPU_RGD9_WORD3         (KINETIS_MPU_BASE+KINETIS_MPU_RGD9_WORD3_OFFSET)
#define KINETIS_MPU_RGD10_WORD0        (KINETIS_MPU_BASE+KINETIS_MPU_RGD10_WORD0_OFFSET)
#define KINETIS_MPU_RGD10_WORD1        (KINETIS_MPU_BASE+KINETIS_MPU_RGD10_WORD1_OFFSET)
#define KINETIS_MPU_RGD10_WORD2        (KINETIS_MPU_BASE+KINETIS_MPU_RGD10_WORD2_OFFSET)
#define KINETIS_MPU_RGD10_WORD3        (KINETIS_MPU_BASE+KINETIS_MPU_RGD10_WORD3_OFFSET)
#define KINETIS_MPU_RGD11_WORD0        (KINETIS_MPU_BASE+KINETIS_MPU_RGD11_WORD0_OFFSET)
#define KINETIS_MPU_RGD11_WORD1        (KINETIS_MPU_BASE+KINETIS_MPU_RGD11_WORD1_OFFSET)
#define KINETIS_MPU_RGD11_WORD2        (KINETIS_MPU_BASE+KINETIS_MPU_RGD11_WORD2_OFFSET)
#define KINETIS_MPU_RGD11_WORD3        (KINETIS_MPU_BASE+KINETIS_MPU_RGD11_WORD3_OFFSET)

#define KINETIS_MPU_RGDAAC(n)          (KINETIS_MPU_BASE+KINETIS_MPU_RGDAAC_OFFSET(n))

#define KINETIS_MPU_RGDAAC0            (KINETIS_MPU_BASE+KINETIS_MPU_RGDAAC0_OFFSET)
#define KINETIS_MPU_RGDAAC1            (KINETIS_MPU_BASE+KINETIS_MPU_RGDAAC1_OFFSET)
#define KINETIS_MPU_RGDAAC2            (KINETIS_MPU_BASE+KINETIS_MPU_RGDAAC2_OFFSET)
#define KINETIS_MPU_RGDAAC3            (KINETIS_MPU_BASE+KINETIS_MPU_RGDAAC3_OFFSET)
#define KINETIS_MPU_RGDAAC4            (KINETIS_MPU_BASE+KINETIS_MPU_RGDAAC4_OFFSET)
#define KINETIS_MPU_RGDAAC5            (KINETIS_MPU_BASE+KINETIS_MPU_RGDAAC5_OFFSET)
#define KINETIS_MPU_RGDAAC6            (KINETIS_MPU_BASE+KINETIS_MPU_RGDAAC6_OFFSET)
#define KINETIS_MPU_RGDAAC7            (KINETIS_MPU_BASE+KINETIS_MPU_RGDAAC7_OFFSET)
#define KINETIS_MPU_RGDAAC8            (KINETIS_MPU_BASE+KINETIS_MPU_RGDAAC8_OFFSET)
#define KINETIS_MPU_RGDAAC9            (KINETIS_MPU_BASE+KINETIS_MPU_RGDAAC9_OFFSET)
#define KINETIS_MPU_RGDAAC10           (KINETIS_MPU_BASE+KINETIS_MPU_RGDAAC10_OFFSET)
#define KINETIS_MPU_RGDAAC11           (KINETIS_MPU_BASE+KINETIS_MPU_RGDAAC11_OFFSET)

/* Register Bit Definitions *************************************************/

/* Control/Error Status Register */

#define MPU_CESR_VLD                   (1 << 0)  /* Bit 0:  Valid (global enable/disable for the MPU) */
                                                 /* Bits 1-7: Reserved */
#define MPU_CESR_NRGD_SHIFT            (8)       /* Bits 8-11: Number of region descriptors */
#define MPU_CESR_NRGD_MASK             (15 << MPU_CESR_NRGD_SHIFT)
#  define MPU_CESR_NRGD_8DESC          (0 << MPU_CESR_NRGD_SHIFT) /* 8 region descriptors */
#  define MPU_CESR_NRGD_12DESC         (1 << MPU_CESR_NRGD_SHIFT) /* 12 region descriptors */
#  define MPU_CESR_NRGD_16DESC         (2 << MPU_CESR_NRGD_SHIFT) /* 16 region descriptors */

#define MPU_CESR_NSP_SHIFT             (12)      /* Bits 12-15: Number of slave ports */
#define MPU_CESR_NSP_MASK              (15 << MPU_CESR_NSP_SHIFT)
#define MPU_CESR_HRL_SHIFT             (16)      /* Bits 16-19: Hardware revision level */
#define MPU_CESR_HRL_MASK              (15 << MPU_CESR_HRL_SHIFT)
                                                 /* Bits 20-26: Reserved */
#define MPU_CESR_SPERR_SHIFT           (27)      /* Bits 27-31: Slave port n error */
#define MPU_CESR_SPERR_MASK            (31 << MPU_CESR_SPERR_SHIFT)
#  define MPU_CESR_SPERR_SPORT(n)      ((1 << (4-(n))) << MPU_CESR_SPERR_SHIFT) /* Slave port nn */

#  define MPU_CESR_SPERR_SPORT0        (16 << MPU_CESR_SPERR_SHIFT) /* Slave port 0 */
#  define MPU_CESR_SPERR_SPORT1        (8 << MPU_CESR_SPERR_SHIFT)  /* Slave port 1 */
#  define MPU_CESR_SPERR_SPORT2        (4 << MPU_CESR_SPERR_SHIFT)  /* Slave port 2 */
#  define MPU_CESR_SPERR_SPORT3        (2 << MPU_CESR_SPERR_SHIFT)  /* Slave port 3 */
#  define MPU_CESR_SPERR_SPORT4        (1 << MPU_CESR_SPERR_SHIFT)  /* Slave port 4 */

/* Error Address Register, Slave Port n.  32-bit error address. */

/* Error Detail Register, Slave Port n */

#define MPU_EDR_ERW                    (1 << 0)  /* Bit 0:  Error read/write */
#define MPU_EDR_EATTR_SHIFT            (1)       /* Bits 1-3: Error attributes */
#define MPU_EDR_EATTR_MASK             (7 << MPU_EDR_EATTR_SHIFT)
#  define MPU_EDR_EATTR_USRINST        (0 << MPU_EDR_EATTR_SHIFT) /* User mode, instruction access */
#  define MPU_EDR_EATTR_USRDATA        (1 << MPU_EDR_EATTR_SHIFT) /* User mode, data access */
#  define MPU_EDR_EATTR_SUPINST        (2 << MPU_EDR_EATTR_SHIFT) /* Supervisor mode, instruction access */
#  define MPU_EDR_EATTR_SUPDATA        (3 << MPU_EDR_EATTR_SHIFT) /* Supervisor mode, data access */

#define MPU_EDR_EMN_SHIFT              (4)       /* Bits 4-7: Error master number */
#define MPU_EDR_EMN_MASK               (15 << MPU_EDR_EMN_SHIFT)
#if defined(KINETIS_K28)
#  define MPU_EDR_EPID_SHIFT           (8)       /* Bits 8-15: Error Process Identification */
#  define MPU_EDR_EPID_MASK            (0xff << MPU_EDR_EPID_SHIFT)
#endif
#define MPU_EDR_EACD_SHIFT             (26)      /* Bits 16-31: Error access control detail */
#define MPU_EDR_EACD_MASK              (0xffff << MPU_EDR_EACD_SHIFT)

/* Region Descriptor n, Word 0 */

                                                 /* Bits 0-4: Reserved */
#define MPU_RGD_WORD0_SRTADDR_SHIFT    (5)       /* Bits 5-31: Start address */
#define MPU_RGD_WORD0_SRTADDR_MASK     (0xffffffe0)

/* Region Descriptor n, Word 1 */

                                                 /* Bits 0-4: Reserved */
#define MPU_RGD_WORD1_ENDADDR_SHIFT    (5)       /* Bits 5-31: End address */
#define MPU_RGD_WORD1_ENDADDR_MASK     (0xffffffe0)

/* Region Descriptor n, Word 2 */

#define MPU_RGD_MSM_RWX                0         /* R/W/X; read, write and execute allowed */
#define MPU_RGD_MSM_RX                 1         /* R/X; read and execute allowed, but no write */
#define MPU_RGD_MSM_RW                 2         /* R/W; read and write allowed, but no execute */
#define MPU_RGD_MSM_UM                 3         /* Same as user mode defined in MUM */

#define MPU_RGD_MUM_R                  4         /* Read allowed */
#define MPU_RGD_MUM_W                  2         /* Write allowed */
#define MPU_RGD_MUM_X                  1         /* Execute allocated */

#define MPU_RGD_WORD2_M0UM_SHIFT       (0)       /* Bits 0-2: Bus master 0 user mode access control */
#define MPU_RGD_WORD2_M0UM_MASK        (7 << MPU_RGD_WORD2_M0UM_SHIFT)
#define MPU_RGD_WORD2_M0SM_SHIFT       (3)       /* Bits 3-4: Bus master 0 supervisor mode access control */
#define MPU_RGD_WORD2_M0SM_MASK        (3 << MPU_RGD_WORD2_M0SM_SHIFT)
#define MPU_RGD_WORD2_M0PE             (1 << 5)  /* Bit 5: Bus Master 0 Process Identifier Enable */
#define MPU_RGD_WORD2_M1UM_SHIFT       (6)       /* Bits 6-8: Bus master 1 user mode access control */
#define MPU_RGD_WORD2_M1UM_MASK        (7 << MPU_RGD_WORD2_M1UM_SHIFT)
#define MPU_RGD_WORD2_M1SM_SHIFT       (9)       /* Bits 9-10: Bus master 1 supervisor mode access control */
#define MPU_RGD_WORD2_M1SM_MASK        (3 << MPU_RGD_WORD2_M1SM_SHIFT)
#define MPU_RGD_WORD2_M1PE             (1 << 11) /* Bit 11: Bus Master 1 Process Identifier Enable */
#define MPU_RGD_WORD2_M2UM_SHIFT       (12)      /* Bits 12-14: Bus master 2 user mode access control */
#define MPU_RGD_WORD2_M2UM_MASK        (7 << MPU_RGD_WORD2_M2UM_SHIFT)
#define MPU_RGD_WORD2_M2SM_SHIFT       (15)      /* Bits 15-16: Bus master 2 supervisor mode access control */
#define MPU_RGD_WORD2_M2SM_MASK        (3 << MPU_RGD_WORD2_M2SM_SHIFT)
#define MPU_RGD_WORD2_M2PE             (1 << 17) /* Bit 17: Bus Master 2 Process Identifier Enable */
#define MPU_RGD_WORD2_M3UM_SHIFT       (18)      /* Bits 18-20: Bus master 3 user mode access control */
#define MPU_RGD_WORD2_M3UM_MASK        (7 << MPU_RGD_WORD2_M3UM_SHIFT)
#define MPU_RGD_WORD2_M3SM_SHIFT       (21)      /* Bits 21-22: Bus master 3 supervisor mode access control */
#define MPU_RGD_WORD2_M3SM_MASK        (3 << MPU_RGD_WORD2_M3SM_SHIFT)
#define MPU_RGD_WORD2_M3PE             (1 << 23) /* Bit 23: Bus Master 3 Process Identifier Enable */
#define MPU_RGD_WORD2_M4WE             (1 << 24) /* Bit 24: Bus master 4 write enable */
#define MPU_RGD_WORD2_M4RE             (1 << 25) /* Bit 25: Bus master 4 read enable */
#define MPU_RGD_WORD2_M5WE             (1 << 26) /* Bit 26: Bus master 5 write enable */
#define MPU_RGD_WORD2_M5RE             (1 << 27) /* Bit 27: Bus master 5 read enable */
#define MPU_RGD_WORD2_M6WE             (1 << 28) /* Bit 28: Bus master 6 write enable */
#define MPU_RGD_WORD2_M6RE             (1 << 29) /* Bit 29: Bus master 6 read enable */
#define MPU_RGD_WORD2_M7WE             (1 << 30) /* Bit 30: Bus master 7 write enable */
#define MPU_RGD_WORD2_M7RE             (1 << 31) /* Bit 31: Bus master 7 read enable */

/* Region Descriptor n, Word 3 */

#define MPU_RGD_WORD3_VLD              (1 << 0)  /* Bit 0:  Valid */
                                                 /* Bits 1-15: Reserved */
#if defined(KINETIS_K28)
#  define MPU_EDR_PIDMASK_SHIFT        (16)      /* Bits 16-23: Process Identifier Mask */
#  define MPU_EDR_PIDMASK_MASK         (0xff << MPU_EDR_EPID_SHIFT)
#  define MPU_EDR_PID_SHIFT            (24)      /* Bits 24-31: Process Identifier */
#  define MPU_EDR_PID_MASK             (0xff << MPU_EDR_EPID_SHIFT)
#endif

/* Region Descriptor Alternate Access Control n */

#define MPU_RGD_RBDACC_M0UM_SHIFT      (0)       /* Bits 0-2: Bus master 0 user mode access control */
#define MPU_RGD_RBDACC_M0UM_MASK       (7 << MPU_RGD_RBDACC_M0UM_SHIFT)
#define MPU_RGD_RBDACC_M0SM_SHIFT      (3)       /* Bits 3-4: Bus master 0 supervisor mode access control */
#define MPU_RGD_RBDACC_M0SM_MASK       (3 << MPU_RGD_RBDACC_M0SM_SHIFT)
#if defined(KINETIS_K28)
#  define MPU_RGD_RBDACC_M0PE          (1 << 5)  /* Bit 5: Bus Master 0 Process Identifier Enable */
#endif
#define MPU_RGD_RBDACC_M1UM_SHIFT      (6)       /* Bits 6-8: Bus master 1 user mode access control */
#define MPU_RGD_RBDACC_M1UM_MASK       (7 << MPU_RGD_RBDACC_M1UM_SHIFT)
#define MPU_RGD_RBDACC_M1SM_SHIFT      (9)       /* Bits 9-10: Bus master 1 supervisor mode access control */
#define MPU_RGD_RBDACC_M1SM_MASK       (3 << MPU_RGD_RBDACC_M1SM_SHIFT)
#if defined(KINETIS_K28)
#  define MPU_RGD_RBDACC_M1PE          (1 << 11) /* Bit 11: Bus Master 1 Process Identifier Enable */
#endif
#define MPU_RGD_RBDACC_M2UM_SHIFT      (12)      /* Bits 12-14: Bus master 2 user mode access control */
#define MPU_RGD_RBDACC_M2UM_MASK       (7 << MPU_RGD_RBDACC_M2UM_SHIFT)
#define MPU_RGD_RBDACC_M2SM_SHIFT      (15)      /* Bits 15-16: Bus master 2 supervisor mode access control */
#define MPU_RGD_RBDACC_M2SM_MASK       (3 << MPU_RGD_RBDACC_M2SM_SHIFT)
#if defined(KINETIS_K28)
#  define MPU_RGD_RBDACC_M2PE          (1 << 17)  /* Bit 17: Bus Master 2 Process Identifier Enable */
#endif
#define MPU_RGD_RBDACC_M3UM_SHIFT      (18)      /* Bits 18-20: Bus master 3 user mode access control */
#define MPU_RGD_RBDACC_M3UM_MASK       (7 << MPU_RGD_RBDACC_M3UM_SHIFT)
#define MPU_RGD_RBDACC_M3SM_SHIFT      (21)      /* Bits 21-22: Bus master 3 supervisor mode access control */
#define MPU_RGD_RBDACC_M3SM_MASK       (3 << MPU_RGD_RBDACC_M3SM_SHIFT)
#if defined(KINETIS_K28)
#  define MPU_RGD_RBDACC_M3PE          (1 << 23) /* Bit 23: Bus Master 3 Process Identifier Enable */
#endif
#define MPU_RGD_RBDACC_M4WE            (1 << 24) /* Bit 24: Bus master 4 write enable */
#define MPU_RGD_RBDACC_M4RE            (1 << 25) /* Bit 25: Bus master 4 read enable */
#define MPU_RGD_RBDACC_M5WE            (1 << 26) /* Bit 26: Bus master 5 write enable */
#define MPU_RGD_RBDACC_M5RE            (1 << 27) /* Bit 27: Bus master 5 read enable */
#define MPU_RGD_RBDACC_M6WE            (1 << 28) /* Bit 28: Bus master 6 write enable */
#define MPU_RGD_RBDACC_M6RE            (1 << 29) /* Bit 29: Bus master 6 read enable */
#define MPU_RGD_RBDACC_M7WE            (1 << 30) /* Bit 30: Bus master 7 write enable */
#define MPU_RGD_RBDACC_M7RE            (1 << 31) /* Bit 31: Bus master 7 read enable */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_K28K64K66MPU_H */
