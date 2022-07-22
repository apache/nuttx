/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_eim.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_EIM_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_EIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EIM Register Offsets *****************************************************/

#define S32K3XX_EIM_EIMCR_OFFSET          (0x0000) /* Error Injection Module Configuration Register (EIMCR) */
#define S32K3XX_EIM_EICHEN_OFFSET         (0x0004) /* Error Injection Channel Enable Register (EICHEN) */

/* Note:  Not all Error Injection Channel Descriptors consist of 6 words.
 * See the S32K3xx Reference Manual for more information and to check which
 * registers are available.
 */

#define S32K3XX_EIM_EICHD_WORD0_OFFSET(n) (0x0100 + ((n) << 6)) /* Error Injection Channel Descriptor n, Word 0 (EICHDn_WORD0) */
#define S32K3XX_EIM_EICHD_WORD1_OFFSET(n) (0x0104 + ((n) << 6)) /* Error Injection Channel Descriptor n, Word 1 (EICHDn_WORD1) */
#define S32K3XX_EIM_EICHD_WORD2_OFFSET(n) (0x0108 + ((n) << 6)) /* Error Injection Channel Descriptor n, Word 2 (EICHDn_WORD2) */
#define S32K3XX_EIM_EICHD_WORD3_OFFSET(n) (0x010c + ((n) << 6)) /* Error Injection Channel Descriptor n, Word 3 (EICHDn_WORD3) */
#define S32K3XX_EIM_EICHD_WORD4_OFFSET(n) (0x0110 + ((n) << 6)) /* Error Injection Channel Descriptor n, Word 4 (EICHDn_WORD4) */
#define S32K3XX_EIM_EICHD_WORD5_OFFSET(n) (0x0114 + ((n) << 6)) /* Error Injection Channel Descriptor n, Word 5 (EICHDn_WORD5) */
#define S32K3XX_EIM_EICHD_WORD6_OFFSET(n) (0x0118 + ((n) << 6)) /* Error Injection Channel Descriptor n, Word 6 (EICHDn_WORD6) */

/* EIM Register Addresses ***************************************************/

#define S32K3XX_EIM_EIMCR                 (S32K3XX_EIM_BASE + S32K3XX_EIM_EIMCR_OFFSET)
#define S32K3XX_EIM_EICHEN                (S32K3XX_EIM_BASE + S32K3XX_EIM_EICHEN_OFFSET)
#define S32K3XX_EIM_EICHD_WORD0(n)        (S32K3XX_EIM_BASE + S32K3XX_EIM_EICHD_WORD0_OFFSET(n))
#define S32K3XX_EIM_EICHD_WORD1(n)        (S32K3XX_EIM_BASE + S32K3XX_EIM_EICHD_WORD1_OFFSET(n))
#define S32K3XX_EIM_EICHD_WORD2(n)        (S32K3XX_EIM_BASE + S32K3XX_EIM_EICHD_WORD2_OFFSET(n))
#define S32K3XX_EIM_EICHD_WORD3(n)        (S32K3XX_EIM_BASE + S32K3XX_EIM_EICHD_WORD3_OFFSET(n))
#define S32K3XX_EIM_EICHD_WORD4(n)        (S32K3XX_EIM_BASE + S32K3XX_EIM_EICHD_WORD4_OFFSET(n))
#define S32K3XX_EIM_EICHD_WORD5(n)        (S32K3XX_EIM_BASE + S32K3XX_EIM_EICHD_WORD5_OFFSET(n))
#define S32K3XX_EIM_EICHD_WORD6(n)        (S32K3XX_EIM_BASE + S32K3XX_EIM_EICHD_WORD6_OFFSET(n))

/* EIM Register Bitfield Definitions ****************************************/

/* Error Injection Module Configuration Register (EIMCR) */

#define EIM_EIMCR_GEIEN                   (1 << 0)  /* Bit 0: Global Error Injection Enable (GEIEN) */
                                                    /* Bits 1-31: Reserved */

/* Error Injection Channel Enable Register (EICHEN) */

                                                    /* Bit 0: Reserved */
#define EIM_EICHEN_EICH30EN               (1 << 1)  /* Bit 1: Error Injection Channel 30 Enable (EICH30EN) */
#define EIM_EICHEN_EICH29EN               (1 << 2)  /* Bit 2: Error Injection Channel 29 Enable (EICH29EN) */
#define EIM_EICHEN_EICH28EN               (1 << 3)  /* Bit 3: Error Injection Channel 28 Enable (EICH28EN) */
#define EIM_EICHEN_EICH27EN               (1 << 4)  /* Bit 4: Error Injection Channel 27 Enable (EICH27EN) */
#define EIM_EICHEN_EICH26EN               (1 << 5)  /* Bit 5: Error Injection Channel 26 Enable (EICH26EN) */
#define EIM_EICHEN_EICH25EN               (1 << 6)  /* Bit 6: Error Injection Channel 25 Enable (EICH25EN) */
#define EIM_EICHEN_EICH24EN               (1 << 7)  /* Bit 7: Error Injection Channel 24 Enable (EICH24EN) */
#define EIM_EICHEN_EICH23EN               (1 << 8)  /* Bit 8: Error Injection Channel 23 Enable (EICH23EN) */
#define EIM_EICHEN_EICH22EN               (1 << 9)  /* Bit 9: Error Injection Channel 22 Enable (EICH22EN) */
#define EIM_EICHEN_EICH21EN               (1 << 10) /* Bit 10: Error Injection Channel 21 Enable (EICH21EN) */
#define EIM_EICHEN_EICH20EN               (1 << 11) /* Bit 11: Error Injection Channel 20 Enable (EICH20EN) */
#define EIM_EICHEN_EICH19EN               (1 << 12) /* Bit 12: Error Injection Channel 19 Enable (EICH19EN) */
#define EIM_EICHEN_EICH18EN               (1 << 13) /* Bit 13: Error Injection Channel 18 Enable (EICH18EN) */
#define EIM_EICHEN_EICH17EN               (1 << 14) /* Bit 14: Error Injection Channel 17 Enable (EICH17EN) */
#define EIM_EICHEN_EICH16EN               (1 << 15) /* Bit 15: Error Injection Channel 16 Enable (EICH16EN) */
#define EIM_EICHEN_EICH15EN               (1 << 16) /* Bit 16: Error Injection Channel 15 Enable (EICH15EN) */
#define EIM_EICHEN_EICH14EN               (1 << 17) /* Bit 17: Error Injection Channel 14 Enable (EICH14EN) */
#define EIM_EICHEN_EICH13EN               (1 << 18) /* Bit 18: Error Injection Channel 13 Enable (EICH13EN) */
#define EIM_EICHEN_EICH12EN               (1 << 19) /* Bit 19: Error Injection Channel 12 Enable (EICH12EN) */
#define EIM_EICHEN_EICH11EN               (1 << 20) /* Bit 20: Error Injection Channel 11 Enable (EICH11EN) */
#define EIM_EICHEN_EICH10EN               (1 << 21) /* Bit 21: Error Injection Channel 10 Enable (EICH10EN) */
#define EIM_EICHEN_EICH9EN                (1 << 22) /* Bit 22: Error Injection Channel 9 Enable (EICH9EN) */
#define EIM_EICHEN_EICH8EN                (1 << 23) /* Bit 23: Error Injection Channel 8 Enable (EICH8EN) */
#define EIM_EICHEN_EICH7EN                (1 << 24) /* Bit 24: Error Injection Channel 7 Enable (EICH7EN) */
#define EIM_EICHEN_EICH6EN                (1 << 25) /* Bit 25: Error Injection Channel 6 Enable (EICH6EN) */
#define EIM_EICHEN_EICH5EN                (1 << 26) /* Bit 26: Error Injection Channel 5 Enable (EICH5EN) */
#define EIM_EICHEN_EICH4EN                (1 << 27) /* Bit 27: Error Injection Channel 4 Enable (EICH4EN) */
#define EIM_EICHEN_EICH3EN                (1 << 28) /* Bit 28: Error Injection Channel 3 Enable (EICH3EN) */
#define EIM_EICHEN_EICH2EN                (1 << 29) /* Bit 29: Error Injection Channel 2 Enable (EICH2EN) */
#define EIM_EICHEN_EICH1EN                (1 << 30) /* Bit 30: Error Injection Channel 1 Enable (EICH1EN) */
#define EIM_EICHEN_EICH0EN                (1 << 31) /* Bit 31: Error Injection Channel 0 Enable (EICH0EN) */

/* Error Injection Channel Descriptor n, Word 0 (EICHDn_WORD0, n=0,1,2) */

                                                    /* Bits 0-23: Reserved */
#define EIM_EICHD0_2_WORD0_CHKBIT_SHIFT   (24)      /* Bits 24-31: Checkbit Mask (CHKBIT_MASK) */
#define EIM_EICHD0_2_WORD0_CHKBIT_MASK    (0xff << EIM_EICHD0_2_WORD0_CHKBIT_SHIFT)

/* Error Injection Channel Descriptor 3, Word 0 (EICHD3_WORD0) */

                                                    /* Bits 0-17: Reserved */
#define EIM_EICHD3_WORD0_CHKBIT_SHIFT     (18)      /* Bits 18-31: Checkbit Mask (CHKBIT_MASK) */
#define EIM_EICHD3_WORD0_CHKBIT_MASK      (0x3fff << EIM_EICHD3_WORD0_CHKBIT_SHIFT)

/* Error Injection Channel Descriptor 4, Word 0 (EICHD4_WORD0) */

                                                    /* Bits 0-15: Reserved */
#define EIM_EICHD4_WORD0_CHKBIT_SHIFT     (16)      /* Bits 16-31: Checkbit Mask (CHKBIT_MASK) */
#define EIM_EICHD4_WORD0_CHKBIT_MASK      (0xffff << EIM_EICHD4_WORD0_CHKBIT_SHIFT)

/* Error Injection Channel Descriptor n, Word 0 (EICHDn_WORD0, n=5,6,7) */

                                                    /* Bits 0-3: Reserved */
#define EIM_EICHD5_7_WORD0_CHKBIT_SHIFT   (4)       /* Bits 4-31: Checkbit Mask (CHKBIT_MASK) */
#define EIM_EICHD5_7_WORD0_CHKBIT_MASK    (0x0fffffff << EIM_EICHD5_7_WORD0_CHKBIT_SHIFT)

/* Error Injection Channel Descriptor 8, Word 0 (EICHD8_WORD0) */

                                                    /* Bits 0-17: Reserved */
#define EIM_EICHD8_WORD0_CHKBIT_SHIFT     (18)      /* Bits 18-31: Checkbit Mask (CHKBIT_MASK) */
#define EIM_EICHD8_WORD0_CHKBIT_MASK      (0x3fff << EIM_EICHD8_WORD0_CHKBIT_SHIFT)

/* Error Injection Channel Descriptor 9, Word 0 (EICHD9_WORD0) */

                                                    /* Bits 0-15: Reserved */
#define EIM_EICHD9_WORD0_CHKBIT_SHIFT     (16)      /* Bits 16-31: Checkbit Mask (CHKBIT_MASK) */
#define EIM_EICHD9_WORD0_CHKBIT_MASK      (0xffff << EIM_EICHD9_WORD0_CHKBIT_SHIFT)

/* Error Injection Channel Descriptor n, Word 0 (EICHDn_WORD0, n=10,11,12) */

                                                    /* Bits 0-3: Reserved */
#define EIM_EICHD10_12_WORD0_CHKBIT_SHIFT (4)       /* Bits 4-31: Checkbit Mask (CHKBIT_MASK) */
#define EIM_EICHD10_12_WORD0_CHKBIT_MASK  (0x0fffffff << EIM_EICHD10_12_WORD0_CHKBIT_SHIFT)

/* Error Injection Channel Descriptor n, Word 0 (EICHDn_WORD0, n=13,...,18) */

                                                    /* Bits 0-23: Reserved */
#define EIM_EICHD13_18_WORD0_CHKBIT_SHIFT (24)      /* Bits 24-31: Checkbit Mask (CHKBIT_MASK) */
#define EIM_EICHD13_18_WORD0_CHKBIT_MASK  (0xff << EIM_EICHD13_18_WORD0_CHKBIT_SHIFT)

/* Error Injection Channel Descriptor n, Word 1 (EICHDn_WORD1, n=0,1,2) */

#define EIM_EICHD0_2_WORD1_B0_3DATA_SHIFT (0)       /* Bits 0-31: Data Mask Bytes 0-3 (B0_3DATA_MASK) */
#define EIM_EICHD0_2_WORD1_B0_3DATA_MASK  (0xffffffff << EIM_EICHD0_2_WORD1_B0_3DATA_SHIFT)

/* Error Injection Channel Descriptor 3, Word 1 (EICHD3_WORD1) */

#define EIM_EICHD3_WORD1_B0_3DATA_SHIFT   (0)       /* Bits 0-11: Data Mask Bytes 0-3 (B0_3DATA_MASK) */
#define EIM_EICHD3_WORD1_B0_3DATA_MASK    (0x0fff << EIM_EICHD3_WORD1_B0_3DATA_SHIFT)
                                                    /* Bits 12-31: Reserved */

/* Error Injection Channel Descriptor 4, Word 1 (EICHD4_WORD1) */

#define EIM_EICHD4_WORD1_B0_3DATA_SHIFT   (0)       /* Bits 0-31: Data Mask Bytes 0-3 (B0_3DATA_MASK) */
#define EIM_EICHD4_WORD1_B0_3DATA_MASK    (0xffffffff << EIM_EICHD4_WORD1_B0_3DATA_SHIFT)

/* Error Injection Channel Descriptor 5, Word 1 (EICHD5_WORD1) */

#define EIM_EICHD5_WORD1_B0_3DATA_SHIFT   (0)       /* Bits 0-7: Data Mask Bytes 0-3 (B0_3DATA_MASK) */
#define EIM_EICHD5_WORD1_B0_3DATA_MASK    (0xff << EIM_EICHD5_WORD1_B0_3DATA_SHIFT)
                                                    /* Bits 8-31: Reserved */

/* Error Injection Channel Descriptor n, Word 1 (EICHDn_WORD1, n=6,7) */

#define EIM_EICHD6_7_WORD1_B0_3DATA_SHIFT (0)       /* Bits 0-31: Data Mask Bytes 0-3 (B0_3DATA_MASK) */
#define EIM_EICHD6_7_WORD1_B0_3DATA_MASK  (0xffffffff << EIM_EICHD6_7_WORD1_B0_3DATA_SHIFT)

/* Error Injection Channel Descriptor 8, Word 1 (EICHD8_WORD1) */

#define EIM_EICHD8_WORD1_B0_3DATA_SHIFT   (0)       /* Bits 0-11: Data Mask Bytes 0-3 (B0_3DATA_MASK) */
#define EIM_EICHD8_WORD1_B0_3DATA_MASK    (0x0fff << EIM_EICHD8_WORD1_B0_3DATA_SHIFT)
                                                    /* Bits 12-31: Reserved */

/* Error Injection Channel Descriptor 9, Word 1 (EICHD9_WORD1) */

#define EIM_EICHD9_WORD1_B0_3DATA_SHIFT   (0)       /* Bits 0-31: Data Mask Bytes 0-3 (B0_3DATA_MASK) */
#define EIM_EICHD9_WORD1_B0_3DATA_MASK    (0xffffffff << EIM_EICHD9_WORD1_B0_3DATA_SHIFT)

/* Error Injection Channel Descriptor 10, Word 1 (EICHD10_WORD1) */

#define EIM_EICHD10_WORD1_B0_3DATA_SHIFT  (0)       /* Bits 0-7: Data Mask Bytes 0-3 (B0_3DATA_MASK) */
#define EIM_EICHD10_WORD1_B0_3DATA_MASK   (0xff << EIM_EICHD10_WORD1_B0_3DATA_SHIFT)
                                                    /* Bits 8-31: Reserved */

/* Error Injection Channel Descriptor n, Word 1 (EICHDn_WORD1, n=11,...,18) */

#define EIM_EICHD11_18_WORD1_B0_3DATA_SHIFT (0)     /* Bits 0-31: Data Mask Bytes 0-3 (B0_3DATA_MASK) */
#define EIM_EICHD11_18_WORD1_B0_3DATA_MASK  (0xffffffff << EIM_EICHD11_18_WORD1_B0_3DATA_SHIFT)

/* Error Injection Channel Descriptor n, Word 1 (EICHDn_WORD1, n=19,...,26) */

#define EIM_EICHD19_26_WORD1_B0_3DATA_SHIFT (0)     /* Bits 0-27: Data Mask Bytes 0-3 (B0_3DATA_MASK) */
#define EIM_EICHD19_26_WORD1_B0_3DATA_MASK  (0x0fffffff << EIM_EICHD19_26_WORD1_B0_3DATA_SHIFT)
                                                    /* Bits 28-31: Reserved */

/* Error Injection Channel Descriptor 27, Word 1 (EICHD27_WORD1) */

#define EIM_EICHD27_WORD1_B0_3DATA_SHIFT  (0)       /* Bits 0-29: Data Mask Bytes 0-3 (B0_3DATA_MASK) */
#define EIM_EICHD27_WORD1_B0_3DATA_MASK   (0x3fffffff << EIM_EICHD27_WORD1_B0_3DATA_SHIFT)
                                                    /* Bits 30-31: Reserved */

/* Error Injection Channel Descriptor 28, Word 1 (EICHD28_WORD1) */

#define EIM_EICHD28_WORD1_B0_3DATA_SHIFT  (0)       /* Bits 0-23: Data Mask Bytes 0-3 (B0_3DATA_MASK) */
#define EIM_EICHD28_WORD1_B0_3DATA_MASK   (0xffffff << EIM_EICHD28_WORD1_B0_3DATA_SHIFT)
                                                    /* Bits 24-31: Reserved */

/* Error Injection Channel Descriptor n, Word 1 (EICHDn_WORD1, n=29,30) */

#define EIM_EICHD29_30_WORD1_B0_3DATA_SHIFT (0)    /* Bits 0-17: Data Mask Bytes 0-3 (B0_3DATA_MASK) */
#define EIM_EICHD29_30_WORD1_B0_3DATA_MASK  (0x03ffff << EIM_EICHD29_30_WORD1_B0_3DATA_SHIFT)
                                                    /* Bits 18-31: Reserved */

/* Error Injection Channel Descriptor n, Word 2 (EICHDn_WORD2) */

#define EIM_EICHD_WORD2_B4_7DATA_SHIFT    (0)       /* Bits 0-31: Data Mask Bytes 4-7 (B4_7DATA_MASK) */
#define EIM_EICHD_WORD2_B4_7DATA_MASK     (0xffffffff << EIM_EICHD_WORD2_B4_7DATA_SHIFT)

/* Error Injection Channel Descriptor n, Word 3 (EICHDn_WORD3) */

#define EIM_EICHD_WORD3_B8_11DATA_SHIFT   (0)       /* Bits 0-31: Data Mask Bytes 8-11 (B8_11DATA_MASK) */
#define EIM_EICHD_WORD3_B8_11DATA_MASK    (0xffffffff << EIM_EICHD_WORD3_B8_11DATA_SHIFT)

/* Error Injection Channel Descriptor n, Word 4 (EICHDn_WORD4) */

#define EIM_EICHD_WORD4_B12_15DATA_SHIFT  (0)       /* Bits 0-31: Data Mask Bytes 12-15 (B12_15DATA_MASK) */
#define EIM_EICHD_WORD4_B12_15DATA_MASK   (0xffffffff << EIM_EICHD_WORD4_B12_15DATA_SHIFT)

/* Error Injection Channel Descriptor n, Word 5 (EICHDn_WORD5) */

#define EIM_EICHD_WORD5_B16_19DATA_SHIFT  (0)       /* Bits 0-31: Data Mask Bytes 16-19 (B16_19DATA_MASK) */
#define EIM_EICHD_WORD5_B16_19DATA_MASK   (0xffffffff << EIM_EICHD_WORD5_B16_19DATA_SHIFT)

/* Error Injection Channel Descriptor n, Word 6 (EICHDn_WORD6) */

#define EIM_EICHD_WORD6_B20_23DATA_SHIFT  (0)       /* Bits 0-31: Data Mask Bytes 20-23 (B20_23DATA_MASK) */
#define EIM_EICHD_WORD6_B20_23DATA_MASK   (0xffffffff << EIM_EICHD_WORD6_B20_23DATA_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_EIM_H */
