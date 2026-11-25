/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_csi.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_CSI_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_CSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_CSI_CR_OFFSET        0x0000  /* Control register */
#define STM32_CSI_PCR_OFFSET       0x0004  /* D-PHY RX control register */
#define STM32_CSI_VC0CFGR1_OFFSET  0x0010  /* Virtual channel 0 config 1 */
#define STM32_CSI_LMCFGR_OFFSET    0x0070  /* Lane merger configuration */
#define STM32_CSI_IER0_OFFSET      0x0080  /* Interrupt enable register 0 */
#define STM32_CSI_IER1_OFFSET      0x0084  /* Interrupt enable register 1 */
#define STM32_CSI_SR0_OFFSET       0x0090  /* Status register 0 */
#define STM32_CSI_SR1_OFFSET       0x0094  /* Status register 1 */
#define STM32_CSI_FCR0_OFFSET      0x0100  /* Flag clear register 0 */
#define STM32_CSI_FCR1_OFFSET      0x0104  /* Flag clear register 1 */
#define STM32_CSI_PRCR_OFFSET      0x1000  /* PHY reset control register */
#define STM32_CSI_PMCR_OFFSET      0x1004  /* PHY mode control register */
#define STM32_CSI_PFCR_OFFSET      0x1008  /* PHY frequency control */
#define STM32_CSI_PTCR0_OFFSET     0x1010  /* PHY test control register 0 */
#define STM32_CSI_PTCR1_OFFSET     0x1014  /* PHY test control register 1 */

/* Register Addresses *******************************************************/

#define STM32_CSI_CR               (STM32_CSI_BASE + STM32_CSI_CR_OFFSET)
#define STM32_CSI_PCR              (STM32_CSI_BASE + STM32_CSI_PCR_OFFSET)
#define STM32_CSI_VC0CFGR1         (STM32_CSI_BASE + \
                                    STM32_CSI_VC0CFGR1_OFFSET)
#define STM32_CSI_LMCFGR           (STM32_CSI_BASE + STM32_CSI_LMCFGR_OFFSET)
#define STM32_CSI_IER0             (STM32_CSI_BASE + STM32_CSI_IER0_OFFSET)
#define STM32_CSI_IER1             (STM32_CSI_BASE + STM32_CSI_IER1_OFFSET)
#define STM32_CSI_SR0              (STM32_CSI_BASE + STM32_CSI_SR0_OFFSET)
#define STM32_CSI_SR1              (STM32_CSI_BASE + STM32_CSI_SR1_OFFSET)
#define STM32_CSI_FCR0             (STM32_CSI_BASE + STM32_CSI_FCR0_OFFSET)
#define STM32_CSI_FCR1             (STM32_CSI_BASE + STM32_CSI_FCR1_OFFSET)
#define STM32_CSI_PRCR             (STM32_CSI_BASE + STM32_CSI_PRCR_OFFSET)
#define STM32_CSI_PMCR             (STM32_CSI_BASE + STM32_CSI_PMCR_OFFSET)
#define STM32_CSI_PFCR             (STM32_CSI_BASE + STM32_CSI_PFCR_OFFSET)
#define STM32_CSI_PTCR0            (STM32_CSI_BASE + STM32_CSI_PTCR0_OFFSET)
#define STM32_CSI_PTCR1            (STM32_CSI_BASE + STM32_CSI_PTCR1_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Control register */

#define CSI_CR_CSIEN               (1 << 0)  /* Bit 0: CSI-2 host enable */
#define CSI_CR_VC0START            (1 << 2)  /* Bit 2: Start virtual channel 0 */
#define CSI_CR_VC0STOP             (1 << 3)  /* Bit 3: Stop virtual channel 0 */

/* D-PHY RX control register */

#define CSI_PCR_NPWRDOWN           (1 << 0)  /* Bit 0: Not in power down */
#define CSI_PCR_CLEN               (1 << 1)  /* Bit 1: Clock lane enable */
#define CSI_PCR_DL0EN              (1 << 2)  /* Bit 2: Data lane 0 enable */
#define CSI_PCR_DL1EN              (1 << 3)  /* Bit 3: Data lane 1 enable */

/* Virtual channel 0 configuration register 1 */

#define CSI_VC0CFGR1_ALLDT         (1 << 0)  /* Bit 0: Accept all data types */
#define CSI_VC0CFGR1_CDTFT_SHIFT   (8)       /* Bits 8-12: Common data type
                                              * format */
#define CSI_VC0CFGR1_CDTFT_MASK    (0x1f << CSI_VC0CFGR1_CDTFT_SHIFT)
#  define CSI_VC0CFGR1_CDTFT(n)    ((uint32_t)(n) << CSI_VC0CFGR1_CDTFT_SHIFT)

/* Data type formats (bits per pixel) for CDTFT */

#define CSI_DTFT_BPP8              2         /* 8 bits per pixel */
#define CSI_DTFT_BPP10             3         /* 10 bits per pixel */
#define CSI_DTFT_BPP12             4         /* 12 bits per pixel */
#define CSI_DTFT_BPP14             5         /* 14 bits per pixel */

/* Lane merger configuration register */

#define CSI_LMCFGR_LANENB_SHIFT    (8)       /* Bits 8-10: Number of data
                                              * lanes */
#define CSI_LMCFGR_LANENB_MASK     (0x7 << CSI_LMCFGR_LANENB_SHIFT)
#  define CSI_LMCFGR_LANENB(n)     ((uint32_t)(n) << CSI_LMCFGR_LANENB_SHIFT)
#define CSI_LMCFGR_DL0MAP_SHIFT    (16)      /* Bits 16-18: Physical lane of
                                              * logical lane 0 */
#define CSI_LMCFGR_DL0MAP_MASK     (0x7 << CSI_LMCFGR_DL0MAP_SHIFT)
#  define CSI_LMCFGR_DL0MAP(n)     ((uint32_t)(n) << CSI_LMCFGR_DL0MAP_SHIFT)
#define CSI_LMCFGR_DL1MAP_SHIFT    (20)      /* Bits 20-22: Physical lane of
                                              * logical lane 1 */
#define CSI_LMCFGR_DL1MAP_MASK     (0x7 << CSI_LMCFGR_DL1MAP_SHIFT)
#  define CSI_LMCFGR_DL1MAP(n)     ((uint32_t)(n) << CSI_LMCFGR_DL1MAP_SHIFT)

/* Interrupt enable register 0, status register 0 and flag clear register 0
 * (same bit positions)
 */

#define CSI_INT0_SOF0              (1 << 8)  /* Bit 8: Start of frame on VC0 */
#define CSI_INT0_EOF0              (1 << 12) /* Bit 12: End of frame on VC0 */
#define CSI_INT0_SPKT              (1 << 16) /* Bit 16: Short packet */
#define CSI_SR0_VC0STATEF          (1 << 17) /* Bit 17: VC0 running (SR0) */
#define CSI_INT0_CCFIFOF           (1 << 21) /* Bit 21: Clock changer FIFO
                                              * full */
#define CSI_INT0_CRCERR            (1 << 24) /* Bit 24: CRC error */
#define CSI_INT0_ECCERR            (1 << 25) /* Bit 25: ECC error */
#define CSI_INT0_CECCERR           (1 << 26) /* Bit 26: Corrected ECC error */
#define CSI_INT0_IDERR             (1 << 27) /* Bit 27: Data ID error */
#define CSI_INT0_SPKTERR           (1 << 28) /* Bit 28: Short packet error */
#define CSI_INT0_WDERR             (1 << 29) /* Bit 29: Watchdog error */
#define CSI_INT0_SYNCERR           (1 << 30) /* Bit 30: Synchronization
                                              * error */

/* Interrupt enable register 1, status register 1 and flag clear register 1:
 * D-PHY data lane n errors (n = 0 or 1)
 */

#define CSI_INT1_ESOTDL(n)         (1 << (0 + 8 * (n))) /* SOT error */
#define CSI_INT1_ESOTSYNCDL(n)     (1 << (1 + 8 * (n))) /* SOT sync error */
#define CSI_INT1_EESCDL(n)         (1 << (2 + 8 * (n))) /* Escape entry
                                                         * error */
#define CSI_INT1_ESYNCESCDL(n)     (1 << (3 + 8 * (n))) /* LP sync error */
#define CSI_INT1_ECTRLDL(n)        (1 << (4 + 8 * (n))) /* Control error */

/* PHY reset control register */

#define CSI_PRCR_PEN               (1 << 1)  /* Bit 1: D-PHY out of reset */

/* PHY frequency control register */

#define CSI_PFCR_CCFR_SHIFT        (0)       /* Bits 0-5: Configuration clock
                                              * frequency range */
#define CSI_PFCR_CCFR_MASK         (0x3f << CSI_PFCR_CCFR_SHIFT)
#  define CSI_PFCR_CCFR(n)         ((uint32_t)(n) << CSI_PFCR_CCFR_SHIFT)
#define CSI_PFCR_HSFR_SHIFT        (8)       /* Bits 8-14: High-speed
                                              * frequency range */
#define CSI_PFCR_HSFR_MASK         (0x7f << CSI_PFCR_HSFR_SHIFT)
#  define CSI_PFCR_HSFR(n)         ((uint32_t)(n) << CSI_PFCR_HSFR_SHIFT)
#define CSI_PFCR_DLD               (1 << 16) /* Bit 16: Data lane direction */

/* PHY test control registers */

#define CSI_PTCR0_TCKEN            (1 << 0)  /* Bit 0: Test clock enable */
#define CSI_PTCR0_TRSEN            (1 << 1)  /* Bit 1: Test interface reset */
#define CSI_PTCR1_TDI_SHIFT        (0)       /* Bits 0-7: Test data input */
#define CSI_PTCR1_TDI_MASK         (0xff << CSI_PTCR1_TDI_SHIFT)
#  define CSI_PTCR1_TDI(n)         ((uint32_t)(n) << CSI_PTCR1_TDI_SHIFT)
#define CSI_PTCR1_TWM              (1 << 16) /* Bit 16: Test write mode */

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_CSI_H */
