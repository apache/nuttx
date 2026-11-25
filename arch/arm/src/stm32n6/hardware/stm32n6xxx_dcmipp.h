/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_dcmipp.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_DCMIPP_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_DCMIPP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Common registers */

#define STM32_DCMIPP_PRCR_OFFSET       0x0104  /* Parallel interface control */
#define STM32_DCMIPP_CMCR_OFFSET       0x0204  /* Common configuration */
#define STM32_DCMIPP_CMIER_OFFSET      0x03f0  /* Common interrupt enable */
#define STM32_DCMIPP_CMSR1_OFFSET      0x03f4  /* Common status register 1 */
#define STM32_DCMIPP_CMSR2_OFFSET      0x03f8  /* Common status register 2 */
#define STM32_DCMIPP_CMFCR_OFFSET      0x03fc  /* Common interrupt clear */

/* Pipe1 (main pipe with ISP) registers */

#define STM32_DCMIPP_P1FSCR_OFFSET     0x0804  /* Flow selection config */
#define STM32_DCMIPP_P1BLCCR_OFFSET    0x0840  /* Black level calibration */
#define STM32_DCMIPP_P1EXCR1_OFFSET    0x0844  /* Exposure control 1 */
#define STM32_DCMIPP_P1EXCR2_OFFSET    0x0848  /* Exposure control 2 */
#define STM32_DCMIPP_P1ST1CR_OFFSET    0x0850  /* Statistics 1 control */
#define STM32_DCMIPP_P1ST2CR_OFFSET    0x0854  /* Statistics 2 control */
#define STM32_DCMIPP_P1ST3CR_OFFSET    0x0858  /* Statistics 3 control */
#define STM32_DCMIPP_P1STSTR_OFFSET    0x085c  /* Statistics window start */
#define STM32_DCMIPP_P1STSZR_OFFSET    0x0860  /* Statistics window size */
#define STM32_DCMIPP_P1ST1SR_OFFSET    0x0864  /* Statistics 1 status */
#define STM32_DCMIPP_P1ST2SR_OFFSET    0x0868  /* Statistics 2 status */
#define STM32_DCMIPP_P1ST3SR_OFFSET    0x086c  /* Statistics 3 status */
#define STM32_DCMIPP_P1DMCR_OFFSET     0x0870  /* Demosaicing configuration */
#define STM32_DCMIPP_P1CCCR_OFFSET     0x0880  /* ColorConv configuration */
#define STM32_DCMIPP_P1CCRR1_OFFSET    0x0884  /* ColorConv red coeff 1 */
#define STM32_DCMIPP_P1CCRR2_OFFSET    0x0888  /* ColorConv red coeff 2 */
#define STM32_DCMIPP_P1CCGR1_OFFSET    0x088c  /* ColorConv green coeff 1 */
#define STM32_DCMIPP_P1CCGR2_OFFSET    0x0890  /* ColorConv green coeff 2 */
#define STM32_DCMIPP_P1CCBR1_OFFSET    0x0894  /* ColorConv blue coeff 1 */
#define STM32_DCMIPP_P1CCBR2_OFFSET    0x0898  /* ColorConv blue coeff 2 */
#define STM32_DCMIPP_P1FCTCR_OFFSET    0x0900  /* Flow control config */
#define STM32_DCMIPP_P1CRSTR_OFFSET    0x0904  /* Crop window start */
#define STM32_DCMIPP_P1CRSZR_OFFSET    0x0908  /* Crop window size */
#define STM32_DCMIPP_P1DCCR_OFFSET     0x090c  /* Decimation */
#define STM32_DCMIPP_P1DSCR_OFFSET     0x0910  /* Downsize configuration */
#define STM32_DCMIPP_P1DSRTIOR_OFFSET  0x0914  /* Downsize ratio */
#define STM32_DCMIPP_P1DSSZR_OFFSET    0x0918  /* Downsize destination size */
#define STM32_DCMIPP_P1GMCR_OFFSET     0x0970  /* Gamma configuration */
#define STM32_DCMIPP_P1PPCR_OFFSET     0x09c0  /* Pixel packer config */
#define STM32_DCMIPP_P1PPM0AR1_OFFSET  0x09c4  /* Memory0 address 1 */
#define STM32_DCMIPP_P1PPM0PR_OFFSET   0x09cc  /* Memory0 pitch */

/* Register Addresses *******************************************************/

#define STM32_DCMIPP_PRCR       (STM32_DCMIPP_BASE + STM32_DCMIPP_PRCR_OFFSET)
#define STM32_DCMIPP_CMCR       (STM32_DCMIPP_BASE + STM32_DCMIPP_CMCR_OFFSET)
#define STM32_DCMIPP_CMIER      (STM32_DCMIPP_BASE + STM32_DCMIPP_CMIER_OFFSET)
#define STM32_DCMIPP_CMSR1      (STM32_DCMIPP_BASE + STM32_DCMIPP_CMSR1_OFFSET)
#define STM32_DCMIPP_CMSR2      (STM32_DCMIPP_BASE + STM32_DCMIPP_CMSR2_OFFSET)
#define STM32_DCMIPP_CMFCR      (STM32_DCMIPP_BASE + STM32_DCMIPP_CMFCR_OFFSET)

#define STM32_DCMIPP_P1FSCR     (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1FSCR_OFFSET)
#define STM32_DCMIPP_P1BLCCR    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1BLCCR_OFFSET)
#define STM32_DCMIPP_P1EXCR1    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1EXCR1_OFFSET)
#define STM32_DCMIPP_P1EXCR2    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1EXCR2_OFFSET)
#define STM32_DCMIPP_P1ST1CR    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1ST1CR_OFFSET)
#define STM32_DCMIPP_P1ST2CR    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1ST2CR_OFFSET)
#define STM32_DCMIPP_P1ST3CR    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1ST3CR_OFFSET)
#define STM32_DCMIPP_P1STSTR    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1STSTR_OFFSET)
#define STM32_DCMIPP_P1STSZR    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1STSZR_OFFSET)
#define STM32_DCMIPP_P1ST1SR    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1ST1SR_OFFSET)
#define STM32_DCMIPP_P1ST2SR    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1ST2SR_OFFSET)
#define STM32_DCMIPP_P1ST3SR    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1ST3SR_OFFSET)
#define STM32_DCMIPP_P1DMCR     (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1DMCR_OFFSET)
#define STM32_DCMIPP_P1CCCR     (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1CCCR_OFFSET)
#define STM32_DCMIPP_P1CCRR1    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1CCRR1_OFFSET)
#define STM32_DCMIPP_P1CCRR2    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1CCRR2_OFFSET)
#define STM32_DCMIPP_P1CCGR1    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1CCGR1_OFFSET)
#define STM32_DCMIPP_P1CCGR2    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1CCGR2_OFFSET)
#define STM32_DCMIPP_P1CCBR1    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1CCBR1_OFFSET)
#define STM32_DCMIPP_P1CCBR2    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1CCBR2_OFFSET)
#define STM32_DCMIPP_P1FCTCR    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1FCTCR_OFFSET)
#define STM32_DCMIPP_P1CRSTR    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1CRSTR_OFFSET)
#define STM32_DCMIPP_P1CRSZR    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1CRSZR_OFFSET)
#define STM32_DCMIPP_P1DCCR     (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1DCCR_OFFSET)
#define STM32_DCMIPP_P1DSCR     (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1DSCR_OFFSET)
#define STM32_DCMIPP_P1DSRTIOR  (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1DSRTIOR_OFFSET)
#define STM32_DCMIPP_P1DSSZR    (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1DSSZR_OFFSET)
#define STM32_DCMIPP_P1GMCR     (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1GMCR_OFFSET)
#define STM32_DCMIPP_P1PPCR     (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1PPCR_OFFSET)
#define STM32_DCMIPP_P1PPM0AR1  (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1PPM0AR1_OFFSET)
#define STM32_DCMIPP_P1PPM0PR   (STM32_DCMIPP_BASE + \
                                 STM32_DCMIPP_P1PPM0PR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Parallel interface control register */

#define DCMIPP_PRCR_ENABLE         (1 << 14) /* Bit 14: Parallel interface
                                              * enable */

/* Common configuration register */

#define DCMIPP_CMCR_INSEL          (1 << 0)  /* Bit 0: Input from CSI-2 host */

/* Common interrupt enable, status 2 and clear registers (pipe1 and AXI) */

#define DCMIPP_CMINT_ATXERR        (1 << 5)  /* Bit 5: AXI transfer error */
#define DCMIPP_CMINT_P1LINE        (1 << 16) /* Bit 16: Pipe1 multi-line */
#define DCMIPP_CMINT_P1FRAME       (1 << 17) /* Bit 17: Pipe1 frame done */
#define DCMIPP_CMINT_P1VSYNC       (1 << 18) /* Bit 18: Pipe1 vertical sync */
#define DCMIPP_CMINT_P1OVR         (1 << 23) /* Bit 23: Pipe1 overrun */

/* Common status register 1 */

#define DCMIPP_CMSR1_P1CPTACT      (1 << 23) /* Bit 23: Pipe1 capture active */

/* Pipe1 flow selection configuration register */

#define DCMIPP_P1FSCR_DTIDA_SHIFT  (0)       /* Bits 0-5: Data type ID A */
#define DCMIPP_P1FSCR_DTIDA_MASK   (0x3f << DCMIPP_P1FSCR_DTIDA_SHIFT)
#  define DCMIPP_P1FSCR_DTIDA(n)   ((uint32_t)(n) << DCMIPP_P1FSCR_DTIDA_SHIFT)
#define DCMIPP_P1FSCR_DTIDB_SHIFT  (8)       /* Bits 8-13: Data type ID B */
#define DCMIPP_P1FSCR_DTIDB_MASK   (0x3f << DCMIPP_P1FSCR_DTIDB_SHIFT)
#define DCMIPP_P1FSCR_DTMODE_SHIFT (16)      /* Bits 16-17: Data type mode */
#define DCMIPP_P1FSCR_DTMODE_MASK  (0x3 << DCMIPP_P1FSCR_DTMODE_SHIFT)
#  define DCMIPP_P1FSCR_DTMODE_DTIDA (0 << DCMIPP_P1FSCR_DTMODE_SHIFT)
#define DCMIPP_P1FSCR_PIPEDIFF     (1 << 18) /* Bit 18: Separate pipe2 flow */
#define DCMIPP_P1FSCR_VC_SHIFT     (19)      /* Bits 19-20: Virtual channel */
#define DCMIPP_P1FSCR_VC_MASK      (0x3 << DCMIPP_P1FSCR_VC_SHIFT)
#  define DCMIPP_P1FSCR_VC(n)      ((uint32_t)(n) << DCMIPP_P1FSCR_VC_SHIFT)
#define DCMIPP_P1FSCR_PIPEN        (1 << 31) /* Bit 31: Pipe enable */

/* CSI-2 data types */

#define DCMIPP_DT_RAW8             0x2a      /* RAW8 Bayer */
#define DCMIPP_DT_RAW10            0x2b      /* RAW10 Bayer */
#define DCMIPP_DT_RAW12            0x2c      /* RAW12 Bayer */

/* Pipe1 black level calibration control register */

#define DCMIPP_P1BLCCR_ENABLE      (1 << 0)  /* Bit 0: Enable */
#define DCMIPP_P1BLCCR_BLCB_SHIFT  (8)       /* Bits 8-15: Blue black level */
#define DCMIPP_P1BLCCR_BLCG_SHIFT  (16)      /* Bits 16-23: Green black level */
#define DCMIPP_P1BLCCR_BLCR_SHIFT  (24)      /* Bits 24-31: Red black level */

/* Pipe1 exposure control registers (gain = MULT * 2^SHF / 128) */

#define DCMIPP_P1EXCR1_ENABLE      (1 << 0)  /* Bit 0: Enable */
#define DCMIPP_P1EXCR1_MULTR_SHIFT (20)      /* Bits 20-27: Red multiplier */
#define DCMIPP_P1EXCR1_SHFR_SHIFT  (28)      /* Bits 28-30: Red shift */
#define DCMIPP_P1EXCR2_MULTB_SHIFT (4)       /* Bits 4-11: Blue multiplier */
#define DCMIPP_P1EXCR2_SHFB_SHIFT  (12)      /* Bits 12-14: Blue shift */
#define DCMIPP_P1EXCR2_MULTG_SHIFT (20)      /* Bits 20-27: Green multiplier */
#define DCMIPP_P1EXCR2_SHFG_SHIFT  (28)      /* Bits 28-30: Green shift */

/* Pipe1 statistics 1-3 control registers */

#define DCMIPP_P1STCR_ENABLE       (1 << 0)  /* Bit 0: Enable */
#define DCMIPP_P1STCR_BINS_SHIFT   (2)       /* Bits 2-3: Bins or average
                                              * range */
#define DCMIPP_P1STCR_BINS_MASK    (0x3 << DCMIPP_P1STCR_BINS_SHIFT)
#  define DCMIPP_P1STCR_AVER_ALL   (0 << DCMIPP_P1STCR_BINS_SHIFT)
#define DCMIPP_P1STCR_SRC_SHIFT    (4)       /* Bits 4-6: Source */
#define DCMIPP_P1STCR_SRC_MASK     (0x7 << DCMIPP_P1STCR_SRC_SHIFT)
#  define DCMIPP_P1STCR_SRC_PRER   (0 << DCMIPP_P1STCR_SRC_SHIFT)
#  define DCMIPP_P1STCR_SRC_PREG   (1 << DCMIPP_P1STCR_SRC_SHIFT)
#  define DCMIPP_P1STCR_SRC_PREB   (2 << DCMIPP_P1STCR_SRC_SHIFT)
#  define DCMIPP_P1STCR_SRC_PREL   (3 << DCMIPP_P1STCR_SRC_SHIFT)
#  define DCMIPP_P1STCR_SRC_POSTR  (4 << DCMIPP_P1STCR_SRC_SHIFT)
#  define DCMIPP_P1STCR_SRC_POSTG  (5 << DCMIPP_P1STCR_SRC_SHIFT)
#  define DCMIPP_P1STCR_SRC_POSTB  (6 << DCMIPP_P1STCR_SRC_SHIFT)
#  define DCMIPP_P1STCR_SRC_POSTL  (7 << DCMIPP_P1STCR_SRC_SHIFT)
#define DCMIPP_P1STCR_MODE_BINS    (1 << 7)  /* Bit 7: Bins, not average */

/* Pipe1 statistics window start and size registers */

#define DCMIPP_P1STSTR_HSTART_SHIFT (0)      /* Bits 0-11: Horizontal start */
#define DCMIPP_P1STSTR_VSTART_SHIFT (16)     /* Bits 16-27: Vertical start */
#define DCMIPP_P1STSZR_HSIZE_SHIFT (0)       /* Bits 0-11: Horizontal size */
#define DCMIPP_P1STSZR_VSIZE_SHIFT (16)      /* Bits 16-27: Vertical size */
#define DCMIPP_P1STSZR_CROPEN      (1 << 31) /* Bit 31: Window enable */

/* Pipe1 statistics 1-3 status registers (accumulation divided by 256) */

#define DCMIPP_P1STSR_ACCU_MASK    (0xffffff) /* Bits 0-23: Accumulation */

/* Pipe1 demosaicing configuration register */

#define DCMIPP_P1DMCR_ENABLE       (1 << 0)  /* Bit 0: Enable */
#define DCMIPP_P1DMCR_TYPE_SHIFT   (1)       /* Bits 1-2: Raw Bayer type */
#define DCMIPP_P1DMCR_TYPE_MASK    (0x3 << DCMIPP_P1DMCR_TYPE_SHIFT)
#  define DCMIPP_P1DMCR_TYPE_RGGB  (0 << DCMIPP_P1DMCR_TYPE_SHIFT)
#  define DCMIPP_P1DMCR_TYPE_GRBG  (1 << DCMIPP_P1DMCR_TYPE_SHIFT)
#  define DCMIPP_P1DMCR_TYPE_GBRG  (2 << DCMIPP_P1DMCR_TYPE_SHIFT)
#  define DCMIPP_P1DMCR_TYPE_BGGR  (3 << DCMIPP_P1DMCR_TYPE_SHIFT)
#define DCMIPP_P1DMCR_PEAK_SHIFT   (16)      /* Bits 16-18: Peak strength */
#define DCMIPP_P1DMCR_LINEV_SHIFT  (20)      /* Bits 20-22: Vertical line
                                              * strength */
#define DCMIPP_P1DMCR_LINEH_SHIFT  (24)      /* Bits 24-26: Horizontal line
                                              * strength */
#define DCMIPP_P1DMCR_EDGE_SHIFT   (28)      /* Bits 28-30: Edge strength */

/* Pipe1 ColorConv configuration register */

#define DCMIPP_P1CCCR_ENABLE       (1 << 0)  /* Bit 0: Enable */

/* Pipe1 ColorConv coefficient registers: two 11-bit signed coefficients
 * (8 fractional bits) per register, or one coefficient and a 10-bit
 * offset in the second register of each row.
 */

#define DCMIPP_P1CC_LOW_SHIFT      (0)       /* Bits 0-10: First coefficient */
#define DCMIPP_P1CC_HIGH_SHIFT     (16)      /* Bits 16-26: Second
                                              * coefficient (RG, GG, BG) */
#define DCMIPP_P1CC_COEF_MASK      (0x7ff)

/* Pipe1 flow control configuration register */

#define DCMIPP_P1FCTCR_FRATE_SHIFT (0)       /* Bits 0-1: Frame capture rate */
#define DCMIPP_P1FCTCR_FRATE_MASK  (0x3 << DCMIPP_P1FCTCR_FRATE_SHIFT)
#define DCMIPP_P1FCTCR_CPTMODE     (1 << 2)  /* Bit 2: Snapshot mode */
#define DCMIPP_P1FCTCR_CPTREQ      (1 << 3)  /* Bit 3: Capture requested */

/* Pipe1 crop window start and size registers */

#define DCMIPP_P1CRSTR_HSTART_SHIFT (0)      /* Bits 0-11: Horizontal start */
#define DCMIPP_P1CRSTR_VSTART_SHIFT (16)     /* Bits 16-27: Vertical start */
#define DCMIPP_P1CRSZR_HSIZE_SHIFT (0)       /* Bits 0-11: Horizontal size */
#define DCMIPP_P1CRSZR_VSIZE_SHIFT (16)      /* Bits 16-27: Vertical size */
#define DCMIPP_P1CRSZR_ENABLE      (1 << 31) /* Bit 31: Crop enable */

/* Pipe1 decimation register (1, 1/2, 1/4 or 1/8) */

#define DCMIPP_P1DCCR_ENABLE       (1 << 0)  /* Bit 0: Enable */
#define DCMIPP_P1DCCR_HDEC_SHIFT   (1)       /* Bits 1-2: Horizontal
                                              * decimation */
#define DCMIPP_P1DCCR_HDEC_MASK    (0x3 << DCMIPP_P1DCCR_HDEC_SHIFT)
#  define DCMIPP_P1DCCR_HDEC(n)    ((uint32_t)(n) << DCMIPP_P1DCCR_HDEC_SHIFT)
#define DCMIPP_P1DCCR_VDEC_SHIFT   (3)       /* Bits 3-4: Vertical decimation */
#define DCMIPP_P1DCCR_VDEC_MASK    (0x3 << DCMIPP_P1DCCR_VDEC_SHIFT)
#  define DCMIPP_P1DCCR_VDEC(n)    ((uint32_t)(n) << DCMIPP_P1DCCR_VDEC_SHIFT)

/* Pipe1 downsize configuration, ratio and destination size registers */

#define DCMIPP_P1DSCR_HDIV_SHIFT   (0)       /* Bits 0-9: Horizontal division
                                              * factor (1024 * dst / src) */
#define DCMIPP_P1DSCR_VDIV_SHIFT   (16)      /* Bits 16-25: Vertical division
                                              * factor */
#define DCMIPP_P1DSCR_ENABLE       (1 << 31) /* Bit 31: Downsize enable */
#define DCMIPP_P1DSRTIOR_HRATIO_SHIFT (0)    /* Bits 0-15: Horizontal ratio
                                              * (8192 * src / dst) */
#define DCMIPP_P1DSRTIOR_VRATIO_SHIFT (16)   /* Bits 16-31: Vertical ratio */
#define DCMIPP_P1DSSZR_HSIZE_SHIFT (0)       /* Bits 0-11: Output width */
#define DCMIPP_P1DSSZR_VSIZE_SHIFT (16)      /* Bits 16-27: Output height */

/* Pipe1 gamma configuration register */

#define DCMIPP_P1GMCR_ENABLE       (1 << 0)  /* Bit 0: Gamma enable */

/* Pipe1 pixel packer configuration register */

#define DCMIPP_P1PPCR_FORMAT_SHIFT (0)       /* Bits 0-3: Output format */
#define DCMIPP_P1PPCR_FORMAT_MASK  (0xf << DCMIPP_P1PPCR_FORMAT_SHIFT)
#  define DCMIPP_P1PPCR_FORMAT_RGB888 (0 << DCMIPP_P1PPCR_FORMAT_SHIFT)
#  define DCMIPP_P1PPCR_FORMAT_RGB565 (1 << DCMIPP_P1PPCR_FORMAT_SHIFT)
#define DCMIPP_P1PPCR_SWAPRB       (1 << 4)  /* Bit 4: Swap red and blue */
#define DCMIPP_P1PPCR_DBM          (1 << 16) /* Bit 16: Double buffer mode */

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_DCMIPP_H */
