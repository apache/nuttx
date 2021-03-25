/****************************************************************************
 * arch/arm/src/kl/hardware/kl_mcg.h
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

#ifndef __ARCH_ARM_SRC_KL_HARDWARE_KL_MCG_H
#define __ARCH_ARM_SRC_KL_HARDWARE_KL_MCG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KL_MCG_C1_OFFSET          0x0000 /* MCG Control 1 Register */
#define KL_MCG_C2_OFFSET          0x0001 /* MCG Control 2 Register */
#define KL_MCG_C3_OFFSET          0x0002 /* MCG Control 3 Register */
#define KL_MCG_C4_OFFSET          0x0003 /* MCG Control 4 Register */
#define KL_MCG_C5_OFFSET          0x0004 /* MCG Control 5 Register */
#define KL_MCG_C6_OFFSET          0x0005 /* MCG Control 6 Register */
#define KL_MCG_S_OFFSET           0x0006 /* MCG Status Register */
#define KL_MCG_ATC_OFFSET         0x0008 /* MCG Auto Trim Control Register */
#define KL_MCG_ATCVH_OFFSET       0x000a /* MCG Auto Trim Compare Value High Register */
#define KL_MCG_ATCVL_OFFSET       0x000b /* MCG Auto Trim Compare Value Low Register */

/* Register Addresses *******************************************************/

#define KL_MCG_C1                 (KL_MCG_BASE+KL_MCG_C1_OFFSET)
#define KL_MCG_C2                 (KL_MCG_BASE+KL_MCG_C2_OFFSET)
#define KL_MCG_C3                 (KL_MCG_BASE+KL_MCG_C3_OFFSET)
#define KL_MCG_C4                 (KL_MCG_BASE+KL_MCG_C4_OFFSET)
#define KL_MCG_C5                 (KL_MCG_BASE+KL_MCG_C5_OFFSET)
#define KL_MCG_C6                 (KL_MCG_BASE+KL_MCG_C6_OFFSET)
#define KL_MCG_S                  (KL_MCG_BASE+KL_MCG_S_OFFSET)
#define KL_MCG_ATC                (KL_MCG_BASE+KL_MCG_ATC_OFFSET)
#define KL_MCG_ATCVH              (KL_MCG_BASE+KL_MCG_ATCVH_OFFSET)
#define KL_MCG_ATCVL              (KL_MCG_BASE+KL_MCG_ATCVL_OFFSET)

/* Register Bit Definitions *************************************************/

/* MCG Control 1 Register (8-bit) */

#define MCG_C1_IREFSTEN           (1 << 0)  /* Bit 0:  Internal Reference Stop Enable */
#define MCG_C1_IRCLKEN            (1 << 1)  /* Bit 1:  Internal Reference Clock Enable */
#define MCG_C1_IREFS              (1 << 2)  /* Bit 2:  Internal Reference Select */
#define MCG_C1_FRDIV_SHIFT        (3)       /* Bits 3-5: FLL External Reference Divider */
#define MCG_C1_FRDIV_MASK         (7 << MCG_C1_FRDIV_SHIFT)
#  define MCG_C1_FRDIV_R0DIV1     (0 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=1 */
#  define MCG_C1_FRDIV_R0DIV2     (1 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=2 */
#  define MCG_C1_FRDIV_R0DIV4     (2 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=4 */
#  define MCG_C1_FRDIV_R0DIV8     (3 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=8 */
#  define MCG_C1_FRDIV_R0DIV16    (4 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=16 */
#  define MCG_C1_FRDIV_R0DIV32    (5 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=32 */
#  define MCG_C1_FRDIV_R0DIV64    (6 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=64 */
#  define MCG_C1_FRDIV_R0DIV128   (7 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=128 */
#  define MCG_C1_FRDIV_DIV32      (0 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=32 */
#  define MCG_C1_FRDIV_DIV64      (1 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=64 */
#  define MCG_C1_FRDIV_DIV128     (2 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=128 */
#  define MCG_C1_FRDIV_DIV256     (3 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=256 */
#  define MCG_C1_FRDIV_DIV512     (4 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=512 */
#  define MCG_C1_FRDIV_DIV1024    (5 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=1024 */

#define MCG_C1_CLKS_SHIFT         (6)       /* Bits 6-7: Clock Source Select */
#define MCG_C1_CLKS_MASK          (3 << MCG_C1_CLKS_SHIFT)
#  define MCG_C1_CLKS_PLL         (0 << MCG_C1_CLKS_SHIFT) /* FLL or PLL output */
#  define MCG_C1_CLKS_INTREF      (1 << MCG_C1_CLKS_SHIFT) /* Internal reference clock */
#  define MCG_C1_CLKS_EXTREF      (2 << MCG_C1_CLKS_SHIFT) /* External reference clock */

/* MCG Control 2 Register */

#define MCG_C2_IRCS               (1 << 0)  /* Bit 0:  Internal Reference Clock Select */
#define MCG_C2_LP                 (1 << 1)  /* Bit 1:  Low Power Select */
#define MCG_C2_EREFS              (1 << 2)  /* Bit 2:  External Reference Select */
#define MCG_C2_HGO                (1 << 3)  /* Bit 3:  High Gain Oscillator Select */
#define MCG_C2_RANGE_SHIFT        (4)       /* Bits 4-5: Frequency Range Select */
#define MCG_C2_RANGE_MASK         (3 << MCG_C2_RANGE_SHIFT)
#  define MCG_C2_RANGE_LOW        (0 << MCG_C2_RANGE_SHIFT) /* Oscillator of 32 kHz to 40 kHz  */
#  define MCG_C2_RANGE_HIGH       (1 << MCG_C2_RANGE_SHIFT) /* Oscillator of 1 MHz to 8 MHz */
#  define MCG_C2_RANGE_VHIGH      (2 << MCG_C2_RANGE_SHIFT) /* Oscillator of 8 MHz to 32 MHz */

                                            /* Bits 6-7: Reserved */

/* MCG Control 3 Register
 * (8-bit Slow Internal Reference Clock Trim Setting)
 */

/* MCG Control 4 Register (8-bit) */

#define MCG_C4_SCFTRIM            (1 << 0)  /* Bit 0:  Slow Internal Reference Clock Fine Trim */
#define MCG_C4_FCTRIM_SHIFT       (1)       /* Bits 1-4: Fast Internal Reference Clock Trim Setting */
#define MCG_C4_FCTRIM_MASK        (15 << MCG_C4_FCTRIM_SHIFT)
#define MCG_C4_DRST_DRS_SHIFT     (5)       /* Bits 5-6: DCO Range Select */
#define MCG_C4_DRST_DRS_MASK      (3 << MCG_C4_DRST_DRS_SHIFT)
#  define MCG_C4_DRST_DRS_LOW     (0 << MCG_C4_DRST_DRS_SHIFT)
#  define MCG_C4_DRST_DRS_MID     (1 << MCG_C4_DRST_DRS_SHIFT)
#  define MCG_C4_DRST_DRS_MIDHIGH (2 << MCG_C4_DRST_DRS_SHIFT)
#  define MCG_C4_DRST_DRS_HIGH    (3 << MCG_C4_DRST_DRS_SHIFT)
#define MCG_C4_DMX32              (1 << 7)  /* Bit 7:  DCO Maximum Frequency with 32.768 kHz Reference */

/* MCG Control 5 Register */

#define MCG_C5_PRDIV_SHIFT        (0)       /* Bits 0-4: PLL External Reference Divider */
#define MCG_C5_PRDIV_MASK         (31 << MCG_C5_PRDIV_SHIFT)
#  define MCG_C5_PRDIV(n)         (((n)-1) << MCG_C5_PRDIV_SHIFT) /* Divide factor n=1..25 */

#define MCG_C5_PLLSTEN            (1 << 5)  /* Bit 5:  PLL Stop Enable */
#define MCG_C5_PLLCLKEN           (1 << 6)  /* Bit 6:  PLL Clock Enable */
                                            /* Bit 7: Reserved */

/* MCG Control 6 Register */

#define MCG_C6_VDIV_SHIFT         (0)       /* Bits 0-4: VCO Divider */
#define MCG_C6_VDIV_MASK          (31 << MCG_C6_VDIV_SHIFT)
#  define MCG_C6_VDIV(n)          (((n)-24) << MCG_C6_VDIV_SHIFT) /* Divide factor n=24..55 */

#define MCG_C6_CME                (1 << 5)  /* Bit 5:  Clock Monitor Enable */
#define MCG_C6_PLLS               (1 << 6)  /* Bit 6:  PLL Select */
#define MCG_C6_LOLIE              (1 << 7)  /* Bit 7:  Loss of Lock Interrupt Enable */

/* MCG Status Register */

#define MCG_S_IRCST               (1 << 0)  /* Bit 0:  Internal Reference Clock Status */
#define MCG_S_OSCINIT             (1 << 1)  /* Bit 1:  OSC Initialization */
#define MCG_S_CLKST_SHIFT         (2)       /* Bits 2-3: Clock Mode Status */
#define MCG_S_CLKST_MASK          (3 << MCG_S_CLKST_SHIFT)
#  define MCG_S_CLKST_FLL         (0 << MCG_S_CLKST_SHIFT) /* Output of the FLL */
#  define MCG_S_CLKST_INTREF      (1 << MCG_S_CLKST_SHIFT) /* Internal reference clock */
#  define MCG_S_CLKST_EXTREF      (2 << MCG_S_CLKST_SHIFT) /* External reference clock */
#  define MCG_S_CLKST_PLL         (3 << MCG_S_CLKST_SHIFT) /* Output of the PLL */

#define MCG_S_IREFST              (1 << 4)  /* Bit 4:  Internal Reference Status */
#define MCG_S_PLLST               (1 << 5)  /* Bit 5:  PLL Select Status */
#define MCG_S_LOCK                (1 << 6)  /* Bit 6:  Lock Status */
#define MCG_S_LOLS                (1 << 7)  /* Bit 7:  Loss of Lock Status */

/* MCG Auto Trim Control Register */

                                            /* Bits 0-4: Reserved */
#define MCG_ATC_ATMF              (1 << 5)  /* Bit 5:  Automatic Trim machine Fail Flag */
#define MCG_ATC_ATMS              (1 << 6)  /* Bit 6:  Automatic Trim Machine Select */
#define MCG_ATC_ATME              (1 << 7)  /* Bit 7:  Automatic Trim Machine Enable */

/* MCG Auto Trim Compare Value High/Low Registers (8-bit compare value) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KL_HARDWARE_KL_MCG_H */
