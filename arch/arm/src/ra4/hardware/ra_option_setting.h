/****************************************************************************
 * arch/arm/src/ra4/hardware/ra_option_setting.h
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

#ifndef __ARCH_ARM_SRC_RA_HARDWARE_RA_OFS_H
#define __ARCH_ARM_SRC_RA_HARDWARE_RA_OFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra_memorymap.h"

#if defined(CONFIG_RA4M1_FAMILY)
#  include "hardware/ra4m1_pinmap.h"
#else
#  error "Unsupported RA memory map"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define R_OFS0_OFFSET          0x0000  /* Option Function Select Register 0 (32-bits) */
#define R_OFS1_OFFSET          0x0004  /* Option Function Select Register 1 (32-bits) */

/* Register Addresses *******************************************************/

/* Option Function Select Registers */

#  define R_OFS0             (R_OFS_BASE + R_OFS0_OFFSET)
#  define R_OFS1             (R_OFS_BASE + R_OFS1_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Option Function Select Register 0 */

#define R_OFS0_RESERVED_31               (1 <<  31)                           /* Bit 31: Reserved */
#define R_OFS0_WDTSTPCTL                 (1 <<  30)                           /* Bit 30: WDT Stop Control */
#define R_OFS0_RESERVED_29               (1 <<  29)                           /* Bit 29: Reserved */
#define R_OFS0_WDTRSTIRQS                (1 <<  28)                           /* Bit 28: WDT Reset Interrupt Request Select*/
#define R_OFS0_WDTRPSS_SHIFT             (26)                                 /* Bit 27-26: WDT Window Start Position Select*/
#define R_OFS0_WDTRPSS_MASK              (3 << R_OFS0_WDTRPSS_SHIFT)
#  define R_OFS0_WDTRPSS_25              (0 << R_OFS0_WDTRPSS_SHIFT)          /* WDT Window Start Position Select 25% */
#  define R_OFS0_WDTRPSS_50              (1 << R_OFS0_WDTRPSS_SHIFT)          /* WDT Window Start Position Select 50% */
#  define R_OFS0_WDTRPSS_75              (2 << R_OFS0_WDTRPSS_SHIFT)          /* WDT Window Start Position Select 75% */
#  define R_OFS0_WDTRPSS_100             (3 << R_OFS0_WDTRPSS_SHIFT)          /* WDT Window Start Position Select 100% */
#define R_OFS0_WDTRPES_SHIFT             (24)                                 /* Bit 25-24: WDT Window End Position Select*/
#define R_OFS0_WDTRPES_MASK              (3 << R_OFS0_WDTRPES_SHIFT)
#  define R_OFS0_WDTRPES_75              (0 << R_OFS0_WDTRPES_SHIFT)          /* WDT Window End Position Select 75% */
#  define R_OFS0_WDTRPES_50              (1 << R_OFS0_WDTRPES_SHIFT)          /* WDT Window End Position Select 50% */
#  define R_OFS0_WDTRPES_25              (2 << R_OFS0_WDTRPES_SHIFT)          /* WDT Window End Position Select 25% */
#  define R_OFS0_WDTRPES_0               (3 << R_OFS0_WDTRPES_SHIFT)          /* WDT Window End Position Select 0% */
#define R_OFS0_WDTCKS_SHIFT              (20)                                 /* Bit 23-20: WDT Clock Frequency Division Ratio Select*/
#define R_OFS0_WDTCKS_MASK               (0xF << R_OFS0_WDTCKS_SHIFT)
#  define R_OFS0_WDTCKS_DIV_4            (1 << R_OFS0_WDTCKS_SHIFT)           /* PCLKB divided by 4  */
#  define R_OFS0_WDTCKS_DIV_64           (4 << R_OFS0_WDTCKS_SHIFT)           /* PCLKB divided by 64  */
#  define R_OFS0_WDTCKS_DIV_128          (15 << R_OFS0_WDTCKS_SHIFT)          /* PCLKB divided by 128 */
#  define R_OFS0_WDTCKS_DIV_512          (6 << R_OFS0_WDTCKS_SHIFT)           /* PCLKB divided by 512 */
#  define R_OFS0_WDTCKS_DIV_2048         (7 << R_OFS0_WDTCKS_SHIFT)           /* PCLKB divided by 2048 */
#  define R_OFS0_WDTCKS_DIV_8192         (8 << R_OFS0_WDTCKS_SHIFT)           /* PCLKB divided by 8192 */
#define R_OFS0_WDTTOPS_SHIFT             (18)                                 /* Bit 19-18: WDT Timeout Period Select */
#define R_OFS0_WDTTOPS_MASK              (3 << R_OFS0_WDTTOPS_SHIFT)
#  define R_OFS0_WDTTOPS_1024_CYCLES     (0 << R_OFS0_WDTTOPS_SHIFT)          /* 1024 cycles  */
#  define R_OFS0_WDTTOPS_4096_CYCLES     (1 << R_OFS0_WDTTOPS_SHIFT)          /* 4096 cycles  */
#  define R_OFS0_WDTTOPS_8192_CYCLES     (2 << R_OFS0_WDTTOPS_SHIFT)          /* 8192 cycles  */
#  define R_OFS0_WDTTOPS_16384_CYCLES    (3 << R_OFS0_WDTTOPS_SHIFT)          /* 16384 cycles  */
#define R_OFS0_RESERVED_16_15_SHIFT      (15)                                 /* Bit 16-15: Reserved */
#define R_OFS0_RESERVED_16_15_MASK       (3 << R_OFS0_RESERVED_16_15_SHIFT)   /* Bit 16-15: Reserved */
#define R_OFS0_WDTSTRT                   (1 <<  17)                           /* Bit 17: WDT Start Mode Select */
#define R_OFS0_IWDTSTPCTL                (1 <<  14)                           /* Bit 14: IWDT Stop Control  */
#define R_OFS0_RESERVED_13               (1 <<  13)                           /* Bit 13: Reserved */
#define R_OFS0_IWDTRSTIRQS               (1 <<  12)                           /* Bit 12: IWDT Reset Interrupt Request Select */
#define R_OFS0_IWDTRPSS_SHIFT            (10)                                 /* Bit 11-10: IWDT Window Start Position Select */
#define R_OFS0_IWDTRPSS_MASK             (3 << R_OFS0_IWDTRPSS_SHIFT)
#  define R_OFS0_IWDTRPSS_25             (0 << R_OFS0_IWDTRPSS_SHIFT)         /* IWDT Window Start Position Select 25% */
#  define R_OFS0_IWDTRPSS_50             (1 << R_OFS0_IWDTRPSS_SHIFT)         /* IWDT Window Start Position Select 50%   */
#  define R_OFS0_IWDTRPSS_75             (2 << R_OFS0_IWDTRPSS_SHIFT)         /* IWDT Window Start Position Select 75%  */
#  define R_OFS0_IWDTRPSS_100            (3 << R_OFS0_IWDTRPSS_SHIFT)         /* IWDT Window Start Position Select 100%   */
#define R_OFS0_IWDTRPES_SHIFT            (8)                                  /* Bit 9-8: IWDT Window End Position Select*/
#define R_OFS0_IWDTRPES_MASK             (3 << R_OFS0_IWDTRPES_SHIFT)
#  define R_OFS0_IWDTRPES_75             (0 << R_OFS0_IWDTRPES_SHIFT)         /* IWDT Window End Position Select 75% */
#  define R_OFS0_IWDTRPES_50             (1 << R_OFS0_IWDTRPES_SHIFT)         /* IWDT Window End Position Select 50% */
#  define R_OFS0_IWDTRPES_25             (2 << R_OFS0_IWDTRPES_SHIFT)         /* IWDT Window End Position Select 25% */
#  define R_OFS0_IWDTRPES_0              (3 << R_OFS0_IWDTRPES_SHIFT)         /* IWDT Window End Position Select 0% */
#define R_OFS0_IWDTCKS_SHIFT             (4)                                  /* Bit 7-4: IWDT Clock Frequency Division Ratio Select*/
#define R_OFS0_IWDTCKS_MASK              (0xF << R_OFS0_IWDTCKS_SHIFT)
#  define R_OFS0_IWDTCKS_DIV_1           (0 << R_OFS0_IWDTCKS_SHIFT)          /* Divided by 1  */
#  define R_OFS0_IWDTCKS_DIV_16          (2 << R_OFS0_IWDTCKS_SHIFT)          /* Divided by 16  */
#  define R_OFS0_IWDTCKS_DIV_32          (3 << R_OFS0_IWDTCKS_SHIFT)          /* Divided by 32 */
#  define R_OFS0_IWDTCKS_DIV_64          (4 << R_OFS0_IWDTCKS_SHIFT)          /* Divided by 64 */
#  define R_OFS0_IWDTCKS_DIV_128         (15 << R_OFS0_IWDTCKS_SHIFT)         /* Divided by 128 */
#  define R_OFS0_IWDTCKS_DIV_256         (5 << R_OFS0_IWDTCKS_SHIFT)          /* Divided by 256 */
#define R_OFS0_IWDTTOPS_SHIFT            (2)                                  /* Bit 3-2: IWDT Timeout Period Select */
#define R_OFS0_IWDTTOPS_MASK             (3 << R_OFS0_IWDTTOPS_SHIFT)
#  define R_OFS0_IWDTTOPS_128_CYCLES     (0 << R_OFS0_IWDTTOPS_SHIFT)         /* 128 cycles  */
#  define R_OFS0_IWDTTOPS_512_CYCLES     (1 << R_OFS0_IWDTTOPS_SHIFT)         /* 512 cycles    */
#  define R_OFS0_IWDTTOPS_1024_CYCLES    (2 << R_OFS0_IWDTTOPS_SHIFT)         /* 1024 cycles   */
#  define R_OFS0_IWDTTOPS_2048_CYCLES    (3 << R_OFS0_IWDTTOPS_SHIFT)         /* 2048 cycles   */
#define R_OFS0_IWDTSTRT                  (1 <<  1)                            /* Bit 1: IWDT Start Mode Select  */
#define R_OFS0_RESERVED_0                (1 <<  0)                            /* Bit 0: Reserved */

/* Option Function Select Register 1 */

#define R_OFS1_RESERVED_31_15_SHIFT     (15)                            /* Bit 31-15: Reserved */
#define R_OFS1_RESERVED_16_15_MASK      (0x1FFFF << R_OFS1_RESERVED_31_15_SHIFT)
#define R_OFS1_HOCOFRQ1_SHIFT           (12)                            /* Bit 14-12: IWDT Timeout Period Select */
#define R_OFS1_HOCOFRQ1_MASK            (7 << R_OFS1_HOCOFRQ1_SHIFT)
#  define R_OFS1_HOCOFRQ1_24MHZ         (0 << R_OFS1_HOCOFRQ1_SHIFT)    /* HOCO 24 Mhz */
#  define R_OFS1_HOCOFRQ1_32MHZ         (2 << R_OFS1_HOCOFRQ1_SHIFT)    /* HOCO 32 Mhz */
#  define R_OFS1_HOCOFRQ1_48MHZ         (4 << R_OFS1_HOCOFRQ1_SHIFT)    /* HOCO 48 Mhz */
#  define R_OFS1_HOCOFRQ1_64MHZ         (5 << R_OFS1_HOCOFRQ1_SHIFT)    /* HOCO 64 Mhz */
#define R_OFS1_RESERVED_11_9_SHIFT      (9)                             /* Bit 11-9: Reserved */
#define R_OFS1_RESERVED_11_9_MASK       (7 << R_OFS1_RESERVED_11_9_SHIFT)
#define R_OFS1_HOCOEN                   (1 <<  8)                       /* Bit 8: HOCO Oscillation Enable */
#define R_OFS1_RESERVED_7_6_SHIFT       (6)                             /* Bit 7-6: Reserved */
#define R_OFS1_RESERVED_7_6_MASK        (3 << R_OFS1_RESERVED_7_6_SHIFT)
#define R_OFS1_VDSEL1_SHIFT             (3)                             /* Bit 5-3: Voltage Detection 0 Level Select */
#define R_OFS1_VDSEL1_MASK              (7 << R_OFS1_VDSEL1_SHIFT)
#  define R_OFS1_VDSEL1_3_84V           (0 << R_OFS1_VDSEL1_SHIFT)      /* Selects 3.84 V */
#  define R_OFS1_VDSEL1_2_82V           (1 << R_OFS1_VDSEL1_SHIFT)      /* Selects 2.82 V*/
#  define R_OFS1_VDSEL1_2_51V           (2 << R_OFS1_VDSEL1_SHIFT)      /* Selects 2.51 V */
#  define R_OFS1_VDSEL1_1_90V           (3 << R_OFS1_VDSEL1_SHIFT)      /* Selects 1.70 V */
#  define R_OFS1_VDSEL1_1_70V           (4 << R_OFS1_VDSEL1_SHIFT)      /* Selects 1.70 V */
#define R_OFS1_LVDAS                    (1 <<  2)                       /* Bit 8: Voltage Detection 0 Circuit Start*/
#define R_OFS1_RESERVED_1_0_SHIFT       (0)                             /* Bit 1-0: Reserved */
#define R_OFS1_RESERVED_1_0_MASK        (3 << R_OFS1_RESERVED_1_0_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA_HARDWARE_RA_OFS_H */
