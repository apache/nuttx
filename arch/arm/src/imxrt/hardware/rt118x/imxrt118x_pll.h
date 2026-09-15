/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_pll.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0.
 ****************************************************************************/

/* Register layout and fields follow NXP MIMXRT1186 CMSIS device headers. */

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_PLL_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_PLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ANADIG PLL registers */

#define IMXRT_ANADIG_PLL_ARM_CTRL_OFFSET       0x4000u
#define IMXRT_ANADIG_PLL_SYS3_CTRL_OFFSET      0x4010u
#define IMXRT_ANADIG_PLL_SYS3_UPDATE_OFFSET    0x4020u
#define IMXRT_ANADIG_PLL_SYS3_PFD_OFFSET       0x4030u
#define IMXRT_ANADIG_PLL_SYS2_CTRL_OFFSET      0x4040u
#define IMXRT_ANADIG_PLL_SYS2_UPDATE_OFFSET    0x4050u
#define IMXRT_ANADIG_PLL_SYS2_SS_OFFSET        0x4060u
#define IMXRT_ANADIG_PLL_SYS2_PFD_OFFSET       0x4070u
#define IMXRT_ANADIG_PLL_SYS2_MFN_OFFSET       0x4080u
#define IMXRT_ANADIG_PLL_SYS2_MFI_OFFSET       0x4090u
#define IMXRT_ANADIG_PLL_SYS2_MFD_OFFSET       0x40a0u
#define IMXRT_ANADIG_PLL_SYS1_CTRL_OFFSET      0x4100u
#define IMXRT_ANADIG_PLL_AUDIO_CTRL_OFFSET     0x4200u

#define IMXRT_ANADIG_PLL_ARM_CTRL       (IMXRT_ANADIG_PLL_BASE + \
                                         IMXRT_ANADIG_PLL_ARM_CTRL_OFFSET)
#define IMXRT_ANADIG_PLL_SYS3_CTRL      (IMXRT_ANADIG_PLL_BASE + \
                                         IMXRT_ANADIG_PLL_SYS3_CTRL_OFFSET)
#define IMXRT_ANADIG_PLL_SYS3_UPDATE    (IMXRT_ANADIG_PLL_BASE + \
                                         IMXRT_ANADIG_PLL_SYS3_UPDATE_OFFSET)
#define IMXRT_ANADIG_PLL_SYS3_PFD       (IMXRT_ANADIG_PLL_BASE + \
                                         IMXRT_ANADIG_PLL_SYS3_PFD_OFFSET)
#define IMXRT_ANADIG_PLL_SYS2_CTRL      (IMXRT_ANADIG_PLL_BASE + \
                                         IMXRT_ANADIG_PLL_SYS2_CTRL_OFFSET)
#define IMXRT_ANADIG_PLL_SYS2_UPDATE    (IMXRT_ANADIG_PLL_BASE + \
                                         IMXRT_ANADIG_PLL_SYS2_UPDATE_OFFSET)
#define IMXRT_ANADIG_PLL_SYS2_SS        (IMXRT_ANADIG_PLL_BASE + \
                                         IMXRT_ANADIG_PLL_SYS2_SS_OFFSET)
#define IMXRT_ANADIG_PLL_SYS2_PFD       (IMXRT_ANADIG_PLL_BASE + \
                                         IMXRT_ANADIG_PLL_SYS2_PFD_OFFSET)
#define IMXRT_ANADIG_PLL_SYS2_MFN       (IMXRT_ANADIG_PLL_BASE + \
                                         IMXRT_ANADIG_PLL_SYS2_MFN_OFFSET)
#define IMXRT_ANADIG_PLL_SYS2_MFI       (IMXRT_ANADIG_PLL_BASE + \
                                         IMXRT_ANADIG_PLL_SYS2_MFI_OFFSET)
#define IMXRT_ANADIG_PLL_SYS2_MFD       (IMXRT_ANADIG_PLL_BASE + \
                                         IMXRT_ANADIG_PLL_SYS2_MFD_OFFSET)
#define IMXRT_ANADIG_PLL_SYS1_CTRL      (IMXRT_ANADIG_PLL_BASE + \
                                         IMXRT_ANADIG_PLL_SYS1_CTRL_OFFSET)
#define IMXRT_ANADIG_PLL_AUDIO_CTRL     (IMXRT_ANADIG_PLL_BASE + \
                                         IMXRT_ANADIG_PLL_AUDIO_CTRL_OFFSET)

/* Fractional PLL register banks used by SYS PLL1 and audio PLL */

#define IMXRT_PLL_CTRL_OFFSET           0x00u
#define IMXRT_PLL_CTRL_SET_OFFSET       0x04u
#define IMXRT_PLL_CTRL_CLR_OFFSET       0x08u
#define IMXRT_PLL_CTRL_TOG_OFFSET       0x0cu
#define IMXRT_PLL_SS_OFFSET             0x10u
#define IMXRT_PLL_NUM_OFFSET            0x20u
#define IMXRT_PLL_DENOM_OFFSET          0x30u

#define IMXRT_PLL_CTRL(base)            ((base) + IMXRT_PLL_CTRL_OFFSET)
#define IMXRT_PLL_CTRL_SET(base)        ((base) + IMXRT_PLL_CTRL_SET_OFFSET)
#define IMXRT_PLL_CTRL_CLR(base)        ((base) + IMXRT_PLL_CTRL_CLR_OFFSET)
#define IMXRT_PLL_CTRL_TOG(base)        ((base) + IMXRT_PLL_CTRL_TOG_OFFSET)
#define IMXRT_PLL_SS(base)              ((base) + IMXRT_PLL_SS_OFFSET)
#define IMXRT_PLL_NUM(base)             ((base) + IMXRT_PLL_NUM_OFFSET)
#define IMXRT_PLL_DENOM(base)           ((base) + IMXRT_PLL_DENOM_OFFSET)

/* ARM PLL */

#define PLL_ARM_DIV_SHIFT               0
#define PLL_ARM_DIV_MASK                (0xffu << PLL_ARM_DIV_SHIFT)
#define PLL_ARM_DIV(n)                  (((uint32_t)(n) << PLL_ARM_DIV_SHIFT) & \
                                         PLL_ARM_DIV_MASK)
#define PLL_ARM_HOLD_RING_OFF            (1u << 12)
#define PLL_ARM_POWERUP                  (1u << 13)
#define PLL_ARM_ENABLE                   (1u << 14)
#define PLL_ARM_POSTDIV_SHIFT            15
#define PLL_ARM_POSTDIV_MASK             (3u << PLL_ARM_POSTDIV_SHIFT)
#define PLL_ARM_POSTDIV(n)               (((uint32_t)(n) << \
                                           PLL_ARM_POSTDIV_SHIFT) & \
                                          PLL_ARM_POSTDIV_MASK)
#define PLL_ARM_BYPASS                   (1u << 17)
#define PLL_ARM_STABLE                   (1u << 29)
#define PLL_ARM_GATE                     (1u << 30)
#define PLL_ARM_GPC_MODE                 (1u << 31)

/* SYS PLL2 and SYS PLL3 */

#define PLL_SYS3_DIV_SHIFT               0
#define PLL_SYS3_DIV_MASK                (7u << PLL_SYS3_DIV_SHIFT)
#define PLL_SYS3_DIV2_ENABLE             (1u << 3)
#define PLL_SYS3_REG_ENABLE              (1u << 4)
#define PLL_SYS3_HOLD_RING_OFF            (1u << 11)
#define PLL_SYS3_ENABLE                  (1u << 13)
#define PLL_SYS3_BYPASS                  (1u << 16)
#define PLL_SYS3_POWERUP                 (1u << 21)
#define PLL_SYS3_STABLE                  (1u << 29)
#define PLL_SYS3_GATE                    (1u << 30)
#define PLL_SYS3_GPC_MODE                (1u << 31)

#define PLL_SYS2_REG_ENABLE              (1u << 3)
#define PLL_SYS2_HOLD_RING_OFF            (1u << 11)
#define PLL_SYS2_ENABLE                  (1u << 13)
#define PLL_SYS2_BYPASS                  (1u << 16)
#define PLL_SYS2_DITHER_ENABLE           (1u << 17)
#define PLL_SYS2_PFD_OFFSET_ENABLE       (1u << 18)
#define PLL_SYS2_DDR_OVERRIDE            (1u << 19)
#define PLL_SYS2_POWERUP                 (1u << 23)
#define PLL_SYS2_STABLE                  (1u << 29)
#define PLL_SYS2_GATE                    (1u << 30)
#define PLL_SYS2_GPC_MODE                (1u << 31)

#define PLL_SYS1_ENABLE                  (1u << 13)
#define PLL_SYS1_GATE                    (1u << 14)
#define PLL_SYS1_DIV2_ENABLE             (1u << 25)
#define PLL_SYS1_DIV5_ENABLE             (1u << 26)
#define PLL_SYS1_STABLE                  (1u << 29)
#define PLL_SYS1_GPC_MODE                (1u << 31)

#define PLL_AUDIO_ENABLE                 (1u << 13)
#define PLL_AUDIO_GATE                   (1u << 14)
#define PLL_AUDIO_STABLE                 (1u << 29)
#define PLL_AUDIO_GPC_MODE               (1u << 31)

/* PFD registers: four identical eight-bit fields */

#define PLL_PFD_COUNT                    4
#define PLL_PFD_FRAC_SHIFT(n)            ((n) << 3)
#define PLL_PFD_FRAC_MASK(n)             (0x3fu << PLL_PFD_FRAC_SHIFT(n))
#define PLL_PFD_FRAC(n, v)               (((uint32_t)(v) << \
                                           PLL_PFD_FRAC_SHIFT(n)) & \
                                          PLL_PFD_FRAC_MASK(n))
#define PLL_PFD_STABLE(n)                (1u << (6 + PLL_PFD_FRAC_SHIFT(n)))
#define PLL_PFD_GATE(n)                  (1u << (7 + PLL_PFD_FRAC_SHIFT(n)))
#define PLL_PFD_UPDATE(n)                (1u << ((n) + 1))
#define PLL_PFD_CONTROL_MODE(n)          (1u << ((n) + 5))

#define PLL_SS_STEP_SHIFT                0
#define PLL_SS_STEP_MASK                 (0x7fffu << PLL_SS_STEP_SHIFT)
#define PLL_SS_ENABLE                    (1u << 15)
#define PLL_SS_STOP_SHIFT                16
#define PLL_SS_STOP_MASK                 (0xffffu << PLL_SS_STOP_SHIFT)

#define PLL_SYS2_MFN_MASK                0x3fffffffu
#define PLL_SYS2_MFI_MASK                0x7fu
#define PLL_SYS2_MFD_MASK                0x3fffffffu

/* Generic fractional PLL fields */

#define PLL_CTRL_DIV_SHIFT               0
#define PLL_CTRL_DIV_MASK                (0x7fu << PLL_CTRL_DIV_SHIFT)
#define PLL_CTRL_ENABLE_ALT              (1u << 8)
#define PLL_CTRL_HOLD_RING_OFF           (1u << 13)
#define PLL_CTRL_POWERUP                 (1u << 14)
#define PLL_CTRL_ENABLE                  (1u << 15)
#define PLL_CTRL_BYPASS                  (1u << 16)
#define PLL_CTRL_DITHER_ENABLE           (1u << 17)
#define PLL_CTRL_REG_ENABLE              (1u << 22)
#define PLL_CTRL_POSTDIV_SHIFT           25
#define PLL_CTRL_POSTDIV_MASK            (7u << PLL_CTRL_POSTDIV_SHIFT)
#define PLL_CTRL_BIAS_SELECT             (1u << 29)
#define PLL_NUM_MASK                     0x3fffffffu
#define PLL_DENOM_MASK                   0x3fffffffu

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_PLL_H */
