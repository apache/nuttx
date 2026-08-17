/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_osc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_OSC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_OSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ANADIG oscillator registers */

#define IMXRT_ANADIG_OSC_RC24M_CTRL_OFFSET      0x4310u
#define IMXRT_ANADIG_OSC_24M_CTRL_OFFSET        0x4320u
#define IMXRT_ANADIG_OSC_400M_CTRL0_OFFSET      0x4340u
#define IMXRT_ANADIG_OSC_400M_CTRL1_OFFSET      0x4350u

#define IMXRT_ANADIG_OSC_RC24M_CTRL \
  (IMXRT_ANADIG_OSC_BASE + IMXRT_ANADIG_OSC_RC24M_CTRL_OFFSET)
#define IMXRT_ANADIG_OSC_24M_CTRL \
  (IMXRT_ANADIG_OSC_BASE + IMXRT_ANADIG_OSC_24M_CTRL_OFFSET)
#define IMXRT_ANADIG_OSC_400M_CTRL0 \
  (IMXRT_ANADIG_OSC_BASE + IMXRT_ANADIG_OSC_400M_CTRL0_OFFSET)
#define IMXRT_ANADIG_OSC_400M_CTRL1 \
  (IMXRT_ANADIG_OSC_BASE + IMXRT_ANADIG_OSC_400M_CTRL1_OFFSET)

#define OSC_RC24M_ENABLE                       (1u << 1)
#define OSC_RC24M_SOURCE_SEL_24M               (1u << 25)
#define OSC_RC24M_GPC_MODE                     (1u << 31)

#define OSC_24M_BYPASS_ENABLE                  (1u << 1)
#define OSC_24M_LOW_POWER_ENABLE               (1u << 2)
#define OSC_24M_COMPARATOR_DIFFERENTIAL        (1u << 3)
#define OSC_24M_ENABLE                         (1u << 4)
#define OSC_24M_GATE                           (1u << 7)
#define OSC_24M_STABLE                         (1u << 30)
#define OSC_24M_GPC_MODE                       (1u << 31)

#define OSC_400M_AI_BUSY                       (1u << 31)
#define OSC_400M_POWERDOWN                     (1u << 0)
#define OSC_400M_GPC_MODE                      (1u << 31)

/* Tunable RC400M register bank */

#define IMXRT_OSC_RC_400M_CTRL0_OFFSET         0x00u
#define IMXRT_OSC_RC_400M_CTRL0_SET_OFFSET     0x04u
#define IMXRT_OSC_RC_400M_CTRL0_CLR_OFFSET     0x08u
#define IMXRT_OSC_RC_400M_CTRL0_TOG_OFFSET     0x0cu
#define IMXRT_OSC_RC_400M_CTRL1_OFFSET         0x10u
#define IMXRT_OSC_RC_400M_CTRL1_SET_OFFSET     0x14u
#define IMXRT_OSC_RC_400M_CTRL1_CLR_OFFSET     0x18u
#define IMXRT_OSC_RC_400M_CTRL1_TOG_OFFSET     0x1cu
#define IMXRT_OSC_RC_400M_CTRL2_OFFSET         0x20u
#define IMXRT_OSC_RC_400M_CTRL2_SET_OFFSET     0x24u
#define IMXRT_OSC_RC_400M_CTRL2_CLR_OFFSET     0x28u
#define IMXRT_OSC_RC_400M_CTRL2_TOG_OFFSET     0x2cu
#define IMXRT_OSC_RC_400M_CTRL3_OFFSET         0x30u
#define IMXRT_OSC_RC_400M_CTRL3_SET_OFFSET     0x34u
#define IMXRT_OSC_RC_400M_CTRL3_CLR_OFFSET     0x38u
#define IMXRT_OSC_RC_400M_CTRL3_TOG_OFFSET     0x3cu
#define IMXRT_OSC_RC_400M_STAT0_OFFSET         0x50u
#define IMXRT_OSC_RC_400M_STAT1_OFFSET         0x60u
#define IMXRT_OSC_RC_400M_STAT2_OFFSET         0x70u

#define IMXRT_OSC_RC_400M_REG(o)               (IMXRT_OSC_RC_400M_BASE + (o))
#define IMXRT_OSC_RC_400M_CTRL0 \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_CTRL0_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL0_SET \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_CTRL0_SET_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL0_CLR \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_CTRL0_CLR_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL1 \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_CTRL1_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL1_SET \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_CTRL1_SET_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL1_CLR \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_CTRL1_CLR_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL2 \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_CTRL2_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL2_SET \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_CTRL2_SET_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL2_CLR \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_CTRL2_CLR_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL3 \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_CTRL3_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL3_SET \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_CTRL3_SET_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL3_CLR \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_CTRL3_CLR_OFFSET)
#define IMXRT_OSC_RC_400M_STAT0 \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_STAT0_OFFSET)
#define IMXRT_OSC_RC_400M_STAT1 \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_STAT1_OFFSET)
#define IMXRT_OSC_RC_400M_STAT2 \
  IMXRT_OSC_RC_400M_REG(IMXRT_OSC_RC_400M_STAT2_OFFSET)

#define OSC_RC400M_REF_DIV_SHIFT                24
#define OSC_RC400M_REF_DIV_MASK                 (0x3fu << \
                                                  OSC_RC400M_REF_DIV_SHIFT)
#define OSC_RC400M_REF_DIV(n)                   (((uint32_t)(n) << \
                                                  OSC_RC400M_REF_DIV_SHIFT) & \
                                                 OSC_RC400M_REF_DIV_MASK)
#define OSC_RC400M_HYST_MINUS_SHIFT              0
#define OSC_RC400M_HYST_MINUS_MASK               0x0fu
#define OSC_RC400M_HYST_PLUS_SHIFT               8
#define OSC_RC400M_HYST_PLUS_MASK                (0x0fu << 8)
#define OSC_RC400M_TARGET_COUNT_SHIFT            16
#define OSC_RC400M_TARGET_COUNT_MASK             (0xffffu << 16)
#define OSC_RC400M_TUNE_BYPASS                   (1u << 10)
#define OSC_RC400M_TUNE_ENABLE                   (1u << 12)
#define OSC_RC400M_TUNE_START                    (1u << 14)
#define OSC_RC400M_TUNE_VALUE_SHIFT              24
#define OSC_RC400M_TUNE_VALUE_MASK               (0xffu << 24)
#define OSC_RC400M_CLEAR_ERROR                   (1u << 0)
#define OSC_RC400M_1M_ENABLE                     (1u << 8)
#define OSC_RC400M_1M_MUX_LOCKED                 (1u << 10)
#define OSC_RC400M_1M_COUNT_SHIFT                16
#define OSC_RC400M_1M_COUNT_MASK                 (0xffffu << 16)
#define OSC_RC400M_1M_ERROR                      (1u << 0)
#define OSC_RC400M_CURRENT_COUNT_SHIFT           16
#define OSC_RC400M_CURRENT_COUNT_MASK            (0xffffu << 16)
#define OSC_RC400M_CURRENT_TUNE_SHIFT            24
#define OSC_RC400M_CURRENT_TUNE_MASK             (0xffu << 24)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_OSC_H */
