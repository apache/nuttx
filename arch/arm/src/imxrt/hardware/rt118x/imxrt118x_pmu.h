/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_pmu.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_PMU_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_PMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ANADIG PMU registers */

#define IMXRT_ANADIG_PMU_REF_CTRL_OFFSET       0x4500u
#define IMXRT_ANADIG_PMU_LDO_PLL_CTRL_OFFSET   0x4510u
#define IMXRT_ANADIG_PMU_LDO_LPSR_CTRL_OFFSET  0x4520u
#define IMXRT_ANADIG_PMU_BIAS_CTRL_OFFSET      0x4530u
#define IMXRT_ANADIG_PMU_BIAS_CTRL2_OFFSET     0x4540u
#define IMXRT_ANADIG_PMU_1P8_CTRL_OFFSET       0x4550u
#define IMXRT_ANADIG_PMU_1P0_CTRL_OFFSET       0x4560u
#define IMXRT_ANADIG_PMU_1P0_ANA_CTRL_OFFSET   0x4570u
#define IMXRT_ANADIG_PMU_DIGPROG_OFFSET        0x4800u

#define IMXRT_ANADIG_PMU_REF_CTRL \
  (IMXRT_ANADIG_PMU_BASE + IMXRT_ANADIG_PMU_REF_CTRL_OFFSET)
#define IMXRT_ANADIG_PMU_LDO_PLL_CTRL \
  (IMXRT_ANADIG_PMU_BASE + IMXRT_ANADIG_PMU_LDO_PLL_CTRL_OFFSET)
#define IMXRT_ANADIG_PMU_LDO_LPSR_CTRL \
  (IMXRT_ANADIG_PMU_BASE + IMXRT_ANADIG_PMU_LDO_LPSR_CTRL_OFFSET)
#define IMXRT_ANADIG_PMU_BIAS_CTRL \
  (IMXRT_ANADIG_PMU_BASE + IMXRT_ANADIG_PMU_BIAS_CTRL_OFFSET)
#define IMXRT_ANADIG_PMU_BIAS_CTRL2 \
  (IMXRT_ANADIG_PMU_BASE + IMXRT_ANADIG_PMU_BIAS_CTRL2_OFFSET)
#define IMXRT_ANADIG_PMU_1P8_CTRL \
  (IMXRT_ANADIG_PMU_BASE + IMXRT_ANADIG_PMU_1P8_CTRL_OFFSET)
#define IMXRT_ANADIG_PMU_1P0_CTRL \
  (IMXRT_ANADIG_PMU_BASE + IMXRT_ANADIG_PMU_1P0_CTRL_OFFSET)
#define IMXRT_ANADIG_PMU_1P0_ANA_CTRL \
  (IMXRT_ANADIG_PMU_BASE + IMXRT_ANADIG_PMU_1P0_ANA_CTRL_OFFSET)
#define IMXRT_ANADIG_PMU_DIGPROG \
  (IMXRT_ANADIG_PMU_BASE + IMXRT_ANADIG_PMU_DIGPROG_OFFSET)

#define PMU_REF_CTRL_REFSEL_SHIFT               0
#define PMU_REF_CTRL_REFSEL_MASK                (3u << PMU_REF_CTRL_REFSEL_SHIFT)
#define PMU_REF_CTRL_REFSEL(n)                  (((uint32_t)(n) << \
                                                  PMU_REF_CTRL_REFSEL_SHIFT) & \
                                                 PMU_REF_CTRL_REFSEL_MASK)
#define PMU_REF_CTRL_FTM_ENABLE                 (1u << 3)
#define PMU_REF_CTRL_ICOMP_TUNE_SHIFT           4
#define PMU_REF_CTRL_ICOMP_TUNE_MASK            (3u << 4)
#define PMU_REF_CTRL_VREF_OUT_ENABLE            (1u << 6)
#define PMU_REF_CTRL_VREF_SHIFT                 8
#define PMU_REF_CTRL_VREF_MASK                  (0x3fu << 8)
#define PMU_REF_CTRL_LPBG_ENABLE                (1u << 20)
#define PMU_REF_CTRL_BIAS_ENABLE                (1u << 21)
#define PMU_REF_CTRL_GPC_MODE                   (1u << 31)

#define PMU_LDO_PLL_ENABLE                      (1u << 0)
#define PMU_LDO_PLL_POWERUP                     (1u << 1)
#define PMU_LDO_PLL_OUTPUT_ENABLE               (1u << 2)
#define PMU_LDO_PLL_BYPASS                      (1u << 3)
#define PMU_LDO_PLL_STANDBY                     (1u << 4)
#define PMU_LDO_PLL_STABLE                      (1u << 31)

#define PMU_LDO_LPSR_ENABLE                     (1u << 0)
#define PMU_LDO_LPSR_POWERUP                    (1u << 1)
#define PMU_LDO_LPSR_OUTPUT_ENABLE              (1u << 2)
#define PMU_LDO_LPSR_BYPASS                     (1u << 3)
#define PMU_LDO_LPSR_STANDBY                    (1u << 4)
#define PMU_LDO_LPSR_STABLE                     (1u << 31)

#define PMU_BIAS_ENABLE                         (1u << 0)
#define PMU_BIAS_WELL_BIAS_ENABLE               (1u << 1)
#define PMU_BIAS_RBB_NWELL_SHIFT                4
#define PMU_BIAS_RBB_NWELL_MASK                 (3u << 4)
#define PMU_BIAS_RBB_PWELL_SHIFT                8
#define PMU_BIAS_RBB_PWELL_MASK                 (3u << 8)
#define PMU_BIAS_OK                             (1u << 31)

#define PMU_DIGPROG_MINOR_SHIFT                 0
#define PMU_DIGPROG_MINOR_MASK                  0xffu
#define PMU_DIGPROG_MAJOR_SHIFT                 8
#define PMU_DIGPROG_MAJOR_MASK                  (0xffu << 8)

/* PHY LDO register bank */

#define IMXRT_PHY_LDO_CTRL0_OFFSET              0x00u
#define IMXRT_PHY_LDO_CTRL0_SET_OFFSET          0x04u
#define IMXRT_PHY_LDO_CTRL0_CLR_OFFSET          0x08u
#define IMXRT_PHY_LDO_CTRL1_OFFSET              0x10u
#define IMXRT_PHY_LDO_CTRL2_OFFSET              0x20u

#define IMXRT_PHY_LDO_CTRL0 \
  (IMXRT_PHY_LDO_BASE + IMXRT_PHY_LDO_CTRL0_OFFSET)
#define IMXRT_PHY_LDO_CTRL0_SET \
  (IMXRT_PHY_LDO_BASE + IMXRT_PHY_LDO_CTRL0_SET_OFFSET)
#define IMXRT_PHY_LDO_CTRL0_CLR \
  (IMXRT_PHY_LDO_BASE + IMXRT_PHY_LDO_CTRL0_CLR_OFFSET)
#define IMXRT_PHY_LDO_CTRL1 \
  (IMXRT_PHY_LDO_BASE + IMXRT_PHY_LDO_CTRL1_OFFSET)
#define IMXRT_PHY_LDO_CTRL2 \
  (IMXRT_PHY_LDO_BASE + IMXRT_PHY_LDO_CTRL2_OFFSET)

#define PHY_LDO_ENABLE                          (1u << 0)
#define PHY_LDO_POWERUP                         (1u << 1)
#define PHY_LDO_CURRENT_LIMIT_ENABLE            (1u << 2)
#define PHY_LDO_BYPASS                          (1u << 3)
#define PHY_LDO_OUTPUT_TARGET_SHIFT             4
#define PHY_LDO_OUTPUT_TARGET_MASK              (0x1fu << \
                                                  PHY_LDO_OUTPUT_TARGET_SHIFT)
#define PHY_LDO_OUTPUT_TARGET(n)                (((uint32_t)(n) << \
                                                  PHY_LDO_OUTPUT_TARGET_SHIFT) & \
                                                 PHY_LDO_OUTPUT_TARGET_MASK)
#define PHY_LDO_STABLE                          (1u << 31)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_PMU_H */
