/****************************************************************************
 * arch/arm64/src/common/arm64_pmu.h
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

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_PMU_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_PMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "arm64_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PMCCFILTR_EL0 */

#define PMCCFILTR_EL0_P          (1ul << 31)  /* Privileged filtering bit */
#define PMCCFILTR_EL0_U          (1ul << 30)  /* User filtering bit */
#define PMCCFILTR_EL0_NSK        (1ul << 29)  /* Non-secure EL1 (kernel) modes filtering bit */
#define PMCCFILTR_EL0_NSU        (1ul << 28)  /* Non-secure EL0 (Unprivileged) filtering */
#define PMCCFILTR_EL0_NSH        (1ul << 27)  /* Non-secure EL2 (Hypervisor) filtering bit */
#define PMCCFILTR_EL0_M          (1ul << 26)  /* Secure EL3 filtering bit */

/* PMCNTENCLR_EL0 */

#define PMCNTENCLR_EL0_C         (1ul << 31)  /* PMCCNTR_EL0 disable bit */

/* PMINTENCLR_EL1 */

#define PMINTENCLR_EL1_C         (1ul << 31)  /* PMCCNTR_EL0 overflow interrupt request disable bit */

/* PMCNTENSET_EL0 */

#define PMCNTENSET_EL0_C         (1ul << 31)  /* Enables the cycle counter register */

/* PMCR_EL0 */

#define PMCR_EL0_LC              (1ul << 6)   /* Long cycle counter enable */
#define PMCR_EL0_DP              (1ul << 5)   /* Disable cycle counter when event counting is prohibited */
#define PMCR_EL0_X               (1ul << 4)   /* Enable export of events */
#define PMCR_EL0_D               (1ul << 3)   /* Clock divider */
#define PMCR_EL0_C               (1ul << 2)   /* Cycle counter reset */
#define PMCR_EL0_P               (1ul << 1)   /* Event counter reset */
#define PMCR_EL0_E               (1ul << 0)   /* All counters that are accessible at Non-secure EL1 are enabled by PMCNTENSET_EL0 */

/* PMINTENCLR_EL1 */

#define PMINTENCLR_EL1_C         (1ul << 31)  /* PMCCNTR_EL0 overflow interrupt request disable bit */

/* PMINTENSET_EL1 */

#define PMINTENSET_EL1_C         (1ul << 31)  /* PMCCNTR_EL0 overflow interrupt request enable bit */

/* PMOVSCLR_EL0 */

#define PMOVSCLR_EL0_C           (1ul << 31)  /* PMCCNTR_EL0 overflow bit */

/* PMSELR_EL0 */

#define PMSELR_EL0_SEL_C         (0x1ful << 0) /* When PMSELR_EL0.SEL is 0b11111, it selects the cycle counter */

/* PMUSERENR_EL0 */

#define PMUSERENR_EL0_ER         (1ul << 3)    /* Event counter read trap control */
#define PMUSERENR_EL0_CR         (1ul << 2)    /* Cycle counter read trap control */
#define PMUSERENR_EL0_SW         (1ul << 1)    /* Software Increment write trap control */
#define PMUSERENR_EL0_EN         (1ul << 0)    /* Software can access all PMU registers at EL0 */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pmu_get_ccntr
 *
 * Description:
 *   Read cycle counter.
 *
 * Return Value:
 *   Cycle count.
 *
 ****************************************************************************/

static inline uint64_t pmu_get_ccntr(void)
{
  return read_sysreg(pmccntr_el0);
}

/****************************************************************************
 * Name: pmu_ccntr_ccfiltr_config
 *
 * Description:
 *   Determines the modes in which the Cycle Counter.
 *
 * Parameters:
 *   mask - Filter flags for Cycle Counters.
 *
 ****************************************************************************/

static inline void pmu_ccntr_ccfiltr_config(uint64_t mask)
{
  write_sysreg(mask, pmccfiltr_el0);
}

/****************************************************************************
 * Name: pmu_cntr_select
 *
 * Description:
 *   Selects the current event counter or the cycle counter.
 *
 * Parameters:
 *   mask - Select counter flag.
 *
 ****************************************************************************/

static inline void pmu_cntr_select(uint64_t mask)
{
  write_sysreg(mask, pmselr_el0);
}

/****************************************************************************
 * Name: pmu_cntr_trap_control
 *
 * Description:
 *   Enables or disables EL0 access to the Performance Monitors.
 *
 * Parameters:
 *   mask - traped caused by operate counters through mask bit control
 *
 ****************************************************************************/

static inline void pmu_cntr_trap_control(uint64_t mask)
{
  write_sysreg(mask, pmuserenr_el0);
}

/****************************************************************************
 * Name: pmu_cntr_control_config
 *
 * Description:
 *   Config counters.
 *
 * Parameters:
 *   mask - Configuration flags for counters.
 *
 ****************************************************************************/

static inline void pmu_cntr_control_config(uint64_t mask)
{
  write_sysreg(mask, pmcr_el0);
}

/****************************************************************************
 * Name: pmu_cntr_enable
 *
 * Description:
 *   Enable counters.
 *
 * Parameters:
 *   mask - Counters to enable.
 *
 * Note:
 *   Enables one or more of the following:
 *     event counters (0-30)
 *     cycle counter
 *
 ****************************************************************************/

static inline void pmu_cntr_enable(uint64_t mask)
{
  write_sysreg(mask, pmcntenset_el0);
}

/****************************************************************************
 * Name: pmu_cntr_irq_enable
 *
 * Description:
 *   Enable counter overflow interrupt request.
 *
 * Parameters:
 *   mask - Counter overflow interrupt request bits to set.
 *
 * Note:
 *   Sets overflow interrupt request bits for one or more of the following:
 *     event counters (0-30)
 *     cycle counter
 *
 ****************************************************************************/

static inline void pmu_cntr_irq_enable(uint64_t mask)
{
  write_sysreg(mask, pmintenset_el1);
}

/****************************************************************************
 * Name: pmu_cntr_irq_disable
 *
 * Description:
 *   Disable counter overflow interrupt request.
 *
 * Parameters:
 *   mask - Counter overflow interrupt request bits to clear.
 *
 * Note:
 *   Sets overflow interrupt request bits for one or more of the following:
 *     event counters (0-30)
 *     cycle counter
 *
 ****************************************************************************/

static inline void pmu_cntr_irq_disable(uint64_t mask)
{
  write_sysreg(mask, pmintenclr_el1);
}

#endif /* __ARCH_ARM64_SRC_COMMON_ARM64_PMU_H */
