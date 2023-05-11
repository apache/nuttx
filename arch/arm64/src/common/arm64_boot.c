/****************************************************************************
 * arch/arm64/src/common/arm64_boot.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>

#include "arm64_internal.h"
#include "arm64_arch.h"

extern void *_vector_table[];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_EL3

/* Some ARM aarch64 Cortex-family processors have not EL3
 * these two function should never called
 * defined to available compile error when with gcc option
 */

void arm64_boot_el3_init(void)
{
  uint64_t reg;

  /* Setup vector table */

  write_sysreg((uint64_t)_vector_table, vbar_el3);
  ARM64_ISB();

  reg   = 0U;                   /* Mostly RES0 */
  reg   &= ~(CPTR_TTA_BIT |     /* Do not trap sysreg accesses */
             CPTR_TFP_BIT |     /* Do not trap SVE, SIMD and FP */
             CPTR_TCPAC_BIT);   /* Do not trap CPTR_EL2 CPACR_EL1 accesses */

  /* CPTR_EL3, Architectural Feature Trap Register (EL3) */

  write_sysreg(reg, cptr_el3);

  reg   = 0U;               /* Reset */
  reg   |= SCR_NS_BIT;      /* EL2 / EL3 non-secure */
  reg   |= (SCR_RES1 |      /* RES1 */
            SCR_RW_BIT |    /* EL2 execution state is AArch64 */
            SCR_ST_BIT |    /* Do not trap EL1 accesses to timer */
            SCR_HCE_BIT |   /* Do not trap HVC */
            SCR_SMD_BIT);   /* Do not trap SMC */
  write_sysreg(reg, scr_el3);

  reg   = read_sysreg(ICC_SRE_EL3);
  reg   |= (ICC_SRE_ELX_DFB_BIT |   /* Disable FIQ bypass */
            ICC_SRE_ELX_DIB_BIT |   /* Disable IRQ bypass */
            ICC_SRE_ELX_SRE_BIT |   /* System register interface is used */
            ICC_SRE_EL3_EN_BIT);    /* Enables lower Exception level access to
                                     * ICC_SRE_EL1 */
  write_sysreg(reg, ICC_SRE_EL3);

  ARM64_ISB();
}

void arm64_boot_el3_get_next_el(uint64_t switch_addr)
{
  uint64_t spsr;

  write_sysreg(switch_addr, elr_el3);

  /* Mask the DAIF */

  spsr  = SPSR_DAIF_MASK;
  spsr  |= SPSR_MODE_EL2T;

  write_sysreg(spsr, spsr_el3);
}
#endif

void arm64_boot_el2_init(void)
{
  uint64_t reg;

  reg   = read_sysreg(sctlr_el2);
  reg   |= (SCTLR_EL2_RES1 |    /* RES1 */
            SCTLR_I_BIT |       /* Enable i-cache */
            SCTLR_SA_BIT);      /* Enable SP alignment check */
  write_sysreg(reg, sctlr_el2);

  reg   = read_sysreg(hcr_el2);
  reg   |= HCR_RW_BIT;      /* EL1 Execution state is AArch64 */
  write_sysreg(reg, hcr_el2);

  reg   = 0U;                   /* RES0 */
  reg   |= CPTR_EL2_RES1;       /* RES1 */
  reg   &= ~(CPTR_TFP_BIT |     /* Do not trap SVE, SIMD and FP */
             CPTR_TCPAC_BIT);   /* Do not trap CPACR_EL1 accesses */
  write_sysreg(reg, cptr_el2);

  /* Enable EL1 access to timers */

  reg   = read_sysreg(cnthctl_el2);
  reg   |= (CNTHCTL_EL2_EL1PCEN_EN | CNTHCTL_EL2_EL1PCTEN_EN);
  write_sysreg(reg, cnthctl_el2);

  zero_sysreg(cntvoff_el2);       /* Set 64-bit virtual timer offset to 0 */

#ifdef CONFIG_ARCH_ARMV8R
  zero_sysreg(cnthps_ctl_el2);
#else
  zero_sysreg(cnthp_ctl_el2);
#endif

#ifdef CONFIG_ARCH_SET_VMPIDR_EL2
  reg = read_sysreg(mpidr_el1);
  write_sysreg(reg, vmpidr_el2);
#endif

  /* Enable this if/when we use the hypervisor timer.
   * write_cnthp_cval_el2(~(uint64_t)0);
   */

  ARM64_ISB();
}

void arm64_boot_el1_init(void)
{
  uint64_t reg;

  /* Setup vector table */

  write_sysreg((uint64_t)_vector_table, vbar_el1);
  ARM64_ISB();

  reg   = 0U;                       /* RES0 */
  reg   |= CPACR_EL1_FPEN_NOTRAP;   /* Do not trap NEON/SIMD/FP initially */

  /* TODO: CONFIG_FLOAT_*_FORBIDDEN */

  write_sysreg(reg, cpacr_el1);

  reg   = read_sysreg(sctlr_el1);
  reg   |= (SCTLR_EL1_RES1 |    /* RES1 */
            SCTLR_I_BIT |       /* Enable i-cache */
            SCTLR_SA_BIT);      /* Enable SP alignment check */
  write_sysreg(reg, sctlr_el1);

  write_sysreg((~(uint64_t)0), cntv_cval_el0);

  /* Enable these if/when we use the corresponding timers.
   * write_cntp_cval_el0(~(uint64_t)0);
   * write_cntps_cval_el1(~(uint64_t)0);
   */

  ARM64_ISB();
}

void arm64_boot_primary_c_routine(void)
{
  arm64_chip_boot();
  up_perf_init(NULL);
  nx_start();
}
