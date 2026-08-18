/****************************************************************************
 * arch/risc-v/src/allwinner-d1/chip.h
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ALLWINNER_D1_CHIP_H
#define __ARCH_RISCV_SRC_ALLWINNER_D1_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/allwinner-d1/chip.h>

#include "allwinner_d1_memorymap.h"
#include "hardware/allwinner_d1_memorymap.h"
#include "hardware/allwinner_d1_plic.h"
#include "riscv_internal.h"
#include "riscv_percpu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef __ASSEMBLY__
#if CONFIG_ARCH_INTERRUPTSTACK > 15 && !defined(CONFIG_SMP) && \
    defined(CONFIG_ARCH_USE_S_MODE)
.macro setintstack tmp0, tmp1
  csrr    \tmp0, CSR_SCRATCH
  REGLOAD sp, RISCV_PERCPU_IRQSTACK(\tmp0)
.endm
#endif
#endif

#endif /* __ARCH_RISCV_SRC_ALLWINNER_D1_CHIP_H */
