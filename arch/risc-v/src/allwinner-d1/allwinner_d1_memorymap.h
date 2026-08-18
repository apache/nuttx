/****************************************************************************
 * arch/risc-v/src/allwinner-d1/allwinner_d1_memorymap.h
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ALLWINNER_D1_ALLWINNER_D1_MEMORYMAP_H
#define __ARCH_RISCV_SRC_ALLWINNER_D1_ALLWINNER_D1_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "riscv_common_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  define ALLWINNER_D1_IDLESTACK_BASE ((uintptr_t)_ebss)
#else
#  define ALLWINNER_D1_IDLESTACK_BASE _ebss
#endif

#endif /* __ARCH_RISCV_SRC_ALLWINNER_D1_ALLWINNER_D1_MEMORYMAP_H */
