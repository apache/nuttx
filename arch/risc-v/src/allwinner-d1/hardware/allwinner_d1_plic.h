/****************************************************************************
 * arch/risc-v/src/allwinner-d1/hardware/allwinner_d1_plic.h
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ALLWINNER_D1_HARDWARE_ALLWINNER_D1_PLIC_H
#define __ARCH_RISCV_SRC_ALLWINNER_D1_HARDWARE_ALLWINNER_D1_PLIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "allwinner_d1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALLWINNER_D1_PLIC_NDEV       175
#define ALLWINNER_D1_PLIC_PRIORITY   (ALLWINNER_D1_PLIC_BASE + 0x000000)
#define ALLWINNER_D1_PLIC_ENABLE     (ALLWINNER_D1_PLIC_BASE + 0x002080)
#define ALLWINNER_D1_PLIC_THRESHOLD  (ALLWINNER_D1_PLIC_BASE + 0x201000)
#define ALLWINNER_D1_PLIC_CLAIM      (ALLWINNER_D1_PLIC_BASE + 0x201004)

#endif /* __ARCH_RISCV_SRC_ALLWINNER_D1_HARDWARE_ALLWINNER_D1_PLIC_H */
