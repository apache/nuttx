/****************************************************************************
 * arch/risc-v/src/allwinner-d1/hardware/allwinner_d1_wdt.h
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ALLWINNER_D1_HARDWARE_ALLWINNER_D1_WDT_H
#define __ARCH_RISCV_SRC_ALLWINNER_D1_HARDWARE_ALLWINNER_D1_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "allwinner_d1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALLWINNER_D1_RISCV_WDT_MODE \
  (ALLWINNER_D1_RISCV_WDT_BASE + 0x18)
#define ALLWINNER_D1_RISCV_WDT_DISABLE_KEY 0x16aa0000u

#endif /* __ARCH_RISCV_SRC_ALLWINNER_D1_HARDWARE_ALLWINNER_D1_WDT_H */
