/****************************************************************************
 * arch/risc-v/src/allwinner-d1/hardware/allwinner_d1_timer.h
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ALLWINNER_D1_HARDWARE_ALLWINNER_D1_TIMER_H
#define __ARCH_RISCV_SRC_ALLWINNER_D1_HARDWARE_ALLWINNER_D1_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "allwinner_d1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALLWINNER_D1_TIMER_IRQ_EN       (ALLWINNER_D1_TIMER_BASE + 0x00)
#define ALLWINNER_D1_TIMER_IRQ_STATUS   (ALLWINNER_D1_TIMER_BASE + 0x04)

#define ALLWINNER_D1_TIMER_CTRL(n) \
  (ALLWINNER_D1_TIMER_BASE + 0x10 * (n) + 0x10)
#define ALLWINNER_D1_TIMER_INTERVAL(n) \
  (ALLWINNER_D1_TIMER_BASE + 0x10 * (n) + 0x14)
#define ALLWINNER_D1_TIMER_CURRENT(n) \
  (ALLWINNER_D1_TIMER_BASE + 0x10 * (n) + 0x18)

#define ALLWINNER_D1_TIMER_IRQ(n)            (1u << (n))
#define ALLWINNER_D1_TIMER_CTRL_ENABLE       (1u << 0)
#define ALLWINNER_D1_TIMER_CTRL_RELOAD       (1u << 1)
#define ALLWINNER_D1_TIMER_CTRL_CLK_SRC_MASK (3u << 2)
#define ALLWINNER_D1_TIMER_CTRL_CLK_OSC24M   (1u << 2)
#define ALLWINNER_D1_TIMER_CTRL_PRES_MASK    (7u << 4)
#define ALLWINNER_D1_TIMER_CTRL_PRES_DIV1    (0u << 4)
#define ALLWINNER_D1_TIMER_CTRL_ONESHOT      (1u << 7)

#endif /* __ARCH_RISCV_SRC_ALLWINNER_D1_HARDWARE_ALLWINNER_D1_TIMER_H */
