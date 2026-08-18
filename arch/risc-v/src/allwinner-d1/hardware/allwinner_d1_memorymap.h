/****************************************************************************
 * arch/risc-v/src/allwinner-d1/hardware/allwinner_d1_memorymap.h
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ALLWINNER_D1_HARDWARE_ALLWINNER_D1_MEMORYMAP_H
#define __ARCH_RISCV_SRC_ALLWINNER_D1_HARDWARE_ALLWINNER_D1_MEMORYMAP_H

#define ALLWINNER_D1_PIO_BASE       0x02000000ul
#define ALLWINNER_D1_CCU_BASE       0x02001000ul
#define ALLWINNER_D1_TIMER_BASE     0x02050000ul
#define ALLWINNER_D1_UART0_BASE     0x02500000ul
#define ALLWINNER_D1_RISCV_WDT_BASE 0x06011000ul
#define ALLWINNER_D1_PLIC_BASE      0x10000000ul
#define ALLWINNER_D1_DRAM_BASE      0x40000000ul

#define ALLWINNER_D1_PB_CFG1        (ALLWINNER_D1_PIO_BASE + 0x34)
#define ALLWINNER_D1_UART_BGR       (ALLWINNER_D1_CCU_BASE + 0x90c)

#endif /* __ARCH_RISCV_SRC_ALLWINNER_D1_HARDWARE_ALLWINNER_D1_MEMORYMAP_H */
