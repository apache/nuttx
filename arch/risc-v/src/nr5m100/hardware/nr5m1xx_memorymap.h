/************************************************************************************
 * arch/risc-v/src/nr5m100/hardware/nr5m1xx_memorymap.h
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_RISCV_SRC_NR5M100_HARDWARE_NR5M1XX_MEMORYMAP_H
#define __ARCH_RISCV_SRC_NR5M100_HARDWARE_NR5M1XX_MEMORYMAP_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* BOOT ROM, SRAM, PERIPHERALS ******************************************************/

#define STM32_CODE_BASE     0x00000000      /* 0x00000000-0x1fffffff: 512Mb code block */
#define STM32_SRAM_BASE     0x20000000      /* 0x20000000 - 384Kb SRAM */
#define STM32_PERIPH_BASE   0x40000000      /* 0x40000000-0x4fffffff: Peripheral block */

/* Register Base Address ************************************************************/

#define NR5_UART1_BASE      0x40000000      /* 0x40000000 - 0x40000fff: UART0 */
#define NR5_GPIO1_BASE      0x40001000      /* 0x40001000 - 0x40001fff: GPIO1 */
#define NR5_GPIO2_BASE      0x40002000      /* 0x40002000 - 0x40002fff: GPIO2 */
#define NR5_GPIO3_BASE      0x40003000      /* 0x40003000 - 0x40003fff: GPIO3 */
#define NR5_TIMER1_BASE     0x40004000      /* 0x40004000 - 0x40004fff: TIMER0 timer */
#define NR5_TIMER2_BASE     0x40005000      /* 0x40005000 - 0x40005fff: TIMER1 timer */
#define NR5_EXTMEM_BASE     0x40006000      /* 0x40006000 - 0x40006fff: EXTMEM Controller*/

#endif /* __ARCH_RISCV_SRC_NR5M100_HARDWARE_NR5M1XX_MEMORYMAP_H */
