/****************************************************************************
 * arch/risc-v/src/fe310/hardware/fe310_memorymap.h
 *
 *   Copyright (C) 2019 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
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
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_MEMORYMAP_H
#define __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Address ****************************************************/

#define FE310_CLINT_BASE   0x02000000
#define FE310_PLIC_BASE    0x0c000000

#define FE310_PRCI_BASE    0x10008000  /* 0x10008000 - 0x10008fff: PRCI  */

#define FE310_GPIO_BASE    0x10012000  /* 0x10012000 - 0x10012fff: GPIO  */
#define FE310_UART0_BASE   0x10013000  /* 0x10013000 - 0x10013fff: UART0 */
#define FE310_QSPI0_BASE   0x10014000  /* 0x10014000 - 0x10014fff: QSPI0 */
#define FE310_UART1_BASE   0x10023000  /* 0x10023000 - 0x10023fff: UART1 */

#endif /* __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_MEMORYMAP_H */
