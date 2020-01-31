/************************************************************************************
 * arch/arm/src/am335x/hardware/am335x_uart.h
 *
 *   Copyright (C) 2018 Petro Karashchenko. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchenko@gmail.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#ifndef __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_UART_H
#define __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/am335x_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AM335X_UART_RBR_OFFSET       0x0000 /* UART Receive Buffer Register */
#define AM335X_UART_THR_OFFSET       0x0000 /* UART Transmit Holding Register */
#define AM335X_UART_DLL_OFFSET       0x0000 /* UART Divisor Latch Low Register */
#define AM335X_UART_DLH_OFFSET       0x0004 /* UART Divisor Latch High Register */
#define AM335X_UART_IER_OFFSET       0x0004 /* UART Interrupt Enable Register */
#define AM335X_UART_IIR_OFFSET       0x0008 /* UART Interrupt Identity Register */
#define AM335X_UART_FCR_OFFSET       0x0008 /* UART FIFO Control Register */
#define AM335X_UART_EFR_OFFSET       0x0008 /* UART Extended Feature Register */
#define AM335X_UART_LCR_OFFSET       0x000c /* UART Line Control Register */
#define AM335X_UART_MCR_OFFSET       0x0010 /* UART Modem Control Register */
#define AM335X_UART_LSR_OFFSET       0x0014 /* UART Line Status Register */
#define AM335X_UART_MSR_OFFSET       0x0018 /* UART Modem Status Register */
#define AM335X_UART_TCR_OFFSET       0x0018 /* UART Transmission Control Register */
#define AM335X_UART_SPR_OFFSET       0x001c /* UART Scratch Register */
#define AM335X_UART_TLR_OFFSET       0x001c /* UART Trigger Level Register */
#define AM335X_UART_MDR1_OFFSET      0x0020 /* UART Mode Definition Register 1 */
#define AM335X_UART_MDR2_OFFSET      0x0024 /* UART Mode Definition Register 2 */
#define AM335X_UART_SCR_OFFSET       0x0040 /* UART Supplementary Control Register */
#define AM335X_UART_SSR_OFFSET       0x0044 /* UART Supplementary Status Register */
#define AM335X_UART_SYSC_OFFSET      0x0054 /* UART System Configuration Register */
#define AM335X_UART_SYSS_OFFSET      0x0058 /* UART System Status Register */
#define AM335X_UART_RFL_OFFSET       0x0064 /* UART Received FIFO Level Register */
#define AM335X_UART_TFL_OFFSET       0x0068 /* UART Transmit FIFO Level Register */


/* Register virtual addresses *******************************************************/

#define AM335X_UART_RBR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_RBR_OFFSET)
#define AM335X_UART_THR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_THR_OFFSET)
#define AM335X_UART_DLL(n)           (AM335X_UART_VADDR(n) + AM335X_UART_DLL_OFFSET)
#define AM335X_UART_DLH(n)           (AM335X_UART_VADDR(n) + AM335X_UART_DLH_OFFSET)
#define AM335X_UART_IER(n)           (AM335X_UART_VADDR(n) + AM335X_UART_IER_OFFSET)
#define AM335X_UART_IIR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_IIR_OFFSET)
#define AM335X_UART_FCR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_FCR_OFFSET)
#define AM335X_UART_EFR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_EFR_OFFSET)
#define AM335X_UART_LCR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_LCR_OFFSET)
#define AM335X_UART_MCR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_MCR_OFFSET)
#define AM335X_UART_LSR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_LSR_OFFSET)
#define AM335X_UART_MSR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_MSR_OFFSET)
#define AM335X_UART_TCR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_TCR_OFFSET)
#define AM335X_UART_SPR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_SPR_OFFSET)
#define AM335X_UART_TLR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_TLR_OFFSET)
#define AM335X_UART_MDR1(n)          (AM335X_UART_VADDR(n) + AM335X_UART_MDR1_OFFSET)
#define AM335X_UART_MDR2(n)          (AM335X_UART_VADDR(n) + AM335X_UART_MDR2_OFFSET)
#define AM335X_UART_SCR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_SCR_OFFSET)
#define AM335X_UART_SSR(n)           (AM335X_UART_VADDR(n) + AM335X_UART_SSR_OFFSET)
#define AM335X_UART_SYSC(n)          (AM335X_UART_VADDR(n) + AM335X_UART_SYSC_OFFSET)
#define AM335X_UART_SYSS(n)          (AM335X_UART_VADDR(n) + AM335X_UART_SYSS_OFFSET)
#define AM335X_UART_RFL(n)           (AM335X_UART_VADDR(n) + AM335X_UART_RFL_OFFSET)
#define AM335X_UART_TFL(n)           (AM335X_UART_VADDR(n) + AM335X_UART_TFL_OFFSET)

#define AM335X_UART0_RBR             (AM335X_UART0_VADDR + AM335X_UART_RBR_OFFSET)
#define AM335X_UART0_THR             (AM335X_UART0_VADDR + AM335X_UART_THR_OFFSET)
#define AM335X_UART0_DLL             (AM335X_UART0_VADDR + AM335X_UART_DLL_OFFSET)
#define AM335X_UART0_DLH             (AM335X_UART0_VADDR + AM335X_UART_DLH_OFFSET)
#define AM335X_UART0_IER             (AM335X_UART0_VADDR + AM335X_UART_IER_OFFSET)
#define AM335X_UART0_IIR             (AM335X_UART0_VADDR + AM335X_UART_IIR_OFFSET)
#define AM335X_UART0_FCR             (AM335X_UART0_VADDR + AM335X_UART_FCR_OFFSET)
#define AM335X_UART0_EFR             (AM335X_UART0_VADDR + AM335X_UART_EFR_OFFSET)
#define AM335X_UART0_LCR             (AM335X_UART0_VADDR + AM335X_UART_LCR_OFFSET)
#define AM335X_UART0_MCR             (AM335X_UART0_VADDR + AM335X_UART_MCR_OFFSET)
#define AM335X_UART0_LSR             (AM335X_UART0_VADDR + AM335X_UART_LSR_OFFSET)
#define AM335X_UART0_MSR             (AM335X_UART0_VADDR + AM335X_UART_MSR_OFFSET)
#define AM335X_UART0_TCR             (AM335X_UART0_VADDR + AM335X_UART_TCR_OFFSET)
#define AM335X_UART0_SPR             (AM335X_UART0_VADDR + AM335X_UART_SPR_OFFSET)
#define AM335X_UART0_TLR             (AM335X_UART0_VADDR + AM335X_UART_TLR_OFFSET)
#define AM335X_UART0_MDR1            (AM335X_UART0_VADDR + AM335X_UART_MDR1_OFFSET)
#define AM335X_UART0_MDR2            (AM335X_UART0_VADDR + AM335X_UART_MDR2_OFFSET)
#define AM335X_UART0_SCR             (AM335X_UART0_VADDR + AM335X_UART_SCR_OFFSET)
#define AM335X_UART0_SSR             (AM335X_UART0_VADDR + AM335X_UART_SSR_OFFSET)
#define AM335X_UART0_SYSC            (AM335X_UART0_VADDR + AM335X_UART_SYSC_OFFSET)
#define AM335X_UART0_SYSS            (AM335X_UART0_VADDR + AM335X_UART_SYSS_OFFSET)
#define AM335X_UART0_RFL             (AM335X_UART0_VADDR + AM335X_UART_RFL_OFFSET)
#define AM335X_UART0_TFL             (AM335X_UART0_VADDR + AM335X_UART_TFL_OFFSET)

#define AM335X_UART1_RBR             (AM335X_UART1_VADDR + AM335X_UART_RBR_OFFSET)
#define AM335X_UART1_THR             (AM335X_UART1_VADDR + AM335X_UART_THR_OFFSET)
#define AM335X_UART1_DLL             (AM335X_UART1_VADDR + AM335X_UART_DLL_OFFSET)
#define AM335X_UART1_DLH             (AM335X_UART1_VADDR + AM335X_UART_DLH_OFFSET)
#define AM335X_UART1_IER             (AM335X_UART1_VADDR + AM335X_UART_IER_OFFSET)
#define AM335X_UART1_IIR             (AM335X_UART1_VADDR + AM335X_UART_IIR_OFFSET)
#define AM335X_UART1_FCR             (AM335X_UART1_VADDR + AM335X_UART_FCR_OFFSET)
#define AM335X_UART1_EFR             (AM335X_UART1_VADDR + AM335X_UART_EFR_OFFSET)
#define AM335X_UART1_LCR             (AM335X_UART1_VADDR + AM335X_UART_LCR_OFFSET)
#define AM335X_UART1_MCR             (AM335X_UART1_VADDR + AM335X_UART_MCR_OFFSET)
#define AM335X_UART1_LSR             (AM335X_UART1_VADDR + AM335X_UART_LSR_OFFSET)
#define AM335X_UART1_MSR             (AM335X_UART1_VADDR + AM335X_UART_MSR_OFFSET)
#define AM335X_UART1_TCR             (AM335X_UART1_VADDR + AM335X_UART_TCR_OFFSET)
#define AM335X_UART1_SPR             (AM335X_UART1_VADDR + AM335X_UART_SPR_OFFSET)
#define AM335X_UART1_TLR             (AM335X_UART1_VADDR + AM335X_UART_TLR_OFFSET)
#define AM335X_UART1_MDR1            (AM335X_UART1_VADDR + AM335X_UART_MDR1_OFFSET)
#define AM335X_UART1_MDR2            (AM335X_UART1_VADDR + AM335X_UART_MDR2_OFFSET)
#define AM335X_UART1_SCR             (AM335X_UART1_VADDR + AM335X_UART_SCR_OFFSET)
#define AM335X_UART1_SSR             (AM335X_UART1_VADDR + AM335X_UART_SSR_OFFSET)
#define AM335X_UART1_SYSC            (AM335X_UART1_VADDR + AM335X_UART_SYSC_OFFSET)
#define AM335X_UART1_SYSS            (AM335X_UART1_VADDR + AM335X_UART_SYSS_OFFSET)
#define AM335X_UART1_RFL             (AM335X_UART1_VADDR + AM335X_UART_RFL_OFFSET)
#define AM335X_UART1_TFL             (AM335X_UART1_VADDR + AM335X_UART_TFL_OFFSET)

#define AM335X_UART2_RBR             (AM335X_UART2_VADDR + AM335X_UART_RBR_OFFSET)
#define AM335X_UART2_THR             (AM335X_UART2_VADDR + AM335X_UART_THR_OFFSET)
#define AM335X_UART2_DLL             (AM335X_UART2_VADDR + AM335X_UART_DLL_OFFSET)
#define AM335X_UART2_DLH             (AM335X_UART2_VADDR + AM335X_UART_DLH_OFFSET)
#define AM335X_UART2_IER             (AM335X_UART2_VADDR + AM335X_UART_IER_OFFSET)
#define AM335X_UART2_IIR             (AM335X_UART2_VADDR + AM335X_UART_IIR_OFFSET)
#define AM335X_UART2_FCR             (AM335X_UART2_VADDR + AM335X_UART_FCR_OFFSET)
#define AM335X_UART2_EFR             (AM335X_UART2_VADDR + AM335X_UART_EFR_OFFSET)
#define AM335X_UART2_LCR             (AM335X_UART2_VADDR + AM335X_UART_LCR_OFFSET)
#define AM335X_UART2_MCR             (AM335X_UART2_VADDR + AM335X_UART_MCR_OFFSET)
#define AM335X_UART2_LSR             (AM335X_UART2_VADDR + AM335X_UART_LSR_OFFSET)
#define AM335X_UART2_MSR             (AM335X_UART2_VADDR + AM335X_UART_MSR_OFFSET)
#define AM335X_UART2_TCR             (AM335X_UART2_VADDR + AM335X_UART_TCR_OFFSET)
#define AM335X_UART2_SPR             (AM335X_UART2_VADDR + AM335X_UART_SPR_OFFSET)
#define AM335X_UART2_TLR             (AM335X_UART2_VADDR + AM335X_UART_TLR_OFFSET)
#define AM335X_UART2_MDR1            (AM335X_UART2_VADDR + AM335X_UART_MDR1_OFFSET)
#define AM335X_UART2_MDR2            (AM335X_UART2_VADDR + AM335X_UART_MDR2_OFFSET)
#define AM335X_UART2_SCR             (AM335X_UART2_VADDR + AM335X_UART_SCR_OFFSET)
#define AM335X_UART2_SSR             (AM335X_UART2_VADDR + AM335X_UART_SSR_OFFSET)
#define AM335X_UART2_SYSC            (AM335X_UART2_VADDR + AM335X_UART_SYSC_OFFSET)
#define AM335X_UART2_SYSS            (AM335X_UART2_VADDR + AM335X_UART_SYSS_OFFSET)
#define AM335X_UART2_RFL             (AM335X_UART2_VADDR + AM335X_UART_RFL_OFFSET)
#define AM335X_UART2_TFL             (AM335X_UART2_VADDR + AM335X_UART_TFL_OFFSET)

#define AM335X_UART3_RBR             (AM335X_UART3_VADDR + AM335X_UART_RBR_OFFSET)
#define AM335X_UART3_THR             (AM335X_UART3_VADDR + AM335X_UART_THR_OFFSET)
#define AM335X_UART3_DLL             (AM335X_UART3_VADDR + AM335X_UART_DLL_OFFSET)
#define AM335X_UART3_DLH             (AM335X_UART3_VADDR + AM335X_UART_DLH_OFFSET)
#define AM335X_UART3_IER             (AM335X_UART3_VADDR + AM335X_UART_IER_OFFSET)
#define AM335X_UART3_IIR             (AM335X_UART3_VADDR + AM335X_UART_IIR_OFFSET)
#define AM335X_UART3_FCR             (AM335X_UART3_VADDR + AM335X_UART_FCR_OFFSET)
#define AM335X_UART3_EFR             (AM335X_UART3_VADDR + AM335X_UART_EFR_OFFSET)
#define AM335X_UART3_LCR             (AM335X_UART3_VADDR + AM335X_UART_LCR_OFFSET)
#define AM335X_UART3_MCR             (AM335X_UART3_VADDR + AM335X_UART_MCR_OFFSET)
#define AM335X_UART3_LSR             (AM335X_UART3_VADDR + AM335X_UART_LSR_OFFSET)
#define AM335X_UART3_MSR             (AM335X_UART3_VADDR + AM335X_UART_MSR_OFFSET)
#define AM335X_UART3_TCR             (AM335X_UART3_VADDR + AM335X_UART_TCR_OFFSET)
#define AM335X_UART3_SPR             (AM335X_UART3_VADDR + AM335X_UART_SPR_OFFSET)
#define AM335X_UART3_TLR             (AM335X_UART3_VADDR + AM335X_UART_TLR_OFFSET)
#define AM335X_UART3_MDR1            (AM335X_UART3_VADDR + AM335X_UART_MDR1_OFFSET)
#define AM335X_UART3_MDR2            (AM335X_UART3_VADDR + AM335X_UART_MDR2_OFFSET)
#define AM335X_UART3_SCR             (AM335X_UART3_VADDR + AM335X_UART_SCR_OFFSET)
#define AM335X_UART3_SSR             (AM335X_UART3_VADDR + AM335X_UART_SSR_OFFSET)
#define AM335X_UART3_SYSC            (AM335X_UART3_VADDR + AM335X_UART_SYSC_OFFSET)
#define AM335X_UART3_SYSS            (AM335X_UART3_VADDR + AM335X_UART_SYSS_OFFSET)
#define AM335X_UART3_RFL             (AM335X_UART3_VADDR + AM335X_UART_RFL_OFFSET)
#define AM335X_UART3_TFL             (AM335X_UART3_VADDR + AM335X_UART_TFL_OFFSET)

#define AM335X_UART4_RBR             (AM335X_UART4_VADDR + AM335X_UART_RBR_OFFSET)
#define AM335X_UART4_THR             (AM335X_UART4_VADDR + AM335X_UART_THR_OFFSET)
#define AM335X_UART4_DLL             (AM335X_UART4_VADDR + AM335X_UART_DLL_OFFSET)
#define AM335X_UART4_DLH             (AM335X_UART4_VADDR + AM335X_UART_DLH_OFFSET)
#define AM335X_UART4_IER             (AM335X_UART4_VADDR + AM335X_UART_IER_OFFSET)
#define AM335X_UART4_IIR             (AM335X_UART4_VADDR + AM335X_UART_IIR_OFFSET)
#define AM335X_UART4_FCR             (AM335X_UART4_VADDR + AM335X_UART_FCR_OFFSET)
#define AM335X_UART4_EFR             (AM335X_UART4_VADDR + AM335X_UART_EFR_OFFSET)
#define AM335X_UART4_LCR             (AM335X_UART4_VADDR + AM335X_UART_LCR_OFFSET)
#define AM335X_UART4_MCR             (AM335X_UART4_VADDR + AM335X_UART_MCR_OFFSET)
#define AM335X_UART4_LSR             (AM335X_UART4_VADDR + AM335X_UART_LSR_OFFSET)
#define AM335X_UART4_MSR             (AM335X_UART4_VADDR + AM335X_UART_MSR_OFFSET)
#define AM335X_UART4_TCR             (AM335X_UART4_VADDR + AM335X_UART_TCR_OFFSET)
#define AM335X_UART4_SPR             (AM335X_UART4_VADDR + AM335X_UART_SPR_OFFSET)
#define AM335X_UART4_TLR             (AM335X_UART4_VADDR + AM335X_UART_TLR_OFFSET)
#define AM335X_UART4_MDR1            (AM335X_UART4_VADDR + AM335X_UART_MDR1_OFFSET)
#define AM335X_UART4_MDR2            (AM335X_UART4_VADDR + AM335X_UART_MDR2_OFFSET)
#define AM335X_UART4_SCR             (AM335X_UART4_VADDR + AM335X_UART_SCR_OFFSET)
#define AM335X_UART4_SSR             (AM335X_UART4_VADDR + AM335X_UART_SSR_OFFSET)
#define AM335X_UART4_SYSC            (AM335X_UART4_VADDR + AM335X_UART_SYSC_OFFSET)
#define AM335X_UART4_SYSS            (AM335X_UART4_VADDR + AM335X_UART_SYSS_OFFSET)
#define AM335X_UART4_RFL             (AM335X_UART4_VADDR + AM335X_UART_RFL_OFFSET)
#define AM335X_UART4_TFL             (AM335X_UART4_VADDR + AM335X_UART_TFL_OFFSET)

#define AM335X_UART5_RBR             (AM335X_UART5_VADDR + AM335X_UART_RBR_OFFSET)
#define AM335X_UART5_THR             (AM335X_UART5_VADDR + AM335X_UART_THR_OFFSET)
#define AM335X_UART5_DLL             (AM335X_UART5_VADDR + AM335X_UART_DLL_OFFSET)
#define AM335X_UART5_DLH             (AM335X_UART5_VADDR + AM335X_UART_DLH_OFFSET)
#define AM335X_UART5_IER             (AM335X_UART5_VADDR + AM335X_UART_IER_OFFSET)
#define AM335X_UART5_IIR             (AM335X_UART5_VADDR + AM335X_UART_IIR_OFFSET)
#define AM335X_UART5_FCR             (AM335X_UART5_VADDR + AM335X_UART_FCR_OFFSET)
#define AM335X_UART5_EFR             (AM335X_UART5_VADDR + AM335X_UART_EFR_OFFSET)
#define AM335X_UART5_LCR             (AM335X_UART5_VADDR + AM335X_UART_LCR_OFFSET)
#define AM335X_UART5_MCR             (AM335X_UART5_VADDR + AM335X_UART_MCR_OFFSET)
#define AM335X_UART5_LSR             (AM335X_UART5_VADDR + AM335X_UART_LSR_OFFSET)
#define AM335X_UART5_MSR             (AM335X_UART5_VADDR + AM335X_UART_MSR_OFFSET)
#define AM335X_UART5_TCR             (AM335X_UART5_VADDR + AM335X_UART_TCR_OFFSET)
#define AM335X_UART5_SPR             (AM335X_UART5_VADDR + AM335X_UART_SPR_OFFSET)
#define AM335X_UART5_TLR             (AM335X_UART5_VADDR + AM335X_UART_TLR_OFFSET)
#define AM335X_UART5_MDR1            (AM335X_UART5_VADDR + AM335X_UART_MDR1_OFFSET)
#define AM335X_UART5_MDR2            (AM335X_UART5_VADDR + AM335X_UART_MDR2_OFFSET)
#define AM335X_UART5_SCR             (AM335X_UART5_VADDR + AM335X_UART_SCR_OFFSET)
#define AM335X_UART5_SSR             (AM335X_UART5_VADDR + AM335X_UART_SSR_OFFSET)
#define AM335X_UART5_SYSC            (AM335X_UART5_VADDR + AM335X_UART_SYSC_OFFSET)
#define AM335X_UART5_SYSS            (AM335X_UART5_VADDR + AM335X_UART_SYSS_OFFSET)
#define AM335X_UART5_RFL             (AM335X_UART5_VADDR + AM335X_UART_RFL_OFFSET)
#define AM335X_UART5_TFL             (AM335X_UART5_VADDR + AM335X_UART_TFL_OFFSET)

/* Register bit field definitions ***************************************************/

/* UART Receive Buffer Register */

#define UART_RBR_MASK                (0x000000ff)

/* UART Transmit Holding Register */

#define UART_THR_MASK                (0x000000ff)

/* UART Divisor Latch Low Register */

#define UART_DLL_MASK                (0x000000ff)

/* UART Divisor Latch High Register */

#define UART_DLH_MASK                (0x0000003f)

/* UART Interrupt Enable Register */

#define UART_IER_RHR_CTI             (1 << 0)  /* Bit 0:  Enable Received Data Available Interrupt */
#define UART_IER_THR                 (1 << 1)  /* Bit 1:  Enable Transmit Holding Register Empty Interrupt */
#define UART_IER_LINE_STS            (1 << 2)  /* Bit 2:  Enable Receiver Line Status Interrupt */
#define UART_IER_MODEM_STS           (1 << 3)  /* Bit 3:  Enable Modem Status Interrupt */
#define UART_IER_RTS                 (1 << 6)  /* Bit 6:  Enable RTS (active-low) interrupt */
#define UART_IER_CTS                 (1 << 7)  /* Bit 7:  Enable CTS (active-low) interrupt */
#define UART_IER_ALLIE               (0x000000cf)

/* UART Interrupt Identification Register */

#define UART_IIR_IID_SHIFT           (0)
#define UART_IIR_IID_MASK            (63 << UART_IIR_IID_SHIFT)
#  define UART_IIR_IID_MODEM         (0 << UART_IIR_IID_SHIFT)
#  define UART_IIR_IID_NONE          (1 << UART_IIR_IID_SHIFT)  /* No interrupt pending */
#  define UART_IIR_IID_THR           (2 << UART_IIR_IID_SHIFT)
#  define UART_IIR_IID_RHR           (4 << UART_IIR_IID_SHIFT)
#  define UART_IIR_IID_RXSTATUS      (6 << UART_IIR_IID_SHIFT)
#  define UART_IIR_IID_RXTIMEOUT     (12 << UART_IIR_IID_SHIFT)
#  define UART_IIR_IID_XOFF          (16 << UART_IIR_IID_SHIFT)
#  define UART_IIR_IID_STATECHANGE   (32 << UART_IIR_IID_SHIFT)
#define UART_IIR_FEFLAG_SHIFT        (6)  /* Bits 6-7: FIFOs Enable Flag */
#define UART_IIR_FEFLAG_MASK         (3 << UART_IIR_FEFLAG_SHIFT)
#  define UART_IIR_FEFLAG_DISABLE    (0 << UART_IIR_FEFLAG_SHIFT)
#  define UART_IIR_FEFLAG_ENABLE     (3 << UART_IIR_FEFLAG_SHIFT)

/* UART FIFO Control Register */

#define UART_FCR_FIFO_EN             (1 << 0)  /* Bit 0: Enable TX and RX FIFOs */
#define UART_FCR_RFIFO_CLEAR         (1 << 1)  /* Bit 1: Clear RX FIFO */
#define UART_FCR_TFIFO_CLEAR         (1 << 2)  /* Bit 2: Clear TX FIFO */
#define UART_FCR_DMA_MODE            (1 << 3)  /* Bit 3: DMA Mode */
#define UART_FCR_TFT_SHIFT           (4)  /* Bits 4-5: TX FIFO Trigger Level */
#define UART_FCR_TFT_MASK            (3 << UART_FCR_TFT_SHIFT)
#  define UART_FCR_TFT_8CHAR         (0 << UART_FCR_TFT_SHIFT)  /* 8 Chars in FIFO */
#  define UART_FCR_TFT_16CHAR        (1 << UART_FCR_TFT_SHIFT)  /* 16 Chars in FIFO */
#  define UART_FCR_TFT_32CHAR        (2 << UART_FCR_TFT_SHIFT)  /* 32 Chars in FIFO */
#  define UART_FCR_TFT_56CHAR        (3 << UART_FCR_TFT_SHIFT)  /* 56 Chars in FIFO */
#define UART_FCR_RFT_SHIFT           (6)  /* Bits 6-7: RX FIFO Trigger Level */
#define UART_FCR_RFT_MASK            (3 << UART_FCR_RFT_SHIFT)
#  define UART_FCR_RFT_8CHAR         (0 << UART_FCR_RFT_SHIFT)  /* 8 Chars in FIFO */
#  define UART_FCR_RFT_16CHAR        (1 << UART_FCR_RFT_SHIFT)  /* 16 Chars in FIFO */
#  define UART_FCR_RFT_56CHAR        (2 << UART_FCR_RFT_SHIFT)  /* 56 Chars in FIFO */
#  define UART_FCR_RFT_60CHAR        (3 << UART_FCR_RFT_SHIFT)  /* 60 Chars in FIFO */

/* UART Line Control Register */

#define UART_LCR_DLS_SHIFT           (0)  /* Bits 0-1: Data Length Select */
#define UART_LCR_DLS_MASK            (3 << UART_LCR_DLS_SHIFT)
#  define UART_LCR_DLS_5BITS         (0 << UART_LCR_DLS_SHIFT)  /* 5 Bits */
#  define UART_LCR_DLS_6BITS         (1 << UART_LCR_DLS_SHIFT)  /* 6 Bits */
#  define UART_LCR_DLS_7BITS         (2 << UART_LCR_DLS_SHIFT)  /* 7 Bits */
#  define UART_LCR_DLS_8BITS         (3 << UART_LCR_DLS_SHIFT)  /* 8 Bits */
#define UART_LCR_STOP_SHIFT          (2)  /* Bit 2:  Number of Stop Bits */
#  define UART_LCR_STOP_1BITS        (0 << UART_LCR_STOP_SHIFT)  /* 1 Stop Bit */
#  define UART_LCR_STOP_2BITS        (1 << UART_LCR_STOP_SHIFT)  /* 2 Stop Bits */
#define UART_LCR_PEN                 (1 << 3)  /* Bit 3:  Parity Enable */
#define UART_LCR_PARITY_SHIFT        (3)  /* Bit 3-4:  Parity Enable and Parity Select */
#  define UART_LCR_PARITY_NONE       (0 << UART_LCR_PARITY_SHIFT)  /* No Parity */
#  define UART_LCR_PARITY_ODD        (1 << UART_LCR_PARITY_SHIFT)  /* Odd Parity Bit */
#  define UART_LCR_PARITY_EVEN       (3 << UART_LCR_PARITY_SHIFT)  /* Even Parity Bit */
#define UART_LCR_BC                  (1 << 6)  /* Bit 6:  Break Control Bit */
#define UART_LCR_DLAB                (1 << 7)  /* Bit 7:  Divisor Latch Access Enable Bit */
#define UART_LCR_CONFIG_MODE_A       (0x00000080)
#define UART_LCR_CONFIG_MODE_B       (0x000000bf)
#define UART_LCR_OPER_MODE           (0x0000007f)

/* UART Modem Control Register */

#define UART_MCR_DTR                 (1 << 0)  /* Bit 0:  Data Terminal Ready */
#define UART_MCR_RTS                 (1 << 1)  /* Bit 1:  Request to Send */
#define UART_MCR_LOOP                (1 << 4)  /* Bit 4:  Enable Loop Back Mode */

/* UART Line Status Register */

#define UART_LSR_DR                  (1 << 0)  /* Bit 0:  Data Ready */
#define UART_LSR_OE                  (1 << 1)  /* Bit 1:  Overrun Error */
#define UART_LSR_PE                  (1 << 2)  /* Bit 2:  Parity Error */
#define UART_LSR_FE                  (1 << 3)  /* Bit 3:  Framing Error */
#define UART_LSR_BI                  (1 << 4)  /* Bit 4:  Break Interrupt */
#define UART_LSR_THRE                (1 << 5)  /* Bit 5:  TX Holding Register Empty */
#define UART_LSR_TEMT                (1 << 6)  /* Bit 6:  Transmitter Empty */
#define UART_LSR_FIFOERR             (1 << 7)  /* Bit 7:  RX Data Error in FIFO */

/* UART Modem Status Register */

#define UART_MSR_CTS                 (1 << 0)  /* Bit 0:  Line State of Clear to Send */
#define UART_MSR_DSR                 (1 << 1)  /* Bit 1:  Line State of Data Set Ready */
#define UART_MSR_RI                  (1 << 2)  /* Bit 2:  Line State of Ring Indicator */
#define UART_MSR_DCD                 (1 << 3)  /* Bit 3:  Line State of Data Carrier Detect */
#define UART_MSR_NCTS                (1 << 4)  /* Bit 4:  Complement of Clear To Send */
#define UART_MSR_NDSR                (1 << 5)  /* Bit 5:  Complement of Data Set Ready */
#define UART_MSR_NRI                 (1 << 6)  /* Bit 6:  Complement of Ring Indicator */
#define UART_MSR_NDCD                (1 << 7)  /* Bit 7:  Complement of Data Carrier Detect */

/* UART Scratchpad Register */

#define UART_SPR_MASK                (0x000000ff)

/* UART Transmit FIFO Level */

#define UART_TFL_SHIFT               (0)  /* Bits 0-7: Transmit FIFO Level */
#define UART_TFL_MASK                (0xff << UART_TFL_SHIFT)
#  define UART_TFL(n)                ((uint32_t)(n) << UART_TFL_SHIFT)

/* UART Receive FIFO Level */

#define UART_RFL_SHIFT               (0)  /* Bits 0-7: Receive FIFO Level */
#define UART_RFL_MASK                (0xff << UART_RFL_SHIFT)
#  define UART_RFL(n)                ((uint32_t)(n) << UART_RFL_SHIFT)

/* UART Mode Definition 1 Register */

#define UART_MDR1_MODE_SHIFT         (0)  /* Bits 0-2: Operation Mode Selection */
#  define UART_MDR1_MODE_16X         (0 << UART_MDR1_MODE_SHIFT)  /* UART 16x Mode. */
#  define UART_MDR1_MODE_SIR         (1 << UART_MDR1_MODE_SHIFT)  /* SIR mode */
#  define UART_MDR1_MODE_16XAUTO     (2 << UART_MDR1_MODE_SHIFT)  /* UART 16x Auto-Baud */
#  define UART_MDR1_MODE_13X         (3 << UART_MDR1_MODE_SHIFT)  /* UART 13x mode */
#  define UART_MDR1_MODE_MIR         (4 << UART_MDR1_MODE_SHIFT)  /* MIR Mode */
#  define UART_MDR1_MODE_FIR         (5 << UART_MDR1_MODE_SHIFT)  /* FIR Mode */
#  define UART_MDR1_MODE_CIR         (6 << UART_MDR1_MODE_SHIFT)  /* CIR Mode */
#  define UART_MDR1_MODE_DISABLE     (7 << UART_MDR1_MODE_SHIFT)  /* Disabled Mode */

/* UART System Configuration Register */

#define UART_SYSC_SRESET             (1 << 1)  /* Bit 1: Software Reset */
#define UART_SYSC_WAKEUP             (1 << 2)  /* Bit 2: Wake-up Control */

/* UART System Status Register */

#define UART_SYSS_RESET_DONE         (1 << 0)  /* Bit 0: Reset Complete */

/* UART Enhanced Feature Register */

#define UART_EFR_ENHANCEDEN          (1 << 4)  /* Bit 4: Enable Enhanced Functions Write */
#define UART_EFR_AUTORTSEN           (1 << 6)  /* Bit 6: Enable Auto-RTS */
#define UART_EFR_AUTOCTSEN           (1 << 7)  /* Bit 7: Enable Auto-CTS */

#endif /* __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_UART_H */
