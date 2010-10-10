/****************************************************************************
 * arch/avr/include/at91uc3/irq.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ****************************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_AVR_INCLUDE_AT91UC3_IRQ_H
#define __ARCH_AVR_INCLUDE_AT91UC3_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
 
/* IRQ numbers */
/* Events.  These exclude:
 *
 * - The Reset event which vectors directly either to 0x8000:0000 (uc3a) or
 *   to 0xa000:0000 (uc3b).
 * - The OCD stop from the OSD system
 * - Autovectored interrupt requests
 *
 * Others vector relative to the contents of the EVBA register.
 */

#define AVR32_IRQ_UNREC         0 /* EVBA+0x00 Unrecoverable exception */
#define AVR32_IRQ_TLBMULT       1 /* EVBA+0x04 TLB multiple hit */
#define AVR32_IRQ_BUSDATA       2 /* EVBA+0x08 Bus error data fetch */
#define AVR32_IRQ_BUSINST       3 /* EVBA+0x0c Bus error instruction fetch */
#define AVR32_IRQ_NMI           4 /* EVBA+0x10 NMI */
#define AVR32_IRQ_INSTADDR      5 /* EVBA+0x14 Instruction Address */
#define AVR32_IRQ_ITLBPROT      6 /* EVBA+0x18 ITLB Protection */
#define AVR32_IRQ_BP            7 /* EVBA+0x1c Breakpoint */
#define AVR32_IRQ_INVINST       8 /* EVBA+0x20 Illegal Opcode */
#define AVR32_IRQ_UNIMPINST     9 /* EVBA+0x24 Unimplemented instruction */
#define AVR32_IRQ_PRIV         10 /* EVBA+0x28 Privilege violation */
#define AVR32_IRQ_FP           11 /* EVBA+0x2c Floating-point */
#define AVR32_IRQ_COP          12 /* EVBA+0x30 Coprocessor absent */
#define AVR32_IRQ_RDDATA       13 /* EVBA+0x34 Data Address (Read) */
#define AVR32_IRQ_WRDATA       14 /* EVBA+0x38 Data Address (Write) */
#define AVR32_IRQ_RDDTLBPROT   15 /* EVBA+0x3c DTLB Protection (Read) */
#define AVR32_IRQ_WRDTLBPROT   16 /* EVBA+0x40 DTLB Protection (Write) */
#define AVR32_IRQ_DLTBMOD      17 /* EVBA+0x44 DTLB Modified */
#define AVR32_IRQ_ITLBMISS     18 /* EVBA+0x50 ITLB Miss */
#define AVR32_IRQ_RDDTLB       19 /* EVBA+0x60 DTLB Miss (Read) */
#define AVR32_IRQ_WRDTLB       20 /* EVBA+0x70 DTLB Miss (Write) */
#define AVR32_IRQ_SUPER        21 /* EVBA+0x100 Supervisor call */
#define AVR32_IRQ_NEVENTS      22

/* "The INTC collects interrupt requests from the peripherals, prioritizes
 *  them, and delivers an interrupt request and an autovector to the CPU. The
 *  AVR32 architecture supports 4 priority levels for regular, maskable
 *  interrupts, and a Non-Maskable Interrupt (NMI)."
 *
 * "The INTC supports up to 64 groups of interrupts. Each group can have up
 *  to 32 interrupt request lines, these lines are connected to the peripherals.
 *  Each group has an Interrupt Priority Register (IPR) and an Interrupt Request
 *  Register (IRR). The IPRs are used to assign a priority level and an autovector
 *  to each group, and the IRRs are used to identify the active interrupt request
 *  within each group. If a group has only one interrupt request line, an active
 *  interrupt group uniquely identifies the active interrupt request line, and
 *  the corresponding IRR is not needed. The INTC also provides one Interrupt
 *  Cause Register (ICR) per priority level. These registers identify the group
 *  that has a pending interrupt of the corresponding priority level. If several
 *  groups have a pending interrupt of the same level, the group with the lowest
 *  number takes priority."
 */

/* Group 0 */

#define AVR32_IRQ_GROUP0       22
#define AVR32_IRQ_UC           22 /* 0 AVR32 UC CPU */

/* Group 1 */

#define AVR32_IRQ_GROUP1       23
#define AVR32_IRQ_EIC0         23 /* 0 External Interrupt Controller 0 */
#define AVR32_IRQ_EIC1         24 /* 1 External Interrupt Controller 1 */
#define AVR32_IRQ_EIC2         25 /* 2 External Interrupt Controller 2 */
#define AVR32_IRQ_EIC3         26 /* 3 External Interrupt Controller 3 */
#define AVR32_IRQ_EIC4         27 /* 4 External Interrupt Controller 4 */
#define AVR32_IRQ_EIC5         28 /* 5 External Interrupt Controller 5 */
#define AVR32_IRQ_EIC6         29 /* 6 External Interrupt Controller 6 */
#define AVR32_IRQ_EIC7         30 /* 7 External Interrupt Controller 7 */
#define AVR32_IRQ_RTC          31 /* 8 Real Time Counter RTC */
#define AVR32_IRQ_PM           32 /* 9 Power Manager PM */

/* Group 2 */

#define AVR32_IRQ_GROUP2       33
#define AVR32_IRQ_GPIO0        33 /* 0 General Purpose Input/Output Controller 0 */
#define AVR32_IRQ_GPIO1        34 /* 1 General Purpose Input/Output Controller 1 */
#define AVR32_IRQ_GPIO2        35 /* 2 General Purpose Input/Output Controller 2 */
#define AVR32_IRQ_GPIO3        36 /* 3 General Purpose Input/Output Controller 3 */
#define AVR32_IRQ_GPIO4        37 /* 4 General Purpose Input/Output Controller 4 */
#define AVR32_IRQ_GPIO5        38 /* 5 General Purpose Input/Output Controller 5 */

/* Group 3 */

#define AVR32_IRQ_GROUP3       39
#define AVR32_IRQ_PDCA0        40 /* 0 Peripheral DMA Controller 0 */
#define AVR32_IRQ_PDCA1        41 /* 1 Peripheral DMA Controller 1 */
#define AVR32_IRQ_PDCA2        42 /* 2 Peripheral DMA Controller 2 */
#define AVR32_IRQ_PDCA3        43 /* 3 Peripheral DMA Controller 3 */
#define AVR32_IRQ_PDCA4        44 /* 4 Peripheral DMA Controller 4 */
#define AVR32_IRQ_PDCA5        45 /* 5 Peripheral DMA Controller 5 */
#define AVR32_IRQ_PDCA6        46 /* 6 Peripheral DMA Controller 6 */

/* Group 4 */

#define AVR32_IRQ_GROUP4       47
#define AVR32_IRQ_FLASHC       47 /* 0 Flash Controller */

/* Group 5 */

#define AVR32_IRQ_GROUP5       48
#define AVR32_IRQ_USART0       48 /* 0 Universal Synchronous/Asynchronous
                                   *   Receiver/Transmitter 0 */
/* Group 6 */

#define AVR32_IRQ_GROUP6       49
#define AVR32_IRQ_USART1       49 /* 0 Universal Synchronous/Asynchronous
                                   *   Receiver/Transmitter 1 */
/* Group 7 */

#define AVR32_IRQ_GROUP7       50
#define AVR32_IRQ_USART2       50 /* 0 Universal Synchronous/Asynchronous
                                   *   Receiver/Transmitter 2 */

#define AVR32_IRQ_GROUP8       51

/* Group 9 */

#define AVR32_IRQ_GROUP9       51
#define AVR32_IRQ_SPI          51 /* 0 Serial Peripheral Interface */

#define AVR32_IRQ_GROUP10      52

/* Group 11 */

#define AVR32_IRQ_GROUP11      52
#define AVR32_IRQ_TWI          52 /* 0 Two-wire Interface TWI */

/* Group 12 */

#define AVR32_IRQ_GROUP12      53
#define AVR32_IRQ_PWM          53 /* 0 Pulse Width Modulation Controller */

/* Group 13 */

#define AVR32_IRQ_GROUP13      54
#define AVR32_IRQ_SSC          54 /* 0 Synchronous Serial Controller */

/* Group 14 */

#define AVR32_IRQ_GROUP14      55
#define AVR32_IRQ_TC0          55 /* 0 Timer/Counter 0 */
#define AVR32_IRQ_TC1          56 /* 1 Timer/Counter 1 */
#define AVR32_IRQ_TC2          57 /* 2 Timer/Counter 2 */

/* Group 15 */

#define AVR32_IRQ_GROUP15      58
#define AVR32_IRQ_ADC          58 /* 0 Analog to Digital Converter */

#define AVR32_IRQ_GROUP16      59

/* Group 17 */

#define AVR32_IRQ_GROUP17      59
#define AVR32_IRQ_USBB         59 /* 0 USB 2.0 Interface USBB */

/* Group 18 */

#define AVR32_IRQ_GROUP18      60
#define AVR32_IRQ_ABDAC        60 /* 0 Audio Bitstream DAC */

#define AVR32_IRQ_GROUP10      61

/* Total number of IRQ numbers */

#define AVR32_IRQ_BADVECTOR    61 /* Not a real IRQ number */
#define NR_IRQS                61

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_AVR_INCLUDE_AT91UC3_IRQ_H */

