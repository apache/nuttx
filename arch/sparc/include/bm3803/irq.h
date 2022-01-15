/****************************************************************************
 * arch/sparc/include/bm3803/irq.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_SPARC_INCLUDE_BM3803_IRQ_H
#define __ARCH_SPARC_INCLUDE_BM3803_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*  Interrupt Sources The interrupt source numbers directly map to the trap
 *  type and to the bits used in the Interrupt Clear, Interrupt Force,
 *  Interrupt Mask,  and the Interrupt Pending Registers.
 */

#if defined(CONFIG_ARCH_CHIP_BM3803)

#define BM3803_IRQ_FIRST                     0x00

#define BM3803_IRQ_RESET		     0x00
#define BM3803_IRQ_INST_ACC_EXCEPTION	     0x01
#define BM3803_IRQ_ILL_INST		     0x02
#define BM3803_IRQ_PRIVELEGE_INST	     0x03
#define BM3803_IRQ_FP_DISABLED	     	     0x04
#define BM3803_IRQ_WINDOW_OVERFLOW	     0x05
#define BM3803_IRQ_WINDOW_UNDERFLOW	     0x06
#define BM3803_IRQ_ADD_NOT_ALIGNED	     0x07
#define BM3803_IRQ_ADD_FP_EXCEPTION	     0x08
#define BM3803_IRQ_DATA_ACC_EXCEPTION	     0x09
#define BM3803_IRQ_TAG_OVERFLOW	     	     0x0A
#define BM3803_IRQ_HW_UNDEFINED_0B     	     0x0B
#define BM3803_IRQ_HW_UNDEFINED_0C     	     0x0C
#define BM3803_IRQ_HW_UNDEFINED_0D     	     0x0D
#define BM3803_IRQ_HW_UNDEFINED_0E     	     0x0E
#define BM3803_IRQ_HW_UNDEFINED_0F     	     0x0F
#define BM3803_IRQ_HW_UNDEFINED_10     	     0x10

#define BM3803_IRQ_FIRST_INTERRUPT           0x11
#define BM3803_IRQ_CORRECTABLE_MEMORY_ERROR  0x11
#define BM3803_IRQ_UART_2_RX_TX              0x12
#define BM3803_IRQ_UART_1_RX_TX              0x13
#define BM3803_IRQ_EXTERNAL_0                0x14
#define BM3803_IRQ_EXTERNAL_1                0x15
#define BM3803_IRQ_EXTERNAL_2                0x16
#define BM3803_IRQ_EXTERNAL_3                0x17
#define BM3803_IRQ_TIMER1                    0x18
#define BM3803_IRQ_TIMER2                    0x19
#define BM3803_IRQ_EMPTY1                    0x1A
#define BM3803_IRQ_EMPTY2                    0x1B
#define BM3803_IRQ_UART_3_RX_TX              0x1C
#define BM3803_IRQ_EMPTY4                    0x1D
#define BM3803_IRQ_EMPTY5                    0x1E
#define BM3803_IRQ_EMPTY6                    0x1F
#define BM3803_IRQ_LAST_INTERRUPT            0x1F

#define BM3803_IRQ_HW_UNDEFINED_20     	     0x20
#define BM3803_IRQ_HW_UNDEFINED_21     	     0x21
#define BM3803_IRQ_HW_UNDEFINED_22     	     0x22
#define BM3803_IRQ_HW_UNDEFINED_23     	     0x23

#define BM3803_IRQ_CP_DISABLED	     	     0x24

#define BM3803_IRQ_HW_UNDEFINED_25     	     0x25
#define BM3803_IRQ_HW_UNDEFINED_26     	     0x26
#define BM3803_IRQ_HW_UNDEFINED_27     	     0x27

#define BM3803_IRQ_CP_EXCEPTION	     	     0x28

#define BM3803_IRQ_HW_UNDEFINED_29     	     0x29
#define BM3803_IRQ_HW_UNDEFINED_7F     	     0x7F

#define BM3803_IRQ_SW_SYSCALL_TA0     	     0x80
#define BM3803_IRQ_SW_UNDEFINED_81     	     0x81
#define BM3803_IRQ_SW_UNDEFINED_82     	     0x82
#define BM3803_IRQ_SW_FLUSH_WINDOWS          0x83

#define BM3803_IRQ_SW_UNDEFINED_84     	     0x84
#define BM3803_IRQ_SW_UNDEFINED_85     	     0x85
#define BM3803_IRQ_SW_UNDEFINED_86     	     0x86
#define BM3803_IRQ_SW_UNDEFINED_87     	     0x87
#define BM3803_IRQ_SW_SYSCALL_TA8     	     0x88

#define BM3803_IRQ_SW_SYSCALL_IRQDIS         0x89
#define BM3803_IRQ_SW_SYSCALL_IRQEN          0x8A

#define BM3803_IRQ_SW_UNDEFINED_8B     	     0x8B
#define BM3803_IRQ_SW_UNDEFINED_FF     	     0xFF

#define BM3803_IRQ_LAST                      0xFF

#  define NR_IRQS         	             256

#else
  #error "Unrecognized chip"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_SPARC_INCLUDE_BM3803_IRQ_H */
