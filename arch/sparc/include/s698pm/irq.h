/****************************************************************************
 * arch/sparc/include/s698pm/irq.h
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

#ifndef __ARCH_SPARC_INCLUDE_S698PM_IRQ_H
#define __ARCH_SPARC_INCLUDE_S698PM_IRQ_H

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

#if defined(CONFIG_ARCH_CHIP_S698PM)

#define S698PM_IRQREG_BASE          0x80000200

#define S698PM_IRQREG_ILEVEL        (S698PM_IRQREG_BASE + 0)
#define S698PM_IRQREG_IPEND         (S698PM_IRQREG_BASE + 0x4)
#define S698PM_IRQREG_IFORCE        (S698PM_IRQREG_BASE + 0x8)
#define S698PM_IRQREG_ICLEAR        (S698PM_IRQREG_BASE + 0xc)
#define S698PM_IRQREG_MPSTATUS      (S698PM_IRQREG_BASE + 0x10)
#define S698PM_IRQREG_BROADCAST     (S698PM_IRQREG_BASE + 0x14)

#define S698PM_IRQREG_P0_MASK       (S698PM_IRQREG_BASE + 0x40)
#define S698PM_IRQREG_P1_MASK       (S698PM_IRQREG_BASE + 0x44)
#define S698PM_IRQREG_P2_MASK       (S698PM_IRQREG_BASE + 0x48)
#define S698PM_IRQREG_P3_MASK       (S698PM_IRQREG_BASE + 0x4c)

#define S698PM_IRQREG_P0_FORCE      (S698PM_IRQREG_BASE + 0x80)
#define S698PM_IRQREG_P1_FORCE      (S698PM_IRQREG_BASE + 0x84)
#define S698PM_IRQREG_P2_FORCE      (S698PM_IRQREG_BASE + 0x88)
#define S698PM_IRQREG_P3_FORCE      (S698PM_IRQREG_BASE + 0x8c)

#define S698PM_IRQREG_P0_EXTACK     (S698PM_IRQREG_BASE + 0xc0)
#define S698PM_IRQREG_P1_EXTACK     (S698PM_IRQREG_BASE + 0xc4)
#define S698PM_IRQREG_P2_EXTACK     (S698PM_IRQREG_BASE + 0xc8)
#define S698PM_IRQREG_P3_EXTACK     (S698PM_IRQREG_BASE + 0xcc)

#define S698PM_IRQ_FIRST                     0x00

#define S698PM_IRQ_RESET		     0x00
#define S698PM_IRQ_INST_ACC_EXCEPTION	     0x01
#define S698PM_IRQ_ILL_INST		     0x02
#define S698PM_IRQ_PRIVELEGE_INST	     0x03
#define S698PM_IRQ_FP_DISABLED	     	     0x04
#define S698PM_IRQ_WINDOW_OVERFLOW	     0x05
#define S698PM_IRQ_WINDOW_UNDERFLOW	     0x06
#define S698PM_IRQ_ADD_NOT_ALIGNED	     0x07
#define S698PM_IRQ_ADD_FP_EXCEPTION	     0x08
#define S698PM_IRQ_DATA_ACC_EXCEPTION	     0x09
#define S698PM_IRQ_TAG_OVERFLOW	     	     0x0A
#define S698PM_IRQ_HW_UNDEFINED_0B     	     0x0B
#define S698PM_IRQ_HW_UNDEFINED_0C     	     0x0C
#define S698PM_IRQ_HW_UNDEFINED_0D     	     0x0D
#define S698PM_IRQ_HW_UNDEFINED_0E     	     0x0E
#define S698PM_IRQ_HW_UNDEFINED_0F     	     0x0F
#define S698PM_IRQ_HW_UNDEFINED_10     	     0x10

#define S698PM_IRQ_FIRST_INT                 0x11

#define S698PM_IRQ_AHB_ERROR                 0x11
#define S698PM_IRQ_UART_1_RX_TX              0x12
#define S698PM_IRQ_UART_2_RX_TX              0x13
#define S698PM_IRQ_LOCKTIMER12               0x14
#define S698PM_IRQ_ETHERNET                  0x15
#define S698PM_IRQ_TIMER1                    0x16
#define S698PM_IRQ_TIMER2                    0x17
#define S698PM_IRQ_TIMER3                    0x18
#define S698PM_IRQ_TIMER4                    0x19
#define S698PM_IRQ_1553B                     0x1A
#define S698PM_IRQ_EXTENDED                  0x1B
#define S698PM_IRQ_EXTERNAL_12               0x1C
#define S698PM_IRQ_EXTERNAL_13               0x1D
#define S698PM_IRQ_EXTERNAL_14               0x1E
#define S698PM_IRQ_EXTERNAL_15               0x1F

#define S698PM_IRQ_LAST_INT                  0x1F

#define S698PM_IRQ_HW_UNDEFINED_20     	     0x20
#define S698PM_IRQ_HW_UNDEFINED_21     	     0x21
#define S698PM_IRQ_HW_UNDEFINED_22     	     0x22
#define S698PM_IRQ_HW_UNDEFINED_23     	     0x23

#define S698PM_IRQ_CP_DISABLED	     	     0x24

#define S698PM_IRQ_HW_UNDEFINED_25     	     0x25
#define S698PM_IRQ_HW_UNDEFINED_26     	     0x26
#define S698PM_IRQ_HW_UNDEFINED_27     	     0x27

#define S698PM_IRQ_CP_EXCEPTION	     	     0x28

#define S698PM_IRQ_HW_UNDEFINED_29     	     0x29
#define S698PM_IRQ_HW_UNDEFINED_7F     	     0x7F

#define S698PM_IRQ_SW_SYSCALL_TA0     	     0x80
#define S698PM_IRQ_SW_UNDEFINED_81     	     0x81
#define S698PM_IRQ_SW_UNDEFINED_82     	     0x82
#define S698PM_IRQ_SW_FLUSH_WINDOWS          0x83

#define S698PM_IRQ_SW_UNDEFINED_84     	     0x84
#define S698PM_IRQ_SW_UNDEFINED_85     	     0x85
#define S698PM_IRQ_SW_UNDEFINED_86     	     0x86
#define S698PM_IRQ_SW_UNDEFINED_87     	     0x87
#define S698PM_IRQ_SW_SYSCALL_TA8     	     0x88

#define S698PM_IRQ_SW_SYSCALL_IRQDIS         0x89
#define S698PM_IRQ_SW_SYSCALL_IRQEN          0x8A

#define S698PM_IRQ_SW_UNDEFINED_8B     	     0x8B
#define S698PM_IRQ_SW_UNDEFINED_FF     	     0xFF

#define S698PM_IRQ_LAST                      0xFF

#define S698PM_IRQ_SPW1                      0x100
#define S698PM_IRQ_SPW2                      0x101
#define S698PM_IRQ_SPW3                      0x102
#define S698PM_IRQ_SPW4                      0x103
#define S698PM_IRQ_CCSDS                     0x104
#define S698PM_IRQ_EXTERNAL                  0x105
#define S698PM_IRQ_USBHOST                   0x106
#define S698PM_IRQ_UART_3_RX_TX              0x107
#define S698PM_IRQ_CAN1                      0x108
#define S698PM_IRQ_CAN2                      0x109
#define S698PM_IRQ_CCSDS_CODE                0x10A
#define S698PM_IRQ_CCSDS_DECODE              0x10B
#define S698PM_IRQ_I2C                       0x10C
#define S698PM_IRQ_SPI                       0x10D
#define S698PM_IRQ_UART_4_RX_TX              0x10E
#define S698PM_IRQ_L2CACHE                   0x10F

#define NR_IRQS         	             272
#define S698PM_EXTENDED_IRQ                  11
#define S698PM_IPI_VECTOR                    14
#define S698PM_IPI_IRQ                       S698PM_IRQ_EXTERNAL_14
#define S698PM_EXTENDED_START                16
#define S698PM_CPUINT_MAX                    32

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

#endif /* __ARCH_SPARC_INCLUDE_S698PM_IRQ_H */
