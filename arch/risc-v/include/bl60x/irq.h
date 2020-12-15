/****************************************************************************
 * arch/risc-v/include/bl60x/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_BL60X_IRQ_H
#define __ARCH_RISCV_INCLUDE_BL60X_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* In mstatus register */

#define MSTATUS_MIE   (0x1 << 3)  /* Machine Interrupt Enable */
#define MSTATUS_MPIE  (0x1 << 7)  /* Machine Previous Interrupt Enable */
#define MSTATUS_MPPM  (0x3 << 11) /* Machine Previous Privilege (m-mode) */

/* In mie (machine interrupt enable) register */

#define MIE_MSIE      (0x1 << 3)  /* Machine Software Interrupt Enable */
#define MIE_MTIE      (0x1 << 7)  /* Machine Timer Interrupt Enable */
#define MIE_MEIE      (0x1 << 11) /* Machine External Interrupt Enable */
#define MIP_MTIP      (0x1 << 7)  /* Machine Timer Interrupt Pending */

/* Map RISC-V exception code to NuttX IRQ */

/* See the listing here for more information
 * https://github.com/bouffalolab/bl_iot_sdk/blob/f8c47051ed9338b0f1036e55d8cb1eb56c86c089/components/bl602/bl602_std/bl602_std/RISCV/Device/Bouffalo/BL602/Startup/interrupt.c
 */

/* IRQ 0-15 : (exception:interrupt=0) */

#define BL_IRQ_IAMISALIGNED    (0)       /* Instruction Address Misaligned */
#define BL_IRQ_IAFAULT         (1)       /* Instruction Address Fault */
#define BL_IRQ_IINSTRUCTION    (2)       /* Illegal Instruction */
#define BL_IRQ_BPOINT          (3)       /* Break Point */
#define BL_IRQ_LAMISALIGNED    (4)       /* Load Address Misaligned */
#define BL_IRQ_LAFAULT         (5)       /* Load Access Fault */
#define BL_IRQ_SAMISALIGNED    (6)       /* Store/AMO Address Misaligned */
#define BL_IRQ_SAFAULT         (7)       /* Store/AMO Access Fault */
#define BL_IRQ_ECALLU          (8)       /* Environment Call from U-mode */
                                         /* 9-10: Reserved */

#define BL_IRQ_ECALLM          (11)      /* Environment Call from M-mode */
                                         /* 12-15: Reserved */

#define BL_IRQ_CLIC_MSIP       (16 + 3)  /* Machine Software Interrupt */
#define BL_IRQ_CLIC_MTIMER     (16 + 7)  /* Machine Timer Interrupt */
#define BL_IRQ_CLIC_MEXT       (16 + 7)  /* Machine External Interrupt */
#define BL_IRQ_CLIC_CSOFT      (16 + 8)
#define BL_IRQ_BMX_ERR         (32 + 0)
#define BL_IRQ_UART0           (32 + 29)
#define BL_IRQ_UART0           (32 + 30)
#define BL_IRQ_WIFI_IPC_PUBLIC (32 + 63)

/* Total number of IRQs */

#define NR_IRQS                (BL_IRQ_WIFI_IPC_PUBLIC + 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN irqstate_t  up_irq_save(void);
EXTERN void up_irq_restore(irqstate_t);
EXTERN irqstate_t up_irq_enable(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_INCLUDE_BL60X_IRQ_H */
