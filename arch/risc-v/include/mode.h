/****************************************************************************
 * arch/risc-v/include/mode.h
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

#ifndef __ARCH_RISCV_INCLUDE_MODE_H
#define __ARCH_RISCV_INCLUDE_MODE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_USE_S_MODE

/* CSR definitions */

#  define CSR_STATUS        CSR_SSTATUS      /* Global status register */
#  define CSR_SCRATCH       CSR_SSCRATCH     /* Scratch register */
#  define CSR_EPC           CSR_SEPC         /* Exception program counter */
#  define CSR_IE            CSR_SIE          /* Interrupt enable register */
#  define CSR_IP            CSR_SIP          /* Interrupt pending register */
#  define CSR_CAUSE         CSR_SCAUSE       /* Interrupt cause register */
#  define CSR_TVAL          CSR_STVAL        /* Trap value register */
#  define CSR_TVEC          CSR_STVEC        /* Trap vector base addr register */
#  define CSR_ENVCFG        CSR_SENVCFG      /* Env configuration register */

/* In status register */

#  define STATUS_IE         SSTATUS_SIE      /* Global interrupt enable */
#  define STATUS_PIE        SSTATUS_SPIE     /* Previous interrupt enable */
#  define STATUS_PPP        SSTATUS_SPPS     /* Previous privilege */
#  define STATUS_SUM        SSTATUS_SUM      /* Access to user memory */

/* Interrupt bits */

#  define IE_EIE            SIE_SEIE         /* External interrupt enable */
#  define IE_SIE            SIE_SSIE         /* Software interrupt enable */
#  define IE_TIE            SIE_STIE         /* Timer interrupt enable */

/* Interrupt pending bits */

#  define IP_EIP            SIP_SEIP         /* External interrupt pending */
#  define IP_SIP            SIP_SSIP         /* Software interrupt pending */
#  define IP_TIP            SIP_STIP         /* Timer interrupt pending */

/* External, timer and software interrupt */

#  define RISCV_IRQ_EXT     RISCV_IRQ_SEXT   /* PLIC IRQ */
#  define RISCV_IRQ_TIMER   RISCV_IRQ_STIMER /* Timer IRQ */
#  define RISCV_IRQ_SOFT    RISCV_IRQ_SSOFT  /* SW IRQ */

/* Define return from exception */

#  define ERET              sret

#else

/* CSR definitions */

#  define CSR_STATUS        CSR_MSTATUS      /* Global status register */
#  define CSR_SCRATCH       CSR_MSCRATCH     /* Scratch register */
#  define CSR_EPC           CSR_MEPC         /* Exception program counter */
#  define CSR_IE            CSR_MIE          /* Interrupt enable register */
#  define CSR_IP            CSR_MIP          /* Interrupt pending register */
#  define CSR_CAUSE         CSR_MCAUSE       /* Interrupt cause register */
#  define CSR_TVAL          CSR_MTVAL        /* Trap value register */
#  define CSR_TVEC          CSR_MTVEC        /* Trap vector base addr register */
#  define CSR_ENVCFG        CSR_MENVCFG      /* Env configuration register */

/* In status register */

#  define STATUS_IE         MSTATUS_MIE      /* Global interrupt enable */
#  define STATUS_PIE        MSTATUS_MPIE     /* Previous interrupt enable */
#  define STATUS_PPP        MSTATUS_MPPM     /* Previous privilege */
#  define STATUS_SUM        0                /* Not needed in M-mode */

/* Interrupt bits */

#  define IE_EIE            MIE_MEIE         /* External interrupt enable */
#  define IE_SIE            MIE_MSIE         /* Software interrupt enable */
#  define IE_TIE            MIE_MTIE         /* Timer interrupt enable */

/* Interrupt pending bits */

#  define IP_EIP            MIP_MEIP         /* External interrupt pending */
#  define IP_SIP            MIP_MSIP         /* Software interrupt pending */
#  define IP_TIP            MIP_MTIP         /* Timer interrupt pending */

/* External, timer and software interrupt */

#  define RISCV_IRQ_EXT     RISCV_IRQ_MEXT   /* PLIC IRQ */
#  define RISCV_IRQ_TIMER   RISCV_IRQ_MTIMER /* Timer IRQ */
#  define RISCV_IRQ_SOFT    RISCV_IRQ_MSOFT  /* SW IRQ */

/* Define return from exception */

#  define ERET              mret

#endif

#endif /* __ARCH_RISCV_INCLUDE_MODE_H */
