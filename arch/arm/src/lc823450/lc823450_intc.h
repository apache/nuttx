/****************************************************************************
 * arch/arm/src/lc823450/lc823450_intc.h
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

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_INTC_H
#define __ARCH_ARM_SRC_LC823450_LC823450_INTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LC823450_INTC_REGBASE   0x40003000

#define IPIREG          (LC823450_INTC_REGBASE + 0x000)
#define   IPIREG_INTISR0_0      (0x1 << 0)
#define   IPIREG_INTISR0_1      (0x1 << 1)
#define   IPIREG_INTISR0_2      (0x1 << 2)
#define   IPIREG_INTISR0_3      (0x1 << 3)
#define   IPIREG_INTISR1_0      (0x1 << 8)
#define   IPIREG_INTISR1_1      (0x1 << 9)
#define   IPIREG_INTISR1_2      (0x1 << 10)
#define   IPIREG_INTISR1_3      (0x1 << 11)
#define IPICLR          (LC823450_INTC_REGBASE + 0x004)
#define   IPICLR_INTISR0_CLR_0  (0x1 << 0)
#define   IPICLR_INTISR0_CLR_1  (0x1 << 1)
#define   IPICLR_INTISR0_CLR_2  (0x1 << 2)
#define   IPICLR_INTISR0_CLR_3  (0x1 << 3)
#define   IPICLR_INTISR1_CLR_0  (0x1 << 8)
#define   IPICLR_INTISR1_CLR_1  (0x1 << 9)
#define   IPICLR_INTISR1_CLR_2  (0x1 << 10)
#define   IPICLR_INTISR1_CLR_3  (0x1 << 11)

#define EXTINT_BASE    (LC823450_INTC_REGBASE + 0x400)
#define EXTINTS_BASE   (LC823450_INTC_REGBASE + 0x418)
#define EXTINTM_BASE   (LC823450_INTC_REGBASE + 0x430)
#define EXTINTC0_BASE  (LC823450_INTC_REGBASE + 0x448)
#define EXTINTC1_BASE  (LC823450_INTC_REGBASE + 0x460)
#define EXTINTCND_BASE (LC823450_INTC_REGBASE + 0x478)
#define EXTINTCLR_BASE (LC823450_INTC_REGBASE + 0x490)
#define EXTINTFEN_BASE (LC823450_INTC_REGBASE + 0x4A8)
#define EXTINTSET_BASE (LC823450_INTC_REGBASE + 0x4C0)

#define INTC_REG(base,port)  ((base) + 4 * (port))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_INTC_H */
