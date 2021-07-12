/****************************************************************************
 * arch/risc-v/src/rv32m1/hardware/rv32m1_eu.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_EU_H
#define __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_EU_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RV32M1_EU_INTPTEN_OFFSET       0x0000 /* Interrupt Enable */
#define RV32M1_EU_INTPTPEND_OFFSET     0x0004 /* Interrupt Pending */
#define RV32M1_EU_INTPTPENDSET_OFFSET  0x0008 /* Set Interrupt Pending */
#define RV32M1_EU_INTPTPENDCLR_OFFSET  0x000c /* Clear Interrupt Pending */
#define RV32M1_EU_INTPTSECURE_OFFSET   0x0010 /* Interrupt Secure */
#define RV32M1_EU_INTPTPRI0_OFFSET     0x0014 /* Interrupt Priority 0 */
#define RV32M1_EU_INTPTPRI1_OFFSET     0x0018 /* Interrupt Priority 1 */
#define RV32M1_EU_INTPTPRI2_OFFSET     0x001c /* Interrupt Priority 2 */
#define RV32M1_EU_INTPTPRI3_OFFSET     0x0020 /* Interrupt Priority 3 */
#define RV32M1_EU_INTPTBASE_OFFSET     0x0024 /* Interrupt Priority Base */
#define RV32M1_EU_INTPTENACTIVE_OFFSET 0x0028 /* Interrupt Active */
#define RV32M1_EU_INTACTPRI0_OFFSET    0x002c /* Interrupt Active Priority 0 */
#define RV32M1_EU_INTACTPRI1_OFFSET    0x0030 /* Interrupt Active Priority 1 */
#define RV32M1_EU_INTACTPRI2_OFFSET    0x0034 /* Interrupt Active Priority 2 */
#define RV32M1_EU_INTACTPRI3_OFFSET    0x0038 /* Interrupt Active Priority 2 */
#define RV32M1_EU_EVENTEN_OFFSET       0x0040 /* Event Enable */
#define RV32M1_EU_EVENTPEND_OFFSET     0x0044 /* Event Pending */
#define RV32M1_EU_EVTPENDSET_OFFSET    0x0048 /* Set Event Pending */
#define RV32M1_EU_EVTPENDCLR_OFFSET    0x004c /* Clear Event Pending */
#define RV32M1_EU_SLPCTRL_OFFSET       0x0080 /* Sleep Control */
#define RV32M1_EU_SLPSTAT_OFFST        0x0084 /* Sleep Status */

/* Register Addresses *******************************************************/

#define RV32M1_EU_INTPTEN          (RV32M1_EU_BASE + RV32M1_EU_INTPTEN_OFFSET)
#define RV32M1_EU_INTPTPEND        (RV32M1_EU_BASE + RV32M1_EU_INTPTPEND_OFFSET)
#define RV32M1_EU_INTPTPENDSET     (RV32M1_EU_BASE + RV32M1_EU_INTPTPENDSET_OFFSET)
#define RV32M1_EU_INTPTPENDCLR     (RV32M1_EU_BASE + RV32M1_EU_INTPTPENDCLR_OFFSET)
#define RV32M1_EU_INTPTSECURE      (RV32M1_EU_BASE + RV32M1_EU_INTPTSECURE_OFFSET)
#define RV32M1_EU_INTPTPRI0        (RV32M1_EU_BASE + RV32M1_EU_INTPTPRI0_OFFSET)
#define RV32M1_EU_INTPTPRI1        (RV32M1_EU_BASE + RV32M1_EU_INTPTPRI1_OFFSET)
#define RV32M1_EU_INTPTPRI2        (RV32M1_EU_BASE + RV32M1_EU_INTPTPRI2_OFFSET)
#define RV32M1_EU_INTPTPRI3        (RV32M1_EU_BASE + RV32M1_EU_INTPTPRI3_OFFSET)
#define RV32M1_EU_INTPTBASE        (RV32M1_EU_BASE + RV32M1_EU_INTPTBASE_OFFSET)
#define RV32M1_EU_INTPTENACTIVE    (RV32M1_EU_BASE + RV32M1_EU_INTPTENACTIVE_OFFSET)
#define RV32M1_EU_INTACTPRI0       (RV32M1_EU_BASE + RV32M1_EU_INTACTPRI0_OFFSET)
#define RV32M1_EU_INTACTPRI1       (RV32M1_EU_BASE + RV32M1_EU_INTACTPRI1_OFFSET)
#define RV32M1_EU_INTACTPRI2       (RV32M1_EU_BASE + RV32M1_EU_INTACTPRI2_OFFSET)
#define RV32M1_EU_INTACTPRI3       (RV32M1_EU_BASE + RV32M1_EU_INTACTPRI3_OFFSET)
#define RV32M1_EU_EVENTEN          (RV32M1_EU_BASE + RV32M1_EU_EVENTEN_OFFSET)
#define RV32M1_EU_EVENTPEND        (RV32M1_EU_BASE + RV32M1_EU_EVENTPEND_OFFSET)
#define RV32M1_EU_EVTPENDSET       (RV32M1_EU_BASE + RV32M1_EU_EVTPENDSET_OFFSET)
#define RV32M1_EU_EVTPENDCLR       (RV32M1_EU_BASE + RV32M1_EU_EVTPENDCLR_OFFSET)
#define RV32M1_EU_SLPCTRL          (RV32M1_EU_BASE + RV32M1_EU_SLPCTRL_OFFSET)
#define RV32M1_EU_SLPSTAT          (RV32M1_EU_BASE + RV32M1_EU_SLPSTAT_OFFSET)

/* Register Bitfield Definitions ********************************************/

#define EU_INTPTPRI_BITS           (4)
#define EU_INTPTPRI0_SHIFT         (0)
#define EU_INTPTPRI0_MASK          (0x7 << EU_INTPTPRI0_SHIFT)

#define EU_INTPRIBASE_IPBASE_SHIFT (0)
#define EU_INTPRIBASE_IPBASE_MASK  (0xf << EU_INTPRIBASE_IPBASE_SHIFT)

#define EU_SLPCTRL_SYSRSTREQST     (1 << 31)

#define EU_SLPCTRL_SLEEP_SHIFT     (0)
#define EU_SLPCTRL_SLEEP_MASK      (0x3 << EU_SLPCTRL_SLEEP_SHIFT)
#define EU_SLPCTRL_SLEEP           (0x1 << EU_SLPCTRL_SLEEP_SHIFT)
#define EU_SLPCTRL_DEEP_SLEEP      (0x2 << EU_SLPCTRL_SLEEP_SHIFT)

#define EU_SLPSTAT_SLEEP_SHIFT     (0)
#define EU_SLPSTAT_SLEEP_MASK      (0x3 << EU_SLPSTAT_SLEEP_SHIFT)
#define EU_SLPSTAT_SLEEP           (0x1 << EU_SLPSTAT_SLEEP_SHIFT)
#define EU_SLPSTAT_DEEP_SLEEP      (0x2 << EU_SLPSTAT_SLEEP_SHIFT)

#endif /* __ARCH_RISCV_SRC_RV32M1_HARDWARE_RV32M1_EU_H */
