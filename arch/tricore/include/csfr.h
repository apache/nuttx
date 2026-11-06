/****************************************************************************
 * arch/tricore/include/csfr.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_TRICORE_INCLUDE_CSFR_H
#define __ARCH_TRICORE_INCLUDE_CSFR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Core Special Function Register (CSFR) offsets for mfcr/mtcr, the same on
 * TC1.6.2 (TC3xx) and TC1.8 (TC4xx) unless noted.
 */

/* Context management */

#define TRICORE_CPU_PCXI            0xfe00  /* Previous Context Information */
#define TRICORE_CPU_PSW             0xfe04  /* Program Status Word */
#define TRICORE_CPU_CORE_ID         0xfe1c  /* Core Identification */
#define TRICORE_CPU_ICR             0xfe2c  /* Interrupt Control Register */
#define TRICORE_CPU_FCX             0xfe38  /* Free CSA List Head Pointer */

/* Core configuration (TC1.8 only) */

#define TRICORE_CPU_CORECON         0xfe14  /* Core Control */

/* Performance counters */

#define TRICORE_CPU_CCTRL           0xfc00  /* Counter Control */
#define TRICORE_CPU_CCNT            0xfc04  /* CPU Clock Cycle Count */

/* Trap status */

#define TRICORE_CPU_PSTR            0x9200  /* Program Synchronous Trap Status */
#define TRICORE_CPU_DSTR            0x9010  /* Data Synchronous Trap Status */
#define TRICORE_CPU_DATR            0x9018  /* Data Asynchronous Trap Register */
#define TRICORE_CPU_DEADD           0x901c  /* Data Error Address */

/* Program and data memory control (DCON1 is TC1.8 only) */

#define TRICORE_CPU_PCON0           0x920c  /* Program Memory Configuration 0 */
#define TRICORE_CPU_PCON1           0x9204  /* Program Memory Configuration 1 */
#define TRICORE_CPU_PCON2           0x9208  /* Program Memory Configuration 2 */
#define TRICORE_CPU_DCON0           0x9040  /* Data Memory Configuration 0 */
#define TRICORE_CPU_DCON1           0x9008  /* Data Memory Configuration 1 */
#define TRICORE_CPU_DCON2           0x9000  /* Data Memory Configuration 2 */

/* Memory protection.  n is the range or protection set number and must be
 * a constant, as mfcr/mtcr take the CSFR offset as an immediate.  TC1.6.2
 * has data ranges 0-17, code ranges 0-9 and sets 0-5; TC1.8 has data
 * ranges 0-23, code ranges 0-15 and sets 0-7.
 */

#define TRICORE_CPU_DPR_L(n)        (0xc000 + ((n) << 3))
#define TRICORE_CPU_DPR_U(n)        (0xc004 + ((n) << 3))
#define TRICORE_CPU_CPR_L(n)        (0xd000 + ((n) << 3))
#define TRICORE_CPU_CPR_U(n)        (0xd004 + ((n) << 3))

#define TRICORE_CPU_CPXE(n)         (0xe000 + (((n) & 3) << 2) + (((n) & 4) << 4))
#define TRICORE_CPU_DPRE(n)         (0xe010 + (((n) & 3) << 2) + (((n) & 4) << 4))
#define TRICORE_CPU_DPWE(n)         (0xe020 + (((n) & 3) << 2) + (((n) & 4) << 4))

/* ICR bits */

#define TRICORE_ICR_CCPN_MASK       0x000000ff  /* Current CPU Priority Number */
#define TRICORE_ICR_IE              (1 << 15)   /* Global Interrupt Enable */

#endif /* __ARCH_TRICORE_INCLUDE_CSFR_H */
