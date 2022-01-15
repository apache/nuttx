/****************************************************************************
 * arch/misoc/include/minerva/csrdefs.h
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

#ifndef __ARCH_MISOC_INCLUDE_MINERVA_CSRDEFS_H
#define __ARCH_MISOC_INCLUDE_MINERVA_CSRDEFS_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CSR_MVENDORID_ADDR   0xf11
#define CSR_MARCHID_ADDR     0xf12
#define CSR_MIMPID_ADDR      0xf13
#define CSR_MHARTID_ADDR     0xf14
#define CSR_MSTATUS_ADDR     0x300
#define CSR_MISA_ADDR        0x301
#define CSR_MEDELEG_ADDR     0x302
#define CSR_MIDELEG_ADDR     0x303
#define CSR_MIE_ADDR         0x304
#define CSR_MTVEC_ADDR       0x305
#define CSR_MCOUTEREN_ADDR   0x306
#define CSR_MSCRATCH_ADDR    0x340
#define CSR_MEPC_ADDR        0x341
#define CSR_MCAUSE_ADDR      0x342
#define CSR_MTVAL_ADDR       0x343
#define CSR_MIP_ADDR         0x344
#define CSR_IRQ_MASK_ADDR    0x330
#define CSR_IRQ_PENDING_ADDR 0x360

#define CSR_MSTATUS_UIE      (1 << 0)
#define CSR_MSTATUS_SIE      (1 << 1)
#define CSR_MSTATUS_MIE      (1 << 3)
#define CSR_MSTATUS_UPIE     (1 << 4)
#define CSR_MSTATUS_SPIE     (1 << 5)
#define CSR_MSTATUS_MPIE     (1 << 7)
#define CSR_MSTATUS_SPP      (1 << 8)
#define CSR_MSTATUS_MPP      (1 << 11)
#define CSR_MSTATUS_FS       (1 << 13)
#define CSR_MSTATUS_XS       (1 << 15)
#define CSR_MSTATUS_MPRV     (1 << 17)
#define CSR_MSTATUS_SUM      (1 << 18)
#define CSR_MSTATUS_MXR      (1 << 19)
#define CSR_MSTATUS_TVM      (1 << 20)
#define CSR_MSTATUS_TW       (1 << 21)
#define CSR_MSTATUS_TSR      (1 << 22)
#define CSR_MSTATUS_SD       (1 << 31)

#endif /* __ARCH_MISOC_INCLUDE_MINERVA_CSRDEFS_H */
