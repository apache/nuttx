/****************************************************************************
 * arch/misoc/src/minerva/csrdefs.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Ramtin Amin <keytwo@gmail.com>
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

#ifndef __RISCV_CSR_DEFS__
#define __RISCV_CSR_DEFS__

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

#endif
