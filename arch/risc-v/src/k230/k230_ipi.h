/****************************************************************************
 * arch/risc-v/src/k230/k230_ipi.h
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

#ifndef __ARCH_RISCV_SRC_K230_K230_IPI_H
#define __ARCH_RISCV_SRC_K230_K230_IPI_H

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* K230 has 4 IPI devices, each has 10 registers with 32 bit width. */

#define K230_IPI_DEVN_MAX           3
#define K230_IPI_LINE_MAX           15
#define K230_IPI_BASE(n)            (0x91104000ul + n * 0x28)

/* The IPI registers are organized in two directions (M2R or R2M), there are
 * 5 registers for each direction, with 16 IPI lines for each node. Though
 * K230 names the two directions as CPU2DSP or DSP2CPU, we use M2R or R2M.
 *
 * The Status register has 2-bit counter for each IPI line.
 * The Set/Clear registers accepts values from 0..15 for 16 IPI lines.
 * Setting a line increases its counter by 1, clearing the line decreases it
 * by 1. The Error register further uses the high/low 16 bits to flag the
 * over/under-run situations of counters.
 *
 * The Enable register's bit 16..31 are for line enable, bit 0 is for device
 * interrupt enable, bit 1 for resetting line counters to zero.
 *
 * Also note the directions M2R/R2M are tied to the CPU cores: little core is
 * M and big core is R. So we can also select the direction based on current
 * core, but due to that we can't read correct MISA register value with NSBI
 * booting environment, that method is not used and the role is passed in.
 */

#define K230_IPI_M2R_INTEN(n)       (K230_IPI_BASE(n) + 0)
#define K230_IPI_M2R_INTSET(n)      (K230_IPI_BASE(n) + 4)
#define K230_IPI_M2R_INTCLR(n)      (K230_IPI_BASE(n) + 8)
#define K230_IPI_M2R_INTSTS(n)      (K230_IPI_BASE(n) + 12)
#define K230_IPI_M2R_INTERR(n)      (K230_IPI_BASE(n) + 16)
#define K230_IPI_R2M_INTEN(n)       (K230_IPI_BASE(n) + 20)
#define K230_IPI_R2M_INTSET(n)      (K230_IPI_BASE(n) + 24)
#define K230_IPI_R2M_INTCLR(n)      (K230_IPI_BASE(n) + 28)
#define K230_IPI_R2M_INTSTS(n)      (K230_IPI_BASE(n) + 32)
#define K230_IPI_R2M_INTERR(n)      (K230_IPI_BASE(n) + 36)

#define IPI_ROLE_MASTER   1    /* master role, for little core */
#define IPI_ROLE_REMOTE   2    /* remote role, for big core */

/* Tools for handling uint16_t device and line id combo */

#define IPI_DEVN(x)      ((x & 0xFF00) >> 8)
#define IPI_LINE(x)      (x & 0xFF)
#define IPI_COMB(d,l)    ((d << 8) | (l & 0xFF))

/****************************************************************************
 * Public types
 ****************************************************************************/

/****************************************************************************
 * Name: ipi_callback_t
 * Description:
 *   Callback for a particular IPI line. Should be brief as maybe running in
 *   ISR context.
 * Params;
 *   comb: combined IPI dev and line ids, see IPI_COMB above
 *   args: the args used in subscription
 ****************************************************************************/

typedef void (*ipi_callback_t)(uint16_t comb, void *args);

/****************************************************************************
 * Public functions
 ****************************************************************************/

/****************************************************************************
 * Name: k230_ipi_init
 * Description:
 *   Initialzie IPI device with receiving and sending line masks.
 * Params:
 *   devn: IPI device number in 0..K230_IPI_DEVN_MAX
 *   mask: allowed lines mask for notifying peers or receiving notifications
 *   role: IPI role of this node (IPI_ROLE_MASTER or IPI_ROLE_REMOTE)
 *   ipcb: callback for incoming IPI notifications
 *   args: last parameter for the callback.
 * Returns:
 *   0 on success, or negative value on errors
 ****************************************************************************/

int k230_ipi_init(uintptr_t devn, uint16_t mask, uint16_t role,
                  ipi_callback_t ipcb, void *args);

/****************************************************************************
 * Name: k230_ipi_notify
 * Description:
 *   Notify peers via IPI.
 * Params:
 *   devn: device id in 0..K230_IPI_DEVN_MAX
 *   line: line id in 0..K230_IPI_LINE_MAX
 ****************************************************************************/

void k230_ipi_notify(uint8_t devn, uint8_t line);

/****************************************************************************
 * Name: k230_ipi_finish
 * Description:
 *   Deinitializes IPI device
 * Params:
 *   devn: IPI device number initialized previously.
 *   mask: line masks iniitialied previously.
 ****************************************************************************/

void k230_ipi_finish(uint8_t devn, uint16_t mask);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_K230_K230_IPI_H */
