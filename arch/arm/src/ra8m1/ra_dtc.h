/****************************************************************************
 * arch/arm/src/ra8m1/ra_dtc.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_RA_DTC_H
#define __ARCH_ARM_SRC_RA8M1_RA_DTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* One 16-byte transfer-information block (RA8M1 User's Manual, figure
 * 17.4).  MRA, MRB, SAR, DAR, CRA and CRB are not peripheral registers:
 * they are this exact byte/halfword/word layout in ordinary SRAM, which
 * the DTC itself reads and writes.  Allocate one of these per DTC vector
 * actually used (word-aligned; the natural alignment of this struct is 4
 * bytes, since it is deliberately left unpacked -- see the note below),
 * fill it the same way section 17.2.2-17.2.7 of the manual describes each
 * field, and point a vector table entry at it with ra8m1_dtc_vectbl_set()
 * -- before setting that vector's ICU.IELSRn.DTCE bit.
 *
 * This must NOT be a packed struct.  A packed attribute drops the type's
 * alignment to 1, so a plain "static struct dtc_transfer_info_s foo;" can
 * land on an odd address; ra8m1_dtc_vectbl_set() below only masks off the
 * low 2 bits of that address before handing it to the DTC, so the DTC
 * would then read/write the wrong 16 bytes.  The (1,1,1,1,4,4,2,2) layout
 * below has no padding either way, so leaving it unpacked costs nothing
 * and the offset asserts still guard the layout.
 */

struct dtc_transfer_info_s
{
  uint8_t  reserved[2];  /* +0x00: Reserved(0), write 0 */
  uint8_t  mrb;          /* +0x02: DTC Mode Register B */
  uint8_t  mra;          /* +0x03: DTC Mode Register A */
  uint32_t sar;          /* +0x04: Transfer Source Register */
  uint32_t dar;          /* +0x08: Transfer Destination Register */
  uint16_t crb;          /* +0x0c: Transfer Count Register B */
  uint16_t cra;          /* +0x0e: Transfer Count Register A */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ra8m1_dtc_vectbl_entry
 *
 * Description:
 *   Address of the 32-bit vector table entry for DTC vector "vector" (an
 *   IELSR slot number, 0-95), given the vector table's base address (the
 *   value programmed into DTCVBR_SEC).
 *
 ****************************************************************************/

uint32_t *ra8m1_dtc_vectbl_entry(uint32_t vectbl_base, uint8_t vector);

/****************************************************************************
 * Name: ra8m1_dtc_vectbl_set
 *
 * Description:
 *   Point DTC vector "vector" at the transfer information block "info"
 *   (privileged access).  "info" must be word-aligned.  Do this before
 *   setting the vector's ICU.IELSRn.DTCE bit.
 *
 ****************************************************************************/

void ra8m1_dtc_vectbl_set(uint32_t vectbl_base, uint8_t vector,
                          FAR struct dtc_transfer_info_s *info);

/****************************************************************************
 * Name: ra8m1_dtc_transfer_info
 *
 * Description:
 *   Dereference DTC vector "vector"'s table entry and return a pointer to
 *   its transfer information block, as currently programmed with
 *   ra8m1_dtc_vectbl_set().  Returns NULL if the entry is unset (0).
 *
 ****************************************************************************/

FAR struct dtc_transfer_info_s *
ra8m1_dtc_transfer_info(uint32_t vectbl_base, uint8_t vector);

/****************************************************************************
 * Name: ra_dtc_initialize
 *
 * Description:
 *   One-time DTC bring-up: release the shared DMAC/DTC module-stop bit,
 *   program the (secure) vector table base and start the DTC (DTCST.DTCST
 *   = 1).  Called once from up_irqinitialize(), after ra_attach_icu() has
 *   run.  Every DTC vector's table entry is 0 (unset) until a caller uses
 *   ra_dtc_configure() on it.
 *
 ****************************************************************************/

void ra_dtc_initialize(void);

/****************************************************************************
 * Name: ra_dtc_configure
 *
 * Description:
 *   Point DTC vector "vector" (an IELSR slot number, 0-95) at the
 *   caller-owned transfer information block "info", which must already be
 *   filled in (mra/mrb/sar/dar/cra/crb) and stay valid and unmodified by
 *   software until the transfer this arms has completed.  Cleans "info"
 *   and its vector table entry from the data cache, if enabled, so the
 *   DTC's view of SRAM is current.  Does not touch IELSRn.DTCE -- call
 *   ra_dtc_enable() to actually arm the vector.
 *
 ****************************************************************************/

void ra_dtc_configure(uint8_t vector, FAR struct dtc_transfer_info_s *info);

/****************************************************************************
 * Name: ra_dtc_enable
 *
 * Description:
 *   Set IELSRn.DTCE for "vector": the next matching interrupt is serviced
 *   by the DTC instead of the CPU.  Call ra_dtc_configure() on this vector
 *   first.  The DTC clears DTCE itself once the transfer this arms
 *   completes (normal/block mode) or on each activation (repeat mode with
 *   MRB.DISEL set) -- the vector's normal CPU interrupt handler then runs
 *   as usual on the next matching event.
 *
 ****************************************************************************/

void ra_dtc_enable(uint8_t vector);

/****************************************************************************
 * Name: ra_dtc_disable
 *
 * Description:
 *   Clear IELSRn.DTCE for "vector", handing its interrupt back to the CPU
 *   immediately instead of waiting for the DTC to finish or exhaust its
 *   count.
 *
 ****************************************************************************/

void ra_dtc_disable(uint8_t vector);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_RA8M1_RA_DTC_H */
